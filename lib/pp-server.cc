// -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-
//  PixelPusher protocol implementation for LED matrix
//
//  Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <arpa/inet.h>
#include <assert.h>
#include <getopt.h>
#include <limits.h>
#include <linux/netdevice.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stropts.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>

#include "pp-server.h"

#include "pp-thread.h"
#include "universal-discovery-protocol.h"

using namespace pp::internal;

static const char kNetworkInterface[] = "eth0";
static const uint16_t kPixelPusherDiscoveryPort = 7331;
static const uint16_t kPixelPusherListenPort = 5078;
static const uint8_t kSoftwareRevision = 122;
static const uint8_t kPixelPusherCommandMagic[16] = { 0x40, 0x09, 0x2d, 0xa6,
                                                      0x15, 0xa5, 0xdd, 0xe5,
                                                      0x6a, 0x9d, 0x4d, 0x5a,
                                                      0xcf, 0x09, 0xaf, 0x50 };

// The maximum packet size we accept.
// Typicall, the PixelPusher network will attempt to send smaller,
// non-fragmenting packets of size 1460; however, we would accept up to
// the UDP packet size.
static const int kMaxUDPPacketSize = 65507;  // largest practical w/ IPv4 header
static const int kDefaultUDPPacketSize = 1460;

// Say we want 60Hz update and 9 packets per frame (7 strips / packet), we
// don't really need more update rate than this.
static const uint32_t kMinUpdatePeriodUSec = 16666 / 9;

static int64_t CurrentTimeMicros() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t result = tv.tv_sec;
    return result * 1000000 + tv.tv_usec;
}

// Given the name of the interface, such as "eth0", fill the IP address and
// broadcast address into "header"
// Some socket and ioctl nastiness.
static bool DetermineNetwork(const char *interface,
                             DiscoveryPacketHeader *header) {
    int s;
    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        return false;
    }

    bool success = true;

    {
        // Get mac address for given interface name.
        struct ifreq mac_addr_query;
        strcpy(mac_addr_query.ifr_name, interface);
        if (ioctl(s, SIOCGIFHWADDR, &mac_addr_query) == 0) {
            memcpy(header->mac_address,  mac_addr_query.ifr_hwaddr.sa_data,
                   sizeof(header->mac_address));
        } else {
            perror("Getting hardware address");
            success = false;
        }
    }

    {
        struct ifreq ip_addr_query;
        strcpy(ip_addr_query.ifr_name, interface);
        if (ioctl(s, SIOCGIFADDR, &ip_addr_query) == 0) {
            struct sockaddr_in *s_in = (struct sockaddr_in *) &ip_addr_query.ifr_addr;
            memcpy(header->ip_address, &s_in->sin_addr, sizeof(header->ip_address));
        } else {
            perror("Getting IP address");
            success = false;
        }
    }

    close(s);

    // Let's print what we're sending.
    char buf[256];
    inet_ntop(AF_INET, header->ip_address, buf, sizeof(buf));
    fprintf(stderr, "%s: IP: %s; MAC: ", interface, buf);
    for (int i = 0; i < 6; ++i) {
        fprintf(stderr, "%s%02x", (i == 0) ? "" : ":", header->mac_address[i]);
    }
    fprintf(stderr, "\n");

    return success;
}

namespace {
// Threads deriving from this should exit Run() as soon as they see !running_
class StoppableThread : public Thread {
public:
    StoppableThread() : running_(true) {}
    virtual ~StoppableThread() { Stop(); }

    // Cause stopping wait until we're done.
    void Stop() {
        MutexLock l(&run_mutex_);
        running_ = false;
    }

protected:
    bool running() { MutexLock l(&run_mutex_); return running_; }

private:
    Mutex run_mutex_;
    bool running_;
};

// Broadcast every second the discovery protocol.
class Beacon : public StoppableThread {
public:
    Beacon(const DiscoveryPacketHeader &header,
           const PixelPusherContainer &pixel_pusher)
        : header_(header), pixel_pusher_(pixel_pusher),
          pixel_pusher_base_size_(CalcPixelPusherBaseSize(pixel_pusher_.base
                                                          ->strips_attached)),
          discovery_packet_size_(sizeof(header_)
                                 + pixel_pusher_base_size_
                                 + sizeof(pixel_pusher_.ext)),
          discovery_packet_buffer_(new uint8_t[discovery_packet_size_]),
          previous_sequence_(-1) {
        fprintf(stderr, "discovery packet size: %zd\n", discovery_packet_size_);
    }

    virtual ~Beacon() {
        Stop();
        delete [] discovery_packet_buffer_;
    }

    void UpdatePacketStats(uint32_t seen_sequence, uint32_t update_micros) {
        MutexLock l(&mutex_);
        pixel_pusher_.base->update_period = (update_micros < kMinUpdatePeriodUSec
                                             ? kMinUpdatePeriodUSec
                                             : update_micros);
        const int32_t sequence_diff = seen_sequence - previous_sequence_ - 1;
        if (sequence_diff > 0)
            pixel_pusher_.base->delta_sequence += sequence_diff;
        previous_sequence_ = seen_sequence;
    }

    virtual void Run() {
        int s;
        if ((s = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("socket");
            exit(1);    // don't worry about graceful exit.
        }

        int enable = 1;
        if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(enable)) < 0) {
            perror("enable broadcast");
            exit(1);
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        addr.sin_port = htons(kPixelPusherDiscoveryPort);

        fprintf(stderr, "Starting PixelPusher discovery beacon "
                "broadcasting to port %d\n", kPixelPusherDiscoveryPort);
        struct timespec sleep_time = { 1, 0 };  // todo: tweak.
        while (running()) {
            // The header is of type 'DiscoveryPacket'.
            {
                MutexLock l(&mutex_);  // protect with stable delta sequnce.

                uint8_t *dest = discovery_packet_buffer_;
                memcpy(dest, &header_, sizeof(header_));
                dest += sizeof(header_);

                // This part is dynamic length.
                memcpy(dest, pixel_pusher_.base, pixel_pusher_base_size_);
                dest += pixel_pusher_base_size_;

                memcpy(dest, &pixel_pusher_.ext, sizeof(pixel_pusher_.ext));
                pixel_pusher_.base->delta_sequence = 0;
            }
            if (sendto(s, discovery_packet_buffer_, discovery_packet_size_, 0,
                       (struct sockaddr *) &addr, sizeof(addr)) < 0) {
                perror("Broadcasting problem");
            }
            nanosleep(&sleep_time, NULL);
        }
    }

private:
    const DiscoveryPacketHeader header_;
    PixelPusherContainer pixel_pusher_;
    const size_t pixel_pusher_base_size_;
    const size_t discovery_packet_size_;
    uint8_t *discovery_packet_buffer_;
    Mutex mutex_;
    uint32_t previous_sequence_;
};

class PacketReceiver : public StoppableThread {
public:
    PacketReceiver(pp::OutputDevice *output, Beacon *beacon)
        : output_(output), beacon_(beacon) { }

    virtual void Run() {
        char *packet_buffer = new char[kMaxUDPPacketSize];
        const int strip_data_len = (1 /* strip number */
                                    + 3 * output_->num_pixel_per_strip());
        struct StripData {
            uint8_t strip_index;
            ::pp::PixelColor pixel[0];
        };

        int s;
        if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
            perror("creating listen socket");
            exit(1);
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(kPixelPusherListenPort);
        if (bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
            perror("bind");
            exit(1);
        }
        fprintf(stderr, "Listening for pixels pushed to port %d\n",
                kPixelPusherListenPort);
        // Create an off-screen canvas to draw on, and get on-screen.
        const int all_rows = output_->num_strips();
        while (running()) {
            ssize_t buffer_bytes = recvfrom(s, packet_buffer, kMaxUDPPacketSize,
                                            0, NULL, 0);
            if (!running())
                break;
            const int64_t start_time = CurrentTimeMicros();
            if (buffer_bytes < 0) {
                perror("receive problem");
                continue;
            }
            if (buffer_bytes <= 4) {
                fprintf(stderr, "weird, no sequence number ? Got %zd bytes\n",
                        buffer_bytes);
            }

            const char *buf_pos = packet_buffer;

            uint32_t sequence;
            memcpy(&sequence, buf_pos, sizeof(sequence));
            buffer_bytes -= 4;
            buf_pos += 4;

            if (buffer_bytes >= (int)sizeof(kPixelPusherCommandMagic)
                && memcmp(buf_pos, kPixelPusherCommandMagic,
                          sizeof(kPixelPusherCommandMagic)) == 0) {
                output_->HandlePusherCommand(
                    buf_pos + sizeof(kPixelPusherCommandMagic),
                    buffer_bytes - sizeof(kPixelPusherCommandMagic));
                continue;
            }

            if (buffer_bytes % strip_data_len != 0) {
                fprintf(stderr, "Expecting multiple of {1 + (rgb)*%d} = %d, "
                        "but got %zd bytes (leftover: %zd)\n",
                        output_->num_pixel_per_strip(),
                        strip_data_len, buffer_bytes,
                        buffer_bytes % strip_data_len);
                continue;
            }

            const int received_strips = buffer_bytes / strip_data_len;
            const bool got_full_update = (received_strips == all_rows);
            output_->StartFrame(got_full_update);
            for (int i = 0; i < received_strips; ++i) {
                StripData *data = (StripData *) buf_pos;
                // Copy into frame buffer.
                for (int x = 0; x < output_->num_pixel_per_strip(); ++x) {
                    output_->SetPixel(data->strip_index, x, data->pixel[x]);
                }
                buf_pos += strip_data_len;
            }
            output_->FlushFrame();
            const int64_t end_time = CurrentTimeMicros();
            beacon_->UpdatePacketStats(sequence, end_time - start_time);
        }
        delete [] packet_buffer;
    }

private:
    pp::OutputDevice *const output_;
    Beacon *const beacon_;
};

// Internal server implemantation.
class PixelPusherServer {
public:
    PixelPusherServer() : discovery_beacon_(NULL), receiver_(NULL) {}

    // Separate Init() from constructor as things can fail.
    bool Init(const ::pp::PPOptions &options, ::pp::OutputDevice *device);
    ~PixelPusherServer();

private:
    DiscoveryPacketHeader header_;
    PixelPusherContainer pixel_pusher_container_;
    Beacon *discovery_beacon_;
    PacketReceiver *receiver_;
};

// Run the PixexlPusher server with the given options, sending pixels
// to the given output device, an implementation of which you have to provide.
bool PixelPusherServer::Init(const ::pp::PPOptions &options,
                             ::pp::OutputDevice *device) {
    if (options.udp_packet_size < 200
        || options.udp_packet_size > kMaxUDPPacketSize) {
        fprintf(stderr, "UDP packet size out of range (200...%d)\n",
                kMaxUDPPacketSize);
        return false;
    }

    // Init PixelPusher protocol
    memset(&header_, 0, sizeof(header_));

    // We might be started in some init script and the network is
    // not there yet. Try up to one minute.
    int network_retries_left = 60;

    while (network_retries_left &&
           !DetermineNetwork(options.network_interface, &header_)) {
        --network_retries_left;
        sleep(1);
    }
    if (!network_retries_left) {
        fprintf(stderr, "Couldn't listen on network interface %s. ",
                options.network_interface);
        return false;
    }

    header_.device_type = PIXELPUSHER;
    header_.protocol_version = 1;  // ?
    header_.vendor_id = 3;  // h.zeller@acm.org
    header_.product_id = 0;
    header_.sw_revision = kSoftwareRevision;
    header_.link_speed = 10000000;  // 10MBit

    const int number_of_strips = device->num_strips();
    const int pixels_per_strip = device->num_pixel_per_strip();

    memset(&pixel_pusher_container_, 0, sizeof(pixel_pusher_container_));
    size_t base_size = CalcPixelPusherBaseSize(number_of_strips);
    pixel_pusher_container_.base = (struct PixelPusherBase*) malloc(base_size);
    memset(pixel_pusher_container_.base, 0, base_size);
    pixel_pusher_container_.base->strips_attached = number_of_strips;
    pixel_pusher_container_.base->pixels_per_strip = pixels_per_strip;
    static const int kUsablePacketSize = options.udp_packet_size - 4; // 4 bytes seq#
    // Whatever fits in one packet, but not more than one 'frame'.
    pixel_pusher_container_.base->max_strips_per_packet
        = std::min(kUsablePacketSize / (1 + 3 * pixels_per_strip),
                   number_of_strips);
    if (pixel_pusher_container_.base->max_strips_per_packet == 0) {
        fprintf(stderr, "Packet size limit (%d Bytes) smaller than needed to "
                "transmit one row (%d Bytes). Change UDP packet size.\n",
                kUsablePacketSize, (1 + 3 * pixels_per_strip));
        return false;
    }
    if (options.artnet_universe >= 0 && options.artnet_channel >= 0) {
        pixel_pusher_container_.base->artnet_universe = options.artnet_universe;
        pixel_pusher_container_.base->artnet_channel = options.artnet_channel;
    }
    fprintf(stderr, "Display: %dx%d (%d pixels each on %d strips)\n"
            "Accepting max %d strips per packet (with UDP packet limit %d).\n",
            pixels_per_strip, number_of_strips,
            pixels_per_strip, number_of_strips,
            pixel_pusher_container_.base->max_strips_per_packet,
            options.udp_packet_size);
    pixel_pusher_container_.base->power_total = 1;         // ?
    pixel_pusher_container_.base->update_period = 1000;   // initial assumption.
    pixel_pusher_container_.base->controller_ordinal = options.controller;
    pixel_pusher_container_.base->group_ordinal = options.group;

    pixel_pusher_container_.base->my_port = kPixelPusherListenPort;
    for (int i = 0; i < number_of_strips; ++i) {
        pixel_pusher_container_.base->strip_flags[i]
            = (options.is_logarithmic ? SFLAG_LOGARITHMIC : 0);
    }
    pixel_pusher_container_.ext.pusher_flags = 0;
    pixel_pusher_container_.ext.segments = 1;    // ?
    pixel_pusher_container_.ext.power_domain = 0;

    // Create our threads.
    discovery_beacon_ = new Beacon(header_, pixel_pusher_container_);
    receiver_ = new PacketReceiver(device, discovery_beacon_);

    // Start threads, choose priority and CPU affinity.
    receiver_->Start(0, (1<<1));         // userspace priority
    discovery_beacon_->Start(5, (1<<2)); // This should accurately send updates.
    return true;
}

PixelPusherServer::~PixelPusherServer() {
    receiver_->Stop();
    discovery_beacon_->Stop();
#if 0
    // TODO: Receiver is blocking in recvfrom(), so we can't reliably force
    // shut down while waiting on that.
    // .. and everything else references that.
    // Don't care, just leak to the end.
    delete receiver;
    delete discovery_beacon;
    free(pixel_pusher_container_.base);
#endif
}

}  // anonymous namespace


static PixelPusherServer *running_instance = NULL;

// Public, exported interface.
namespace pp {
PPOptions::PPOptions()
    : network_interface("eth0"),
      udp_packet_size(1460),
      is_logarithmic(true),
      group(0), controller(0),
      artnet_universe(-1), artnet_channel(-1) {
}

bool StartPixelPusherServer(const PPOptions &options, OutputDevice *device) {
    if (running_instance) {
        fprintf(stderr, "Attempt to run PixelPusher server twice\n");
        return false;
    }
    PixelPusherServer *server = new PixelPusherServer();
    if (server->Init(options, device)) {
        running_instance = server;
        return true;
    }
    delete server;
    return false;
}

void ShutdownPixelPusherServer() {
    delete running_instance;
    running_instance = NULL;
}
}  // namespace pp

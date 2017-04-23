// -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-
//  PixelPusher protocol implementation for generic output matrix.
//  This implements the protocol of the PixelPusher devices by Heroic Robotics
//    http://www.heroicrobotics.com/products/pixelpusher
//  .. on a Posix system.
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

#ifndef PIXEL_PUSH_SERVER_H
#define PIXEL_PUSH_SERVER_H

namespace pp {

// Pixel color.
//
// TODO(hzeller): this might change if we ever support more color resolution
// or additional LEDs (I think some PixelPusher implementations have additional
// LEDs for high CRI?).
struct PixelColor {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

// This is an abstract class that you have to implement that fits your
// particular output device. Implementations by Henner Zeller are available
// for RGB Matrix and Spixels.
class OutputDevice {
public:
    // Return number of strips this output device has available.
    virtual int num_strips() const = 0;

    // Return number of pixels there are on each strip.
    virtual int num_pixel_per_strip() const = 0;

    /*
     * The following three calls will happen in sequence when a new
     * data packet arrived. StartFrame() is called on arrival of the
     * packet, followed by a sequence of SetPixel() calls, followed by
     * FlushFrame().
     */

    // Callback from server when a new frame is received. The boolean
    // "full_update" indicates if the following calls to SetPixel() is about
    // to set all available pixels in our device (an FYI, which can
    // internally be used to do double-buffering).
    virtual void StartFrame(bool full_update) {}

    // Callback from server with a pixel.
    virtual void SetPixel(int strip, int pixel,
                          const ::pp::PixelColor &col) = 0;

    // Called from the PixelPusher server, after all the Pixels for a received
    // packet have ben SetPixel()ed.
    virtual void FlushFrame() = 0;

    // Received a PixelPusher command that couldn't be handled in the server.
    // This callback passes it through as-is in case the OutputDevice wants
    // to deal with it.
    virtual void HandlePusherCommand(const char *buf, size_t size) {}

    // TODO: possible more callbacks for more functionality.
};

// Configuration options when starting the PixelPusher server. If you don't
// set any values here, reasonable defaults are used.
struct PPOptions {
    PPOptions();
    // The name of the network interface, such as eth0 or wlan0
    const char *network_interface;
    int udp_packet_size;

    bool is_logarithmic;    // If out output is logarithmic

    // PixelPusher group and controller
    int group;
    int controller;

    // Artnet configuration.
    int artnet_universe;
    int artnet_channel;
};

// Start a PixelPusher server with the given options and and OutputDevice
// implementation. Does not take over the ownership of the OutputDevice.
// This returns 'true' after initialization, if the start was successful.
// A running instance should be stopped by calling ShutdownPixelPusherServer().
bool StartPixelPusherServer(const ::pp::PPOptions &options,
                            ::pp::OutputDevice *device);

// Shuts down the current instance of the pixel pusher server.
void ShutdownPixelPusherServer();
}  // namespace pp
#endif  /* PIXEL_PUSH_SERVER_H */

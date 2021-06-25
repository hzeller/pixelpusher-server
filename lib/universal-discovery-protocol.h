
/*  -*- mode: c; c-basic-offset: 4; indent-tabs-mode: nil; -*-
 *  Universal Discovery Protocol
 *  A UDP protocol for finding Etherdream/Heroic Robotics lighting devices
 *
 *  (c) 2012 Jas Strong and Jacob Potter
 *  <jasmine@electronpusher.org> <jacobdp@gmail.com>
 *
 *	PixelPusherBase/PixelPusherExt split created by Henner Zeller 2016
 *
 *	pusher command stuff added by Christopher Schardt 2017
 *	organized by Christopher Schardt 2017
 */
 
#ifndef PIXELPUSHER_UNIVERSAL_DISCOVERY_PROTOCOL_H
#define PIXELPUSHER_UNIVERSAL_DISCOVERY_PROTOCOL_H

#include <stdint.h>

#ifdef __GNUC__
	#define PACKED __attribute__((__packed__))
#else
	#define PACKED
#endif


/////////////////////////////////////////////////
#pragma mark - IP PORTS:

#define PixelPusherDiscoveryPort	7331
#define PixelPusherListenPort		5078


/////////////////////////////////////////////////
#pragma mark - PIXELPUSHER DISCOVERY FORMAT:

// bits found in the pusher_flags field below
#define PFLAG_PROTECTED			(1<<0) // require qualified registry.getStrips() call.
#define PFLAG_FIXEDSIZE			(1<<1) // Requires every datagram same size.
#define PFLAG_GLOBALBRIGHTNESS 	(1<<2) // Pusher supports PPPusherCommandGlobalBrightness (NOT whether any strip supports hardware brightness)
#define PFLAG_STRIPBRIGHTNESS	(1<<3) // Pusher supports PPPusherCommandStripBrightness (NOT whether any strip supports hardware brightness)
#define PFLAG_DYNAMICS			(1<<4) // Pusher supports PPPusherCommandDynamics, and the functionality IS supported

// bits found in the strip_flags field below
#define SFLAG_RGBOW          (1<<0)  // High CRI strip					// not implemented
#define SFLAG_WIDEPIXELS     (1<<1)  // 48 Bit/pixel RGBrgb				// not implemented
#define SFLAG_LOGARITHMIC    (1<<2)  // LED has logarithmic response.	// not implemented
#define SFLAG_MOTION         (1<<3)  // A motion controller.
#define SFLAG_NOTIDEMPOTENT  (1<<4)  // motion controller with side-effects.
#define	SFLAG_BRIGHTNESS	 (1<<5)  // Strip configured for hardware that supports brightness

// This is slightly modified from the original to separate the dynamic from the
// static parts.
// All multi-byte values here are Little-Endian (read: host byte order).
typedef struct PACKED PixelPusherBase {
    uint8_t  strips_attached;
    uint8_t  max_strips_per_packet;
    uint16_t pixels_per_strip;  // uint16_t used to make alignment work
    uint32_t update_period; // in microseconds
    uint32_t power_total;   // in PWM units
    uint32_t delta_sequence;  // difference between received and expected sequence numbers
    int32_t controller_ordinal; // ordering number for this controller.
    int32_t group_ordinal;      // group number for this controller.
    uint16_t artnet_universe;   // configured artnet starting point for this controller
    uint16_t artnet_channel;
    uint16_t my_port;
    uint16_t padding1_;
    // The following has a dynamic length: max(8, strips_attached). So this
    // PixelPusherBase can grow beyond its limits. For the dynamic case, we
    // just allocate more and write beyond the 8
    uint8_t strip_flags[8];     // flags for each strip, for up to eight strips
} PixelPusherBase;

typedef struct PACKED PixelPusherExt {
    uint16_t padding2_;          // The following is read by 32+stripFlagSize
    uint32_t pusher_flags;      // flags for the whole pusher
    uint32_t segments;          // number of segments in each strip
    uint32_t power_domain;      // power domain of this pusher
    uint8_t last_driven_ip[4];  // last host to drive this pusher
    uint16_t last_driven_port;  // source port of last update
} PixelPusherExt;

typedef struct PixelPusherContainer {
    struct PixelPusherBase *base;  // dynamically sized.
    struct PixelPusherExt  ext;
} PixelPusherContainer;
static inline uint32_t CalcPixelPusherBaseSize(int num_strips)
{
    return sizeof(struct PixelPusherBase) + (num_strips <= 8 ? 0 : num_strips-8);
}

typedef struct PACKED StaticSizePixelPusher {
    struct PixelPusherBase base;   // Good for up to 8 strips.
    struct PixelPusherExt  ext;
} PixelPusher;


/////////////////////////////////////////////////
#pragma mark - UNIVERSAL DISCOVERY FORMAT:

typedef struct DiscoveryPacketHeader {
    uint8_t mac_address[6];
    uint8_t ip_address[4];  // network byte order
    uint8_t device_type;
    uint8_t protocol_version; // for the device, not the discovery
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t hw_revision;
    uint16_t sw_revision;
    uint32_t link_speed;    // in bits per second
} PACKED DiscoveryPacketHeader;

typedef enum DeviceType {
    ETHERDREAM = 0,
    LUMIABRIDGE = 1,
    PIXELPUSHER = 2
} DeviceType;

typedef struct EtherDream {
    uint16_t buffer_capacity;
    uint32_t max_point_rate;
    uint8_t light_engine_state;
    uint8_t playback_state;
    uint8_t source;     //   0 = network
    uint16_t light_engine_flags;
    uint16_t playback_flags;
    uint16_t source_flags;
    uint16_t buffer_fullness;
    uint32_t point_rate;                // current point playback rate
    uint32_t point_count;           //  # points played
} EtherDream;

typedef struct LumiaBridge {
    // placekeeper
} LumiaBridge;

typedef union {
    PixelPusher pixelpusher;
    LumiaBridge lumiabridge;
    EtherDream etherdream;
} Particulars;

typedef struct DiscoveryPacket {
    DiscoveryPacketHeader header;
    Particulars p;
} DiscoveryPacket;


/////////////////////////////////////////////////
#pragma mark - PUSHER COMMANDS:

typedef enum PPPusherCommandType
{
	PPPusherCommandNone =				0x00,	// returned when data not present
	PPPusherCommandReset =				0x01,
	PPPusherCommandGlobalBrightness =	0x02,
	PPPusherCommandWifiConfigure =		0x03,
	PPPusherCommandLedConfigure =		0x04,
	PPPusherCommandStripBrightness =	0x05,
	PPPusherCommandDynamics =			0x06,
} PPPusherCommandType;

typedef enum PPPusherCommandSecurityType
{
	PPPusherCommandSecurityNone =		0,
	PPPusherCommandSecurityWep =		1,
	PPPusherCommandSecurityWpa =		2,
	PPPusherCommandSecurityWpa2 =		3,
} PPPusherCommandSecurityType;

typedef enum PPPusherCommandStripType
{
	PPPusherCommandStripLPD8806 =	0,
	PPPusherCommandStripWS2801 =	1,
	PPPusherCommandStripWS2811 =	2,
	PPPusherCommandStripAPA102 =	3,
} PPPusherCommandStripType;
typedef uint8_t PPPusherCommandStripTypes[8];

typedef enum PPPusherCommandComponentOrdering
{
	PPPusherCommandComponentRGB =	0,
	PPPusherCommandComponentRBG =	1,
	PPPusherCommandComponentGBR =	2,
	PPPusherCommandComponentGRB =	3,
	PPPusherCommandComponentBGR =	4,
	PPPusherCommandComponentBRG =	5,
} PPPusherCommandComponentOrdering;
typedef uint8_t PPPusherCommandComponentOrderings[8];

typedef struct PACKED PPPusherCommandDynamicsData
{
	uint32_t			brightnessScaleRed16;
	uint32_t			brightnessScaleGreen16;
	uint32_t			brightnessScaleBlue16;
	uint16_t			options;				// values in PPPusherCommandDynamicsOption
	uint16_t			gammaTableLength;		// for now, must be 0 or 256
	uint16_t			gammaTable[0];
} PPPusherCommandDynamicsData;

enum PPPusherCommandDynamicsOption
{
	PPPusherCommandDynamicsOption_16bitColor = 0x0001,
};
typedef uint16_t	PPPusherCommandDynamicsOptions;


#endif /* UNIVERSAL_DISCOVERY_PROTOCOL_H */

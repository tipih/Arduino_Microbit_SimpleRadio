#pragma once
#define MICROBIT_RADIO_STATUS_INITIALISED       0x0001

// Default configuration values
#define MICROBIT_RADIO_BASE_ADDRESS             0x75626974
#define MICROBIT_RADIO_DEFAULT_GROUP            1
#define MICROBIT_RADIO_DEFAULT_TX_POWER         6
#define MICROBIT_RADIO_MAX_PACKET_SIZE          32
#define MICROBIT_RADIO_HEADER_SIZE              4
#define MICROBIT_RADIO_MAXIMUM_RX_BUFFERS       4
#define MICROBIT_BLE_POWER_LEVELS               8

// Known Protocol Numbers
#define MICROBIT_RADIO_PROTOCOL_DATAGRAM        1       // A simple, single frame datagram. a little like UDP but with smaller packets. :-)
#define MICROBIT_RADIO_PROTOCOL_EVENTBUS        2       // Transparent propogation of events from one micro:bit to another.
#define MICROBIT_BLE_ENABLED                    0
// Events
#define MICROBIT_RADIO_EVT_DATAGRAM             1       // Event to signal that a new datagram has been received.

#define MICROBIT_OK								0x01
#define MICROBIT_NOT_SUPPORTED					0xff
#define MICROBIT_INVALID_PARAMETER				0xfe
#define	MICROBIT_NO_RESOURCES					0xfd

// Sets the default radio channel
#ifndef MICROBIT_RADIO_DEFAULT_FREQUENCY
#define MICROBIT_RADIO_DEFAULT_FREQUENCY 7
#endif

// Sets the minimum frequency band permissable for the device
#ifndef MICROBIT_RADIO_LOWER_FREQ_BAND
#define MICROBIT_RADIO_LOWER_FREQ_BAND 0
#endif

// Sets the maximum frequency band permissable for the device
#ifndef MICROBIT_RADIO_UPPER_FREQ_BAND
#define MICROBIT_RADIO_UPPER_FREQ_BAND 83
#endif


extern "C" void RADIO_IRQHandler(void);

struct FrameBuffer
{
	uint8_t         length;                             // The length of the remaining bytes in the packet. includes protocol/version/group fields, excluding the length field itself.
	uint8_t         version;                            // Protocol version code.
	uint8_t         group;                              // ID of the group to which this packet belongs.
	uint8_t         protocol;                           // Inner protocol number c.f. those issued by IANA for IP protocols

	uint8_t         payload[MICROBIT_RADIO_MAX_PACKET_SIZE];    // User / higher layer protocol data
	FrameBuffer     *next;                              // Linkage, to allow this and other protocols to queue packets pending processing.
	int             rssi;                               // Received signal strength of this frame.
};


FrameBuffer             *rxQueue;   // A linear list of incoming packets, queued awaiting processing.
FrameBuffer             *rxBuf;     // A pointer to the buffer being actively used by the RADIO hardware.
FrameBuffer				*myDataSendData;
uint8_t                 group;      // The radio group to which this micro:bit belongs.
uint8_t                 queueDepth; // The number of packets in the receiver queue.
int status;
int		                rssi;       // Received signal strength of this frame.
int						id;

const int COL1 = 3;     // Column #1 control
const int LED = 26;     // 'row 1' led
const long interval = 5000;
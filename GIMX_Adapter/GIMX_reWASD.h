/*
*    Protocol definitions for communication between reWASD and GIMX adapter with reWASD firmware.
*    To emulate virtual USB device for other PC or console reWASD uses the same hardware as GIMX project (see https://blog.gimx.fr),
*    but it flashes its own firmware tailored specifically to reWASD.
*    After this flashing the adapter will be able to work with reWASD only and not be compatible with GIMX project anymore -
*    you will not need to install GIMX at all, because reWASD will communicate with the adapter directly.
*    Of course, if you do not need reWASD anymore, the firmware can be changed again with GIMX launcher.
*    The adapter is based on the pupular 'Pro Micro' board with atmega32u4 chip and USB-to-TTL converter.
*    You can build your own adapter using these instructions: https://gimx.fr/wiki/index.php?title=DIY_USB_adapter.
*    Alternatively, you can buy ready adapter from GIMX shop: https://blog.gimx.fr/product/gimx-adapter.
*    There can be different types of USB-to-TTL converters on the board, CP2102 is among the most popular.
*    The only requirement for reWASD as that it must support at least 500000 baud rate.
*    reWASD team wants to express special thanks to Mathieu Laurendeau for his hard work on GIMX project and developing this adapter!
*    Now reWASD is an alternative way to use this adapter and give more options to users.
*
*    (C) 2020-2025 Disc Soft FZE LLC.
*/

#ifndef REWASD_GIMX_PROTOCOL_H_
#define REWASD_GIMX_PROTOCOL_H_

//Current version of reWASD firmware in the adapter.
#define REWASD_GIMX_MAJOR_VERSION        1
#define REWASD_GIMX_MINOR_VERSION        3

//This is internal value to identify the board as GIMX adapter based on atmega32u4 chip.
#define REWASD_GIMX_BOARD_TYPE           0xC1

//Baud rate used by GIMX adapter with reWASD firmware.
#define REWASD_GIMX_BAUDRATE             500000UL

// The atmega32u4 has 2.5Kbytes SRAM only, so this is reasonable limit for all descriptors.
#define REWASD_GIMX_MAX_DESCRIPTORS_SIZE 1024

#define REWASD_GIMX_MAX_PACKET_SIZE_EP0  64

#define REWASD_GIMX_MAX_PAYLOAD_SIZE_EP  64 // for non-control endpoints

//Timeout in milliseconds of inactivity when adapter sends keepalive packet to reWASD.
//This value should not exceed 4194 because the counter in atmega32u4 will overflow.
#define REWASD_GIMX_KEEP_ALIVE_TIMEOUT   1250UL

#pragma pack(push,1)

#define REWASD_GIMX_MAX_DESCRIPTORS      8

typedef struct _REWASD_GIMX_DESCRIPTOR_INDEX {
    uint16_t wLength; //if 0 zero then treated as terminator
    uint16_t wValue;
    uint16_t wIndex;
} REWASD_GIMX_DESCRIPTOR_INDEX, * PREWASD_GIMX_DESCRIPTOR_INDEX;

//If this flag is set then firmware may send some debug packets to reWASD in case of transmission errors.
#define REWASD_GIMX_FLAG_DEBUG           0x0001

//If this flag is set then firmware can combine IN reports if they arrive faster than host retrieves them.
#define REWASD_GIMX_FLAG_COMBINE         0x0002

//This header is transmitted by reWASD on first REWASD_GIMX_PACKET_TYPE_DESCRIPTORS packet.
//The header is followed by descriptors.
typedef struct _REWASD_GIMX_DESCRIPTOR_HEADER {
    //Total size of data including this header and all descriptors.
    //It should not exceed REWASD_GIMX_MAX_DESCRIPTORS_SIZE.
    uint16_t wTotalLength;
    uint8_t  InEndpoint;
    uint8_t  OutEndpoint;
    uint16_t flags;//REWASD_GIMX_FLAG_... definition above.
    REWASD_GIMX_DESCRIPTOR_INDEX descriptorIndex[REWASD_GIMX_MAX_DESCRIPTORS];
}REWASD_GIMX_DESCRIPTOR_HEADER, * PREWASD_GIMX_DESCRIPTOR_HEADER;

typedef enum _REWASD_GIMX_PACKET_TYPE {
    //Used to send IN report to adapter for input endpoint or receive OUT report from adapter for output endpoint.
    //This is base value for the range 0x70 - 0xAF, which corresponds to values from 1 to 64 (REWASD_GIMX_MAX_PAYLOAD_SIZE_EP) bytes.
    REWASD_GIMX_PACKET_TYPE_REPORT =      0x70,
    //Used to send reply to adapter for control endpoint or receive data from adapter for control endpoint.
    //This is base value for the range 0xB0 - 0xF0, which corresponds to values from 0 to 64 (REWASD_GIMX_MAX_PACKET_SIZE_EP0) bytes.
    REWASD_GIMX_PACKET_TYPE_CONTROL =     0xB0,
    //Used to send reply to adapter indicating STALL condition for control endpoint.
    REWASD_GIMX_PACKET_TYPE_STALL =       0xF1,
    //Used to send new descriptors to adapter after it has performed reset.
    //After receiving this command adapter sends back REWASD_GIMX_PACKET_TYPE_DESCRIPTORS packet with 2 bytes payload indicating number of already received bytes.
    //ReWASD sends multiple REWASD_GIMX_PACKET_TYPE_DESCRIPTORS packets in REWASD_GIMX_MAX_PACKET_VALUE_SIZE chunks (last chunk can be partial)
    //until all descriptors have been transferred.
    //The first chunk starts with REWASD_GIMX_DESCRIPTOR_HEADER structure which shows total size of all descriptors in the wTotalLength field.
    //When the last chunk is transferred the adapter enables USB interface and does not recognize this command anymore.
    REWASD_GIMX_PACKET_TYPE_DESCRIPTORS = 0xF2,
    //This command has 6 bytes payload with 'reWASD' signature and requests adapter to return its version.
    //Adapter returns 64 bytes payload: 6 bytes for signature, 1 byte of adapter type and 2 bytes for major and minor versions, other bytes are reserved and returned as zeroes.
    //This command is only used after reset before descriptors are loaded - if it is received when the adapter is already running then hardware reset is performed.
    //Also if this packet is received by reWASD already after adapter was started then it indicates that adapter was restarted (USB host was replugged).
    REWASD_GIMX_PACKET_TYPE_VERSION =     0xF3,
    //Used to send debug information from adapter.
    //Size of payload is 2 bytes, where meaning of bytes can depend on situation.
    REWASD_GIMX_PACKET_TYPE_DEBUG =       0xF4,
    //This packet has payload with 1 parameter byte containing value 0x01 and informs reWASD that adapter was connected to USB bus (configured by host).
    //Other values of paramater are reserved.
    REWASD_GIMX_PACKET_TYPE_CONNECT     = 0xF6,
    //This packet has no payload and no CRC and informs reWASD that adapter is still working.
    REWASD_GIMX_PACKET_TYPE_KEEP_ALIVE =  0xFD
}REWASD_GIMX_PACKET_TYPE, * PREWASD_GIMX_PACKET_TYPE;

// This is the max serial packet size
#define REWASD_GIMX_MAX_PACKET_SIZE      255

#define REWASD_GIMX_MAX_PACKET_VALUE_SIZE (REWASD_GIMX_MAX_PACKET_SIZE - 2) //253

typedef struct _REWASD_GIMX_PACKET {
    //Type of packet - size of payload is encoded in the command.
    uint8_t type;
    //Contains up to 253 bytes of payload.
    uint8_t value[REWASD_GIMX_MAX_PACKET_VALUE_SIZE];
    //CRC-8 of first bytes (type + value)
    uint8_t crc;
}REWASD_GIMX_PACKET, * PREWASD_GIMX_PACKET;

#pragma pack(pop)

#endif//REWASD_GIMX_PROTOCOL_H_


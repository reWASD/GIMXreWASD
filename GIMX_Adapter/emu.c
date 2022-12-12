/*
*    GIMX adapter firmware for reWASD.
*    (C) 2020-2022 Disc Soft Ltd.
*
* The code is based on the original serialusb project by Mathieu Laurendeau (see https://github.com/matlo/serialusb),
* but was rewritten from scratch to optimize it for reWASD.
* With this firmware adapter works like USB proxy and reWASD can load any descriptors to it and emulate almost any gamepad.
* CRC checking was added to make it more robust and many other enhancements have been made, especially ISR offloading and
* asynchronous processing of CONTROL requests.
* It is allowed to modify this code and adapt to other architectures, provided Disc Soft Ltd is informed about changes and
* compatibility with existing firmware is maintained at the protocol level.
*/

#include <asf.h>

#include <avr/wdt.h>
#include <avr/power.h>
#include <util/crc16.h>

#include "GIMX_reWASD.h"

#define KEEP_ALIVE_COUNT (REWASD_GIMX_KEEP_ALIVE_TIMEOUT * 15625 / 1000)  //19531

static uint8_t descriptors[REWASD_GIMX_MAX_DESCRIPTORS_SIZE];

static uint16_t desc_received = 0;

static uint8_t version_buffer[64] = { 'r', 'e', 'W', 'A', 'S', 'D', REWASD_GIMX_BOARD_TYPE, REWASD_GIMX_MAJOR_VERSION, REWASD_GIMX_MINOR_VERSION };

#define REWASD_GIMX_RECEIVE_BUFFER_SIZE 256

//Circular buffer for USART reception.
//This size is exactly 256 bytes in order for receive_start and receive_end to wrap automatically to 0 on overflow.
static uint8_t receive_buf[REWASD_GIMX_RECEIVE_BUFFER_SIZE];

//This value is used to verify that reWASD reply to control packet refers to the same original request.
//It is incremented automatically on every incoming CONTROL packet and serves as a 'generation' counter.
//The firmware stores this value in byte 0 of control packet and compares with byte 0 of CONTROL packet reply returned by reWASD.
static uint8_t control_sequence = 0;

//Buffer for incoming CONTROL endpoint request and for reply from reWASD.
//Note that byte 0 contains control_sequence (see comments above).
static uint8_t control_buf[sizeof(USB_ControlRequest) + REWASD_GIMX_MAX_PACKET_SIZE_EP0 + 1];

//Buffer with pending data for IN report.
//It will be transferred to host immediately when the pipe becomes ready.
static uint8_t input_buf1[REWASD_GIMX_MAX_PAYLOAD_SIZE_EP];
//Buffer where next IN report is prepared if current report in input_buf1 was not yet sent to host.
//If new IN report from reWASD arrives but input_buf1 was still not transferred then its contents are moved to input_buf1 and new data is prepared in input_buf2.
//So these buffers act like FIFO with 2 entries and only 2 last IN reports are kept if host cannot retrieve them as fast as they arrive.
static uint8_t input_buf2[REWASD_GIMX_MAX_PAYLOAD_SIZE_EP];

//Size of valid data in input_buf1 if IN report is pending. It will be transferred to host when the pipe becomes ready.
static uint8_t inputDataLen1 = 0;
//Size of valid data in input_buf2 if new report is ready but current report in input_buf1 was not yet sent to host.
static uint8_t inputDataLen2 = 0;

static uint8_t out_buf[REWASD_GIMX_MAX_PAYLOAD_SIZE_EP];

static uint8_t inEndpoint = 0;
static uint8_t outEndpoint = 0;

static bool waiting_control_reply = false;

static bool use_debug = false;

static bool started = false;

//Some variables are used in both the main code and the interrupt (ISR), therefore they must be declared as volatile.
static          uint8_t receive_start = 0;
static volatile uint8_t receive_end = 0;

//This is core function to send packet to reWASD.
//It calcultes CRC automatically and adds it to end of packet.
//NOTE: noinline attribute was added because sometimes wrong data is sent via COM port due to optimization.
void __attribute__ ((noinline)) send_buffer_with_crc(uint8_t type, uint8_t* data, uint8_t len)
{
    Serial_SendByte(type);

    uint8_t crc = _crc8_ccitt_update(0xFF, type);

    while (len--)
    {
        uint8_t value = *data++;

        Serial_SendByte(value);

        crc = _crc8_ccitt_update(crc, value);
    }

    Serial_SendByte(crc);

    //Reset keep-alive timer
    TCNT1 = 0;
}

//This routine is called after hardware reset or on REWASD_GIMX_PACKET_TYPE_VERSION when device has not yet started.
//When device has started (descriptors loaded) it automatically reboots.
//This allows reWASD to always start from scratch and reload descriptors even if adapter has stuck somewhere in the main loop.
void send_version(void)
{
    send_buffer_with_crc(REWASD_GIMX_PACKET_TYPE_VERSION, version_buffer, sizeof(version_buffer));
}

//Called on REWASD_GIMX_PACKET_TYPE_VERSION if it was received when device has already started (see comments above).
static inline void forceHardReset(void) {
    USB_Detach();
    GlobalInterruptDisable();
    wdt_enable(WDTO_15MS); // enable watchdog
    while (1); // wait for watchdog to reset processor
}

//This ISR services incoming packets from reWASD.
//It is made as small as possible in order to drain incoming bytes fast into circular buffer for later processing.
//This minimizes risk of losing some data.
ISR(USART1_RX_vect)
{
    do
    {
        receive_buf[receive_end++] = UDR1;
    } while (Serial_IsCharReceived());
}

//Called from USB_USBTask.
void EVENT_USB_Device_ConfigurationChanged(void)
{
    PREWASD_GIMX_DESCRIPTOR_HEADER header = (PREWASD_GIMX_DESCRIPTOR_HEADER)descriptors;

    uint8_t ep[2];

    if ((header->InEndpoint & ENDPOINT_EPNUM_MASK) &&
        (header->InEndpoint & ENDPOINT_DIR_MASK) == ENDPOINT_DIR_IN)
    {
        ep[0] = header->InEndpoint;
    }
    else
    {//IN endpoint is invalid
        ep[0] = 0;
    }

    //NOTE: AVR8 does not support sharing IN and OUT endpoints with same address, so we check this below.
    if ((header->OutEndpoint & ENDPOINT_EPNUM_MASK) &&
        (header->OutEndpoint & ENDPOINT_EPNUM_MASK) != (header->InEndpoint & ENDPOINT_DIR_MASK) &&
        (header->OutEndpoint & ENDPOINT_DIR_MASK) == ENDPOINT_DIR_OUT)
    {
        ep[1] = header->OutEndpoint;
    }
    else
    {//OUT endpoint is invalid
        ep[1] = 0;
    }

    if ((ep[0] & ENDPOINT_EPNUM_MASK) > (ep[1] & ENDPOINT_EPNUM_MASK))
    {//swap endpoints because firmware is compiled with ORDERED_EP_CONFIG and endpoints must be in ascending order
        uint8_t ep0 = ep[0];

        ep[0] = ep[1];
        ep[1] = ep0;
    }

    inEndpoint = 0;
    outEndpoint = 0;

    for (uint8_t i = 0; i < 2; i++)
    {
        if (ep[i] &&
            Endpoint_ConfigureEndpoint(ep[i],
                EP_TYPE_INTERRUPT,
                REWASD_GIMX_MAX_PAYLOAD_SIZE_EP,
                1))
        {
            if ((ep[i] & ENDPOINT_DIR_MASK) == ENDPOINT_DIR_OUT)
            {
                outEndpoint = ep[i];
            }
            else
            {
                inEndpoint = ep[i];
            }
        }
    }

    if (inEndpoint)
    {
        uint8_t param = 0x01;

        send_buffer_with_crc(REWASD_GIMX_PACKET_TYPE_CONNECT, &param, 1);
    }
}

//Called from USB_USBTask.
uint16_t CALLBACK_USB_GetDescriptor(
    const uint16_t wValue,
    const uint16_t wIndex,
    const void** const DescriptorAddress)
{
    PREWASD_GIMX_DESCRIPTOR_HEADER header = (PREWASD_GIMX_DESCRIPTOR_HEADER)descriptors;
    PREWASD_GIMX_DESCRIPTOR_INDEX  index = header->descriptorIndex;
    uint16_t                       offset = sizeof(REWASD_GIMX_DESCRIPTOR_HEADER);

    for (uint8_t i = 0; i < REWASD_GIMX_MAX_DESCRIPTORS; i++, index++)
    {
        if (!index->wLength)
        {
            break;
        }

        if (wValue == index->wValue && wIndex == index->wIndex)
        {
            *DescriptorAddress = descriptors + offset;

            return index->wLength;
        }

        offset += index->wLength;
    }

    return 0;
}

//Called from USB_USBTask.
void EVENT_USB_Device_ControlRequest(void)
{
    waiting_control_reply = false;

    if ((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_TYPE) == REQTYPE_STANDARD)
    {//Let LUFA handle all standard requests
        //Handle REQ_GetInterface and REQ_SetInterface here because LUFA does not handle them.
        //reWASD currently supports only AltSetting 0 on main interface, which has inEndpoint and outEndpoint.
        //So any Set Interface requests are just completed with success.
        if (((USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST) &&
              USB_ControlRequest.bRequest == REQ_GetInterface)
              ||
            (!(USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST) &&
                 USB_ControlRequest.bRequest == REQ_SetInterface))
        {
            Endpoint_ClearSETUP();

            if (USB_ControlRequest.bRequest == REQ_GetInterface && USB_ControlRequest.wLength)
            {//always return setting 0
                uint8_t setting = 0;

                Endpoint_Write_Control_Stream_LE(&setting, 1);
            }

            Endpoint_ClearStatusStage();
        }

        return;
    }

    uint8_t data_length = 0;

    //Increment control_sequence - it will be used to verify that reply refers to the same CONTROL packet.
    control_sequence++;

    control_buf[0] = control_sequence;

    memcpy(control_buf + 1, &USB_ControlRequest, sizeof(USB_ControlRequest));

    //Tell LUFA we have processed this request and do not touch it anymore.
    Endpoint_ClearSETUP();

    if (!(USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST))
    {//get additional bytes from host if any
        if (USB_ControlRequest.wLength > REWASD_GIMX_MAX_PACKET_SIZE_EP0 || //host is trying to send too much data
            Endpoint_Read_Control_Stream_LE(control_buf + sizeof(USB_ControlRequest) + 1, USB_ControlRequest.wLength) != ENDPOINT_RWSTREAM_NoError)
        {
            Endpoint_StallTransaction();

            return;
        }

        data_length = (uint8_t)USB_ControlRequest.wLength;
    }

    waiting_control_reply = true;

    //Send 2 copies of control packets to process by reWASD for reliability (reWASD will drop 2nd copy with same control_sequence).
    send_buffer_with_crc(REWASD_GIMX_PACKET_TYPE_CONTROL + data_length, control_buf, sizeof(USB_ControlRequest) + data_length + 1);
    send_buffer_with_crc(REWASD_GIMX_PACKET_TYPE_CONTROL + data_length, control_buf, sizeof(USB_ControlRequest) + data_length + 1);
}

//Processes new packets from reWASD.
//Note that version command will cause reset!
void SendNextInput(void)
{
    uint8_t len = receive_end - receive_start;

    while (len)
    {
        uint8_t Cmd = receive_buf[receive_start];
        uint8_t combine = false;
        uint8_t minlen;
        uint8_t* target_buf;
        uint8_t signature[6];

        if (Cmd == REWASD_GIMX_PACKET_TYPE_VERSION)
        {
            minlen = 6;

            target_buf = signature;
        }
        else if (Cmd >= REWASD_GIMX_PACKET_TYPE_REPORT &&
            Cmd <= (REWASD_GIMX_PACKET_TYPE_REPORT + 0x3F))
        {
            minlen = (Cmd - REWASD_GIMX_PACKET_TYPE_REPORT) + 1;

            if (((PREWASD_GIMX_DESCRIPTOR_HEADER)descriptors)->flags & REWASD_GIMX_FLAG_COMBINE)
            {
                if (inputDataLen2)
                {
                    //First and second IN reports was still not transferred to host but third packet has arrived.
                    //Check if we can just add data to tail of second packet.
                    if ((inputDataLen2 + minlen) <= REWASD_GIMX_MAX_PAYLOAD_SIZE_EP)
                    {
                        target_buf = input_buf2 + inputDataLen2;

                        combine = true;
                    }
                }
                else if (inputDataLen1)
                {
                    //First IN report was still not transferred to host but second packet has arrived.
                    //Check if we can just add data to tail of first packet.
                    if ((inputDataLen1 + minlen) <= REWASD_GIMX_MAX_PAYLOAD_SIZE_EP)
                    {
                        target_buf = input_buf1 + inputDataLen1;

                        combine = true;
                    }
                }
            }

            if (!combine)
            {
                if (inputDataLen2)
                {//First and second IN reports was still not transferred to host but third packet has arrived.
                    //Copy data of second packet to current (its data will be lost) and prepare new data in input_buf2.
                    //So if reWASD sends reports faster than the host retrieves them, then only 2 last actual IN reports will be kept.
                    memcpy(input_buf1, input_buf2, inputDataLen2);

                    inputDataLen1 = inputDataLen2;

                    inputDataLen2 = 0;
                }

                if (inputDataLen1)
                {
                    //Prepare IN report in the second buffer because the first one is still not transferred.
                    target_buf = input_buf2;
                }
                else
                {
                    //Prepare IN report in the first buffer directly as it has no data pending.
                    target_buf = input_buf1;
                }
            }
        }
        else if (Cmd >= REWASD_GIMX_PACKET_TYPE_CONTROL &&
            Cmd <= (REWASD_GIMX_PACKET_TYPE_CONTROL + 0x40))
        {
            minlen = Cmd - REWASD_GIMX_PACKET_TYPE_CONTROL + sizeof(USB_ControlRequest) + 1;

            target_buf = control_buf;
        }
        else if (Cmd == REWASD_GIMX_PACKET_TYPE_STALL)
        {
            minlen = sizeof(USB_ControlRequest) + 1;

            target_buf = signature;
        }
        else
        {//invalid command
            minlen = 0;
        }

        if (minlen)
        {
            if ((len - 1) < (minlen + 1))
            {//not enough data yet
                break;
            }

            uint8_t offset = receive_start + 1;

            uint8_t crc = _crc8_ccitt_update(0xFF, Cmd);

            for (uint8_t i = 0; i < minlen; i++, offset++)
            {
                uint8_t byte = receive_buf[offset];

                target_buf[i] = byte;

                crc = _crc8_ccitt_update(crc, byte);
            }

            if (crc == receive_buf[offset])
            {
                if (Cmd == REWASD_GIMX_PACKET_TYPE_VERSION)
                {
                    if (signature[0] != 'r' ||
                        signature[1] != 'e' ||
                        signature[2] != 'W' ||
                        signature[3] != 'A' ||
                        signature[4] != 'S' ||
                        signature[5] != 'D')
                    {//invalid version command
                        goto unknown_command;
                    }

                    forceHardReset();
                }

                if (Cmd >= REWASD_GIMX_PACKET_TYPE_CONTROL)
                {//CONTROL reply has arrived
                    if (control_sequence == control_buf[0] &&
                        waiting_control_reply &&
                        !memcmp(control_buf + 1, &USB_ControlRequest, sizeof(USB_ControlRequest)))
                    {
                        Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);

                        //NOTE: reWASD salways sends 2 copies of control reply for reliability -
                        //resetting this flag indicates that 2nd copy should be dropped if it has same control_sequence.
                        waiting_control_reply = false;

                        if (Cmd == REWASD_GIMX_PACKET_TYPE_STALL)
                        {
                            //Finally complete the whole control request with error.
                            Endpoint_StallTransaction();
                        }
                        else
                        {
                            if (USB_ControlRequest.bmRequestType & REQDIR_DEVICETOHOST)
                            {
                                uint8_t value_len = minlen - sizeof(USB_ControlRequest) - 1;

                                if (value_len > USB_ControlRequest.wLength)
                                {//make sure we return only as much bytes as host has requested
                                    value_len = USB_ControlRequest.wLength;
                                }

                                //It is important to note that it calls Endpoint_ClearIN automatically, thus freeing up the endpoint for the next packet.
                                Endpoint_Write_Control_Stream_LE(control_buf + sizeof(USB_ControlRequest) + 1, value_len);
                            }

                            //Finally complete the whole control request as successful.
                            Endpoint_ClearStatusStage();
                        }
                    }
                }
                else
                {//IN
                    if (combine)
                    {
                        if (inputDataLen2)
                        {//update length of second buffer
                            inputDataLen2 += minlen;
                        }
                        else
                        {//update length of first buffer
                            inputDataLen1 += minlen;
                        }
                    }
                    else
                    {
                        if (inputDataLen1)
                        {//update length of second buffer
                            inputDataLen2 = minlen;
                        }
                        else
                        {//update length of first buffer
                            inputDataLen1 = minlen;
                        }
                    }
                }

                receive_start += (minlen + 2);
                len -= (minlen + 2);
            }
            else
            {
                if (use_debug)
                {
                    //Send debug packet to indicate bad CRC
                    uint8_t dbg[2];

                    dbg[0] = 0x22;
                    dbg[1] = Cmd;

                    send_buffer_with_crc(REWASD_GIMX_PACKET_TYPE_DEBUG, dbg, sizeof(dbg));
                }

                receive_start++;
                len--;
            }
        }
        else
        {
        unknown_command:

            if (use_debug)
            {
                //Send debug packet to indicate unknown command
                uint8_t dbg[2];

                dbg[0] = 0x11;
                dbg[1] = Cmd;

                send_buffer_with_crc(REWASD_GIMX_PACKET_TYPE_DEBUG, dbg, sizeof(dbg));
            }

            receive_start++;
            len--;
        }
    }

    if (USB_DeviceState == DEVICE_STATE_Configured && inEndpoint && inputDataLen1)
    {
        Endpoint_SelectEndpoint(inEndpoint);

        if (Endpoint_IsINReady())
        {
            uint8_t* buffer = input_buf1;

            while (Endpoint_IsReadWriteAllowed())
            {
                Endpoint_Write_8(*buffer++);

                inputDataLen1--;

                if (!inputDataLen1)
                {
                    break;
                }
            }

            Endpoint_ClearIN();

            if (inputDataLen2)
            {//next IN packet is already pending, copy it to input_buf1 so it will be transferred next time.
                memcpy(input_buf1, input_buf2, inputDataLen2);

                inputDataLen1 = inputDataLen2;

                inputDataLen2 = 0;
            }
            else
            {//no new IN packet is available
                inputDataLen1 = 0;
            }
        }
    }
}

//Processes new OUT packets to reWASD.
void ReceiveNextOutput(void)
{
    if (USB_DeviceState == DEVICE_STATE_Configured &&
        outEndpoint)
    {
        Endpoint_SelectEndpoint(outEndpoint);

        if (Endpoint_IsOUTReceived())
        {
            uint8_t* buffer = out_buf;
            uint16_t length = 0;

            while (Endpoint_IsReadWriteAllowed())
            {
                *buffer++ = Endpoint_Read_8();

                length++;

                if (length == sizeof(out_buf))
                {
                    break;
                }
            }

            Endpoint_ClearOUT();

            if (length)
            {
                send_buffer_with_crc(REWASD_GIMX_PACKET_TYPE_REPORT + length - 1, out_buf, length);
            }
        }
    }

    if (TCNT1 > KEEP_ALIVE_COUNT)
    {//need to send keep-alive packet
        Serial_SendByte(REWASD_GIMX_PACKET_TYPE_KEEP_ALIVE);

        TCNT1 = 0;
    }
}

void SetupHardware(void)
{
    uint8_t i;

    // Disable watchdog.
    MCUSR = 0;
    wdt_disable();

    clock_prescale_set(clock_div_1);

    TCCR1B |= (1 << CS12) | (1 << CS10); // Set up timer at FCPU / 1024  (15625 ticks per second, 1 ms = 15.625 ticks)

    Serial_Init(REWASD_GIMX_BAUDRATE, true);

    //Inform reWASD that hardware reset occured
    send_version();

    for (uint8_t i = 0; i < 10; i++)
    {
        //drain receive buffer
        while (Serial_IsCharReceived())
        {
            UDR1;
        }

        Delay_MS(1);
    }

    UCSR1B = ((1 << TXEN1) | (1 << RXEN1) | (1 << RXCIE1));//Enable the USART Receive Complete interrupt (USART1_RX_vect)

    GlobalInterruptEnable();

    while (!started)
    {
        uint8_t end = receive_end;
        uint8_t len = end - receive_start;

        if (len)
        {
            if (receive_buf[receive_start] == REWASD_GIMX_PACKET_TYPE_VERSION)
            {
                if (len >= 8)
                {
                    uint8_t signature[6];
                    uint8_t offset = receive_start + 1;

                    for (i = 0; i < 6; i++, offset++)
                    {
                        signature[i] = receive_buf[offset];
                    }

                    if (signature[0] != 'r' ||
                        signature[1] != 'e' ||
                        signature[2] != 'W' ||
                        signature[3] != 'A' ||
                        signature[4] != 'S' ||
                        signature[5] != 'D')
                    {//invalid version command
                        receive_start++;
                    }
                    else
                    {
                        uint8_t crc = _crc8_ccitt_update(0xFF, REWASD_GIMX_PACKET_TYPE_VERSION);

                        for (i = 0; i < 6; i++)
                        {
                            crc = _crc8_ccitt_update(crc, signature[i]);
                        }

                        if (crc == receive_buf[offset])
                        {
                            send_version();

                            desc_received = 0;

                            receive_start += 8;
                        }

                        else
                        {
                            receive_start++;
                        }
                    }
                }
            }
            else if (receive_buf[receive_start] == REWASD_GIMX_PACKET_TYPE_DESCRIPTORS)
            {
                uint8_t minlen;

                if (desc_received)
                {
                    PREWASD_GIMX_DESCRIPTOR_HEADER header = (PREWASD_GIMX_DESCRIPTOR_HEADER)descriptors;

                    if ((header->wTotalLength - desc_received) >= REWASD_GIMX_MAX_PACKET_VALUE_SIZE)
                    {
                        minlen = REWASD_GIMX_MAX_PACKET_VALUE_SIZE;
                    }
                    else
                    {
                        minlen = header->wTotalLength - desc_received;
                    }
                }
                else
                {
                    minlen = sizeof(REWASD_GIMX_DESCRIPTOR_HEADER);
                }

                if ((len - 1) >= (minlen + 1))
                {
                    uint8_t offset = receive_start + 1;

                    uint8_t crc = _crc8_ccitt_update(0xFF, REWASD_GIMX_PACKET_TYPE_DESCRIPTORS);

                    for (i = 0; i < minlen; i++, offset++)
                    {
                        uint8_t byte = receive_buf[offset];

                        descriptors[desc_received + i] = byte;

                        crc = _crc8_ccitt_update(crc, byte);
                    }

                    if (crc == receive_buf[offset])
                    {
                        PREWASD_GIMX_DESCRIPTOR_HEADER header = (PREWASD_GIMX_DESCRIPTOR_HEADER)descriptors;

                        desc_received += minlen;

                        if (header->wTotalLength < sizeof(REWASD_GIMX_DESCRIPTOR_HEADER) || header->wTotalLength > REWASD_GIMX_MAX_DESCRIPTORS_SIZE)
                        {
                            desc_received = 0;
                        }
                        else
                        {
                            if (header->wTotalLength == desc_received)
                            {
                                if (header->flags & REWASD_GIMX_FLAG_DEBUG)
                                {
                                    use_debug = true;
                                }

                                started = true;//exit this loop and start USB
                            }
                        }

                        send_buffer_with_crc(REWASD_GIMX_PACKET_TYPE_DESCRIPTORS, (uint8_t*)&desc_received, sizeof(desc_received));

                        receive_start += (minlen + 2);
                    }
                    else
                    {
                        receive_start++;
                    }
                }
            }
            else
            {//invalid command
                receive_start++;
            }
        }
    }

    USB_Init();

    TCNT1 = 0;
}

int main(void)
{
    SetupHardware();

    while (1)
    {
        SendNextInput();

        ReceiveNextOutput();

        USB_USBTask();//NOTE: EVENT_USB_Device_ControlRequest will be called from inside this routine.
    }
}

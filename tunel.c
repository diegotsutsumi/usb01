/*
 * File:   tunel.c
 * Author: diego.tsutsumi
 *
 * Created on 17th December 2014, 14:38
 */

#ifndef TUNEL_C
#define	TUNEL_C

#include "tunel.h"

void TunelLoginPacket(uint8_t * in_serialNumber, uint8_t * in_firmwareVersion, uint8_t * out_packetPointer, uint16_t * out_size)
{
    uint8_t crc;
    out_packetPointer[0] = 0x74; //Start Packet
    out_packetPointer[1] = 0x1B; //My Address
    out_packetPointer[2] = 0x01; //Server Address
    out_packetPointer[3] = 0x00; //Size
    out_packetPointer[4] = 0x0D; //Size

    out_packetPointer[5] = 0x5B; //Start Payload

    //Firmware Version - 6 bytes ASCII
    out_packetPointer[6] = in_firmwareVersion[0];
    out_packetPointer[7] = in_firmwareVersion[1];
    out_packetPointer[8] = in_firmwareVersion[2];
    out_packetPointer[9] = in_firmwareVersion[3];
    out_packetPointer[10] = in_firmwareVersion[4];
    out_packetPointer[11] = in_firmwareVersion[5];

    //Serial Number - 6 bytes ASCII
    out_packetPointer[12] = in_serialNumber[0];
    out_packetPointer[13] = in_serialNumber[1];
    out_packetPointer[14] = in_serialNumber[2];
    out_packetPointer[15] = in_serialNumber[3];
    out_packetPointer[16] = in_serialNumber[4];
    out_packetPointer[17] = in_serialNumber[5];

    calcCRC(out_packetPointer,&(crc));
    
    //CRC
    out_packetPointer[18] = crc;
    *out_size = 19;

}

void TunelServerAnswer(bool answer, uint8_t * out_packetPointer, uint16_t * out_size)
{
    uint8_t crc;
    out_packetPointer[0] = 0x74; //Start Packet
    out_packetPointer[1] = 0x1B; //My Address
    out_packetPointer[2] = 0x01; //Server Address
    out_packetPointer[3] = 0x00; //Size
    out_packetPointer[4] = 0x02; //Size

    if(answer)
    {
        out_packetPointer[5] = 0x5B;
        out_packetPointer[6] = 0x5B;
    }
    else
    {
        out_packetPointer[5] = 0x5A;
        out_packetPointer[6] = 0x5A;
    }

    calcCRC(out_packetPointer,&(crc));

    //CRC
    out_packetPointer[7] = crc;
    *out_size = 8;
}

void calcCRC(uint8_t * in_packet, uint8_t * out_crc)
{
    uint8_t checksum;
    uint16_t size,i;

    size = in_packet[3];
    size = size << 8;
    size = in_packet[4] + 5;

    checksum = 0x35;
    if(in_packet[2] > 0x02)
        checksum ^= in_packet[2];

    for(i=0;i<size;i++)
    {
        checksum ^= *in_packet;
        in_packet++;
    }
    *(out_crc) = checksum;
}

bool checkCRC(uint8_t * packet)
{
    uint8_t checksum,i;
    uint16_t size;

    size = packet[3];
    size = (size<<8) + packet[4] + 6;

    checksum = 0x35;
    checksum ^= 0x1B; //My Address
    for(i = 0; i< size; i++)
    {
        checksum ^= *packet;
        packet++;
    }
    if(checksum)
        return FALSE;
    
    return TRUE;
}

void fwByteCRC(BYTE value, WORD_VAL * checksum)
{
    BYTE i;

    (*checksum).v[1] ^= value;
    for (i = 0; i < 8 ; i++)
    {
        if ((*checksum).v[1] & 0x80)
        {
            (*checksum).Val <<= 1;
            (*checksum).v[1] ^= 0x10;
            (*checksum).v[0] ^= 0x21;
        }
        else
        {
            (*checksum).Val <<= 1;
        }
    }
}

#endif
/* 
 * File:   tunel.h
 * Author: diego.tsutsumi
 *
 * Created on 17th Dezembro 2014, 14:48
 */

#ifndef _TUNEL_H_
#define	_TUNEL_H_

#include "includes.h"

void TunelLoginPacket(uint8_t * in_serialNumber, uint8_t * in_firmwareVersion,
                        uint8_t * out_packetPointer, uint16_t * out_size);

void TunelFwAnswer(bool in_cmdOk, uint8_t * out_packetPointer, uint16_t * out_size);

void calcCRC(BYTE * in_packet, BYTE * out_crc);

bool checkCRC(uint8_t * packet);

void fwByteCRC(BYTE value, WORD_VAL * checksum);

#endif	/* TUNEL_H */


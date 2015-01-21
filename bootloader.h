/* 
 * File:   bootloader.h
 * Author: diego.tsutsumi
 *
 * Created on 6th January 2015, 10:52
 */

#ifndef _BOOTLOADER_H_
#define	_BOOTLOADER_H_

#include <xc.h>
#include <GenericTypeDefs.h>
#include <sys/kmem.h>

#define BootSection __attribute__ ((section (".bloader")))

#define KSEG0_MAIN_PROGRAM_START 0x9D000000

void BootLoader(UINT16 size);
UINT32 NVMUnlock (UINT32 nvmop);
UINT32 NVMWriteRow (UINT32 nvmAddress, void* SRAMDataPointer);
UINT32 NVMErasePFM(void);
void readExtFlash512Bytes(DWORD_VAL addr);

#endif	/* BOOTLOADER_H */
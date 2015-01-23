/*
 * File:   bootloader.c
 * Author: diego.tsutsumi
 *
 * Created on 6th January 2015, 10:52
 */

#include "bootloader.h"

UINT32 newFwRow[128];
UINT32 flashAddr;

void BootSection BootLoader(UINT16 size)
{
    UINT16 i;
    DWORD_VAL extFlashAddr;

    PORTBbits.RB13 = 1;
    
    NVMErasePFM();

    extFlashAddr.Val=0;
    flashAddr = KSEG0_MAIN_PROGRAM_START;

    for(i=0;i<size;i+=512)
    {
        readExtFlash512Bytes(extFlashAddr);
        NVMWriteRow(flashAddr,(void*)&newFwRow[0]);
        extFlashAddr.Val += 512;
        flashAddr += 128;
    }

    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;

    RSWRSTSET = 1;

    unsigned int dummy;
    dummy = RSWRST;
    while(1);

}

UINT32 BootSection NVMUnlock(UINT32 nvmop)
{
    UINT32 status;
    asm volatile ("di %0" : "=r" (status));
    NVMCON = nvmop;
    NVMKEY = 0xAA996655;
    NVMKEY = 0x556699AA;

    NVMCONSET = 0x8000;
    
    while (NVMCON & 0x8000);
    if (status & 0x00000001)
    {
        asm volatile ("ei");
    }
    else
    {
        asm volatile ("di");
    }
    NVMCONCLR = 0x0004000;
    return (NVMCON & 0x3000);
}

UINT32 BootSection NVMWriteRow (UINT32 nvmAddress, void* SRAMDataPointer)
{
    NVMADDR = KVA_TO_PA(nvmAddress);
    NVMSRCADDR = KVA_TO_PA(SRAMDataPointer);
    return NVMUnlock(0x4003);
}

UINT32 BootSection NVMErasePFM(void)
{
    return NVMUnlock(0x4005);
}

void BootSection readExtFlash512Bytes(DWORD_VAL addr)
{
    UINT8 i,j,buff[4];
    UINT32 wbuff;

    if(!SPI1CONbits.ON)
    {
        SPI1CONbits.ON=1;
    }

    PORTBbits.RB13 = 0;
    SPI1BUF = 0x03;
    while(!SPI1STATbits.SPIRBF);

    SPI1BUF = addr.byte.UB;
    while(!SPI1STATbits.SPIRBF);
    SPI1BUF = addr.byte.HB;
    while(!SPI1STATbits.SPIRBF);
    SPI1BUF = addr.byte.LB;
    while(!SPI1STATbits.SPIRBF);

    for(i=0;i<128;i++)
    {
        wbuff = 0;

        for(j=0;j<4;j++)
        {
            SPI1BUF = 0x00;
            while(!SPI1STATbits.SPIRBF);
            buff[j] = SPI1BUF;
            wbuff = wbuff | (UINT32)(buff[j] << (j*8));
        }

        newFwRow[i] = wbuff;
    }
    PORTBbits.RB13 = 1;
}

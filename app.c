/*
 * File:   app.c
 * Author: diego.tsutsumi
 *
 * Created on 10th November 2014, 10:12
 */
#ifndef APP_C
#define APP_C

#include "app.h"
#include "bsp_config.h"

APP_DATA appData;

#ifdef MemTest
static uint8_t memTestBuff[300] =
{
    0,1,2,3,4,5,6,7,8,9,
    10,11,12,13,14,15,16,17,18,19,
    20,21,22,23,24,25,26,27,28,29,
    30,31,32,33,34,35,36,37,38,39,
    40,41,42,43,44,45,46,47,48,49,
    50,51,52,53,54,55,56,57,58,59,
    60,61,62,63,64,65,66,67,68,69,
    70.71,72,73,74,75,76,77,78,79,
    80,81,82,83,84,85,86,87,88,89,
    90,91,92,93,94,95,96,97,98,99,
    100,101,102,103,104,105,106,107,108,109,
    110,111,112,113,114,115,116,117,118,119,
    120,121,122,123,124,125,126,127,128,129,
    130,131,132,133,134,135,136,137,138,139,
    140,141,142,143,144,145,146,147,148,149,
    150,151,152,153,154,155,156,157,158,159,
    160,161,162,163,164,165,166,167,168,169,
    170,171,172,173,174,175,176,177,178,179,
    180,181,182,183,184,185,186,187,188,189,
    190,191,192,193,194,195,196,197,198,199,
    200,201,202,203,204,205,206,207,208,209,
    210,211,212,213,214,215,216,217,218,219,
    220,221,222,223,224,225,226,227,228,229,
    230,231,232,233,234,235,236,237,238,239,
    240,241,242,243,244,245,246,247,248,249,
    250,251,252,253,254,255,255,255,2,3,
    4,5,6,7,8,9,10,11,12,13,
    14,15,16,17,18,19,20,21,22,23,
    24,25,26,27,28,29,30,31,32,33,
    34,35,36,37,38,39,40,41,42,43,
};
#endif


static BYTE serialNumber[6]="000000";
static BYTE fwVersion[6]="1.0";

//local variables
bool entry_flag;

void APP_Initialize ( void )
{
#ifdef MemTest
    appData.current_state = APP_STATE0_MemoryTesting;
#else
    appData.current_state = APP_STATE0_Initializing;
#endif
    appData.sysTmrHandle = 0;

    appData.serverPacketSize=0;
    appData.serverPacket=0;
    appData.serverBuffHandler=255;

    appData.loginPacketSize=0;
    appData.loginPacket=0;
    appData.loginBuffHandler=255;

    appData.i2cRxSize=0;
    appData.i2cRX=0;
    appData.genericRxHandler=254;
    appData.androidHandler=254;
    appData.memHandler=254;

    appData.readUSBSize=0;
    appData.readUSB=0;
    appData.i2cTxHandler=255;

    appData.AVLDataReady=0;
    appData.andrDataReady=false;
    appData.loggedIntoServer=false;
    appData.andrConnected=false;


    appData.performFlashBoot=false;

    appData.fwSizeCount.Val=0;
    appData.fwPacketSize.Val=0;
    appData.fwLastOffset.Val=0;
    appData.PPPChecksumOut.Val=0;
    appData.fwUpdating=false;
    appData.memWriting=false;
    appData.fwCRC=9;

    entry_flag=true;

    appData.srvAnswer=APP_SERVER_NOANSWER;

    appData.txI2C=0;
    appData.rxI2C=0;
    appData.txUSB=0;
    appData.rxUSB=0;

#ifdef MemTest
    appData.checkMem=0;
#endif
}

void APP_Tasks ( void )
{
    switch (appData.current_state)
    {
        case APP_STATE0_Initializing:
        {
            if(entry_flag)
            {
                I2C_InitObject(); //Initializing I2C
                I2C_SetEventHandler(APP_I2CEventHandler);

                AND_InitObject(); //Initializing android driver
                AND_SetEventHandler(APP_AndrEventHandler);

                entry_flag = true;
                appData.current_state = APP_STATE0_Running;
            }
        }
        break;

#ifdef MemTest
        case APP_STATE0_MemoryTesting:
        {
            int i,j;
            char dummy;
            if(entry_flag)
            {
                entry_flag=false;
                MEM_InitObj(true);
                MEM_SetEventHandler(APP_MEMEventHandler);
                MEM_FillBuffer(memTestBuff,300);
                appData.checkMem = 0;
            }
            if(appData.checkMem==1)
            {
                PORTCbits.RC9 = 0;
                SPI2BUF = 0x03;
                while(!SPI2STATbits.SPIRBF);
                dummy = SPI2BUF;

                SPI2BUF = 0;
                while(!SPI2STATbits.SPIRBF);
                dummy = SPI2BUF;

                SPI2BUF = 0;
                while(!SPI2STATbits.SPIRBF);
                dummy = SPI2BUF;

                SPI2BUF = 0;
                while(!SPI2STATbits.SPIRBF);
                dummy = SPI2BUF;

                for(j=0;j<600;j++)
                {
                    SPI2BUF = 0x00;
                    while(!SPI2STATbits.SPIRBF);
                    appData.checkBuffer[j] = SPI2BUF;
                }
                
                appData.checkMem=2;
                PORTCbits.RC9 = 1;
            }
            else if(appData.checkMem==2)
            {
                i=0;
                for(j=0;j<600;j++)
                {
                    if(appData.checkBuffer[j] != memTestBuff[j%300])
                    {
                        i=1;
                        break;
                    }
                }
                if(i)
                {
                    appData.current_state = APP_STATE0_MemoryError;
                }
                else
                {
                    appData.current_state = APP_STATE0_MemoryOk;
                }
            }
        }
        break;
        case APP_STATE0_MemoryOk:
        {
            Nop();
        }
        break;
        case APP_STATE0_MemoryError:
        {
            Nop();
        }
        break;
#endif
        
        case APP_STATE0_Running:
        {
            if(entry_flag)
            {
                entry_flag = false;
                //Sending first login Try
                appData.loginPacket = allocI2CTXBuffer(&(appData.loginBuffHandler),&(appData.loginPacketSize));
                if(appData.loginPacket) //Buffer not Full
                {
                    TunelLoginPacket(serialNumber,fwVersion,appData.loginPacket,&(appData.loginPacketSize));
                    I2CStartTX(appData.loginBuffHandler, appData.loginPacketSize,AVL_ADDR);
                    appData.sysTmrHandle = SYS_TMR_DelayMS(5000);
                }
            }
            if(appData.performFlashBoot && appData.srvAnswer==APP_SERVER_NOANSWER)
            {
#ifdef NoDebug
                BootLoader(appData.fwSizeCount.Val);
#endif
            }

            if(appData.andrConnected)
            {
                if(appData.i2cTxHandler==255) //Last try I2C TX buffer was full
                {
                    appData.readUSB = allocI2CTXBuffer(&(appData.i2cTxHandler),&(appData.readUSBSize));
                    if(appData.i2cTxHandler!=255) //TX buffer full
                    {
                        if(!AND_Read(appData.readUSB,appData.readUSBSize))
                        {
                            I2CAppFreeTX(appData.i2cTxHandler);
                            appData.i2cTxHandler=255;
                        }
                    }
                    else
                    {
                        Nop();
                    }
                }
            }

            if(SYS_TMR_DelayStatusGet(appData.sysTmrHandle))
            {
                if(!appData.loggedIntoServer)
                {
                    appData.loginPacket = allocI2CTXBuffer(&(appData.loginBuffHandler),&(appData.loginPacketSize));

                    if(appData.loginPacket) //Buffer not Full
                    {
                        TunelLoginPacket(serialNumber,fwVersion,appData.loginPacket,&(appData.loginPacketSize));
                        I2CStartTX(appData.loginBuffHandler, appData.loginPacketSize,AVL_ADDR);
                        appData.sysTmrHandle = SYS_TMR_DelayMS(5000);
                    }
                }
            }
            else
            {
                if(appData.srvAnswer!=APP_SERVER_NOANSWER)
                {
                    appData.serverPacket = allocI2CTXBuffer(&(appData.serverBuffHandler),&(appData.serverPacketSize));
                    if(appData.serverPacket) //Buffer not Full
                    {
                        TunelServerAnswer((appData.srvAnswer==APP_SERVER_ERROR)?false:true,appData.serverPacket,&(appData.serverPacketSize));
                        I2CStartTX(appData.serverBuffHandler, appData.serverPacketSize,AVL_ADDR);
                    }
                    appData.srvAnswer = APP_SERVER_NOANSWER;
                }
            }

            if(appData.AVLDataReady>0)
            {
                appData.AVLDataReady--;
                appData.i2cRX = getI2CRxBuffer(&(appData.i2cRxSize), &(appData.genericRxHandler));

                if(!appData.i2cRX)
                {
                    appData.current_state = APP_STATE0_Error; // Should never end up here
                }
                
                APP_ProcessAVLPacket();
            }
            
            if(appData.andrDataReady)
            {
                if(appData.readUSB[2]<=2)
                {
                    I2CStartTX(appData.i2cTxHandler, appData.readUSBSize, AVL_ADDR);//i2cBufferHandler should not be 254 if the code falls here
                }
                else
                {
                    I2CStartTX(appData.i2cTxHandler, appData.readUSBSize, appData.readUSB[2]);//i2cBufferHandler should not be 254 if the code falls here
                }

                appData.i2cTxHandler=255;
                appData.andrDataReady = false;
            }
        }
        break;

        case APP_STATE0_Error:
        {
            //Resetting when it ends up in the error state
            SYSKEY = 0x00000000;
            SYSKEY = 0xAA996655;
            SYSKEY = 0x556699AA;

            RSWRSTSET = 1;

            unsigned int dummy;
            dummy = RSWRST;
            while(1);
        }
        break;
        
        default:
        {
            //appData.current_state = APP_STATE0_Error;
        }
        break;
    }
}

void APP_ProcessAVLPacket()
{
    uint16_t size,i;
    if(*(appData.i2cRX+2)==0x1B)
    {
        if(!appData.loggedIntoServer)
        {
            if(appData.i2cRxSize<15) //Server Answer
            {
                if(*(appData.i2cRX+6)==0x60 && *(appData.i2cRX+7)==0x01)
                {
                    appData.loggedIntoServer=true;
                }
                freeI2CRxBuffIndex(appData.genericRxHandler);
                return;
            }
        }
        else
        {
            if(*(appData.i2cRX+5)==0x70)
            {
                if(*(appData.i2cRX+6)==0x05) //Firmware upgrade Finished
                {
                    if(!appData.fwUpdating)
                    {
                        freeI2CRxBuffIndex(appData.genericRxHandler);
                        appData.srvAnswer = APP_SERVER_ERROR;
                        return;
                    }
                    else
                    {
                        if((*(appData.i2cRX+7)!=appData.fwSizeCount.byte.HB) ||
                           (*(appData.i2cRX+8)!=appData.fwSizeCount.byte.LB) ||
                           (appData.fwSizeCount.Val<0x400) ) //Total Size check
                        {
                            freeI2CRxBuffIndex(appData.genericRxHandler);
                            appData.srvAnswer = APP_SERVER_ERROR;
                            return;
                        }

                        if(appData.CRC.Val != appData.PPPChecksumOut.Val)
                        {
                            freeI2CRxBuffIndex(appData.genericRxHandler);
                            appData.srvAnswer = APP_SERVER_ERROR;
                            return;
                        }
                        
                        appData.fwUpdating = false;
                        appData.srvAnswer = APP_SERVER_OK;
                        appData.performFlashBoot = true;
                        freeI2CRxBuffIndex(appData.genericRxHandler);
                        return;
                        //TODO: Start booting
                    }
                }
                else if(*(appData.i2cRX+6)==0x07 &&
                        *(appData.i2cRX+7)==0x06) // Firmware Update
                {
                    if(!appData.fwUpdating) //First Firmware packet
                    {
                        appData.fwUpdating = true;
                        appData.fwCRC = true;
                        MEM_InitObj(true);
                        MEM_SetEventHandler(APP_MEMEventHandler);
                        appData.PPPChecksumOut.Val=0;
                        appData.fwSizeCount.Val=0;
                        appData.CRC.Val=0;
                    }

                    if(!appData.memWriting)
                    {
                        appData.memWriting = true;
                        appData.fwLastOffset.v[1]=*(appData.i2cRX+8);
                        appData.fwLastOffset.v[0]=*(appData.i2cRX+9);
                        appData.fwPacketSize.v[1]=*(appData.i2cRX+10);
                        appData.fwPacketSize.v[0]=*(appData.i2cRX+11);
                        size = appData.i2cRxSize-12;

                        if(size != appData.fwPacketSize.Val)
                        {
                            freeI2CRxBuffIndex(appData.genericRxHandler);
                            appData.srvAnswer = APP_SERVER_ERROR;
                            return; //It shouldn't write a corrupted packet into the Flash
                        }

                        appData.fwSizeCount.Val += size;
                        for(i=0;i<size;i++)
                        {
                            fwByteCRC(*(appData.i2cRX+12+i), &(appData.PPPChecksumOut));
                        }

                        appData.CRC.byte.HB = *(appData.i2cRX+10+size);
                        appData.CRC.byte.LB = *(appData.i2cRX+11+size);

                        appData.memHandler = appData.genericRxHandler;
                        MEM_FillBuffer((appData.i2cRX+12), size);
                        return;
                    }
                    else
                    {
                        freeI2CRxBuffIndex(appData.genericRxHandler);
                        appData.srvAnswer = APP_SERVER_ERROR;
                        return;
                    }
                }
                else
                {
                    freeI2CRxBuffIndex(appData.genericRxHandler);
                    appData.srvAnswer = APP_SERVER_ERROR;
                    return;
                }
            }
            else
            {
                freeI2CRxBuffIndex(appData.genericRxHandler);
                appData.srvAnswer = APP_SERVER_ERROR;
                return;
            }
        }
    }
    else
    {
        if(appData.andrConnected)
        {
            appData.androidHandler = appData.genericRxHandler;
            if(!AND_Write(appData.i2cRX,appData.i2cRxSize))
            {
                freeI2CRxBuffIndex(appData.genericRxHandler); //Dropping packet in error case
                return;
            }
        }
        else
        {
            freeI2CRxBuffIndex(appData.genericRxHandler); //Dropping packet if Android is not connected
            return;
        }
    }
}

void APP_I2CEventHandler(I2C_EVENT event/*, void * eventData*/)
{
    switch(event)
    {
        case I2C_EVENT_DATA_READY:
        {
            appData.rxI2C++;
            appData.AVLDataReady++;
        }
        break;
        case I2C_EVENT_DATA_SENT:
        {
            appData.txI2C++;
            //TODO: Implement a way to check which handler was sent
        }
        break;
        case I2C_EVENT_DATA_NOT_SENT:
        {
            appData.txErrorI2C++;
            //TODO: Implement a way to check which handler wasn't sent
        }
        break;
        default:
        {
        }
        break;
    }
}

void APP_AndrEventHandler(AND_EVENT event, uint32_t eventData)
{
    switch(event)
    {
        case AND_EVENT_CONNECTED:
        {
            appData.andrConnected = true;
        }
        break;
        case AND_EVENT_DISCONNECTED:
        {
            if(appData.i2cTxHandler!=255)
            {
                I2CAppFreeTX(appData.i2cTxHandler);
                appData.i2cTxHandler=255;
            }

            appData.andrConnected = false;
        }
        break;
        case AND_EVENT_DATA_READY:
        {
            appData.readUSBSize=eventData;
            appData.andrDataReady=true;
            appData.rxUSB++;
        }
        break;
        case AND_EVENT_DATA_SENT:
        {
            appData.txUSB++;
            freeI2CRxBuffIndex(appData.androidHandler);
        }
        break;
        case AND_EVENT_DATA_NOT_SENT:
        {
            appData.txErrorUSB++;
            freeI2CRxBuffIndex(appData.androidHandler);
        }
        break;
        default:
        {
        }
        break;
    }
}

#ifdef MemTest
int x=0;
#endif

void APP_MEMEventHandler(MEM_EVENT event)
{
    switch(event)
    {
        case MEM_EVENT_ERASED:
        {

        }
        break;
        case MEM_EVENT_BUFFER_WRITTEN:
        {
#ifdef MemTest
            if(x==0)
            {
                MEM_FillBuffer(memTestBuff,300);
                x=1;
            }
            else
            {
                appData.checkMem = 1;
            }
#else
            freeI2CRxBuffIndex(appData.memHandler);
            appData.memWriting = false;
            appData.srvAnswer = APP_SERVER_OK;
#endif
        }
        break;
        case MEM_EVENT_BUFFER_NOT_WRITTEN:
        {
#ifdef MemTest
            Nop();
#else
            freeI2CRxBuffIndex(appData.memHandler);
            appData.memWriting = false;
            appData.srvAnswer = APP_SERVER_ERROR;
#endif
        }
        break;
        default:
        {
        }
        break;
    }
}

#endif
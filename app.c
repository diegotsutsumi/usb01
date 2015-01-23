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

static BYTE serialNumber[6]="000000";
static BYTE fwVersion[6]="1.0";

//local variables
bool entry_flag;

void APP_Initialize ( void )
{
    appData.current_state =  APP_STATE0_Initializing;
    appData.sysTmrHandle = 0;

    appData.serverPacketSize=0;
    appData.serverPacket=0;
    appData.serverBuffHandler=255;

    appData.i2cRxSize=0;
    appData.i2cRX=0;
    appData.genericRxHandler=254;
    appData.androidHandler=254;
    appData.memHandler=254;

    appData.readUSBSize=0;
    appData.readUSB=0;
    appData.i2cBufferHandler=255;

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

    appData.srvAnswer=APP_SERVER_NOANSWER;
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
                AND_InitObject(); //Initializing android driver
                appData.current_state = APP_STATE0_Running;
            }
        }
        break;
        
        case APP_STATE0_Running:
        {
            if(appData.performFlashBoot)
            {
                BootLoader(appData.fwSizeCount.Val);
            }
            if(appData.i2cBufferHandler==255) //Last try I2C TX buffer was full
            {
                if(appData.andrConnected)
                {
                    appData.readUSB = allocI2CTXBuffer(&(appData.i2cBufferHandler),&(appData.readUSBSize));
                    if(appData.i2cBufferHandler!=255) //TX buffer full
                    {
                        if(!AND_Read(appData.readUSB,appData.readUSBSize))
                        {
                            //TODO: This error is not being treated yet. Implement it
                        }
                    }
                }
            }
            
            if(!appData.loggedIntoServer) //TODO: Implement the first login before 5 seconds
            {
                if(SYS_TMR_DelayStatusGet(appData.sysTmrHandle))
                {
                    appData.serverPacket = allocI2CTXBuffer(&(appData.serverBuffHandler),&(appData.serverPacketSize));
                    //TODO: Test if buffer full (255)
                    TunelLoginPacket(serialNumber,fwVersion,appData.serverPacket,&(appData.serverPacketSize));
                    I2CStartTX(appData.serverBuffHandler, appData.serverPacketSize,AVL_ADDR);
                    appData.sysTmrHandle = SYS_TMR_DelayMS(5000);
                }
            }
            else
            {
                if(appData.srvAnswer!=APP_SERVER_NOANSWER)
                {
                    //TODO: Send Answer to a server
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
                    I2CStartTX(appData.i2cBufferHandler, appData.readUSBSize, AVL_ADDR);//i2cBufferHandler should not be 254 if the code falls here
                }
                else
                {
                    I2CStartTX(appData.i2cBufferHandler, appData.readUSBSize, appData.readUSB[2]);//i2cBufferHandler should not be 254 if the code falls here
                }

                if(appData.andrConnected)
                {
                    appData.readUSB = allocI2CTXBuffer(&(appData.i2cBufferHandler),&(appData.readUSBSize));
                    if(appData.i2cBufferHandler!=255) //TX buffer full
                    {
                        if(!AND_Read(appData.readUSB,appData.readUSBSize))
                        {
                            //TODO: This error is not being treated yet. Implement it
                        }
                    }
                }
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
            appData.current_state = APP_STATE0_Error;
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
                    freeI2CRxBuffIndex(appData.genericRxHandler);
                    return;
                }
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
                           (*(appData.i2cRX+8)!=appData.fwSizeCount.byte.LB)) //Total Size check
                        {
                            appData.srvAnswer = APP_SERVER_ERROR;
                            return;
                        }

                        if(appData.CRC.Val != appData.PPPChecksumOut.Val)
                        {
                            appData.srvAnswer = APP_SERVER_ERROR;
                            return;
                        }
                        
                        appData.fwUpdating = false;
                        appData.srvAnswer = APP_SERVER_OK;
                    }
                }
                else if(*(appData.i2cRX+6)==0x07 &&
                        *(appData.i2cRX+7)==0x06) // Firmware Update
                {
                    if(!appData.fwUpdating) //First Firmware packet
                    {
                        appData.fwUpdating = true;
                        appData.fwCRC = true;
                        MEM_InitObj();
                        MEM_Set_Event_Handler(APP_MEMEventHandler);
                        appData.PPPChecksumOut.Val=0;
                        appData.fwSizeCount.Val=0;
                        appData.CRC.Val=0;
                    }

                    if(!appData.memWriting)
                    {
                        appData.memWriting = true;
                        appData.fwLastOffset.v[1]=*(appData.i2cRX+8);
                        appData.fwLastOffset.v[0]=*(appData.i2cRX+9);
                        appData.fwPacketSize.v[1] = *(appData.i2cRX+10);
                        appData.fwPacketSize.v[0] = *(appData.i2cRX+11);
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
        }
    }
    else
    {
        if(appData.andrConnected)
        {
            appData.androidHandler = appData.genericRxHandler;
            if(!AND_Write(appData.readUSB,appData.readUSBSize))
            {
                freeI2CRxBuffIndex(appData.genericRxHandler); //Dropping packet in error case
                return;
            }
        }
        else
        {
            freeI2CRxBuffIndex(appData.genericRxHandler);
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
            appData.AVLDataReady++;
        }
        break;
        case I2C_EVENT_DATA_SENT:
        {
            //TODO: Implement a way to check which handler was sent
        }
        break;
        case I2C_EVENT_DATA_NOT_SENT:
        {
            //TODO: Implement a way to check which handler wasn't sent
        }
        break;
        default:
        {
        }
        break;
    }
}

void APP_AndrEventHandler(AND_EVENT event)
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
            appData.andrConnected = false;
        }
        break;
        case AND_EVENT_DATA_READY:
        {
            appData.andrDataReady=true;
        }
        break;
        case AND_EVENT_DATA_SENT:
        {
            freeI2CRxBuffIndex(appData.androidHandler);
        }
        break;
        case AND_EVENT_DATA_NOT_SENT:
        {
            freeI2CRxBuffIndex(appData.androidHandler);
        }
        break;
        default:
        {
        }
        break;
    }
}

void APP_MEMEventHandler(MEM_EVENT event)
{
    switch(event)
    {
        case MEM_EVENT_BUFFER_WRITTEN:
        {
            freeI2CRxBuffIndex(appData.memHandler);
            appData.memWriting = false;
            appData.srvAnswer = APP_SERVER_OK;
        }
        break;
        case MEM_EVENT_BUFFER_NOT_WRITTEN:
        {
            freeI2CRxBuffIndex(appData.memHandler);
            appData.memWriting = false;
            appData.srvAnswer = APP_SERVER_ERROR;
        }
        break;
        default:
        {
        }
        break;
    }
}

#endif
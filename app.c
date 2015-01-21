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

static char manufacturer[] = "Grupo Inlog";
static char model[] = "USB01";
static char description[] = "Inlog Android Accessory";
static char version[] = "1.0";
static char uri[] = "http://www.inlog.com.br/";
static char serial[] = "000";

static BYTE serialNumber[6]="000000";
static BYTE fwVersion[6]="1.0";

#ifdef AVLSimulation
static BYTE avl_packet[87] = {0x5B,0x0A,0x12,0x06,0x06,0x1B,0x0B,0x0E,0x01,0xC3,0x4E,0x7A,
                              0x00,0xE9,0x92,0xBE,0x00,0x05,0x09,0x1B,0x34,0x07,0x00,0x7F,
                              0x00,0x56,0x93,0xCE,0x07,0x1D,0x5D,0x00,0x00,0x00,0x00,0xDE,
                              0x6E,0x41,0x30,0x30,0x30,0x35,0x31,0x33,0x35,0x49,0x4E,0x54,
                              0x52,0x41,0x43,0x4B,0x3A,0xFF,0xFF,0x00,0x00,0x05,0xAF,0xCE,
                              0x93,0x56,0x00,0x00,0x00,0x00,0x00,0x00,0x85,0xFF,0xFF,0xFF,
                              0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x56,
                              0x93,0xCE,0x00};
#endif



#ifdef I2CTXTest
static BYTE i2c_test[8] = {0x74,0x0B,0x01,0x00,0x02,0x01,0x02,0x4A};
#endif



ANDROID_ACCESSORY_INFORMATION accessoryInfo =
{
    manufacturer,
    sizeof(manufacturer),
    model,
    sizeof(model),
    description,
    sizeof(description),
    version,
    sizeof(version),
    uri,
    sizeof(uri),
    serial,
    sizeof(serial)
};

//local variables
bool entry_flag;
BYTE macroConnect[2];

void APP_Initialize ( void )
{
#ifdef I2CTXTest
    appData.current_state.lvl0 =  APP_STATE0_I2CTest;
    appData.current_state.lvl1 =  APP_STATE1_I2CInit;
    appData.current_state.lvl2 =  APP_STATE2_None;
#elif defined(LoginIntoServerTest)
    appData.current_state.lvl0 =  APP_STATE0_LoggingIntoServer;
    appData.current_state.lvl1 =  APP_STATE1_SendingTunelLogin;
    appData.current_state.lvl2 =  APP_STATE2_None;
#else
    appData.current_state.lvl0 =  APP_STATE0_SettingUpUSBHost;
    appData.current_state.lvl1 =  APP_STATE1_OpenningHostLayer;
    appData.current_state.lvl2 =  APP_STATE2_None;
#endif
    
    appData.sysTmrHandle = 0;
    appData.hostHandle = 0;
    appData.accessoryMode_trials = 0;
    appData.AVLDataReady = false;
    appData.macroDataReady = false;
    appData.loggedIntoServer = false;
    appData.i2cBufferHandler = 254;

    entry_flag = true;
    appData.writeUSB=0;
    appData.writeUSBSize=0;

    appData.readUSB=0;
    appData.readUSBSize=0;

    appData.fwLastOffset.Val = 0;
    appData.fwSize.Val = 0;
    appData.fwUpdating = false;
    appData.fwCRC = 9;
    appData.lastPacket = false;
    appData.memWriting = false;

#ifdef I2CTXTest
    appData.i2c_test_sent = false;
    appData.waiting_answer = false;
    appData.requested=0;
    appData.sent=0;
    appData.received=0;
    appData.answered=0;
#endif

    AndroidAppStart_Pv1();
}

void changeAppState(APP_STATES_LVL0 a, APP_STATES_LVL1 b, APP_STATES_LVL2 c)
{
    appData.current_state.lvl0 = a;
    appData.current_state.lvl1 = b;
    appData.current_state.lvl2 = c;
}

void APP_Tasks ( void )
{
    switch (appData.current_state.lvl0)
    {
#ifdef I2CTXTest
        case APP_STATE0_I2CTest:
        {
            switch(appData.current_state.lvl1)
            {
                case APP_STATE1_I2CInit:
                {
                    I2C_Set_Event_Handler(APP_I2CEventHandler);
                    I2C_InitObject();
                    entry_flag = true;
                    changeAppState(APP_STATE0_I2CTest,APP_STATE1_I2CTesting,APP_STATE2_None);
                }
                break;

                case APP_STATE1_I2CTesting:
                {
                    int i;
                    BYTE answer[9];
                    if(entry_flag)
                    {
                        entry_flag = false;
                        appData.readUSB = allocI2CTXBuffer(&(appData.i2cBufferHandler),&(appData.readUSBSize));
                        if(appData.readUSB)
                        {
                            /*for (i=0;i<50;i++)
                            {
                                *(appData.readUSB + i) = i2c_test[i];
                            }*/
                            TunelLoginPacket(serialNumber,fwVersion,appData.readUSB,&(appData.readUSBSize));
                        }
                        appData.sysTmrHandle = SYS_TMR_DelayMS(1000);
                    }
                    if(SYS_TMR_DelayStatusGet(appData.sysTmrHandle))
                    {
                        if(appData.waiting_answer)
                        {
                            appData.sysTmrHandle = SYS_TMR_DelayMS(100);
                        }
                        else
                        {
                            entry_flag = true;
                            appData.i2c_test_sent = false;
                            I2CStartTX(appData.i2cBufferHandler,appData.readUSBSize,AVL_ADDR);
                            appData.waiting_answer = true;
                            appData.requested++;
                        }
                    }
                    if(appData.AVLDataReady>0)
                    {
                        appData.AVLDataReady--;
                        appData.writeUSB = getI2CRxBuffer(&(appData.writeUSBSize), &(appData.writeUSBHandler)); //Getting the I2C Message
                        freeI2CRxBuffIndex(appData.writeUSBHandler); //Freeing the I2C Buffer simulating it is immediatly transmitted
                        appData.received++;
                        if(appData.waiting_answer && appData.writeUSBSize==9)
                        {
                            for(i=0;i<9;i++)
                            {
                                answer[i] = *(appData.writeUSB+i);
                            }
                            appData.waiting_answer = false;
                            appData.answered++;
                        }
                    }
                }
                break;
            }
        }
        break;
#endif
        case APP_STATE0_SettingUpUSBHost:
        {
            switch (appData.current_state.lvl1)
            {
                case APP_STATE1_OpenningHostLayer:
                {
                    appData.hostHandle = USB_HOST_Open(USB_HOST_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
                    if (appData.hostHandle != USB_HOST_HANDLE_INVALID)
                    {
                        USB_HOST_OperationEnable(appData.hostHandle );
                        changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_WaitingHostEnable,
                                    APP_STATE2_None);
                    }
                }
                break;

                case APP_STATE1_WaitingHostEnable:
                {
                    if(USB_HOST_OperationIsEnabled(appData.hostHandle))
                    {
                        USB_HOST_EventCallBackSet(appData.hostHandle, APP_USBHostEventHandler, 0 );
                        USB_HOST_Android_EventHandlerSet(APP_USBHostAndroidEventHandler);
                        changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_WaitingDeviceAttach,
                                    APP_STATE2_None);
                    }
                }
                break;

                case APP_STATE1_WaitingDeviceAttach:
                {
                }
                break;

                case APP_STATE1_GettingProtocol:
                {
                    if(entry_flag)
                    {
                        appData.protocol = 0;
                        AndroidGetProtocol_Pv1(&(appData.protocol));
                        entry_flag = false;
                    }
                }
                break;

                case APP_STATE1_SendingAndroidInfo:
                {
                    switch (appData.current_state.lvl2)
                    {
                        case APP_STATE2_SendingManufacturer:
                        {
                            if(entry_flag)
                            {
                                AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_MANUFACTURER, accessoryInfo.manufacturer, accessoryInfo.manufacturer_size);
                                entry_flag = false;
                            }
                        }
                        break;
                        case APP_STATE2_SendingModel:
                        {
                            if(entry_flag)
                            {
                                AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_MODEL, accessoryInfo.model, accessoryInfo.model_size);
                                entry_flag = false;
                            }
                        }
                        break;
                        case APP_STATE2_SendingDescription:
                        {
                            if(entry_flag)
                            {
                                AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_DESCRIPTION, accessoryInfo.description, accessoryInfo.description_size);
                                entry_flag = false;
                            }
                        }
                        break;
                        case APP_STATE2_SendingVersion:
                        {
                            if(entry_flag)
                            {
                                AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_VERSION, accessoryInfo.version, accessoryInfo.version_size);
                                entry_flag = false;
                            }
                        }
                        break;
                        case APP_STATE2_SendingURI:
                        {
                            if(entry_flag)
                            {
                                AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_URI, accessoryInfo.URI, accessoryInfo.URI_size);
                                entry_flag = false;
                            }
                        }
                        break;
                        case APP_STATE2_SendingSerialNum:
                        {
                            if(entry_flag)
                            {
                                AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_SERIAL, accessoryInfo.serial, accessoryInfo.serial_size);
                                entry_flag = false;
                            }
                        }
                        break;
                        case APP_STATE2_None:
                        {
                            changeAppState(APP_STATE0_Error,APP_STATE1_None,APP_STATE2_None);
                        }
                        break;
                    }
                }
                break;

                case APP_STATE1_StartingAndroidAccessory:
                {
                    if(entry_flag)
                    {
                        AndroidCommandStart_Pv1();
                        entry_flag = false;
                    }
                }
                break;

                case APP_STATE1_WaitingAndroidDetach:
                {
                }
                break;
                case APP_STATE1_WaitingUsbReady:
                {
                    if(SYS_TMR_DelayStatusGet(appData.sysTmrHandle))
                    {
                        entry_flag = true;
                        changeAppState(APP_STATE0_WaitingMacroConnect,APP_STATE1_None,
                                    APP_STATE2_None);
                    }
                }
                break;

                default:
                {
                   changeAppState(APP_STATE0_Error,APP_STATE1_None,APP_STATE2_None);
                }
                break;
            }
        }
        break;

        case APP_STATE0_WaitingMacroConnect:
        {
            if(entry_flag)
            {
                AndroidAppRead_Pv1(macroConnect,sizeof(macroConnect));
                entry_flag = false;
            }
        }
        break;

#ifdef AVLSimulation
        case APP_STATE0_SimulatingAVL:
        {
            switch (appData.current_state.lvl1)
            {
                case APP_STATE1_SendingAVLStatus:
                {
                    if(entry_flag)
                    {
                        AndroidAppWrite_Pv1(avl_packet,sizeof(avl_packet));
                        entry_flag = false;
                    }
                }
                break;

                case APP_STATE1_Waiting_Timer:
                {
                    if(SYS_TMR_DelayStatusGet(appData.sysTmrHandle))
                    {
                        entry_flag = true;
                        changeAppState(APP_STATE0_SimulatingAVL,APP_STATE1_SendingAVLStatus,
                                    APP_STATE2_None);
                    }
                }
                break;
            }
            
        }
        break;
#endif

#ifdef LoginIntoServerTest
        case APP_STATE0_LoggingIntoServer:
        {
            switch(appData.current_state.lvl1)
            {
                case APP_STATE1_SendingTunelLogin:
                {
                    if(entry_flag)
                    {
                        entry_flag = false;
                        I2C_Set_Event_Handler(APP_I2CEventHandler);
                        I2C_InitObject();
                        appData.loginPacket = allocI2CTXBuffer(&(appData.loginBuffHandler),&(appData.loginPacketSize));
                        TunelLoginPacket(serialNumber,fwVersion,appData.loginPacket,&(appData.loginPacketSize));
                        I2CStartTX(appData.loginBuffHandler, appData.loginPacketSize,AVL_ADDR);
                        appData.sysTmrHandle = SYS_TMR_DelayMS(5000);
                        changeAppState(APP_STATE0_LoggingIntoServer,APP_STATE1_WaitingServerAnswer,APP_STATE2_None);
                    }
                }
                break;
                case APP_STATE1_WaitingServerAnswer:
                {
                    int test[9],i;
                    if(appData.AVLDataReady>0)
                    {
                        appData.writeUSB = getI2CRxBuffer(&(appData.writeUSBSize), &(appData.writeUSBHandler));
                        if(!appData.writeUSB)
                        {
                            Nop();
                        }


                        appData.AVLDataReady--;
                        if(appData.writeUSBSize<15)//The Server Answer is a small packet
                        {
                            for(i=0;i<9;i++)
                            {
                                test[i] = *(appData.writeUSB+i);
                            }
                            if(*(appData.writeUSB+2)==0x1B)
                            {
                                if(*(appData.writeUSB+6)==0x60 && *(appData.writeUSB+7)==0x01)
                                {
                                    changeAppState(APP_STATE0_LoggingIntoServer,APP_STATE1_LoginSuccessful,APP_STATE2_None);
                                }
                            }
                        }
                        freeI2CRxBuffIndex(appData.writeUSBHandler);
                    }
                    if(SYS_TMR_DelayStatusGet(appData.sysTmrHandle))
                    {
                        changeAppState(APP_STATE0_LoggingIntoServer,APP_STATE1_LoginUnSuccessful,APP_STATE2_None);
                    }
                }
                break;
                case APP_STATE1_LoginSuccessful:
                {
                    Nop();
                }
                break;
                case APP_STATE1_LoginUnSuccessful:
                {
                    Nop();
                }
                break;
            }
        }
        break;
#endif

        case APP_STATE0_TransferingData:
        {
            switch(appData.current_state.lvl1)
            {
                case APP_STATE1_WaitingForData:
                {
                    //WORD_VAL FwSize;
                    bool aux;
                    uint16_t size,i;
                    if(appData.i2cBufferHandler==254) //Last try I2C TX buffer was full
                    {
                        appData.readUSB = allocI2CTXBuffer(&(appData.i2cBufferHandler),&(appData.readUSBSize));
                        if(appData.i2cBufferHandler!=254) //TX buffer full
                        {
                            AndroidAppRead_Pv1(appData.readUSB,appData.readUSBSize); //Thats the maximum size
                        }
                    }
                    if(!appData.loggedIntoServer)
                    {
                        if(SYS_TMR_DelayStatusGet(appData.sysTmrHandle))
                        {
                            appData.loginPacket = allocI2CTXBuffer(&(appData.loginBuffHandler),&(appData.loginPacketSize));
                            TunelLoginPacket(serialNumber,fwVersion,appData.loginPacket,&(appData.loginPacketSize));
                            I2CStartTX(appData.loginBuffHandler, appData.loginPacketSize,AVL_ADDR);
                            appData.sysTmrHandle = SYS_TMR_DelayMS(4000);
                        }
                    }

                    if(appData.fwCRC!=9)
                    {
                        if(appData.fwCRC)
                        {
                            //TODO: Send Server fw upgrade OK answer
                            appData.fwCRC=9;
                        }
                        else
                        {
                            //TODO: Send Server fw upgrade CRC ERROR answer
                            appData.fwCRC=9;
                        }

                    }
                    
                    if(appData.AVLDataReady>0)
                    {
                        appData.AVLDataReady--;
                        appData.writeUSB = getI2CRxBuffer(&(appData.writeUSBSize), &(appData.genericRxHandler));

                        if(!appData.writeUSB)
                        {
                            changeAppState(APP_STATE0_Error,APP_STATE1_None,APP_STATE2_None);// Should never end up here
                        }

                        if(*(appData.writeUSB+2)==0x1B)
                        {
                            aux = true;
                            if(!appData.loggedIntoServer)
                            {
                                if(appData.writeUSBSize<15)//Server Answer
                                {
                                    if(*(appData.writeUSB+6)==0x60 && *(appData.writeUSB+7)==0x01)
                                    {
                                        appData.loggedIntoServer=true;
                                        aux=false;
                                        freeI2CRxBuffIndex(appData.genericRxHandler);
                                    }
                                }
                            }
                            if(aux)
                            {
                                if(*(appData.writeUSB+5)==0x6B &&
                                   *(appData.writeUSB+6)==0x07 &&
                                   *(appData.writeUSB+7)==0x06) // Firmware Update
                                {
                                    if(!appData.fwUpdating) //First Firmware packet
                                    {
                                        appData.fwUpdating = true;
                                        appData.fwCRC = true;
                                        appData.lastPacket = false;
                                        MEM_InitObj();
                                        MEM_Set_Event_Handler(APP_MEMEventHandler);
                                        appData.PPPChecksumOut.Val=0;
                                        appData.fwSizeCount.Val=0;
                                    }

                                    if(!appData.memWriting) //Memory Module not busy
                                    {
                                        appData.memWriting = true;
                                        
                                        appData.fwLastOffset.v[1]=*(appData.writeUSB+8);
                                        appData.fwLastOffset.v[0]=*(appData.writeUSB+9);
                                        appData.fwSize.v[1] = *(appData.writeUSB+10); //Size of the current part of the firmware
                                        appData.fwSize.v[0] = *(appData.writeUSB+11);
                                        size = appData.writeUSBSize-12;
                                        appData.fwSizeCount.Val += size;

                                        if(appData.fwSizeCount.Val >= appData.fwSize.Val) //Last Firmware packet
                                        {
                                            size = size-2;
                                            appData.fwSizeCount.Val -= 2;
                                            appData.lastPacket = true;
                                        }

                                        for(i=0;i<size;i++)
                                        {
                                            fwByteCRC(*(appData.writeUSB+12+i), &(appData.PPPChecksumOut));
                                        }

                                        if(appData.lastPacket)
                                        {
                                            appData.fwCRC = 1;
                                            if(appData.PPPChecksumOut.v[1] != *(appData.writeUSB+13+size))
                                            {
                                                appData.fwCRC = 0;
                                            }
                                            if(appData.PPPChecksumOut.v[0] != *(appData.writeUSB+14+size))
                                            {
                                                appData.fwCRC = 0;
                                            }
                                        }

                                        appData.memUSBHandler = appData.genericRxHandler;
                                        MEM_FillBuffer((appData.writeUSB+12), size);
                                    }
                                    else
                                    {
                                        freeI2CRxBuffIndex(appData.genericRxHandler);
                                        //TODO: Send "Nack" to Server.
                                    }
                                }
                            }
                        }
                        else
                        {
                            appData.writeUSBHandler = appData.genericRxHandler;
                            AndroidAppWrite_Pv1(appData.writeUSB,appData.writeUSBSize);
                        }
                    }
                    if(appData.macroDataReady)
                    {
                        if(appData.readUSB[2]<=2)
                        {
                            I2CStartTX(appData.i2cBufferHandler, appData.readUSBSize, AVL_ADDR);//i2cBufferHandler should not be 254 if the code falls here
                        }
                        else
                        {
                            I2CStartTX(appData.i2cBufferHandler, appData.readUSBSize, appData.readUSB[2]);//i2cBufferHandler should not be 254 if the code falls here
                        }

                        appData.readUSB = allocI2CTXBuffer(&(appData.i2cBufferHandler),&(appData.readUSBSize));
                        if(appData.i2cBufferHandler!=254) //TX buffer full
                        {
                            AndroidAppRead_Pv1(appData.readUSB,appData.readUSBSize);
                        }

                        appData.macroDataReady = false;
                    }
                }
                break;
                default:
                {
                    changeAppState(APP_STATE0_Error,APP_STATE1_None,APP_STATE2_None);
                }
                break;
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
    }
}

void APP_I2CEventHandler(I2C_EVENT event)
{
    switch(event)
    {
        case I2C_EVENT_AVL_DATA_READY:
        {
            appData.AVLDataReady++;
        }
        break;
        case I2C_EVENT_PACKET_SENT:
        {
#ifdef I2CTXTest
            appData.i2c_test_sent = true;
            appData.sent++;
#endif
        }
        break;
        case I2C_EVENT_PACKET_NOT_SENT:
        {
            Nop();
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
            freeI2CRxBuffIndex(appData.memUSBHandler);
            appData.memWriting = false;
            if(appData.lastPacket)
            {
                appData.fwUpdating = false;
            }
            //TODO: Send "Ack" to server
        }
        break;
        case MEM_EVENT_BUFFER_NOT_WRITTEN:
        {
            freeI2CRxBuffIndex(appData.memUSBHandler);
            appData.memWriting = false;
            //TODO: Send "Nack" to server
        }
        break;
        default:
        {
        }
        break;
    }
}

void APP_USBHostAndroidEventHandler(USB_HOST_ANDROID_EVENT event, uint8_t eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_HOST_ANDROID_EVENT_ATTACH:
        {
            if(eventData==0 || eventData==1)
            {
                appData.sysTmrHandle = SYS_TMR_DelayMS(20); //Waiting for USB to be ready to read and write.
                changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_WaitingUsbReady,
                            APP_STATE2_None);
            }
            else
            {
                entry_flag = true;
                changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_GettingProtocol,
                            APP_STATE2_None);
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_DETACH: //TODO: Create a state for deinit usb
        {
            if(appData.current_state.lvl1==APP_STATE1_WaitingAndroidDetach)
            {
                USB_HOST_OperationEnable(appData.hostHandle);
                changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_WaitingHostEnable,
                                APP_STATE2_None);
            }

            else
            {
                USB_HOST_OperationDisable(appData.hostHandle);
                //wait for DInit
                USB_HOST_Close(appData.hostHandle);
                changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_OpenningHostLayer,APP_STATE2_None);
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_PROTOCOL_COMPLETE:
        {
            if(appData.protocol != 0)
            {
                entry_flag = true;
                changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_SendingAndroidInfo,
                            APP_STATE2_SendingManufacturer);
            }
            else
            {
                changeAppState(APP_STATE0_Error,APP_STATE1_None,APP_STATE2_None);
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_SEND_STRING_COMPLETE:
        {
            if(appData.current_state.lvl1==APP_STATE1_SendingAndroidInfo)
            {
                entry_flag = true;
                switch(appData.current_state.lvl2)
                {
                    case APP_STATE2_SendingManufacturer:
                    {
                        changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_SendingAndroidInfo,
                                    APP_STATE2_SendingModel);
                    }
                    break;
                    case APP_STATE2_SendingModel:
                    {
                        changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_SendingAndroidInfo,
                                    APP_STATE2_SendingDescription);
                    }
                    break;
                    case APP_STATE2_SendingDescription:
                    {
                        changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_SendingAndroidInfo,
                                    APP_STATE2_SendingVersion);
                    }
                    break;
                    case APP_STATE2_SendingVersion:
                    {
                        changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_SendingAndroidInfo,
                                    APP_STATE2_SendingURI);
                    }
                    break;
                    case APP_STATE2_SendingURI:
                    {
                        changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_SendingAndroidInfo,
                                    APP_STATE2_SendingSerialNum);
                    }
                    break;
                    case APP_STATE2_SendingSerialNum:
                    {
                        changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_StartingAndroidAccessory,
                                    APP_STATE2_None);
                    }
                    break;

                    default:
                    {
                        changeAppState(APP_STATE0_Error,APP_STATE1_None,APP_STATE2_None);
                    }
                    break;
                }
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_START_COMPLETE:
        {
            if(appData.current_state.lvl1==APP_STATE1_StartingAndroidAccessory)
            {
                changeAppState(APP_STATE0_SettingUpUSBHost,APP_STATE1_WaitingAndroidDetach,
                            APP_STATE2_None);
                appData.accessoryMode_trials++;
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_WRITE_COMPLETE:
        {
#ifdef AVLSimulation
            if(appData.current_state.lvl1==APP_STATE1_SendingAVLStatus)
            {
                appData.sysTmrHandle = SYS_TMR_DelayMS(1000);
                changeAppState(APP_STATE0_SimulatingAVL,APP_STATE1_Waiting_Timer,
                            APP_STATE2_None);
            }
#else
            if(appData.current_state.lvl0==APP_STATE0_TransferingData &&
               appData.current_state.lvl1==APP_STATE1_WaitingForData)
            {
                freeI2CRxBuffIndex(appData.writeUSBHandler);
            }
#endif
        }

        break;

        case USB_HOST_ANDROID_EVENT_READ_COMPLETE:
        {
            if(appData.current_state.lvl0 == APP_STATE0_WaitingMacroConnect)
            {
                if(macroConnect[0]==0xFE)
                {
                    entry_flag=true;

#ifdef AVLSimulation
                    changeAppState(APP_STATE0_SimulatingAVL,APP_STATE1_SendingAVLStatus,
                                APP_STATE2_None);
#else
                    if(!I2C_Set_Event_Handler(APP_I2CEventHandler))
                    {
                        changeAppState(APP_STATE0_Error,APP_STATE1_None,
                                     APP_STATE2_None);
                    }
                    else
                    {
                        I2C_InitObject();
                        appData.readUSB = allocI2CTXBuffer(&(appData.i2cBufferHandler),&(appData.readUSBSize));
                        if(appData.i2cBufferHandler!=254) //TX buffer full
                        {
                            AndroidAppRead_Pv1(appData.readUSB,appData.readUSBSize);
                        }
                        changeAppState(APP_STATE0_TransferingData,APP_STATE1_WaitingForData, APP_STATE2_None);
                        appData.loggedIntoServer = false;
                        appData.sysTmrHandle = SYS_TMR_DelayMS(4000); //Login Timeout
                    }
#endif
                }
            }
            else if(appData.current_state.lvl0 == APP_STATE0_TransferingData)
            {
                appData.readUSBSize = eventData;
                appData.macroDataReady = true;
            }
        }
        break;

         default:
            break;
    }
    return USB_HOST_ANDROID_EVENT_RESPONSE_NONE;
}

USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENTS event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_HOST_EVENT_VBUS_REQUEST_POWER:
            break;
        case USB_HOST_EVENT_UNSUPPORTED_DEVICE:
            //TODO restart usb config
            break;
        case USB_HOST_EVENT_CANNOT_ENUMERATE:
            //TODO restart usb config
            break;
        case USB_HOST_EVENT_CONFIGURATION_COMPLETE:
            break;
        case USB_HOST_EVENT_CONFIGURATION_FAILED:
            //TODO restart usb config
            break;
        case USB_HOST_EVENT_DEVICE_SUSPENDED:
            break;
        case USB_HOST_EVENT_DEVICE_RESUMED:
            break;
    }
    return USB_HOST_EVENT_RESPONSE_NONE;
}

#endif
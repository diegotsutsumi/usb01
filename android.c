/*
 * File:   usb.c
 * Author: diego.tsutsumi
 *
 * Created on 22th January 2015, 09:54
 */

#include "android.h"

static char manufacturer[] = "Grupo Inlog";
static char model[] = "USB01";
static char description[] = "Inlog Android Accessory";
static char version[] = "1.0";
static char uri[] = "http://www.inlog.com.br/";
static char serial[] = "000";


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

AND_Object andr_obj;

void AND_ChangeState(AND_STATES_LVL0 _lvl0, AND_STATES_LVL1 _lvl1)
{
    andr_obj.current_state.lvl0 = _lvl0;
    andr_obj.current_state.lvl1 = _lvl1;
}

void AND_SetEventHandler(AND_EVENT_HANDLER handler)
{
    if(handler!=0)
    {
        andr_obj.event_handler = handler;
    }
}

void AND_Init()
{
    AND_ChangeState(AND_STATE0_Uninitialized,AND_STATE1_None);
}

void AND_InitObject()
{
    AND_ChangeState(AND_STATE0_OpeningHostLayer,AND_STATE1_None);
    andr_obj.event_handler=0;
    andr_obj.sysTmrHandle=0;
    andr_obj.hostHandle=0;

    andr_obj.readUSB=0;
    andr_obj.readUSBSize=0;
    andr_obj.writeUSB=0;
    andr_obj.writeUSBSize=0;

    andr_obj.protocol=0;
    andr_obj.accessoryMode_trials=0;
    andr_obj.entry_flag=true;
    andr_obj.macroConnect[0]=0;
    andr_obj.macroConnect[1]=0;

    AndroidAppStart_Pv1();
}

void AND_DeinitObject()
{
    AND_ChangeState(AND_STATE0_Uninitialized,AND_STATE1_None);
    andr_obj.event_handler=0;
    andr_obj.sysTmrHandle=0;
    andr_obj.hostHandle=0;

    andr_obj.readUSB=0;
    andr_obj.readUSBSize=0;
    andr_obj.writeUSB=0;
    andr_obj.writeUSBSize=0;

    andr_obj.protocol=0;
    andr_obj.accessoryMode_trials=0;
    andr_obj.entry_flag=false;
    andr_obj.macroConnect[0]=0;
    andr_obj.macroConnect[1]=0;

    AndroidAppStart_Pv1();
}

bool AND_Read(BYTE* buff, size_t size)
{
    if(andr_obj.current_state.lvl0 == AND_STATE0_TransferingData)
    {
        if(AndroidAppRead_Pv1(buff, size)!=USB_ANDROID_RESULT_OK)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

bool AND_Write(BYTE* buff, size_t size)
{
    if(andr_obj.current_state.lvl0 == AND_STATE0_TransferingData)
    {
        if(AndroidAppWrite_Pv1(buff, size)!=USB_ANDROID_RESULT_OK)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

void AND_Tasks()
{
    switch (andr_obj.current_state.lvl0)
    {
        case AND_STATE0_Uninitialized:
        {
        }
        break;
        
        case AND_STATE0_OpeningHostLayer:
        {
            andr_obj.hostHandle = USB_HOST_Open(USB_HOST_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
            if (andr_obj.hostHandle != USB_HOST_HANDLE_INVALID)
            {
                USB_HOST_OperationEnable(andr_obj.hostHandle );
                AND_ChangeState(AND_STATE0_WaitingHostEnable, AND_STATE1_None);
            }
        }
        break;

        case AND_STATE0_WaitingHostEnable:
        {
            if(USB_HOST_OperationIsEnabled(andr_obj.hostHandle))
            {
                USB_HOST_EventCallBackSet(andr_obj.hostHandle, USB_HostEventHandler, 0 );
                USB_HOST_Android_EventHandlerSet(AND_AndroidEventHandler);
                AND_ChangeState(AND_STATE0_WaitingDeviceAttach, AND_STATE1_None);
            }
        }
        break;

        case AND_STATE0_WaitingDeviceAttach:
        {
        }
        break;

        case AND_STATE0_GettingProtocol:
        {
            if(andr_obj.entry_flag)
            {
                andr_obj.protocol = 0;
                AndroidGetProtocol_Pv1(&(andr_obj.protocol));
                andr_obj.entry_flag = false;
            }
        }
        break;

        case AND_STATE0_SendingAndroidInfo:
        {
            switch (andr_obj.current_state.lvl1)
            {
                case AND_STATE1_SendingManufacturer:
                {
                    if(andr_obj.entry_flag)
                    {
                        AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_MANUFACTURER, accessoryInfo.manufacturer, accessoryInfo.manufacturer_size);
                        andr_obj.entry_flag = false;
                    }
                }
                break;

                case AND_STATE1_SendingModel:
                {
                    if(andr_obj.entry_flag)
                    {
                        AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_MODEL, accessoryInfo.model, accessoryInfo.model_size);
                        andr_obj.entry_flag = false;
                    }
                }
                break;

                case AND_STATE1_SendingDescription:
                {
                    if(andr_obj.entry_flag)
                    {
                        AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_DESCRIPTION, accessoryInfo.description, accessoryInfo.description_size);
                        andr_obj.entry_flag = false;
                    }
                }
                break;

                case AND_STATE1_SendingVersion:
                {
                    if(andr_obj.entry_flag)
                    {
                        AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_VERSION, accessoryInfo.version, accessoryInfo.version_size);
                        andr_obj.entry_flag = false;
                    }
                }
                break;

                case AND_STATE1_SendingURI:
                {
                    if(andr_obj.entry_flag)
                    {
                        AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_URI, accessoryInfo.URI, accessoryInfo.URI_size);
                        andr_obj.entry_flag = false;
                    }
                }
                break;

                case AND_STATE1_SendingSerialNum:
                {
                    if(andr_obj.entry_flag)
                    {
                        AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRING_SERIAL, accessoryInfo.serial, accessoryInfo.serial_size);
                        andr_obj.entry_flag = false;
                    }
                }
                break;

                case AND_STATE1_None:
                {
                    AND_ChangeState(AND_STATE0_Error,AND_STATE1_None);
                }
                break;
            }
        }
        break;

        case AND_STATE0_StartingAndroidAccessory:
        {
            if(andr_obj.entry_flag)
            {
                AndroidCommandStart_Pv1();
                andr_obj.entry_flag = false;
            }
        }
        break;

        case AND_STATE0_WaitingAndroidDetach:
        {
        }
        break;

        case AND_STATE0_WaitingUsbReady:
        {
            if(SYS_TMR_DelayStatusGet(andr_obj.sysTmrHandle))
            {
                andr_obj.entry_flag = true;
                AND_ChangeState(AND_STATE0_WaitingMacroConnect, AND_STATE1_None);
            }
        }
        break;

        case AND_STATE0_WaitingMacroConnect:
        {
            if(andr_obj.entry_flag)
            {
                AndroidAppRead_Pv1(andr_obj.macroConnect,sizeof(andr_obj.macroConnect));
                andr_obj.entry_flag = false;
            }
        }
        break;

        case AND_STATE0_TransferingData:
        {

        }
        break;

        case AND_STATE0_Error:
        {
            if(andr_obj.entry_flag)
            {
                andr_obj.event_handler(AND_EVENT_ERROR, NULL);
                andr_obj.entry_flag = false;
            }
        }
        break;
        
        default:
        {
            AND_ChangeState(AND_STATE0_Error,AND_STATE1_None);
            andr_obj.entry_flag = true;
        }
        break;
    }
}


void AND_AndroidEventHandler(USB_HOST_ANDROID_EVENT event, uint32_t eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_HOST_ANDROID_EVENT_ATTACH:
        {
            if(eventData==0 || eventData==1)
            {
                andr_obj.sysTmrHandle = SYS_TMR_DelayMS(100); //Waiting for USB to be ready to read and write.
                AND_ChangeState(AND_STATE0_WaitingUsbReady, AND_STATE1_None);
                PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_C, BSP_LED1);
            }
            else
            {
                andr_obj.entry_flag = true;
                AND_ChangeState(AND_STATE0_GettingProtocol, AND_STATE1_None);
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_DETACH:
        {
            andr_obj.event_handler(AND_EVENT_DISCONNECTED, NULL);
            PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_C, BSP_LED1);

            //USB_HOST_OperationEnable(andr_obj.hostHandle);
            AND_ChangeState(AND_STATE0_WaitingDeviceAttach, AND_STATE1_None);
        }
        break;

        case USB_HOST_ANDROID_EVENT_PROTOCOL_COMPLETE:
        {
            if(andr_obj.protocol != 0)
            {
                andr_obj.entry_flag = true;
                AND_ChangeState(AND_STATE0_SendingAndroidInfo, AND_STATE1_SendingManufacturer);
            }
            else
            {
                AND_ChangeState(AND_STATE0_Error,AND_STATE1_None);
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_SEND_STRING_COMPLETE:
        {
            if(andr_obj.current_state.lvl0==AND_STATE0_SendingAndroidInfo)
            {
                andr_obj.entry_flag = true;
                switch(andr_obj.current_state.lvl1)
                {
                    case AND_STATE1_SendingManufacturer:
                    {
                        AND_ChangeState(AND_STATE0_SendingAndroidInfo, AND_STATE1_SendingModel);
                    }
                    break;
                    case AND_STATE1_SendingModel:
                    {
                        AND_ChangeState(AND_STATE0_SendingAndroidInfo, AND_STATE1_SendingDescription);
                    }
                    break;
                    case AND_STATE1_SendingDescription:
                    {
                        AND_ChangeState(AND_STATE0_SendingAndroidInfo, AND_STATE1_SendingVersion);
                    }
                    break;
                    case AND_STATE1_SendingVersion:
                    {
                        AND_ChangeState(AND_STATE0_SendingAndroidInfo, AND_STATE1_SendingURI);
                    }
                    break;
                    case AND_STATE1_SendingURI:
                    {
                        AND_ChangeState(AND_STATE0_SendingAndroidInfo, AND_STATE1_SendingSerialNum);
                    }
                    break;
                    case AND_STATE1_SendingSerialNum:
                    {
                        AND_ChangeState(AND_STATE0_StartingAndroidAccessory, AND_STATE1_None);
                    }
                    break;

                    default:
                    {
                        AND_ChangeState(AND_STATE0_Error,AND_STATE1_None);
                    }
                    break;
                }
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_START_COMPLETE:
        {
            if(andr_obj.current_state.lvl0==AND_STATE0_StartingAndroidAccessory)
            {
                AND_ChangeState(AND_STATE0_WaitingAndroidDetach, AND_STATE1_None);
                andr_obj.accessoryMode_trials++;
            }
        }
        break;

        case USB_HOST_ANDROID_EVENT_WRITE_COMPLETE:
        {
            if(andr_obj.current_state.lvl0==AND_STATE0_TransferingData)
            {
                andr_obj.event_handler(AND_EVENT_DATA_SENT, NULL);
            }
        }

        break;

        case USB_HOST_ANDROID_EVENT_READ_COMPLETE:
        {
            if(andr_obj.current_state.lvl0 == AND_STATE0_WaitingMacroConnect)
            {
                if(andr_obj.macroConnect[0]==0xFE)
                {
                    andr_obj.entry_flag=true;
                    andr_obj.event_handler(AND_EVENT_CONNECTED, NULL);
                    AND_ChangeState(AND_STATE0_TransferingData,AND_STATE1_None);
                }
            }
            else if(andr_obj.current_state.lvl0 == AND_STATE0_TransferingData)
            {
                andr_obj.readUSBSize = eventData;
                andr_obj.event_handler(AND_EVENT_DATA_READY,eventData);
            }
        }
        break;

         default:
            break;
    }
    return USB_HOST_ANDROID_EVENT_RESPONSE_NONE;
}

USB_HOST_EVENT_RESPONSE USB_HostEventHandler (USB_HOST_EVENTS event, void * eventData, uintptr_t context)
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

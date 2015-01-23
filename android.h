/*
 * File:   android.c
 * Author: diego.tsutsumi
 *
 * Created on 22th January 2015, 09:54
 */

#ifndef _ANDROID_H_
#define	_ANDROID_H_

#include "system_config.h"
#include "system_definitions.h"
#include "usb_host_android_protocol_v1_local.h"

typedef enum
{
    AND_EVENT_CONNECTED,
    AND_EVENT_DISCONNECTED,
    AND_EVENT_DATA_READY,
    AND_EVENT_DATA_SENT,
    AND_EVENT_DATA_NOT_SENT,
    AND_EVENT_ERROR
}AND_EVENT;

typedef enum
{
    AND_STATE0_Uninitialized,
    AND_STATE0_OpeningHostLayer,
    AND_STATE0_WaitingHostEnable,
    AND_STATE0_WaitingDeviceAttach,
    AND_STATE0_GettingProtocol,
    AND_STATE0_SendingAndroidInfo,
    AND_STATE0_StartingAndroidAccessory,
    AND_STATE0_WaitingAndroidDetach,
    AND_STATE0_WaitingUsbReady,
    AND_STATE0_WaitingMacroConnect,
    AND_STATE0_TransferingData,
    AND_STATE0_WaitingHostDisable,
    AND_STATE0_Error
}AND_STATES_LVL0;

typedef enum
{
    AND_STATE1_SendingManufacturer,
    AND_STATE1_SendingModel,
    AND_STATE1_SendingDescription,
    AND_STATE1_SendingVersion,
    AND_STATE1_SendingURI,
    AND_STATE1_SendingSerialNum,
    AND_STATE1_None
}AND_STATES_LVL1;

typedef struct
{
    AND_STATES_LVL0 lvl0;
    AND_STATES_LVL1 lvl1;
}AND_State;

typedef void (*AND_EVENT_HANDLER)(AND_EVENT event);

typedef struct
{
    AND_State current_state;
    AND_EVENT_HANDLER event_handler;
    SYS_TMR_HANDLE sysTmrHandle;
    USB_HOST_HANDLE hostHandle;

    uint16_t  readUSBSize;
    uint8_t * readUSB;
    uint16_t  writeUSBSize;
    uint8_t * writeUSB;

    uint32_t protocol;
    uint8_t accessoryMode_trials;

    bool entry_flag;

    uint8_t macroConnect[2];
    
}AND_Object;

void AND_ChangeState(AND_STATES_LVL0 _lvl0, AND_STATES_LVL1 _lvl1);
void AND_SetEventHandler(AND_EVENT_HANDLER handler);
void AND_Init();
void AND_InitObject();
void AND_DeinitObject();
bool AND_Read(BYTE* buff, size_t size);
bool AND_Write(BYTE* buff, size_t size);
void AND_Tasks();
void AND_AndroidEventHandler(USB_HOST_ANDROID_EVENT event, uint8_t eventData, uintptr_t context);
USB_HOST_EVENT_RESPONSE USB_HostEventHandler (USB_HOST_EVENTS event, void * eventData, uintptr_t context);

#endif
/*
 * File:   app.h
 * Author: diego.tsutsumi
 *
 * Created on 10th November 2014, 10:12
 */

#ifndef _APP_H_
#define _APP_H_

#include "includes.h"
#include "system_config.h"
#include "system_definitions.h"
#include "usb_host_android_protocol_v1_local.h"
#include "i2c.h"
#include "tunel.h"
#include "memory.h"

//#define AVLSimulation // Never define
//#define I2CTXTest     // both together
//#define LoginIntoServerTest

typedef enum
{
    APP_STATE0_SettingUpUSBHost,

#ifdef AVLSimulation
    APP_STATE0_SimulatingAVL,
#endif

#ifdef I2CTXTest
    APP_STATE0_I2CTest,
#endif

    APP_STATE0_WaitingMacroConnect,

#ifdef LoginIntoServerTest
    APP_STATE0_LoggingIntoServer,
#endif

    APP_STATE0_TransferingData,
    APP_STATE0_Error
} APP_STATES_LVL0;

typedef enum
{
    //States inside APP_STATE0_SettingUpUSBHost
    APP_STATE1_OpenningHostLayer,
    APP_STATE1_WaitingHostEnable,
    APP_STATE1_WaitingDeviceAttach,
    APP_STATE1_GettingProtocol,
    APP_STATE1_SendingAndroidInfo,
    APP_STATE1_StartingAndroidAccessory,
    APP_STATE1_WaitingAndroidDetach,
    APP_STATE1_WaitingUsbReady,

    APP_STATE1_WaitingForData,

#ifdef LoginIntoServerTest
    APP_STATE1_SendingTunelLogin,
    APP_STATE1_WaitingServerAnswer,
    APP_STATE1_LoginSuccessful,
    APP_STATE1_LoginUnSuccessful,
#endif
            
#ifdef AVLSimulation
    APP_STATE1_Waiting_Timer,
    APP_STATE1_SendingAVLStatus,
#endif

#ifdef I2CTXTest
    APP_STATE1_I2CInit,
    APP_STATE1_I2CTesting,
#endif

    APP_STATE1_None
} APP_STATES_LVL1;

typedef enum
{
    //States inside APP_STATE1_SendingAndroidInfo
    APP_STATE2_SendingManufacturer,
    APP_STATE2_SendingModel,
    APP_STATE2_SendingDescription,
    APP_STATE2_SendingVersion,
    APP_STATE2_SendingURI,
    APP_STATE2_SendingSerialNum,

    APP_STATE2_None
} APP_STATES_LVL2;

typedef struct
{
    APP_STATES_LVL0 lvl0;
    APP_STATES_LVL1 lvl1;
    APP_STATES_LVL2 lvl2;
}APP_STATE;

typedef struct
{
    APP_STATE current_state;
    USB_HOST_HANDLE hostHandle;
    SYS_TMR_HANDLE sysTmrHandle;
    uint32_t protocol;
    uint8_t accessoryMode_trials;
    volatile bool loggedIntoServer;

    uint16_t loginPacketSize;
    BYTE * loginPacket;
    uint8_t loginBuffHandler;

    uint16_t  writeUSBSize;
    BYTE * writeUSB;
    uint8_t genericRxHandler;
    uint8_t memUSBHandler;
    uint8_t writeUSBHandler;

    uint16_t  readUSBSize;
    uint8_t * readUSB;
    uint8_t i2cBufferHandler;
    
    volatile uint8_t AVLDataReady;
    volatile bool macroDataReady;


    WORD_VAL fwSizeCount;
    WORD_VAL fwSize;
    WORD_VAL fwLastOffset;
    WORD_VAL PPPChecksumOut;
    bool fwUpdating;
    bool memWriting;
    bool lastPacket;
    uint8_t fwCRC;


    bool sendingToAVL;

#ifdef I2CTXTest
    bool i2c_test_sent;
    bool waiting_answer;
    uint32_t requested;
    uint32_t sent;
    uint32_t received;
    uint32_t answered;
#endif

} APP_DATA;

void APP_Initialize ( void );
void APP_Tasks ( void );
void APP_I2CEventHandler(I2C_EVENT event);
void APP_MEMEventHandler(MEM_EVENT event);
void changeAppState(APP_STATES_LVL0 a, APP_STATES_LVL1 b, APP_STATES_LVL2 c);
void APP_USBHostAndroidEventHandler(USB_HOST_ANDROID_EVENT event, uint8_t eventData, uintptr_t context);
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler(USB_HOST_EVENTS event, void * eventData, uintptr_t context);

#endif
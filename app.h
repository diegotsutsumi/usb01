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

#include "i2c.h"
#include "tunel.h"
#include "memory.h"
#include "android.h"

#ifdef NoDebug
#include "bootloader.h"
#endif

#define MemTest

typedef enum
{
    APP_STATE0_Initializing,
    APP_STATE0_Running,

#ifdef MemTest
    APP_STATE0_MemoryTesting,
    APP_STATE0_MemoryOk,
    APP_STATE0_MemoryError,
#endif

    APP_STATE0_Error
} APP_STATE;

typedef enum
{
    APP_SERVER_ERROR=0,
    APP_SERVER_OK,
    APP_SERVER_NOANSWER
} APP_SERVER_ANSWER;

typedef struct
{
    APP_STATE current_state;
    SYS_TMR_HANDLE sysTmrHandle;

    uint16_t serverPacketSize;
    BYTE * serverPacket;
    uint8_t serverBuffHandler;

    uint16_t loginPacketSize;
    BYTE * loginPacket;
    uint8_t loginBuffHandler;

    uint16_t  i2cRxSize;
    BYTE * i2cRX;
    uint8_t genericRxHandler;
    uint8_t androidHandler;
    uint8_t memHandler;
    
    uint16_t  readUSBSize;
    uint8_t * readUSB;
    uint8_t i2cTxHandler;

    volatile uint8_t AVLDataReady;
    volatile bool andrDataReady;
    volatile bool loggedIntoServer;
    volatile bool andrConnected;

    bool performFlashBoot;

    WORD_VAL fwSizeCount;
    WORD_VAL fwPacketSize;
    WORD_VAL fwLastOffset;
    WORD_VAL PPPChecksumOut;
    WORD_VAL CRC;
    bool fwUpdating;
    bool memWriting;
    uint8_t fwCRC;

    APP_SERVER_ANSWER srvAnswer;

    volatile uint32_t txUSB;
    volatile uint32_t rxUSB;
    volatile uint32_t txI2C;
    volatile uint32_t rxI2C;
    volatile uint32_t txErrorI2C;
    volatile uint32_t txErrorUSB;

#ifdef MemTest
    uint8_t checkMem;
    uint8_t checkBuffer[600];
#endif

} APP_DATA;

void APP_Initialize ( void );
void APP_Tasks ( void );
void APP_ProcessAVLPacket( void );
void APP_AndrEventHandler(AND_EVENT event, uint32_t eventData);
void APP_I2CEventHandler(I2C_EVENT event);
void APP_MEMEventHandler(MEM_EVENT event);

#endif
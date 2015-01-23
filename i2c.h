/*
 * File:   i2c.h
 * Author: diego.tsutsumi
 *
 * Created on 3th December 2014, 10:35
 */

#ifndef _I2C_H_
#define	_I2C_H_

#include "includes.h"
#include "peripheral/i2c/plib_i2c.h"
#include "system_definitions.h"

#define BUFFER_SIZE 270
#define BUFFER_RX_NUMBER 5
#define BUFFER_TX_NUMBER 10
#define TX_MAX_TRIES 10

#define TABLET_ADDR      0x0B
#define MY_ADDR          0x1B
#define SERVER_ADDR      0x01
#define AVL_ADDR         0x00


typedef enum
{
    I2C_EVENT_DATA_READY,
    I2C_EVENT_DATA_SENT,
    I2C_EVENT_DATA_NOT_SENT,
    I2C_EVENT_NONE
} I2C_EVENT;

typedef void (*I2C_EVENT_HANDLER)(I2C_EVENT event);

typedef enum
{
    I2C_STATE0_Uninitialized,
    I2C_STATE0_SettingUpSlave,
    I2C_STATE0_ListeningLine,
    I2C_STATE0_SettingUpMaster,
    I2C_STATE0_Sending,
    I2C_STATE0_Error
}I2C_STATES_LVL0;

typedef enum
{
    I2C_STATE1_WaitingStart,
    I2C_STATE1_WaitingAddress,
    I2C_STATE1_ReceivingByte,


    I2C_STATE1_Starting,
    I2C_STATE1_Addressing,
    I2C_STATE1_Writting,
    I2C_STATE1_Stopping,
    I2C_STATE1_WaitingAnswer,
    I2C_STATE1_None
}I2C_STATES_LVL1;

typedef struct
{
    I2C_STATES_LVL0 lvl0;
    I2C_STATES_LVL1 lvl1;
}I2C_STATE;

typedef enum
{
    I2C_MASTER_MODE,
    I2C_SLAVE_MODE,
    I2C_NO_MODE
}I2C_MODES;

typedef enum
{
    I2C_RX_BUFFER_FREE,
    I2C_RX_BUFFER_READY,
    I2C_RX_BUFFER_I2C_USING,
    I2C_RX_BUFFER_APP_USING
}I2C_RX_BUFFER_STATUS;

typedef enum
{
    I2C_TX_BUFFER_FREE,
    I2C_TX_BUFFER_READY_TO_TRANSMIT,
    I2C_TX_BUFFER_I2C_USING,
    I2C_TX_BUFFER_APP_USING
}I2C_TX_BUFFER_STATUS;

typedef enum
{
    I2C_RETURN_FAIL=-1,
    I2C_RETURN_SUCCESS
}I2C_RETURN;


typedef struct
{
    //RX Buffer management variables
    uint16_t I2CRxBuffSize[BUFFER_RX_NUMBER]; //RX buffer size
    BYTE I2CRxBuff[BUFFER_RX_NUMBER][BUFFER_SIZE]; //RX buffer
    I2C_RX_BUFFER_STATUS rxStatus[BUFFER_RX_NUMBER];
    uint8_t rxNumPackets;
    uint8_t rx_alloc_idx;

    //TX Buffer management variables
    uint16_t I2CTxBuffSize[BUFFER_TX_NUMBER]; //RX buffer size
    BYTE I2CTxBuff[BUFFER_TX_NUMBER][BUFFER_SIZE];
    BYTE TXAddress[BUFFER_TX_NUMBER][2];
    I2C_TX_BUFFER_STATUS txStatus[BUFFER_TX_NUMBER];
    volatile uint8_t tx_num_tries;
    uint8_t tx_count;
    uint8_t txNumPackets;
    uint8_t tx_alloc_idx;
    volatile bool tx_ok;
    volatile uint8_t to_transmit;

    uint8_t addrIndex;
    uint8_t nackCounter;

    I2C_STATE current_state;
    I2C_MODES current_mode;
    I2C_EVENT_HANDLER event_handler;

    volatile bool waiting_start_ok;
    volatile bool waiting_stop_ok;
    volatile bool waiting_byte_ok;
    volatile bool slave_data_interrupt;
    volatile bool slave_address_interrupt;
    bool entry_flag;

} I2C_Object;


void I2C_Init(uint32_t baudRate, uint32_t clockFrequency);
void I2C_InitMaster();
void I2C_DeInitMaster();
void I2C_InitSlave();
void I2C_DeInitSlave();
void I2C_Set_Event_Handler(I2C_EVENT_HANDLER handler);
char I2C_SlaveRead();
void I2C_InitObject();
void I2C_DeinitObject();

BYTE * getI2CRxBuffer(uint16_t * size, uint8_t * handler);
I2C_RETURN freeI2CRxBuffIndex(uint8_t handler);
uint8_t allocI2CRxBuffIndex();
I2C_RETURN I2CFinishedReceiving();

BYTE * allocI2CTXBuffer(uint8_t * out_index, uint16_t *out_maxSize);
I2C_RETURN I2CStartTX(uint8_t index, uint16_t size, uint8_t txaddress);
uint8_t getI2CTxBufferIndex();
I2C_RETURN freeI2CTXBuffer();


void I2C_Tasks();
void I2C_Tasks_ISR();
void changeI2CState(I2C_STATES_LVL0 a, I2C_STATES_LVL1 b);

#endif
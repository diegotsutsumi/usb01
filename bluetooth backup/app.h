/*
 * File:   app.h
 * Author: diego.tsutsumi
 *
 * Created on 22 de Outubro de 2014, 10:12
 */

#ifndef _APP_HEADER_H
#define _APP_HEADER_H


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_definitions.h"
#include "system_config.h"
#include "system/clk/sys_clk.h"
#include "peripheral/i2c/plib_i2c.h"
#include "bsp_config.h"

extern SYS_MODULE_OBJ    usartModule;

//I2C application
#define I2C_MODULE_MASTER           I2C_ID_1
#define I2C_MODULE_SLAVE            I2C_ID_2

#define BAUDRATE_100KHZ 100000
//#define slaveAddress 0x1A
#define AVLSlaveAddress 0xA0

#define address7bit_mask_none   0x00
#define address7bit_mask_all    0xFF

#define APP_MAX_BUFFER_SIZE         50
#define APP_UART_BAUDRATE       115200

typedef enum
{
    APP_BT_WakingUp = 1, // It holds in this state until a peer answers
    APP_BT_SettingUp = 2,
    APP_None_LV2 = 3

} APP_STATES_LV2;

typedef enum
{
    APP_Openning_UART = 1,
    APP_Configuring_Bluetooth = 2,
    APP_Waiting_Peer_Connection = 3,
    APP_BT_Advertising = 4,
    APP_BT_Setting_MLDP = 5,
    APP_Starting_I2C = 6,
    APP_Writting_I2C = 7,
    APP_Stopping_I2C = 8,
    APP_None_LV1 = 9
} APP_STATES_LV1;

typedef enum
{
    APP_SettingUp_Bluetooth = 1,
    APP_SettingUp_I2C = 2,
    APP_Waiting_BT_Connection = 3,
    APP_Waiting_for_Data = 4,
    APP_Sending_to_I2C = 5,
    APP_Sending_to_Bluetooth = 6,
    APP_Error = 7
} APP_STATES_LV0;

typedef struct
{
    APP_STATES_LV0 lvl_0;
    APP_STATES_LV1 lvl_1;
    APP_STATES_LV2 lvl_2;
} APP_STATE;

typedef struct
{
    APP_STATE currentState;
    char * buffer;
    uint32_t bufferSize;
} APP_DATA;

typedef struct
{
    volatile bool receiving_i2c;
    volatile bool waiting_usart_answer;
    volatile bool waiting_bt_data;
    volatile bool usart_write_completed;
    volatile bool bt_rebooting;
    volatile bool bt_waiting_connection;
    char * usart_right_answer;
} APP_CONTROL;

void strCat(char * str1, char * str2);
void APP_UARTTX_Completed();
void APP_UARTError();
void BTConnectUpdateStr();
void writeUSART(char * data, size_t size);
void writeUSARTBuffer();
bool writeByteI2C(char * data);
void changeState(APP_STATES_LV0 zero, APP_STATES_LV1 one, APP_STATES_LV2 two);


void APP_Initialize ( void );


void APP_Tasks ( void );


#endif /* _APP_HEADER_H */
/*******************************************************************************
 End of File
*/


/*
 * File:   app.c
 * Author: diego.tsutsumi
 *
 * Created on 22 de Outubro de 2014, 10:12
 */

#include "app.h"
#include "i2c.h"

APP_DATA appData;
APP_CONTROL appControl;

extern SYSTEM_OBJECTS sysObj;

//Bluetooth Commands
//char * BTInit[5] = {"SF,1\r\n","SS,C0000001\r\n","SR,92000800\r\n","S-,UBT01\r\n","R,1\r\n"}; // Master
char * BTInit[8] = {
                    "SF,1\r\n",
                    "SS,F0000001\r\n",
                    /*"SS,00000001\r\n",*/
                    "SR,22000000\r\n",
                    "S-,UBT01\r\n",
                    "PZ\r\n",
                    "PS,0000110100001000800000805F9B34FB\r\n",
                    "PC,0000110100001000800000805F9B34FB,1F,05\r\n",
                    "R,1\r\n"
                    }; // Slave
char * BTConnect[2] = {"X\r\n",""};
char * BTReboot = "R,1\r\n";
char * BTStartScan = "F\r\n";
char * BTStartAdv = "A\r\n";
char * BTStopScan = "X\r\n";
char * BTBond = "B,0\r\n";
char * BTConnectStr = "E,0,";
char * BTMac = "00035B0358E6\r\n"; // Initializing with a "Random" Value

void strCat(char * str1, char * str2)
{
    int i=0;
    int j=0;
    while(*(str1+i)!=0)
    {
        i++;
    }

    while(*(str2+j)!=0)
    {
        *(str1+i+j) = *(str2+j);
        j++;
    }
    *(str1+i+j) = 0;
}

void BTConnectUpdateStr()
{
    strCat(BTConnectStr,BTMac);
    BTConnect[1] = BTConnectStr;
}

void APP_Initialize( void )
{
    appData.currentState.lvl_0      = APP_SettingUp_Bluetooth;
    appData.currentState.lvl_1      = APP_Openning_UART;
    appData.currentState.lvl_2      = APP_None_LV2;
    appData.bufferSize        = 0;
    appData.buffer = malloc(APP_MAX_BUFFER_SIZE);
    
    appControl.receiving_i2c = false;
    appControl.waiting_usart_answer = false;
    appControl.usart_write_completed = true;
    appControl.bt_rebooting = false;
    appControl.bt_waiting_connection = false;
    
    appControl.usart_right_answer="";
    return;
}

void APP_UARTTX_Completed()
{
    //TODO: put into appData.buffer
    appControl.usart_write_completed = true;
}

void APP_Event_Detected()
{
    appControl.waiting_usart_answer = false;
    appControl.usart_right_answer="";

}

void APP_UARTError()
{
    changeState(APP_Error,APP_None_LV1,APP_None_LV2);
}

void writeUSART(char * data, size_t size)
{
    int i=0;
    while (i<size)
    {
        /* Send character */
        PLIB_USART_TransmitterByteSend(USART_ID_1, *data);

        /* Increment to address of next character */
        data++;
        i++;
        /* Wait for the transmit shift register to empty (transfer completed) */
        while (!PLIB_USART_TransmitterIsEmpty(USART_ID_1));
    }
}

void writeUSARTBuffer()
{
    writeUSART(appData.buffer, appData.bufferSize);
}

bool writeByteI2C(char * data)
{
    PLIB_I2C_TransmitterByteSend(I2C_MODULE_MASTER, *data);
    while(PLIB_I2C_TransmitterIsBusy(I2C_MODULE_MASTER));               //Wait as long as TBF = 1
    while(!PLIB_I2C_TransmitterByteHasCompleted(I2C_MODULE_MASTER));    //Wait as long as TRSTAT == 1
    return true;
}

void changeState(APP_STATES_LV0 zero, APP_STATES_LV1 one, APP_STATES_LV2 two)
{
    appData.currentState.lvl_0 = zero;
    appData.currentState.lvl_1 = one;
    appData.currentState.lvl_2 = two;
}

void APP_Tasks( void )
{
    char len_of_str = 8;

    switch ( appData.currentState.lvl_0 )
    {
        case APP_SettingUp_Bluetooth:
        {
            switch(appData.currentState.lvl_1)
            {
                case APP_Openning_UART:
                {
                    PLIB_USART_Enable(USART_ID_1);

                    changeState(APP_SettingUp_Bluetooth,APP_Configuring_Bluetooth,APP_BT_WakingUp);
                }
                break;
                case APP_Configuring_Bluetooth:
                {
                    switch(appData.currentState.lvl_2)
                    {
                        case APP_BT_WakingUp:
                        {
                            PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
                            appControl.waiting_usart_answer = true;
                            appControl.usart_right_answer="CMD\r\n";
                            setWaitingString(appControl.usart_right_answer);

                            PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_C, BSP_BTH_MLDP );
                            delay_some(20);
                            PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_B, BSP_BTH_WS);

                            while(appControl.waiting_usart_answer);
                            PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);

                            while(!PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_A, BSP_BTH_ACTV));
                            
                            changeState(APP_SettingUp_Bluetooth,APP_Configuring_Bluetooth,APP_BT_SettingUp);
                            return;
                        }
                        break;
                        case APP_BT_SettingUp:
                        {
                            int i;

                            appControl.waiting_usart_answer = true;
                            appControl.usart_right_answer="CMD\r\n";
                            setWaitingString(appControl.usart_right_answer);

                            PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
                            for(i=0; i<=7; i++)
                            {
                                writeUSART(BTInit[i], strlen(BTInit[i]));
                                delay_some(200);
                            }

                            while(appControl.waiting_usart_answer);
                            PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
                            changeState(APP_Waiting_BT_Connection,APP_BT_Advertising,APP_None_LV2);
                        }
                        break;
                    }
                }
                break;
            }
        }
        break;
        case APP_Waiting_BT_Connection:
        {
            switch(appData.currentState.lvl_1)
            {
                case APP_BT_Advertising:
                {
                    //appControl.usart_write_completed = false;
                    //writeUSART(BTStartScan, strlen(BTStartScan));

                    //appControl.waiting_usart_answer = true;
                    //appControl.usart_right_answer="Connected\r\n";
                    //setWaitingString(appControl.usart_right_answer);

                    //while(appControl.waiting_usart_answer);
                    
                    while(!PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_C, BSP_BTH_CON))
                    {
                        delay_some(10);
                    }

                    changeState(APP_Waiting_BT_Connection,APP_BT_Setting_MLDP,APP_None_LV2);
                    return;
                }
                break;
                case APP_BT_Setting_MLDP:
                {
                    writeUSART(BTBond, strlen(BTBond));

                    appControl.usart_right_answer="MLDP\r\n";
                    appControl.waiting_usart_answer = true;
                    PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_C, BSP_BTH_MLDP );
                    while(appControl.waiting_usart_answer);
                    return;
                }
                break;
            }
        }
        break;

        case APP_SettingUp_I2C:
        {
            PLIB_I2C_Disable(I2C_MODULE_MASTER);
            PLIB_I2C_Enable(I2C_MODULE_MASTER);
            delay_some(50);
            changeState(APP_Waiting_for_Data,APP_None_LV1,APP_None_LV2);
            return;
        }
        break;
        
        case APP_Waiting_for_Data:
        {
            if(!appControl.waiting_bt_data)
            {
                appControl.waiting_bt_data = false;
            }

            while (len_of_str > 0) // TODO: Change the length according to the app protocol
            {
                PLIB_I2C_MasterReceiverClock1Byte(I2C_MODULE_MASTER);
                while(!PLIB_I2C_ReceivedByteIsAvailable(I2C_MODULE_MASTER));
                appControl.receiving_i2c = true;
                I2CRxBuff[len_of_str] = PLIB_I2C_ReceivedByteGet(I2C_MODULE_MASTER);
                if (len_of_str > 1)
                {
                    if ( PLIB_I2C_MasterReceiverReadyToAcknowledge ( I2C_MODULE_MASTER ) )
                    PLIB_I2C_ReceivedByteAcknowledge ( I2C_MODULE_MASTER, true );     //Send ACK to Slave
                }
                len_of_str--;
                delay_some(100);
            }
            if ( PLIB_I2C_MasterReceiverReadyToAcknowledge ( I2C_MODULE_MASTER ) )
            {
               PLIB_I2C_ReceivedByteAcknowledge ( I2C_MODULE_MASTER, false );     //last byte; send NACK to Slave, no more data needed
               appControl.receiving_i2c = false;
               changeState(APP_Sending_to_Bluetooth,APP_None_LV1,APP_None_LV2);
               return;
            }
        }
        break;
        
        case APP_Sending_to_I2C:
        {
            switch(appData.currentState.lvl_1)
            {
                case APP_Starting_I2C:
                {
                    while(!PLIB_I2C_TransmitterByteHasCompleted(I2C_MODULE_MASTER));   //Wait as long as TRSTAT == 1
                    while(PLIB_I2C_ArbitrationLossHasOccurred(I2C_MODULE_MASTER));     //Wait as long as BCL = 1
                    I2C1STATCLR = 0x00000080;           //clear IWCOL bit
                    PLIB_I2C_MasterStart(I2C_MODULE_MASTER);

                    //TODO: Next State
                }
                break;

                case APP_Writting_I2C:
                {
                    int i=0;
                    while (i <= appData.bufferSize)
                    {
                        writeByteI2C(&appData.buffer[i]);
                        i++;
                    }
                    //TODO: Next State
                }
                break;

                case APP_Stopping_I2C:
                {
                    PLIB_I2C_MasterStop(I2C_MODULE_MASTER);

                    //TODO: Next State
                }
                break;
            }
        }
        break;

        case APP_Sending_to_Bluetooth:
        {
            appControl.usart_write_completed = false;
            writeUSARTBuffer();

            while(!appControl.usart_write_completed);
            
            changeState(APP_Waiting_for_Data,APP_None_LV1,APP_None_LV2);
            return;
        }
        break;
        
        case APP_Error:
        {

        }
        break;

        default:
            break;
    }
}
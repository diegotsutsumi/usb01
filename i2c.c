/*
 * File:   i2c.c
 * Author: diego.tsutsumi
 *
 * Created on 3th December 2014, 10:35
 */

#ifndef I2C_C
#define  I2C_C

#include "i2c.h"

I2C_Object i2c_obj;

bool masterTransferIsComplete(void)
{
    return !PLIB_I2C_TransmitterIsBusy(I2C_ID_1);
}

void I2C_Init(uint32_t baudRate, uint32_t clockFrequency)
{
    PLIB_I2C_Disable(I2C_ID_1);
    
    PLIB_I2C_BaudRateSet(I2C_ID_1, clockFrequency, baudRate);
    PLIB_I2C_HighFrequencyEnable(I2C_ID_1);
    PLIB_I2C_SlaveAddress10BitSet(I2C_ID_1, MY_ADDR);
    PLIB_I2C_SlaveClockStretchingDisable (I2C_ID_1);
    PLIB_I2C_GeneralCallEnable(I2C_ID_1);
    PLIB_I2C_SlaveClockRelease(I2C_ID_1);
    PLIB_I2C_SlaveMask10BitSet(I2C_ID_1, 0x10);

    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_I2C_1_SLAVE);
    PLIB_INT_SourceDisable(INT_ID_0,INT_SOURCE_I2C_1_SLAVE);

    SYS_INT_VectorPrioritySet(INT_VECTOR_I2C1, INT_PRIORITY_LEVEL3);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_I2C1, INT_SUBPRIORITY_LEVEL0);
    
    i2c_obj.current_mode = I2C_NO_MODE;
    changeI2CState(I2C_STATE0_Uninitialized,I2C_STATE1_None);
}

void I2C_InitObject()
{
    int i;
    i2c_obj.waiting_start_ok = false;
    i2c_obj.waiting_stop_ok = false;
    i2c_obj.waiting_byte_ok = false;
    i2c_obj.slave_data_interrupt = false;
    i2c_obj.slave_address_interrupt = false;
    i2c_obj.entry_flag = true;
    i2c_obj.addrIndex = 0;
    i2c_obj.nackCounter = 0;

    i2c_obj.rxNumPackets = 0;
    i2c_obj.txNumPackets = 0;
    i2c_obj.to_transmit = 0;
    i2c_obj.tx_ok = false;
    i2c_obj.tx_num_tries = 0;

    i2c_obj.rx_alloc_idx = 254;
    i2c_obj.tx_alloc_idx = 255;

    for(i=0;i<BUFFER_RX_NUMBER;i++)
    {
        i2c_obj.rxStatus[i] = I2C_RX_BUFFER_FREE;
    }

    for(i=0;i<BUFFER_TX_NUMBER;i++)
    {
        i2c_obj.TXAddress[i][0] = 0xF0; //AVL address as default
        i2c_obj.TXAddress[i][1] = 0x00; //AVL address as default
        i2c_obj.txStatus[i] = I2C_TX_BUFFER_FREE;
    }

    changeI2CState(I2C_STATE0_SettingUpSlave,I2C_STATE1_None);
}

void I2C_DeinitObject()
{
    int i;

    I2C_DeInitSlave();
    I2C_DeInitMaster();

    i2c_obj.waiting_start_ok = false;
    i2c_obj.waiting_stop_ok = false;
    i2c_obj.waiting_byte_ok = false;
    i2c_obj.slave_data_interrupt = false;
    i2c_obj.slave_address_interrupt = false;
    i2c_obj.entry_flag = true;
    i2c_obj.addrIndex = 0;
    i2c_obj.nackCounter = 0;

    i2c_obj.rxNumPackets = 0;
    i2c_obj.txNumPackets = 0;
    i2c_obj.to_transmit = 0;
    i2c_obj.tx_ok = false;
    i2c_obj.tx_num_tries = 0;

    i2c_obj.rx_alloc_idx = 254;
    i2c_obj.tx_alloc_idx = 255;

    for(i=0;i<BUFFER_RX_NUMBER;i++)
    {
        i2c_obj.rxStatus[i] = I2C_RX_BUFFER_FREE;
    }

    for(i=0;i<BUFFER_TX_NUMBER;i++)
    {
        i2c_obj.TXAddress[i][0] = 0xF0; //AVL address as default
        i2c_obj.TXAddress[i][1] = 0x00; //AVL address as default
        i2c_obj.txStatus[i] = I2C_TX_BUFFER_FREE;
    }

    changeI2CState(I2C_STATE0_Uninitialized,I2C_STATE1_None);
}

void I2C_InitMaster()
{
    I2C_DeInitSlave();

    i2c_obj.current_mode = I2C_MASTER_MODE;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_I2C_1_MASTER);
    PLIB_INT_SourceEnable(INT_ID_0,INT_SOURCE_I2C_1_MASTER);
    PLIB_I2C_Enable(I2C_ID_1);
}

void I2C_DeInitMaster()
{
    PLIB_I2C_Disable(I2C_ID_1);

    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_I2C_1_MASTER);
    PLIB_INT_SourceDisable(INT_ID_0,INT_SOURCE_I2C_1_MASTER);
    i2c_obj.current_mode = I2C_NO_MODE;
}

void I2C_InitSlave()
{
    I2C_DeInitMaster();

    i2c_obj.current_mode = I2C_SLAVE_MODE;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_I2C_1_SLAVE);
    PLIB_INT_SourceEnable(INT_ID_0,INT_SOURCE_I2C_1_SLAVE);
    PLIB_I2C_Enable(I2C_ID_1);
}
void I2C_DeInitSlave()
{
    PLIB_I2C_Disable(I2C_ID_1);

    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_I2C_1_SLAVE);
    PLIB_INT_SourceDisable(INT_ID_0,INT_SOURCE_I2C_1_SLAVE);
    i2c_obj.current_mode = I2C_NO_MODE;
}

char I2C_SlaveRead(void)
{
     PLIB_I2C_ReceiverOverflowClear(I2C_ID_1);
     return (PLIB_I2C_ReceivedByteGet(I2C_ID_1));
}

void I2C_Set_Event_Handler(I2C_EVENT_HANDLER handler)
{
    if(handler!=0)
    {
        i2c_obj.event_handler = handler;
    }
}

void changeI2CState(I2C_STATES_LVL0 a, I2C_STATES_LVL1 b)
{
    i2c_obj.current_state.lvl0 = a;
    i2c_obj.current_state.lvl1 = b;
}

//----------------------------RX BUFFER MANAGEMENT BEGIN----------------------------------
uint8_t allocI2CRxBuffIndex() //Called By I2C when address matched
{
    int i;
    if(i2c_obj.rxNumPackets>BUFFER_RX_NUMBER)
    {
        return 254; //Buffer FULL
    }
    for(i=0;i<BUFFER_RX_NUMBER;i++)
    {
        if(i2c_obj.rxStatus[i]==I2C_RX_BUFFER_FREE)
        {
            i2c_obj.rxNumPackets++;
            i2c_obj.rxStatus[i] = I2C_RX_BUFFER_I2C_USING;
            return i;
        }
    }
    return 254; //Should never end up here
}
I2C_RETURN I2CFinishedReceiving() // Called when either a stop bit was detected during reception or the received packet reached 256 bytes
{
    int i;
    for(i=0;i<BUFFER_RX_NUMBER;i++)
    {
        if(i2c_obj.rxStatus[i]==I2C_RX_BUFFER_I2C_USING)
        {
            i2c_obj.rxStatus[i] = I2C_RX_BUFFER_READY;
            i2c_obj.event_handler(I2C_EVENT_DATA_READY);
            return I2C_RETURN_SUCCESS;
        }
    }
    return I2C_RETURN_FAIL;
}
BYTE * getI2CRxBuffer(uint16_t * size, uint8_t * handler) //Called by APP when starting transmition
{
    int i;
    if(i2c_obj.rxNumPackets==0)
    {
        *size=0;
        *handler=254;
        return 0;
    }
    
    for(i=0;i<BUFFER_RX_NUMBER;i++)
    {
        if(i2c_obj.rxStatus[i]==I2C_RX_BUFFER_READY)
        {
            i2c_obj.rxStatus[i] = I2C_RX_BUFFER_APP_USING;
            *handler=i;
            *size = i2c_obj.I2CRxBuffSize[i];
            return (BYTE *)(&(i2c_obj.I2CRxBuff[i][0]));
        }
    }
    *size=0;
    *handler=254;
    return 0;
}
I2C_RETURN freeI2CRxBuffIndex(uint8_t handler) //Called by APP when transmition is finished
{
    if(i2c_obj.rxStatus[handler]==I2C_RX_BUFFER_APP_USING)
    {
        i2c_obj.I2CRxBuffSize[handler]=0;
        i2c_obj.rxStatus[handler] = I2C_RX_BUFFER_FREE;
        i2c_obj.rxNumPackets--;
        return I2C_RETURN_SUCCESS;
    }
    return I2C_RETURN_FAIL;
}

//----------------------------RX BUFFER MANAGEMENT END------------------------------------



//----------------------------TX BUFFER MANAGEMENT BEGIN----------------------------------
BYTE * allocI2CTXBuffer(uint8_t * out_index, uint16_t *out_maxSize) //Called By APP when it starts listening USB line
{
    int i;
    if(i2c_obj.txNumPackets>BUFFER_TX_NUMBER)
    {
        *out_maxSize=0;
        *out_index=255;
        return 0; //Buffer FULL
    }

    for(i=0;i<BUFFER_TX_NUMBER;i++)
    {
        if(i2c_obj.txStatus[i]==I2C_TX_BUFFER_FREE)
        {
            i2c_obj.txNumPackets++;
            i2c_obj.txStatus[i] = I2C_TX_BUFFER_APP_USING;
            *out_maxSize = BUFFER_SIZE;
            *out_index = i;
            return (BYTE *)(&(i2c_obj.I2CTxBuff[i][0]));
        }
    }
    
    *out_maxSize=0;
    *out_index=255;
    return 0; //Buffer FULL
}

I2C_RETURN I2CStartTX(uint8_t index, uint16_t size, uint8_t txaddress) //Called by APP when it received something on the "index" previously allocated
{
    if(index<0 || index>(BUFFER_TX_NUMBER-1))
    {
        return I2C_RETURN_FAIL;
    }
    if(i2c_obj.txStatus[index] != I2C_TX_BUFFER_APP_USING)
    {
        return I2C_RETURN_FAIL;
    }

    i2c_obj.TXAddress[index][0]=0xF0;
    i2c_obj.TXAddress[index][1]=txaddress;
    
    i2c_obj.txStatus[index] = I2C_TX_BUFFER_READY_TO_TRANSMIT;
    i2c_obj.I2CTxBuffSize[index] = size;
    i2c_obj.to_transmit++; //this flag is decreased when i2c finished transmitting
    return I2C_RETURN_SUCCESS;
}

uint8_t getI2CTxBufferIndex() //Called by I2C when starting transmition
{
    int i;
    if(i2c_obj.txNumPackets==0 || i2c_obj.to_transmit==0)
    {
        return 255;
    }

    for(i=0;i<BUFFER_TX_NUMBER;i++)
    {
        if(i2c_obj.txStatus[i]==I2C_TX_BUFFER_READY_TO_TRANSMIT)
        {
            i2c_obj.txStatus[i] = I2C_TX_BUFFER_I2C_USING;
            return i;
        }
    }

    return 255;
}

I2C_RETURN freeI2CTXBuffer() //Called by I2C when finished transmitting the transfer previously started by "I2CStartTx()"
{
    int i;

    for(i=0;i<BUFFER_TX_NUMBER;i++)
    {
        if(i2c_obj.txStatus[i]==I2C_TX_BUFFER_I2C_USING)
        {
            i2c_obj.I2CTxBuffSize[i]=0;
            i2c_obj.txStatus[i] = I2C_TX_BUFFER_FREE;
            i2c_obj.txNumPackets--;
            i2c_obj.to_transmit--;
            return I2C_RETURN_SUCCESS;
        }
    }
    return I2C_RETURN_FAIL;
}
//----------------------------TX BUFFER MANAGEMENT END------------------------------------



void I2C_Tasks()
{
    char dummy;
    switch (i2c_obj.current_state.lvl0)
    {
        case I2C_STATE0_Uninitialized:
        {

        }
        break;
        case I2C_STATE0_SettingUpSlave:
        {
            I2C_InitSlave();
            changeI2CState(I2C_STATE0_ListeningLine,I2C_STATE1_WaitingStart);
        }
        break;
        case I2C_STATE0_ListeningLine:
        {
            switch(i2c_obj.current_state.lvl1)
            {
                case I2C_STATE1_WaitingStart:
                {
                    if(I2C1STATbits.S && !I2C1STATbits.P) //Start condition
                    {
                        changeI2CState(I2C_STATE0_ListeningLine,I2C_STATE1_WaitingAddress);
                        i2c_obj.addrIndex=0;
                    }
                    if(i2c_obj.to_transmit>0)
                    {
                        i2c_obj.tx_alloc_idx = getI2CTxBufferIndex();
                        if(i2c_obj.tx_alloc_idx!=255) //Should always be true
                        {
                            changeI2CState(I2C_STATE0_SettingUpMaster,I2C_STATE1_None);
                            i2c_obj.tx_num_tries = 0;
                        }
                        else
                        {
                            changeI2CState(I2C_STATE0_Error,I2C_STATE1_None);
                        }
                    }
                }
                break;

                case I2C_STATE1_WaitingAddress:
                {
                    if(!I2C1STATbits.S && I2C1STATbits.P) //Stop condition
                    {
                        changeI2CState(I2C_STATE0_ListeningLine,I2C_STATE1_WaitingStart);
                    }
                    if(i2c_obj.slave_address_interrupt)
                    {
                        if(PLIB_I2C_ReceivedByteIsAvailable(I2C_ID_1))
                        {
                            dummy = I2C_SlaveRead();
                            PLIB_I2C_SlaveClockRelease(I2C_ID_1);
                            i2c_obj.slave_address_interrupt = false;
                            i2c_obj.addrIndex++;
                            if(i2c_obj.addrIndex>=2 || dummy==0)
                            {
                                i2c_obj.addrIndex=0;
                                i2c_obj.rx_alloc_idx = allocI2CRxBuffIndex();
                                changeI2CState(I2C_STATE0_ListeningLine,I2C_STATE1_ReceivingByte);
                            }
                        }
                    }
                }
                break;

                case I2C_STATE1_ReceivingByte:
                {
                    if(!I2C1STATbits.S && I2C1STATbits.P) //Stop condition
                    {
                        if(i2c_obj.rx_alloc_idx<BUFFER_RX_NUMBER && i2c_obj.rx_alloc_idx>=0)
                        {
                            I2CFinishedReceiving();
                            i2c_obj.rx_alloc_idx=99;
                        }
                        changeI2CState(I2C_STATE0_ListeningLine,I2C_STATE1_WaitingStart);
                    }
                    if(i2c_obj.slave_data_interrupt)
                    {
                        if(PLIB_I2C_ReceivedByteIsAvailable(I2C_ID_1))
                        {
                            i2c_obj.slave_data_interrupt = false;
                            if(i2c_obj.rx_alloc_idx==254) //Buffer FULL
                            {
                                i2c_obj.nackCounter++;
                                if(i2c_obj.nackCounter<2)
                                {
                                    dummy = I2C_SlaveRead(); //ACK the first BYTE and NACK the rest of them until master stops
                                }
                            }
                            else
                            {
                                if(i2c_obj.I2CRxBuffSize[i2c_obj.rx_alloc_idx]<(BUFFER_SIZE-1))
                                {
                                    i2c_obj.I2CRxBuff[i2c_obj.rx_alloc_idx][i2c_obj.I2CRxBuffSize[i2c_obj.rx_alloc_idx]] = I2C_SlaveRead();
                                    i2c_obj.I2CRxBuffSize[i2c_obj.rx_alloc_idx]++;
                                    PLIB_I2C_SlaveClockRelease(I2C_ID_1);
                                }
                                else
                                {
                                    if(i2c_obj.rx_alloc_idx<BUFFER_RX_NUMBER && i2c_obj.rx_alloc_idx>=0)
                                    {
                                        I2CFinishedReceiving();
                                        i2c_obj.rx_alloc_idx=99;
                                    }
                                    changeI2CState(I2C_STATE0_ListeningLine,I2C_STATE1_WaitingStart);
                                }
                            }
                        }
                    }
                }
                break;
                default:
                {
                    changeI2CState(I2C_STATE0_Error,I2C_STATE1_None);
                }
                break;
            }
        }
        break;
        
        case I2C_STATE0_SettingUpMaster:
        {
            I2C_InitMaster();
            changeI2CState(I2C_STATE0_Sending,I2C_STATE1_Starting);
            i2c_obj.entry_flag = true;
        }
        break;
        
        case I2C_STATE0_Sending:
        {
            switch(i2c_obj.current_state.lvl1)
            {
                case I2C_STATE1_Starting:
                {
                    if(!PLIB_I2C_TransmitterIsBusy(I2C_ID_1) && PLIB_I2C_TransmitterByteHasCompleted(I2C_ID_1))
                    {
                        if(i2c_obj.entry_flag)
                        {
                            PLIB_I2C_MasterStart(I2C_ID_1);
                            i2c_obj.waiting_start_ok = true;
                            i2c_obj.tx_ok = false;
                            i2c_obj.entry_flag = false;
                        }
                    }
                }
                break;

                case I2C_STATE1_Addressing:
                {
                    if(i2c_obj.entry_flag)
                    {
                        i2c_obj.tx_count=0;
                        i2c_obj.entry_flag = false;
                        i2c_obj.waiting_byte_ok = false;
                    }
                    if(!i2c_obj.waiting_byte_ok)
                    {
                        if(!PLIB_I2C_TransmitterIsBusy(I2C_ID_1) && PLIB_I2C_TransmitterByteHasCompleted(I2C_ID_1))
                        {
                            PLIB_I2C_TransmitterByteSend(I2C_ID_1,i2c_obj.TXAddress[i2c_obj.tx_alloc_idx][i2c_obj.tx_count]);
                            i2c_obj.waiting_byte_ok = true;
                            i2c_obj.tx_count++;
                            if(i2c_obj.tx_count == 2)
                            {
                                changeI2CState(I2C_STATE0_Sending,I2C_STATE1_Writting);
                                i2c_obj.entry_flag = true;
                            }
                        }
                    }
                }
                break;
                
                case I2C_STATE1_Writting:
                {
                    if(i2c_obj.entry_flag)
                    {
                        i2c_obj.tx_count=0;
                        i2c_obj.entry_flag = false;
                        i2c_obj.waiting_byte_ok = false;
                    }
                    if(!i2c_obj.waiting_byte_ok)
                    {
                        if(!PLIB_I2C_TransmitterIsBusy(I2C_ID_1) && PLIB_I2C_TransmitterByteHasCompleted(I2C_ID_1))
                        {
                            PLIB_I2C_TransmitterByteSend(I2C_ID_1,i2c_obj.I2CTxBuff[i2c_obj.tx_alloc_idx][i2c_obj.tx_count]);
                            i2c_obj.waiting_byte_ok = true;
                            i2c_obj.tx_count++;
                            if(i2c_obj.tx_count >= i2c_obj.I2CTxBuffSize[i2c_obj.tx_alloc_idx])
                            {
                                changeI2CState(I2C_STATE0_Sending,I2C_STATE1_Stopping); //Reached buffer size
                                i2c_obj.tx_ok = true;
                                i2c_obj.entry_flag = true;
                            }
                        }
                    }
                }
                break;
                
                case I2C_STATE1_Stopping:
                {
                    if(!PLIB_I2C_TransmitterIsBusy(I2C_ID_1) && PLIB_I2C_TransmitterByteHasCompleted(I2C_ID_1))
                    {
                        if(i2c_obj.entry_flag)
                        {
                            PLIB_I2C_MasterStop(I2C_ID_1);
                            i2c_obj.waiting_stop_ok = true;
                            i2c_obj.entry_flag = false;
                        }
                    }
                }
                break;
                default:
                {
                    changeI2CState(I2C_STATE0_Error,I2C_STATE1_None);
                }
                break;
            }
        }
        break;
        
        case I2C_STATE0_Error:
        {
        }
        break;
    }
}

void I2C_Tasks_ISR()
{
    if(i2c_obj.current_mode == I2C_SLAVE_MODE)
    {
        if ((!PLIB_I2C_SlaveReadIsRequested(I2C_ID_1)) && (PLIB_I2C_SlaveAddressIsDetected(I2C_ID_1)))
        {
            if(i2c_obj.current_state.lvl0==I2C_STATE0_ListeningLine &&
               i2c_obj.current_state.lvl1==I2C_STATE1_WaitingAddress)
            {
                i2c_obj.slave_address_interrupt = true;
            }
        }
        if ((!PLIB_I2C_SlaveReadIsRequested(I2C_ID_1)) && (PLIB_I2C_SlaveDataIsDetected(I2C_ID_1)))
        {
            if(i2c_obj.current_state.lvl0==I2C_STATE0_ListeningLine &&
               i2c_obj.current_state.lvl1==I2C_STATE1_ReceivingByte)
            {
                i2c_obj.slave_data_interrupt = true;
            }
        }
    }

    if(i2c_obj.current_mode == I2C_MASTER_MODE)
    {
        if(PLIB_I2C_ArbitrationLossHasOccurred(I2C_ID_1))
        {
            changeI2CState(I2C_STATE0_SettingUpMaster,I2C_STATE1_None); //Bus Collision detected,
                                                                        //restarting transmission
            if(i2c_obj.tx_num_tries>=TX_MAX_TRIES)
            {
                freeI2CTXBuffer();
                i2c_obj.event_handler(I2C_EVENT_DATA_NOT_SENT);
                changeI2CState(I2C_STATE0_SettingUpSlave,I2C_STATE1_None);
                i2c_obj.tx_num_tries=0;
            }
            else
            {
                changeI2CState(I2C_STATE0_SettingUpMaster,I2C_STATE1_None);
                i2c_obj.tx_num_tries++;
            }
        }
        else
        {
            if(i2c_obj.waiting_start_ok && I2C1CONbits.SEN==0)
            {
                changeI2CState(I2C_STATE0_Sending,I2C_STATE1_Addressing);
                i2c_obj.entry_flag = true;
                I2C1STATCLR = 0x00000080;  //clear IWCOL bit
                i2c_obj.waiting_start_ok = false;
            }
            if(i2c_obj.waiting_byte_ok)
            {
                if(!PLIB_I2C_TransmitterByteWasAcknowledged(I2C_ID_1))
                {
                    changeI2CState(I2C_STATE0_Sending,I2C_STATE1_Stopping); //NACK received by slave (AVL)
                    i2c_obj.tx_ok = false;
                    i2c_obj.entry_flag = true;
                }
                i2c_obj.waiting_byte_ok = false;
            }
            if(i2c_obj.waiting_stop_ok && I2C1CONbits.PEN==0)
            {
                if(i2c_obj.tx_ok)
                {
                    freeI2CTXBuffer();
                    i2c_obj.event_handler(I2C_EVENT_DATA_SENT);
                    changeI2CState(I2C_STATE0_SettingUpSlave,I2C_STATE1_None);
                }
                else //Slave Nacked
                {
                    if(i2c_obj.tx_num_tries>=TX_MAX_TRIES)
                    {
                        freeI2CTXBuffer();
                        i2c_obj.event_handler(I2C_EVENT_DATA_NOT_SENT);
                        changeI2CState(I2C_STATE0_SettingUpSlave,I2C_STATE1_None);
                        i2c_obj.tx_num_tries=0;
                    }
                    else
                    {
                        changeI2CState(I2C_STATE0_SettingUpMaster,I2C_STATE1_None);
                        i2c_obj.tx_num_tries++;
                    }
                }
                i2c_obj.waiting_stop_ok = false;
            }
        }
    }
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_I2C_1_SLAVE);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_I2C_1_MASTER);
}

#endif
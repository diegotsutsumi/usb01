/*
 * File:   memory.c
 * Author: diego.tsutsumi
 *
 * Created on 8th January 2014, 14:32
 */

#include "memory.h"


MEM_Object mem_obj;
int aux;
char dummy;

void MEM_Init(uint32_t baudRate, uint32_t clockFreq)
{
    char dummy;
    PLIB_SPI_Disable(SPI_ID_2);
    dummy = PLIB_SPI_BufferRead(SPI_ID_2);
    PLIB_SPI_MasterEnable(SPI_ID_2);
    PLIB_SPI_BaudRateSet(SPI_ID_2,clockFreq,baudRate);
    PLIB_SPI_OutputDataPhaseSelect(SPI_ID_2,SPI_OUTPUT_DATA_PHASE_ON_ACTIVE_TO_IDLE_CLOCK);


    changeMEMState(MEM_STATE0_Uninitialized,MEM_STATE1_None);
}

void changeMEMState(MEM_STATES_LVL0 _lvl0, MEM_STATES_LVL1 _lvl1)
{
    mem_obj.current_state.lvl0 = _lvl0;
    mem_obj.current_state.lvl1 = _lvl1;
}

void MEM_InitObj(bool erase)
{
    PLIB_SPI_Enable(SPI_ID_2);
    mem_obj.buff_size=0;
    mem_obj.buffer=0;
    mem_obj.current_addr.Val=0;
    mem_obj.addrCount=0;
    mem_obj.byteCount=0;
    mem_obj.pageCount=0;

    //mem_obj.waiting_write=false;
    mem_obj.entry_flag=true;
    mem_obj.page_overflow=false;
    mem_obj.page_init=true;
    mem_obj.buffer_empty=true;
    mem_obj.event_handler=0;
    if(erase)
    {
        changeMEMState(MEM_STATE0_Erasing,MEM_STATE1_WritingEnable);
    }
    else
    {
        changeMEMState(MEM_STATE0_WaitingBufferFill,MEM_STATE1_None);
    }
}

void MEM_DeinitObj()
{
    PLIB_SPI_Disable(SPI_ID_2);
    mem_obj.buff_size=0;
    mem_obj.buffer=0;
    mem_obj.current_addr.Val=0;
    mem_obj.addrCount=0;
    mem_obj.byteCount=0;
    mem_obj.pageCount=0;

    //mem_obj.waiting_write=false;
    mem_obj.entry_flag=false;
    mem_obj.page_overflow=false;
    mem_obj.page_init=true;
    mem_obj.buffer_empty=true;
    mem_obj.event_handler=0;
    changeMEMState(MEM_STATE0_Uninitialized,MEM_STATE1_None);
}

bool MEM_FillBuffer(uint8_t * _buffer, uint16_t _buff_size)
{
    if(_buffer!=0 && _buff_size!=0)
    {
        mem_obj.buffer = _buffer;
        mem_obj.buff_size = _buff_size;
        //mem_obj.entry_flag=true;
        //changeMEMState(MEM_STATE0_Erasing,MEM_STATE1_WritingEnable);
        mem_obj.buffer_empty = false;
        return true;
    }
    return false;
}

bool MEM_SetEventHandler(MEM_EVENT_HANDLER handler)
{
    if(handler!=0)
    {
        mem_obj.event_handler = handler;
        return true;
    }
    return false;
}

void MEM_Tasks()
{
    switch(mem_obj.current_state.lvl0)
    {
        case MEM_STATE0_Uninitialized:
        {
        }
        break;

        case MEM_STATE0_Erasing:
        {
            switch(mem_obj.current_state.lvl1)
            {
                case MEM_STATE1_WritingEnable:
                {
                    if(mem_obj.entry_flag)
                    {
                        mem_obj.entry_flag=false;
                        //mem_obj.waiting_write=true;
                        PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                        PLIB_SPI_BufferWrite(SPI_ID_2,0x06);
                    }
                    else
                    {
                        if(PLIB_SPI_ReceiverBufferIsFull(SPI_ID_2))
                        {
                            dummy = PLIB_SPI_BufferRead(SPI_ID_2);
                            PLIB_SPI_ReceiverOverflowClear(SPI_ID_2);
                            mem_obj.entry_flag=true;
                            PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                            changeMEMState(MEM_STATE0_Erasing,MEM_STATE1_ErasingChip);
                        }
                    }
                }
                break;

                case MEM_STATE1_ErasingChip:
                {
                    if(mem_obj.entry_flag)
                    {
                        mem_obj.entry_flag=false;
                        PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                        PLIB_SPI_BufferWrite(SPI_ID_2,0xC7);
                        mem_obj.aux=0;
                    }
                    else
                    {
                        if(PLIB_SPI_ReceiverBufferIsFull(SPI_ID_2))
                        {
                            dummy = PLIB_SPI_BufferRead(SPI_ID_2);
                            PLIB_SPI_ReceiverOverflowClear(SPI_ID_2);
                            PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                            mem_obj.entry_flag=true;
                            changeMEMState(MEM_STATE0_Erasing,MEM_STATE1_WaitingErase);
                        }
                    }
                }
                break;

                case MEM_STATE1_WaitingErase:
                {
                    char miso;
                    if(mem_obj.entry_flag)
                    {
                        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                        mem_obj.entry_flag=false;
                        PLIB_SPI_BufferWrite(SPI_ID_2,0x05/*Register Address*/);
                        aux=0;
                    }
                    else
                    {
                        if(PLIB_SPI_ReceiverBufferIsFull(SPI_ID_2))
                        {
                            miso = PLIB_SPI_BufferRead(SPI_ID_2);
                            PLIB_SPI_ReceiverOverflowClear(SPI_ID_2);
                            if(aux>0)
                            {
                                if(!(miso & MEM_WIP_MASK)) //Erase performed
                                {
                                    PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                                    mem_obj.event_handler(MEM_EVENT_ERASED);
                                    changeMEMState(MEM_STATE0_WaitingBufferFill,MEM_STATE1_None);

                                }
                                else
                                {
                                    PLIB_SPI_BufferWrite(SPI_ID_2,0/*Dummy*/);
                                }
                            }
                            else
                            {
                                PLIB_SPI_BufferWrite(SPI_ID_2,0/*Dummy*/);
                            }
                            aux++;
                        }
                    }
                }
                break;
                default:
                {
                    //Just to Keep Optimizations working better
                }
                break;
            }
        }
        break;

        case MEM_STATE0_WaitingBufferFill:
        {
            if(!mem_obj.buffer_empty)
            {
                mem_obj.entry_flag=true;
                mem_obj.page_init=true;
                mem_obj.byteCount=0;
                mem_obj.pageCount=0;
                changeMEMState(MEM_STATE0_Writing,MEM_STATE1_WritingEnable);
            }
        }
        break;

        case MEM_STATE0_Writing:
        {
            switch(mem_obj.current_state.lvl1)
            {
                case MEM_STATE1_WritingEnable:
                {
                    if(mem_obj.entry_flag)
                    {
                        mem_obj.entry_flag=false;
                        //mem_obj.waiting_write=true;
                        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                        PLIB_SPI_BufferWrite(SPI_ID_2,0x06/*TODO Check Endianess*/);
                    }
                    else
                    {
                        if(PLIB_SPI_ReceiverBufferIsFull(SPI_ID_2))
                        {
                            dummy = PLIB_SPI_BufferRead(SPI_ID_2);
                            PLIB_SPI_ReceiverOverflowClear(SPI_ID_2);
                            PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                            mem_obj.entry_flag=true;
                            changeMEMState(MEM_STATE0_Writing,MEM_STATE1_PageProgramCommand);
                        }
                    }
                }
                break;

                case MEM_STATE1_PageProgramCommand:
                {
                    if(mem_obj.entry_flag)
                    {
                        mem_obj.entry_flag=false;
                        //mem_obj.waiting_write=true;
                        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                        PLIB_SPI_BufferWrite(SPI_ID_2,0x02/*TODO Check Endianess*/);
                    }
                    else
                    {
                        if(PLIB_SPI_ReceiverBufferIsFull(SPI_ID_2))
                        {
                            mem_obj.entry_flag=true;
                            mem_obj.addrCount=3;
                            changeMEMState(MEM_STATE0_Writing,MEM_STATE1_Addressing);
                        }
                    }
                }
                break;

                case MEM_STATE1_Addressing:
                {
                    if(PLIB_SPI_ReceiverBufferIsFull(SPI_ID_2))
                    {
                        if(mem_obj.addrCount<=0)
                        {
                            mem_obj.entry_flag=true;
                            mem_obj.page_overflow = false;
                            changeMEMState(MEM_STATE0_Writing,MEM_STATE1_ProgrammingBytes);
                        }
                        else
                        {
                            dummy = PLIB_SPI_BufferRead(SPI_ID_2);
                            PLIB_SPI_ReceiverOverflowClear(SPI_ID_2);
                            PLIB_SPI_BufferWrite(SPI_ID_2,mem_obj.current_addr.v[mem_obj.addrCount-1]);
                            mem_obj.addrCount--;
                        }
                    }
                }
                break;
                
                case MEM_STATE1_ProgrammingBytes:
                {
                    if(PLIB_SPI_ReceiverBufferIsFull(SPI_ID_2))
                    {
                        dummy = PLIB_SPI_BufferRead(SPI_ID_2);
                        PLIB_SPI_ReceiverOverflowClear(SPI_ID_2);

                        if(mem_obj.current_addr.byte.LB!=0 || mem_obj.page_init)
                        {
                            mem_obj.page_init=false;
                            if(mem_obj.byteCount>=mem_obj.buff_size)
                            {
                                mem_obj.entry_flag=true;
                                PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                                changeMEMState(MEM_STATE0_Writing,MEM_STATE1_WaitingWrite);
                            }
                            else
                            {
                                PLIB_SPI_BufferWrite(SPI_ID_2,*(mem_obj.buffer + mem_obj.byteCount));
                                mem_obj.current_addr.Val++;
                                mem_obj.byteCount++;
                            }
                        }
                        else  //New Flash Page
                        {
                            PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                            mem_obj.entry_flag=true;
                            mem_obj.page_overflow=true;
                            changeMEMState(MEM_STATE0_Writing,MEM_STATE1_WaitingWrite);
                        }
                    }
                }
                break;

                case MEM_STATE1_WaitingWrite:
                {
                    char miso;
                    if(mem_obj.entry_flag)
                    {
                        PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                        mem_obj.entry_flag=false;
                        aux=0;
                        PLIB_SPI_BufferWrite(SPI_ID_2,0x05); //Status Register
                    }
                    else
                    {
                        if(PLIB_SPI_ReceiverBufferIsFull(SPI_ID_2))
                        {
                            miso = PLIB_SPI_BufferRead(SPI_ID_2);
                            PLIB_SPI_ReceiverOverflowClear(SPI_ID_2);
                            if(aux>0)
                            {
                                if(!(miso & MEM_WIP_MASK)) //Write performed
                                {
                                    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, BSP_SPI2_CS);
                                    mem_obj.entry_flag=true;

                                    if(mem_obj.page_overflow)
                                    {
                                        mem_obj.pageCount++;
                                        mem_obj.page_init = true;
                                        changeMEMState(MEM_STATE0_Writing,MEM_STATE1_WritingEnable);
                                    }
                                    else
                                    {
                                        mem_obj.buffer=0;
                                        mem_obj.buff_size=0;
                                        mem_obj.buffer_empty = true;
                                        mem_obj.page_init = true;
                                        changeMEMState(MEM_STATE0_WaitingBufferFill,MEM_STATE1_None);
                                        mem_obj.event_handler(MEM_EVENT_BUFFER_WRITTEN);
                                    }
                                }
                                else
                                {
                                    PLIB_SPI_BufferWrite(SPI_ID_2,0/*Dummy*/);
                                }
                            }
                            else
                            {
                                PLIB_SPI_BufferWrite(SPI_ID_2,0/*Dummy*/);
                            }
                            aux++;
                        }
                    }
                }
                break;
                default:
                {
                    //Just to Keep Optimizations working better
                }
                break;
            }
        }
        break;

        case MEM_STATE0_Error:
        {
        }
        break;
        
        default:
        {
            //Just to Keep Optimizations working better
        }
        break;
    }
}
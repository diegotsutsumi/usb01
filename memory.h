/* 
 * File:   memory.h
 * Author: diego.tsutsumi
 *
 * Created on 8th January 2014, 14:32
 */

#ifndef MEMORY_H
#define	MEMORY_H

#include "includes.h"
#include "peripheral/spi/plib_spi.h"
#include "peripheral/ports/plib_ports.h"
#include "system_definitions.h"
#include "bsp_config.h"

#define MEM_BASE_ADDR 0x0
#define MEM_WIP_MASK 0x1

typedef enum
{
    MEM_EVENT_ERASED,
    MEM_EVENT_BUFFER_WRITTEN,
    MEM_EVENT_BUFFER_NOT_WRITTEN,
    MEM_EVENT_NONE
} MEM_EVENT;

typedef void (*MEM_EVENT_HANDLER)(MEM_EVENT event);

typedef enum
{
    MEM_STATE0_Uninitialized,
    MEM_STATE0_WaitingBufferFill,
    MEM_STATE0_Erasing,
    MEM_STATE0_Writing,
    MEM_STATE0_Error
}MEM_STATES_LVL0;

typedef enum
{
    MEM_STATE1_EnablingWrite,
    MEM_STATE1_PageProgramCommand,

    //States inside MEM_STATE0_ErasingBlocks
    MEM_STATE1_ErasingChip, //Later implement the block erase separately
    MEM_STATE1_WaitingErase,

    //States inside MEM_STATE0_Writing
    MEM_STATE1_Addressing,
    MEM_STATE1_ProgrammingBytes,
    MEM_STATE1_WaitingWrite,
    MEM_STATE1_None
}MEM_STATES_LVL1;

typedef struct
{
    MEM_STATES_LVL0 lvl0;
    MEM_STATES_LVL1 lvl1;
}MEM_State;

typedef struct
{
    MEM_State current_state;
    uint8_t * buffer;
    uint16_t buff_size;
    DWORD_VAL current_addr;
    int8_t addrCount;
    uint16_t byteCount;
    uint16_t pageCount;

    MEM_EVENT_HANDLER event_handler;

    //volatile bool waiting_write;
    bool buffer_empty;
    bool page_init;
    bool entry_flag;
    bool page_overflow;

    uint8_t aux;
} MEM_Object;

void MEM_Init(uint32_t baudRate, uint32_t clockFreq);
void changeMEMState(MEM_STATES_LVL0 _lvl0, MEM_STATES_LVL1 _lvl1);
bool MEM_InitObj(bool erase);
bool MEM_DeinitObj();
bool MEM_FillBuffer(uint8_t * _buffer, uint16_t _buff_size);
bool MEM_SetEventHandler(MEM_EVENT_HANDLER handler);
void MEM_Tasks();

#endif	/* MEMORY_H */
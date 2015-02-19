/*
 * File:   bsp_config.h
 * Author: diego.tsutsumi
 *
 * Created on 22th October 2014, 10:14
 */


#ifndef _BSP_CONFIG_H
#define _BSP_CONFIG_H

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "peripheral/ports/plib_ports.h"


#define BSP_OSC_FREQUENCY 8000000

typedef enum
{
     /*BSP_UART_RX = PORTS_BIT_POS_6, // Channel C
     BSP_UART_TX = PORTS_BIT_POS_7, // Channel C
     BSP_UART_CTS = PORTS_BIT_POS_8, // Channel C
     BSP_UART_RTS = PORTS_BIT_POS_9, // Channel C*/

     BSP_USB_VBON = PORTS_BIT_POS_14, // Channel B
     BSP_USB_SELECT = PORTS_BIT_POS_7, // Channel B
     BSP_SPI2_CS = PORTS_BIT_POS_9, // Channel B

     BSP_LED1 = PORTS_BIT_POS_4, // Channel C
     BSP_LED2 = PORTS_BIT_POS_3, // Channel C
     BSP_LED3 = PORTS_BIT_POS_9 // Channel A


} BSP_PIN;

void BSP_uart_mapping(void);

void BSP_Initialize(void);




#endif
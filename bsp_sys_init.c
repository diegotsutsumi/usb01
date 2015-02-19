/*
 * File:   bsp_sys_init.c
 * Author: diego.tsutsumi
 *
 * Created on 22th October 2014, 10:14
 */

#include "bsp_config.h"
#include "peripheral/ports/plib_ports.h"

void BSP_Initialize(void )
{
    ANSELA = 0;            // Disable all ADC inputs
    ANSELB = 0;            // Disable all ADC inputs
    ANSELC = 0;            // Disable all ADC inputs

    //Port remap for SPI
    SDI2Rbits.SDI2R=0b0011; //SDI2 -> RPB13
    RPC8Rbits.RPC8R=0b0100; //SDO2 -> RC8
    //RPC9Rbits.RPC9R=0b0100; //SS2 -> RC9

    PLIB_PORTS_PinDirectionOutputSet ( PORTS_ID_0 , PORT_CHANNEL_C , BSP_SPI2_CS );
    PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_B, BSP_SPI2_CS);

    PLIB_PORTS_PinDirectionOutputSet ( PORTS_ID_0 , PORT_CHANNEL_B , BSP_USB_SELECT );
    PLIB_PORTS_PinSet( PORTS_ID_0, PORT_CHANNEL_B, BSP_USB_SELECT);


    
    PLIB_PORTS_PinDirectionOutputSet ( PORTS_ID_0 , PORT_CHANNEL_C , BSP_LED1 );
    PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_C, BSP_LED1);

    PLIB_PORTS_PinDirectionOutputSet ( PORTS_ID_0 , PORT_CHANNEL_C , BSP_LED2 );
    PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_C, BSP_LED2);

    PLIB_PORTS_PinDirectionOutputSet ( PORTS_ID_0 , PORT_CHANNEL_A , BSP_LED3 );
    PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_A, BSP_LED3);
}
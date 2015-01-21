/*
 * File:   system_tasks.c
 * Author: diego.tsutsumi
 *
 * Created on 22th October 2014, 10:12
 */


#include "system_config.h"
#include "system_definitions.h"
#include "app.h"


void SYS_Tasks ( void )
{
    /* Maintain the state machines of all library modules executing polled in
    the system. */

    /* Maintain system services */
    SYS_DEVCON_Tasks(sysObj.sysDevcon);
    SYS_TMR_Tasks(sysObj.sysTmr);

    /* Maintain Device Drivers */
    DRV_TMR_Tasks(sysObj.drvTmr0);

    /* Host layer task routine.*/
#ifndef I2CTXTest
    USB_HOST_Tasks( sysObj.usbHostObject0);
#endif

    /*External Flash Memory Tasks*/
    MEM_Tasks();

    /*I2C Task*/
    I2C_Tasks();

    /* Application task */
    APP_Tasks();
}

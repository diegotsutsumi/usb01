/*
 * File:   system_definitions.h
 * Author: diego.tsutsumi
 *
 * Created on 22th October 2014, 10:14
 */

#ifndef _SYS_DEFINITIONS_H_
#define _SYS_DEFINITIONS_H_

#include <stddef.h>
#include "system/clk/sys_clk.h"
#include "system/devcon/sys_devcon.h"
#include "system/int/sys_int.h"
#include "system/tmr/sys_tmr.h"
#include "driver/tmr/drv_tmr.h"

#include "usb/usb_host.h"
#include "usb/src/usb_host_local.h"

typedef struct
{
    SYS_MODULE_OBJ  sysDevcon;
    SYS_MODULE_OBJ  sysTmr;
    SYS_MODULE_OBJ  drvTmr0;
    SYS_MODULE_OBJ  drvTmr1;


    SYS_MODULE_OBJ  usbHostObject0;

} SYSTEM_OBJECTS;

extern SYSTEM_OBJECTS sysObj;


#endif

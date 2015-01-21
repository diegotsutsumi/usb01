/*
 * File:   system_config.h
 * Author: diego.tsutsumi
 *
 * Created on 22th October 2014, 10:14
 */

#ifndef _SYSTEM_CONFIG_H_
#define _SYSTEM_CONFIG_H_

//#define _PLIB_UNSUPPORTED
#include "bsp_config.h"

#define SYS_CLK_SOURCE                      SYS_CLK_SOURCE_PRIMARY_SYSPLL
#define SYS_CLK_FREQ                        40000000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         20000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       00000000ul
#define SYS_CLK_CONFIG_SYSPLL_INP_DIVISOR   5
#define SYS_CLK_CONFIG_FREQ_ERROR_LIMIT     10
#define SYS_CLK_CONFIGBIT_USBPLL_ENABLE     true
#define SYS_CLK_CONFIGBIT_USBPLL_DIVISOR    5
#define SYS_CLK_WAIT_FOR_SWITCH             true
#define SYS_CLK_KEEP_SECONDARY_OSC_ENABLED  false
#define SYS_CLK_ON_WAIT                     OSC_ON_WAIT_IDLE


#define SYS_BUFFER  false
#define SYS_QUEUE   false


#define SYS_DEVCON_SYSTEM_CLOCK         40000000
#define SYS_DEVCON_PIC32MX_MAX_PB_FREQ  40000000

#define SYS_INT                     true


#define SYS_TMR_POWER_STATE             SYS_MODULE_POWER_RUN_FULL
#define SYS_TMR_DRIVER_INDEX            DRV_TMR_INDEX_0
#define SYS_TMR_MAX_CLIENT_OBJECTS      5
#define SYS_TMR_FREQUENCY               1000
#define SYS_TMR_FREQUENCY_TOLERANCE     10
#define SYS_TMR_UNIT_RESOLUTION         10000
#define SYS_TMR_CLIENT_TOLERANCE        10
#define SYS_TMR_INTERRUPT_NOTIFICATION  false

#define DRV_TMR_INSTANCES_NUMBER           1
#define DRV_TMR_CLIENTS_NUMBER             1
#define DRV_TMR_INTERRUPT_MODE             true

/*** Timer Driver Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX0          TMR_ID_2
#define DRV_TMR_INTERRUPT_SOURCE_IDX0       INT_SOURCE_TIMER_2
#define DRV_TMR_INTERRUPT_VECTOR_IDX0       INT_VECTOR_T2
#define DRV_TMR_ISR_VECTOR_IDX0             _TIMER_2_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX0     INT_PRIORITY_LEVEL4
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX0           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX0               TMR_PRESCALE_VALUE_256
#define DRV_TMR_OPERATION_MODE_IDX0         DRV_TMR_OPERATION_MODE_16_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX0     false
#define DRV_TMR_POWER_STATE_IDX0            SYS_MODULE_POWER_RUN_FULL



/*** USB Driver Configuration ***/
/* Enables Device Support */
#define DRV_USB_DEVICE_SUPPORT      false
/* Enables Device Support */
#define DRV_USB_HOST_SUPPORT        true
/* Maximum USB driver instances */
#define DRV_USB_INSTANCES_NUMBER    1
/* Interrupt mode enabled */
#define DRV_USB_INTERRUPT_MODE      true
/* Number of Endpoints used */
#define DRV_USB_ENDPOINTS_NUMBER    1

/* Section: USB Device Layer Configuration*/
/* Maximum Host layer instances */
#define USB_HOST_INSTANCES_NUMBER  1
/* Provides Host pipes number */
#define DRV_USB_HOST_PIPES_NUMBER    5
/* NAK Limit for Control transfer data stage and Status Stage */
#define DRV_USB_HOST_NAK_LIMIT		200

/* Section: USB Host Layer Configuration */
/* Target peripheral list entries */
#define  USB_HOST_TPL_ENTRIES                1
/* Total number of drivers in this application */
#define USB_HOST_MAX_DRIVER_SUPPORTED        1
/* Total number of devices to be supported */
#define USB_HOST_MAX_DEVICES_NUMBER         1

/* Maximum number of configurations supported per device */
#define USB_HOST_MAX_CONFIGURATION          1

/* Maximum number of interfaces supported per device */
#define USB_HOST_MAX_INTERFACES             2

/* Number of Host Layer Clients */
#define USB_HOST_CLIENTS_NUMBER             1


/* Number of MSD Function driver instances in the application */
#define USB_HOST_CDC_INSTANCES_NUMBER         1


#endif


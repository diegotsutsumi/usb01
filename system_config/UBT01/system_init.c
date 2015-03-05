/*
 * File:   system_init.c
 * Author: diego.tsutsumi
 *
 * Created on 22th Outubro 2014, 10:12
 */
#ifndef _SYSTEM_INIT_C_
#define _SYSTEM_INIT_C_

#include "system_config.h"
#include "app.h"
#include "system_definitions.h"
#include "i2c.h"
#include "memory.h"
#include "android.h"
#include "xc.h"

/*** DEVCFG0 ***/


// PIC32MX250F128D Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_4         // PLL Input Divider (4x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_4         // USB PLL Input Divider (5x Divider)
#pragma config UPLLEN = ON              // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
//#pragma config FWDTWINSZ = WISZ_25      // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF              // JTAG Enable (JTAG Port Enabled)
#pragma config DEBUG = OFF
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)




/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

/*** System Device Control Initialization Data ***/
const SYS_DEVCON_INIT sysDevconInit =
{
    .moduleInit = {0},
};


#define GetSystemClock()            (40000000ul)
#define GetInstructionClock()       (GetSystemClock())
#define GetPeripheralClock()        (GetSystemClock()/(1<<OSCCONbits.PBDIV))

extern USB_HOST_CLASS_DRIVER androidDriver;


const USB_HOST_TARGET_PERIPHERAL_LIST USBTPList[6] =
{
    {TPL_MATCH_VID_PID(0x18D1, 0x2D00), TPL_FLAG_VID_PID, &androidDriver},
    {TPL_MATCH_VID_PID(0x18D1, 0x2D01), TPL_FLAG_VID_PID, &androidDriver},
    {TPL_MATCH_VID_PID(0x0E8D, 0x0000), TPL_FLAG_IGNORE_PID, &androidDriver}, // MediaTek
    {TPL_MATCH_VID_PID(0x18D1, 0x0000), TPL_FLAG_IGNORE_PID, &androidDriver}, // Generic Android
    {TPL_MATCH_VID_PID(0x1662, 0x0000), TPL_FLAG_IGNORE_PID, &androidDriver}, // Positivo Tablet
    {TPL_MATCH_VID_PID(0x2207, 0x0000), TPL_FLAG_IGNORE_PID, &androidDriver}  // Chinese tablet not sending android VID PID, but ubuntu VID PID
};

/****************************************************
 * Endpoint Table needed by the controller driver .
 ****************************************************/

uint8_t __attribute__((aligned(512))) endpointTable[USB_HOST_ENDPOINT_TABLE_SIZE];


const USB_HOST_INIT usbHostInitData =
{
    .moduleInit =  { SYS_MODULE_POWER_RUN_FULL },

    /* Identifies peripheral (PLIB-level) ID */
    .usbID = USB_ID_1,

    .stopInIdle = false,
    .suspendInSleep = false,
    .endpointTable = endpointTable,
    .interruptSource = INT_SOURCE_USB_1,
    .nTPLEntries = 6,
    .usbSpeed = USB_SPEED_FULL,
    .tplList = (USB_HOST_TARGET_PERIPHERAL_LIST *) USBTPList,
};

/*** TMR Driver Initialization Data ***/

const DRV_TMR_INIT drvTmr0InitData =
{
    .moduleInit = { DRV_TMR_POWER_STATE_IDX0 },
    .tmrId = DRV_TMR_PERIPHERAL_ID_IDX0,
    .clockSource = DRV_TMR_CLOCK_SOURCE_IDX0,
    .prescale = DRV_TMR_PRESCALE_IDX0,
    .mode = DRV_TMR_OPERATION_MODE_IDX0,
    .interruptSource = DRV_TMR_INTERRUPT_SOURCE_IDX0,
    .asyncWriteEnable = false,
};

/*** TMR Service Initialization Data ***/
const SYS_TMR_INIT sysTmrInitData =
{
    .moduleInit = {SYS_MODULE_POWER_RUN_FULL},
    .drvIndex = DRV_TMR_INDEX_0,
    .tmrFreq = 1000,
};


SYSTEM_OBJECTS sysObj;

const SYS_CLK_INIT sysClkInit =
{
    .moduleInit = {0},
    .systemClockSource = SYS_CLK_SOURCE,
    .systemClockFrequencyHz = SYS_CLK_FREQ,
    .waitTillComplete = true,
    .secondaryOscKeepEnabled = false,
    .onWaitInstruction = SYS_CLK_ON_WAIT,
};

void SYS_Initialize ( void* data )
{
    SYS_CLK_Initialize(&sysClkInit);
    //SYS_CLK_Initialize(NULL);
    sysObj.sysDevcon = SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, NULL);
    //SYSKEY=0;
    //SYSKEY=0xaa996655;
    //SYSKEY=0x556699aa;  // unlock OSCCON
    //OSCCONbits.UFRCEN = 1; //The provided libraries doesn't support internal FRC as USB clock source
    //OSCTUNbits.TUN0 = 0;
    //OSCTUNbits.TUN1 = 0;
    //OSCTUNbits.TUN2 = 0;
    //OSCTUNbits.TUN3 = 0;
    //OSCTUNbits.TUN4 = 0;
    //OSCTUNbits.TUN5 = 1;
    //SYSKEY = 0x33333333; // Locking sequence
    SYS_DEVCON_PerformanceConfig(40000000);

    BSP_Initialize();

    SYS_INT_Initialize();

    sysObj.drvTmr0 = DRV_TMR_Initialize(DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&drvTmr0InitData);
    SYS_INT_VectorPrioritySet(INT_VECTOR_T2, INT_PRIORITY_LEVEL4);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_T2, INT_SUBPRIORITY_LEVEL1);
    SYS_INT_SourceEnable(INT_SOURCE_TIMER_2);
    sysObj.sysTmr  = SYS_TMR_Initialize(SYS_TMR_INDEX_0, (const SYS_MODULE_INIT  * const)&sysTmrInitData);

    sysObj.usbHostObject0 = USB_HOST_Initialize (USB_HOST_INDEX_0, (SYS_MODULE_INIT *)&usbHostInitData);
    SYS_INT_VectorPrioritySet(INT_VECTOR_USB1, INT_PRIORITY_LEVEL4);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_USB1, INT_SUBPRIORITY_LEVEL0);

    SYS_INT_Enable();

    I2C_Init(20000,GetPeripheralClock()); //I2C 20KHz
    
    MEM_Init(5000000,GetPeripheralClock()); //SPI 5MHz

    AND_Init();

    APP_Initialize();
}
#endif
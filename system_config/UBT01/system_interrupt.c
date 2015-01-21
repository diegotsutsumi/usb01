#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_definitions.h"

void __ISR(_TIMER_2_VECTOR, ipl4) _IntHandlerDrvTmrInstance0(void)
{
    DRV_TMR_Tasks_ISR(sysObj.drvTmr0);
}
void __ISR( _USB_1_VECTOR , ipl4)_InterruptHandler_USB_stub ( void )
{
    USB_HOST_Tasks_ISR(sysObj.usbHostObject0);
}
void __ISR( _I2C_1_VECTOR, ipl3) _IntHandler_I2C_Slave(void)
{
    I2C_Tasks_ISR();
}

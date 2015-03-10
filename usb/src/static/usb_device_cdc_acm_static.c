/*******************************************************************************
 USB CDC ACM SubClass

 Company:
   Microchip Technology Inc.

 File Name:
   usb_device_cdc_acm.c

 Summary:
   USB CDC ACM SubClass

 Description:
   USB CDC ACM SubClass
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
 Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

 Microchip licenses to you the right to use, modify, copy and distribute
 Software only when embedded on a Microchip microcontroller or digital signal
 controller that is integrated into your product or third party product
 (pursuant to the sublicense terms in the accompanying license agreement).

 You should refer to the license agreement accompanying this Software for
 additional information regarding your rights and obligations.

 SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
 CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
 OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
 CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
 SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.
 */

#include "usb/usb_device_cdc.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* AT Command

  Summary:
    Contains the latest AT commands received from the host.

  Description:
    This data type conatins the latest AT commands received from the host.

  Remarks:
    None.
*/

uint8_t atCommand[2];


// *****************************************************************************
/* AT Command Buffer

  Summary:
    Defines the AT Command buffer.

  Description:
    This data type defines the AT Command buffer.

  Remarks:
    None.
*/

USB_CDC_AT_COMMAND atCommandBuffer =
{
    /* AT Command place holder */
    .command = &atCommand[0],
            
    /* size of the command */
    .length = 2
};


// *****************************************************************************
/* Line coding parameters.

  Summary:
    Defines the line coding parameters.

  Description:
    This data type defines the line coding parameters.

  Remarks:
    This is internal to the CDC.
*/

USB_CDC_RS232_PARAMS serialParams;


// *****************************************************************************
/* Serial State Response.

  Summary:
    Defines the serial state response.

  Description:
    This data type defines the serial state response.

  Remarks:
    This is internal to the CDC.
*/

USB_CDC_SERIAL_STATE_RESPONSE serialState =
{
    .bmRequestType  = 0xA1,
    .bNotification  = USB_CDC_NOTIFICATION_SERIAL_STATE,
    .wValue         = 0,
    .wIndex         = 0,
    .wLength        = 2,
    .stSerial       = { 0 }
};


// *****************************************************************************
/* Response available.

  Summary:
    Defines the response available structure.

  Description:
    This data type defines the response available structure.

  Remarks:
    This is internal to the CDC.
*/
USB_CDC_SERIAL_RESPONSE_AVAILABLE response =
{
    .bmRequestType  = 0xA1,
    .bNotification  = USB_CDC_NOTIFICATION_RESPONSE_AVAILABLE,
    .wValue         = 0,
    .wIndex         = 0,
    .wLength        = 0
};


// *****************************************************************************
/* Macro: CDC_ACM

  Summary:
    Helps in creating the non-volatile memory section.

  Description:
    This macro helps in creating the non-volatile memory section.
*/

#define CDC_ACM                            __attribute__( ( section( "Cdc_Acm" ) ) )


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    void USB_DEVICE_CDC_ACMSetUpPacketHandler( USB_CDC_INSTANCE * instance,
                                               uint16_t value )

  Summary:
    Handles ACM sub class specific requests.

  Description:
    This function handles ACM sub-class specific requests.

  Remarks:
    Called by the CDC function driver.
 */

CDC_ACM void USB_DEVICE_CDC_ACMSetUpPacketHandler ( USB_CDC_INSTANCE * instance, uint16_t value )
{
    /* transfer handle */
    DRV_USB_XFER_HANDLE cdcXferHandle;
    // TODO check the bmCapabilities before servicing the request
    /* check the request */
    switch ( instance->request )
    {
        /* set line coding */
        case USB_CDC_REQUEST_SET_LINE_CODING:
            /* set the CDC EP0 state to pending */
            instance->ep0State |= USB_CDC_EP0_REQUEST_RX_PENDING;

            /* place a request to the device layer to get data from the host */
            cdcXferHandle = _USB_DEVICE_MAKE_NAME( ControlTransferRequest ) ( instance->cdcEP0OutBuffer,
                                                                              sizeof (USB_CDC_LINE_CODING ),
                                                                              USB_XFER_CONTROL_WRITE,
                                                                              instance,
                                                                              USB_DEVICE_CDC_Ep0OUTCallBack );
        break;

        /* get line coding */
        case USB_CDC_REQUEST_GET_LINE_CODING:
            /* set the CDC EP0 state to pending */
            instance->ep0State |= USB_CDC_EP0_REQUEST_TX_PENDING;

            /* copy the line coding to IN buffer */
            memcpy( instance->cdcEP0InBuffer, &serialParams.lineCoding, sizeof( USB_CDC_LINE_CODING ) );

            /* place a request to the device layer for sending the line coding
                to host */
            cdcXferHandle = _USB_DEVICE_MAKE_NAME( ControlTransferRequest ) ( instance->cdcEP0InBuffer,
                                                                            sizeof (USB_CDC_LINE_CODING ),
                                                                            USB_XFER_CONTROL_READ,
                                                                            instance,
                                                                            USB_DEVICE_CDC_Ep0INCallBack );
        break;

        /* set control line state */
        case USB_CDC_REQUEST_SET_CONTROL_LINE_STATE:
            /* set the control line state */
            _USB_CDC_MAKE_NAME( ACMSetControlLineState )( value );

            /* inform the application about this request */
            instance->appCallBack ( USB_CDC_INDEX,
                                    USB_CDC_CALLBACK_MANAGEMENT ,
                                    USB_CDC_CALLBACK_SET_CONTROL_LINE_STATE );

            /* close the request */
            instance->ep0State = USB_CDC_EP0_REQUEST_CLOSED;
        break;

        /* AT commands */
        case USB_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND:
        case USB_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE:
            /* mark the state as pending */
            instance->ep0State |= USB_CDC_EP0_REQUEST_RX_PENDING;

            /* get the length of the command */
            atCommandBuffer.length = instance->requestDataLen;

            /* place a request to device layer to get the data */
            cdcXferHandle = _USB_DEVICE_MAKE_NAME( ControlTransferRequest ) ( instance->cdcEP0OutBuffer,
                                                                              instance->requestDataLen,
                                                                              USB_XFER_CONTROL_WRITE,
                                                                              instance,
                                                                              USB_DEVICE_CDC_Ep0OUTCallBack );
        break;

        /* break request */
        case USB_CDC_REQUEST_SEND_BREAK:
            /* update the break duration */
            serialParams.breakDur = value;

            /* close the request */
            instance->ep0State = USB_CDC_EP0_REQUEST_CLOSED;

            if ( instance->appCallBack )
            {
                /* inform the application about this request */
                instance->appCallBack ( USB_CDC_INDEX,
                                        USB_CDC_CALLBACK_MANAGEMENT ,
                                        USB_CDC_CALLBACK_ISSUE_BREAK );
            }
        break;

        /* requests that do not belog to ACM sub class */
        /* GCC is upset that we are not using these enums . These statements are
         provided to escape from the GCC warnings */
        case USB_CDC_REQUEST_SET_COMM_FEATUE:
        case USB_CDC_REQUEST_GET_COMM_FEATUE:
        case USB_CDC_REQUEST_CLEAR_COMM_FEATUE:
        case USB_CDC_REQUEST_SET_AUX_LINE_STATE:
        case USB_CDC_REQUEST_SET_HOOK_STATE:
        case USB_CDC_REQUEST_PULSE_SETUP:
        case USB_CDC_REQUEST_SEND_PULSE:
        case USB_CDC_REQUEST_SET_PULSE_TIME:
        case USB_CDC_REQUEST_RING_AUX_JACK:
        case USB_CDC_REQUEST_SET_RINGER_PARMS:
        case USB_CDC_REQUEST_GET_RINGER_PARMS:
        case USB_CDC_REQUEST_SET_OPERATIONAL_PARMS:
        case USB_CDC_REQUEST_GET_OPERATIONAL_PARMS:
        case USB_CDC_REQUEST_SET_LINE_PARMS:
        case USB_CDC_REQUEST_GET_LINE_PARMS:
        case USB_CDC_REQUEST_DIAL_DIGITS:
        case USB_CDC_REQUEST_SET_UNIT_PARAMETER:
        case USB_CDC_REQUEST_GET_UNIT_PARAMETER:
        case USB_CDC_REQUEST_CLEAR_UNIT_PARAMETER:
        case USB_CDC_REQUEST_GET_PROFILE:
        case USB_CDC_REQUEST_SET_ETHERNET_MULTICAST_FILTERS:
        case USB_CDC_REQUEST_SET_ETHERNET_POWER_MANAGEMENT_FILTER:
        case USB_CDC_REQUEST_GET_ETHERNET_POWER_MANAGEMENT_FILTER:
        case USB_CDC_REQUEST_SET_ETHERNET_PACKET_FILTER:
        case USB_CDC_REQUEST_GET_ETHERNET_STATISTIC:
        case USB_CDC_REQUEST_SET_ATM_DATA_FORMAT:
        case USB_CDC_REQUEST_GET_ATM_DEVICE_STATISTICS:
        case USB_CDC_REQUEST_SET_ATM_DEFAULT_VC:
        case USB_CDC_REQUEST_GET_ATM_VC_STATISTICS:
        default:

        break;

    }

}


//******************************************************************************
/* Function:
    void USB_DEVICE_CDC_ACMSetControlLineState( USB_CDC_CLIENT_HANDLE iCDC,
                                                uint8_t lineState )

  Summary:
    Sets the control line state.

  Description:
    This function sets the control line state.

  Remarks:
    Called by the CDC function driver.
 */

CDC_ACM void _USB_CDC_MAKE_NAME( ACMSetControlLineState )( uint8_t lineState )
{
    /* copy carrier state */
    serialParams.lineState.carrier = lineState & USB_CDC_LINESTATE_CARRIER;

    /* copy data terminal ready status (host alive) */
    serialParams.lineState.dtr = lineState & USB_CDC_LINESTATE_DTR;

} /* USB_DEVICE_CDC_ACMSetControlLineState */


//******************************************************************************
/* Function:
    void USB_DEVICE_CDC_ACMSetLineCoding( USB_CDC_CLIENT_HANDLE iCDC,
                                          USB_CDC_LINE_CODING * lineCoding )

  Summary:
    Updates the line coding.

  Description:
    This function updates the line coding.

  Remarks:
    Called by the CDC function driver.
 */

CDC_ACM void _USB_CDC_MAKE_NAME( ACMSetLineCoding )( USB_CDC_LINE_CODING * lineCoding )
{
    /* copy the line coding to cdc instance structure */
    memcpy( &serialParams.lineCoding, lineCoding, sizeof( USB_CDC_LINE_CODING ) );

} /* USB_DEVICE_CDC_ACMSetLineCoding */


//******************************************************************************
/* Function:
    void USB_DEVICE_CDC_UpdateATCommand( SYS_MODULE_INDEX iCDC, void * command )

  Summary:
    Updates the AT command received.

  Description:
    This function updates the AT command received.

  Remarks:
    Called by the CDC function driver.
 */

CDC_ACM void _USB_CDC_MAKE_NAME( UpdateATCommand )( void * command )
{
    /* get the command buffer */
    uint8_t * commandBuffer = ( uint8_t * ) command;
    /* temp local */
    uint8_t i;

    /* copy length number of bytes */
    for( i = 0; i < atCommandBuffer.length; i ++ )
    {
        /* copy the at command into the CDC AT command buffer */
        atCommandBuffer.command[i] = commandBuffer[i];
    }

} /* USB_DEVICE_CDC_UpdateATCommand */

// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_ACMGetControlLineState( USB_CDC_CLIENT_HANDLE iCDC,
                                                            USB_CDC_CONTROL_LINE_STATE * state )
 
  Summary:
    Gives the control line state.

  Description:
    Upon request this function gives the control line state to the application.

  Precondition:
    None.

  Parameters:
    iCDC    - USB function driver index

    state   - User space pointer to the buffer into which the values are copied

  Returns:
    USB_ERROR_STATUS - Error status

  Example:
    <code>
   // create a line state structure
    USB_CDC_CONTROL_LINE_STATE  state;

    // put a request for latest line state values
     if(!USB_DEVICE_CDC_GetControlLineState(0, &state))
        {
            // state is updated
        }
    </code>

  Remarks:
    None.
*/

CDC_ACM USB_ERROR_STATUS _USB_CDC_MAKE_NAME( ACMGetControlLineState )( USB_CDC_CONTROL_LINE_STATE * state )
{
    /* check if the user passed buffer is valid */
    if( state )
    {
        /* copy carrier state to user space */
        state->carrier = serialParams.lineState.carrier;
        
        /* copy the dtr status to the user space */
        state->dtr = serialParams.lineState.dtr;

        /* we are good */
        return USB_ERROR_OK;
    }    
    else /* buffer passed is invalid */
    {
        /* tell user the same */
        return USB_CDC_ERROR_INVALID_BUFFER;
    }

} /* USB_DEVICE_CDC_ACMGetControlLineState */


// *****************************************************************************
/* Function:
    uint16_t USB_DEVICE_CDC_ACMGetBreak( USB_CDC_CLIENT_HANDLE iCDC )

  Summary:
    Returns the break duration obtained from the host.

  Description:
    This function returns the break duration obtained from the host.

  Precondition:
    None.

  Parameters:
    iCDC    - USB function driver index

  Returns:
    uint16_t - Break duration
 
  Example:
    <code>
        // break duration
        uint16_t breakDur;

        // put a request for break duration
        breakDur = USB_DEVICE_CDC_ACMGetBreak(0);
    </code>

  Remarks:
    None.
*/

CDC_ACM uint16_t _USB_CDC_MAKE_NAME( ACMGetBreak )( void )
{
    /* return the break duration */
    return( serialParams.breakDur );

} /* USB_DEVICE_CDC_ACMGetBreak */


// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_ACMGetLineCoding( USB_CDC_CLIENT_HANDLE iCDC,
                                                      USB_CDC_LINE_CODING* lineCoding )

  Summary:
    Gives the line coding data.

  Description:
    Upon request, this function gives the line coding to the application.

  Precondition:
    None.

  Parameters:
    iCDC        - USB function driver index

    lineCoding  - User space pointer to the buffer into which the values are copied

  Returns:
   
    USB_ERROR_STATUS - error status

  Example:
    <code>
   // create a line state structure
    USB_CDC_LINE_CODING  lineCoding;

    // put a request for latest line state values
     if(!USB_DEVICE_CDC_ACMGetLineCoding(0, &lineCoding))
        {
            // line coding is updated
        }
    </code>

  Remarks:
    None.
*/

CDC_ACM USB_ERROR_STATUS _USB_CDC_MAKE_NAME( ACMGetLineCoding )( USB_CDC_LINE_CODING * lineCoding )
{
    /* check if the application passed a valid buffer */
    if( lineCoding )
    {
        /* copy the line coding to user space buffer */
        memcpy( lineCoding, &serialParams.lineCoding, sizeof( USB_CDC_LINE_CODING ) );

        /* return suceess*/
        return USB_ERROR_OK;
    }    
    else /* invalid buffer passed*/
    {
        /* return error */
        return USB_CDC_ERROR_INVALID_BUFFER;
    }

} /* USB_DEVICE_CDC_ACMGetLineCoding */


// *****************************************************************************
/* Function:
    void USB_DEVICE_CDC_ACMSendResponseAvailableNotification( USB_CDC_CLIENT_HANDLE iCDC )

  Summary:
    Gives a request to the DC to send a response available notification to the host.

  Description:
    This function gives a request to the CDC to send a response available
    notification to the host.

  Precondition:
    None.

  Parameters:
    iCDC        - USB function driver index

  Returns:
    None.

  Example:
    <code>
        // client handle
        USB_CDC_CLIENT_HANDLE iCDC;

        // Open CDC to get proper handle
        // Application code

        //Place a request to send response available to host
        USB_DEVICE_CDC_ACMSendResponseAvailableNotification (iCDC);
    </code>

  Remarks:
    None.
*/

CDC_ACM void _USB_CDC_MAKE_NAME( ACMSendResponseAvailableNotification )( void )
{
    /* copy the interface number */
    response.wIndex = _USB_CDC_MAKE_NAME( GetNotificationInterfaceNum )();

    /* request for a notification */
    _USB_CDC_MAKE_NAME( NotificationHandler )( &response, sizeof( USB_CDC_SERIAL_RESPONSE_AVAILABLE ) );

} /* USB_DEVICE_CDC_ACMSendResponseAvailableNotification */


// *****************************************************************************
/* Function:
    void USB_DEVICE_CDC_ACMSendSerialStateNotification( USB_CDC_CLIENT_HANDLE iCDC,
                                                        USB_CDC_SERIAL_STATE * state )

  Summary:
    Gives a request to the CDC to send a serial state notification to the host.

  Description:
    This function gives a request to the CDC to send a serial state
    notification to host.

  Precondition:
    None.

  Parameters:
    iCDC    - USB function driver index
    state   - Pointer to the serial state data

  Returns:
    None.

  Example:
    <code>
        // client handle
        USB_CDC_CLIENT_HANDLE iCDC;
        USB_CDC_SERIAL_STATE state =
            {
                .bBreak = 0,
                .bFraming = 0,
                .bOverRun =0,
                .bParity = 0,
                .bRingSignal = 0,
                .bRxCarrier = 0,
                .bTxCarrier = 0
            };

        // Open CDC to get proper handle
        // Application code

        // Update serial state based on UART data
        //Place a request to send serial state to host
        USB_DEVICE_CDC_ACMSendSerialStateNotification ( iCDC, &state);
    </code>

  Remarks:
    None.
*/

CDC_ACM void _USB_CDC_MAKE_NAME( ACMSendSerialStateNotification )( USB_CDC_SERIAL_STATE * state )
{
    /* update the serial state with the data sent by the application */
    memcpy( &serialState.stSerial, state, 1 );
    
    /* set the interface number */
    serialState.wIndex = _USB_CDC_MAKE_NAME( GetNotificationInterfaceNum )();

    /* request for a notification */
    _USB_CDC_MAKE_NAME( NotificationHandler )( &serialState, sizeof( USB_CDC_SERIAL_STATE_RESPONSE ) );

} /* USB_DEVICE_CDC_ACMSendSerialStateNotification */


// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_QueryATCommand( USB_CDC_CLIENT_HANDLE iCDC,
                                                    USB_CDC_AT_COMMAND* cBuffer )

  Summary:
    Gets the latest AT command received from the host.

  Description:
    This function gets the latest AT command received from the host.

  Precondition:
    None.

  Parameters:
    iCDC    - USB function driver index
    cBuffer - Destination user space buffer

  Returns:

    USB_ERROR_STATUS - Error status

  Example:
    <code>
        // client handle
        USB_CDC_CLIENT_HANDLE iCDC;
        USB_CDC_AT_COMMAND cBuffer;

        // Open CDC to get proper handle
        // Application code

        // Update serial state based on UART data
        //Place a request to send serial state to host
        if(!USB_DEVICE_CDC_QueryATCommand ( iCDC,&cBuffer ) )
            {
                //AT command is updated
            }
    </code>

  Remarks:
    None.
*/

CDC_ACM USB_ERROR_STATUS _USB_CDC_MAKE_NAME( QueryATCommand )( USB_CDC_AT_COMMAND* cBuffer )
{
    /* check if the application passed a valid buffer */
    if( cBuffer )
    {
        /* copy the line coding to user space buffer */
        memcpy( cBuffer, &atCommandBuffer, sizeof( USB_CDC_AT_COMMAND ) );

        /* return suceess*/
        return USB_ERROR_OK;
    }    
    else /* invalid buffer passed*/
    {
        /* return error */
        return USB_CDC_ERROR_INVALID_BUFFER;
    }

} /* USB_DEVICE_CDC_QueryATCommand */


/*******************************************************************************
 End of File
 */


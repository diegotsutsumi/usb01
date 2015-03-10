/*******************************************************************************
 USB CDC class function driver for single static instance

 Company:
    Microchip Technology Inc.

 File Name:
    usb_device_cdc_static.c

 Summary:
    USB CDC class function driver

 Description:
    USB CDC class function driver for a single static instance.
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

#include "usb/usb_device_cdc.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* CDC Instance structure

  Summary:
    Defines the CDC instance(s).

  Description:
    This data type defines the CDC instance(s). The number of instances is
    defined by the application using USB_CDC_MAX_INSTANCES

  Remarks:
    This structure is private to the CDC.
 */

USB_CDC_INSTANCE                                cdcInstance;

extern USB_CDC_RS232_PARAMS                     serialParams;

extern uint8_t                                  atCommand[2];

extern USB_CDC_AT_COMMAND                       atCommandBuffer;


// *****************************************************************************
/* Notification buffer object

  Summary:
    Contains the notification details.

  Description:
    This data type contains the notification details.

  Remarks:
    None.
 */

USB_CDC_NOTIFICATION_OBJ                        nObject;


// *****************************************************************************
/* Macro: CDC

  Summary:
    Helps in creating the non-volatile memory section.

  Description:
    This macro helps in creating the non-volatile memory section.
*/

#define CDC                                     __attribute__( ( section( "Cdc" ) ) )


// *****************************************************************************
/* CDC Device function driver structure

  Summary:
    Defines the function driver structure required for the device layer.

  Description:
    This data type defines the function driver structure required for the
    device layer.

  Remarks:
    This structure is private to the USB stack.
 */

USB_DEVICE_FUNCTION_DRIVER cdcFuncDriver = 
{
    /* CDC init function */
    .initialize                 = USB_DEVICE_CDC0_Initialization ,

    /* CDC de-init function */
    .deInitialize               = USB_DEVICE_CDC0_Deinitialization ,

    /* CDC check interface function */
    .checkInterfaceNumber       = USB_DEVICE_CDC0_CheckInterface ,

    /* This function helps the device layer to forward the class specific setup packets
    to right function driver. The device layer calls this function when class sepcific
    setup packet arrives. If a function driver claims the endpoint number by returning
    "USB_ERROR_OK", then the setup packet is forwarded to the corresponding function driver
    only. */
    .checkEndpointNumber        = NULL ,

    /* CDC set-up packet handler */
    .setupPacketHandler         = USB_DEVICE_CDC0_SetUpPacketHandler ,

    /* CDC tasks function */
    .tasks                      = USB_DEVICE_CDC0_Tasks
};


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************

/* Function:
    uint8_t USB_DEVICE_CDC_GetNotificationInterfaceNum( USB_CDC_CLIENT_HANDLE iCDC )

  Summary:
    Returns the notification interface number.

  Description:
    This function returns the notification interface number.

  Remarks:
    None.
 */

CDC uint8_t _USB_CDC_MAKE_NAME( GetNotificationInterfaceNum )( void )
{
    return( cdcInstance.nInf.interfaceNum );

} /* USB_DEVICE_CDC_GetNotificationInterfaceNum */


//******************************************************************************

/* Function:
    void USB_DEVICE_CDC_NotificationHandler( USB_CDC_CLIENT_HANDLE cHandle,
                                             void* data,
                                             uint8_t length )

  Summary:
    Handles notification requests.

  Description:
    This function handles notification requests.

  Remarks:
    Called by the CDC driver.
 */

CDC void _USB_CDC_MAKE_NAME( NotificationHandler )( void * data, uint8_t length )
{
    /* Place an IN request with the given notification details */
    cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].cdxferHandle =
        DRV_USB_Device_TransferRequest ( cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle ,
                                         USB_XFER_INTERRUPT_READ ,
                                         data ,
                                         length ,
                                         USB_XFER_REGULAR ,
                                         &cdcInstance ,
                                         USB_DEVICE_CDC_TxNotificationCallBack );

    /* Check if controller driver accepted the request */
    if( DRV_HANDLE_INVALID == cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].cdxferHandle )
    {
        /* We are not checking why it failed because all the cases where it could fail
         * are already checked in the CDC layer. The only possibility of error here is the
         * non availability of slot in the controller driver's Q.*/

        /* copy all the required data. This notification is sent in the CDC
         tasks function */
        /* copy the data buffer */
        nObject.nBuffer = data;

        /* copy length of the data */
        nObject.len = length;

        /* copy the client handle */
        nObject.cHandle = USB_CDC_INDEX;

        /* mark that notification needs to be sent */
        nObject.inUse = true;
    }

} /* USB_DEVICE_CDC_NotificationHandler */


//******************************************************************************

/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_CheckInterface( SYS_MODULE_INDEX funcDriverIndex ,
                                                    uint16_t interfaceNumber )

  Summary:
    Checks if the specified interface number is handled by the CDC.

  Description:
    This function checks if the specified interface number is handled by CDC.

  Remarks:
    Called by the USB device layer.
 */

CDC USB_ERROR_STATUS _USB_CDC_MAKE_NAME( CheckInterface )( uint16_t interfaceNumber )
{
    /* Compare the interface number with the one the current CDC instance has got*/
    if( ( cdcInstance.dInf.interfaceNum == interfaceNumber ) || ( cdcInstance.nInf.interfaceNum == interfaceNumber ) )
    {
        /* we got a match. Return and inform the device layer */
        return( USB_ERROR_OK );
    }
    else /* No match */
    {
        return( USB_ERROR_INVALID );
    }

} /* USB_DEVICE_CDC_CheckInterface */


//******************************************************************************
/* Function:
    void USB_DEVICE_CDC_SetUpPacketHandler( SYS_MODULE_INDEX iCDC,
                                            SETUP_PKT *setupPacketBuff )

  Summary:
    Handles the CDC specific management requests.

  Description:
    This function handles the CDC specific management requests.

  Remarks:
    Called by the USB device layer.
*/

CDC void _USB_CDC_MAKE_NAME( SetUpPacketHandler )( SETUP_PKT *setupPacketBuff )
{
    /* Check if the given request is CDC specific */
    if( ! ( setupPacketBuff->bmRequestType & USB_CDC_REQUEST_CLASS_SPECIFIC ) )
    {
        /* return error to the device layer*/
        // TODO: insert stall request
    }        
    else /* The request can be handled by the CDC device */
    {
        /* Copy the requested data length (if there is a data stage) */
        cdcInstance.requestDataLen = ( unsigned short int ) setupPacketBuff->W_Length.Val;
        /* Copy the request issued by the host */
        cdcInstance.request = ( unsigned short int ) setupPacketBuff->bRequest;

        /* Switch on request received */
        switch( setupPacketBuff->bRequest )
        {
            /* Check if the requests belong to ACM sub-class */
            case USB_CDC_REQUEST_SET_LINE_CODING:
            case USB_CDC_REQUEST_GET_LINE_CODING:
            case USB_CDC_REQUEST_SET_CONTROL_LINE_STATE:
            case USB_CDC_REQUEST_SEND_BREAK:
            case USB_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND:
            case USB_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE:
                /* Call the ACM sub-class specific setup packet handler */
                USB_DEVICE_CDC_ACMSetUpPacketHandler( &cdcInstance, setupPacketBuff->W_Value.Val );
            break;

                /* request cannot be handled */
            default:
                /* return error to the device layer */
                cdcInstance.state = USB_CDC_REQUEST_NONE;
            break;
        }
    }

} /* USB_DEVICE_CDC_SetUpPacketHandler */


//******************************************************************************
/* Function:
    void USB_DEVICE_CDC_Ep0INCallBack( void * referenceData,
                                       DRV_USB_PIPE_HANDLE hPipe,
                                       DRV_USB_XFER_HANDLE hTransfer,
                                       unsigned short int transferByteCount,
                                       DRV_USB_DEVICE_XFER_STATUS statusTransfer )

  Summary:
    Callback for the IN transfers placed on Endpoint 0

  Description:
    This function is the callback for the IN transfers placed on Endpoint 0.

  Remarks:
    Called by the USB Controller driver.
*/

CDC void USB_DEVICE_CDC_Ep0INCallBack( void * referenceData ,
                                   DRV_USB_PIPE_HANDLE hPipe ,
                                   DRV_USB_XFER_HANDLE hTransfer ,
                                   unsigned short int transferByteCount ,
                                   DRV_USB_DEVICE_XFER_STATUS statusTransfer )
{
    /* Get the instance pointer from the reference data */
    USB_CDC_INSTANCE * iCDC = ( USB_CDC_INSTANCE * ) referenceData;

    /* Check if the transfer is complete */
    if( statusTransfer == USB_XFER_COMPLETED )
    {
        /* Clear the request */
        iCDC->request = USB_CDC_REQUEST_NONE;

        /* Set the CDC EP0 state to request closed */
        iCDC->ep0State = USB_CDC_EP0_REQUEST_CLOSED;
    }

} /* USB_DEVICE_CDC_Ep0INCallBack */


//******************************************************************************
/* Function:
   void USB_DEVICE_CDC_Ep0OUTCallBack( void * referenceData,
                                       DRV_USB_PIPE_HANDLE hPipe,
                                       DRV_USB_XFER_HANDLE hTransfer,
                                       unsigned short int transferByteCount,
                                       DRV_USB_DEVICE_XFER_STATUS statusTransfer )

  Summary:
    Callback for the OUT transfers placed on Endpoint 0.

  Description:
    This function is the callback for the OUT transfers placed on Endpoint 0.

  Remarks:
    Called by the USB Controller driver.
*/

CDC void USB_DEVICE_CDC_Ep0OUTCallBack( void * referenceData ,
                                    DRV_USB_PIPE_HANDLE hPipe ,
                                    DRV_USB_XFER_HANDLE hTransfer ,
                                    unsigned short int transferByteCount ,
                                    DRV_USB_DEVICE_XFER_STATUS statusTransfer )
{
    /* Get the instance pointer from the reference data */
    USB_CDC_INSTANCE *iCDC = ( USB_CDC_INSTANCE * ) referenceData;
    /* count for total received data */
    static unsigned short int rxDataLen;

    /* Check if the transfer is complete */
    if( statusTransfer == USB_XFER_COMPLETED )
    {
        /* Check the request pending */
        switch( iCDC->request )
        {
            case USB_CDC_REQUEST_SET_LINE_CODING:
                /* Set the line coding as per the data issued by the host */
                _USB_CDC_MAKE_NAME( ACMSetLineCoding )( ( USB_CDC_LINE_CODING* ) iCDC->cdcEP0OutBuffer );

                if ( iCDC->appCallBack )
                {
                    /* Inform the application that there is a change in the line coding */
                    iCDC->appCallBack ( USB_CDC_INDEX,
                                        USB_CDC_CALLBACK_MANAGEMENT ,
                                        USB_CDC_CALLBACK_SET_LINE_CODING );
                }
            break;

            case USB_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND:
                /* Update the AT command received */
                _USB_CDC_MAKE_NAME( UpdateATCommand )( iCDC->cdcEP0OutBuffer );

                if ( iCDC->appCallBack )
                {
                    /* Inform the application that an AT command is received
                     and the application needs to act accordingly */
                    iCDC->appCallBack ( USB_CDC_INDEX,
                                        USB_CDC_CALLBACK_MANAGEMENT ,
                                        USB_CDC_CALLBACK_SEND_ENCAPSULATED_COMMAND );
                }
            break;

            case USB_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE:
                /* Update the AT command received */
                _USB_CDC_MAKE_NAME( UpdateATCommand )( iCDC->cdcEP0OutBuffer );

                if ( iCDC->appCallBack )
                {
                    /* Inform the application that an AT command is received
                     and the application needs to respond for the AT Command */
                    iCDC->appCallBack ( USB_CDC_INDEX,
                                        USB_CDC_CALLBACK_MANAGEMENT ,
                                        USB_CDC_CALLBACK_GET_ENCAPSULATED_RESPONSE );
                }
            break;

            default:
            break;
        }

        /* accumulate the total bytes received */
        rxDataLen += transferByteCount;

        /* Check if we got the entire data from the host */
        if ( rxDataLen == iCDC->requestDataLen )
        {
            /* We got all the required data. Reset the data count */
            rxDataLen = 0;

            /* Clear the request */
            iCDC->request = USB_CDC_REQUEST_NONE;

            /* Set the CDC EP0 state to request closed */
            iCDC->ep0State = USB_CDC_EP0_REQUEST_CLOSED;

        }
    }

} /* USB_DEVICE_CDC_Ep0OUTCallBack */


//******************************************************************************
/* Function:
  void USB_DEVICE_CDC_TxDataCallBack( void * hClient,
                                      DRV_USB_PIPE_HANDLE hPipe,
                                      DRV_USB_XFER_HANDLE hTransfer,
                                      unsigned short int transferByteCount,
                                      DRV_USB_DEVICE_XFER_STATUS statusTransfer )

  Summary:
    Callback for the IN data transfers on the data endpoint.

  Description:
    This routine is the callback for the IN data transfers on the data endpoint.

  Remarks:
    Called by the USB controller driver.
*/

CDC void USB_DEVICE_CDC_TxDataCallBack ( void * hClient ,
                                     DRV_USB_PIPE_HANDLE hPipe ,
                                     DRV_USB_XFER_HANDLE hTransfer ,
                                     unsigned short int transferByteCount ,
                                     DRV_USB_DEVICE_XFER_STATUS statusTransfer )
{
    /* Get the instance structure */
    USB_CDC_INSTANCE * instance = ( USB_CDC_INSTANCE * ) hClient;

    /* Check the status of the transfer */
    switch( statusTransfer )
    {
        /* Transfer is complete */
        case USB_XFER_COMPLETED:
            /* Update the transfered byte count in the buffer object given by 
             the application during the write request. This represents number of
             bytes successfully transferred */
            instance->dInf.ep[USB_CDC_EP_IN_INDEX].bufferObject->transferCount = transferByteCount;
            /* CDC is ready to accept new requests on IN data EP */
            instance->dInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_TX_READY;

            if( instance->appCallBack )
            {
                /* Inform the application */
                instance->appCallBack( USB_CDC_INDEX,
                                       USB_CDC_CALLBACK_TX ,
                                       USB_CDC_CALLBACK_TX_COMPLETE );
            }
        break;

        /* Transfer is not through */
        case USB_XFER_ABORTED:
            /* CDC is ready to accept new requests on IN data EP */
            instance->dInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_TX_READY;

            if( instance->appCallBack )
            {
                //TODO: check if abort and error are same for application
                /* Inform the application */
                instance->appCallBack( USB_CDC_INDEX,
                                       USB_CDC_CALLBACK_TX ,
                                       USB_CDC_CALLBACK_TX_ABORT );
            }
        break;

            /* other cases */
        case USB_XFER_WRONG_PIPE:
        case USB_XFER_BAD_HANDLE:
        case USB_XFER_BAD_PIPE:
        case USB_XFER_QUEUE_FULL:
        case USB_XFER_HANDLE_NOTINUSE:
        case USB_XFER_PENDING:
        case USB_XFER_IN_PROGRESS:
        default:
        break;
    }

} /* USB_DEVICE_CDC_TxDataCallBack */


//******************************************************************************

/* Function:
 void USB_DEVICE_CDC_RxDataCallBack( void * hClient,
                                     DRV_USB_PIPE_HANDLE hPipe,
                                     DRV_USB_XFER_HANDLE hTransfer,
                                     unsigned short int transferByteCount,
                                     DRV_USB_DEVICE_XFER_STATUS statusTransfer )

  Summary:
    Callback for the OUT data transfers on the data endpoint.

  Description:
    This routine is the callback for the OUT data transfers on the data endpoint.

  Remarks:
    Called by the USB controller driver.
*/

CDC void USB_DEVICE_CDC_RxDataCallBack( void * hClient ,
                                    DRV_USB_PIPE_HANDLE hPipe ,
                                    DRV_USB_XFER_HANDLE hTransfer ,
                                    unsigned short int transferByteCount ,
                                    DRV_USB_DEVICE_XFER_STATUS statusTransfer )
{
    /* Get the instance structure */
    USB_CDC_INSTANCE * instance = ( USB_CDC_INSTANCE * ) hClient;

    /* Check the status of the transfer */
    switch( statusTransfer )
    {
        /* Transfer is complete */
        case USB_XFER_COMPLETED:
            /* Update the transfered byte count in the buffer object given by
             the application during the write request. This represents number of
             bytes successfully transferred */
            instance->dInf.ep[USB_CDC_EP_OUT_INDEX].bufferObject->transferCount = transferByteCount;
            /* CDC is ready to accept new requests on IN data EP */
            instance->dInf.ep[USB_CDC_EP_OUT_INDEX].dataState = USB_CDC_INSTANCE_RX_READY;

            if ( instance->appCallBack )
            {
                /* Inform the application */
                instance->appCallBack ( USB_CDC_INDEX,
                                        USB_CDC_CALLBACK_RX ,
                                        USB_CDC_CALLBACK_RX_COMPLETE );
            }
        break;

        /* Transfer is not through */
        case USB_XFER_ABORTED:
            /* CDC is ready to accept new requests on IN data EP */
            instance->dInf.ep[USB_CDC_EP_OUT_INDEX].dataState = USB_CDC_INSTANCE_RX_READY;

            if ( instance->appCallBack )
            {
                //TODO: check if abort and error are same for application
                /* Inform the application */
                instance->appCallBack ( USB_CDC_INDEX,
                                        USB_CDC_CALLBACK_RX ,
                                        USB_CDC_CALLBACK_RX_ABORT );
            }
        break;

            /* other cases */
        case USB_XFER_WRONG_PIPE:
        case USB_XFER_BAD_HANDLE:
        case USB_XFER_BAD_PIPE:
        case USB_XFER_QUEUE_FULL:
        case USB_XFER_HANDLE_NOTINUSE:
        case USB_XFER_PENDING:
        case USB_XFER_IN_PROGRESS:
        default:
        break;
    }

} /* USB_DEVICE_CDC_RxDataCallBack */


//******************************************************************************
/* Function:
    void USB_DEVICE_CDC_TxNotificationCallBack( void * hClient,
                                                DRV_USB_PIPE_HANDLE hPipe,
                                                DRV_USB_XFER_HANDLE hTransfer,
                                                unsigned short int transferByteCount,
                                                DRV_USB_DEVICE_XFER_STATUS statusTransfer )

  Summary:
    Callback for the IN notification transfers on the notification (interrupt) 
    endpoint.

  Description:
    This routine is the callback for the IN notification transfers on the
    notification (interrupt) endpoint.

  Remarks:
    Called by the USB Controller driver.
*/

CDC void USB_DEVICE_CDC_TxNotificationCallBack( void * hClient ,
                                            DRV_USB_PIPE_HANDLE hPipe ,
                                            DRV_USB_XFER_HANDLE hTransfer ,
                                            unsigned short int transferByteCount ,
                                            DRV_USB_DEVICE_XFER_STATUS statusTransfer )
{
    /* Get the instance structure */
    USB_CDC_INSTANCE * instance = ( USB_CDC_INSTANCE * ) hClient;

    /* reset the inuse flag (set if controller driver gives Q full for any
       notification request sent by CDC. This rejected notification is 
       re-tried in the CDC tasks function )*/
    if( nObject.inUse )
        nObject.inUse = false;

    /* Check the status of the transfer */
    switch( statusTransfer )
    {
        /* Transfer is complete */
        case USB_XFER_COMPLETED:
            /* CDC is ready to send new notification on interrupt IN EP */
            instance->nInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_TX_READY;

            if ( instance->appCallBack )
            {
                /* Inform the application */
                instance->appCallBack ( USB_CDC_INDEX,
                                        USB_CDC_CALLBACK_MANAGEMENT ,
                                        USB_CDC_CALLBACK_SERIAL_STATE_NOTIFICATION_SENT );
            }
        break;

        /* Transfer is not through */
        case USB_XFER_ABORTED:
            /* CDC is ready to send new notification on interrupt IN EP */
            instance->nInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_TX_READY;


            if ( instance->appCallBack )
            {
                //TODO: check if abort and error are same for application
                /* Inform application the sad news */
                instance->appCallBack ( USB_CDC_INDEX,
                                        USB_CDC_CALLBACK_MANAGEMENT ,
                                        USB_CDC_CALLBACK_RETRY );
            }
        break;

        /* other cases */
        case USB_XFER_WRONG_PIPE:
        case USB_XFER_BAD_HANDLE:
        case USB_XFER_BAD_PIPE:
        case USB_XFER_QUEUE_FULL:
        case USB_XFER_HANDLE_NOTINUSE:
        case USB_XFER_PENDING:
        case USB_XFER_IN_PROGRESS:
        default:
        break;
    }

} /* USB_DEVICE_CDC_TxNotificationCallBack */


// *****************************************************************************
// *****************************************************************************
// Section: CDC Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_Initialization( SYS_MODULE_INDEX funcDriverIndex,
                                                    SYS_MODULE_INDEX usbCdIndex,
                                                    SYS_MODULE_INDEX usbDevIndex,
                                                    void* funcDriverInit ,
                                                    uint8_t* pConfigDesc )

  Summary:
    Initializes the CDC function driver.

  Description:
    This function initializes the specified instance of the CDC function driver. 
    This function is called by the USB device layer.

  Precondition:
    None.

  Parameters:
    funcDriverIndex	- USB function driver index
    usbCdIndex		- USB Controller driver index
    usbDevIndex         - USB device layer index
    funcDriverInit 	- pointer to the function driver initialization structure
    pConfigDesc         - Pointer to the active configuration descriptor

  Returns:
    USB_ERROR_STATUS	- Status code of the return value

  Example:
    <code>
    // Called by the device layer.
    </code>

  Remarks:
    This function is internal to the USB stack. This API should not be
    called explicitely.
*/

CDC USB_ERROR_STATUS _USB_CDC_MAKE_NAME( Initialization )( void * funcDriverInit , uint8_t * pConfigDesc )
{
    /* local counters */
    uint8_t i , j;

    static uint8_t  * pInfDesc  = NULL;
    uint8_t         * pEpDesc   = NULL;
    int8_t          index       = - 1;

    /* Init structure passed by the device layer */
    USB_CDC_INIT * funcDrInit = funcDriverInit;

    /* Error status to be returned to the device layer */
    USB_ERROR_STATUS cdcErrCode = USB_ERROR_OK;

    for ( i = 0; i < 2; i ++ )
    {
        pInfDesc = USB_DEVICE_SERVICE_GetDescriptor ( pConfigDesc , USB_DESCRIPTOR_INTERFACE , ( pInfDesc ) );

        if ( ( ( USB_INTERFACE_DESCRIPTOR* ) pInfDesc )->bInterfaceClass == USB_CDC_COMM_INF_CLASS_CODE )
        {
            cdcInstance.nInf.interfaceNum = ( ( USB_INTERFACE_DESCRIPTOR* ) pInfDesc )->bInterfaceNumber;

            pEpDesc = USB_DEVICE_SERVICE_GetDescriptor ( pConfigDesc , USB_DESCRIPTOR_ENDPOINT , pInfDesc );

            cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].epAddr = ( ( USB_ENDPOINT_DESCRIPTOR* ) pEpDesc )->bEndpointAddress;

            cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].epMaxPacketSize = ( ( USB_ENDPOINT_DESCRIPTOR* ) pEpDesc )->wMaxPacketSize;

        }

        if ( ( ( USB_INTERFACE_DESCRIPTOR* ) pInfDesc )->bInterfaceClass == USB_CDC_DATA_INF_CLASS_CODE )
        {
            cdcInstance.dInf.interfaceNum = ( ( USB_INTERFACE_DESCRIPTOR* ) pInfDesc )->bInterfaceNumber;

            for ( j = 0; j < 2; j ++ )
            {
                if ( j == 0 )
                {
                    pEpDesc = pInfDesc;
                }
                
                pEpDesc = USB_DEVICE_SERVICE_GetDescriptor ( pConfigDesc , USB_DESCRIPTOR_ENDPOINT , pEpDesc );


                if ( ( ( USB_ENDPOINT_DESCRIPTOR* ) pEpDesc )->bEndpointAddress & ( 0x80 ) )
                {
                    index = USB_CDC_EP_IN_INDEX;
                }

                else
                {
                    index = USB_CDC_EP_OUT_INDEX;
                }

                cdcInstance.dInf.ep[index].epAddr = ( ( USB_ENDPOINT_DESCRIPTOR* ) pEpDesc )->bEndpointAddress;

                cdcInstance.dInf.ep[index].epMaxPacketSize = ( ( USB_ENDPOINT_DESCRIPTOR* ) pEpDesc )->wMaxPacketSize;
            }
        }
    }

    /* Copy the controller driver index into the CDC instance structure */
    cdcInstance.drvCdIndex  = DRV_USB_INDEX;
    /* Copy the usb device layer index into the CDC instance structure */
    cdcInstance.devIndex    = USB_DEVICE_INDEX;
    /* store the CDC index for quick access in future */
    cdcInstance.cdcIndex    = USB_CDC_INDEX;

    /* mark CDC as initialized */
    cdcInstance.state = USB_CDC_INSTANCE_INITIALIZED;

    /* Copy the init structure passed by the device layer (given by application) */
    cdcInstance.init = funcDrInit;

    /* set the default line coding parameters */
    /* number of stop bits */
    /* Copy the default line parameters given by the application */
    memcpy ( &serialParams.lineCoding, funcDrInit->lineCoding, sizeof( USB_CDC_LINE_CODING ) );

    cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_TX_PROCESSING;
    cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].dataState = USB_CDC_INSTANCE_RX_PROCESSING;

    /* open controller driver */
    cdcInstance.cdHandle = DRV_USB_Open( cdcInstance.drvCdIndex, ( DRV_IO_INTENT_NONBLOCKING | DRV_IO_INTENT_READWRITE ) );

    /* open the data pipes */
    //TODO: keep a check if we need to create the pipes (if incase there is no data and/or notification interfaces
    cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle =
        DRV_USB_Device_PipeSetup( cdcInstance.cdHandle,
                                  ( ( cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].epAddr ) & ( 0x0f ) ) ,
                                  USB_EP_TX ,
                                  cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].epMaxPacketSize ,
                                  USB_CDC_EP_MAX_Q_SIZE ,
                                  USB_BULK_PIPE );

    /* Assert on invalid pipe handle */
    SYS_ASSERT ( DRV_HANDLE_INVALID != cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle ,
                 "Invalid Pipe Handle for data IN EP" );

    cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].pipeHandle =
        DRV_USB_Device_PipeSetup( cdcInstance.cdHandle,
                                  ( ( cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].epAddr ) & ( 0x0f ) ) ,
                                  USB_EP_RX ,
                                  cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].epMaxPacketSize ,
                                  USB_CDC_EP_MAX_Q_SIZE ,
                                  USB_BULK_PIPE );

    /* Assert on invalid pipe handle */
    SYS_ASSERT ( DRV_HANDLE_INVALID != cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].pipeHandle ,
                 "Invalid Pipe Handle for data OUT EP" );

    /* Open the notification pipe */
    cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle =
        DRV_USB_Device_PipeSetup( cdcInstance.cdHandle,
                                  ( ( cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].epAddr ) & ( 0x0f ) ) ,
                                  USB_EP_TX ,
                                  cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].epMaxPacketSize ,
                                  USB_CDC_EP_MAX_Q_SIZE ,
                                  USB_INTERRUPT_PIPE );

    /* Assert on invalid pipe handle */
    SYS_ASSERT ( DRV_HANDLE_INVALID != cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle ,
                 "Invalid Pipe Handle for Notification IN EP" );

    /* Return the status to the device layer */
    return (cdcErrCode );

} /* USB_DEVICE_CDC_Initialization */


// *****************************************************************************
/* Function:
    void USB_DEVICE_CDC_Deinitialization( SYS_MODULE_INDEX iCDC )

  Summary:
    CDC function driver deinitialization.

  Description:
    This function deinitializes the specified instance of the CDC function driver. 
    This function is called by the USB device layer.

  Precondition:
    None.

  Parameters:
    iCDC	- USB function driver index

  Returns:
    None.

  Example:
    <code>
    // Called by the device layer.
    </code>

  Remarks:
    This function is internal to the USB stack. This API should not be
    called explicitely.
*/

CDC void _USB_CDC_MAKE_NAME( Deinitialization )( void )
{
    /* Mark the state of the function driver as not initialized */
    cdcInstance.state = USB_CDC_INSTANCE_NOT_INITIALIZED;

    /* invalidate BULK IN EP state so that we don't accept any write requests further
       untill the CDC is initialized again */
    cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_INVALID;

    /* invalidate BULK OUT EP state so that we don't accept any read requests further
       untill the CDC is initialized again */
    cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].dataState = USB_CDC_INSTANCE_INVALID;

    /* invalidate INTERRUPT IN EP state so that we don't accept any notification requests further
       untill the CDC is initialized again */
    cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_INVALID;

    /* close the bulk IN pipe */
    DRV_USB_Device_PipeRelease ( cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle );

    /* close the bulk OUT pipe */
    DRV_USB_Device_PipeRelease ( cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].pipeHandle );

    /* close the Interrupt IN pipe */
    DRV_USB_Device_PipeRelease ( cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle );

    /* invalidate the application callback */
    cdcInstance.appCallBack = NULL;

    /* close the controller driver instance */
    DRV_USB_Close();

} /* USB_DEVICE_CDC_Deinitialization */


// *****************************************************************************
/* Function:
    USB_CDC_CLIENT_HANDLE USB_DEVICE_CDC_Open( const SYS_MODULE_INDEX iCDC )

  Summary:
    USB CDC function driver instance open.

  Description:
    This function creates a CDC function driver instance.

  Precondition:
    The CDC function driver instance should have been initialized.

  Parameters:
    iCDC	- USB function driver index

  Returns:
    USB_CDC_CLIENT_HANDLE	- A unique integer to identify the client

  Example:
    <code>
    // Client Handle
    USB_CDC_CLIENT_HANDLE cdcHandle;

    // open CDC function driver instance 0
    cdcHandle = USB_DEVICE_CDC_Open ( 0 );

    // Check the validit of the handle returned
    if(USB_CDC_CLIENT_HANDLE_INVALID == cdcHandle)
    {
        \\ Handle the error
    }
    </code>

  Remarks:
    None.
*/

CDC void _USB_CDC_MAKE_NAME( Open )( void )
{
    /* Check if CDC instance is initialized or closed */
    if ( ( cdcInstance.state == USB_CDC_INSTANCE_INITIALIZED ) ||
         ( cdcInstance.state == USB_CDC_INSTANCE_CLOSED ) )
    {
        /* Mark the instance as opened */
        cdcInstance.state = USB_CDC_INSTANCE_OPENED;

        /* Make the data IN endpoint ready for handling the transfers */
        cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_TX_READY;

        /* Make the data OUT endpoint ready for handling the transfers */
        cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].dataState = USB_CDC_INSTANCE_RX_READY;

    }
    else /* The function driver is not yet initialized or already opened */
    {
        /* Do Nothing */
    }

} /* USB_DEVICE_CDC_Open */


// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_Close( USB_CDC_CLIENT_HANDLE iCDC )

  Summary:
    USB CDC function driver instance close.

  Description:
    This function closes an opened CDC function driver instance.

  Precondition:
    The CDC function instance should have been opened.

  Parameters:
    funcDriverIndex	- USB function driver index

  Returns:
    USB_ERROR_STATUS	- Return status

  Example:
    <code>
    // Return status
    USB_ERROR_STATUS status;

    // Close CDC instance 1
    status = USB_DEVICE_CDC_Close(1);

    if(USB_ERROR_INVALID_HANDLE =status)
    {

        // CDC function driver instance 0 is either not opened/initialized yet

    }

    else if(status == USB_ERROR_OK)
    {
        // CDC function driver instance 0 is successfully closed.
        // app code
    }

    </code>

  Remarks:
    None.
*/

CDC USB_ERROR_STATUS _USB_CDC_MAKE_NAME( Close )( void )
{
    // TODO: check the validity of the handle passed
    /* Check if the instance is opened */
    if ( cdcInstance.state == USB_CDC_INSTANCE_OPENED )
    {
        /* The instace is already opened. We can close it without any worries */
        cdcInstance.state = USB_CDC_INSTANCE_CLOSED;

        /* return succes */
        return USB_ERROR_OK;
    }
        // TODO: check if there are any pending transfers. If so cancel these pending transfers
        /* the given instance cannot be closed */
    else
    {
        /* return error */
        return USB_ERROR_INVALID_HANDLE;
    }

} /* USB_DEVICE_CDC_Close */


// *****************************************************************************
/* Function:
    void USB_DEVICE_CDC_Tasks( SYS_MODULE_INDEX funcDriverIndex )

  Summary:
    USB CDC function driver tasks routine.

  Description:
    This function manages the CDC function driver state and does housekeeping.

  Precondition:
    Enumeration should be completed before calling the CDC tasks function.

  Parameters:
    funcDriverIndex	- USB function driver index

  Returns:
    None.

  Example:
    <code>
    // Called by the device layer.
    </code>

  Remarks:
    This function is internal to the USB stack. This API should not be
    called explicitely.
*/

CDC void _USB_CDC_MAKE_NAME( Tasks )( void )
{
    /* check if the notification buffer needs to be processed */
    if( nObject.inUse )
    {
        /* Place an IN request with the given notification details */
        cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].cdxferHandle =
            DRV_USB_Device_TransferRequest ( cdcInstance.nInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle ,
                                             USB_XFER_INTERRUPT_READ ,
                                             nObject.nBuffer ,
                                             nObject.len ,
                                             USB_XFER_REGULAR ,
                                             &cdcInstance ,
                                             USB_DEVICE_CDC_TxNotificationCallBack );
    }

    /* we dont see if the controller driver accepted this request or not. Because
     this is the second attempt in sending this notification. We hardly got any traffic
     on this interrupt EP. If we cannot push things in two attempts something
     wrong with the design */

} /* USB_DEVICE_CDC_Tasks */


// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_Read( USB_CDC_CLIENT_HANDLE cHandle ,
                                          USB_CDC_DATA_BUFFER_OBJECT* bufferObj )

  Summary:
    Read request to CDC function driver.

  Description:
    This function reads data from the host. The data direction is from the host to 
    the device.

  Precondition:
    None.

  Parameters:
    cHandle     - CDC client handle

    bufferObj   - Pointer to buffer object

  Returns:
    USB_ERROR_STATUS   - return status of type USB_ERROR_STATUS

  Example:
    <code>
    // Application callback to handle relevant events.
    void app_cdc_CallBack ( SYS_MODULE_INDEX funcDriverIndex,
                            USB_CDC_CALLBACK_TYPE callback,
                            USB_CDC_CALLBACK_EVENT event)
    {
        switch (callback)
        {
            case USB_CDC_CALLBACK_TX:
                break;
            case USB_CDC_CALLBACK_RX:
                if (readBufferObject.transferCount == readBufferObject.dataLen)
                {
                    //entire data is read from the host.
                }

                else
                {
                    //some of the data is not read.
                    //get some logic in place to handle this
                }
                break;
            case USB_CDC_CALLBACK_MANAGEMENT:
                break;
            default:
                break;
        }
    }

    int main ()
    {
        // create buffer
        uint8_t outBuffer[64], datalen = 64;
        USB_CDC_CLIENT_HANDLE cdcHandle;
        USB_ERROR_STATUS cdcErr;
        // create the buffer object with datalength 64
        USB_CDC_DATA_BUFFER_OBJECT readBufferObject = {&outBuffer, 64, 0};

        // open the CDC instance 0 ( or what ever is applicable)
        cdcHandle = USB_DEVICE_CDC_Open ( 0 );

        if(USB_DEVICE_CDC_RegisterCallBacks (cdcHandle,app_cdc_CallBack ))
        {
            //handle error
        }

        cdcErr = USB_DEVICE_CDC_Read( cdcHandle, &readBufferObject);
        if( !(USB_ERROR_OK == cdcErr))
        {
            //read request is accepted
            //if app callback is registered, rest needs to be handled
            //in the appcallback, which is called by the CDC function
            //driver when it is intimated by the device layer about
            //the transfer.
        }
        else
        {
            // something wrong in the read request
            // handle the error case
             switch ( cdcErr )
             {
                 case USB_ERROR_INVALID_HANDLE:
                 break;
                 case USB_ERROR_INVALID_BUFFER:
                 break;
                 case USB_ERROR_BUSY:
                 break;
             }
        }
        // All's Well That Ends Well
        return 0;
    }
    </code>

  Remarks:
    None.
*/

CDC USB_ERROR_STATUS _USB_CDC_MAKE_NAME( Read )( USB_CDC_DATA_BUFFER_OBJECT * bufferObj )
{
    /* Check if user passed valid buffer */
    if ( bufferObj == NULL )
    {
        /* return error */
        return USB_ERROR_INVALID_BUFFER;
    }

    /* Check if CDC is ready for the next transfer */
    if ( cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].dataState != USB_CDC_INSTANCE_RX_READY )
    {
        /* previous trasfer is not yet done. retry */
        return USB_ERROR_BUSY;
    }
    else /* We are good. We can accept the transfer */
    {
        /* Copy a reference of the application buffer for future use */
        cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].bufferObject = bufferObj;

        /* update the data OUT end point state to pending */
        cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].dataState = USB_CDC_INSTANCE_RX_PENDING;

        /* Place a transfer request to controller driver */
        cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].cdxferHandle =
            DRV_USB_Device_TransferRequest ( cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].pipeHandle ,
                                             USB_XFER_BULK_WRITE ,
                                             bufferObj->data ,
                                             bufferObj->dataLen ,
                                             USB_XFER_REGULAR ,
                                             &cdcInstance ,
                                             USB_DEVICE_CDC_RxDataCallBack );

        /* Check if controller driver accepted the request */
        if ( DRV_HANDLE_INVALID == cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].cdxferHandle )
        {
            /* We are not checking why it failed because all the cases where it could fail
             * are already checked in the CDC layer. The only possibility of error here is the
             * non availability of slot in the controller driver's Q.*/

            /* prepare CDC to accept new requets */
            cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].dataState = USB_CDC_INSTANCE_RX_READY;

            /* We dont want to inform the application via callback, because
             the application is given a return status right away */

            cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].bufferObject = NULL;

            /* return error. Inform the application to try after some time */
            return USB_ERROR_RETRY;
        }
    }

} /* USB_DEVICE_CDC_Read */


// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_Write( USB_CDC_CLIENT_HANDLE iCDC,
                                           USB_CDC_DATA_BUFFER_OBJECT* bufferObj )

  Summary:
    Write request to the CDC function driver.

  Description:
    This function writes data to the host. The data direction is from the device to
    the host.

  Precondition:
    None.

  Parameters:
    cHandle     - CDC client handle

    bufferObj   - Pointer to buffer Object

  Returns:
    USB_ERROR_STATUS - error code.

  Example:
    <code>
    
    // Application callback to handle relevant events.
    void app_cdc_CallBack ( SYS_MODULE_INDEX funcDriverIndex,
                            USB_CDC_CALLBACK_TYPE callback,
                            USB_CDC_CALLBACK_EVENT event)
    {

        switch (callback)
        {
            case USB_CDC_CALLBACK_TX:
                if (writeBufferObject.transferCount == writeBufferObject.dataLen)
                {
                    //entire data is written to host.
                }
                    
                else
                {
                    //some of the data is not sent to the host.
                    //think and get some logic in place to handle this
                }
                break;
            case USB_CDC_CALLBACK_RX:
                break;
            case USB_CDC_CALLBACK_MANAGEMENT:
                break;
            default:
                break;
        }    
    }
    
    int main ()
    {
        // create buffer. Assuming data end-point max packet size is 64 bytes.
        uint8_t inBuffer[64] ={'C','D','C'}, datalen = 64;
        USB_CDC_CLIENT_HANDLE cdcHandle;        
        //create the buffer object with datalength 64
        USB_CDC_DATA_BUFFER_OBJECT writeBufferObject = {&inBuffer, 64, 0};
        
        // open the CDC instance 0 ( or what ever is applicable)
        cdcHandle = USB_DEVICE_CDC_Open ( 0 );
        
        if(USB_DEVICE_CDC_RegisterCallBacks (cdcHandle,app_cdc_CallBack ))
        {
            //handle error
        }
        
        cdcErr = USB_DEVICE_CDC_Write( cdcHandle, &writeBufferObject);
        if( !(USB_ERROR_OK == cdcErr))
        {
            //write request is accepted
            //if app callback is registered, rest needs to be handled
            //in the appcallback, which is called by the CDC function
            //driver when it is intimated by the device layer about
            //the transfer.
        }
        else
        {
            // something wrong in the read request
            // handle the error case
             switch ( cdcErr )
             {
                 case USB_ERROR_INVALID_HANDLE:
                 break;
                 case USB_ERROR_INVALID_BUFFER:
                 break;
                 case USB_ERROR_BUSY:
                 break;
             }
        }        
        // All's Well That Ends Well
        return 0;        
    }
    </code>

  Remarks:
    None.
*/

CDC USB_ERROR_STATUS _USB_CDC_MAKE_NAME( Write )( USB_CDC_DATA_BUFFER_OBJECT* bufferObj )
{
    /* Check if user passed valid buffer */
    if ( bufferObj == NULL )
    {
        /* return error */
        return USB_ERROR_INVALID_BUFFER;
    }

    /* Check if the data length exceeds the max packet size */
    if ( bufferObj->dataLen > cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].epMaxPacketSize )
    {
        /* return error */
        return USB_ERROR_INVALID_DATA_LENGTH;
    }
    /* Check if CDC is ready to accept a new request */
    else if ( cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].dataState != USB_CDC_INSTANCE_TX_READY )
    {
        /* previous trasfer is not yet done. retry */
        return USB_ERROR_BUSY;
    }        
    else /* We can accept the new request */
    {
        /* copy a reference to the application buffer object for future use */
        cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].bufferObject = bufferObj;

        /* mark the IN EP state as pending */
        cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_TX_PROCESSING;

        /* Place a request to the controller driver */
        cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].cdxferHandle =
                DRV_USB_Device_TransferRequest( cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].pipeHandle ,
                                                USB_XFER_BULK_READ ,
                                                bufferObj->data ,
                                                bufferObj->dataLen ,
                                                USB_XFER_REGULAR ,
                                                &cdcInstance ,
                                                USB_DEVICE_CDC_TxDataCallBack );

        /* Check if controller driver accepted the request */
        if ( DRV_HANDLE_INVALID == cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].cdxferHandle )
        {
            /* We are not checking why it failed because all the cases where it could fail
             * are already checked in the CDC layer. The only possibility of error here is the
             * non availability of slot in the controller driver's Q.*/

            /* prepare CDC to accept new requets */
            cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].dataState = USB_CDC_INSTANCE_TX_READY;

            cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].bufferObject = NULL;

            /* return error */
            return USB_ERROR_RETRY;
        }            
    }
    /* return success */
    return USB_ERROR_OK;

} /* USB_DEVICE_CDC_Write */


// *****************************************************************************
/* Function:
    USB_CDC_DATA_STATE USB_DEVICE_CDC_RXStatusGet( USB_CDC_CLIENT_HANDLE iCDC )

  Summary:
    USB CDC receive request status.

  Description:
    Returns the status of previously submitted read request using USB_DEVICE_CDC_Read.

  Precondition:
    None.

  Parameters:
    iCDC	- CDC client handle

  Returns:
    USB_CDC_DATA_STATE	- CDC RX Status

  Example:
    <code>
    int main ()
    {
        // create buffer
        uint8_t outBuffer[64];

        uint8_t datalen = 64;

        USB_CDC_CLIENT_HANDLE cdcHandle;

        //create the buffer object with datalength 64
        USB_CDC_DATA_BUFFER_OBJECT readBufferObject = {&outBuffer, 64, 0};

        // open the CDC instance 0 ( or what ever is applicable)
        cdcHandle = USB_DEVICE_CDC_Open ( 0 );

        if(!USB_DEVICE_CDC_Read( cdcHandle, &readBufferObject))
        {
            //read request is accepted
            //if app callback is registered, rest needs to be handled
            //in the appcallback, which is called by the CDC function
            //driver when it is intimated by the device layer about
            //the transfer.
        }

        else
        {
            // something wrong in the read request
            // handle the error case
        }

        // In general CDC issues a callback after the read/write is successful/aborted. If a callback is registered with CDC.
        // The status can be checked before placing a read/write request with CDC. CDC will not accept any request
        // for read/write if a previously placed request is pending.

        // Check the status of read request
        if(USB_CDC_INSTANCE_RX_READY == USB_DEVICE_CDC_RXStatusGet(cdcHandle))
        {
            // The last request was successful. In other words CDC is ready to accept another read request
        }

        // All's Well That Ends Well
        return 0;

    }
    </code>

  Remarks:
    None.
*/

CDC USB_CDC_DATA_STATE _USB_CDC_MAKE_NAME( RXStatusGet )( void )
{
    /* return the state of data OUT endpoint */
    return( cdcInstance.dInf.ep[USB_CDC_EP_OUT_INDEX].dataState );

} /* USB_DEVICE_CDC_RXStatusGet */


// *****************************************************************************
/* Function:
    USB_CDC_DATA_STATE USB_DEVICE_CDC_TXStatusGet( USB_CDC_CLIENT_HANDLE iCDC )

  Summary:
    USB CDC transmit request status.

  Description:
    This function returns the USB CDC transmit request status.

  Precondition:
    None.

  Parameters:
    iCDC	- CDC client handle

  Returns:
    None.

  Example:
    <code>
    int main ()
    {
        // create buffer. Assuming data end-point max packet size is 64 bytes.
        uint8_t inBuffer[64] ={'C','D','C'};

        uint8_t datalen = 64;

        USB_CDC_CLIENT_HANDLE cdcHandle;

        //create the buffer object with datalength 64
        USB_CDC_DATA_BUFFER_OBJECT writeBufferObject = {&inBuffer, 64, 0};

        // open the CDC instance 0 ( or what ever is applicable)
        cdcHandle = USB_DEVICE_CDC_Open ( 0 );

        if(!USB_DEVICE_CDC_Write( cdcHandle, &writeBufferObject))
        {
            //write request is accepted
            //if app callback is registered, rest needs to be handled
            //in the appcallback, which is called by the CDC function
            //driver when it is intimated by the device layer about
            //the transfer.
        }

        else
        {
            // something wrong in the write request
            // handle the error case
        }

        // In general CDC issues a callback after the read/write is successful/aborted. If a callback is registered with CDC.
        // The status can be checked before placing a read/write request with CDC. CDC will not accept any request
        // for read/write if a previously placed request is pending.

        // Check the status of read request
        if(USB_CDC_INSTANCE_TX_READY == USB_DEVICE_CDC_TXStatusGet(cdcHandle))
        {
            // The last request was successful. In other words CDC is ready to accept another read request
        }

        // All's Well That Ends Well
        return 0;

    }
    </code>

  Remarks:
    None.
*/

CDC USB_CDC_DATA_STATE _USB_CDC_MAKE_NAME( TXStatusGet )( void )
{
    /* return the state of data IN endpoint */
    return( cdcInstance.dInf.ep[USB_CDC_EP_IN_INDEX].dataState );

} /* USB_DEVICE_CDC_TXStatusGet */


// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_RegisterCallBacks( USB_CDC_CLIENT_HANDLE iCDC,
                                                       USB_DEVICE_CDC_CALLBACK callBack )

  Summary:
    Registers a callback with the CDC function driver.

  Description:
    This function registers a callback with the CDC function driver. The registered 
    function is called if any event of type USB_CDC_CALLBACK_EVENT occurs.

  Precondition:
    None.

  Parameters:
    iCDC        - CDC client handle

    callBack  	- The function to be registered

  Returns:
    USB_ERROR_STATUS	- Error status

  Example:
    <code>
   // error returned by CDC
    USB_ERROR_STATUS err;

    // Application callback to handle relevant events.
    void app_cdc_CallBack ( SYS_MODULE_INDEX funcDriverIndex,
                            USB_CDC_CALLBACK_TYPE callback,
                            USB_CDC_CALLBACK_EVENT event)
    {

        switch (callback)
        {
            case USB_CDC_CALLBACK_TX:
                break;
            case USB_CDC_CALLBACK_RX:
                break;
            case USB_CDC_CALLBACK_MANAGEMENT:
                break;
            default:
                break;
        }

    }

    int main ()
    {

        USB_CDC_CLIENT_HANDLE cdcHandle;

        // open the CDC instance 0 ( or what ever is applicable)
        cdcHandle = USB_DEVICE_CDC_Open ( 0 );

        err = USB_DEVICE_CDC_RegisterCallBacks (cdcHandle,app_cdc_CallBack );

        if(err)
        {
            //handle error
        }

        // app code

        // Occurrance of any event of type USB_CDC_CALLBACK_EVENT results in calling of
        // the registered application callback.

        // All's Well That Ends Well
        return 0;

    }
    </code>

  Remarks:
    None.
*/

CDC USB_ERROR_STATUS _USB_CDC_MAKE_NAME( RegisterCallBacks )( USB_DEVICE_CDC_CALLBACK callBack )
{
    /* check if the callback is pointing to hell */
    if ( callBack != NULL )
    {
        /* callback is nice, copy it */
        cdcInstance.appCallBack = callBack;

        /* inform the good news to the application */
        return USB_ERROR_OK;
    }        
    else /* callback is bad */
    {
        /* inform the application that I cannot dereference hell */
        return USB_ERROR_INVALID_CALLBACK;
    }

} /* USB_DEVICE_CDC_RegisterCallBacks */


// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_CDC_SendEncapsulatedResponse( USB_CDC_CLIENT_HANDLE iCDC ,
                                                              void* data ,
                                                              uint8_t length )

  Summary:
    Sends the response to the host for the command it received via
    a GET_ENCAPSULATED_RESPONSE request.

  Description:
    This function sends the response to the host for the command it received via
    a GET_ENCAPSULATED_RESPONSE request.

  Precondition:
    The device should receive GET_ENCAPSULATED_RESPONSE.

  Parameters:
    iCDC    - CDC client handle
    data    - Pointer to the data to be transmitted to host
    length  - Length of the data

  Returns:
    USB_ERROR_STATUS	- Error status

  Example:
    <code>
    USB_ERROR_STATUS err;
    USB_CDC_AT_COMMAND cBuffer;

    void app_cdc_CallBack ( USB_CDC_CLIENT_HANDLE cdcHandle,
                            USB_CDC_CALLBACK_TYPE callback,
                            USB_CDC_CALLBACK_EVENT event)
    {

    uint8_t response[5];
    uint8_t len;
    switch (callback)
        {
            case USB_CDC_CALLBACK_TX:
                break;
            case USB_CDC_CALLBACK_RX:
                break;
            case USB_CDC_CALLBACK_MANAGEMENT:
                {
                    if(event == USB_CDC_CALLBACK_GET_ENCAPSULATED_RESPONSE)
                    {
                        // get the command for which response is required 
                        if(!USB_DEVICE_CDC_QueryATCommand ( cdcHandle ,&cBuffer ) )
                            {
                                //AT command is updated
                            }

                        // process the AT command and update the response in respose[]
                        // update the length of the response
                        len = 3;

                        // send the response to the host
                        if(!USB_DEVICE_CDC_SendEncapsulatedResponse ( cdcHandle, &response , len ) )
                        {
                            // response is sent to host
                        }
                    }
                    break;
                }
            default:
                break;
        }

    }

    int main ()
    {

        USB_CDC_CLIENT_HANDLE cdcHandle;

        // open the CDC instance 0 ( or what ever is applicable)
        cdcHandle = USB_DEVICE_CDC_Open ( 0 );

        err = USB_DEVICE_CDC_RegisterCallBacks (cdcHandle,app_cdc_CallBack );

        if(err)
        {
            //handle error
        }

        // app code

        // Occurrance of any event of type USB_CDC_CALLBACK_EVENT results in calling of
        // the registered application callback.

        // All's Well That Ends Well
        return 0;

    }
    </code>

  Remarks:
    None.
*/

CDC USB_ERROR_STATUS _USB_CDC_MAKE_NAME( SendEncapsulatedResponse )( void* data, uint8_t length )
{
    /* transfer handle */
    DRV_USB_XFER_HANDLE cdcXferHandle;

    if ( cdcInstance.request == USB_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE )
    {
        memcpy ( cdcInstance.cdcEP0InBuffer , data , length );

        /* data length */
        cdcInstance.requestDataLen = length;

        /* mark EP0 state as processing */
        cdcInstance.ep0State = USB_CDC_EP0_REQUEST_PROCESSING;

        /* place a request to device layer to get the data */
        cdcXferHandle = _USB_DEVICE_MAKE_NAME( ControlTransferRequest )( cdcInstance.cdcEP0InBuffer ,
                                                                         cdcInstance.requestDataLen ,
                                                                         USB_XFER_CONTROL_READ ,
                                                                         &cdcInstance ,
                                                                         USB_DEVICE_CDC_Ep0INCallBack );
        if ( DRV_HANDLE_INVALID == cdcXferHandle )
        {
            return USB_ERROR_INVALID_HANDLE;
        }

        return USB_ERROR_OK;
    }
    else
    {
        return USB_ERROR_RETRY;
    }

} /* USB_DEVICE_CDC_SendEncapsulatedResponse */



/*******************************************************************************
 End of File
 */

/*******************************************************************************
  USB HID Function Driver

  Company:
    Microchip Technology Inc.

  File Name:
    usb_hid_function_driver.c

  Summary:
    USB HID Function Driver 

  Description:
    USB HID Function Driver
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

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
#include "usb\usb_device_hid.h"
#include "usb\usb_device_hid_static.h"
#include "usb\usb_device.h"


// *****************************************************************************
/* HID Device driver function structure

  Summary:
    Defines the driver function structure required by the device layer.

  Description:
    This data type defines the driver function structure required by the
    device layer.

  Remarks:
    This structure is private to the USB stack.
*/
USB_DEVICE_FUNCTION_DRIVER hidFuncDriver =
{
    .initialize = USB_DEVICE_HID_MAKE_NAME(Initialize),
    .deInitialize = USB_DEVICE_HID_MAKE_NAME(Deinitialize),
    .tasks = NULL,
    .setupPacketHandler = USB_DEVICE_HID_MAKE_NAME(SetupPacketHandler),
    .checkInterfaceNumber = USB_DEVICE_HID_MAKE_NAME(CheckInterfaceNumber),
    .checkEndpointNumber = USB_DEVICE_HID_MAKE_NAME(CheckEndpointNumber)
};

// *****************************************************************************
/* HID Instance structure

  Summary:
    Defines the HID instance(s).

  Description:
    This data type defines the HID instance(s). The number of instances is
    defined by the application using USB_DEVICE_HID_MAX_INSTANCES

  Remarks:
    This structure is private to the HID.
*/
USB_DEVICE_HID_INSTANCE hidInstance;

// *****************************************************************************
/* Function:
   USB_ERROR_STATUS USB_DEVICE_HID_Initialize(const SYS_MODULE_INDEX funcDriverIndex,
                                       const SYS_MODULE_INDEX devLayerIndex,
                                       const SYS_MODULE_INDEX drvCdIndex,
                                       void* funcDriverInit,
                                       uint8_t* pConfigDesc);

  Summary:
    Initializes the instance of the HID driver.

  Description:
    This function initializes the instance of the HID driver.

  Precondition:
    None.

  Parameters:
    funcDriverIndex	- Index for the instance of HID function driver
    			  to be initialized
    devLayerInde	- USB device layer index
    drvCdIndex	 	- USB controller driver index
    funcDriverInit	- Initialization data
    pConfigDesc         - Configuration descriptor

  Returns:
    Returns the USB error status.

  Example:
    <code>
    SYS_MODULE_OBJ obj;

    obj = USB_DEVICE_HID_Initialize(USB_DEVICE_HID_INDEX_0,	//USB function driver index
                             USB_DEVICE_INDEX_0, 	//USB device layer index
                             DRV_USB_INDEX_0, 	//USB controller driver index
                             funcDriverInit);		//Initialization dat

    if(obj == SYS_MODULE_OBJ_INVALID){
	// Error handler
    }
    </code>

  Remarks:
    This routine is called in response to SET_CONFIGURATION event by the USB device layer.
*/
                
USB_ERROR_STATUS USB_DEVICE_HID_MAKE_NAME(Initialize)	(void* funcDriverInit,			
                                           		 uint8_t* pConfigDesc)
{
    USB_ERROR_STATUS errorCode = USB_ERROR_OK;
    USB_DEVICE_HID_INIT_INSTANCE *funcDrvInit = funcDriverInit;
    USB_DEVICE_HID_INSTANCE_HANDLE pHIDInstance = &hidInstance;
    USB_DEVICE_HID_EP_INSTANCE_HANDLE pHIDInEPInstance = &(pHIDInstance->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX]);
    USB_DEVICE_HID_EP_INSTANCE_HANDLE pHIDOutEPInstance = &(pHIDInstance->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX]);

    pHIDInstance->hidState = USB_DEVICE_HID_INSTANCE_INITIALIZED;
    pHIDInstance->funcDriverIndex = USB_DEVICE_HID_INDEX;
    pHIDInstance->hidFuncInit = funcDrvInit;
    pHIDInstance->devLayerIndex = USB_DEVICE_INDEX;
    pHIDInstance->drvCdIndex = DRV_USB_INDEX;
    pHIDInstance->idleRate = 0;
    pHIDInstance->activeProtocol = USB_DEVICE_HID_PROTOCOL_REPORT;
    pHIDInstance->ep0State = USB_DEVICE_HID_EP0_REQUEST_CLOSED;

    pHIDInstance->hidInterface.interfaceNum = funcDrvInit->hidInterface->interfaceNum;
    pHIDInEPInstance->epNum = funcDrvInit->hidInterface->ep[USB_DEVICE_HID_EP_IN_INDEX].epNum;
    pHIDInEPInstance->pipeHandle = 0;
    pHIDInEPInstance->epMaxPacketSize = funcDrvInit->hidInterface->ep[USB_DEVICE_HID_EP_IN_INDEX].epMaxPacketSize;
    pHIDInEPInstance->dataState = USB_DEVICE_HID_INSTANCE_WRITE_READY;
    pHIDOutEPInstance->epNum = funcDrvInit->hidInterface->ep[USB_DEVICE_HID_EP_OUT_INDEX].epNum;
    pHIDOutEPInstance->pipeHandle = 0;
    pHIDOutEPInstance->epMaxPacketSize = funcDrvInit->hidInterface->ep[USB_DEVICE_HID_EP_OUT_INDEX].epMaxPacketSize;
    pHIDOutEPInstance->dataState = USB_DEVICE_HID_INSTANCE_READ_READY;

    pHIDInstance->drvCdHandle = DRV_USB_Open(pHIDInstance->drvCdIndex, DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE);
    SYS_ASSERT((pHIDInstance->drvCdHandle != DRV_HANDLE_INVALID), "HID could not open the driver handle.");

    pHIDInEPInstance->pipeHandle =
    DRV_USB_Device_PipeSetup(pHIDInstance->drvCdHandle,
                             ((pHIDInEPInstance->epNum) & (0x0f)),
                             USB_EP_TX,
                             pHIDInEPInstance->epMaxPacketSize,
                             USB_DEVICE_HID_MAX_TRANSFERS,
                             USB_INTERRUPT_PIPE);
    SYS_ASSERT((pHIDInEPInstance->pipeHandle != DRV_HANDLE_INVALID), "HID could not setup an IN Interrupt pipe.");

    pHIDOutEPInstance->pipeHandle =
    DRV_USB_Device_PipeSetup(pHIDInstance->drvCdHandle,
                             ((pHIDOutEPInstance->epNum) & (0x0f)),
                             USB_EP_RX,
                             pHIDOutEPInstance->epMaxPacketSize,
                             USB_DEVICE_HID_MAX_TRANSFERS,
                             USB_INTERRUPT_PIPE);
    SYS_ASSERT((pHIDOutEPInstance->pipeHandle != DRV_HANDLE_INVALID), "HID could not setup an Out Interrupt pipe.");

    return errorCode;
}

// *****************************************************************************
/* Function:
   void	USB_DEVICE_HID_Deinitialize(const SYS_MODULE_INDEX funcDriverIndex)

  Summary:
    HID function driver instance deinitialization.

  Description:
    This function deinitializes the instance of the HID driver.

  Precondition:

  Parameters:
    funcDriverIndex	- Index for the instance of HID function driver
    			  to be deinitialized

  Returns:
    None.

  Remarks:
    This function is internal to the USB stack. This API should not be
    called explicitely.
*/

void USB_DEVICE_HID_MAKE_NAME(Deinitialize)	(void)
{
    USB_DEVICE_HID_INSTANCE_HANDLE pHIDInstance = &hidInstance;
    USB_DEVICE_HID_EP_INSTANCE_HANDLE pHIDInEPInstance = &(pHIDInstance->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX]);
    USB_DEVICE_HID_EP_INSTANCE_HANDLE pHIDOutEPInstance = &(pHIDInstance->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX]);

    DRV_USB_Device_PipeRelease(pHIDInEPInstance->pipeHandle);
    DRV_USB_Device_PipeRelease(pHIDOutEPInstance->pipeHandle);
    DRV_USB_Close(pHIDInstance->drvCdHandle);
    
    memset(pHIDInstance, 0, sizeof(USB_DEVICE_HID_INSTANCE));
    pHIDInstance->hidState = USB_DEVICE_HID_INSTANCE_DEINITIALIZED;
    pHIDInEPInstance->dataState = USB_DEVICE_HID_INSTANCE_INVALID;
    pHIDOutEPInstance->dataState = USB_DEVICE_HID_INSTANCE_INVALID;
}

//******************************************************************************
/* Function:
    void USB_DEVICE_HID_ControlTransferCallback(void * referenceData,
                                                DRV_USB_PIPE_HANDLE hPipe,
                                                DRV_USB_XFER_HANDLE hTransfer,
                                                uint16_t transferByteCount,
                                                DRV_USB_DEVICE_XFER_STATUS statusTransfer)
  Summary:
    Callback for the IN transfers placed on EP0.

  Description:
    This routine is the callback for the IN transfers placed on EP0.

  Remarks:
    Called by controller driver.
 */
void USB_DEVICE_HID_MAKE_NAME(ControlTransferCallback)      (void *referenceData,
                                                            DRV_USB_PIPE_HANDLE hPipe,
                                                            DRV_USB_XFER_HANDLE hTransfer,
                                                            uint16_t transferByteCount,
                                                            DRV_USB_DEVICE_XFER_STATUS statusTransfer)
{
    if(statusTransfer == USB_XFER_COMPLETED)
    {
        ((USB_DEVICE_HID_INSTANCE *)referenceData)->hidRequest = USB_DEVICE_HID_REQUEST_NONE;
        ((USB_DEVICE_HID_INSTANCE *)referenceData)->ep0State = USB_DEVICE_HID_EP0_REQUEST_CLOSED;

        if(hTransfer == ((USB_DEVICE_HID_INSTANCE *)referenceData)->hidSetReportHandle)

            if(((USB_DEVICE_HID_INSTANCE *)referenceData)->appCallBack != NULL)
            {
                ((USB_DEVICE_HID_INSTANCE *)referenceData)->appCallBack(((USB_DEVICE_HID_INSTANCE *)referenceData)->funcDriverIndex,
                                                                 USB_DEVICE_HID_CALLBACK_READ,
                                                                 USB_DEVICE_HID_CALLBACK_SET_REPORT);
            }
    }

}

// *****************************************************************************
/* Function:
   void	USB_DEVICE_HID_SetupPacketHandler(const SYS_MODULE_INDEX funcDriverIndex,
                                   uint8_t* setupPacketBuff,                
                                   uint16_t setupPacketBuffSize);           

  Summary:
    HID Class specific setup packet handler.

  Description:
    This function handles the HID class specific requests from the Host.

  Remarks:
    This function is internal to the USB stack. This API should not be
    called explicitely.
*/
void USB_DEVICE_HID_MAKE_NAME(SetupPacketHandler)	(SETUP_PKT* setupPacketBuff)
{
    USB_DEVICE_HID_INSTANCE_HANDLE pHIDInstance = &hidInstance;
    DRV_USB_XFER_HANDLE hidControlTransferHandle;

    if (!(((PSETUP_PKT)setupPacketBuff)->bmRequestType & USB_DEVICE_HID_REQUEST_CLASS_SPECIFIC ))
    {
        switch(((PSETUP_PKT)setupPacketBuff)->bRequest)
        {
            case USB_REQUEST_GET_DESCRIPTOR:
                hidControlTransferHandle =
                USB_DEVICE_ControlTransferRequest(pHIDInstance->drvCdIndex,
                                                  (void*)pHIDInstance->hidFuncInit->hidReport,
                                                  pHIDInstance->hidFuncInit->hidReportSize,
                                                  USB_XFER_CONTROL_READ,
                                                  pHIDInstance,
                                                  USB_DEVICE_HID_MAKE_NAME(ControlTransferCallback));
                SYS_ASSERT((hidControlTransferHandle != DRV_HANDLE_INVALID), "HID could not schedule control tranfer.");
                break;

            default: break;
        }
											
    }
    else
    {
        switch(((PSETUP_PKT)setupPacketBuff)->bRequest)
        {
            case USB_DEVICE_HID_GET_REPORT:
                //The Get_Report request allows the host to receive a
                //report via the Control pipe.
                //AppCallBack is expected to fill the ep0BufferObject[USB_DEVICE_HID_EP_IN_INDEX]

                if(pHIDInstance->appCallBack != NULL)
                {
                    pHIDInstance->appCallBack(pHIDInstance->funcDriverIndex,
                                              USB_DEVICE_HID_CALLBACK_WRITE,
                                              USB_DEVICE_HID_CALLBACK_GET_REPORT);
                }

                SYS_ASSERT((pHIDInstance->ep0BufferObject[USB_DEVICE_HID_EP_IN_INDEX].data != NULL), "Report data is NULL");

                hidControlTransferHandle =
                USB_DEVICE_ControlTransferRequest(pHIDInstance->drvCdIndex,
                                                  (void*)pHIDInstance->ep0BufferObject[USB_DEVICE_HID_EP_IN_INDEX].data,
                                                  pHIDInstance->ep0BufferObject[USB_DEVICE_HID_EP_IN_INDEX].dataLength,
                                                  USB_XFER_CONTROL_READ,
                                                  pHIDInstance,
                                                  USB_DEVICE_HID_MAKE_NAME(ControlTransferCallback));
                SYS_ASSERT((hidControlTransferHandle != DRV_HANDLE_INVALID), "HID could not schedule control tranfer.");
                break;

            case USB_DEVICE_HID_SET_REPORT:
                //The Set_Report request allows the host to send a report
                //to the device, possibly setting the state of input,
                //output, or feature controls.
                //AppCallBack should expect data in the ep0BufferObject[USB_DEVICE_HID_EP_OUT_INDEX]
                pHIDInstance->hidSetReportHandle =
                USB_DEVICE_ControlTransferRequest(pHIDInstance->drvCdIndex,
                                                  (void*)pHIDInstance->ep0BufferObject[USB_DEVICE_HID_EP_OUT_INDEX].data,
                                                  pHIDInstance->ep0BufferObject[USB_DEVICE_HID_EP_OUT_INDEX].dataLength,
                                                  USB_XFER_CONTROL_WRITE,
                                                  pHIDInstance,
                                                  USB_DEVICE_HID_MAKE_NAME(ControlTransferCallback));
                SYS_ASSERT((pHIDInstance->hidSetReportHandle != DRV_HANDLE_INVALID), "HID could not schedule control tranfer.");
                break;

            case USB_DEVICE_HID_GET_IDLE:
                //The Get_Idle request reads the current idle rate for a
                //particular Input report
                hidControlTransferHandle =
                USB_DEVICE_ControlTransferRequest(pHIDInstance->drvCdIndex,
                                                  (void*)&(pHIDInstance->idleRate),
                                                  1,
                                                  USB_XFER_CONTROL_READ,
                                                  pHIDInstance,
                                                  USB_DEVICE_HID_MAKE_NAME(ControlTransferCallback));
                SYS_ASSERT((hidControlTransferHandle != DRV_HANDLE_INVALID), "HID could not schedule control tranfer.");
                break;

            case USB_DEVICE_HID_SET_IDLE:
                //The Set_Idle request silences a particular report on the
                //Interrupt In pipe until a new event occurs or the
                //specified amount of time passes.
                pHIDInstance->idleRate = ((PSETUP_PKT)setupPacketBuff)->W_Value.byte.HB;
                break;

            case USB_DEVICE_HID_GET_PROTOCOL:
                //The Get_Protocol request reads which protocol is
                //currently active (either the boot protocol
                //or the report protocol.)
                hidControlTransferHandle =
                USB_DEVICE_ControlTransferRequest(pHIDInstance->devLayerIndex,
                                                  (void*)&(pHIDInstance->activeProtocol),
                                                  1,
                                                  USB_XFER_CONTROL_READ,
                                                  pHIDInstance,
                                                  USB_DEVICE_HID_MAKE_NAME(ControlTransferCallback));
                SYS_ASSERT((hidControlTransferHandle != DRV_HANDLE_INVALID), "HID could not schedule control tranfer.");  
                break;

            case USB_DEVICE_HID_SET_PROTOCOL:
                //The Set_Protocol switches between the boot protocol
                //and the report protocol (or vice versa).
                pHIDInstance->activeProtocol = ((PSETUP_PKT)setupPacketBuff)->W_Value.byte.LB;
                break;

            default: break;
        }
    }
	
}

// *****************************************************************************
/* Function:
   USB_ERROR_STATUS USB_DEVICE_HID_CheckInterfaceNumber(SYS_MODULE_INDEX funcDriverIndex, uint16_t interfaceNumber)

  Summary:
    Checks the HID interface.

  Description:
    This function checks if a HID interface is available.

  Returns:
    USB_ERROR_STATUS

  Remarks:
    This function is internal to the USB stack. This API should not be
    called explicitely.
*/
USB_ERROR_STATUS USB_DEVICE_HID_MAKE_NAME(CheckInterfaceNumber)		(uint16_t interfaceNumber)
{
    USB_ERROR_STATUS hidErrCode;

    if(hidInstance.hidInterface.interfaceNum == interfaceNumber)
        return(USB_ERROR_OK);

    return(USB_ERROR_INVALID);
}

// *****************************************************************************
/* Function:
   USB_ERROR_STATUS USB_DEVICE_HID_CheckEndpointNumber(SYS_MODULE_INDEX funcDriverIndex, uint16_t interfaceNumber)

  Summary:
    Checks the HID endpoint.

  Description:
    This function checks if a HID endpoint is available.

  Returns:
    USB_ERROR_STATUS

  Remarks:
    This function is internal to the USB stack. This API should not be
    called explicitely.
*/
USB_ERROR_STATUS USB_DEVICE_HID_MAKE_NAME(CheckEndpointNumber)	(uint16_t endpointNumber)
{
    USB_ERROR_STATUS hidErrCode;
    USB_DEVICE_HID_INSTANCE_HANDLE pHIDInstance = &hidInstance;

    if((pHIDInstance->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].epNum == endpointNumber) ||
       (pHIDInstance->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].epNum == endpointNumber))
        return(USB_ERROR_OK);

    return(USB_ERROR_INVALID);
}

//******************************************************************************
/* Function:
    void USB_DEVICE_HID_FUNC_ReadDataCallBack(void* funcDriverHandle,
                                     DRV_USB_PIPE_HANDLE hPipe,
                                     DRV_USB_XFER_HANDLE hTransfer,
                                     uint16_t transferByteCount,
                                     DRV_USB_DEVICE_XFER_STATUS transferStatus)

  Summary:
    Callback for the OUT data transfers on the data endpoint.

  Description:
    This function is the callback for the OUT data transfers on the data endpoint.

  Remarks:
    Called by the USB Controller driver.
 */
void USB_DEVICE_HID_MAKE_NAME(ReadDataCallBack)        (DRV_USB_PIPE_HANDLE hPipe,
                                                       DRV_USB_XFER_HANDLE hTransfer,
                                                       uint16_t transferByteCount,
                                                       DRV_USB_DEVICE_XFER_STATUS transferStatus)
{

    USB_DEVICE_HID_INSTANCE_HANDLE hidInstanceHandle = &hidInstance;

    switch (transferStatus)
    {
        case USB_XFER_COMPLETED:
        {
            ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].bufferObject->transferCount = transferByteCount;
            ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].dataState = USB_DEVICE_HID_INSTANCE_READ_READY;
            if(((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->appCallBack)
            {
                ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->appCallBack(((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->funcDriverIndex,
                                                                          USB_DEVICE_HID_CALLBACK_READ,
                                                                          USB_DEVICE_HID_CALLBACK_READ_COMPLETE);
            }
            break;
        }

        case USB_XFER_ABORTED:
        {
            ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].dataState = USB_DEVICE_HID_INSTANCE_READ_READY;
            if (((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->appCallBack)
            {
                ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->appCallBack(((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->funcDriverIndex,
                                                                          USB_DEVICE_HID_CALLBACK_READ,
                                                                          USB_DEVICE_HID_CALLBACK_READ_ABORT);
            }
            break;
        }

        /* other cases */
        case USB_XFER_WRONG_PIPE:
        case USB_XFER_BAD_HANDLE:
        case USB_XFER_BAD_PIPE:
        case USB_XFER_QUEUE_FULL:
        case USB_XFER_HANDLE_NOTINUSE:
        case USB_XFER_PENDING:
        case USB_XFER_IN_PROGRESS:
        {
            break;
        }

    }

}

//******************************************************************************
/* Function:
    void USB_DEVICE_HID_FUNC_WriteDataCallBack(void* pclientHIDHandle,
                                     DRV_USB_PIPE_HANDLE hPipe,
                                     DRV_USB_XFER_HANDLE hTransfer,
                                     uint16_t transferByteCount,
                                     DRV_USB_DEVICE_XFER_STATUS transferStatus)

  Summary:
    Callback for the IN notification transfers on the notification (interrupt) endpoint.

  Description:
    This routine is the callback for the IN notification transfers on the
    notification (interrupt) endpoint.

  Remarks:
    Called by the USB Controller driver.
 */
void USB_DEVICE_HID_MAKE_NAME(WriteDataCallBack)        (DRV_USB_PIPE_HANDLE hPipe,
                                                        DRV_USB_XFER_HANDLE hTransfer,
                                                        uint16_t transferByteCount,
                                                        DRV_USB_DEVICE_XFER_STATUS transferStatus)
{

    USB_DEVICE_HID_INSTANCE_HANDLE hidInstanceHandle = &hidInstance;
    switch (transferStatus)
    {
        case USB_XFER_COMPLETED:
        {
            ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].bufferObject->transferCount = transferByteCount;
            ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].dataState = USB_DEVICE_HID_INSTANCE_WRITE_READY;
            if(((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->appCallBack)
            {
                ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->appCallBack(((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->funcDriverIndex,
                                                                          USB_DEVICE_HID_CALLBACK_WRITE,
                                                                          USB_DEVICE_HID_CALLBACK_WRITE_COMPLETE);
            }
            break;
        }


        case USB_XFER_ABORTED:
        {
            ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].dataState = USB_DEVICE_HID_INSTANCE_WRITE_READY;
            if (((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->appCallBack)
            {
                ((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->appCallBack(((USB_DEVICE_HID_INSTANCE_HANDLE)hidInstanceHandle)->funcDriverIndex,
                                                                          USB_DEVICE_HID_CALLBACK_WRITE,
                                                                          USB_DEVICE_HID_CALLBACK_WRITE_ABORT);
            }
            break;
        }

        /* other cases */
        case USB_XFER_WRONG_PIPE:
        case USB_XFER_BAD_HANDLE:
        case USB_XFER_BAD_PIPE:
        case USB_XFER_QUEUE_FULL:
        case USB_XFER_HANDLE_NOTINUSE:
        case USB_XFER_PENDING:
        case USB_XFER_IN_PROGRESS:
        {
            break;
        }

    }


}

// *****************************************************************************
/* Function:
    void USB_DEVICE_HID_Tasks(SYS_MODULE_INDEX funcDriverIndex)

  Summary:
    USB HID function driver tasks routine.

  Description:
    This function manages the HID function driver state and does housekeeping.

  Precondition:
    Enumeration should be completed before calling the HID tasks function.

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
    called explicitly.
*/

void USB_DEVICE_HID_MAKE_NAME(Tasks)	(void)
{					
}

// *****************************************************************************
/* Function:
    USB_DEVICE_HID_CLIENT_HANDLE USB_DEVICE_HID_Open(const SYS_MODULE_INDEX funcDriverIndex)

  Summary:
    USB HID function driver client open.

  Description:
    This function creates a HID function driver instance.

  Precondition:
    HID function driver instance should have been initialized.

  Parameters:
    funcDriverIndex	- USB function driver index

  Returns:
    USB_DEVICE_HID_CLIENT_HANDLE	- A handle to the opened client.

  Example:
    <code>
    // Client Handle
    USB_DEVICE_HID_CLIENT_HANDLE hidClientHandle;

    // open HID function driver instance 0
    hidClientHandle = USB_DEVICE_HID_Open ( 0 );

    // Check the validit of the handle returned
    if(hidClientHandle == USB_DEVICE_HID_CLIENT_HANDLE_INVALID)
    {
        // Handle the error
    }
    </code>

  Remarks:
    None.
*/
USB_DEVICE_HID_CLIENT_HANDLE USB_DEVICE_HID_MAKE_NAME(Open)	(void)
{
    USB_DEVICE_HID_INSTANCE_HANDLE pHIDInstance = &hidInstance;

    if(pHIDInstance->hidState == USB_DEVICE_HID_INSTANCE_INITIALIZED)
    {
       pHIDInstance->hidState = USB_DEVICE_HID_INSTANCE_OPENED;
       pHIDInstance->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].dataState = USB_DEVICE_HID_INSTANCE_WRITE_READY;
       pHIDInstance->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].dataState = USB_DEVICE_HID_INSTANCE_READ_READY;

       return pHIDInstance->funcDriverIndex;
    }

    return USB_DEVICE_HID_CLIENT_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    void USB_DEVICE_HID_Close(USB_DEVICE_HID_CLIENT_HANDLE hidClientHandle)

  Summary:
    USB HID function driver client close.

  Description:
    This function closes an opened HID function driver client.

  Precondition:
    The HID function client should have been opened.

  Parameters:
    hidClientHandle	- USB function driver index

  Returns:
    USB_ERROR_STATUS	- Return status

  Example:
    <code>
    // Return status
    USB_ERROR_STATUS hidClientStatus;

    // Close HID client 1
    hidClientStatus = USB_DEVICE_HID_Close(1);

    if(hidClientStatus == USB_ERROR_INVALID_HANDLE)
    {
        // HID function driver instance 0 is either not opened/initialized yet

    }
    else if(hidClientStatus == USB_ERROR_OK)
    {
        // HID function driver instance 0 is successfully closed.
        // app code
    }

    </code>

  Remarks:
    None.
*/

USB_ERROR_STATUS USB_DEVICE_HID_MAKE_NAME(Close)	(void)
{
    USB_DEVICE_HID_INSTANCE_HANDLE pHIDClient = &hidInstance;

    if(pHIDClient->hidState == USB_DEVICE_HID_INSTANCE_OPENED)
    {
       pHIDClient->hidState = USB_DEVICE_HID_INSTANCE_CLOSED;
       return USB_ERROR_OK;
    }
    
    return USB_ERROR_INVALID_HANDLE;
}

// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_HID_ClientStatus(USB_DEVICE_HID_CLIENT_HANDLE clientHIDHandle)

  Summary:
    USB HID function driver client status.

  Description:
    This function returns the state of HID function driver client.

  Precondition:
    The HID function client should have been opened.

  Parameters:
    hidClientHandle	- USB function driver index

  Returns:
    USB_ERROR_STATUS - Return state

  Example:
 
  Remarks:
    None.
*/
USB_ERROR_STATUS USB_DEVICE_HID_MAKE_NAME(ClientStatus)		(void)
{
    if(hidInstance.hidState == USB_DEVICE_HID_INSTANCE_OPENED)
        return USB_ERROR_OK;
    else
        return USB_ERROR_INVALID_HANDLE;
}


// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_HID_RegisterCallBacks(USB_DEVICE_HID_CLIENT_HANDLE clientHIDHandle,
                                                        USB_DEVICE_HID_CALLBACK appCallBack)

  Summary:
    Registers a callback with the HID function driver.

  Description:
    This function registers a callback with the HID function driver. The registered 
    function is called if any event of type USB_DEVICE_HID_CALLBACK_EVENT occurs.

  Precondition:
    None.

  Parameters:
    clientHIDHandle - HID client handle
    appCallBack - The callback function to be registered

  Returns:
    USB_ERROR_STATUS	- Error status

  Example:
    <code>
   // error returned by HID
    USB_ERROR_STATUS err;

    // Application callback to handle relevant events.
    void app_hid_CallBack ( SYS_MODULE_INDEX funcDriverIndex,
                            USB_DEVICE_HID_CALLBACK_TYPE callback,
                            USB_DEVICE_HID_CALLBACK_EVENT event)
    {

        switch (callback)
        {
            case USB_DEVICE_HID_CALLBACK_WRITE:
                break;
            case USB_DEVICE_HID_CALLBACK_READ:
                break;
            case USB_DEVICE_HID_CALLBACK_MANAGEMENT:
                break;
            default:
                break;
        }

    }

    int main ()
    {

        USB_DEVICE_HID_CLIENT_HANDLE hidHandle;

        // open the HID instance 0 ( or what ever is applicable)
        hidHandle = USB_DEVICE_HID_Open ( 0 );

        err = USB_DEVICE_HID_RegisterCallBacks (hidHandle,app_hid_CallBack );

        if(err)
        {
            //handle error
        }

        // app code

        // Occurrance of any event of type USB_DEVICE_HID_CALLBACK_EVENT results in calling of
        // the registered application callback.

        // All's Well That Ends Well
        return 0;

    }
    </code>

  Remarks:
    None.
 */

USB_ERROR_STATUS USB_DEVICE_HID_MAKE_NAME(RegisterCallBacks)	(USB_DEVICE_HID_CALLBACK appCallBack)
{
    if(appCallBack != NULL)
    {
        hidInstance.appCallBack = appCallBack;
        return USB_ERROR_OK;
    }

    return USB_ERROR_INVALID_CALLBACK;
}

// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_HID_Write(USB_DEVICE_HID_CLIENT_HANDLE clientHIDHandle,
                                          USB_DEVICE_HID_DATA_BUFFER_OBJECT* bufferHIDObj)

  Summary:
    Write request to HID function driver.

  Description:
    This function writes data to the host. The data direction is from the device to 
    the host.

  Precondition:
    None.

  Parameters:
    clientHIDHandle     - HID client handle
    dataPtr             - Pointer to data buffer
    dataLength          - Length of data to be written

  Returns:
    USB_ERROR_STATUS - error code.

  Example:
    <code>

    // Application callback to handle relevant events.
    void app_hid_CallBack ( SYS_MODULE_INDEX clientHIDHandle,
                            USB_DEVICE_HID_CALLBACK_TYPE callback,
                            USB_DEVICE_HID_CALLBACK_EVENT event)
    {

        switch (callback)
        {
            case USB_DEVICE_HID_CALLBACK_WRITE:
                if (writeBufferObject.transferCount == writeBufferObject.dataLen)
                {
                    //all data is written to host.
                }

                else
                {
                    //some of the data is not sent to the host.
                    //user handles discrepancy
                }
                break;
            case USB_DEVICE_HID_CALLBACK_READ:
                break;

            case USB_DEVICE_HID_CALLBACK_MANAGEMENT:
                break;

            default:
                break;
        }

    }

    int main ()
    {
        // create buffer. Assuming data end-point max packet size is 64 bytes.
        uint8_t inBuffer[3] ={'X','Y','Z'};

        uint8_t datalen = 3;

        USB_DEVICE_HID_DATA_BUFFER_OBJECT hidBufferObj = { &inBuffer, datalen, 0};

        USB_DEVICE_HID_CLIENT_HANDLE hidClientHandle;

        // Open the HID instance 0 ( or whatever is applicable)
        hidClientHandle = USB_DEVICE_HID_Open ( 0 );

        if(USB_DEVICE_HID_RegisterCallBacks (hidHandle, app_hid_CallBack))
        {
            //handle error
        }

        hidErr = USB_DEVICE_HID_Write( hidHandle, &hidBufferObj);
        if(hidErr == USB_ERROR_OK)
        {
            //write request is accepted
            //if app callback is registered, rest needs to be handled
            //in the appcallback, which is called by the HID function
            //driver when it is intimated by the device layer about
            //the transfer.
        }

        else
        {
            // something wrong in the write request
            // handle the error case
             switch (hidErr)
             {
                 case USB_ERROR_INVALID_HANDLE:
                    break;
                 case USB_ERROR_INVALID_BUFFER:
                    break;
                 case USB_ERROR_BUSY:
                    break;
             }

        }

        return 0;

    }
    </code>

  Remarks:
    None.
 */

USB_ERROR_STATUS USB_DEVICE_HID_MAKE_NAME(Write)	(USB_DEVICE_HID_DATA_BUFFER_OBJECT* bufferHIDObj)
{
    USB_DEVICE_HID_INSTANCE_HANDLE pHIDClient = &hidInstance;

    if(bufferHIDObj->data == NULL)
        return USB_ERROR_INVALID_BUFFER;

    if(pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].dataState != USB_DEVICE_HID_INSTANCE_WRITE_READY)
        return USB_ERROR_BUSY;
    else
    {
        pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].bufferObject = bufferHIDObj;
        pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].dataState = USB_DEVICE_HID_INSTANCE_WRITE_PROCESSING;
        
        pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].cdXferHandle =
        DRV_USB_Device_TransferRequest(pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].pipeHandle,
                                       USB_XFER_INTERRUPT_READ,
                                       (void*) bufferHIDObj->data,
                                       bufferHIDObj->dataLength,
                                       USB_XFER_REGULAR,
                                       pHIDClient,
                                       (DRV_USB_XFER_COMPLETE_CALLBACK)USB_DEVICE_HID_MAKE_NAME(WriteDataCallBack));
        
        if(pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].cdXferHandle == DRV_HANDLE_INVALID)
        {
            pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].bufferObject = NULL;
            pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].dataState = USB_DEVICE_HID_INSTANCE_WRITE_READY;
            return USB_ERROR_RETRY;
        }
    }

    return USB_ERROR_OK;
}

// *****************************************************************************
/* Function:
    USB_ERROR_STATUS USB_DEVICE_HID_Read(USB_DEVICE_HID_CLIENT_HANDLE clientHIDHandle,
                                         USB_DEVICE_HID_DATA_BUFFER_OBJECT* bufferHIDObj)

  Summary:
    Read request to HID function driver.

  Description:
    This function reads data from the host.  The data direction is from the host to 
    the device.

  Precondition:
    None.

  Parameters:
    clientHIDHandle     - HID client handle
    dataPtr             - Pointer to data buffer
    dataLength          - Maximum length of data expected to be read

  Returns:
    USB_ERROR_STATUS   - return status of type USB_ERROR_STATUS

  Example:
    <code>
    // Application callback to handle relevant events.
    void app_hid_CallBack ( SYS_MODULE_INDEX clientHIDHandle,
                            USB_DEVICE_HID_CALLBACK_TYPE callback,
                            USB_DEVICE_HID_CALLBACK_EVENT event)
    {

        switch (callback)
        {
            case USB_DEVICE_HID_CALLBACK_WRITE:
                break;
            case USB_DEVICE_HID_CALLBACK_READ:
                if (readBufferObject.transferCount == readBufferObject.dataLen)
                {
                    //all data is read from the host.
                }

                else
                {
                    //some of the data is not read.
                    //user handles discrepancy
                }
                break;
            case USB_DEVICE_HID_CALLBACK_MANAGEMENT:
                break;
            default:
                break;
        }

    }

    int main ()
    {
        // create buffer
        uint8_t outBuffer[3];

        uint8_t maxdatalen = 3;

        USB_DEVICE_HID_DATA_BUFFER_OBJECT hidBufferObj = { &outBuffer, maxdatalen, 0};

        USB_DEVICE_HID_CLIENT_HANDLE hidHandle;

        USB_ERROR_STATUS hidErr;

        // Open the HID instance 0 ( or whatever is applicable)
        hidHandle = USB_DEVICE_HID_Open ( 0 );

        if(USB_DEVICE_HID_RegisterCallBacks (hidHandle, app_hid_CallBack ))
        {
            //handle error
        }

        hidErr = USB_DEVICE_HID_Read(hidHandle, hidBufferObj);
        if(hidErr == USB_ERROR_OK)
        {
            //read request is accepted
            //if app callback is registered, rest needs to be handled
            //in the appcallback, which is called by the HID function
            //driver when it is intimated by the device layer about
            //the transfer.
        }

        else
        {
            // something wrong in the read request
            // handle the error case
             switch (hidErr)
             {
                 case USB_ERROR_INVALID_HANDLE:
                 break;
                 case USB_ERROR_INVALID_BUFFER:
                 break;
                 case USB_ERROR_BUSY:
                 break;
             }

        }

        return 0;

    }
    </code>

  Remarks:
    None.
*/

USB_ERROR_STATUS USB_DEVICE_HID_MAKE_NAME(Read)		(USB_DEVICE_HID_DATA_BUFFER_OBJECT* bufferHIDObj)
{
    USB_DEVICE_HID_INSTANCE_HANDLE pHIDClient = &hidInstance;
    
    if(bufferHIDObj->data == NULL)
        return USB_ERROR_INVALID_BUFFER;

    if(pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].dataState != USB_DEVICE_HID_INSTANCE_READ_READY)
        return USB_ERROR_BUSY;
    else
    {
        pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].bufferObject = bufferHIDObj;
        pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].dataState = USB_DEVICE_HID_INSTANCE_READ_PENDING;
        pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].cdXferHandle =
        DRV_USB_Device_TransferRequest(pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].pipeHandle,
                                       USB_XFER_INTERRUPT_WRITE,
                                       (void*) bufferHIDObj->data,
                                       bufferHIDObj->dataLength,
                                       USB_XFER_REGULAR,
                                       pHIDClient,
                                       (DRV_USB_XFER_COMPLETE_CALLBACK)USB_DEVICE_HID_MAKE_NAME(ReadDataCallBack));
    
        if(pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].cdXferHandle == DRV_HANDLE_INVALID)
        {

            pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].bufferObject = NULL;
            pHIDClient->hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].dataState = USB_DEVICE_HID_INSTANCE_READ_READY;
            return USB_ERROR_RETRY;
        }
    }

    return USB_ERROR_OK;
}

// *****************************************************************************
/* Function:
    USB_DEVICE_HID_DATA_STATE USB_DEVICE_HID_ReadStatusGet(USB_DEVICE_HID_CLIENT_HANDLE clientHIDHandle)

  Summary:
    USB HID read request status.

  Description:
    Returns the status of the last read request using USB_DEVICE_HID_Read.

  Precondition:
    None.

  Parameters:
    clientHIDHandle	- HID client handle

  Returns:
    USB_DEVICE_HID_DATA_STATE	- HID Read Status

  Example:
    <code>

    int main ()
    {
        // create buffer
        uint8_t outBuffer[3];

        uint8_t maxdatalen = 3;

        USB_DEVICE_HID_CLIENT_HANDLE hidHandle;

        USB_ERROR_STATUS hidErr;

        // Open the HID instance 0 ( or whatever is applicable)
        hidHandle = USB_DEVICE_HID_Open ( 0 );

        if(USB_DEVICE_HID_RegisterCallBacks (hidHandle, app_hid_CallBack ))
        {
            //handle error
        }

        hidErr = USB_DEVICE_HID_Read(hidHandle, outBuffer, maxdatalen);
        if(hidErr == USB_ERROR_OK)
        {
            //read request is accepted
            //if app callback is registered, rest needs to be handled
            //in the appcallback, which is called by the HID function
            //driver when it is intimated by the device layer about
            //the transfer.
        }

        else
        {
            // something wrong in the read request
            // handle the error case
             switch (hidErr)
             {
                 case USB_ERROR_INVALID_HANDLE:
                 break;
                 case USB_ERROR_INVALID_BUFFER:
                 break;
                 case USB_ERROR_BUSY:
                 break;
             }

        }

        // In general HID issues a callback after the read/write is successful/aborted if a callback is registered with HID.
        // The status can be checked before placing a read/write request with HID. HID will not accept any request
        // for read/write if a previously placed request is pending.

        // Check the status of read request
        if(USB_DEVICE_HID_ReadStatusGet(hidHandle) == USB_DEVICE_HID_INSTANCE_READ_READY)
        {
            // The last request was successful. HID is now ready to accept another read request.
        }

        return 0;

    }
    </code>

  Remarks:
    None.
*/
USB_DEVICE_HID_DATA_STATE USB_DEVICE_HID_MAKE_NAME(ReadStatusGet)	(void)
{
    return(hidInstance.hidInterface.ep[USB_DEVICE_HID_EP_OUT_INDEX].dataState);

}

// *****************************************************************************
/* Function:
    USB_DEVICE_HID_DATA_STATE USB_DEVICE_HID_WriteStatusGet(USB_DEVICE_HID_CLIENT_HANDLE clientHIDHandle)

  Summary:
    USB HID write request status.

  Description:
    This function returns the status of the last write request using USB_DEVICE_HID_Write.

  Precondition:
    None.

  Parameters:
    clientHIDHandle	- HID client handle

  Returns:
    USB_DEVICE_HID_DATA_STATE	- HID write status

  Example:
    <code>

    int main ()
    {
        // create buffer
        uint8_t outBuffer[3];

        uint8_t datalen = 3;

        USB_DEVICE_HID_CLIENT_HANDLE hidHandle;

        USB_ERROR_STATUS hidErr;

        // Open the HID instance 0 ( or whatever is applicable)
        hidHandle = USB_DEVICE_HID_Open ( 0 );

        if(USB_DEVICE_HID_RegisterCallBacks (hidHandle, app_hid_CallBack ))
        {
            //handle error
        }

        hidErr = USB_DEVICE_HID_Write(hidHandle, outBuffer, datalen);
        if(hidErr == USB_ERROR_OK)
        {
            //write request is accepted
            //if app callback is registered, rest needs to be handled
            //in the appcallback, which is called by the HID function
            //driver when it is intimated by the device layer about
            //the transfer.
        }

        else
        {
            // something wrong in the write request
            // handle the error case
             switch (hidErr)
             {
                 case USB_ERROR_INVALID_HANDLE:
                 break;
                 case USB_ERROR_INVALID_BUFFER:
                 break;
                 case USB_ERROR_BUSY:
                 break;
             }

        }

        // In general HID issues a callback after the read/write is successful/aborted if a callback is registered with HID.
        // The status can be checked before placing a read/write request with HID. HID will not accept any request
        // for read/write if a previously placed request is pending.

        // Check the status of write request
        if(USB_DEVICE_HID_WriteStatusGet(hidHandle) == USB_DEVICE_HID_INSTANCE_WRITE_READY)
        {
            // The last write request was successful. HID is now ready to accept another write request.
        }

        return 0;

    }
    </code>

  Remarks:
    None.
*/

USB_DEVICE_HID_DATA_STATE USB_DEVICE_HID_MAKE_NAME(WriteStatusGet)	(void)
{
    return (hidInstance.hidInterface.ep[USB_DEVICE_HID_EP_IN_INDEX].dataState );
}



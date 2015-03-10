/*******************************************************************************
 USB Audio Class Function Driver - Read and Write functions. 

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device_audio.c

  Summary:
    USB audio class function driver.

  Description:
    USB audio class function driver.
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
#include "usb/usb_device_audio_v1_0.h"
#include "usb/src/usb_device_audio_local.h"



// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Audio function instance objects.

  Summary:
    Holds all of the variables required by audio instance.

  Description:
    This structure holds all of the variables required by audio instance.

  Remarks:
    None.
*/

USB_DEVICE_AUDIO_INSTANCE gUsbDeviceAudioInstance[USB_DEVICE_AUDIO_INSTANCES_NUMBER];


// *****************************************************************************
/* AUDIO Device IRPs

  Summary:
    Array of AUDIO Device IRP.

  Description:
    Array of AUDIO Device IRP. This array of IRP will be shared by read, write and
    notification data requests.

  Remarks:
    This array is private to the USB stack.
 */

USB_DEVICE_IRP gUSBDeviceAudioIRP[USB_DEVICE_AUDIO_QUEUE_DEPTH_COMBINED];


// *****************************************************************************
/* AUDIO Device IRP Data

  Summary:
    Array of AUDIO Device IRP Data.

  Description:
    Array of AUDIO Device IRP. This array of IRP will be shared by read, write and
    notification data requests.

  Remarks:
    This array is private to the USB stack.
 */
USB_DEVICE_AUDIO_IRP_DATA gUSBDeviceAudioIrpData [USB_DEVICE_AUDIO_QUEUE_DEPTH_COMBINED];


/* ******************************************************************************
  Function:
    USB_DEVICE_AUDIO_RESULT USB_DEVICE_AUDIO_EventHandlerSet
    (
        USB_DEVICE_AUDIO_INDEX instance ,
        USB_DEVICE_AUDIO_EVENT_HANDLER eventHandler ,
        uintptr_t context
    );
    
  Summary:
    This function registers an event handler for the specified Audio function
    driver instance. 

  Description:
  Refer  usb_device_audio_v1_0.h for detailed description of this function. 
  
  */ 
USB_DEVICE_AUDIO_RESULT USB_DEVICE_AUDIO_EventHandlerSet
(
    USB_DEVICE_AUDIO_INDEX iAudio ,
    USB_DEVICE_AUDIO_EVENT_HANDLER eventHandler ,
    uintptr_t userData
)
{
    USB_DEVICE_AUDIO_RESULT error = USB_DEVICE_AUDIO_RESULT_ERROR_PARAMETER_INVALID;

    if(eventHandler != NULL)
    {
        gUsbDeviceAudioInstance[iAudio].appEventCallBack = eventHandler;
        gUsbDeviceAudioInstance[iAudio].userData = userData;
        error = USB_DEVICE_AUDIO_RESULT_OK;
    }
    return error;
}

// *****************************************************************************
/* Function:
    USB_DEVICE_AUDIO_RESULT USB_DEVICE_AUDIO_Read ( USB_DEVICE_AUDIO_INDEX iAudio ,
                                      uint8_t interfaceNum ,
                                      void * data ,
                                      size_t size )

  Summary:
    Reads the data received from the Audio Interface for
 *  specified instance of the USB device layer.

  Description:
    Reads the data received from the Audio Interface for
 *  specified instance of the USB device layer.

 *
  Parameters:
    USB_DEVICE_AUDIO_INDEX iAudio    - Audio function driver Index number
 *
 *  uint8_t interfaceNum    - Audio streaming or Control Interface number
 *
 *  data - pointer to the data buffer where read data will be stored.
 *  size - Size of the data buffer. Refer to the description section for more
           details on how the size affects the transfer.
 *
 *
  Returns:
    USB_DEVICE_AUDIO_RESULT

*/

USB_DEVICE_AUDIO_RESULT USB_DEVICE_AUDIO_Read
(
    USB_DEVICE_AUDIO_INDEX iAudio ,
    USB_DEVICE_AUDIO_TRANSFER_HANDLE* transferHandle,
    uint8_t interfaceNum ,
    void * data ,
    size_t size
)
{
        USB_DEVICE_AUDIO_RESULT audioResult;
        USB_DEVICE_AUDIO_INSTANCE * thisAudioDevice;
        thisAudioDevice = &gUsbDeviceAudioInstance[iAudio];

         /* Make sure that we are with in the queue size for this instance */
        if(thisAudioDevice->currentQSizeRead >= thisAudioDevice->queueSizeRead)
        {
            SYS_ASSERT(false, "Read Queue is full");
            return(USB_DEVICE_AUDIO_RESULT_ERROR_TRANSFER_QUEUE_FULL);
        }
	audioResult =   _USB_DEVICE_AUDIO_Transfer(iAudio, transferHandle, interfaceNum, data, size, USB_DEVICE_AUDIO_READ );
        return audioResult; 
}


//******************************************************************************

// *****************************************************************************
// *****************************************************************************
/* Function:
    USB_ERROR USB_DEVICE_AUDIO_Write ( USB_DEVICE_AUDIO_INDEX iAudio ,
                                         uint8_t interfaceNum ,
                                         USB_DEVICE_AUDIO_DATA_BUFFER_OBJECT* bufferObj )

  Summary:
    sends to the Audio Interface for
 *  the specified instance of the USB device layer.

  Description:
    sends to the Audio Interface for
 *  the specified instance of the USB device layer.

 *
  Parameters:
    USB_DEVICE_AUDIO_INDEX iAudio    - Audio function driver Index number
 *
 *  uint8_t interfaceNum    - Audio streaming or Control Interface number
 *
 *  USB_DEVICE_AUDIO_DATA_BUFFER_OBJECT* bufferObj - pointer to the buffer where received data
 *
 * is to be stored.
 *
 *
  Returns:
    USB_DEVICE_AUDIO_RESULT

*/


USB_DEVICE_AUDIO_RESULT USB_DEVICE_AUDIO_Write
(
    USB_DEVICE_AUDIO_INDEX iAudio ,
    USB_DEVICE_AUDIO_TRANSFER_HANDLE* transferHandle,
    uint8_t interfaceNum ,
    void * data ,
    size_t size
)
{
        USB_DEVICE_AUDIO_RESULT audioResult;
        USB_DEVICE_AUDIO_INSTANCE * thisAudioDevice;
        thisAudioDevice = &gUsbDeviceAudioInstance[iAudio];

         /* Make sure that we are with in the queue size for this instance */
        if(thisAudioDevice->currentQSizeWrite >= thisAudioDevice->queueSizeWrite)
        {
            SYS_ASSERT(false, "Write Queue is full");
            return(USB_DEVICE_AUDIO_RESULT_ERROR_TRANSFER_QUEUE_FULL);
        }
	audioResult =   _USB_DEVICE_AUDIO_Transfer(iAudio, transferHandle, interfaceNum, data, size, USB_DEVICE_AUDIO_WRITE );
        return audioResult;
}
// *****************************************************************************


/* ******************************************************************************
  Function:
    USB_DEVICE_AUDIO_RESULT _USB_DEVICE_AUDIO_Transfer
	(
		USB_DEVICE_AUDIO_INDEX iAudio,
		USB_DEVICE_AUDIO_TRANSFER_HANDLE *transferHandle,
		uint8_t interfaceNum ,
		void * data ,
		size_t size,
		USB_DEVICE_AUDIO_TRANSFER_DIRECTION direction
	); 
    
  Summary:
    This function schedules an Audio transfer. This is a local function and should
	be called by applications directly. 

  */ 
// *****************************************************************************
USB_DEVICE_AUDIO_RESULT _USB_DEVICE_AUDIO_Transfer
(
    USB_DEVICE_AUDIO_INDEX iAudio,
    USB_DEVICE_AUDIO_TRANSFER_HANDLE *transferHandle,
    uint8_t interfaceNum ,
    void * data ,
    size_t size,
    USB_DEVICE_AUDIO_TRANSFER_DIRECTION direction
)
{
    /* Holds Audio Stream Interface array index for the corresponding streaming interface interfaceNum */
    uint8_t streamInfIndex;

    /*Holds audio Control Interface ID of audio function instance iAudio*/
    uint8_t audioControlIntrfcID;

    /*Holds value of active alternate settings for this interface*/
    uint8_t activeAlternateSetting;

    uint8_t cnt;

    USB_ENDPOINT epStruct;

    USB_DEVICE_IRP_STATUS irpStatus;

    USB_ERROR irpErr;

    USB_DEVICE_AUDIO_EP_INSTANCE* tempEndpointInstance;

    bool freeSlot = false;

    USB_DEVICE_IRP * irp;

    USB_DEVICE_AUDIO_IRP_DATA *audioIrpData;

    /* Get a pointer to the current USB audio instance that is being addressed*/
    USB_DEVICE_AUDIO_INSTANCE * thisAudioInstance = &gUsbDeviceAudioInstance[iAudio];

    /* Initially send the transfer handle to invalid */
    *transferHandle = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;

    /* check the validity of the function driver index */
    if ( USB_DEVICE_AUDIO_INSTANCES_NUMBER <= iAudio )
    {
        /* invalid handle */
        return USB_DEVICE_AUDIO_RESULT_ERROR_INSTANCE_INVALID;
    }

    /* Check if user passed valid buffer */
    if ( data == NULL )
    {
        /* Write data should not be null. It should point to valid data location
         return error */
        return USB_DEVICE_AUDIO_RESULT_ERROR_INVALID_BUFFER;
    }

    /* Retrieve Audio Control Interface ID. We are going to use it to locate the Audio streaming
     * Interface array index from the interfaceNum passed to this function */
    audioControlIntrfcID = thisAudioInstance->infCollection.bControlInterfaceNum;

    /*Find out the array streaming interface */
    streamInfIndex = interfaceNum - audioControlIntrfcID- 1;

    /*Retrieve the active alternate setting of the interface from Audio Instance object */
    activeAlternateSetting = thisAudioInstance->infCollection.streamInf[streamInfIndex].activeSetting;

    if (activeAlternateSetting == 0)
    {
        SYS_ASSERT ( false , "alternate setting 0 does not allow Data Payload" );
        return USB_DEVICE_AUDIO_RESULT_ERROR_INSTANCE_NOT_CONFIGURED; 
    }

    /* Check if the interface number passed to this function is valid*/
    if (interfaceNum != thisAudioInstance->infCollection.streamInf[streamInfIndex].interfaceNum)
    {
        SYS_ASSERT ( false , "Invalid interface number " );
        return USB_DEVICE_AUDIO_RESULT_ERROR_INVALID_INTERFACE_ID;
    }

    tempEndpointInstance = &(thisAudioInstance->infCollection.streamInf[streamInfIndex].alterntSetting[activeAlternateSetting].isoDataEp);

    /* Loop and find a free IRP in the Q */
    for ( cnt = 0; cnt < USB_DEVICE_AUDIO_QUEUE_DEPTH_COMBINED; cnt ++ )
    {
        /* get the IRP status */
        irp = &gUSBDeviceAudioIRP[cnt];
        audioIrpData =&gUSBDeviceAudioIrpData[cnt];
        irpStatus = irp->status;
        /* check if it can be used */
        if ( ( irpStatus != USB_DEVICE_IRP_STATUS_IN_PROGRESS ) && ( irpStatus != USB_DEVICE_IRP_STATUS_PENDING ) )
        {
            /* got a free slot in the Q 'cnt' denotes the free slot index */
            freeSlot = true;

            break;
        }
    }

    /* check if free slot is available */
    if(freeSlot)
    {
        //retrieve endpoint address
        epStruct = tempEndpointInstance->epAddr;

        /* fill IRP object with the pointer to the data that is to be transferred to the Host*/
        irp->data = data;

        /* fill IRP object with size of the data that is to be transferred to the USB host*/
        irp->size = size;

        /* save Interface ID */
        audioIrpData->interfaceNum = interfaceNum;

        /* save Data direction */
        audioIrpData->direction = direction;

        /* save Audio Function driver instance */
        audioIrpData->iAudio = iAudio;

        /* Provide function address to call back when IRP is complete */
        irp->callback = _USB_DEVICE_AUDIO_TransferIRPCallBack;

        /* Save array index. We will need this to retrieve data when we get IRP call back */
        irp->userData = cnt;

        /* save transfer handle */
        *transferHandle = (USB_DEVICE_AUDIO_TRANSFER_HANDLE) irp;

        if (direction == USB_DEVICE_AUDIO_READ )
        {
            thisAudioInstance->currentQSizeRead++;
        }
        else
        {
            thisAudioInstance->currentQSizeWrite++;
        }
        /* Submit IRP */ 
        irpErr = USB_DEVICE_IRPSubmit ( thisAudioInstance->devLayerHandle ,
                                            epStruct , irp);
        /* check if IRP submit is success */
        if ( USB_ERROR_NONE == irpErr )
        {
            /* return success */
            return USB_DEVICE_AUDIO_RESULT_OK;
        }
        else
        {
            if (direction == USB_DEVICE_AUDIO_READ )
            {
                thisAudioInstance->currentQSizeRead--;
            }
            else
            {
                thisAudioInstance->currentQSizeWrite--;
            }
            /* only possibility  */
            return USB_DEVICE_AUDIO_RESULT_ERROR_TRANSFER_QUEUE_FULL;
        }
    }
    /* no free slot */
    else
    {

        /* Q full */
        return USB_DEVICE_AUDIO_RESULT_ERROR_TRANSFER_QUEUE_FULL;
    }
}
/*******************************************************************************
 End of File
 */

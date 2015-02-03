/*
 * File:   usb_host_android_protocol_v1.c
 * Author: diego.tsutsumi
 *
 * Created on 11th November 2014, 10:12
 */

#include "GenericTypeDefs.h"

#include "usb_host_android_protocol_v1_local.h"

static ANDROID_PROTOCOL_V1_DEVICE_DATA device_pv1;

USB_HOST_CLASS_DRIVER androidDriver =
{
    /*name of the driver */
    .initializeDeviceDriver = _USB_HOST_Android_DriverInitialize ,
    .initializeInterfaceDriver =  NULL,
    .deInitializeDriver = _USB_HOST_Android_DeInitialize ,
    .task           = _USB_HOST_Android_Task
};


void AndroidAppStart_Pv1(void)
{
    memset(&device_pv1,0x00,sizeof(device_pv1));
}

USB_ERROR _USB_HOST_Android_DriverInitialize (USB_HOST_DEVICE_ID hostId,uint8_t *androidInstance, USB_SPEED speed)
{
    USB_HOST_QUERY  androidQuery;
    USB_ERROR  status;
    USB_ENDPOINT_DESCRIPTOR *bulkInEndpointDesc ,
                            *bulkOutEndpointDesc;
    USB_ENDPOINT_DESCRIPTOR controlEndpointDesc;
    ANDROID_PROTOCOL_V1_DEVICE_DATA *androidInstanceInfo;

    status = USB_ERROR_NONE;
    androidInstanceInfo = &(device_pv1);

    *androidInstance = 0;
    androidInstanceInfo->assigned = true;
    androidInstanceInfo->androidDeviceId = hostId;

    androidQuery.id = hostId;

    /* active configuration number will change after*/
    androidQuery.configurationNumber = 0;
    androidQuery.class = 0xFF;
    androidQuery.subClass = 0xFF;
    androidQuery.protocol = 0x00;

    /* class match */
    androidQuery.flags = USB_HOST_MATCH_CLASS_SUBCLASS_PROTOCOL;
    androidQuery.alternateSettingNumber = 0;

    /* Get control pipe information */
    controlEndpointDesc.bmAttributes =  USB_TRANSFER_TYPE_CONTROL;
    androidInstanceInfo->controlPipeHandle =  _USB_HOST_PipeSetup
                                ( hostId ,  &controlEndpointDesc , speed );
    if(androidInstanceInfo->controlPipeHandle == USB_HOST_PIPE_HANDLE_INVALID )
    {
        return USB_ERROR_HOST_PIPE_INVALID;
    }
    /* Fill Control IRP */
    androidInstanceInfo->controlIRP.callback =
                                    _USB_HOST_Android_ControlTransferComplete;
    /*find CDC instance in callback functions */
    androidInstanceInfo->controlIRP.userData = (uintptr_t)androidInstanceInfo;

    /* Fine bulkIn endpoint descriptor */
    androidQuery.endpointAddress = 0 ; // we have to get temp
    androidQuery.endpointType = USB_TRANSFER_TYPE_BULK;
    androidQuery.endpointDirection = USB_DATA_DIRECTION_DEVICE_TO_HOST;
    androidQuery.interfaceNumber = androidInstanceInfo->datainterfaceNumber;
    androidQuery.flags = USB_HOST_MATCH_ENDPOINT_TYPE | USB_HOST_MATCH_ENDPOINT_DIRECTION;
    status = _USB_HOST_FindEndpoint (&(androidQuery), &(bulkInEndpointDesc));

    /* Validate the Endpoint is available */
    if( status != USB_ERROR_NONE )
    {
        return USB_ERROR_HOST_ENDPOINT_INVALID;
    }

    /* Create Bulkin pipe */
    androidInstanceInfo->bulkInPipeHandle = _USB_HOST_PipeSetup
                                ( hostId ,  bulkInEndpointDesc , speed );
    /* validate BulkInPipe handle*/
    if( androidInstanceInfo->bulkInPipeHandle == USB_HOST_PIPE_HANDLE_INVALID )
    {
        return USB_ERROR_HOST_PIPE_INVALID;
    }

    /* Fill Bulkin IRP */
    androidInstanceInfo->bulkInIRP.callback = (void * )_USB_HOST_Android_BulkInTransferComplete;
    androidInstanceInfo->bulkInIRP.size = bulkInEndpointDesc->wMaxPacketSize ;
    /*Instance info required in callback function */
    androidInstanceInfo->bulkInIRP.userData = (uintptr_t)androidInstanceInfo;
     /* Fill the cdcQuery to find bulkOut endpoint */

    androidQuery.endpointAddress = 0 ; // we have to get temp
    androidQuery.endpointType = USB_TRANSFER_TYPE_BULK;
    androidQuery.endpointDirection = USB_DATA_DIRECTION_HOST_TO_DEVICE;
    androidQuery.flags = USB_HOST_MATCH_ENDPOINT_TYPE | USB_HOST_MATCH_ENDPOINT_DIRECTION;

    /* Find the bulkout endpoint descriptor */
    status = _USB_HOST_FindEndpoint (&(androidQuery), &(bulkOutEndpointDesc));
      if( status != USB_ERROR_NONE )
    {
        return USB_ERROR_HOST_ENDPOINT_INVALID;
    }

    /* Create Bulkout pipe */
    androidInstanceInfo->bulkOutPipeHandle = _USB_HOST_PipeSetup
                                ( hostId ,  bulkOutEndpointDesc ,speed);
    /* Validate BulkOutPipe handle*/
    if( androidInstanceInfo->bulkOutPipeHandle == USB_HOST_PIPE_HANDLE_INVALID )
    {
        return USB_ERROR_HOST_PIPE_INVALID;
    }

    /* Fill Bulk out piep */
    androidInstanceInfo->bulkOutIRP.callback = _USB_HOST_Android_BulkOutTransferComplete;
    androidInstanceInfo->bulkOutIRP.size = bulkOutEndpointDesc->wMaxPacketSize ;

    /*Instance info for callback function */
    androidInstanceInfo->bulkOutIRP.userData = (uintptr_t )androidInstanceInfo;

    androidInstanceInfo->tpl_index = USB_HOST_GetVendorIndex();
    androidInstanceInfo->appEventCallBack(USB_HOST_ANDROID_EVENT_ATTACH, (uint32_t)(androidInstanceInfo->tpl_index),androidInstanceInfo->context);
    return status;
}

void  _USB_HOST_Android_DeInitialize ( USB_HOST_DEVICE_ID id  )
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA *device;
    
    device = &device_pv1;

    if(device->assigned == false)
    {
        return;
    }

    device->appEventCallBack( USB_HOST_ANDROID_EVENT_DETACH, NULL, device->context);

    _USB_HOST_PipeClose( device->bulkInPipeHandle );

    _USB_HOST_PipeClose( device->bulkOutPipeHandle );

    //_USB_HOST_PipeClose( device->controlPipeHandle );

    device->assigned = false ;

    memset(&device_pv1,0x00,sizeof(device_pv1));
}

USB_ANDROID_RETURN AndroidAppWrite_Pv1(BYTE* data, size_t size)
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA* device = &device_pv1;
    USB_HOST_IRP *irp;
    USB_ERROR status;


    if(device == NULL)
    {
        return USB_ANDROID_UNKNOWN_DEVICE;
    }

    if(device->assigned == false)
    {
        return USB_HOST_ANDROID_RESULT_ERROR_INSTANCE_NOT_READY;
    }

    irp = &(device->bulkOutIRP);
    //transferHandle = (USB_HOST_ANDROID_TRANSFER_HANDLE *) &(device->bulkOutIRP);

    irp->data = (void*)data;
    irp->size = size;
    irp->callback = _USB_HOST_Android_BulkOutTransferComplete;
    irp->userData = (uintptr_t)device;
     
    status = _USB_HOST_IRPSubmit(device->bulkOutPipeHandle,
                                &(device->bulkOutIRP));
    
    if(status != USB_ERROR_NONE )
    {
        return USB_ANDROID_ERROR_TRANSFER_QUEUE_FULL;
    }
    return USB_ANDROID_RESULT_OK;
}

void _USB_HOST_Android_BulkOutTransferComplete( USB_HOST_IRP *irp)
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA *device;
    USB_HOST_ANDROID_EVENT_DATA_WRITE_COMPLETE writeCompleteEventData;
    device = ((ANDROID_PROTOCOL_V1_DEVICE_DATA *) (irp->userData));


    if (irp->status == USB_HOST_IRP_STATUS_COMPLETED ||
            irp->status == USB_HOST_IRP_STATUS_COMPLETED_SHORT)
    {

        /* This means the Read IRP has terminated */
        writeCompleteEventData.length = irp->size;
        /* Transfer was successful */
        writeCompleteEventData.transferStatus =
                USB_HOST_ANDROID_TRANSFER_STATUS_OK;
    }
    else
    {
        /* Transfer failed */
        writeCompleteEventData.transferStatus =
                USB_HOST_ANDROID_TRANSFER_STATUS_ERROR;
    }
    writeCompleteEventData.transferHandle = (USB_HOST_ANDROID_TRANSFER_HANDLE) irp;

    /* Callback to the application */
    device->appEventCallBack(USB_HOST_ANDROID_EVENT_WRITE_COMPLETE, NULL, device->context);
}

USB_ERROR USB_HOST_Android_EventHandlerSet(USB_HOST_ANDROID_EVENT_HANDLER appAndroidHandler)
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA *device;

    if( NULL == appAndroidHandler )
    {
        return USB_ERROR_HOST_POINTER_INVALID;
    }
    device = &(device_pv1);
    device->appEventCallBack = appAndroidHandler ;

    return USB_ERROR_NONE;
}

USB_ANDROID_RETURN AndroidAppRead_Pv1(BYTE* data, size_t size)
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA* device = &device_pv1;
    USB_HOST_IRP *irp;
    USB_ERROR status;

    if(device == NULL)
    {
        return USB_ANDROID_UNKNOWN_DEVICE;
    }

    irp = &(device->bulkInIRP);
    
    //transferHandle = (USB_HOST_ANDROID_TRANSFER_HANDLE  *) &(device->bulkInIRP);

    
    irp->data = (void*)data;
    irp->size = size;
    irp->callback = _USB_HOST_Android_BulkInTransferComplete;
    irp->userData = (uintptr_t) device;
    
    status = _USB_HOST_IRPSubmit(device->bulkInPipeHandle, &(device->bulkInIRP));

    if(status != USB_ERROR_NONE )
    {
        return USB_ANDROID_ERROR_TRANSFER_QUEUE_FULL;
    }
    return USB_ANDROID_RESULT_OK;
}

void _USB_HOST_Android_BulkInTransferComplete(USB_HOST_IRP *irp)
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA *device;
    USB_HOST_ANDROID_EVENT_DATA_READ_COMPLETE readCompleteEventData;

    device = ((ANDROID_PROTOCOL_V1_DEVICE_DATA *) (irp->userData));
    if (irp->status == USB_HOST_IRP_STATUS_COMPLETED
            || irp->status == USB_HOST_IRP_STATUS_COMPLETED_SHORT)
    {
        readCompleteEventData.length = irp->size;
        /* Transfer was successful */
        readCompleteEventData.transferStatus = USB_HOST_ANDROID_TRANSFER_STATUS_OK;
    }
    else
    {
        /* Transfer failed */
        readCompleteEventData.transferStatus =
                USB_HOST_ANDROID_TRANSFER_STATUS_ERROR;
    }

    readCompleteEventData.transferHandle = (USB_HOST_ANDROID_TRANSFER_HANDLE) irp;
    device->appEventCallBack(USB_HOST_ANDROID_EVENT_READ_COMPLETE, irp->size, device->context);
}

void _USB_HOST_Android_ControlTransferComplete(USB_HOST_IRP *irp)
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA *device;
    USB_HOST_ANDROID_EVENT_ACCESSORY_STRING_COMPLETE strComplete;
    USB_HOST_ANDROID_EVENT_ACCESSORY_START_COMPLETE startComplete;

    device = ((ANDROID_PROTOCOL_V1_DEVICE_DATA *) (irp->userData));

    if (irp->status == USB_HOST_IRP_STATUS_COMPLETED ||
            irp->status == USB_HOST_IRP_STATUS_COMPLETED_SHORT)
    {
        switch (device->controlRequest)
        {
            case ANDROID_ACCESSORY_SEND_STRING:
                strComplete.length = irp->size;
                strComplete.transferHandle = (USB_HOST_ANDROID_TRANSFER_HANDLE) irp;
                strComplete.transferStatus =
                                            USB_HOST_ANDROID_TRANSFER_STATUS_OK;
                device->appEventCallBack(
                        USB_HOST_ANDROID_EVENT_SEND_STRING_COMPLETE, NULL,
                        device->context);
                break;

            case ANDROID_ACCESSORY_START:
                startComplete.length = irp->size;
                startComplete.transferHandle =
                        (USB_HOST_ANDROID_TRANSFER_HANDLE) irp;
                strComplete.transferStatus =
                        USB_HOST_ANDROID_TRANSFER_STATUS_OK;
                device->appEventCallBack(
                        USB_HOST_ANDROID_EVENT_START_COMPLETE, NULL,
                        device->context);
                break;

            case ANDROID_ACCESSORY_GET_PROTOCOL:
                startComplete.length = irp->size;
                startComplete.transferHandle =
                        (USB_HOST_ANDROID_TRANSFER_HANDLE) irp;
                strComplete.transferStatus =
                        USB_HOST_ANDROID_TRANSFER_STATUS_OK;
                device->appEventCallBack(
                        USB_HOST_ANDROID_EVENT_PROTOCOL_COMPLETE, NULL,
                        device->context);
                break;
        }
    }
    else
    {
        if (irp->status == USB_HOST_IRP_STATUS_ERROR_NAK_TIMEOUT)
        {
            //Timing problem will resend the same packet
        }
        if (irp->status == USB_HOST_IRP_STATUS_ERROR_DATA ||
                irp->status == USB_HOST_IRP_STATUS_ERROR_BUS)
        {
            //Resend the request
        }
        if (irp->status == USB_HOST_IRP_STATUS_ERROR_STALL)
        {
            //_USB_HOST_ClearEndpointHalt(cdcInstanceInfo->cdcDeviceId, (uint8_t) 0 , _USB_HOST_CDC_ControlTransferComplete );
            //clear the stall
        }
        switch (device->controlRequest)
        {
            case ANDROID_ACCESSORY_SEND_STRING:
                strComplete.length = irp->size;
                strComplete.transferHandle =
                                            (USB_HOST_ANDROID_TRANSFER_HANDLE) irp;
                strComplete.transferStatus =
                                            USB_HOST_ANDROID_TRANSFER_STATUS_ERROR;
                device->appEventCallBack(
                        USB_HOST_ANDROID_EVENT_SEND_STRING_COMPLETE, NULL,
                        device->context);

                break;

            case ANDROID_ACCESSORY_START:

                startComplete.length = irp->size;
                startComplete.transferHandle =
                        (USB_HOST_ANDROID_TRANSFER_HANDLE) irp;
                strComplete.transferStatus =
                        USB_HOST_ANDROID_TRANSFER_STATUS_ERROR;
                device->appEventCallBack(
                        USB_HOST_ANDROID_EVENT_START_COMPLETE, NULL,
                        device->context);

                break;
                
            case ANDROID_ACCESSORY_GET_PROTOCOL:
                startComplete.length = irp->size;
                startComplete.transferHandle =
                        (USB_HOST_ANDROID_TRANSFER_HANDLE) irp;
                strComplete.transferStatus =
                        USB_HOST_ANDROID_TRANSFER_STATUS_ERROR;
                device->appEventCallBack(
                        USB_HOST_ANDROID_EVENT_PROTOCOL_COMPLETE, NULL,
                        device->context);
                break;

        }
    }
}

USB_ANDROID_RETURN AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRINGS stringType, const char *string, size_t stringLength)
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA* device = &device_pv1;
    USB_ERROR status;


    if(device == NULL)
    {
        return USB_ANDROID_UNKNOWN_DEVICE;
    }

    device->setupPacket.bmRequestType  = ( USB_SETUP_DIRN_HOST_TO_DEVICE |
                                                  USB_SETUP_TYPE_VENDOR |
                                                  USB_SETUP_RECIPIENT_DEVICE );
    device->setupPacket.bRequest =  ANDROID_ACCESSORY_SEND_STRING;
    device->setupPacket.wValue =  0;

    device->setupPacket.wIndex = (WORD)stringType;

    device->setupPacket.wLength = stringLength;

    device->controlIRP.data = (uint8_t *) string;
    device->controlIRP.setup = &( device->setupPacket );
    device->controlIRP.size = stringLength;

    device->controlRequest = ANDROID_ACCESSORY_SEND_STRING;

    status = _USB_HOST_IRPSubmit(device->controlPipeHandle, &(device->controlIRP));

    if(status != USB_ERROR_NONE )
    {
        return USB_ANDROID_ERROR_TRANSFER_QUEUE_FULL;
    }
    return USB_ANDROID_RESULT_OK;
}

USB_ANDROID_RETURN AndroidCommandStart_Pv1()
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA* device = &device_pv1;
    USB_ERROR status;

    if(device == NULL)
    {
        return USB_ANDROID_UNKNOWN_DEVICE;
    }

    device->setupPacket.bmRequestType  = ( USB_SETUP_DIRN_HOST_TO_DEVICE |
                                                  USB_SETUP_TYPE_VENDOR |
                                                  USB_SETUP_RECIPIENT_DEVICE );
    device->setupPacket.bRequest =  ANDROID_ACCESSORY_START;
    device->setupPacket.wValue =  0;

    device->setupPacket.wIndex = 0;

    device->setupPacket.wLength = 0;

    /* Data Buffer for get line code */
    device->controlIRP.data = NULL;
    device->controlIRP.setup = &( device->setupPacket );
    device->controlIRP.size = 0;
    /* set the CDC Task states for submit IRP*/
    device->controlRequest = ANDROID_ACCESSORY_START;

    status = _USB_HOST_IRPSubmit(device->controlPipeHandle, &(device->controlIRP));

    if(status != USB_ERROR_NONE )
    {
        return USB_ANDROID_ERROR_TRANSFER_QUEUE_FULL;
    }
    return USB_ANDROID_RESULT_OK;
}

USB_ANDROID_RETURN AndroidGetProtocol_Pv1(uint32_t * protocol)
{
    ANDROID_PROTOCOL_V1_DEVICE_DATA* device = &device_pv1;
    USB_ERROR status;

    if(device == NULL)
    {
        return USB_ANDROID_UNKNOWN_DEVICE;
    }
    if(protocol == NULL)
    {
        return USB_ANDROID_INVALID_BUFFER;
    }

    device->setupPacket.bmRequestType  = ( USB_SETUP_DIRN_DEVICE_TO_HOST |
                                                  USB_SETUP_TYPE_VENDOR |
                                                  USB_SETUP_RECIPIENT_DEVICE );
    device->setupPacket.bRequest =  ANDROID_ACCESSORY_GET_PROTOCOL;
    device->setupPacket.wValue =  0;

    device->setupPacket.wIndex = 0;

    device->setupPacket.wLength = sizeof(uint32_t);

    /* Data Buffer for get line code */
    device->controlIRP.data = protocol;
    device->controlIRP.setup = &( device->setupPacket );
    device->controlIRP.size = sizeof(uint32_t);;
    
    device->controlRequest = ANDROID_ACCESSORY_GET_PROTOCOL;

    status = _USB_HOST_IRPSubmit(device->controlPipeHandle, &(device->controlIRP));

    if(status != USB_ERROR_NONE )
    {
        return USB_ANDROID_ERROR_TRANSFER_QUEUE_FULL;
    }
    return USB_ANDROID_RESULT_OK;
}

void _USB_HOST_Android_Task( uint8_t instanceNumber)
{
}
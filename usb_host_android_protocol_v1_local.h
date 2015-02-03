/*
 * File:   usb_host_android_protocol_v1.c
 * Author: diego.tsutsumi
 *
 * Created on 11th November 2014, 10:12
 */

#ifndef _USB_HOST_ANDROID_V1_
#define _USB_HOST_ANDROID_V1_

#include <limits.h>
#include "includes.h"
#include "system_definitions.h"

//** Control transfer codes **
#define ANDROID_ACCESSORY_GET_PROTOCOL  51
#define ANDROID_ACCESSORY_SEND_STRING   52
#define ANDROID_ACCESSORY_START         53

#define USB_HOST_APP_EVENT_HANDLER APP_USBHostAndroidEventHandler

typedef enum
{
    USB_HOST_ANDROID_EVENT_ATTACH,
    USB_HOST_ANDROID_EVENT_DETACH,
    USB_HOST_ANDROID_EVENT_SEND_STRING_COMPLETE,
    USB_HOST_ANDROID_EVENT_START_COMPLETE,
    USB_HOST_ANDROID_EVENT_PROTOCOL_COMPLETE,
    USB_HOST_ANDROID_EVENT_WRITE_COMPLETE,
    USB_HOST_ANDROID_EVENT_READ_COMPLETE

} USB_HOST_ANDROID_EVENT;

typedef uintptr_t USB_HOST_ANDROID_TRANSFER_HANDLE;
typedef void (*USB_HOST_ANDROID_EVENT_HANDLER)(USB_HOST_ANDROID_EVENT event, uint32_t eventData, uintptr_t context);

typedef struct
{
    bool assigned;

    uint8_t tpl_index;

    USB_HOST_DEVICE_ID androidDeviceId;
    uint8_t interfaceNumber;
    uint8_t datainterfaceNumber;

    USB_HOST_IRP controlIRP;
    USB_HOST_IRP bulkOutIRP;
    USB_HOST_IRP bulkInIRP;

    USB_HOST_PIPE_HANDLE controlPipeHandle;
    USB_HOST_PIPE_HANDLE bulkOutPipeHandle;
    USB_HOST_PIPE_HANDLE bulkInPipeHandle;

    USB_SETUP_PACKET setupPacket;
    uintptr_t context;
    uint8_t   controlRequest;

    USB_HOST_ANDROID_EVENT_HANDLER appEventCallBack;
} ANDROID_PROTOCOL_V1_DEVICE_DATA;


typedef struct
{
    char* manufacturer;         //String: manufacturer name
    BYTE manufacturer_size;     //length of manufacturer string

    char* model;                //String: model name
    BYTE model_size;            //length of model name string

    char* description;          //String: description of the accessory
    BYTE description_size;      //length of the description string

    char* version;              //String: version number
    BYTE version_size;          //length of the version number string

    char* URI;                  //String: URI for the accessory (most commonly a URL)
    BYTE URI_size;              //length of the URI string

    char* serial;               //String: serial number of the device
    BYTE serial_size;           //length of the serial number string
} ANDROID_ACCESSORY_INFORMATION;


//If the user hasn't specified a timeout period, make one up for them
#ifndef ANDROID_DEVICE_ATTACH_TIMEOUT
    #define ANDROID_DEVICE_ATTACH_TIMEOUT 1500
#endif

#define USB_HOST_ANDROID_EVENT_RESPONSE_NONE

typedef enum
{
    USB_ANDROID_UNKNOWN_DEVICE,
    USB_ANDROID_INVALID_BUFFER,
    USB_ANDROID_INVALID_STATE,
    USB_ANDROID_ENDPOINT_BUSY,
    USB_ANDROID_ERROR_TRANSFER_QUEUE_FULL,
    USB_ANDROID_BUFFER_TOO_SMALL,
    USB_HOST_ANDROID_RESULT_ERROR_INSTANCE_NOT_READY,
    USB_ANDROID_RESULT_OK

}USB_ANDROID_RETURN;

typedef enum _ANDROID_ACCESSORY_STRINGS
{
    ANDROID_ACCESSORY_STRING_MANUFACTURER   = 0,
    ANDROID_ACCESSORY_STRING_MODEL          = 1,
    ANDROID_ACCESSORY_STRING_DESCRIPTION    = 2,
    ANDROID_ACCESSORY_STRING_VERSION        = 3,
    ANDROID_ACCESSORY_STRING_URI            = 4,
    ANDROID_ACCESSORY_STRING_SERIAL         = 5
} ANDROID_ACCESSORY_STRINGS;


typedef enum
{
    /* The transfer completed successfully. In case of read transfers, either
       the requested amount of data was received or the device sent less
       data */
    USB_HOST_ANDROID_TRANSFER_STATUS_OK,

    /* The transfer terminated due to an error. The error could be due to device
       malfunction on the bus, a NAK timeout occurred or the device stalled the
       request */
    USB_HOST_ANDROID_TRANSFER_STATUS_ERROR
}
USB_HOST_ANDROID_TRANSFER_STATUS;

typedef struct
{
    /* Transfer handle of this transfer */
    USB_HOST_ANDROID_TRANSFER_HANDLE transferHandle;

    /* Termination transfer status */
    USB_HOST_ANDROID_TRANSFER_STATUS transferStatus;

    /* Amount of data transferred */
    size_t length;
}
USB_HOST_ANDROID_EVENT_ACCESSORY_STRING_COMPLETE,
USB_HOST_ANDROID_EVENT_ACCESSORY_START_COMPLETE,
USB_HOST_ANDROID_EVENT_DATA_READ_COMPLETE,
USB_HOST_ANDROID_EVENT_DATA_WRITE_COMPLETE;


void AndroidAppStart_Pv1(void);
USB_ERROR _USB_HOST_Android_DriverInitialize (USB_HOST_DEVICE_ID hostId,uint8_t *androidInstance, USB_SPEED speed);
//USB_ERROR _USB_HOST_Android_InterfaceInitialize(USB_HOST_DEVICE_ID id, uint8_t interfaceId, uint8_t *androidInstance, USB_SPEED speed);
void  _USB_HOST_Android_DeInitialize ( USB_HOST_DEVICE_ID id  );
void USB_HOST_Android_DeInit();

USB_ANDROID_RETURN AndroidAppWrite_Pv1(BYTE* data, size_t size);
void _USB_HOST_Android_BulkOutTransferComplete( USB_HOST_IRP *irp);
USB_ERROR USB_HOST_Android_EventHandlerSet(USB_HOST_ANDROID_EVENT_HANDLER appAndroidHandler);
USB_ANDROID_RETURN AndroidAppRead_Pv1(BYTE* data, size_t size);
void _USB_HOST_Android_BulkInTransferComplete(USB_HOST_IRP *irp);
USB_ANDROID_RETURN AndroidCommandStart_Pv1();
void _USB_HOST_Android_ControlTransferComplete(USB_HOST_IRP *irp);
USB_ANDROID_RETURN AndroidCommandSendString_Pv1(ANDROID_ACCESSORY_STRINGS stringType,const char *string, size_t stringLength);
USB_ANDROID_RETURN AndroidGetProtocol_Pv1(uint32_t * protocol);
void _USB_HOST_Android_Task( uint8_t instanceNumber);

#endif
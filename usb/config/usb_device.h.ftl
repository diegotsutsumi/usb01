<#--
/*******************************************************************************
  USB Device Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    usb_device.c.ftl

  Summary:
    USB Device Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
-->
/*** USB Device Stack Configuration ***/

<#-- Instance 0 -->
/* Maximum device layer instances */
#define USB_DEVICE_INSTANCES_NUMBER     ${CONFIG_USB_DEVICE_INSTANCES_NUMBER}

/* EP0 size in bytes */
#define USB_DEVICE_EP0_BUFFER_SIZE      ${CONFIG_USB_DEVICE_EP0_BUFFER_SIZE}

<#if CONFIG_USB_DEVICE_SOF_EVENT_ENABLE == true>
#define USB_DEVICE_SOF_EVENT_ENABLE     
</#if>

<#if CONFIG_USB_DEVICE_SET_DESCRIPTOR_EVENT_ENABLE == true>
#define USB_DEVICE_SET_DESCRIPTOR_EVENT_ENABLE
</#if>

<#if CONFIG_USB_DEVICE_SYNCH_FRAME_EVENT_ENABLE == true>
#define USB_DEVICE_SYNCH_FRAME_EVENT_ENABLE
</#if>

<#if CONFIG_USB_DEVICE_USE_AUDIO == true>
/* Maximum instances of Audio function driver */
#define USB_DEVICE_AUDIO_INSTANCES_NUMBER  ${CONFIG_USB_DEVICE_AUDIO_NUMBER_OF_INSTANCES}

/* Audio Queue Depth Combined */
#define USB_DEVICE_AUDIO_QUEUE_DEPTH_COMBINED     ${CONFIG_USB_DEVICE_AUDIO_QUEUE_DEPTH_COMBINED}

/* No of Audio streaming interfaces */
#define USB_DEVICE_AUDIO_MAX_STREAMING_INTERFACES   ${CONFIG_USB_DEVICE_AUDIO_MAX_STREAMING_INTERFACES}

/* No of alternate settings */
#define USB_DEVICE_AUDIO_MAX_ALTERNATE_SETTING      ${CONFIG_USB_DEVICE_AUDIO_MAX_ALTERNATE_SETTING}

</#if>
<#if CONFIG_USB_DEVICE_USE_CDC == true>
/* Maximum instances of CDC function driver */
#define USB_DEVICE_CDC_INSTANCES_NUMBER     ${CONFIG_USB_DEVICE_CDC_NUMBER_OF_INSTANCES}

/* CDC Transfer Queue Size for both read and
   write. Applicable to all instances of the
   function driver */
#define USB_DEVICE_CDC_QUEUE_DEPTH_COMBINED ${CONFIG_USB_DEVICE_CDC_QUEUE_DEPTH_COMBINED}

</#if>
<#if CONFIG_USB_DEVICE_USE_HID == true>
/* Maximum instances of HID function driver */
#define USB_DEVICE_HID_INSTANCES_NUMBER     ${CONFIG_USB_DEVICE_HID_NUMBER_OF_INSTANCES}

/* HID Transfer Queue Size for both read and
   write. Applicable to all instances of the
   function driver */
#define USB_DEVICE_HID_QUEUE_DEPTH_COMBINED ${CONFIG_USB_DEVICE_HID_QUEUE_DEPTH_COMBINED}

</#if>
<#if CONFIG_USB_DEVICE_USE_MSD == true>
/* Number of MSD Function driver instances in the application */
#define USB_DEVICE_MSD_INSTANCES_NUMBER ${CONFIG_USB_DEVICE_MSD_NUMBER_OF_INSTANCES}

/* Number of Logical Units */
#define USB_DEVICE_MSD_LUNS_NUMBER      ${CONFIG_USB_DEVICE_MSD_LUN_NUMBER}

/* Size of disk image (in KB) in Program Flash Memory */
#define DRV_NVM_BLOCK_MEMORY_SIZE       36

</#if>
<#if CONFIG_USB_DEVICE_USE_ENDPOINT_FUNCTIONS == true>
/* Endpoint Transfer Queue Size combined for Read and write */
#define USB_DEVICE_ENDPOINT_QUEUE_DEPTH_COMBINED    ${CONFIG_USB_DEVICE_ENDPOINT_QUEUE_SIZE_READ?number + CONFIG_USB_DEVICE_ENDPOINT_QUEUE_SIZE_WRITE?number}

</#if>
<#--
/*******************************************************************************
 End of File
*/
-->


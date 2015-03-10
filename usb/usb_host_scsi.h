 /*******************************************************************************
  USB CDC class definitions

  Company:
    Microchip Technology Inc.

  File Name:
    usb_host_cdc_local.h

  Summary:
    USB CDC class definitions

  Description:
    This file describes the CDC class specific definitions.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
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
// DOM-IGNORE-END


#ifndef _USB_HOST_SCSI_H
#define _USB_HOST_SCSI_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "usb/usb_host_msd.h"
#include "system/fs/sys_fs_media_manager.h"

// *****************************************************************************
/*  USB Host SCSI subClass Driver  Handle

  Summary:
     USB Host SCSI Class Driver  Handle

  Description:

  Remarks:
    None.
*/

typedef uintptr_t USB_HOST_SCSI_HANDLE;

/*****************************************************************************
*  USB Host scsi Class Driver Invalid Transfer Handle Definition
*****************************************************************************/

#define USB_HOST_SCSI_HANDLE_INVALID /*DOM-IGNORE-BEGIN*/((USB_HOST_SCSI_HANDLE)(-1))/*DOM-IGNORE-END*/

typedef uintptr_t USB_HOST_SCSI_CLIENT_OBJ_HANDLE ;
typedef uintptr_t USB_HOST_SCSI_BUFFER_HANDLE;

/*****************************************************************************
*  SCSI command status
*****************************************************************************/

typedef enum
{
    USB_HOST_SCSI_CMD_SUCCESS = 0 ,
    USB_HOST_SCSI_CMD_PENDING ,
    USB_HOST_SCSI_CMD_FAIL ,
    USB_HOST_SCSI_CMD_INVALID
} USB_HOST_SCSI_CMD_STATES ;

/*****************************************************************************
*  Media status
*****************************************************************************/
typedef enum
{
    /* Client in an invalid state */
    USB_HOST_SCSI_MEDIA_STATUS_INVALID = -1 ,
    /* Unspecified error condition */
    USB_HOST_SCSI_MEDIA_STATUS_ERROR = -2,
    /* Client is not open */
    USB_HOST_SCSI_MEDIA_STATUS_CLOSED = -3,
    /* An operation is currently in progress */
    USB_HOST_SCSI_MEDIA_STATUS_BUSY = -4,
    /* Up and running, no operations running */
    USB_HOST_SCSI_MEDIA_STATUS_READY  = 0,
    USB_HOST_SCSI_MEDIA_STATUS_STOPPED = -5

}USB_HOST_SCSI_MEDIA_STATUS;


typedef struct
{
    uint8_t  instanceNumber;
    uint8_t  luns;

}USB_HOST_SCSI_INIT;



//******************************************************************************
/* Function:
SYS_MODULE_OBJ USB_HOST_SCSI_Initialize ( const SYS_MODULE_INDEX    drvIndex,
                                      const SYS_MODULE_INIT     * const init )

  Summary:
    Initializes SCSI and register with Filesystem ,MSD module

  Description:


  Parameters:
    drvIndex        - Identifies the driver instance to be initialized.

    init            - Pointer to the data structure containing all data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and static initialization
                      values are to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/

SYS_MODULE_OBJ USB_HOST_SCSI_Initialize ( const SYS_MODULE_INDEX    drvIndex,
                                      const SYS_MODULE_INIT     * const init );


//******************************************************************************
/* Function:
SYS_MODULE_OBJ USB_HOST_SCSI_DeInitialize ( const SYS_MODULE_INDEX    drvIndex,
                                      const SYS_MODULE_INIT     * const init )

  Summary:
    DeInitializes SCSI and unregister with Filesystem .

  Description:


  Parameters:
    drvIndex        - DeIdentifies the driver instance to be initialized.

    init            - Pointer to the data structure containing all data
                      necessary to deinitialize the FS.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/

SYS_MODULE_OBJ USB_HOST_SCSI_DeInitialize ( const SYS_MODULE_INDEX    drvIndex,
                                      const SYS_MODULE_INIT     * const init );

// *****************************************************************************
/* Function:
    void USB_HOST_SCSI_Task( SYS_MODULE_OBJ obj)

  Summary:
    Scsi task will call the USB MSD Task

  Description:
    

  Precondition:
   File system will call this function

  Parameters:
    object      - Object handle for the specified scsi instance (returned from
                  USB_HOST_SCSI_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from USB_HOST_Initialize

    
        USB_HOST_Tasks (object);

        // Do other tasks
    
    </code>

  Remarks:
    This routine is normally not called directly by File System.
*/
void USB_HOST_SCSI_Task( SYS_MODULE_OBJ obj);

//******************************************************************************
/* Function:
   USB_HOST_SCSI_HANDLE USB_HOST_SCSI_Open( const SYS_MODULE_INDEX index,
                                         const DRV_IO_INTENT    intent )

  Summary:
    Opens the specific module instance and returns a handle

  Description:
    This routine opens a driver for use by any client module and provides a
    handle that must be provided to any of the other driver operations to
    identify the caller and the instance of the driver module.

  Parameters:
    drvIndex        - Identifier for the instance to be initialized
    ioIntent        - Possible values from the enumeration DRV_IO_INTENT

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance)
    If an error occurs, the return value is DRV_HANDLE_INVALID
*/

USB_HOST_SCSI_HANDLE USB_HOST_SCSI_Open( const SYS_MODULE_INDEX index,
                                         const DRV_IO_INTENT    intent );

//*****************************************************************************
/* Function:
SYS_FS_MEDIA_BUFFER_STATUS USB_HOST_SCSI_BufferStatusGet ( DRV_HANDLE handle,
					SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle )

  Summary:
    Gets the status of USB MSD command operation (read/write)

  Description:
	This function gets the status of USB MSD command operation.
	To be called only after a read or write is scheduled.

  Precondition:
      USB_HOST_SCSI_Open must have been called to obtain a valid opened device
	handle.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

	bufferHandle - Handle returned by a 'sector write' or a 'sector read'
				function.

  Returns:
    SYS_FS_MEDIA_BUFFER_STATUS - Buffer status.

  Example:
    <code>
    int globalState = 0;
    SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle;

	switch (globalState)
	{
		case 0:
			bufferHandle = SYS_FS_MEDIA_MANAGER_SectorWrite ( handle,
									,readData //buffer
									, 20 // Sector
									, 1 // Number of Sectors
									);
			globalState++;
			break;
		case 1:
			if ( SYS_FS_MEDIA_MANAGER_BufferStatusGet(handle, bufferHandle) ==
						SYS_FS_MEDIA_BUFFER_COMPLETED)
			{
				//Write complete
			}
			break;
    </code>

  Remarks:
	None.
*/

SYS_FS_MEDIA_BUFFER_STATUS USB_HOST_SCSI_BufferStatusGet ( DRV_HANDLE handle,
					SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle );
					
					
// *****************************************************************************
/* Function:
	SYS_FS_MEDIA_STATUS USB_HOST_SCSI_MediaStatusGet ( DRV_HANDLE handle )

  Summary:
    Gets the status of the MSD device.

  Description:
	This function gets the status of the MSD device ( attached/detached ).

  Precondition:
    The USB_HOST_SCSI_Initialize routine must have been called.

    USB_HOST_SCSI_Open must have been called to obtain a valid opened device
	handle.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

  Returns:
    SYS_FS_MEDIA_STATUS - Status of the device.

  Example:
    <code>
    DRV_HANDLE sdcardHandle;  // Returned from DRV_SDCARD_Open

    if( USB_HOST_SCSI_MediaStatusGet ( handle ) == SYS_FS_MEDIA_ATTACHED )
	{
		//Device is attached
	}

    </code>

  Remarks:
	None.

*/
SYS_FS_MEDIA_STATUS USB_HOST_SCSI_MediaStatusGet ( DRV_HANDLE handle );

// *****************************************************************************
/* Function:
   USB_HOST_SCSI_BUFFER_HANDLE USB_HOST_SCSI_SectorRead ( USB_HOST_SCSI_HANDLE handle,
                                                        uint8_t *buffer,
                                                        uint32_t sectorAddr,
                                                        uint32_t sectorCount )
  Summary:
    Reads data from the sectors of the MSD device.

  Description:
	This function reads data from the sectors of the MSD device starting from
        the sector address and stores it in the location pointed by 'buffer'.

  Precondition:
    The USB_HOST_SCSI_Initialize routine must have been called.

    USB_HOST_SCSI_Open must have been called to obtain a valid opened device
	handle.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

	sector_addr - The address of the sector on the card.

	sectorCount - Number of sectors to be read.

	buffer -  The buffer where the retrieved data will be stored.  If
				buffer is NULL, do not store the data anywhere.

  Returns:
    SYS_FS_MEDIA_BUFFER_HANDLE - Buffer handle.

  Example:
    <code>
    SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle;

    bufferHandle = DRV_SDCARD_SectorRead ( handle,readData //buffer
 					, 20 //  Sector
        				, 1 // Number of Sectors
					);

    </code>

  Remarks:
    
*/
USB_HOST_SCSI_BUFFER_HANDLE USB_HOST_SCSI_SectorRead ( USB_HOST_SCSI_HANDLE handle,
                                                        uint8_t *buffer,
                                                        uint32_t sectorAddr,
                                                        uint32_t sectorCount );
														
														// *****************************************************************************
/* Function:
USB_HOST_SCSI_BUFFER_HANDLE USB_HOST_SCSI_SectorWrite ( DRV_HANDLE handle,
                                                        uint32_t sectorAddr,
                                                        uint8_t *buffer,
                                                        uint32_t sectorCount )

  Summary:
    Writes sector(s) of data to MSD.

  Description:
	This function writes sector(s) of data (512 bytes) of data from the
	location pointed to by 'buffer' to the specified sector of the MSD.

  Precondition:
    The USB_HOST_SCSI_Initialize routine must have been called.

    USB_HOST_SCSI_Open must have been called to obtain a valid opened device
	handle.

  Parameters:
	handle - A valid open-instance handle, returned from the driver's
				open routine

	sector_addr - The address of the sector on the MSD.

	buffer -  The buffer with the data to write.

  Returns:
    SYS_FS_MEDIA_BUFFER_HANDLE - Buffer handle.

  Example:
    <code>
    SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle;

	bufferHandle = DRV_SDCARD_SectorWrite ( handle, readData //buffer
						, 20 // Sector
					        , 1 // Number of Sectors
						);
    </code>

  Remarks:
	

*/
USB_HOST_SCSI_BUFFER_HANDLE USB_HOST_SCSI_SectorWrite ( DRV_HANDLE handle,
                                                        uint32_t sectorAddr,
                                                        uint8_t *buffer,
                                                        uint32_t sectorCount );
														// *****************************************************************************
/* Function:
    void USB_HOST_SCSI_Close(USB_HOST_SCSI_HANDLE handle )

  Summary:
    Closes an opened-instance of the SCSI instance

  Description:
    This routine closes an opened-instance of the MSD, invalidating
    the handle.

  Precondition:
    The USB_HOST_SCSI_Initialize routine must have been called for the specified
    Scsi driver instance.

    USB_HOST_SCSI_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    USB_HOST_SCSI_HANDLE handle;  // Returned from DRV_SDCARD_Open

    USB_HOST_SCSI_Close ( handle );
    </code>

  Remarks:
  
*/

void USB_HOST_SCSI_Close(USB_HOST_SCSI_HANDLE handle );



#endif



 /************ End of file *************************************/

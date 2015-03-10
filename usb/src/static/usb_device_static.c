/**************************************************************************
 USB Device Layer Implementation

  Company:
    Microchip Technology Inc.
    
  File Name:
    usb_device_static.c
    
  Summary:
    This file contains implementations of both the private and public functions
    of the USB device layer.
    
  Description:
    This file contains the USB device layer implementation.
**************************************************************************/

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
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "usb/src/usb_device_local.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Device layer instance objects.

  Summary:
    This structure holds all the varaiables required by the instance.

  Description:
    
  Remarks:
    
*/

static USB_DEVICE_INSTANCE_STRUCT           usbDeviceInstance ;


// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the client instances objects that are available on the device.

  Description:
    This data type defines the client instance objects that are available on
    the device, so as to capture the Client state of the instance.

  Remarks:
    None.
*/

static USB_DEVICE_CLIENT_STRUCT             usbDeviceClient;


// *****************************************************************************
/* Macro: DEVICE

  Summary:
    Helps in creating the non-volatile memory section.

  Description:
    This macro helps in creating the non-volatile memory section.
*/

#define DEVICE                              __attribute__( ( section( "Device" ) ) )


// *****************************************************************************
/* Function:
    static USB_ERROR_STATUS _USB_DEVICE_InitializeFunctionDrivers(uint16_t configValue)

  Summary:
    Initializes the correct function drivers.
    
  Description:
    This function initializes the correct function drivers.
    
  Precondition:
    None.

  Parameters:
    configValue          -   Configuration value requested by the host
    usbDeviceInstance    -   This instance of the USB device layer.

  Returns:
    Error status.

  Example:
    <code>
    
    </code>

  Remarks:
    
*/    

static USB_ERROR_STATUS _USB_DEVICE_InitializeFunctionDrivers( uint16_t configValue );


// *****************************************************************************
/* Function:
    static void _USB_DEVICE_ProcessSetupPacket(SETUP_PKT* setupPkt,
                                USB_DEVICE_INSTANCE_STRUCT* usbDeviceInstance)


  Summary:
    Processes the setup packet.
    
  Description:
    This function processes the setup packet.

  Precondition:
    None.

  Parameters:
    setupPkt             -   Pointer to the setup packet
    usbDeviceInstance    -   This instance of the USB device layer

  Returns:
    None.

  Example:
    <code>
    
    </code>

  Remarks:
    None.
    
*/

static void _USB_DEVICE_ProcessSetupPacket( SETUP_PKT* setupPkt );


// *****************************************************************************
/* Function:
    static void _USB_DEVICE_ProcessStandardSetRequests(SETUP_PKT* setupPkt,
                                USB_DEVICE_INSTANCE_STRUCT* usbDeviceInstance)


  Summary:
    Processes the standard "set" requests.
    
  Description:
    

  Precondition:
    None.

  Parameters:
    setupPkt             -   Pointer to the setup packet.
    
    usbDeviceInstance    -   This instance of the USB device layer.

  Returns:
    none. 

  Example:
    <code>
    
    </code>

  Remarks:
    
*/

static void _USB_DEVICE_ProcessStandardSetRequests( SETUP_PKT *setupPkt );


// *****************************************************************************
/* Function:
   static void _USB_DEVICE_ProcessStandardGetRequests(SETUP_PKT* setupPkt,
                                USB_DEVICE_INSTANCE_STRUCT* usbDeviceInstance)


  Summary:
    Processes the standard "get" requests.
    
  Description:
    This function processes the standard "get" requests.

  Precondition:
    None.

  Parameters:
    setupPkt             -   Pointer to the setup packet
    usbDeviceInstance    -   This instance of the USB device layer

  Returns:
    None.

  Example:
    <code>
    
    </code>

  Remarks:
    None.
    
*/

static void _USB_DEVICE_ProcessStandardGetRequests( SETUP_PKT* setupPkt );


// *****************************************************************************
/* Function:
    static void _USB_DEVICE_ProcessOtherRequests( SETUP_PKT* setupPkt )


  Summary:
    Processes "other" requests.
    
  Description:
    This function processes "other" requests.

  Precondition:
    None.

  Parameters:
    setupPkt             -   Pointer to the setup packet
    usbDeviceInstance    -   This instance of the USB device layer

  Returns:
    None.

  Example:
    <code>
    
    </code>

  Remarks:
    None.
    
*/

static void _USB_DEVICE_ProcessOtherRequests( SETUP_PKT* setupPkt );


/******************************************************************************
  Function:
    static void _USB_DEVICE_DeInitializeAllFuncDrivers
                            ( USB_DEVICE_INSTANCE_STRUCT* usbDeviceInstance )

  Summary:
    Deinitializes all function drivers.

  Description:
    This function deinitializes all function drivers.

  Parameters:
    usbDeviceInstance - USB device instance

  Returns:
    None.

  Example:
    <code>

    </code>

  Remarks:
    None.
*/

static void _USB_DEVICE_DeInitializeAllFuncDrivers ( void );


/******************************************************************************
  Function:
    static void _USB_DEVICE_BroadcastEventToClients
       ( USB_DEVICE_EVENT event, USB_DEVICE_INSTANCE_STRUCT* usbDeviceInstance )

  Summary:
    Broadcasts events to the application client.

  Description:
    This function broadcasts device layer events to all application
    level clients.

  Parameters:
    event - Device layer event
    usbDeviceInstance - USB device instance

  Returns:
    None.

  Example:
    <code>

    </code>

  Remarks:
    None.
*/

static void _USB_DEVICE_BroadcastEventToClients( USB_DEVICE_EVENT event );


/******************************************************************************
  Function:
    SYS_MODULE_OBJ USB_DEVICE_Initialize(const SYS_MODULE_INDEX index, 
                                       const SYS_MODULE_INIT * const initData)

  Summary:
    USB device layer initialization function.

  Description:
    This function initializes the required state machines for the USB device layer.

  Remarks:
    None.
*/

DEVICE void _USB_DEVICE_MAKE_NAME( Initialize )( const SYS_MODULE_INIT * const initData )
{
    USB_DEVICE_INIT *deviceInit; 
         
    // Copy init data to local variable.
    deviceInit = ( USB_DEVICE_INIT * )initData;
    
    // Make sure that initData is not NULL.
    SYS_ASSERT( ( deviceInit != NULL ) ,"InitData passed is NULL" );
    
     // Make sure that master descriptor table is not NULL.
    SYS_ASSERT( ( deviceInit->usbMasterDescriptor != NULL ) ,"USB master descriptor table passed is NULL" );
      
    // Set the device state to unconfigured.
    usbDeviceInstance.usbDeviceState = USB_DEVICE_STATE_NOT_CONFIGURED;
    
    // Initialize the instance structure.
    usbDeviceInstance.ptrMasterDescTable = deviceInit->usbMasterDescriptor;

    // Make sure that master descriptor table is not NULL.
    SYS_ASSERT( ( deviceInit->registeredFuncCount == 1 ) ,"Driver supports only one function" );

    /* Only one function driver supported at any point of time */
    usbDeviceInstance.registeredFuncDriverCount = deviceInit->registeredFuncCount;
    usbDeviceInstance.registeredFuncDrivers = deviceInit->registeredFunctions;
    
    // Reset set address flag.
    usbDeviceInstance.setAddressPending = false;
                                      
    // Invalidate the control endpipe handle. Control endpipe will be initialized
    // after reset.
    usbDeviceInstance.ctrlEpInHandle = DRV_HANDLE_INVALID;

    //Initialize this instance.
    usbDeviceInstance.usbDeviceInstanceState = SYS_STATUS_READY; 
   
} /* USB_DEVICE_Initialize */


// *****************************************************************************
/* Function:
    void USB_DEVICE_Reinitialize( SYS_MODULE_OBJ usbDeviceObj,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes the USB device layer

  Description:
    This function reinitializes the USB device layer.

  Parameters:
    usbDeviceObj    - Driver object handle, returned by USB_DEVICE_Initialize
                      routine

    init            - Pointer to a data structure containing any data necessary
                      to reinitialize the USB device layer.

  Returns:
    None.
*/

DEVICE void _USB_DEVICE_MAKE_NAME( Reinitialize ) ( const SYS_MODULE_INIT * const init )
{
    // Check if this instance is initialized already.
    SYS_ASSERT( ( usbDeviceInstance.usbDeviceInstanceState > SYS_STATUS_UNINITIALIZED ),
                               " This USB device instance is not yet initialized ");

    // Go for re-initialization.
    _USB_DEVICE_MAKE_NAME( Initialize )( init );

} /* USB_DEVICE_Reinitialize */


// *****************************************************************************
/* Function:
    void USB_DEVICE_Deinitialize ( SYS_MODULE_OBJ usbDeviceobj)

  Summary:
    Deinitializes the specified instance of the USB device layer.

  Description:
    This function deinitializes the specified instance of the USB device layer, 
    disabling its operation (and any hardware) and invalidates all of the internal data.

  Parameters:
    usbDeviceObj    - USB Device Layer object handle, returned by USB_DEVICE_Initialize

  Returns:
    None.

*/

DEVICE void _USB_DEVICE_MAKE_NAME( Deinitialize )( void )
{
    // Just invalidate the device state.
    usbDeviceInstance.usbDeviceInstanceState =  SYS_STATUS_UNINITIALIZED;

    // Close the handle to control endpoint (EP0)
    DRV_USB_Device_PipeRelease(	usbDeviceInstance.ctrlEpInHandle );
    
} /* USB_DEVICE_Deinitialize */


// *****************************************************************************
/* Function:
    SYS_STATUS USB_DEVICE_Status( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the USB device layer

  Description:
    This function provides the current status of the USB device layer.

  Precondition:
    The USB_DEVICE_Initialize function must have been called before calling this
    function.

  Parameters:
    object  - Driver object handle, returned from the USB_DEVICE_Initialize routine

  Returns:
    SYS_STATUS_READY          - Indicates that the device is busy with a
                                previous system level operation and cannot start
                                another

                                Note: Any value greater than SYS_STATUS_READY is
                                also a normal running state in which the device
                                is ready to accept new operations.

    SYS_STATUS_BUSY           - Indicates that the device is busy with a
                                previous system level operation and cannot start
                                another

    SYS_STATUS_UNINITIALIZED  - Indicates that the device has never been initialized

    SYS_STATUS_ERROR          - Indicates that the device is in an error state

                                Note:  Any value less than SYS_STATUS_ERROR is
                                also an error state.
*/

DEVICE SYS_STATUS _USB_DEVICE_MAKE_NAME( Status )( void )
{
   return( usbDeviceInstance.usbDeviceInstanceState );

} /* USB_DEVICE_Status */


/******************************************************************************
  Function:
    void USB_DEVICE_Tasks( SYS_MODULE_OBJ devLayerObj )

  Summary:
    USB Device layer Tasks

  Description:
    This function internally calls all USB Device layer function driver
    tasks.

  Remarks:
    None.
*/

DEVICE void _USB_DEVICE_MAKE_NAME( Tasks )( void )
{
    USB_DEVICE_EVENT event;
    uint8_t count;

    // Proceed only if this instance is in initialized state.
    if( usbDeviceInstance.usbDeviceInstanceState == SYS_STATUS_UNINITIALIZED )
    {
        // Instance is not yet initialized. Just do a return.
        return;
    }

    // Copy event (work on the copy, since it is never known when an interrupt
    // occurs and updates the event.
    event = usbDeviceInstance.event;

    // Broadcast event to all application clients.
    if( event != 0 )
    {
        _USB_DEVICE_BroadcastEventToClients( event );
    }

    if( ( event == USB_DEVICE_EVENT_RESET ) ||
        ( event == USB_DEVICE_EVENT_DECONFIGURED ) ||
        ( event == USB_DEVICE_EVENT_DETACHED ) )
    {
        // Deinitialize previously loaded function drivers.
        _USB_DEVICE_DeInitializeAllFuncDrivers();
    }

    if( usbDeviceInstance.usbDeviceState == USB_DEVICE_STATE_CONFIGURED )
    {
        // Handle function driver tasks only if device is configured.
        for(count = 0; count < usbDeviceInstance.functionDriverCount; count++)
        {
            if(usbDeviceInstance.functionDriver[count].driver->tasks != NULL)
            {
                // Call function driver tasks
                usbDeviceInstance.functionDriver[count].driver->tasks();
            }
        }
    }

    // During this time, check if ISR event has updated the original flag.
    if( event == usbDeviceInstance.event )
    {
        // There is no new event occured. Reset the event to 0.
        usbDeviceInstance.event = 0;
    }

} /* USB_DEVICE_Tasks */


/******************************************************************************
  Function:
    DRV_HANDLE USB_DEVICE_Open( const SYS_MODULE_INDEX index,
                                const DRV_IO_INTENT intent )

  Summary:
    Opens the specific module instance and returns a handle.

  Description:
    This function opens the USB device layer for use by any client module and
    provides a handle that must be provided to any of the other device layer 
    operations to identify the caller and the instance of the 
    driver/hardware module.

  Parameters:
    index           - Identifier for the instance to be initialized
   
  Returns:
    If successful, the function returns a valid open-instance handle.
    If an error occurs, the return value is DRV_HANDLE_INVALID.
*/

DEVICE void _USB_DEVICE_MAKE_NAME( Open )( const DRV_IO_INTENT intent )
{
    if( intent & ( DRV_IO_INTENT_BLOCKING | DRV_IO_INTENT_EXCLUSIVE ) )
    {
        /* The driver only supports this mode */
        SYS_ASSERT( false, "Device layer supports non blocking and shared access only" );
        return; /* Just return back */
    }

    // Check if the instance is initialized.
    SYS_ASSERT( ( usbDeviceInstance.usbDeviceInstanceState > SYS_STATUS_UNINITIALIZED ),
                         "USB_DEVICE_Initialize() is not yet called for this instance");

    // Check for a free handle.
    if( usbDeviceClient.clientState == DRV_CLIENT_STATUS_CLOSED )
    {
        // Ready the client.
        usbDeviceClient.clientState = DRV_CLIENT_STATUS_READY;
    }

} /* USB_DEVICE_Open */


// *****************************************************************************
/* Function:
    void USB_DEVICE_Close( DRV_HANDLE usbDevHandle )

  Summary:
    Closes an opened instance of the USB device layer.

  Description:
    This function closes an opened instance of the USB device layer, invalidating the
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from USB_DEVICE_Open

  Returns:
    None.


*/

DEVICE void _USB_DEVICE_MAKE_NAME( Close )( void )
{
    // close the handle.
    usbDeviceClient.clientState = DRV_CLIENT_STATUS_CLOSED;

} /* USB_DEVICE_Close */


// *****************************************************************************
/* Function:
    DRV_CLIENT_STATUS USB_DEVICE_ClientStatus( USB_DEVICE_HANDLE usbDevHandle )

  Summary:
    Gets the current client-specific status of the USB device layer.

  Description:
    This function gets the client-specfic status of the USB device layer associated
    with the specified handle.

  Precondition:
    The USB_DEVICE_Initialize routine must have been called. USB_DEVICE_Open must 
    have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from USB_DEVICE_Open

  Returns:
    A value of enum type DRV_CLIENT_STATUS describing the current status of 
    the USB device layer.
*/

DEVICE DRV_CLIENT_STATUS _USB_DEVICE_MAKE_NAME( ClientStatus )( void )
{
    // Return the state of the client.
    return( usbDeviceClient.clientState );

} /* USB_DEVICE_ClientStatus */


/******************************************************************************
  Function:
    USB_ERROR_STATUS USB_DEVICE_EventCallBackSet(DRV_HANDLE hHandle, 
                                       const USB_DEVICE_CALLBACK *callBackFunc)

  Summary:
    Sets up the callback function that will be called in case of an
    event from the USB device layer.

  Description:
    This function sets up the callback funtion. This callback function 
    will be called when an event occurs in the USB device layer.

  Parameters:
    hHandle           - Device layer handle that is returned USB_DEVICE_Open
                        
    callBackFunc      - Call back function that should be called in case
                        of an event
   
  Returns:
    If successful, this function returns the error status "USB_ERROR_OK".
*/

DEVICE USB_ERROR_STATUS _USB_DEVICE_MAKE_NAME( EventCallBackSet )( const USB_DEVICE_CALLBACK callBackFunc )
{       
    // Check if this handle is in a ready state.
    if( usbDeviceClient.clientState == DRV_CLIENT_STATUS_READY )
    {    
        // Register the callback function.
        usbDeviceClient.callBackFunc = callBackFunc;
        
        // Return OK
        return( USB_ERROR_OK );
    }   
    else
    {
        // Return failed
        return( USB_ERROR_FAILED );
    }

} /* USB_DEVICE_EventCallBackSet */


/******************************************************************************
  Function:
    DRV_USB_DEVICE_XFER_STATUS USB_DEVICE_ControlTransferRequest( SYS_MODULE_INDEX usbDevIndex,
                                                                  uint8_t * controlDataBuffer,
                                                                  uint16_t controlDataCount,
                                                                  DRV_USB_XFER_TYPE xferType,
                                                                  void * hClient,
                                                                  const USB_DEVICE_CONTROL_TX_STATUS_CB controlTxStatusCallback)

  Summary:
    Sets up a control transfer request.

  Description:
    This function will set up a control transfer request

  Parameters:
    usbDevIndex       - ID that identifies this instance of USB device layer
    
    controlDataBuffer - Buffer which holds the data for control read or write
    
    controlDataCount  - Buffer byte count
    
    xferType          - Transfer type that specifies the operation,
                        control read or control write
  
    hClient           - Reference handle of the calling client. This handle 
                        will be passed back as parameter of the callback 
                        function.
                        
    controlTxStatusCallback      - Call back function that will be called 
                                   after the completion of the control transfer
   
  Returns:
     USB transfer handle.
*/

DEVICE DRV_USB_XFER_HANDLE _USB_DEVICE_MAKE_NAME( ControlTransferRequest )( uint8_t * controlDataBuffer,
                                                                     uint16_t controlDataCount,
                                                                     DRV_USB_XFER_TYPE xferType,
                                                                     void * refData,
                                                                     const DRV_USB_XFER_COMPLETE_CALLBACK controlTxStatusCallback )
{
    // Check if the instance is initialized.   
    SYS_ASSERT( ( usbDeviceInstance.usbDeviceInstanceState > SYS_STATUS_UNINITIALIZED ),
                         "USB_DEVICE_Initialize() is not yet called for this instance");
    // Validate the transfer type.                     
    SYS_ASSERT( ( ( xferType == USB_XFER_CONTROL_READ ) || ( xferType == USB_XFER_CONTROL_WRITE ) ),
                         " Transfer type must be either control read or control write " );
    
    // Request the transfer into the controller driver.
    return( DRV_USB_Device_TransferRequest( usbDeviceInstance.ctrlEpInHandle,
                                            xferType,
                                            ( void * )controlDataBuffer,
                                            controlDataCount,
                                            false,
                                            refData,
                                            controlTxStatusCallback ) );

} /* USB_DEVICE_ControlTransferRequest */


/******************************************************************************
  Function:
    DRV_USB_DEVICE_XFER_STATUS USB_DEVICE_ControlTransferStatusGet
                                            (DRV_USB_XFER_HANDLE hTransfer)

  Summary:
    Returns the status of the control transfer.

  Description:
    This function returns the status of the control transfer.

  Parameters:
    hTransfer       - Transfer handle returned by USB_DEVICE_ControlTransferRequest
    
   
  Returns:
    Transfer status.
*/

DEVICE DRV_USB_DEVICE_XFER_STATUS USB_DEVICE_ControlTransferStatusGet( DRV_USB_XFER_HANDLE hTransfer )
{
    return( DRV_USB_Device_TransferStatusGet( hTransfer ) );

} /* USB_DEVICE_ControlTransferStatusGet */


/******************************************************************************
  Function:
    void USB_DEVICE_ControlTransferPipeStall( SYS_MODULE_INDEX usbDevIndex , 
                                              USB_EP_TXRX pipeDirection )

  Summary:
    Stalls the TX/RX control pipe. The parameter "pipeDirection" specifies
    the direction of the pipe.

  Description:
     

  Parameters:
    usbDevIndex     - ID that identifies this instance of the USB device layer
    pipeDirection   - Direction of the pipe (EP_TX/EP_RX)
    
   
  Returns:
    none.
*/

DEVICE void _USB_DEVICE_MAKE_NAME( ControlTransferPipeStall )( USB_EP_TXRX pipeDirection )
{
    // Check if the instance is initialized.   
    SYS_ASSERT( ( usbDeviceInstance.usbDeviceInstanceState > SYS_STATUS_UNINITIALIZED ),
                         "USB_DEVICE_Initialize() is not yet called for this instance"); 
    
    if( pipeDirection == USB_EP_TX )
    {
        // Remember that we are stalling this TX endpoint. 
        // Next setup token, we will have to clear the stall. 
        usbDeviceInstance.ctrlEpInStalled = true; 
    }      
    
    // Stall the endpoint.
    DRV_USB_Device_PipeStall( usbDeviceInstance.ctrlEpInHandle, pipeDirection );

} /* USB_DEVICE_ControlTransferPipeStall */


/******************************************************************************
  Function:
   USB_DEVICE_STATE USB_DEVICE_GetDeviceState( DRV_HANDLE hHandle )

  Summary:
    Gets the USB device state.

  Description:
    This function gets the USB device state.

  Parameters:
    hHandle  - Client handle that is returned by USB_DEVICE_Open
    
  Returns:
    USB device state.
*/

DEVICE USB_DEVICE_STATE _USB_DEVICE_MAKE_NAME( GetDeviceState )( void )
{
    return( usbDeviceInstance.usbDeviceState );

} /* USB_DEVICE_GetDeviceState */


/******************************************************************************
  Function:
    uint16_t USB_DEVICE_GetConfigurationValue( DRV_HANDLE hHandle )

  Summary:
    Gets the configuration selected by the Host.

  Description:
    This function gets the current active USB configuration.

  Parameters:
    hHandle - Client handle that is returned by USB_DEVICE_Open
   
  Returns:
    Present active configuration of the device.
*/

DEVICE uint16_t _USB_DEVICE_MAKE_NAME( GetConfigurationValue )( void )
{
    return( usbDeviceInstance.activeConfiguration );

} /* USB_DEVICE_GetConfigurationValue */


/******************************************************************************
  Function:
    DRV_USB_SPEED USB_DEVICE_GetDeviceSpeed( DRV_HANDLE hHandle )

  Summary:
    Gets the speed on which the device has enumerated.

  Description:
    This function gets the speed on which the device has enumerated.

  Parameters:
    hHandle - Client handle that is returned by USB_DEVICE_Open
   
  Returns:
    USB speed.
*/

DEVICE DRV_USB_SPEED _USB_DEVICE_MAKE_NAME( GetDeviceSpeed )( void )
{
    return( usbDeviceInstance.usbSpeed );

} /* USB_DEVICE_GetDeviceSpeed */


/******************************************************************************
  Function:
    void USB_DEVICE_EventHandler( void                  * referenceHandle,
                                  DRV_USB_EVENT         eventType,
                                  DRV_USB_EVENT_DATA    * eventData )

  Summary:
    Handles the events originating from USB Controller driver.

  Description:
    This function is registered into the USB Controller driver as a callback
    function. The USB Controller driver calls this function in case of events
    from the USB Controller driver.

  Parameters:
    referenceHandle           - USB device instance
    eventType                 - Event type
    eventData                 - Data associated with the event
    
  Returns:
    none.
*/

DEVICE void USB_DEVICE_EventHandler( DRV_USB_EVENT         eventType,
                              DRV_USB_EVENT_DATA    * eventData )
{
    // Handle events, only if this instance is in initialized state.
    if( usbDeviceInstance.usbDeviceInstanceState <= SYS_STATUS_UNINITIALIZED )
    {
        // Instance is deInitialized. Just do a return.
        // Doing just return, will cause the driver to send ZLPs for set packets.
        // This is a known thing and must be caught during debuggging.
        return;                
    }    
    
    switch(eventType)
    {
        case USB_DEVICE_RESET:
            // Host has issued a reset.
            // If the control endpoint is already opened, close it.
            if( usbDeviceInstance.ctrlEpInHandle != DRV_HANDLE_INVALID )
            {
                // Close the handle to endpoint
                DRV_USB_Device_PipeRelease( usbDeviceInstance.ctrlEpInHandle );
                // Invalidate the handle
                usbDeviceInstance.ctrlEpInHandle = DRV_HANDLE_INVALID;
            }

            // Reopen the control endpoint
            usbDeviceInstance.ctrlEpInHandle = _DRV_USB_MAKE_NAME( Device_PipeSetup )( 0, USB_EP_TX_RX, 64, 1, USB_CONTROL_PIPE );

            SYS_ASSERT( ( usbDeviceInstance.ctrlEpInHandle != DRV_HANDLE_INVALID ), " USB Device Layer: Driver returned an invalid handle");
					
            // Change device state to Default
            usbDeviceInstance.usbDeviceState = USB_DEVICE_STATE_DEFAULT;
            // Set the event to indicate reset to application clients
            usbDeviceInstance.event = USB_DEVICE_EVENT_RESET;
            // Reset means chirping has already happened. So, we must be knowing the speed.
            // get the speed.
            usbDeviceInstance.usbSpeed = DRV_USB_Device_CurrentSpeedGet( usbDeviceInstance.driverHandle );

            // Now we know the speed. So for this speed get the pointer that
            // points to correct group of configurations.
            if( usbDeviceInstance.usbSpeed == USB_SPEED_HIGH )
            {
                usbDeviceInstance.configDescriptorsPtr
                    = usbDeviceInstance.ptrMasterDescTable->ptrHighSpeedConfigDescriptor;
                // Also get the max configurations available in this group.
                usbDeviceInstance.maxConfigs =
                    usbDeviceInstance.ptrMasterDescTable->highSpeedConfigDescriptorCount;
            }
            else
            {
                // Classic speeds (full/low speed)
                usbDeviceInstance.configDescriptorsPtr
                    = usbDeviceInstance.ptrMasterDescTable->ptrConfigDescriptor;

                // Get the maximum configurations available in this group.
                usbDeviceInstance.maxConfigs =
                    usbDeviceInstance.ptrMasterDescTable->configDescriptorCount;
            }

            SYS_ASSERT( ( usbDeviceInstance.configDescriptorsPtr != NULL ),
                "USB Device Layer: Configuration descriptors for the selected group is NULL");

            // Invalidate the current active configuration
            usbDeviceInstance.activeConfiguration = 0;

        break;

        case USB_DEVICE_RESUME:
            // USB device resumed.
            // Set the event to inform the application later that device resumed.
            usbDeviceInstance.event = USB_DEVICE_EVENT_RESUMED;
            // Restore the device state to previous state.
            usbDeviceInstance.usbDeviceState = usbDeviceInstance.usbDevStatePriorSuspend;
        break;
			
        case USB_ERROR:
            // USB error detected.
            // Set event to inform the application later.
            usbDeviceInstance.event = USB_DEVICE_EVENT_ERROR;
	    break;		    
	    
	    case USB_IDLE_DETECT:
            // This means USB device is suspended.
            // Set event to inform the application later that device is suspended.
            usbDeviceInstance.event = USB_DEVICE_EVENT_SUSPENDED;
            // Save the device state. Once the device resumes device has to fall back
            // to its previous state.
            usbDeviceInstance.usbDevStatePriorSuspend = usbDeviceInstance.usbDeviceState;
            // Set the device state to "Suspended"
            usbDeviceInstance.usbDeviceState = USB_DEVICE_STATE_SUSPENDED;
        break;
	        
        case USB_DEVICE_SETUP_HANDSHAKE:
            // CD generates this event after sending ZLP to host.
            // Check if set address is pending
            if( usbDeviceInstance.setAddressPending == true )
            {
                // Reset the flag.
		        usbDeviceInstance.setAddressPending = false;
                // Request USB controller driver to set new address.
                DRV_USB_Device_AddressSet( usbDeviceInstance.driverHandle, usbDeviceInstance.deviceAddress );
                
                if( usbDeviceInstance.deviceAddress == 0 )
                {                    
                    if( usbDeviceInstance.usbDeviceState == USB_DEVICE_STATE_ADDRESSED )
                    {
                        // If the device is already in addressed state and the new address is zero, then enter
                        // default state.
                        usbDeviceInstance.usbDeviceState = USB_DEVICE_STATE_DEFAULT;
                    }
                }
                else
                {
                    // Change the device state to Addressed only if the new address is not zero.
                    usbDeviceInstance.usbDeviceState = USB_DEVICE_STATE_ADDRESSED;
                }     	
            }                    
        break;
	
        case USB_DEVICE_SETUP_TOKEN:
            if( usbDeviceInstance.ctrlEpInStalled == true )
            {
                // Control IN endpoint stalled previously. Clear it now.
                DRV_USB_Device_PipeStallClear( usbDeviceInstance.ctrlEpInHandle, USB_EP_TX );
                usbDeviceInstance.ctrlEpInStalled = false;
            }   
            // Process setup packet.
            _USB_DEVICE_ProcessSetupPacket( ( SETUP_PKT * ) eventData->setupEventData.pSetupPktBuffer );
        break;
            
        case USB_STALL:
        case USB_DEVICE_GOT_SOF:
        default:
            // Nothing to do for all other cases.
        break;
        
    }

} /* USB_DEVICE_EventHandler */


/******************************************************************************
  Function:
    static void _USB_DEVICE_ProcessSetupPacket( SETUP_PKT* setupPkt )

  Summary:
    Processes the setup packet received from the USB Controller driver.

  Description:
    This function processes the setup packet received from the USB controller driver.

  Parameters:
    setupPkt           - Setup packet buffer
   
  Returns:
    None.
*/

DEVICE static void _USB_DEVICE_ProcessSetupPacket( SETUP_PKT* setupPkt )
{
    if( setupPkt->bmRequestType == ( SETUP_DIRN_HOST_TO_DEVICE | SETUP_TYPE_STANDARD | SETUP_RECIPIENT_DEVICE ) )
    {  
        // Serve standard SET requests
        _USB_DEVICE_ProcessStandardSetRequests( setupPkt );
    }
    else if( setupPkt->bmRequestType == ( SETUP_DIRN_DEVICE_TO_HOST | SETUP_TYPE_STANDARD | SETUP_RECIPIENT_DEVICE ) )
    {
        
        // Serve standard GET requests
         _USB_DEVICE_ProcessStandardGetRequests( setupPkt );
    }
    else
    {
        // Serve any requests that is not "standard" type and whose
        // recipient is not "device". (Any request whose recipient is interface/endpoint
        // must be handled by the function driver. This is because function driver has all the information
        // about endpoint and interface) 
         _USB_DEVICE_ProcessOtherRequests( setupPkt );
    }
   
} /* _USB_DEVICE_ProcessSetupPacket */


/******************************************************************************
  Function:
    static void _USB_DEVICE_ProcessStandardGetRequests( SETUP_PKT* setupPkt )

  Summary:
    Processes the standard "get" requests received from the USB Controller driver

  Description:
    This function processes the standard "get" requests received from the USB Controller
    driver.

  Parameters:
    setupPkt           - Setup packet buffer
   
  Returns:
    None.
*/

DEVICE static void _USB_DEVICE_ProcessStandardGetRequests( SETUP_PKT * setupPkt )
{
 
    USB_CONFIGURATION_DESCRIPTOR* lConfigDescriptor = NULL;
    uint8_t * stringDesc;
    uint16_t responseBufferSize = 0;
    void * responseBufferPtr = NULL;
    uint8_t index;
    DRV_USB_XFER_HANDLE hTransfer;
    
    if( setupPkt->bRequest == USB_REQUEST_GET_DESCRIPTOR )
    {
        switch( setupPkt->bDescriptorType )
        {
            case USB_DESCRIPTOR_DEVICE:
                if( usbDeviceInstance.usbSpeed == USB_SPEED_HIGH )
                {
                    // High speed descriptor.
                    SYS_ASSERT( ( usbDeviceInstance.ptrMasterDescTable->ptrHighSpeedDeviceDescriptor != NULL ),
                                                                    "High speed device descriptor is NULL" );
                    // pointer to high speed device descriptor.
                    responseBufferPtr = ( (uint8_t*)usbDeviceInstance.ptrMasterDescTable->ptrHighSpeedDeviceDescriptor );
                }
                else
                {
                    // Full/low speed descriptor.
                    SYS_ASSERT( ( usbDeviceInstance.ptrMasterDescTable->ptrDeviceDescriptor != NULL ),
                                                            "Full/Low speed device descriptor is NULL");
                    // full/low speed device descriptor pointer.
                    responseBufferPtr = ( (uint8_t * )usbDeviceInstance.ptrMasterDescTable->ptrDeviceDescriptor);
                }
                // Total size of the device descriptor (It is always 18).
                responseBufferSize = 18;   
            break;
                
            case USB_DESCRIPTOR_CONFIGURATION:                
                // Get correct pointer to the descriptor based on config value.
                // setupPkt->bDscIndex indicates the host requested configuration index.
                // Make sure that the requested configuration index is with in the limits.
                if( ( setupPkt->bDscIndex ) < usbDeviceInstance.maxConfigs )
                {
                    lConfigDescriptor =
                            ( USB_CONFIGURATION_DESCRIPTOR * )usbDeviceInstance.configDescriptorsPtr[ setupPkt->bDscIndex ];
                    // Assert that pointer is not NULL
                    SYS_ASSERT( ( lConfigDescriptor != NULL ),
                        "USB Device Layer: Invalid pointer to configuration descriptor");
                    // Total Size of the descriptor to be sent to Host.
                    responseBufferSize = lConfigDescriptor->wTotalLength;
                }
                responseBufferPtr = (void*)lConfigDescriptor;
            break;
            
            case USB_DESCRIPTOR_DEVICE_QUALIFIER:
                if( usbDeviceInstance.usbSpeed == USB_SPEED_HIGH )
                {
                    // For high speed, respond with the other speed (full speed) device_qualifier.
                    responseBufferPtr = ( ( uint8_t * )usbDeviceInstance.ptrMasterDescTable->ptrFullSpeedDeviceQualifier );
                }
                else
                {
                    // For full speed, repond with the other speed (high speed) device_qualifier. 
                    responseBufferPtr = ( ( uint8_t * )usbDeviceInstance.ptrMasterDescTable->ptrHighSpeedDeviceQualifier );
                } 
                          
                // Size of device_qualifier descriptor is always 10.
                responseBufferSize = 10;            
            break;
            
                
            case USB_DESCRIPTOR_STRING:
                index = setupPkt->bDscIndex;
                // Check the string descriptor index
                if( setupPkt->bDscIndex < usbDeviceInstance.ptrMasterDescTable->stringDescCount )
                {
                    // Get correct string descriptor   
                    stringDesc = (uint8_t*)( usbDeviceInstance.ptrMasterDescTable->ptrStringDesc[ setupPkt->bDscIndex ] );
                    
                    // Prepare the response
                    responseBufferPtr = (void*)stringDesc;
                
                    // Length of the string is the first byte in the setring descriptor.
                    responseBufferSize = stringDesc[0];                  
                }                                   
            break;
                
            default:
                // All other requests are not supported.
                // Stall the endpoint.
            break;
        } 
    }   
   
    if( responseBufferPtr == NULL )
    {
        // There is no data to respond. Stall the endpoint
        DRV_USB_Device_PipeStall( usbDeviceInstance.ctrlEpInHandle, USB_EP_TX_RX );
        
        // Mark that we have stalled the control IN endpoint.
        // Stall will be removed when we receive the next setup packet.
        usbDeviceInstance.ctrlEpInStalled = true;
    }
    else // ( responseBufferPtr != NULL )
    {
        // There is something to send back to host.   
        if( responseBufferSize > setupPkt->wLength )
        {
            // Clip response length to the length requested by the host.
            responseBufferSize = setupPkt->wLength;
        }            
        
        // Request driver to start the control transfer.
        hTransfer = DRV_USB_Device_TransferRequest(  usbDeviceInstance.ctrlEpInHandle,
                                                     USB_XFER_CONTROL_READ,
                                                     ( void * )responseBufferPtr,
                                                     responseBufferSize, false, NULL, NULL );
                                               
        SYS_ASSERT( ( DRV_HANDLE_INVALID != hTransfer ), "USB device layer: Transfer request failed");
    }   

} /* _USB_DEVICE_ProcessStandardGetRequests */
   

/******************************************************************************
  Function:
    static void _USB_DEVICE_ProcessStandardSetRequests( SETUP_PKT * setupPkt )

  Summary:
    Processes the standard "set" requests received from the USB Controller driver.

  Description:
    This function processes the standard "set" requests received from the USB Controller
    driver.

  Parameters:
    setupPkt           - Setup packet buffer
   
  Returns:
    None.
*/
   
DEVICE static void _USB_DEVICE_ProcessStandardSetRequests( SETUP_PKT * setupPkt )
{   
    
    bool stallInEndpoint = false;
     
    switch( setupPkt->bRequest )
    {
        case USB_REQUEST_SET_ADDRESS:
            //Got set address command. Change the address only after responding to the current request.
            usbDeviceInstance.setAddressPending = true;
            usbDeviceInstance.deviceAddress = setupPkt->bDevADR;
        break;
       
        case USB_REQUEST_SET_CONFIGURATION:
            // Device falls back to addressed state if configuration value is 0, and 
            // if the device is already in configured state.
            if( ( setupPkt->wValue == 0 ) && ( usbDeviceInstance.usbDeviceState == USB_DEVICE_STATE_CONFIGURED ) )
            {
                // Configuration value 0 means, host is trying to deconfigure the device.
                // Set a event here. USB device layer task will deinitialize the function drivers later.
                usbDeviceInstance.event = USB_DEVICE_EVENT_DECONFIGURED;
                // Change the current active configuration to Zero
                usbDeviceInstance.activeConfiguration = 0;
                // Change the state to Addressed  
                usbDeviceInstance.usbDeviceState = USB_DEVICE_STATE_ADDRESSED;
            }  
            else
            {
                // Proceed only if new configuration value is different from
                // current configuration value.
                if( usbDeviceInstance.activeConfiguration != setupPkt->wValue )
                {
                    // This will be set to false later if we find correct configuration.
                    stallInEndpoint = true;
                    
                    // Initialize all function drivers and change to configured state only if 
                    // all function drivers are initialized successfully.
                    if( _USB_DEVICE_InitializeFunctionDrivers( setupPkt->wValue ) == USB_ERROR_OK )
                    {
                        // Save the current active configuration.
                        // This may be required for clients to know which configuration is presently active.
                        usbDeviceInstance.activeConfiguration = setupPkt->wValue;
                        // Change the state to configured.
                        usbDeviceInstance.usbDeviceState = USB_DEVICE_STATE_CONFIGURED;
                        // Set an event, so that application and function drivers are informed
                        // about the same.
                        usbDeviceInstance.event = USB_DEVICE_EVENT_CONFIGURED;
                        // Do not stall the endpoint. Reset the flag.
                        stallInEndpoint = false;
                    }                    
                }
            }     
        break;
       
        default:
            // Respond with a request error.
            // Stall the endpoint.
            stallInEndpoint = true;         
        break;
    }  
    
    if( stallInEndpoint == true )
    {
        // There is no data to respond. Stall the endpoint
        DRV_USB_Device_PipeStall( usbDeviceInstance.ctrlEpInHandle, USB_EP_TX );
        // Mark that we have stalled the control IN endpoint.
        // Stall will be removed when we receive the next setup packet.
        usbDeviceInstance.ctrlEpInStalled = true;
    }
    
} /* _USB_DEVICE_ProcessStandardSetRequests */


/******************************************************************************
  Function:
    static void _USB_DEVICE_ProcessOtherRequests( SETUP_PKT * setupPkt )

  Summary:
    Processes the setup requests that are not targeted to the device.
    All other requests are forwarded to the appropriate function driver.

  Description:
    This function processes the setup requests that are not targeted to the device.
    All other requests are forwarded to the appropriate function driver.

  Parameters:
    setupPkt           - Setup packet buffer
   
  Returns:
    None.
*/
   
DEVICE static void _USB_DEVICE_ProcessOtherRequests( SETUP_PKT * setupPkt )
{
    uint8_t lCount;
    uint8_t interfaceNumber;
    uint8_t endpointNumber;
    USB_DEVICE_FUNC_DRIVER_DATA * lfunctionDriverPtr;
      
    // Active function drivers attached to this instance.
    lfunctionDriverPtr = &usbDeviceInstance.functionDriver[0];
    
    if( setupPkt->bmRequestType & SETUP_RECIPIENT_INTERFACE )
    {
        // Get the interface number.
        interfaceNumber = setupPkt->bIntfID;
        
        // Get the endpoint number
        endpointNumber = setupPkt->bEPID;
            
        // Check which function driver has to handle this setup packet.
        for( lCount = 0; lCount < usbDeviceInstance.functionDriverCount; lCount++ )
        {
            // Request each function driver to check the interface number
            //and see which function driver takes the claim of the interface number.
            if( lfunctionDriverPtr[lCount].driver->checkInterfaceNumber != NULL )
            {
                if( lfunctionDriverPtr[lCount].driver->checkInterfaceNumber( interfaceNumber ) == USB_ERROR_OK )
                {
                    // This function driver has claimed the ownership of the interface number.
                    // Pass the setup request to this fuction driver
                    if( lfunctionDriverPtr[lCount].driver->setupPacketHandler != NULL )
                    {
                        lfunctionDriverPtr[lCount].driver->setupPacketHandler( setupPkt );
        				                	
                        // Break from the loop.
                        break;
                    }
                 }  
            }
        }
    }
    else if( setupPkt->bmRequestType & SETUP_RECIPIENT_ENDPOINT )
    {
         // Get the endpoint number
        endpointNumber = setupPkt->bEPID;
            
        // Check which function driver has to handle this setup packet.
        for( lCount = 0; lCount < usbDeviceInstance.functionDriverCount; lCount++ )
        {
            // Request each function driver to check the endpoint number
            //and see which function driver takes the claim of the endpoint number.
            if( lfunctionDriverPtr[lCount].driver->checkEndpointNumber != NULL )
            {
                if( lfunctionDriverPtr[lCount].driver->checkEndpointNumber( endpointNumber ) == USB_ERROR_OK )
                {
                    // This function driver has claimed the ownership of the endpoint number.
                    // Pass the setup request to this fuction driver
                    if( lfunctionDriverPtr[lCount].driver->setupPacketHandler != NULL )
                    {
                        lfunctionDriverPtr[lCount].driver->setupPacketHandler( setupPkt );
        				                	
                        //break from the loop.
                        break;  
                    }
                 }  
            }      
        }
    }
    else
    {
        // There is no data to respond. Stall the endpoint
        DRV_USB_Device_PipeStall( usbDeviceInstance.ctrlEpInHandle, USB_EP_TX );

        // Mark that we have stalled the control IN endpoint.
        // Stall will be removed when we receive the next setup packet.
        usbDeviceInstance.ctrlEpInStalled = true;
    }

} /* _USB_DEVICE_ProcessOtherRequests */


/******************************************************************************
  Function:
    static USB_ERROR_STATUS _USB_DEVICE_InitializeFunctionDrivers(uint16_t configValue,
                            USB_DEVICE_INSTANCE_STRUCT* usbDeviceInstance)

  Summary:
    This function is invoked when there is a set configuration event.

  Description:
    This function is responsible for initializing all the registered function
    drivers corresponding to the configuration. 

  Parameters:
    configValue        - Configuration value
    usbDeviceInstance  - This instance of the USB device layer
   
  Returns:
    Returns USB_ERROR_OK on success.
    
*/

DEVICE static USB_ERROR_STATUS _USB_DEVICE_InitializeFunctionDrivers( uint16_t configValue )
{
    uint16_t funcDriverCount;
    USB_DEVICE_FUNC_REGISTRATION_TABLE * registeredFuncDrivers;
    void * funcDriverInit;
    USB_ERROR_STATUS result;
    SYS_MODULE_INDEX usbCD_Index;
    uint8_t * descrPointer = NULL;
    uint8_t count;
    DRV_USB_SPEED usbSpeed;

    funcDriverCount         = usbDeviceInstance.registeredFuncDriverCount;
    registeredFuncDrivers   = usbDeviceInstance.registeredFuncDrivers;
    usbCD_Index             = usbDeviceInstance.usbCD_Index;
    usbSpeed                = usbDeviceInstance.usbSpeed;
    
    usbDeviceInstance.functionDriverCount = 0;
    
    // Initialize the return value to init failed.
    result = USB_ERROR_INIT_FAILED;

    for(count = 0; count < usbDeviceInstance.maxConfigs; count++)
    {
        // 5th byte in the configuration descriptor table specifies the
        // configuration value.
        if( usbDeviceInstance.configDescriptorsPtr[count][5] == configValue )
        {
            // Got a configuration match. Get the pointer to configuration
            // descriptor. We have to pass this to function driver, so that
            // function driver can parse configuration descriptor and get the required info.
            descrPointer = usbDeviceInstance.configDescriptorsPtr[count];
        }
    }   
    
    // Initialize all the interfaces corresponding to this active configuration.
    for( count = 0; count < funcDriverCount; count++ )
    {
        if( ( registeredFuncDrivers[count].speed == usbSpeed ) &&
            ( registeredFuncDrivers[count].configurationValue == configValue ) )
        {

            // User has to increase the config value of USB_DEVICE_MAX_FUNCTION_DRIVER.
            // No more function drivers can be loaded.
            SYS_ASSERT(( usbDeviceInstance.functionDriverCount < USB_DEVICE_MAX_FUNCTION_DRIVER),
                            "USB Device Layer: No more function drivers can be loaded. Increase the value of USB_DEVICE_MAX_FUNCTION_DRIVER");

            // Function driver index
            funcDriverInit =  registeredFuncDrivers[count].funcDriverInit;

            // Copy function driver details in to the module variable.
            usbDeviceInstance.functionDriver[usbDeviceInstance.functionDriverCount].driver = registeredFuncDrivers[count].driver;
            usbDeviceInstance.functionDriver[usbDeviceInstance.functionDriverCount].funcDriverInit = registeredFuncDrivers[count].funcDriverInit;

			// initialize the function driver.
            result = registeredFuncDrivers[count].driver->initialize( funcDriverInit, descrPointer );

            SYS_ASSERT( ( result == USB_ERROR_OK ), "USB device layer: Failed to initialize a function driver ");

            if( result != USB_ERROR_OK )
            {
                //break from the loop.
                break;
            }

            // So, we found a function driver for this speed and config value.
            // Increment the count.
            usbDeviceInstance.functionDriverCount++;
        }
    }

    SYS_ASSERT( ( result == USB_ERROR_OK ), "USB device layer: No function drivers initialized ");
    
    return ( result );
}    


/******************************************************************************
  Function:
    static void _USB_DEVICE_DeInitializeAllFuncDrivers ( void )

  Summary:
    Deinitializes all function drivers.

  Description:
    This function deinitializes all function drivers.

  Parameters:
    usbDeviceInstance - USB device instance.

  Returns:
    None.

*/

DEVICE static void _USB_DEVICE_DeInitializeAllFuncDrivers ( void )
{
    uint16_t count;

    for( count = 0; count < usbDeviceInstance.functionDriverCount ; count++ )
    {
    	if(usbDeviceInstance.functionDriver[count].driver->deInitialize != NULL)
    	{   // Deinitialize previously loaded function driver
            usbDeviceInstance.functionDriver[count].driver->deInitialize();
        }
    }

    // Reset function drivers count (No more active function drivers)
    usbDeviceInstance.functionDriverCount = 0;
}


/******************************************************************************
  Function:
    static void _USB_DEVICE_BroadcastEventToClients
       ( USB_DEVICE_EVENT event, USB_DEVICE_INSTANCE_STRUCT* usbDeviceInstance )

  Summary:
    Broadcasts events to the application client.

  Description:
    This function broadcasts device layer events to all application level clients.

  Parameters:
    event - Device layer event
    usbDeviceInstance - USB device instance

  Returns:
    None.

*/

DEVICE static void _USB_DEVICE_BroadcastEventToClients( USB_DEVICE_EVENT event )
{
    // Inform all application layer clients about the event.
    if( usbDeviceClient.clientState == DRV_CLIENT_STATUS_READY )
    {
        if( usbDeviceClient.callBackFunc != NULL)
        {
            // Pass event to application
            usbDeviceClient.callBackFunc( event );
        }
    }
} /* _USB_DEVICE_BroadcastEventToClients */


/*******************************************************************************
End of File
*/


/*
 * File:   app.h
 * Author: diego.tsutsumi
 *
 * Created on 22 October 2014, 10:12
 */

#include "app.h"
//#define NoDebug

int main ( void )
{

    /*Call the SYS Init routine. App init routine gets called from this*/
    SYS_Initialize ( NULL );

    while ( true )
    {
        /*Invoke SYS tasks. APP tasks gets called from this*/
        SYS_Tasks ( );
    }

    /* Should not come here during normal operation */
    SYS_ASSERT ( false , "about to exit main" );

    return ( EXIT_FAILURE );
}

/*******************************************************************************
 End of File
*/
#pragma once

#include <xc.h>

#define IO_RA4_SetHigh( )	do { LATAbits.LATA4 = 1; } while(0)
#define IO_RA4_SetLow( )	do { LATAbits.LATA4 = 0; } while(0)
#define IO_RA4_Toggle( )	do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define IO_RA4_GetValue( )	PORTAbits.RA4

#define IO_RA5_SetHigh( )	do { LATAbits.LATA5 = 1; } while(0)
#define IO_RA5_SetLow( )	do { LATAbits.LATA5 = 0; } while(0)
#define IO_RA5_Toggle( )	do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define IO_RA5_GetValue( )	PORTAbits.RA5

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
__reentrant void PIN_MANAGER_Initialize( void );

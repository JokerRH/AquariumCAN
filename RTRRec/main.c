#include "peripherals/device_config.h"
#include "peripherals/interrupt_manager.h"
#include "peripherals/oscillator.h"
#include "peripherals/pin_manager.h"
#include "peripherals/pmd.h"
#include "FreeRTOS.h"
#include "task.h"

asm( "GLOBAL _TaskBlinkRedLED" );
__reentrant void TaskBlinkRedLED( void* pvParameters )
{
	LATAbits.LATA4 = 1;
	IO_RA4_SetHigh( );
	while( 1 )
	{
		IO_RA4_Toggle( );
		vTaskDelay( 2 );
	}
}

asm( "GLOBAL _TaskBlinkGreenLED" );
__reentrant void TaskBlinkGreenLED( void* pvParameters )
{
	LATAbits.LATA5 = 1;
	IO_RA5_SetHigh( );
	while( 1 )
	{
		IO_RA5_Toggle( );
		vTaskDelay( 1 );
	}
}

StackType_t xStackRed[ configMINIMAL_STACK_SIZE ];
StaticTask_t xHandleRed;
StackType_t xStackGreen[ configMINIMAL_STACK_SIZE ];
StaticTask_t xHandleGreen;
extern void IVTTest( void );
void main( )
{
	uint8_t bReason = 0;
	if( PCON0bits.STKUNF )
		bReason = 1;
	else if( PCON0bits.STKOVF )
		bReason = 2;
	else if( PCON1bits.MEMV == 0 )
		bReason = 3;
	else if( STATUS == 0b01100000 && ( PCON0 & 0xFE ) == 0b00111100 )
		bReason = 4;
	else if( PCON0bits.RMCLR == 0 )
		bReason = 4;
	else //if( PCON0bits.RI == 0 )
		bReason = 4;

	INTERRUPT_Initialize( );
	PMD_Initialize( );
	PIN_MANAGER_Initialize( );
	OSCILLATOR_Initialize( );

	if( bReason != 4 )
	{
		IO_RA4_SetHigh( );
		IO_RA5_SetHigh( );
		__delay_ms( 1000 );
		IO_RA4_SetLow( );
		IO_RA5_SetLow( );
		__delay_ms( 1000 );
		IO_RA4_SetHigh( );
		IO_RA5_SetHigh( );
		__delay_ms( 1000 );
		IO_RA4_SetLow( );
		IO_RA5_SetLow( );
		__delay_ms( 1000 );
		IO_RA4_SetHigh( );
		IO_RA5_SetHigh( );
		if( bReason & 1 )
			IO_RA5_SetLow( );
		if( bReason & 2 )
			IO_RA4_SetLow( );
		        
        while( 1 );
	}

	IO_RA4_SetLow( );
	IO_RA5_SetLow( );
    
    PIE0bits.SWIE = 1;

    xTaskCreateStatic( TaskBlinkRedLED, (const portCHAR*) "RedLED", configMINIMAL_STACK_SIZE, NULL, 3, xStackRed, &xHandleRed );
    xTaskCreateStatic( TaskBlinkGreenLED, (const portCHAR*) "GreenLED", configMINIMAL_STACK_SIZE, NULL, 3, xStackGreen, &xHandleGreen );
	vTaskStartScheduler( );
	while( 1 );
}

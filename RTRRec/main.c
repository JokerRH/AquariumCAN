#include "peripherals/device_config.h"
#include "peripherals/ecan.h"
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

static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, uint8_t *passedInEIDH, uint8_t *passedInEIDL, uint8_t *passedInSIDH, uint8_t *passedInSIDL) {
    uint8_t wipSIDL = 0;

        *passedInEIDH = 0;
        *passedInEIDL = 0;
        tempPassedInID = tempPassedInID << 5;
        *passedInSIDL = 0xFF & tempPassedInID;
        tempPassedInID = tempPassedInID >> 8;
        *passedInSIDH = 0xFF & tempPassedInID;
}

extern ecan_msg_t *pxCurrentMsg;
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

	ECAN_Initialize( );
    INTCON0bits.GIEL = 1;
    INTCON0bits.GIEH = 1;
    
    TXB0D7 = 0;
    TXB0D6 = 0;
    TXB0D5 = 0;
    TXB0D4 = 0;
    TXB0D3 = 0;

	ecan_msg_t msg;
    pxCurrentMsg = &msg;
	convertCANid2Reg( 0x124, 1, &msg.ucEIDH, &msg.ucEIDL, &msg.ucSIDH, &msg.ucSIDL );
	msg.ucD7 = 0x77;
	msg.ucD6 = 0x66;
	msg.ucD5 = 0x55;
	msg.ucD4 = 0x44;
	msg.ucD3 = 0x33;
	msg.ucD2 = 0x22;
	msg.ucD1 = 0x11;
	msg.ucD0 = 0x00;
	msg.ucDLC = 4;

	PIE5bits.TXBnIE = 1;
	__delay_ms( 1000 );
	PIE5bits.TXBnIE = 1;
    while( 1 )
    {
    }

    PIE0bits.SWIE = 1;

    xTaskCreateStatic( TaskBlinkRedLED, (const portCHAR*) "RedLED", configMINIMAL_STACK_SIZE, NULL, 3, xStackRed, &xHandleRed );
    xTaskCreateStatic( TaskBlinkGreenLED, (const portCHAR*) "GreenLED", configMINIMAL_STACK_SIZE, NULL, 3, xStackGreen, &xHandleGreen );
	vTaskStartScheduler( );
	while( 1 );
}

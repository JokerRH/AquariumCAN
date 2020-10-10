#include "peripherals/device_config.h"
#include "peripherals/ecan.h"
#include "peripherals/interrupt_manager.h"
#include "peripherals/oscillator.h"
#include "peripherals/pin_manager.h"
#include "peripherals/pmd.h"
#include "FreeRTOS.h"
#include "task.h"

TaskHandle_t xHandleTxTest;
StackType_t xStackTxTest[ configMINIMAL_STACK_SIZE ];
StaticTask_t xBufferTxTest;

static __reentrant void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, uint8_t *passedInEIDH, uint8_t *passedInEIDL, uint8_t *passedInSIDH, uint8_t *passedInSIDL)
{
	*passedInEIDH = 0;
	*passedInEIDL = 0;
	tempPassedInID = tempPassedInID << 5;
	*passedInSIDL = 0xFF & tempPassedInID;
	tempPassedInID = tempPassedInID >> 8;
	*passedInSIDH = 0xFF & tempPassedInID;
}

asm( "GLOBAL _TxCallback" );
void TxCallback( void )
{
	xTaskNotifyGive( xHandleTxTest );
}

ListItem_t li;
ecan_msg_t msg;
asm( "GLOBAL _TaskTxTest" );
__reentrant void TaskTxTest( void* pvParameters )
{
	uint8_t ucCount = 3;

	msg.pvCallback = TxCallback;
	listSET_LIST_ITEM_OWNER( &li, &msg );
	convertCANid2Reg( 0x124, 1, &msg.ucEIDH, &msg.ucEIDL, &msg.ucSIDH, &msg.ucSIDL );
	msg.ucDLC = 1;

	while( 1 )
	{
		LATAbits.LATA4 = ucCount & 1;
		LATAbits.LATA5 = ( ucCount >> 1 ) & 1;

		msg.ucD7 = 0x77;
		msg.ucD6 = 0x66;
		msg.ucD5 = 0x55;
		msg.ucD4 = 0x44;
		msg.ucD3 = 0x33;
		msg.ucD2 = 0x22;
		msg.ucD1 = 0x11;
		msg.ucD0 = ucCount;

		vECANTransmit( &li );
		ulTaskNotifyTake( true, portMAX_DELAY );

		if( ucCount == 0 )
			ucCount = 4;
		ucCount--;

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

	xHandleTxTest = xTaskCreateStatic( TaskTxTest, (const portCHAR*) "TXTest", configMINIMAL_STACK_SIZE, NULL, 3, xStackTxTest, &xBufferTxTest );
	vTaskStartScheduler( );
	while( 1 );
}

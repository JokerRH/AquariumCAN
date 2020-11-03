#include "peripherals/device_config.h"
#include "peripherals/ecan.h"
#include "peripherals/interrupt_manager.h"
#include "peripherals/oscillator.h"
#include "peripherals/pin_manager.h"
#include "peripherals/pmd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stack.h"
#include <FreeRTOS/include/semphr.h>

TaskHandle_t xHandleTest;
StackType_t xStackTest[ stackSIZE_TEST ];
StaticTask_t xBufferTest;

static SemaphoreHandle_t xRetransmitMutexHandle;
static StaticSemaphore_t xRetransmitMutexBuffer;

ListItem_t xTransmitLI;
static ecan_msg_t xTransmitMsg;

#if 0
asm( "GLOBAL _TaskTxTest" );
void TaskTxTest( void* pvParameters )
{
	B5DLC = 1;
	B5D0 = 3;
	B5SIDH = 0x24;
	B5SIDL = 0x80;
	B5CON = 0x4;

	static uint8_t ucCount = 3;
	while( 1 )
	{
		LATAbits.LATA4 = ucCount & 1;
		LATAbits.LATA5 = ( ucCount >> 1 ) & 1;

		vTaskDelay( 2 );

		if( ucCount == 0 )
			ucCount = 4;
		ucCount--;

		while( B5D0 != ucCount )
			B5D0 = ucCount;
	}
}
#endif

#if 1
static void convertCANid2Reg( uint32_t tempPassedInID, uint8_t *passedInEIDH, uint8_t *passedInEIDL, uint8_t *passedInSIDH, uint8_t *passedInSIDL )
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
	xSemaphoreTake( xRetransmitMutexHandle, portMAX_DELAY );
	if( xTransmitMsg.ucD0 )
		vECANTransmitDelayed( pxECANCurrentTxLI, 2 );
	else
		xTaskNotifyGive( xHandleTest );
	xSemaphoreGive( xRetransmitMutexHandle );
}

asm( "GLOBAL _RxCallback" );
bool RxCallback( void )
{
	if( RXB0DLCbits.RXRTR )
		return false;

	xSemaphoreTake( xRetransmitMutexHandle, portMAX_DELAY );
	if( xECANAbortTransmit( &xTransmitLI ) )
		xTaskNotifyGive( xHandleTest );
	xTransmitMsg.ucD0 = 0;
	xTransmitMsg.ucD1 = RXB0D0;
	xSemaphoreGive( xRetransmitMutexHandle );

	vTaskResume( xHandleTest );
	return true;
}

asm( "GLOBAL _TaskRxTest" );
void TaskRxTest( void *pvParameters )
{
	xRetransmitMutexHandle = xSemaphoreCreateMutexStatic( &xRetransmitMutexBuffer );
	xTransmitMsg.ucD0 = true;

	//Initialise receiver
	ListItem_t li;
	vListInitialiseItem( &li );
	listSET_LIST_ITEM_OWNER( &li, RxCallback );
	vECANReceive( &li );

	//Initialise RTR message
	xTransmitMsg.pvCallback = TxCallback;
	vListInitialiseItem( &xTransmitLI );
	listSET_LIST_ITEM_OWNER( &xTransmitLI, &xTransmitMsg );
	convertCANid2Reg( 0x124, &xTransmitMsg.ucEIDH, &xTransmitMsg.ucEIDL, &xTransmitMsg.ucSIDH, &xTransmitMsg.ucSIDL );
	xTransmitMsg.ucDLC = 0x40;
	vECANTransmit( &xTransmitLI );

	while( 1 )
	{
		//Wait for message
		xSemaphoreTake( xRetransmitMutexHandle, portMAX_DELAY );
		if( xTransmitMsg.ucD0 )
		{
			vTaskSuspendAll( );
			xSemaphoreGive( xRetransmitMutexHandle );
			vTaskSuspend( NULL );
			(void) xTaskResumeAll( );
			xSemaphoreTake( xRetransmitMutexHandle, portMAX_DELAY );
		}
		
		const uint8_t ucCount = xTransmitMsg.ucD1;
		xSemaphoreGive( xRetransmitMutexHandle );
		
		LATAbits.LATA4 = ucCount & 1;
		LATAbits.LATA5 = ( ucCount >> 1 ) & 1;

		//Wait for transmit buffer
		(void) ulTaskNotifyTake( true, portMAX_DELAY );
		xTransmitMsg.ucD0 = true;
		vECANTransmitDelayed( &xTransmitLI, 1 );
	}
}
#endif

void __interrupt( irq( ERRIF ), base( 8 ), low_priority ) prvCanErrorISR( void )
{
	PIR5bits.ERRIF = 0;
	__delay_ms( 50 );
}

void __interrupt( irq( IRXIF ), base( 8 ), low_priority ) prvCanIRXErrorISR( void )
{
	PIR5bits.IRXIF = 0;
}

extern StackType_t uxIdleTaskStack[ stackSIZE_IDLE ];
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

	//xHandleTest = xTaskCreateStatic( TaskTxTest, (const portCHAR*) "TXTest", stackSIZE_TEST, NULL, 3, xStackTest, &xBufferTest );
	xHandleTest = xTaskCreateStatic( TaskRxTest, (const portCHAR*) "RXTest", stackSIZE_TEST, NULL, 2, xStackTest, &xBufferTest );
	vTaskStartScheduler( );
	xTaskGenericNotifyFromISR( NULL, 0, 0, 0, NULL, NULL ); //Circumvent compiler bug on "warning" 1498 that removes code.
	while( 1 );
}

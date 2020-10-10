#include <xc.h>
#include <limits.h>
#include <stdbool.h>
#include "ecan.h"

#include "FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/list.h"

#include "pin_manager.h"

#define ECAN_QUEUE_LEN	3
#define ECAN_QUEUE_ITEM	1

//Transmit variables
TaskHandle_t xHandleECANTransmit;
StackType_t xStackECANTransmit[ configMINIMAL_STACK_SIZE ];
StaticTask_t xBufferECANTransmit;
static volatile UBaseType_t uxTopReadyPriority;
ListItem_t *pxECANCurrentLI;
ecan_msg_t *pxECANCurrentMsg;

static SemaphoreHandle_t xMutexMsg;
static StaticSemaphore_t xMutexMsgBuffer;

static List_t pxReadyMessagesLists[ configMAX_PRIORITIES ];			/*< Prioritised message lists */
PRIVILEGED_DATA static List_t xDelayedMessageList1;					/*< Delayed messages. */
PRIVILEGED_DATA static List_t xDelayedMessageList2;					/*< Delayed messages (two lists are used - one for delays that have overflowed the current tick count. */
PRIVILEGED_DATA static List_t *volatile pxDelayedTaskList;			/*< Points to the delayed message list currently being used. */
PRIVILEGED_DATA static List_t *volatile pxOverflowDelayedTaskList;	/*< Points to the delayed message list currently being used to hold messages that have overflowed the current tick count. */

//Receive variables
TaskHandle_t xECANRecHandle;

static SemaphoreHandle_t xMutexRec;
static StaticSemaphore_t xMutexRecBuffer;

static List_t pxReceiveList;

asm( "GLOBAL _vECANTransmitTask" );
__reentrant void vECANTransmitTask( void *pvParameters )
{
	//Set to max priority and check.
	//If a different task has queued a message before this one was allowed to initialize uxTopReadyPriority is overridden to max and a few empty lists might be checked unnecessarily.
	//If a different task queues a message after initialization of uxTopPriority and before the mutex lock the task works normally.
	static UBaseType_t uxTopPriority;	//Only a single instance of this function can ever exist, no need to write this on the stack.
	uxTopReadyPriority = configMAX_PRIORITIES - 1;
	while( 1 )
	{
		// Select a new message to transmit
		xSemaphoreTake( xMutexMsg, portMAX_DELAY );
		uxTopPriority = uxTopReadyPriority;	//Must be updated. While the mutex was released to transmit another task could have queued a higher priority message.

		// Find the highest priority queue that contains ready messages.
		while( listLIST_IS_EMPTY( &( pxReadyMessagesLists[ uxTopPriority ] ) ) ) 
		{
			if( uxTopPriority )
			{
				--uxTopPriority;
				continue;
			}

			//No message was in the ready lists, suspend until a new message is inserted.
			//Use a critical region to ensure that no task tried to resume this one between releasing the mutex and suspension.
			taskENTER_CRITICAL( );
			xSemaphoreGive( xMutexMsg );
			vTaskSuspend( NULL );
			taskEXIT_CRITICAL( );
			xSemaphoreTake( xMutexMsg, portMAX_DELAY );
			uxTopPriority = uxTopReadyPriority;

			//Recheck the lists after resumption.
			//Scenario: Low priority task queues a message and resumes this task.
			//Before this task is resumed, a high priority task is scheduled and deletes the single uxTopReadyPriority message.
			//In short: It is not guaranteed that there are no tasks running between a call to resume and the actual resumption.
			//During this time, however, the mutex is released.
		}

		//Update current prio
		if( uxTopReadyPriority != uxTopPriority )
		{
			uxTopReadyPriority = uxTopPriority;
			vTaskPrioritySet( NULL, uxTopReadyPriority );	//May call taskYIELD. vTaskSwitchContext should only set xYieldPending and return.
		}

		//Fetch the current list item, then release the mutex
		pxECANCurrentLI = listGET_HEAD_ENTRY( &( pxReadyMessagesLists[ uxTopReadyPriority ] ) );
		uxListRemove( pxECANCurrentLI );
		xSemaphoreGive( xMutexMsg );

		pxECANCurrentMsg = listGET_LIST_ITEM_OWNER( pxECANCurrentLI );

		//Request transmition and wait
		PIE5bits.TXBnIE = 1;
		ulTaskNotifyTake( true, portMAX_DELAY );

		//Message has been loaded into a transmission buffer, execute callback.
		//pxCurrentMsg is now free.
		pxECANCurrentMsg->pvCallback( );
	}
}

__reentrant void vTaskECANReceive( void *pvParameters )
{
	while( 1 )
	{
		ulTaskNotifyTake( true, portMAX_DELAY );	//Wait for message

		xSemaphoreTake( xMutexRec, portMAX_DELAY );
		ListItem_t *pLI = listGET_HEAD_ENTRY( &pxReceiveList );
		for( BaseType_t ucCount = listCURRENT_LIST_LENGTH( &pxReceiveList ); ucCount != 0; --ucCount )
		{
			if( ( (rxcallback_t) listGET_LIST_ITEM_OWNER( pLI ) )( pLI ) )
				break;	//Message taken care of by callback
			pLI = listGET_NEXT( pLI );
		}
		xSemaphoreGive( xMutexRec );

		RXB0CONbits.RXFUL = 0;	//Clear RXFUL
		PIR5bits.FIFOFIF = 1;	//Enable FIFO interrupt
	}
}

void vECANTransmit( ListItem_t *pMsg )
{
	const UBaseType_t uxPriority = uxTaskPriorityGet( NULL );
	xSemaphoreTake( xMutexMsg, portMAX_DELAY );
	vListInsert( &( pxReadyMessagesLists[ uxPriority ] ), pMsg );
	if( uxTopReadyPriority < uxPriority )
	{
		vTaskPrioritySet( xHandleECANTransmit, uxPriority );
		uxTopReadyPriority = uxPriority;
	}
	xSemaphoreGive( xMutexMsg );

	//Resume the message task
	vTaskResume( xHandleECANTransmit );
}

#if 0
#include "pin_manager.h"
void __interrupt( irq( TXB2IF ), base( 8 ), low_priority ) prvECANTxISR( void )
{
	IO_RA4_SetHigh( );
	IO_RA5_SetHigh( );
	asm( "\
		MOVF	CANSTAT, w, a	    ; Create backup of CANSTAT\
		MOVFF	CANSTAT, ECANCON\
		\
		MOVFF	_pxCurrentMsg, FSR0L\
		MOVFF	_pxCurrentMsg + 1, FSR0H\
		\
		MOVFF	POSTDEC0, RXB0D7	; D7\
		MOVFF	POSTDEC0, RXB0D6	; D6\
		MOVFF	POSTDEC0, RXB0D5	; D5\
		MOVFF	POSTDEC0, RXB0D4	; D4\
		MOVFF	POSTDEC0, RXB0D3	; D3\
		MOVFF	POSTDEC0, RXB0D2	; D2\
		MOVFF	POSTDEC0, RXB0D1	; D1\
		MOVFF	POSTDEC0, RXB0D0	; D0\
		MOVFF	POSTDEC0, RXB0DLC	; DLC\
		MOVFF	POSTDEC0, RXB0EIDL	; EIDL\
		MOVFF	POSTDEC0, RXB0EIDH	; EIDH\
		MOVFF	POSTDEC0, RXB0SIDL	; SIDL\
		MOVFF	POSTDEC0, RXB0SIDH	; SIDH\
		\
		BSF     RXB0CON, TXB0CON_TXREQ_POSN, a  ; Send message\
		MOVF    CANSTAT, f, a		    ; Restore CANSTAT\
		\
		BANKSEL( PIR5 )\
		BCF	    PIR5, PIR5_TXBnIF_POSN, b\
		BCF	    PIE5, \
	" );

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR( xECANMsgHandle, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
#endif

#if 0
void __interrupt( irq( RXB1IF ), base( 8 ), high_priority ) prvECANRecISR( void )
{
	ECANCONbits.EWIN = 0x10 + CANCONbits.FP;	//Set EWIN to map current FIFO buffer into access bank
	PIR5bits.RXBnIF = 0;
	PIE5bits.RXBnIE = 0;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveIndexedFromISR( xECANRecHandle, ecanNOTIFICATION_IDX, &xHigherPriorityTaskWoken );
	if( xHigherPriorityTaskWoken )
		PIR0bits.SWIF = 1;	//Only works if this interrupt has high priority! (Otherwise the context of the interrupt would be switched)
}
#endif

void ECAN_Initialize( void )
{
	xMutexMsg = xSemaphoreCreateMutexStatic( &xMutexMsgBuffer );
	configASSERT( xMutexMsg );

	for( UBaseType_t uxPriority = (UBaseType_t) 0U; uxPriority < (UBaseType_t) configMAX_PRIORITIES; uxPriority++ )
		vListInitialise( &( pxReadyMessagesLists[ uxPriority ] ) );

	CANCON = 0x80;
	while( 0x80 != ( CANSTAT & 0xE0 ) ); //Wait until ECAN is in config mode

	ECANCON = 0x90;	//Mode 2

	/**
	Initialize CAN I/O
	*/
	CIOCON = 0x00;
	
	/**
	Mask and Filter definitions
	........................................................	
	CAN ID		ID Type		Mask				Filter		Buffer	
	........................................................	
	0x124		SID		Acceptance Mask 0		Filter 0	FIFO
	........................................................
	*/
	
	/**
	Configure Generic Buffers to be Transmit or Receive
	*/
	BSEL0 = 0x00;
	
	/**	
		Initialize Receive Masks
	*/
	RXM0EIDH = 0xFF;
	RXM0EIDL = 0xFF;
	RXM0SIDH = 0xFF;
	RXM0SIDL = 0xE3;
	RXM1EIDH = 0xFF;
	RXM1EIDL = 0xFF;
	RXM1SIDH = 0xFF;
	RXM1SIDL = 0xE3;
	
	/**
	Enable Filters
	*/
	RXFCON0 = 0x01;
	RXFCON1 = 0x00;
	
	/**
	Assign Filters to Masks
	*/
	MSEL0 = 0x00;
	MSEL1 = 0x00;
	MSEL2 = 0x00;
	MSEL3 = 0x00;
	
	/**
	Initialize Receive Filters
	*/
	
	RXF0EIDH = 0x00;
	RXF0EIDL = 0x00;
	RXF0SIDH = 0x24;
	RXF0SIDL = 0x80;
	RXF1EIDH = 0x00;
	RXF1EIDL = 0x00;
	RXF1SIDH = 0x00;
	RXF1SIDL = 0x00;
	RXF2EIDH = 0x00;
	RXF2EIDL = 0x00;
	RXF2SIDH = 0x00;
	RXF2SIDL = 0x00;
	RXF3EIDH = 0x00;
	RXF3EIDL = 0x00;
	RXF3SIDH = 0x00;
	RXF3SIDL = 0x00;
	RXF4EIDH = 0x00;
	RXF4EIDL = 0x00;
	RXF4SIDH = 0x00;
	RXF4SIDL = 0x00;
	RXF5EIDH = 0x00;
	RXF5EIDL = 0x00;
	RXF5SIDH = 0x00;
	RXF5SIDL = 0x00;
	
	RXF6EIDH = 0x00;
	RXF6EIDL = 0x00;
	RXF6SIDH = 0x00;
	RXF6SIDL = 0x00;
	RXF7EIDH = 0x00;
	RXF7EIDL = 0x00;
	RXF7SIDH = 0x00;
	RXF7SIDL = 0x00;
	RXF8EIDH = 0x00;
	RXF8EIDL = 0x00;
	RXF8SIDH = 0x00;
	RXF8SIDL = 0x00;
	RXF9EIDH = 0x00;
	RXF9EIDL = 0x00;
	RXF9SIDH = 0x00;
	RXF9SIDL = 0x00;
	RXF10EIDH = 0x00;
	RXF10EIDL = 0x00;
	RXF10SIDH = 0x00;
	RXF10SIDL = 0x00;
	RXF11EIDH = 0x00;
	RXF11EIDL = 0x00;
	RXF11SIDH = 0x00;
	RXF11SIDL = 0x00;
	RXF12EIDH = 0x00;
	RXF12EIDL = 0x00;
	RXF12SIDH = 0x00;
	RXF12SIDL = 0x00;
	RXF13EIDH = 0x00;
	RXF13EIDL = 0x00;
	RXF13SIDH = 0x00;
	RXF13SIDL = 0x00;
	RXF14EIDH = 0x00;
	RXF14EIDL = 0x00;
	RXF14SIDH = 0x00;
	RXF14SIDL = 0x00;
	RXF15EIDH = 0xFF;
	RXF15EIDL = 0xFF;
	RXF15SIDH = 0xFF;
	RXF15SIDL = 0xE3;

	/**
	Initialize CAN Timings
	*/
	
	/**
	Baud rate: 500kbps
	System frequency: 16000000
	ECAN clock frequency: 16000000
	Time quanta: 8
	Sample point: 1-1-4-2
	Sample point: 75%
	*/
	
	BRGCON1 = 0x01;
	BRGCON2 = 0x98;
	BRGCON3 = 0x81;

	
	CANCON = 0x00;
	while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode

	//Create transmit task
	xHandleECANTransmit = xTaskCreateStatic( vECANTransmitTask, (const portCHAR*) "ECANTX", configMINIMAL_STACK_SIZE, NULL, 0, xStackECANTransmit, &xBufferECANTransmit );

	//Enable interrupts
	TXBIEbits.TXB0IE = 1;
	TXBIEbits.TXB1IE = 1;
	TXBIEbits.TXB2IE = 1;

	TXB0CONbits.TXBIF = 1;
	//TXB1CONbits.TXBIF = 1;
	//TXB2CONbits.TXBIF = 1;
}

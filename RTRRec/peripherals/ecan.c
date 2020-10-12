#include <xc.h>
#include <limits.h>
#include <stdbool.h>
#include "ecan.h"

#include "FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/list.h"

#include "pin_manager.h"

//Transmit variables
TaskHandle_t xECANTransmitHandle;
static StackType_t xECANTransmitStack[ configMINIMAL_STACK_SIZE ];
static StaticTask_t xECANTransmitBuffer;
static volatile UBaseType_t uxTopReadyPriority;
ListItem_t *pxECANCurrentTxLI;
ecan_msg_t *pxECANCurrentMsg;

static SemaphoreHandle_t xTransmitMutexHandle;
static StaticSemaphore_t xTransmitMutexBuffer;

static List_t pxReadyMessagesLists[ configMAX_PRIORITIES ];			/*< Prioritised message lists */
PRIVILEGED_DATA static List_t xDelayedMessageList1;					/*< Delayed messages. */
PRIVILEGED_DATA static List_t xDelayedMessageList2;					/*< Delayed messages (two lists are used - one for delays that have overflowed the current tick count. */
PRIVILEGED_DATA static List_t *volatile pxDelayedTaskList;			/*< Points to the delayed message list currently being used. */
PRIVILEGED_DATA static List_t *volatile pxOverflowDelayedTaskList;	/*< Points to the delayed message list currently being used to hold messages that have overflowed the current tick count. */

//Receive variables
static TaskHandle_t xECANReceiveHandle;
static StackType_t xECANReceiveStack[ configMINIMAL_STACK_SIZE ];
static StaticTask_t xECANReceiveBuffer;
ListItem_t *pxECANCurrentRxLI;

static SemaphoreHandle_t xReceiveMutexHandle;
static StaticSemaphore_t xReceiveMutexBuffer;

static List_t xReceiveList;

extern void prvECANTxCallback( void );
asm( "GLOBAL _prvECANTransmitTask" );
__reentrant void prvECANTransmitTask( void *pvParameters )
{
	//Set to max priority and check.
	//If a different task has queued a message before this one was allowed to initialize uxTopReadyPriority is overridden to max and a few empty lists might be checked unnecessarily.
	//If a different task queues a message after initialization of uxTopPriority and before the mutex lock the task works normally.
	static UBaseType_t uxTopPriority;	//Only a single instance of this function can ever exist, no need to write this on the stack.
	uxTopReadyPriority = configMAX_PRIORITIES - 1;
	while( 1 )
	{
		// Select a new message to transmit
		xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );
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
			xSemaphoreGive( xTransmitMutexHandle );
			vTaskSuspend( NULL );
			taskEXIT_CRITICAL( );
			xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );
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
		pxECANCurrentTxLI = listGET_HEAD_ENTRY( &( pxReadyMessagesLists[ uxTopReadyPriority ] ) );
		uxListRemove( pxECANCurrentTxLI );
		xSemaphoreGive( xTransmitMutexHandle );

		pxECANCurrentMsg = listGET_LIST_ITEM_OWNER( pxECANCurrentTxLI );

		//Request transmition and wait
		PIE5bits.TXBnIE = 1;
		ulTaskNotifyTake( true, portMAX_DELAY );

		//Message has been loaded into a transmission buffer, execute callback.
		//pxCurrentMsg is now free.
		prvECANTxCallback( );
	}
}

extern bool prvECANRxCallback( void );
asm( "GLOBAL _prvECANReceiveTask" );
__reentrant void prvECANReceiveTask( void *pvParameters )
{
	PIE5bits.RXBnIE = 1;
	while( 1 )
	{
		ulTaskNotifyTake( true, portMAX_DELAY );	//Wait for message

		xSemaphoreTake( xReceiveMutexHandle, portMAX_DELAY );
		pxECANCurrentRxLI = listGET_HEAD_ENTRY( &xReceiveList );
		for( BaseType_t ucCount = listCURRENT_LIST_LENGTH( &xReceiveList ); ucCount != 0; --ucCount )
		{
			if( prvECANRxCallback( ) )
				break;
			pxECANCurrentRxLI = listGET_NEXT( pxECANCurrentRxLI );
		}
		xSemaphoreGive( xReceiveMutexHandle );

		RXB0CONbits.RXFUL = 0;	//Clear RXFUL
		PIR5bits.RXBnIF = 0;	//Clear FIFO interrupt
		PIE5bits.RXBnIE = 1;	//Enable FIFO interrupt
	}
}

void vECANTransmit( ListItem_t *pMsg )
{
	const UBaseType_t uxPriority = uxTaskPriorityGet( NULL );
	xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );
	vListInsert( &( pxReadyMessagesLists[ uxPriority ] ), pMsg );
	if( uxTopReadyPriority < uxPriority )
	{
		vTaskPrioritySet( xECANTransmitHandle, uxPriority );
		uxTopReadyPriority = uxPriority;
	}
	xSemaphoreGive( xTransmitMutexHandle );

	//Resume the message task
	vTaskResume( xECANTransmitHandle );
}

void vECANReceive( ListItem_t *pMsg )
{
	const UBaseType_t uxPriority = uxTaskPriorityGet( NULL );
	listSET_LIST_ITEM_VALUE( pMsg, uxPriority );
	xSemaphoreTake( xReceiveMutexHandle, portMAX_DELAY );
	if( listGET_ITEM_VALUE_OF_HEAD_ENTRY( &xReceiveList ) < uxPriority )
		vTaskPrioritySet( xECANReceiveHandle, uxPriority );
	vListInsert( &xReceiveList, pMsg );
	xSemaphoreGive( xReceiveMutexHandle );
}

void __interrupt( irq( RXB1IF ), base( 8 ), low_priority ) prvECANReceiveISR( void )
{
	ECANCONbits.EWIN = 0x10 + CANCONbits.FP;	//Set EWIN to map current FIFO buffer into access bank
	PIE5bits.RXBnIE = 0;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR( xECANReceiveHandle, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void ECAN_Initialize( void )
{
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

	//Initialise transmitter
	xTransmitMutexHandle = xSemaphoreCreateMutexStatic( &xTransmitMutexBuffer );
	configASSERT( xTransmitMutexHandle );

	for( UBaseType_t uxPriority = (UBaseType_t) 0U; uxPriority < (UBaseType_t) configMAX_PRIORITIES; uxPriority++ )
		vListInitialise( &( pxReadyMessagesLists[ uxPriority ] ) );

	xECANTransmitHandle = xTaskCreateStatic( prvECANTransmitTask, (const portCHAR*) "ECANTX", configMINIMAL_STACK_SIZE, NULL, 0, xECANTransmitStack, &xECANTransmitBuffer );
	
	//Initialise receiver
	xReceiveMutexHandle = xSemaphoreCreateMutexStatic( &xReceiveMutexBuffer );
	configASSERT( xReceiveMutexHandle );
	vListInitialise( &xReceiveList );
	xECANReceiveHandle = xTaskCreateStatic( prvECANReceiveTask, (const portCHAR*) "ECANRX", configMINIMAL_STACK_SIZE, NULL, 0, xECANReceiveStack, &xECANReceiveBuffer );

	//Enable interrupts
	TXBIEbits.TXB0IE = 1;
	TXBIEbits.TXB1IE = 1;
	TXBIEbits.TXB2IE = 1;

	TXB0CONbits.TXBIF = 1;
	TXB1CONbits.TXBIF = 1;
	TXB2CONbits.TXBIF = 1;
}

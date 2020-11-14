#include <xc.h>
#include <limits.h>
#include <stdbool.h>
#include "ecan.h"
#include "../stack.h"

#include "FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/semphr.h"
#include "FreeRTOS/include/list.h"

#include "pin_manager.h"

/*
	Mutex order:
		ReceiveMutex
		DelayedMutex
		TransmitMutex

	Any function that can be called from within the receive callback _must_ expect the ReceiveMutex to be held!
*/

extern volatile TickType_t xTickCount;

//Transmit variables
TaskHandle_t xECANTransmitHandle;
static StackType_t xECANTransmitStack[ stackSIZE_CANTX ];
static StaticTask_t xECANTransmitBuffer;
ListItem_t *pxECANCurrentTxLI;
ecan_msg_t *pxECANCurrentMsg;

static SemaphoreHandle_t xTransmitMutexHandle;
static StaticSemaphore_t xTransmitMutexBuffer;
#if configTASK_NOTIFICATION_ARRAY_ENTRIES == 1
static SemaphoreHandle_t xTransmitEventHandle;
static StaticSemaphore_t xTransmitEventBuffer;
#endif

static List_t xReadyMessagesList;										/*< Prioritised message list */

//Receive variables
static TaskHandle_t xECANReceiveHandle;
static StackType_t xECANReceiveStack[ stackSIZE_CANRX ];
static StaticTask_t xECANReceiveBuffer;
ListItem_t *pxECANCurrentRxLI;

static SemaphoreHandle_t xReceiveMutexHandle;
static StaticSemaphore_t xReceiveMutexBuffer;

static List_t xReceiveList;

//Delay variables
TaskHandle_t xECANDelayedHandle;
static StackType_t xECANDelayedStack[ stackSIZE_CANDEL ];
static StaticTask_t xECANDelayedBuffer;

static SemaphoreHandle_t xDelayedMutexHandle;
static StaticSemaphore_t xDelayedMutexBuffer;

static TickType_t xNextDelayedMessage;
static UBaseType_t xDelayedPriorityCount[ configMAX_PRIORITIES ] = { 1, 0 };	//Initialize priority 0 to count 1; This ensures the delayed task will at least be assigned to priority 0.

PRIVILEGED_DATA static List_t xDelayedMessageList1;						/*< Delayed messages. */
PRIVILEGED_DATA static List_t xDelayedMessageList2;						/*< Delayed messages (two lists are used - one for delays that have overflowed the current tick count. */
PRIVILEGED_DATA static List_t *volatile pxDelayedMessagesList;			/*< Points to the delayed message list currently being used. */
PRIVILEGED_DATA static List_t *volatile pxOverflowDelayedMessagesList;	/*< Points to the delayed message list currently being used to hold messages that have overflowed the current tick count. */

ListItem_t *pxLI;	//Circumvent compiler bug (clears upper half of address)

extern void prvAddCurrentTaskToDelayedList( TickType_t xTicksToWait, const BaseType_t xCanBlockIndefinitely );

extern void prvECANTxCallback( void );
asm( "GLOBAL _prvECANTransmitTask" );
void prvECANTransmitTask( void *pvParameters )
{
	while( 1 )
	{
		//Clear TX interrupt flag and enable TX interrupts
		PIR5bits.TXBnIF = 0;
		PIE5bits.TXBnIE = 1;

		//Wait for a transmit buffer to be ready
		xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY );

		// Select a new message to transmit
		xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );

		// Find the highest priority queue that contains ready messages.
		while( listLIST_IS_EMPTY( &xReadyMessagesList ) ) 
		{
			//No message was in the ready lists, suspend until a new message is inserted.
#if configTASK_NOTIFICATION_ARRAY_ENTRIES == 1
			xSemaphoreTake( xTransmitEventHandle, 0 );	//This is cheaper than an additional (unnecessary) round in the loop
			xSemaphoreGive( xTransmitMutexHandle );
			xSemaphoreTake( xTransmitEventHandle, portMAX_DELAY );
#else
			xTaskNotifyStateClearIndexed( NULL, 1 );	//This is cheaper than an additional (unnecessary) round in the loop
			xSemaphoreGive( xTransmitMutexHandle );
			xTaskNotifyWaitIndexed( 1, 0, 0, NULL, portMAX_DELAY );
#endif
			xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );

			//Recheck the lists after resumption.
			//Scenario: Low priority task queues a message and resumes this task.
			//Before this task is resumed, a high priority task is scheduled and deletes the single message.
			//In short: It is not guaranteed that there are no tasks running between a call to resume and the actual resumption.
			//During this time, however, the mutex is released.
		}

		//Fetch the current list item, then release the mutex
		pxECANCurrentTxLI = listGET_HEAD_ENTRY( &xReadyMessagesList );
		pxECANCurrentMsg = listGET_LIST_ITEM_OWNER( pxECANCurrentTxLI );

		//Load the message into a transmit buffer
		//Since we know that a buffer is available TaskNofityTake will always return a valid buffer.
		vTaskSuspendAll( );	//We need temporary access to the ECANCON window, suspend the receive task (and unfortunately all others) to prevent it from acessing it.
		
		INTCON0bits.GIEH = 0;
		uint8_t ucPIE5 = PIE5;
		PIE5bits.RXBnIE = 0;
		INTCON0bits.GIEH = 1;
		
		static uint8_t ucECANCON;
		ucECANCON = ECANCON;
		ECANCON = (uint8_t) ulTaskNotifyTake( true, 0 );	//Set window. MDSEL bits are readonly, FIFOWM is irrelevant.

		asm( "MOVFF	_pxECANCurrentMsg, FSR0L" );
		asm( "MOVFF	_pxECANCurrentMsg + 1, FSR0H" );

		asm( "MOVFF	POSTINC0, RXB0D7" );	// D7
		asm( "MOVFF	POSTINC0, RXB0D6" );	// D6
		asm( "MOVFF	POSTINC0, RXB0D5" );	// D5
		asm( "MOVFF	POSTINC0, RXB0D4" );	// D4
		asm( "MOVFF	POSTINC0, RXB0D3" );	// D3
		asm( "MOVFF	POSTINC0, RXB0D2" );	// D2
		asm( "MOVFF	POSTINC0, RXB0D1" );	// D1
		asm( "MOVFF	POSTINC0, RXB0D0" );	// D0
		asm( "MOVFF	POSTINC0, RXB0DLC" );	// DLC
		asm( "MOVFF	POSTINC0, RXB0EIDL" );	// EIDL
		asm( "MOVFF	POSTINC0, RXB0EIDH" );	// EIDH
		asm( "MOVFF	POSTINC0, RXB0SIDL" );	// SIDL
		asm( "MOVFF	POSTINC0, RXB0SIDH" );	// SIDH

		RXB0CON |= 1 << _TXB0CON_TXREQ_POSN;	//Send message
		ECANCON = ucECANCON;					//Restore ECANCON
		if( ucPIE5 & _PIE5_RXBnIE_MASK )
			PIE5bits.RXBnIE = 1;
		(void) xTaskResumeAll( );				//Resume the receive task (and all others)

		//Message has been loaded into a transmission buffer, remove from ready list, execute callback and adjust priority
		uxListRemove( pxECANCurrentTxLI );	//The message is flagged for transmission, remove from waiting list

		//pxCurrentMsg is now free.
		//prvECANTxCallback releases the mutex before calling the callback
		prvECANTxCallback( );

		xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );
		vTaskPrioritySet( NULL, (UBaseType_t) listGET_ITEM_VALUE_OF_HEAD_ENTRY( &xReadyMessagesList ) );	//Update current prio
		xSemaphoreGive( xTransmitMutexHandle );
	}
}

extern bool prvECANRxCallback( void );
asm( "GLOBAL _prvECANReceiveTask" );
void prvECANReceiveTask( void *pvParameters )
{
	PIE5bits.RXBnIE = 1;
	while( 1 )
	{
		xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY );	//Wait for message
		xSemaphoreTake( xReceiveMutexHandle, portMAX_DELAY );
		pxECANCurrentRxLI = listGET_HEAD_ENTRY( &xReceiveList );
		for( UBaseType_t ucCount = listCURRENT_LIST_LENGTH( &xReceiveList ); ucCount != 0; --ucCount )
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

asm( "GLOBAL _prvECANDelayedTask" );
void prvECANDelayedTask( void *pvParameters )
{
	while( 1 )
	{
		xSemaphoreTake( xDelayedMutexHandle, portMAX_DELAY );

		//Check if there are any messages available
		//pxDelayedMessagesList and pxOverflowDelayedMessagesList could be swapped by the tick ISR and would have to be accessed with the scheduler suspended.
		//Access the underlying lists directly as we do not have to differentiate here and only have to know if there is _any_ message in _either_ list.
		//As the tick increment function may run at any point during checking (potentially moving messages between the two lists), other operations on these lists might not work reliably!
		if( listLIST_IS_EMPTY( &xDelayedMessageList1 ) && listLIST_IS_EMPTY( &xDelayedMessageList2 ) )
		{
			xTaskNotifyStateClear( NULL );
			xSemaphoreGive( xDelayedMutexHandle );
			xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY );
			continue;
		}

		//Check if a message is ready to be transmitted.
		//If a new message was inserted the previous delay call was aborted and this statement has to reinitialize the delay.
		vTaskSuspendAll( );	//Required to access pxDelayedMessagesList
		{
			TickType_t uxTickDiff = xNextDelayedMessage - xTickCount;
			if( listLIST_IS_EMPTY( pxDelayedMessagesList ) )
			{
				//Delayed list is empty, but we know that at least the overflow list must contain items.
				//xNextMessage has therefore overflowed and the new formula to calculate the tick difference is:
				//uxTickDiff = ( 0xFFFF + xNextMessage ) - xTickCount
				//Whereby 0xFFFF = -1
				uxTickDiff -= 1;
				goto DELAY;
			}

			if( uxTickDiff )
			{
DELAY:
				//Release the mutex and set the new delay
				//xSemaphoreGiveFromISR does not yield
				xSemaphoreGive( xDelayedMutexHandle );
				prvAddCurrentTaskToDelayedList( uxTickDiff, false );

				if( !xTaskResumeAll( ) )
					taskYIELD( );	//No need to yield if the resume-all function already did

				continue;
			}
		}

		//At this point the front element in the delayed messages list is ready to be scheduled for transmission.
		pxLI = listGET_HEAD_ENTRY( pxDelayedMessagesList );
		(void) uxListRemove( pxLI );
		(void) xTaskResumeAll( );

		//Priority management
		{
			ecan_msg_t *const pxMsg = (ecan_msg_t *) listGET_LIST_ITEM_OWNER( pxLI );
			listSET_LIST_ITEM_VALUE( pxLI, pxMsg->puxPriority );	//Set the item value to the priority
			xDelayedPriorityCount[ pxMsg->puxPriority ]--;			//Decrease this tasks priority counter

			//Search for the current top priority.
			//If no more messages are in the lists, xDelayedPriorityCount[ 0 ] is set to 1, forcing a priority of 0.
			for( UBaseType_t uxPriority = configMAX_PRIORITIES - 1; uxPriority != 0; --uxPriority )
				if( xDelayedPriorityCount[ pxMsg->puxPriority ] )
				{
					vTaskPrioritySet( NULL, uxPriority );
					break;
				}
		}

		//Take the transmit mutex before releasing the delayed mutex.
		//This is to ensure that the message is never "floating" between two lists
		xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );
		xSemaphoreGive( xDelayedMutexHandle );

		//Add to transmit list
		vListInsert( &xReadyMessagesList, pxLI );
		vTaskPrioritySet( xECANTransmitHandle, (UBaseType_t) listGET_ITEM_VALUE_OF_HEAD_ENTRY( &xReadyMessagesList ) );
		xSemaphoreGive( xTransmitMutexHandle );

		//Resume the message task
#if configTASK_NOTIFICATION_ARRAY_ENTRIES == 1
		xSemaphoreGive( xTransmitEventHandle );
#else
		xTaskNotifyIndexed( xECANTransmitHandle, 1, 0, eNoAction );
#endif
	}
}

void vECANTransmit( ListItem_t *pxLI )
{
	{
		ecan_msg_t *pxMsg = (ecan_msg_t *) listGET_LIST_ITEM_OWNER( pxLI );
		pxMsg->puxPriority = uxTaskPriorityGet( NULL );
		listSET_LIST_ITEM_VALUE( pxLI, pxMsg->puxPriority );
	}

	xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );
	vListInsert( &xReadyMessagesList, pxLI );
	vTaskPrioritySet( xECANTransmitHandle, (UBaseType_t) listGET_ITEM_VALUE_OF_HEAD_ENTRY( &xReadyMessagesList ) );
	xSemaphoreGive( xTransmitMutexHandle );

	//Resume the message task
#if configTASK_NOTIFICATION_ARRAY_ENTRIES == 1
	xSemaphoreGive( xTransmitEventHandle );
#else
	xTaskNotifyIndexed( xECANTransmitHandle, 1, 0, eNoAction );
#endif
}

/*!
	\warning This function must not be called from within the receive callback!
*/
void vECANReceive( ListItem_t *pxLI )
{
	listSET_LIST_ITEM_VALUE( pxLI, uxTaskPriorityGet( NULL ) );
	xSemaphoreTake( xReceiveMutexHandle, portMAX_DELAY );
	vListInsert( &xReceiveList, pxLI );
	vTaskPrioritySet( xECANReceiveHandle, (UBaseType_t) listGET_ITEM_VALUE_OF_HEAD_ENTRY( &xReceiveList ) );
	xSemaphoreGive( xReceiveMutexHandle );
}

void vECANTransmitDelayed( ListItem_t *const pxLI, const TickType_t xTicksToWait )
{
	xSemaphoreTake( xDelayedMutexHandle, portMAX_DELAY );
	{
		vTaskSuspendAll( );	// Also prevents tick increments
		
		const TickType_t xConstTickCount = xTickCount;	// Minor optimisation. The tick count cannot change in this block.
		const TickType_t xTimeToTransmit = xConstTickCount + xTicksToWait;

		// The list item will be inserted in wake time order.
		listSET_LIST_ITEM_VALUE( pxLI, xTimeToTransmit );

		//If both delayed lists are empty pxLI will be the next message to be transmitted.
		bool fIsNextMessage = listLIST_IS_EMPTY( &xDelayedMessageList1 ) && listLIST_IS_EMPTY( &xDelayedMessageList2 );
		if( xTimeToTransmit < xConstTickCount )
		{
			// Wake time has overflowed. Place this item in the overflow list.
			vListInsert( pxOverflowDelayedMessagesList, pxLI );
			if( xNextDelayedMessage < xConstTickCount && xTimeToTransmit < xNextDelayedMessage )
				fIsNextMessage = true;
		}
		else
		{
			// The wake time has not overflowed, so the current block list is used.
			vListInsert( pxDelayedMessagesList, pxLI );

			// If the previous next message is in the overflow list we have a new message that needs to be transmitted earlier.
			// Otherwise, the time to transmit determines a delay recalculation.
			if( xNextDelayedMessage < xConstTickCount || xTimeToTransmit < xNextDelayedMessage )
				fIsNextMessage = true;
		}
		(void) xTaskResumeAll( );

		if( fIsNextMessage )
		{
			xNextDelayedMessage = xTimeToTransmit;
			if( !xTaskAbortDelay( xECANDelayedHandle ) )	//Update task delay
				xTaskNotify( xECANDelayedHandle, 0, eNoAction );
		}
	}


	{
		const ecan_msg_t *const pxMsg = (ecan_msg_t *) listGET_LIST_ITEM_OWNER( pxLI );
		xDelayedPriorityCount[ pxMsg->puxPriority ]++;

		//Update priority
		for( UBaseType_t uxPriority = configMAX_PRIORITIES - 1; pxMsg->puxPriority; ++uxPriority )
			if( xDelayedPriorityCount[ uxPriority ] )
				goto DELAYED_SKIP_PRIO;

		vTaskPrioritySet( xECANDelayedHandle, pxMsg->puxPriority );
	}
DELAYED_SKIP_PRIO:
	xSemaphoreGive( xDelayedMutexHandle );
}

bool xECANAbortTransmit( ListItem_t *const pxLI )
{
	xSemaphoreTake( xDelayedMutexHandle, portMAX_DELAY );
	xSemaphoreTake( xTransmitMutexHandle, portMAX_DELAY );

	if( listLIST_ITEM_CONTAINER( pxLI ) == NULL )
	{
		xSemaphoreGive( xTransmitMutexHandle );
		xSemaphoreGive( xDelayedMutexHandle );
		return false;
	}
	else if( listIS_CONTAINED_WITHIN( &xReadyMessagesList, pxLI ) )
	{
		//pxLI is in the transmit list, release the delayed mutex
		xSemaphoreGive( xDelayedMutexHandle );

		uxListRemove( pxLI );
		vTaskPrioritySet( xECANTransmitHandle, (UBaseType_t) listGET_ITEM_VALUE_OF_HEAD_ENTRY( &xReadyMessagesList ) );
		xSemaphoreGive( xTransmitMutexHandle );
		return true;
	}
	else
	{
		//pxLI is in the delayed list, release the transmit mutex
		xSemaphoreGive( xTransmitMutexHandle );

		vTaskSuspendAll( );
		uxListRemove( pxLI );

		//Set the tick count for the next delayed message.
		//If both lists are empty xNextDelayedMessage will be invalid. Upon waking up the delay task will notice that both lists are empty and suspend itself.
		if( listLIST_IS_EMPTY( pxDelayedMessagesList ) )
			xNextDelayedMessage = listGET_ITEM_VALUE_OF_HEAD_ENTRY( pxOverflowDelayedMessagesList );
		else
			xNextDelayedMessage = listGET_ITEM_VALUE_OF_HEAD_ENTRY( pxDelayedMessagesList );
		(void) xTaskResumeAll( );
		if( !xTaskAbortDelay( xECANDelayedHandle ) )
			xTaskNotify( xECANDelayedHandle, 0, eNoAction );

		//Priority management
		{
			ecan_msg_t *const pxMsg = (ecan_msg_t *) listGET_LIST_ITEM_OWNER( pxLI );
			listSET_LIST_ITEM_VALUE( pxLI, pxMsg->puxPriority );	//Set the item value to the priority
			xDelayedPriorityCount[ pxMsg->puxPriority ]--;			//Decrease this tasks priority counter

			//Search for the current top priority.
			//If no more messages are in the lists, xDelayedPriorityCount[ 0 ] is set to 1, forcing a priority of 0.
			for( UBaseType_t uxPriority = configMAX_PRIORITIES - 1; uxPriority != (UBaseType_t) -1; --uxPriority )
				if( xDelayedPriorityCount[ uxPriority ] )
				{
					vTaskPrioritySet( NULL, uxPriority );
					break;
				}
		}
		xSemaphoreGive( xDelayedMutexHandle );
		return true;
	}
}

void __interrupt( irq( IRQ_RXB1IF ), base( 8 ), low_priority ) prvECANReceiveISR( void )
{
	ECANCONbits.EWIN = 0x10 + CANCONbits.FP;	//Set EWIN to map current FIFO buffer into access bank
	PIE5bits.RXBnIE = 0;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR( xECANReceiveHandle, 0, eNoAction, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void vECANSwitchDelayedLists( void )
{
	while( !listLIST_IS_EMPTY( pxDelayedMessagesList ) )
	{
		//The delayed list is not yet empty but being switched out.
		//Move all remaining messages into the overflowed list so that they are transmitted as early as possible.
		ListItem_t *const pxLI = listGET_HEAD_ENTRY( pxDelayedMessagesList );
		listSET_LIST_ITEM_VALUE( pxLI, 0 );	//vECANSwitchDelayedLists is only called when xTickCount == 0
		vListInsert( pxOverflowDelayedMessagesList, pxLI );
	}

	List_t *const pxTemp = pxDelayedMessagesList;
	pxDelayedMessagesList = pxOverflowDelayedMessagesList;
	pxOverflowDelayedMessagesList = pxTemp;
}

//#define CAN_SET_TX
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
#ifdef CAN_SET_TX
	BSEL0 = 0x80;
#else
	BSEL0 = 0x00;
#endif
	
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
#ifdef CAN_SET_TX
	RXFBCON0 = 0x7;
#endif
	
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
#ifdef CAN_SET_TX
	RXF15EIDH = 0x00;
	RXF15EIDL = 0x00;
	RXF15SIDH = 0x00;
	RXF15SIDL = 0x00;
#else
	RXF15EIDH = 0xFF;
	RXF15EIDL = 0xFF;
	RXF15SIDH = 0xFF;
	RXF15SIDL = 0xE3;
#endif

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
#if configTASK_NOTIFICATION_ARRAY_ENTRIES == 1
	xTransmitEventHandle = xSemaphoreCreateBinaryStatic( &xTransmitEventBuffer );
#endif
	configASSERT( xTransmitMutexHandle );
	vListInitialise( &xReadyMessagesList );
	xECANTransmitHandle = xTaskCreateStatic( prvECANTransmitTask, (const portCHAR*) "ECANTX", stackSIZE_CANTX, NULL, 0, xECANTransmitStack, &xECANTransmitBuffer );
	
	//Initialise receiver
	xReceiveMutexHandle = xSemaphoreCreateMutexStatic( &xReceiveMutexBuffer );
	configASSERT( xReceiveMutexHandle );
	vListInitialise( &xReceiveList );
	xECANReceiveHandle = xTaskCreateStatic( prvECANReceiveTask, (const portCHAR*) "ECANRX", stackSIZE_CANRX, NULL, 0, xECANReceiveStack, &xECANReceiveBuffer );

	//Initialise delay task
	xDelayedMutexHandle = xSemaphoreCreateMutexStatic( &xDelayedMutexBuffer );
	configASSERT( xDelayedMutexHandle );
	vListInitialise( &xDelayedMessageList1 );
	vListInitialise( &xDelayedMessageList2 );
	pxDelayedMessagesList = &xDelayedMessageList1;
	pxOverflowDelayedMessagesList = &xDelayedMessageList2;
	xECANDelayedHandle = xTaskCreateStatic( prvECANDelayedTask, (const portCHAR*) "ECANDEL", stackSIZE_CANDEL, NULL, 0, xECANDelayedStack, &xECANDelayedBuffer );

	//Enable interrupts
	TXBIEbits.TXB0IE = 1;
	TXBIEbits.TXB1IE = 1;
	TXBIEbits.TXB2IE = 1;

	TXB0CONbits.TXBIF = 1;
	TXB1CONbits.TXBIF = 1;
	TXB2CONbits.TXBIF = 1;

	PIE5bits.ERRIE = 1;
	PIE5bits.IRXIE = 1;
}

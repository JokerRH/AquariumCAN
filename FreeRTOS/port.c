#include <pic18f25k83.h>

#include "FreeRTOS.h"
#include "task.h"
#include "../RTRRec/stack.h"

uint8_t ucCriticalNesting = 0;	//Counter for critical nestings. Saved and restored by each task during context switch
StackType_t uxIdleTaskStack[ stackSIZE_IDLE ];

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the PIC port.
 *----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the scheduler for the PIC port will get stopped
	once running.  If required disable the tick interrupt here, then return 
	to xPortStartScheduler(). */
}

/*
	configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
	implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
	used by the Idle task.
*/
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	/*  If the buffers to be provided to the Idle task are declared inside this
	function then they must be declared static ? otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xIdleTaskTCB;

	/* Pass out a pointer to the StaticTask_t structure in which the Idle task?s
	state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task?s stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = stackSIZE_IDLE;
}

#include "../RTRRec/peripherals/device_config.h"
#include <xc.h>
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
	LATAbits.LATA4 = 1;
	LATAbits.LATA5 = 1;
	while( 1 )
	{
		LATAbits.LATA4 = ~LATAbits.LATA4;
		LATAbits.LATA5 = ~LATAbits.LATA5;
		__delay_ms( 1000 );
	}
}

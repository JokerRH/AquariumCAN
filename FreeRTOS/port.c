/*
 * FreeRTOS Kernel V10.4.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 * 1 tab == 4 spaces!
 */

/* 
Changes between V1.2.4 and V1.2.5

	+ Introduced portGLOBAL_INTERRUPT_FLAG definition to test the global 
	  interrupt flag setting.  Using the two bits defined within
	  portINITAL_INTERRUPT_STATE was causing the w register to get clobbered
	  before the test was performed.

Changes from V1.2.5

	+ Set the interrupt vector address to 0x08.  Previously it was at the
	  incorrect address for compatibility mode of 0x18.

Changes from V2.1.1

	+ PCLATU and PCLATH are now saved as part of the context.  This allows
	  function pointers to be used within tasks.  Thanks to Javier Espeche
	  for the enhancement. 

Changes from V2.3.1

	+ TABLAT is now saved as part of the task context.
	
Changes from V3.2.0

	+ TBLPTRU is now initialised to zero as the MPLAB compiler expects this
	  value and does not write to the register.
*/

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* MPLAB library include file. */
#include "timers.h"

#define STRINGIFY( s )	#s
asm("GLOBAL _prvIdleTask");

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the PIC port.
 *----------------------------------------------------------*/

/* Hardware setup for tick. */
#define portTIMER_FOSC_SCALE			( ( uint32_t ) 4 )

/* Initial interrupt enable state for newly created tasks.  This value is
copied into INTCON when a task switches in for the first time. */
#define portINITAL_INTERRUPT_STATE			0xC0

/* Just the bit within INTCON for the global interrupt flag. */
#define portGLOBAL_INTERRUPT_FLAG			0x80

/* Constant used for context switch macro when we require the interrupt 
enable state to be unchanged when the interrupted task is switched back in. */
#define portINTERRUPTS_UNCHANGED			0x00

/* Some memory areas get saved as part of the task context.  These memory
area's get used by the compiler for temporary storage, especially when 
performing mathematical operations, or when using 32bit data types.  This
constant defines the size of memory area which must be saved. */
#define portCOMPILER_MANAGED_MEMORY_SIZE	( ( uint8_t ) 0x13 )

/* We require the address of the pxCurrentTCB variable, but don't want to know
any details of its type. */
typedef void TCB_t;
extern volatile TCB_t *volatile pxCurrentTCB;

/* IO port constants. */
#define portBIT_SET		( ( uint8_t ) 1 )
#define portBIT_CLEAR	( ( uint8_t ) 0 )

/*
 * The serial port ISR's are defined in serial.c, but are called from portable
 * as they use the same vector as the tick ISR.
 */
void vSerialTxISR( void );
void vSerialRxISR( void );

/*
 * Perform hardware setup to enable ticks.
 */
//static
void prvSetupTimerInterrupt( void );

/* 
 * ISR to maintain the tick, and perform tick context switches if the
 * preemptive scheduler is being used.
 */
static void prvTickISR( void );

/*
 * ISR placed on the low priority vector.  This calls the appropriate ISR for
 * the actual interrupt.
 */
static void prvLowInterrupt( void );

uint8_t ucCriticalCount = 0;

/* 
 * Macro that pushes all the registers that make up the context of a task onto
 * the stack, then saves the new top of stack into the TCB.
 * 
 * If this is called from an ISR then the interrupt enable bits must have been 
 * set for the ISR to ever get called.  Therefore we want to save the INTCON
 * register with the enable bits forced to be set - and ucForcedInterruptFlags 
 * must contain these bit settings.  This means the interrupts will again be
 * enabled when the interrupted task is switched back in.
 *
 * If this is called from a manual context switch (i.e. from a call to yield),
 * then we want to save the INTCON so it is restored with its current state,
 * and ucForcedInterruptFlags must be 0.  This allows a yield from within
 * a critical section.
 *
 * The compiler uses some locations at the bottom of the memory for temporary
 * storage during math and other computations.  This is especially true if
 * 32bit data types are utilised (as they are by the scheduler).  The .tmpdata
 * and MATH_DATA sections have to be stored in there entirety as part of a task
 * context.  This macro stores from data address 0x00 to 
 * portCOMPILER_MANAGED_MEMORY_SIZE.  This is sufficient for the demo 
 * applications but you should check the map file for your project to ensure 
 * this is sufficient for your needs.  It is not clear whether this size is 
 * fixed for all compilations or has the potential to be program specific.
 */

/* 
 * See header file for description. 
 */
#define PORT_PUSH( x )  do{ *pxTopOfStack = ( x ); pxTopOfStack++; } while( 0 )
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
    uint32_t ulAddress;
    uint8_t ucBlock;

	/* Place a few bytes of known values on the bottom of the stack. 
	This is just useful for debugging. */
    PORT_PUSH( 0x11 );
    PORT_PUSH( 0x22 );
    PORT_PUSH( 0x33 );

	/*
        Simulate how the stack would look after a call to vPortYield() generated by the compiler. 
        First store the function parameters.  This is where the task will expect to find them when it starts running.
    */
	ulAddress = (uint32_t) pvParameters;
    PORT_PUSH( (StackType_t) ( ulAddress & ( uint32_t ) 0x00ff ) );
    ulAddress >>= 8;
    PORT_PUSH( (StackType_t) ( ulAddress & ( uint32_t ) 0x00ff ) );

	/* Next we just leave a space.  When a context is saved the stack pointer
	is incremented before it is used so as not to corrupt whatever the stack
	pointer is actually pointing to.  This is especially necessary during 
	function epilogue code generated by the compiler. */
	PORT_PUSH( 0x44 );

	/* Next are all the registers that form part of the task context. */
    PORT_PUSH( 0xcc );                          //STATUS
    PORT_PUSH( 0x66 );                          //WREG
	PORT_PUSH( 0x11 );                          //BSR
    PORT_PUSH( 0x00 );                          //PCLATH
    PORT_PUSH( 0x00 );                          //PCLATU
    PORT_PUSH( 0x44 );                          //FSR0L
	PORT_PUSH( 0x55 );                          //FSR0H
    PORT_PUSH( 0x44 );                          //FSR1L
	PORT_PUSH( 0x55 );                          //FSR1H
	PORT_PUSH( 0x22 );                          //FSR2L
	PORT_PUSH( 0x33 );                          //FSR2H
    PORT_PUSH( 0xbb );                          //PRODL
    PORT_PUSH( 0xaa );                          //PRODH
	PORT_PUSH( 0x66 ); //TABLAT
    PORT_PUSH( 0x99 ); //TBLPTRUL
    PORT_PUSH( 0x88 ); //TBLPTRUH
	PORT_PUSH( 0x00 ); //TBLPTRU
	
	

	/* The only function return address so far is the address of the 
	task. */
	ulAddress = (uint32_t) pxCode;
	PORT_PUSH( (StackType_t) ( ulAddress & ( uint32_t ) 0x00ff ) ); /* TOS low. */
	ulAddress >>= 8;
	PORT_PUSH( (StackType_t) ( ulAddress & ( uint32_t ) 0x00ff ) ); /* TOS high. */
	ulAddress >>= 8;
	PORT_PUSH( (StackType_t) ( ulAddress & ( uint32_t ) 0x00ff ) );    /* TOS even higher. */

	/* Store the number of return addresses on the hardware stack - so far only
	the address of the task entry point. */
	PORT_PUSH( 1 );

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the scheduler for the PIC port will get stopped
	once running.  If required disable the tick interrupt here, then return 
	to xPortStartScheduler(). */
}

/*-----------------------------------------------------------*/

/*
 * Setup a timer for a regular tick.
 */
//static
void prvSetupTimerInterrupt( void )
{
	const uint32_t ulConstCompareValue = 0xFFF7;//( ( configCPU_CLOCK_HZ / portTIMER_FOSC_SCALE ) / configTICK_RATE_HZ );
	uint32_t ulCompareValue;
	uint8_t ucByte;

	/*
	Interrupts are disabled when this function is called.
	Setup CCP1 to provide the tick interrupt using a compare match on timer	1.
	Clear the time count then setup timer.
	*/
	TMR1H = (uint8_t) 0x00;
	TMR1L = (uint8_t) 0x00;

	/* Set the compare match value. */
	ulCompareValue = ulConstCompareValue;
	CCPR1L = (uint8_t) ( ulCompareValue & (uint32_t) 0xff );
	ulCompareValue >>= (uint32_t) 8;
	CCPR1H = (uint8_t) ( ulCompareValue & (uint32_t) 0xff );	

	CCP1CON = 0x81;					// MODE Toggle_cleartmr; EN enabled; FMT right_aligned
	CCPTMRS0bits.C1TSEL = 0x1;		// Selecting Timer 1
	PIR4bits.CCP1IF = 0;			// Clear the CCP1 interrupt flag
	PIE4bits.CCP1IE = portBIT_SET;	// Interrupt enable

	/* We are only going to use the global interrupt bit, so set the peripheral	bit to true. */
	INTCON0bits.GIEL = portBIT_SET;

	//Set up the timer that will produce the tick.
	T1GCON = 0x00;	//T1GE disabled; T1GTM disabled; T1GPOL low; T1GGO done; T1GSPM disabled
	T1GATE = 0x00;	//GSS T1G_pin
	T1CLK = 0x01;	//CS FOSC/4

	PIR4bits.TMR1IF = 0;	//Clearing IF flag.

	T1CON = 0x33;   //CKPS 1:8; NOT_SYNC synchronize; TMR1ON enabled; T1RD16 enabled
}

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /*  If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static ? otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task?s
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task?s stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

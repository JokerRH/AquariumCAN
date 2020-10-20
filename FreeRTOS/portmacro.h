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

#ifndef PORTMACRO_H
#define PORTMACRO_H

#include <xc.h>
#include <stdint.h>

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		int
#define portSTACK_TYPE	uint8_t
#define portBASE_TYPE	char

typedef portSTACK_TYPE StackType_t;
typedef signed char BaseType_t;
typedef unsigned char UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif
/*-----------------------------------------------------------*/

/* Hardware specifics. */
#define portBYTE_ALIGNMENT			1
#define portGLOBAL_INT_ENABLE_BIT	_INTCON0_GIEH_MASK
#define portSTACK_GROWTH			1
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
/*-----------------------------------------------------------*/
    
/* Critical section management. */
#define portDISABLE_INTERRUPTS( )	INTCON0bits.GIEH = 0
#define portENABLE_INTERRUPTS( )	INTCON0bits.GIEH = 1

extern uint8_t ucCriticalNesting;
#define portENTER_CRITICAL( )	portDISABLE_INTERRUPTS( );\
								ucCriticalNesting++

#define portEXIT_CRITICAL( )	if( --ucCriticalNesting == 0 )\
									portENABLE_INTERRUPTS( )

/*-----------------------------------------------------------*/

//portYIELD may be called from within a section with interrupts disabled
//In this case, the interrupt will be executed _after_ portENABLE_INTERRUPTS has run, allowing the context switcher to return with interrupts disabled.
//If interrupts were already enabled, portENABLE_INTERRUPTS is called superfluously and has no effect.
#define portYIELD( )	{\
							PIR0bits.SWIF = 1;\
							portENABLE_INTERRUPTS( );\
						}

//If a high priority interrupt is executed within a critical section do not allow a yield to switch the context!
//Instead, the interrupt flag will remain set until the critical section is exited, upon which the yield is executed.
#define portYIELD_FROM_ISR( x )	if( x )\
									PIR0bits.SWIF = 1

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )
/*-----------------------------------------------------------*/

/* Required by the kernel aware debugger. */
#ifdef __DEBUG
	#define portREMOVE_STATIC_QUALIFIER
#endif

extern void vECANSwitchDelayedLists( void );
#define portTICK_OVERFLOW( )	vECANSwitchDelayedLists( )

#endif /* PORTMACRO_H */


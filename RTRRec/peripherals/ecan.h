#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <FreeRTOS/include/list.h>

typedef void( *txcallback_t )( void );	//Use pxECANCurrentTxLI and pxECANCurrentMsg to access the corresponding list item/message
typedef struct
{
	uint8_t ucD7;
	uint8_t ucD6;
	uint8_t ucD5;
	uint8_t ucD4;
	uint8_t ucD3;
	uint8_t ucD2;
	uint8_t ucD1;
	uint8_t ucD0;
	uint8_t ucDLC;
	uint8_t ucEIDL;
	uint8_t ucEIDH;
	uint8_t ucSIDL;
	uint8_t ucSIDH;
	txcallback_t pvCallback;
	UBaseType_t puxPriority;
} ecan_msg_t;

typedef bool( *rxcallback_t )( void );

extern ListItem_t *pxECANCurrentTxLI;
extern ecan_msg_t *pxECANCurrentMsg;
extern ListItem_t *pxECANCurrentRxLI;

__reentrant void ECAN_Initialize( void );
void vECANTransmit( ListItem_t *pxLI );
void vECANReceive( ListItem_t *pxLI );
void vECANTransmitDelayed( ListItem_t *pxLI, TickType_t xTicksToWait );
bool xECANAbortTransmit( ListItem_t *const pxLI );

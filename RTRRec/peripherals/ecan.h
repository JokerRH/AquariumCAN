#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <FreeRTOS/include/list.h>

#define ecanNOTIFICATION_IDX	0

typedef void( *txcallback_t )( ListItem_t *pxLI );
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
} ecan_msg_t;

typedef bool( *rxcallback_t )( ListItem_t *pxLI );

void ECAN_Initialize( void );

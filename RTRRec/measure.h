#pragma once

#define messagePRIORITY_MESSAGE		3
#define messagePRIORITY_BLINK		3
#define messagePRIORITY_CALIBRATE	3

#define measureDELAY_TICKS	( (TickType_t) 2000 / portTICK_PERIOD_MS )

void vMeasureInitialize( void );
#pragma once

#include <../FreeRTOS/FreeRTOSConfig.h>

#define stackSIZE_IDLE	( configMINIMAL_STACK_SIZE + 64 )
#define stackSIZE_CANTX	( configMINIMAL_STACK_SIZE + 64 )
#define stackSIZE_CANRX	( configMINIMAL_STACK_SIZE + 64 )
#define stackSIZE_TEST	( configMINIMAL_STACK_SIZE + 64 )
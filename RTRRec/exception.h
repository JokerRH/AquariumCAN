#pragma once
#include <stdint.h>

#define exceptionINVALID_CALIBRATION	( 1 << 0 )

typedef uint8_t exception_t;

void SetException( exception_t exception );
void ClearException( exception_t exception );

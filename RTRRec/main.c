#include "peripherals/device_config.h"
#include "peripherals/ecan.h"
#include "peripherals/oscillator.h"
#include "peripherals/pin_manager.h"
#include "peripherals/pmd.h"

void main( )
{
	PMD_Initialize( );
	PIN_MANAGER_Initialize( );
	OSCILLATOR_Initialize( );
	ECAN_Initialize( );

	IO_RA4_SetHigh( );
	IO_RA5_SetHigh( );
	while( 1 )
	{
		IO_RA4_Toggle( );
		IO_RA5_Toggle( );
		__delay_ms( 500 );
		IO_RA4_Toggle( );
		__delay_ms( 500 );
	}
}

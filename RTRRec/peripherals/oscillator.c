#include <xc.h>
#include "oscillator.h"

void OSCILLATOR_Initialize( void )
{
	OSCCON1 = 0x70;	//NOSC EXTOSC; NDIV 1
	OSCCON3 = 0x00;	//CSWHOLD may proceed; SOSCPWR Low power
	OSCEN = 0x00;	//MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled
	OSCFRQ = 0x02;	//HFFRQ 4_MHz
	OSCTUNE = 0x00;	//TUN 0
}

#include <xc.h>
#include <stdbool.h>
#include "interrupt_manager.h"

void INTERRUPT_Initialize( )
{
	INTCON0bits.IPEN = 1;

	bool state = (unsigned char) GIE;
	GIE = 0;
	IVTLOCK = 0x55;
	IVTLOCK = 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x00; // unlock IVT

	IVTBASEU = 0;
	IVTBASEH = 0;
    //IVTBASEL = 0xA8;
    IVTBASEL = 8;

	IVTLOCK = 0x55;
	IVTLOCK = 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x01; // lock IVT

	GIE = state;
	// Assign peripheral interrupt priority vectors
	//IPR4bits.CCP1IP = 0;
}

#if 1
void __interrupt(irq(default),base(8)) Default_ISR()
{

}
#endif

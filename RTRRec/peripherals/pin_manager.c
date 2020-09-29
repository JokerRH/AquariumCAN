#include <xc.h>
#include "pin_manager.h"

void PIN_MANAGER_Initialize(void)
{
	/**
	LATx registers
	*/
	LATA = 0x00;
	LATB = 0x00;
	LATC = 0x00;

	/**
	TRISx registers
	*/
	TRISA = 0xCF;
	TRISB = 0xD0;
	TRISC = 0x53;

	/**
	ANSELx registers
	*/
	ANSELC = 0x03;
	ANSELB = 0xCF;
	ANSELA = 0xC9;

	/**
	WPUx registers
	*/
	WPUE = 0x00;
	WPUB = 0x00;
	WPUA = 0x00;
	WPUC = 0x00;

	/**
	ODx registers
	*/
	ODCONA = 0x30;
	ODCONB = 0x00;
	ODCONC = 0x0C;

	/**
	SLRCONx registers
	*/
	SLRCONA = 0xFF;
	SLRCONB = 0xFF;
	SLRCONC = 0xFF;

	/**
	INLVLx registers
	*/
	INLVLA = 0xFF;
	INLVLB = 0xFF;
	INLVLC = 0xFF;
	INLVLE = 0x08;
}
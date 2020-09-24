#include <xc.h>
#include "ecan.h"

void ECAN_Initialize( void )
{
	CANCON = 0x80;
	while( 0x80 != ( CANSTAT & 0xE0 ) ); //Wait until ECAN is in config mode

	ECANCON = 0x90;	//Mode 2

	/**
	Initialize CAN I/O
	*/
	CIOCON = 0x00;
	
	/**
	Mask and Filter definitions
	........................................................	
	CAN ID		ID Type		Mask				Filter		Buffer	
	........................................................	
	0x124		SID		Acceptance Mask 0		Filter 0	FIFO
	........................................................
	*/
	
	/**
	Configure Generic Buffers to be Transmit or Receive
	*/
	BSEL0 = 0x00;
	
	/**	
		Initialize Receive Masks
	*/
	RXM0EIDH = 0xFF;
	RXM0EIDL = 0xFF;
	RXM0SIDH = 0xFF;
	RXM0SIDL = 0xE3;
	RXM1EIDH = 0xFF;
	RXM1EIDL = 0xFF;
	RXM1SIDH = 0xFF;
	RXM1SIDL = 0xE3;
	
	/**
	Enable Filters
	*/
	RXFCON0 = 0x01;
	RXFCON1 = 0x00;
	
	/**
	Assign Filters to Masks
	*/
	MSEL0 = 0x00;
	MSEL1 = 0x00;
	MSEL2 = 0x00;
	MSEL3 = 0x00;
	
	/**
	Initialize Receive Filters
	*/
	
	RXF0EIDH = 0x00;
	RXF0EIDL = 0x00;
	RXF0SIDH = 0x24;
	RXF0SIDL = 0x80;
	RXF1EIDH = 0x00;
	RXF1EIDL = 0x00;
	RXF1SIDH = 0x00;
	RXF1SIDL = 0x00;
	RXF2EIDH = 0x00;
	RXF2EIDL = 0x00;
	RXF2SIDH = 0x00;
	RXF2SIDL = 0x00;
	RXF3EIDH = 0x00;
	RXF3EIDL = 0x00;
	RXF3SIDH = 0x00;
	RXF3SIDL = 0x00;
	RXF4EIDH = 0x00;
	RXF4EIDL = 0x00;
	RXF4SIDH = 0x00;
	RXF4SIDL = 0x00;
	RXF5EIDH = 0x00;
	RXF5EIDL = 0x00;
	RXF5SIDH = 0x00;
	RXF5SIDL = 0x00;
	
	RXF6EIDH = 0x00;
	RXF6EIDL = 0x00;
	RXF6SIDH = 0x00;
	RXF6SIDL = 0x00;
	RXF7EIDH = 0x00;
	RXF7EIDL = 0x00;
	RXF7SIDH = 0x00;
	RXF7SIDL = 0x00;
	RXF8EIDH = 0x00;
	RXF8EIDL = 0x00;
	RXF8SIDH = 0x00;
	RXF8SIDL = 0x00;
	RXF9EIDH = 0x00;
	RXF9EIDL = 0x00;
	RXF9SIDH = 0x00;
	RXF9SIDL = 0x00;
	RXF10EIDH = 0x00;
	RXF10EIDL = 0x00;
	RXF10SIDH = 0x00;
	RXF10SIDL = 0x00;
	RXF11EIDH = 0x00;
	RXF11EIDL = 0x00;
	RXF11SIDH = 0x00;
	RXF11SIDL = 0x00;
	RXF12EIDH = 0x00;
	RXF12EIDL = 0x00;
	RXF12SIDH = 0x00;
	RXF12SIDL = 0x00;
	RXF13EIDH = 0x00;
	RXF13EIDL = 0x00;
	RXF13SIDH = 0x00;
	RXF13SIDL = 0x00;
	RXF14EIDH = 0x00;
	RXF14EIDL = 0x00;
	RXF14SIDH = 0x00;
	RXF14SIDL = 0x00;
	RXF15EIDH = 0xFF;
	RXF15EIDL = 0xFF;
	RXF15SIDH = 0xFF;
	RXF15SIDL = 0xE3;

	/**
	Initialize CAN Timings
	*/
	
   /**
	Baud rate: 500kbps
	System frequency: 16000000
	ECAN clock frequency: 16000000
	Time quanta: 8
	Sample point: 1-1-4-2
	Sample point: 75%
	*/
	
	BRGCON1 = 0x01;
	BRGCON2 = 0x98;
	BRGCON3 = 0x81;

	
	CANCON = 0x00;
	while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode
}
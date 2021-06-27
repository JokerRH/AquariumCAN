#include "log.h"
#include <stdint.h>
#include <pic18f25k83.h>

char sdout[ 10 ];

void vUARTInitialize( void )
{
	sdout[ 9 ] = '\n';

	//UART 1
	U1P1L = 0x00;	// P1L 0; 
	U1P1H = 0x00;	// P1H 0; 
	U1P2L = 0x00;	// P2L 0; 
	U1P2H = 0x00;	// P2H 0; 
	U1P3L = 0x00;	// P3L 0; 
	U1P3H = 0x00;	// P3H 0; 
	U1CON0 = 0xA0;	// BRGS high speed; MODE Asynchronous 8-bit mode; RXEN disabled; TXEN enabled; ABDEN disabled; 
	U1CON1 = 0x80;	// RXBIMD Set RXBKIF on rising RX input; BRKOVR disabled; WUE disabled; SENDB disabled; ON enabled; 
	U1CON2 = 0x00;	// TXPOL not inverted; FLO off; C0EN Checksum Mode 0; RXPOL not inverted; RUNOVF RX input shifter stops all activity; STP Transmit 1Stop bit, receiver verifies first Stop bit; 
	U1BRGL = 0xA0;	// BRGL 160; 
	U1BRGH = 0x01;	// BRGH 1; 
	U1FIFO = 0x00;	// STPMD in middle of first Stop bit; TXWRE No error; 
	U1UIR = 0x00;	// ABDIF Auto-baud not enabled or not complete; WUIF WUE not enabled by software; ABDIE disabled; 
	U1ERRIR = 0x00;	// ABDOVF Not overflowed; TXCIF 0; RXBKIF No Break detected; RXFOIF not overflowed; CERIF No Checksum error; 
	U1ERRIE = 0x00;	// TXCIE disabled; FERIE disabled; TXMTIE disabled; ABDOVE disabled; CERIE disabled; RXFOIE disabled; PERIE disabled; RXBKIE disabled; 
	
	//DMA 1
	DMA1SSA = (uint24_t) &sdout;	//Source Address : sdout
	DMA1DSA = (uint16_t) &U1TXB;	//Destination Address : &U1TXB
	DMA1CON1 = 0x03;	//DMODE unchanged; DSTP not cleared; SMR GPR; SMODE incremented; SSTP cleared; 
	DMA1SSZ = 10;		//Source Message Size : 10
	DMA1DSZ = 1;		//Destination Message Size : 1
	DMA1SIRQ = 0x1C;	//Start Trigger : SIRQ U1TX; 
	DMA1AIRQ = 0x00;	//Abort Trigger : AIRQ None; 
	DMA1CON0 = 0xC0;	//EN enabled; SIRQEN enabled; DGO not in progress; AIRQEN disabled; 

	//Change priorities
	PRLOCK = 0x55;
	PRLOCK = 0xAA;
	PRLOCKbits.PRLOCKED = 0;
	
	DMA1PR = 0;
	ISRPR = 1;
	MAINPR = 2;
	
	PRLOCK = 0x55;
	PRLOCK = 0xAA;
	PRLOCKbits.PRLOCKED = 1;
}

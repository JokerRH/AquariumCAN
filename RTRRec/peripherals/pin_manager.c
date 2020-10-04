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
    
    I2C1SDAPPS = 0x12;   //RC2->I2C1:SDA1;    
    RC3PPS = 0x21;   //RC3->I2C1:SCL1;    
    RB5PPS = 0x33;   //RB5->ECAN:CANTX0;    
    CANRXPPS = 0x0C;   //RB4->ECAN:CANRX;    
    RC2PPS = 0x22;   //RC2->I2C1:SDA1;    
    RC5PPS = 0x13;   //RC5->UART1:TX1;    
    U1RXPPS = 0x16;   //RC6->UART1:RX1;    
    RC7PPS = 0x15;   //RC7->UART1:RTS1;    
    I2C1SCLPPS = 0x13;   //RC3->I2C1:SCL1;    
    U1CTSPPS = 0x14;   //RC4->UART1:CTS1;   
}
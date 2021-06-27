#pragma once

#define LOG_INIT	vUARTInitialize( )
#define LOG( x0, x1, x2, x3, x4, x5, x6, x7, x8 )\
	sdout[ 0 ] = x0;\
	sdout[ 1 ] = x1;\
	sdout[ 2 ] = x2;\
	sdout[ 3 ] = x3;\
	sdout[ 4 ] = x4;\
	sdout[ 5 ] = x5;\
	sdout[ 6 ] = x6;\
	sdout[ 7 ] = x7;\
	sdout[ 8 ] = x8;\
	DMA1CON0bits.SIRQEN = 1;\
	while( DMA1CON0bits.SIRQEN );\
	while( 0 == U1FIFObits.TXBE )

extern char sdout[ 10 ];

void vUARTInitialize( void );

#include "measure.h"
#include "FreeRTOS.h"
#include "stack.h"
#include "priority.h"
#include "peripherals/ecan.h"
#include <FreeRTOS/include/task.h>

TaskHandle_t xHandleMessage;
static StackType_t xStackMessage[ stackSIZE_MESSAGE ];
static StaticTask_t xBufferMessage;

TaskHandle_t xHandleCalibrate;
static StackType_t xStackCalibrate[ stackSIZE_CALIBRATE ];
static StaticTask_t xBufferCalibrate;

static ListItem_t xMeasureLI;
static ecan_msg_t xMeasureMsg;

static volatile uint16_t s_wDutyCycle = 0;	//10 bit duty cycle, left aligned (i.e. shifted left 6 bit)
uint16_t xMeasureM = 327;		// = 3 / 602 * 0x10000
uint32_t xMeasureB = 1127611 + 128;	// = 5179 / 301 * 0x10000 [ + '0.5' in 8.8 FPN]

static uint8_t sdout[ 10 ];

asm( "GLOBAL _prvMsgCallback" );
static void prvMsgCallback( void )
{

}

static void vUARTInitialize( void )
{
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
	DMA1SSA = &sdout;	//Source Address : sdout
	DMA1DSA = &U1TXB;	//Destination Address : &U1TXB
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

asm( "GLOBAL _vTaskMeasure" );
void vTaskMeasure( void *pvParameters )
{
	//Initialice the ADC
	ADLTHL = 0x00;	// ADLTH 0
	ADLTHH = 0x00;	// ADLTH 0
	ADUTHL = 0x00;	// ADUTH 0
	ADUTHH = 0x00;	// ADUTH 0
	ADSTPTL = 0x00;	// ADSTPT 0
	ADSTPTH = 0x00;	// ADSTPT 0
	ADACCU = 0x00;	// ADACC 0
	ADRPT = 0x20;	// ADRPT 32
	ADPCH = 0x00;	// ADPCH ANA0
	ADACQL = 0x06;	// ADACQ 6 (10.2us) => 93°C (10.18us)
	ADACQH = 0x00;	// ADACQ 0
	ADCAP = 0x1F;	// ADCAP Additional uC of 31pF
	ADPREL = 0x00;	// ADPRE 0
	ADPREH = 0x00;	// ADPRE 0
	ADCON1 = 0x00;	// ADDSEN disabled; ADGPOL digital_low; ADIPEN disabled; ADPPOL Vss
	ADCON2 = 0x53;	// ADCRS 5; ADMD Burst_average_mode; ADACLR disabled; ADPSIS RES
	ADCON3 = 0x07;	// ADCALC First derivative of Single measurement; ADTMD enabled; ADSOI ADGO not cleared
	ADSTAT = 0x00;	// ADMATH registers not updated
	ADREF = 0x02;	// ADNREF VSS; ADPREF external
	ADACT = 0x00;	// ADACT disabled
	ADCLK = 0x00;	// ADCS FOSC/2
	ADCON0 = 0x94;	// ADGO stop; ADFM right; ADON enabled; ADCS Frc; ADCONT disabled

	PIR1bits.ADTIF = 0;	// Clear the ADC Threshold interrupt flag
	IPR1bits.ADTIP = 1;	// Set the ADC Threshold interrupt to high priority
	PIE1bits.ADTIE = 1;	// Enable the ADCC threshold interrupt

	//Initialise CAN message
	xMeasureMsg.pvCallback = prvMsgCallback;
	vListInitialiseItem( &xMeasureLI );
	listSET_LIST_ITEM_OWNER( &xMeasureLI, &xMeasureMsg );
	convertCANid2Reg( 0x117, &xMeasureMsg.ucEIDH, &xMeasureMsg.ucEIDL, &xMeasureMsg.ucSIDH, &xMeasureMsg.ucSIDL );

	vUARTInitialize( );
	sdout[ 0 ] = 0xAA;
	sdout[ 1 ] = 0;
	sdout[ 2 ] = 0;
	sdout[ 3 ] = 0xBB;
	sdout[ 4 ] = 0;
	sdout[ 5 ] = 0;
	sdout[ 6 ] = 0xCC;
	sdout[ 7 ] = 0;
	sdout[ 8 ] = 0;
	sdout[ 9 ] = '\n';

	TickType_t xLastWakeTime = xTaskGetTickCount( );
	while( 1 )
	{
		//Enable ADC charge pump
		//ADCPbits.CPON = 1;
		//while( ADCPbits.CPRDY == 0 );	//Wait for charge pump to initialize
	
		// Measure
		INTCON0bits.GIEL = 0;	//Disable low priority interrupts
		ADCON0bits.GO = 1;
		asm( "SLEEP" );
		while( 0 == INTCON0bits.GIEL );	//Ensure the interrupt has run!

		//AD->V: V = Ref/4096/2*AD = 2.45/4096/2*AD = 2.45/8192*AD
		//60mV = 49000 / 163840 [AD steps] = 200.62... [AD steps]
		//pH7 = 2048	=> 7 = m * 2048 + b
		//pH4 = 2650	=> 4 = m * 2650 + b
		//m = -3 / 602
		//b = 5179 / 301

		uint16_t wRes = ( xMeasureB - (uint32_t) ADFLTR * xMeasureM ) >> 8;	//8.8 FPN
		uint16_t wMilliV = (uint16_t) ( (uint32_t) ADFLTR * 49000 / 163840 );

		sdout[ 1 ] = wRes >> 8;
		sdout[ 2 ] = wRes & 0xFF;
		sdout[ 4 ] = wMilliV >> 8;
		sdout[ 5 ] = wMilliV & 0xFF;

		DMA1CON0bits.SIRQEN = 1;
		while( DMA1CON0bits.SIRQEN );
		while( 0 == U1FIFObits.TXBE );

		//vECANTransmit( &xMeasureLI );
		
		vTaskDelayUntil( &xLastWakeTime, measureDELAY_TICKS );
	}
}

asm( "GLOBAL _vTaskCalibrate" );
void vTaskCalibrate( void *pvParameters )
{
	//Set up button
	//Timer 2
	T2CON = 0x70;		// T2CKPS 1:128; T2OUTPS 1:1; TMR2ON off
	T2CLKCON = 0x04;	// T2CS LFINTOSC
	T2HLT = 0x12;		// T2PSYNC Not Synchronized; T2MODE Starts on falling edge on TMR2_ers; T2CKPOL Rising Edge; T2CKSYNC Not Synchronized
	T2RST = 0x00;		// T2RSEL T2CKIPPS pin
	T2PR = 0x0B;		// PR2 11
	T2TMR = 0x00;		// Initialise TMR2 to 0
	//T2INPPS = 0x01;		// RA1->TMR2:T2IN

	//CLC 1
	//CLCIN0PPS = 0x01;		// RA1->CLC1:CLCIN0
	CLC1POL = 0x00;			// LC1G1POL not_inverted; LC1G2POL not_inverted; LC1G3POL not_inverted; LC1G4POL not_inverted; LC1POL not_inverted
	CLC1SEL0 = 0x0E;		// LC1D1S TMR2_OUT
	CLC1SEL1 = 0x00;		// LC1D2S CLCIN0 (CLCIN0PPS)
	CLC1SEL2 = 0x00;		// LC1D3S CLCIN0 (CLCIN0PPS)
	CLC1SEL3 = 0x16;		// LC1D4S CCP2_OUT
	CLC1GLS0 = 0x02;		// LC1G1D3N disabled; LC1G1D2N disabled; LC1G1D4N disabled; LC1G1D1T enabled; LC1G1D3T disabled; LC1G1D2T disabled; LC1G1D4T disabled; LC1G1D1N disabled
	CLC1GLS1 = 0x04;		// LC1G2D2N enabled; LC1G2D1N disabled; LC1G2D4N disabled; LC1G2D3N disabled; LC1G2D2T disabled; LC1G2D1T disabled; LC1G2D4T disabled; LC1G2D3T disabled
	CLC1GLS2 = 0x88;		// LC1G3D1N disabled; LC1G3D2N disabled; LC1G3D3N disabled; LC1G3D4N disabled; LC1G3D1T disabled; LC1G3D2T enabled; LC1G3D3T disabled; LC1G3D4T enabled
	CLC1GLS3 = 0x00;		// LC1G4D1N disabled; LC1G4D2N disabled; LC1G4D3N disabled; LC1G4D4N disabled; LC1G4D1T disabled; LC1G4D2T disabled; LC1G4D3T disabled; LC1G4D4T disabled
	CLC1CON = 0x8C;			// LC1EN enabled; INTN enabled; INTP disabled; MODE 1-input D flip-flop with S and R

	//SMT 1
	SMT1CON0 = 0x80;			// WPOL high/rising edge enabled; SMT1STP rolls over to 24'h000000; SMT1SPOL high/rising edge enabled; SMT1EN enabled; SMT1PS 1:1 Prescaler; SMT1CPOL rising edge;
	SMT1CON1 = 0x43;			// SMT1REPEAT Repeat Data Acquisition; SMT1MODE High and Low time; SMT1GO disabled;
	SMT1STAT = 0x00;			// SMT1CPWUP SMT1CPW1 update complete; SMT1CPRUP SMT1PR1 update complete; SMT1RST SMT1TMR1 update complete;
	SMT1CLK = 0x03;				// SMT1CSEL LFINTOSC;
	SMT1WIN = 0x00;				// SMT1WSEL SMT1WINPPS;
	SMT1SIG = 0x16;				// SMT1SSEL CLC1OUT;
	SMT1PRU = 0xFF;				// SMT1PR 0;
	SMT1PRH = 0xFF;				// SMT1PR 0;
	SMT1PRL = 0xFF;				// SMT1PR 0;
	PIR1bits.SMT1PWAIF = 0;		// Clear the SMT1 pulse width acquisition interrupt flag
	//IPR1bits.SMT1PWAIP = 0;	// Set SMT1 pulse width acquisition interrupt to low priority (default)
	PIE1bits.SMT1PWAIE = 1;		// Enable the SMT1 pulse width acquisition interrupt
	SMT1CON1bits.SMT1GO = 1;	// Start the SMT module by writing to SMTxGO bit

	T2CONbits.ON = 1;	//Enable timer 2
	
	//Set up PWM
	//Timer 4
	T4CLKCON = 0x01;	// T4CS FOSC/4; 
	T4HLT = 0x00;		// T4PSYNC Not Synchronized; T4MODE Software control; T4CKPOL Rising Edge; T4CKSYNC Not Synchronized;
	T4RST = 0x00;		// T4RSEL T4CKIPPS pin;
	T4PR = 0xFF;		// PR4 255;
	T4TMR = 0x00;		// TMR4 0;
	T4CON = 0x80;		// T4CKPS 1:1; T4OUTPS 1:1; TMR4ON on;

	//PWM 5
	PWM5CON = 0x90;				// PWM5POL active_lo; PWM5EN enabled;
	PWM5DCH = 0x00;				// DC 127;
	PWM5DCL = 0x00;				// DC 3;
	CCPTMRS1bits.P5TSEL = 2;	// Select timer

	while( 1 )
	{
		vTaskSuspend( NULL );
	}
}

void __interrupt( irq( IRQ_ADT ), base( 8 ), high_priority ) prvADCThresholdISR( void )
{
	INTCON0bits.GIEL = 1;	// Re-enable low priority interrupts
	PIR1bits.ADTIF = 0;		// Clear the ADCC Threshold interrupt flag
}

void __interrupt( irq( IRQ_SMT1PWA ), base( 8 ), high_priority ) prvSwitchISR( void )
{
	PIR1bits.SMT1PWAIF = 0;

	if( SMT1CPW >= 0xFFFF )
	{
		//Longpress
		s_wDutyCycle = 0;
	}
	else
	{
		//Shortpress
		s_wDutyCycle += 32 << 6;
	}
	
	PWM5DCH = ( s_wDutyCycle >> 8 ) & 0xFF;
	PWM5DCL = s_wDutyCycle & 0xFF;
}

void vMeasureInitialize( void )
{
	xHandleMessage = xTaskCreateStatic( vTaskMeasure, (const portCHAR*) "Measure", stackSIZE_MESSAGE, NULL, messagePRIORITY_MESSAGE, xStackMessage, &xBufferMessage );
	xHandleCalibrate = xTaskCreateStatic( vTaskCalibrate, (const portCHAR*) "Calibrate", stackSIZE_CALIBRATE, NULL, messagePRIORITY_CALIBRATE, xStackCalibrate, &xBufferCalibrate );
}

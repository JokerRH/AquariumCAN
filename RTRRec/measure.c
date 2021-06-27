#include "measure.h"
#include "FreeRTOS.h"
#include "stack.h"
#include "priority.h"
#include "exception.h"
#include "peripherals/ecan.h"
#include "peripherals/pin_manager.h"
#include "log.h"
#include <FreeRTOS/include/task.h>
#include <FreeRTOS/include/semphr.h>
#include <pic18f25k83.h>

#define measureEVENT_BUTTON_LONGPRESS	( 1 << 0 )
#define measureEVENT_ADC_READY			( 1 << 1 )
#define measureEVENT_BUTTON_SHORTPRESS	( 1 << 2 )

#define measureLED_BLIP					portMS_TO_TICK( 500 )
#define measureLED_BLINK				portMS_TO_TICK( 1000 )
#define measureBLINK_TOGGLES			240
#define measureCALIBRATE_TIMEOUT		( measureBLINK_TOGGLES * measureLED_BLINK )

#define measureADACQ_DERIV	0x1FFF
#define measureADACQ		0x0006	// (10.2us) => 93°C (10.18us)

TaskHandle_t xHandleMeasure;
static StackType_t xStackMeasure[ stackSIZE_MEASURE ];
static StaticTask_t xBufferMeasure;

TaskHandle_t xHandleCalibrate;
static StackType_t xStackCalibrate[ stackSIZE_CALIBRATE ];
static StaticTask_t xBufferCalibrate;

static ListItem_t xMeasureLI;
static ecan_msg_t xMeasureMsg;

static volatile uint16_t s_wDutyCycle = 0;	//10 bit duty cycle, left aligned (i.e. shifted left 6 bit)
uint16_t xMeasureM = 327;		// = 3 / 602 * 0x10000
uint32_t xMeasureB = 1127611 + 128;	// = 5179 / 301 * 0x10000 [ + '0.5' in 8.8 FPN]

static SemaphoreHandle_t xHandleProbeMutex;
static StaticSemaphore_t xBufferProbeMutex;

asm( "GLOBAL _prvMsgCallback" );
static void prvMsgCallback( void )
{

}

asm( "GLOBAL _vTaskMeasure" );
void vTaskMeasure( void *pvParameters )
{
	//Initialise CAN message
	xMeasureMsg.pvCallback = prvMsgCallback;
	vListInitialiseItem( &xMeasureLI );
	listSET_LIST_ITEM_OWNER( &xMeasureLI, &xMeasureMsg );
	convertCANid2Reg( 0x117, &xMeasureMsg.ucEIDH, &xMeasureMsg.ucEIDL, &xMeasureMsg.ucSIDH, &xMeasureMsg.ucSIDL );

	TickType_t xLastWakeTime = xTaskGetTickCount( );
	
	//Ensure AD interrupt is enabled
	PIE1bits.ADTIE = 1;
	
	while( 1 )
	{
		(void) xSemaphoreTake( xHandleProbeMutex, portMAX_DELAY );

		//Enable ADC charge pump
		ADCPbits.CPON = 1;
		while( ADCPbits.CPRDY == 0 );	//Wait for charge pump to initialize
	
		// Measure
		INTCON0bits.GIEL = 0;	//Disable low priority interrupts
		ADCON0bits.GO = 1;
		asm( "SLEEP" );
		while( 0 == INTCON0bits.GIEL );	//Ensure the interrupt has run!

		ADCPbits.CPON = 0;
		const uint16_t wADFLTR = ADFLTR;
		(void) xSemaphoreGive( xHandleProbeMutex );

		//AD->V: V = Ref/4096/2*AD = 2.45/4096/2*AD = 2.45/8192*AD
		//60mV = 49000 / 163840 [AD steps] = 200.62... [AD steps]
		//pH7 = 2048	=> 7 = m * 2048 + b
		//pH4 = 2650	=> 4 = m * 2650 + b
		//m = -3 / 602
		//b = 5179 / 301

		uint16_t wRes = ( xMeasureB - (uint32_t) wADFLTR * xMeasureM ) >> 8;	//8.8 FPN
		uint16_t wMilliV = (uint16_t) ( (uint32_t) wADFLTR * 49000 / 163840 );

		LOG( 'M', 'm', 't', ' ',
			( wRes >> 8 ) &0xFF, wRes & 0xFF,
			';',
			( wMilliV >> 8 ) &0xFF, wMilliV & 0xFF
		);

		//vECANTransmit( &xMeasureLI );
		
		vTaskDelay( measureDELAY_TICKS );
	}
}

asm( "GLOBAL _vTaskCalibrate" );
void vTaskCalibrate( void *pvParameters )
{
	//Timer 2
	T2CON = 0x70;		// T2CKPS 1:128; T2OUTPS 1:1; TMR2ON off
	T2CLKCON = 0x04;	// T2CS LFINTOSC
	T2HLT = 0x12;		// T2PSYNC Not Synchronized; T2MODE Starts on falling edge on TMR2_ers; T2CKPOL Rising Edge; T2CKSYNC Not Synchronized
	T2RST = 0x00;		// T2RSEL T2CKIPPS pin
	T2PR = 0x0B;		// PR2 11
	T2TMR = 0x00;		// Initialise TMR2 to 0

	//CLC 1
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
	T4CON = 0x00;		// T4CKPS 1:1; T4OUTPS 1:1; TMR4ON off;

	//PWM 5
	PWM5CON = 0x90;				// PWM5POL active_lo; PWM5EN enabled;
	PWM5DCH = 0x00;				// DC 127;
	PWM5DCL = 0x00;				// DC 3;
	CCPTMRS1bits.P5TSEL = 2;	// Select timer
	
	//Ensure AD interrupt is enabled
	PIE1bits.ADTIE = 1;

	uint32_t uxEvent;
	while( 1 )
	{
		//Wait for longpress
#define MASK	( measureEVENT_ADC_READY | measureEVENT_BUTTON_LONGPRESS | measureEVENT_BUTTON_SHORTPRESS )
		do
		{
			(void) xTaskNotifyWait( MASK, MASK, &uxEvent, portMAX_DELAY );
		} while( 0 == ( uxEvent & measureEVENT_BUTTON_LONGPRESS ) );
		(void) xSemaphoreTake( xHandleProbeMutex, portMAX_DELAY );
#undef MASK
		
		LOG( 'C', 'a', 'l', ' ', 's', 't', 'a', 'r', 't' );
		
		//Enable ADC charge pump
		ADCPbits.CPON = 1;
		while( 0 == ADCPbits.CPRDY );	//Wait for charge pump to initialize

		asm( "CALL CALIBRATE_TAKE_MEASUREMENT" );
		const uint16_t wADFLTR1 = ADFLTR;	//Save result

		IO_RA5_SetLow( );
		vTaskDelay( measureLED_BLIP );
		IO_RA5_SetHigh( );

		asm( "CALL CALIBRATE_TAKE_MEASUREMENT" );
		const uint16_t wADFLTR2 = ADFLTR;	//Save result
		
		ADCPbits.CPON = 0;	//Disable ADC charge pump
		
		//Calculate new calibration value
		
		//Wait for user to acknowledge correct probe position
		{
			uint8_t u = measureBLINK_TOGGLES;
			for( ; u != 0; --u )
			{
				IO_RA5_Toggle( );
#define MASK	( measureEVENT_BUTTON_SHORTPRESS | measureEVENT_BUTTON_LONGPRESS )
				while( xTaskNotifyWait( MASK, MASK, &uxEvent, measureLED_BLINK ) )
					if( uxEvent & MASK )
					{
						IO_RA5_SetHigh( );
						if( uxEvent & measureEVENT_BUTTON_LONGPRESS )
							goto CALIBRATE_CANCEL;
						goto CALIBRATE_FINISH;
					}
#undef MASK
			}
			//Timeout
			goto CALIBRATE_TIMEOUT;
		}
		
CALIBRATE_FINISH:
		if( !xSemaphoreGive( xHandleProbeMutex ) )
			goto CALIBRATE_TAKE_MEASUREMENT;
		
		LOG(
				'C', 'a', 'l', ' ',
				( wADFLTR1 >> 8 ) & 0xFF, wADFLTR1 & 0xFF,
				';',
				( wADFLTR2 >> 8 ) & 0xFF, wADFLTR2 & 0xFF
		);

		continue;
		
		//================
		//Take measurement
		//================#
CALIBRATE_TAKE_MEASUREMENT:
		asm( "CALIBRATE_TAKE_MEASUREMENT:" );
		//Enable continuous measurement to indicate stability
		ADACQ = measureADACQ_DERIV;
		RA5PPS = 0x0D;		//Configure RA5 as PWM output (RA5->PWM5:PWM5)
		T4CONbits.ON = 1;	//Enable indicator
		ADCON0bits.GO = 1;

		//Allow the user to wait for the electrode to settle
		//A short button press actually causes the Switch IRQ to disable the indicator.
		//The ADT IRQ will then stop re-enabling measurements and send the measureEVENT_ADC_READY event.
		//This must be used as disabling the GO bit in software requires a delay of unknown length before being reactivated again.
		//This in turn caused a bug that the precise measurement would block indefinitely.
		uint32_t uxEvent;
#define MASK	( measureEVENT_ADC_READY | measureEVENT_BUTTON_LONGPRESS | measureEVENT_BUTTON_SHORTPRESS )
		//Bits are only cleared on entry if no notification is pending!
		while( xTaskNotifyWait( MASK, MASK, &uxEvent, measureCALIBRATE_TIMEOUT ) )
		{
			if( uxEvent & measureEVENT_BUTTON_LONGPRESS )
			{
				//The user cancelled the measurement, the probe is assumed to be reinstalled properly.
				//Pop the return address, as this "TakeMeasurement function" will not return.
				asm( "POP" );
				goto CALIBRATE_CANCEL;
			}
			else if( uxEvent & measureEVENT_ADC_READY )
				goto CALIBRATE_MEASURE;	//The ADC_READY event will always occur after the BUTTON_SHORTPRESS event. Therefore, the latter will be cleared before continuing.
		}
#undef MASK
		//At this point the loop was interrupted due to a timeout.
		asm( "POP" );
		goto CALIBRATE_TIMEOUT;

CALIBRATE_MEASURE:
		RA5PPS = 0x00;	//Reset RA5 to output

		//Do an accurate measurement
		ADACQ = measureADACQ;
		INTCON0bits.GIEL = 0;	//Disable low priority interrupts
		asm( "BSF ADCON0, 0" );	//Enable ADC
		asm( "SLEEP" );
		while( 0 == INTCON0bits.GIEL );		//Ensure the interrupt has run!
		asm( "RETURN" );
		
CALIBRATE_TIMEOUT:
		LOG( 'C', 'a', 'l', ' ', 't', '-', 'o', 'u', 't' );
		SetException( exceptionINVALID_CALIBRATION );
		
CALIBRATE_CANCEL:
		LOG( 'C', 'a', 'l', ' ', 'c', 'a', 'n', 'c', 'l' );
		ADCPbits.CPON = 0;
		RA5PPS = 0x00;	//Reset RA5 to output
		(void) xSemaphoreGive( xHandleProbeMutex );
	}
}

void __interrupt( irq( IRQ_ADT ), base( 8 ), high_priority ) prvADCThresholdISR( void )
{
	INTCON0bits.GIEL = 1;	// Re-enable low priority interrupts
	PIR1bits.ADTIF = 0;		// Clear the ADCC Threshold interrupt flag
	
	//If the PWM timer is enabled, display current AD value
	if( T4CONbits.ON )
	{
		int16_t wTemp = (int16_t) ADERR;
		if( wTemp < 0 )
			wTemp = -wTemp;
		wTemp <<= 6;
		PWM5DCH = ( wTemp >> 8 ) & 0xFF;
		PWM5DCL = wTemp & 0xFF;
		
		ADCON0bits.GO = 1;
	}
	else if( RA5PPS )
	{
		//RA5 is not configured as GPIO, must have been used for PWM.
		//Notify the task that the ADC operation is complete
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		(void) xTaskNotifyFromISR( xHandleCalibrate, measureEVENT_ADC_READY, eSetBits, &xHigherPriorityTaskWoken );	//Never returns anything but pdPASS with eSetBits
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void __interrupt( irq( IRQ_SMT1PWA ), base( 8 ), high_priority ) prvSwitchISR( void )
{
	PIR1bits.SMT1PWAIF = 0;

	uint8_t ucButtonEvent;
	if( SMT1CPW >= 0xFFFF )
	{
		//Longpress
		ucButtonEvent = measureEVENT_BUTTON_LONGPRESS;
	}
	else
	{
		//Shortpress
		//Could separate this out, as EVENT_BUTTON_SHORTPRESS is disregarded when T4 is disabled. This solution saves program space, though.
		ucButtonEvent = measureEVENT_BUTTON_SHORTPRESS;
	}
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	(void) xTaskNotifyFromISR( xHandleCalibrate, ucButtonEvent, eSetBits, &xHigherPriorityTaskWoken );	//Never returns anything but pdPASS with eSetBits
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	
	if( ucButtonEvent & measureEVENT_BUTTON_SHORTPRESS )
		T4CONbits.ON = 0;	//Disable indicator. This will prevent the ADT IRQ from setting the GO bit again, therefore stopping the measurement.
	//The ADT IRQ will notify the task.
}

void vMeasureInitialize( void )
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
	ADACQ = measureADACQ;
	ADCAP = 0x1F;	// ADCAP Additional uC of 31pF
	ADPREL = 0x06;	// ADPRE 6
	ADPREH = 0x00;	// ADPRE 0
	ADCON1 = 0x00;	// ADDSEN disabled; ADGPOL digital_low; ADIPEN disabled; ADPPOL Vss
	ADCON2 = 0x53;	// ADCRS 5; ADMD Burst_average_mode; ADACLR disabled; ADPSIS RES
	ADCON3 = 0x47;	// ADCALC First derivative of filtered value; ADTMD enabled; ADSOI ADGO not cleared
	ADSTAT = 0x00;	// ADMATH registers not updated
	ADREF = 0x02;	// ADNREF VSS; ADPREF external
	ADACT = 0x00;	// ADACT disabled
	ADCLK = 0x00;	// ADCS FOSC/2
	ADCON0 = 0x94;	// ADGO stop; ADFM right; ADON enabled; ADCS Frc; ADCONT disabled
	PIR1bits.ADTIF = 0;	// Clear the ADC Threshold interrupt flag
	IPR1bits.ADTIP = 1;	// Set the ADC Threshold interrupt to high priority
	//Do not enable the interrupt here!

	xHandleProbeMutex = xSemaphoreCreateMutexStatic( &xBufferProbeMutex );
	xHandleMeasure = xTaskCreateStatic( vTaskMeasure, (const portCHAR*) "Measure", stackSIZE_MEASURE, NULL, messagePRIORITY_MEASURE, xStackMeasure, &xBufferMeasure );
	xHandleCalibrate = xTaskCreateStatic( vTaskCalibrate, (const portCHAR*) "Calibrate", stackSIZE_CALIBRATE, NULL, messagePRIORITY_CALIBRATE, xStackCalibrate, &xBufferCalibrate );
}

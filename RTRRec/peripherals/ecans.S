#include <xc.inc>
GLOBAL	prvECANTxISR

PSECT ivt0x8,global,class=CODE,reloc=2,ovrld,optim=
ORG 88
dw prvECANTxISR shr 2	; Vector 44 : TXBnIF

PSECT ecantext1,global,class=CODE,reloc=4

;
; prvECANTxISR
;
GLOBAL	___intlo_sp
GLOBAL	_pxECANCurrentMsg
GLOBAL	_xECANTransmitHandle
GLOBAL	_xTaskGenericNotifyFromISR
SIGNAT	_xTaskGenericNotifyFromISR, 0x6079
prvECANTxISR:
	LFSR	1, ___intlo_sp	; Load low interrupt stack pointer

	; pxHigherPriorityTaskWoken - Push the address of the element currently on top of the stack
	CLRF	INDF1, a	; Set xHigherPriorityTaskWoken to pdFALSE (i.e. *pxHigherPriorityTaskWoken)
	MOVWF	FSR1H, w, a
	MOVFF	FSR1L, PREINC1
	MOVWF	PREINC1, f, a

	; pulPreviousNotificationValue - Set to NULL
	MOVLW	0
	MOVWF	PREINC1, f, a
	MOVWF	PREINC1, f, a

	; eAction - Set to eSetValueWithOverwrite (3)
	MOVLW	3
	MOVWF	PREINC1, f, a

	; Fetch current window value
	;MOVWF	CANSTAT, w, a		; Use this if any of B0-B5 are configured as TX buffer
	BANKSEL( TXB0CON )
	BTFSC	TXB2CON, TXB0CON_TXBIF_POSN, b
	MOVLW	5
	BTFSC	TXB1CON, TXB0CON_TXBIF_POSN, b
	MOVLW	4
	BTFSC	TXB0CON, TXB0CON_TXBIF_POSN, b
	MOVLW	3

	; ulValue - Upper 24 bits are ignored, just write the same value 4 times
	MOVWF	PREINC1, f, a
	MOVWF	PREINC1, f, a
	MOVWF	PREINC1, f, a
	MOVWF	PREINC1, f, a

	; uxIndexToNotify - Set to 0
	MOVLW	0
	MOVWF	PREINC1, f, a

	; xTaskToNotify  - Set to transmit task
	MOVFF	_xECANTransmitHandle, PREINC1
	MOVFF	_xECANTransmitHandle + 1, PREINC1

	; Leave a byte space (SP must point to the next available byte) and call notify
	; Restore the stack after the call
	MOVWF	PREINC1, f, a
	CALL	_xTaskGenericNotifyFromISR

	; Disable TX interrupts
	BANKSEL( PIR5 )
	BCF		PIE5, PIE5_TXBnIE_POSN, b

	; If( *pxHigherPriorityTaskWoken ) portYIELD_FROM_ISR( );
	MOVWF	POSTDEC1, w, a	; Pop buffer element
	TSTFSZ	INDF1, a
	BSF		PIR0, PIR0_SWIF_POSN, b	; Yield

	RETFIE	f


PSECT ecantext2,global,class=CODE,reloc=2

;
; _prvECANTxCallback
;
GLOBAL	_pxECANCurrentMsg
GLOBAL	_prvECANTxCallback
GLOBAL	_xTransmitMutexHandle
GLOBAL	_xQueueGenericSend
_prvECANTxCallback:
	LFSR	0, _pxECANCurrentMsg	; FSR0 = &pxECANCurrentMsg
	MOVF	POSTINC0, w, a
	MOVFF	INDF0, FSR0H
	MOVWF	FSR0L, f, a	; FSR0 = pxECANCurrentMsg
	ADDFSR	0, 13		; FSR0 = &( pxECANCurrentMsg->pvCallback )

	; Push( pxECANCurrentMsg->pvCallback )
	MOVFF	POSTINC0, POSTINC1  //Low
	MOVFF	POSTINC0, POSTINC1  //High

	;
	; Release the mutex
	;
	CLRF	POSTINC1, a	; xCopyPosition = queueSEND_TO_BACK
	CLRF	POSTINC1, a
	CLRF	POSTINC1, a	; xTicksToWait = semGIVE_BLOCK_TIME
	CLRF	POSTINC1, a
	CLRF	POSTINC1, a
	CLRF	POSTINC1, a	; pvItemToQueue = NULL

	; xQueue = xTransmitMutexHandle
	MOVFF	_xTransmitMutexHandle, POSTINC1
	MOVFF	_xTransmitMutexHandle + 1, POSTINC1
	CALL	_xQueueGenericSend

	; Pop( pxECANCurrentMsg->pvCallback )
	MOVWF	POSTDEC1, w, a		; Discard
	MOVFF	POSTDEC1, PCLATH
	MOVWF	INDF1, w, a
	CLRF	PCLATU, a
	CALLW
	RETURN


PSECT ecantext3,global,class=CODE,reloc=2

;
; _prvECANRxCallback
;
GLOBAL	_pxECANCurrentRxLI
GLOBAL	_prvECANRxCallback
_prvECANRxCallback:
	BANKSEL( _pxECANCurrentRxLI )
	LFSR	2, 6
	MOVF	BANKMASK( _pxECANCurrentRxLI ), w, b
	ADDWF	FSR2L, f, a
	MOVF	BANKMASK( _pxECANCurrentRxLI + 1 ), w, b
	ADDWFC	FSR2H, f, a
	MOVF	POSTINC2, w, a
	MOVFF	POSTINC2, PCLATH
	CLRF	PCLATU, a
	CALLW
	RETURN
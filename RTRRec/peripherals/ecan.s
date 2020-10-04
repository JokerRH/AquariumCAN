#include <xc.inc>

GLOBAL	prvECANTxISR

PSECT ivt0x8,global,class=CODE,reloc=2,ovrld,optim=
ORG 88
dw prvECANTxISR shr 2	; Vector 44 : TXBnIF

PSECT ecantext1,global,class=CODE,reloc=4

;
; prvECANTxISR
;
GLOBAL	_pxCurrentMsg
GLOBAL	_xECANMsgHandle
GLOBAL	_xTaskResumeFromISR
;GLOBAL	_xTaskResumeFromISR@xTaskToResume
prvECANTxISR:
	MOVFF	ECANCON, FSR2L		; Create backup of ECANCON
	;MOVWF	CANSTAT, w, a		; Use this if any of B0-B5 are configured as TX buffer
	BANKSEL( TXB0CON )
	BTFSC	TXB2CON, TXB0CON_TXBIF_POSN, b
	MOVLW	5
	BTFSC	TXB1CON, TXB0CON_TXBIF_POSN, b
	MOVLW	4
	BTFSC	TXB0CON, TXB0CON_TXBIF_POSN, b
	MOVLW	3
	MOVWF	ECANCON, f, a	; Set window. MDSEL bits are readonly, FIFOWM is irrelevant.

	MOVFF	_pxCurrentMsg, FSR0L
	MOVFF	_pxCurrentMsg + 1, FSR0H

	MOVFF	POSTINC0, RXB0D7	; D7
	MOVFF	POSTINC0, RXB0D6	; D6
	MOVFF	POSTINC0, RXB0D5	; D5
	MOVFF	POSTINC0, RXB0D4	; D4
	MOVFF	POSTINC0, RXB0D3	; D3
	MOVFF	POSTINC0, RXB0D2	; D2
	MOVFF	POSTINC0, RXB0D1	; D1
	MOVFF	POSTINC0, RXB0D0	; D0
	MOVFF	POSTINC0, RXB0DLC	; DLC
	MOVFF	POSTINC0, RXB0EIDL	; EIDL
	MOVFF	POSTINC0, RXB0EIDH	; EIDH
	MOVFF	POSTINC0, RXB0SIDL	; SIDL
	MOVFF	POSTINC0, RXB0SIDH	; SIDH

	BSF		RXB0CON, TXB0CON_TXREQ_POSN, a	; Send message
	MOVFF	FSR2L, ECANCON					; Restore ECANCON

	BANKSEL( PIR5 )
	BCF		PIR5, PIR5_TXBnIF_POSN, b
	BCF		PIE5, PIE5_TXBnIE_POSN, b

	;MOVFF	_xECANMsgHandle,xTaskResumeFromISR@xTaskToResume
	;MOVFF	_xECANMsgHandle+1,xTaskResumeFromISR@xTaskToResume+1
	;CALL	_xTaskResumeFromISR
	RETFIE	f
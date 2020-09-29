#include <xc.inc>

GLOBAL _prvSetupTimerInterrupt
SIGNAT _prvSetupTimerInterrupt, 89
GLOBAL _vTaskSwitchContext
SIGNAT _vTaskSwitchContext, 89
GLOBAL _xTaskIncrementTick
SIGNAT _xTaskIncrementTick, 89
GLOBAL _pxCurrentTCB

GLOBAL prvPortInitISR
PSECT ivt0xA8,global,class=CODE,reloc=2,ovrld,optim=,abs
ORG 0xA8	
dw prvPortInitISR shr 2   ; Vector 0 : SWINT

PSECT ivt0x8,global,class=CODE,reloc=2,ovrld,optim=
ORG 0	
dw prvPortISR_SWINT shr 2   ; Vector 0 : SWINT
ORG 70	
dw prvPortISR_CCP1 shr 2   ; Vector 35 : CCP1

PSECT mytext1,global,class=CODE,reloc=4
;
; prvPortInitISR
;
GLOBAL prvPortInitISR
prvPortInitISR:
	BANKSEL( PIR0 )
	BCF BANKMASK( PIR0 ), PIR0_SWIF_POSN, b

	; Unlock IVTBASE
	BCF		INTCON0, INTCON0_GIEH_POSN, a	; Disable interrupts
	MOVLW	0x55
	MOVWF	IVTLOCK, f, a
	MOVLW	0xAA
	MOVWF	IVTLOCK, f, a
	BCF		IVTLOCK, IVTLOCK_IVTLOCKED_POSN, a

	;Move IVTBASE to default location (0x8)
	;MOVLW	0
	;MOVWF	IVTBASEU, a
	;MOVLW	0
	;MOVWF	IVTBASEH, a
	MOVLW	8
	MOVWF	IVTBASEL, f, a

	; Lock IVTBASE
	MOVLW	0x55
	MOVWF	IVTLOCK, f, a
	MOVLW	0xAA
	MOVWF	IVTLOCK, f, a
	BSF		IVTLOCK, IVTLOCK_IVTLOCKED_POSN, a
	BSF INTCON0, INTCON0_GIEH_POSN, a   ; Enable Interrupts

	; Initiate context
	GOTO prvPortRestoreContext


PSECT mytext2,local,class=CODE,reloc=4

;
; prvPortISR
;
GLOBAL _ucTest
GLOBAL prvPortISR_SWINT
GLOBAL prvPortISR_CCP1
prvPortISR_SWINT:
	BANKSEL( PIR0 )
	BCF BANKMASK( PIR0 ), PIR0_SWIF_POSN, b	; Clear SWINT interrupt flag

prvPortISR_CCP1:    
	; Set FSR0 to point to pxCurrentTCB->pxTopOfStack
	MOVFF	_pxCurrentTCB, FSR0L
	MOVFF	_pxCurrentTCB + 1, FSR0H
	
	; De-reference FSR0 to set the address it holds into FSR1 (i.e. *( pxCurrentTCB->pxTopOfStack ))
	MOVFF	POSTINC0, FSR1L
	MOVFF	INDF0, FSR1H

	; Save the context
	MOVFF	STATUS_SHAD, PREINC1
	MOVFF	WREG_SHAD, PREINC1
	MOVFF	BSR_SHAD, PREINC1
	MOVFF	PCLATH_SHAD, PREINC1
	MOVFF	PCLATU_SHAD, PREINC1
	MOVFF	FSR0L_SHAD, PREINC1
	MOVFF	FSR0H_SHAD, PREINC1
	MOVFF	FSR1L_SHAD, PREINC1
	MOVFF	FSR1H_SHAD, PREINC1
	MOVFF	FSR2L_SHAD, PREINC1
	MOVFF	FSR2H_SHAD, PREINC1
	MOVFF	PRODL_SHAD, PREINC1
	MOVFF	PRODH_SHAD, PREINC1
	MOVFF	TABLAT, PREINC1
	MOVFF	TBLPTRL, PREINC1
	MOVFF	TBLPTRH, PREINC1
	MOVFF	TBLPTRU, PREINC1
	MOVFF	STATUS_CSHAD, PREINC1
	MOVFF	WREG_CSHAD, PREINC1
	MOVFF	BSR_CSHAD, PREINC1

	; Store the hardware stack pointer in a temp register before we modify it
	MOVFF	STKPTR, FSR2L

	; Store each address from the hardware stack
	; At least a single address has to be saved.
	; If (due to an error) the hardware-stack is empty, POP will cause a STKUNF reset to prevent data corruption.
LOOP_SAVE:
		MOVFF	TOSL, PREINC1
		MOVFF	TOSH, PREINC1
		MOVFF	TOSU, PREINC1
		POP
		TSTFSZ	STKPTR, a	;do ... while( STKPTR != 0 )
		GOTO LOOP_SAVE

	; Store the number of addresses on the hardware stack (from the temporary register)
	MOVFF	FSR2L, PREINC1
	MOVFF	FSR2L, PREINC1

	; Save the new top of the software stack in the TCB
	MOVFF	FSR1H, POSTDEC0
	MOVFF	FSR1L, INDF0

	; Check if task requested a yield (i.e. SWINT: WREG = 0)
	MOVF	WREG, w, a
	BZ	SWITCH

	; Tick interrupt
	BANKSEL( PIR4 )
	BCF BANKMASK( PIR4 ), PIR4_CCP1IF_POSN, b   ; Clear CCP1 interrupt flag
	CALL	_xTaskIncrementTick, 0
	TSTFSZ	WREG, a
SWITCH:
	CALL	_vTaskSwitchContext, 0

;
; prvPortRestoreContext
;
prvPortRestoreContext:
	; Set FSR0 to point to pxCurrentTCB->pxTopOfStack
	MOVFF	_pxCurrentTCB, FSR0L
	MOVFF	_pxCurrentTCB + 1, FSR0H
	
	; De-reference FSR0 to set the address it holds into FSR1 (i.e. *( pxCurrentTCB->pxTopOfStack ))
	MOVFF	POSTINC0, FSR1L
	MOVFF	INDF0, FSR1H
	
	; Copy software to hardware stack
	CLRF	STKPTR, a
	MOVF	POSTDEC1, w, a
	MOVF	POSTDEC1, w, a
	BZ	RETINS		; Software stack is empty (error). RETFIE will cause a STKUNF reset to prevent data corruption.
LOOP_RESTORE:
		;Push PC onto the stack, then override it with the software stack address
		PUSH
		MOVFF	POSTDEC1, TOSU
		MOVFF	POSTDEC1, TOSH
		MOVFF	POSTDEC1, TOSL
		DECFSZ	WREG, a
		GOTO LOOP_RESTORE

	; Restore the context
	MOVFF	POSTDEC1, BSR_CSHAD
	MOVFF	POSTDEC1, WREG_CSHAD
	MOVFF	POSTDEC1, STATUS_CSHAD
	MOVFF	POSTDEC1, TBLPTRU
	MOVFF	POSTDEC1, TBLPTRH
	MOVFF	POSTDEC1, TBLPTRL
	MOVFF	POSTDEC1, TABLAT
	MOVFF	POSTDEC1, PRODH_SHAD
	MOVFF	POSTDEC1, PRODL_SHAD
	MOVFF	POSTDEC1, FSR2H_SHAD
	MOVFF	POSTDEC1, FSR2L_SHAD
	MOVFF	POSTDEC1, FSR1H_SHAD
	MOVFF	POSTDEC1, FSR1L_SHAD
	MOVFF	POSTDEC1, FSR0H_SHAD
	MOVFF	POSTDEC1, FSR0L_SHAD
	MOVFF	POSTDEC1, PCLATU_SHAD
	MOVFF	POSTDEC1, PCLATH_SHAD
	MOVFF	POSTDEC1, BSR_SHAD
	MOVFF	POSTDEC1, WREG_SHAD
	MOVFF	POSTDEC1, STATUS_SHAD
	
	; Save the new top of the software stack in the TCB
	MOVFF	FSR1H, POSTDEC0
	MOVFF	FSR1L, INDF0
RETINS:
	; Return swapping the shadow registers
	RETFIE f

PSECT mytext3,local,class=CODE,reloc=2

;
; _xPortStartScheduler
;
GLOBAL _xPortStartScheduler
SIGNAT _xPortStartScheduler, 89
_xPortStartScheduler:
	CALL _prvSetupTimerInterrupt
	BANKSEL( PIR0 )
	BSF BANKMASK( PIR0 ), PIR0_SWIF_POSN, b
 	BSF INTCON0, INTCON0_GIEH_POSN, a
	GOTO $

#include <xc.inc>
#include <FreeRTOSConfig.h>

GLOBAL _prvSetupTimerInterrupt
SIGNAT _prvSetupTimerInterrupt, 0x59
GLOBAL _vTaskSwitchContext
SIGNAT _vTaskSwitchContext, 0x59
GLOBAL _xTaskIncrementTick
SIGNAT _xTaskIncrementTick, 0x59
GLOBAL _pxCurrentTCB
GLOBAL _ucCriticalNesting

GLOBAL prvPortInitISR
PSECT ivt0xA8,global,class=CODE,reloc=2,ovrld,optim=,abs
ORG 0xA8	
dw prvPortInitISR shr 2   ; Vector 0 : SWINT

PSECT ivt0x8,global,class=CODE,reloc=2,ovrld,optim=
ORG 0	
dw prvPortISR_SWINT shr 2   ; Vector 0 : SWINT
ORG 70	
dw prvPortISR_CCP1 shr 2   ; Vector 35 : CCP1

PSECT porttext1,global,class=CODE,reloc=4
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


PSECT porttext2,local,class=CODE,reloc=4

;
; prvPortISR
;
GLOBAL btemp
GLOBAL	___intlo_sp
GLOBAL prvPortISR_CCP1
GLOBAL prvPortISR_SWINT
prvPortISR_SWINT:
	BANKSEL( PIR0 )
	BCF BANKMASK( PIR0 ), PIR0_SWIF_POSN, b	; Clear SWINT interrupt flag
	GOTO SAVE_CTX


PSECT porttext3,local,class=CODE,reloc=4

prvPortISR_CCP1:
	MOVFF	INTCON0, PREINC1
SAVE_CTX:
	; Save the context
	MOVFF	STATUS_SHAD, PREINC1
	MOVFF	WREG_SHAD, PREINC1
	MOVFF	BSR_SHAD, PREINC1
	MOVFF	PCLATH_SHAD, PREINC1
	MOVFF	PCLATU_SHAD, PREINC1
	MOVFF	FSR0L, PREINC1
	MOVFF	FSR0H, PREINC1
	MOVFF	FSR2L, PREINC1
	MOVFF	FSR2H, PREINC1
	MOVFF	PRODL_SHAD, PREINC1
	MOVFF	PRODH_SHAD, PREINC1
	MOVFF	TABLAT, PREINC1
	MOVFF	TBLPTRL, PREINC1
	MOVFF	TBLPTRH, PREINC1
	MOVFF	TBLPTRU, PREINC1
	MOVFF	STATUS_CSHAD, PREINC1
	MOVFF	WREG_CSHAD, PREINC1
	MOVFF	BSR_CSHAD, PREINC1

	; Save temp registers
	LFSR	2, btemp
	REPT	configTEMP_SIZE
		MOVFF	POSTINC2, PREINC1
	ENDM

	; Save critical nesting
	MOVFF	_ucCriticalNesting, PREINC1

	; Store the hardware stack pointer in a temp register before we modify it
	MOVFF	STKPTR, FSR2L

	; Store each address from the hardware stack
	; At least a single address has to be saved.
	; If (due to an error) the hardware-stack is empty, POP will cause a STKUNF reset to prevent data corruption.
LOOP_SAVE:
		MOVFF	TOSL, PREINC1
		MOVFF	TOSH, PREINC1
		;MOVFF	TOSU, PREINC1
		POP
		TSTFSZ	STKPTR, a	;do ... while( STKPTR != 0 )
		GOTO LOOP_SAVE

	; Store the number of addresses on the hardware stack (from the temporary register)
	MOVFF	FSR2L, PREINC1
	MOVFF	FSR2L, PREINC1

	; Set FSR0 to point to pxCurrentTCB->pxTopOfStack
	MOVFF	_pxCurrentTCB, FSR0L
	MOVFF	_pxCurrentTCB + 1, FSR0H

	; Save the new top of the software stack in the TCB
	MOVFF	FSR1L, POSTINC0
	MOVFF	FSR1H, INDF0

	; Load low interrupt stack pointer
	LFSR	1, ___intlo_sp

	; Check if the task requested a yield (i.e. SWINT: WREG = 0)
	MOVF	WREG, w, a
	BZ	SWITCH

	; Tick interrupt
	BANKSEL( PIR4 )
	BCF BANKMASK( PIR4 ), PIR4_CCP1IF_POSN, b   ; Clear CCP1 interrupt flag
	CALL	_xTaskIncrementTick, 0
	TSTFSZ	btemp, a
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
		;MOVFF	POSTDEC1, TOSU
		MOVFF	POSTDEC1, TOSH
		MOVFF	POSTDEC1, TOSL
		DECFSZ	WREG, a
		GOTO LOOP_RESTORE

	; Restore critical nesting
	MOVFF	POSTDEC1, _ucCriticalNesting

	; Restore temp registers
	LFSR	2, btemp + configTEMP_SIZE - 1
	REPT	configTEMP_SIZE
		MOVFF	POSTDEC1, POSTDEC2
	ENDM

	; Restore the context
	MOVFF	POSTDEC1, BSR_CSHAD
	MOVFF	POSTDEC1, WREG_CSHAD
	MOVFF	POSTDEC1, STATUS_CSHAD
	MOVFF	POSTDEC1, TBLPTRU
	MOVFF	POSTDEC1, TBLPTRH
	MOVFF	POSTDEC1, TBLPTRL
	MOVFF	POSTDEC1, TABLAT
	MOVFF	POSTDEC1, PRODH
	MOVFF	POSTDEC1, PRODL
	MOVFF	POSTDEC1, FSR2H
	MOVFF	POSTDEC1, FSR2L
	MOVFF	POSTDEC1, FSR0H
	MOVFF	POSTDEC1, FSR0L
	MOVFF	POSTDEC1, PCLATU
	MOVFF	POSTDEC1, PCLATH
	MOVFF	POSTDEC1, BSR
	MOVFF	POSTDEC1, WREG
	MOVFF	POSTDEC1, STATUS
	
	; Restore INTCON0
	BTFSS	POSTDEC1, INTCON0_GIEH_POSN, a
	BCF	INTCON0, INTCON0_GIEH_POSN, a	; GIEH must have been high for the interrupt to get triggered. Restore saved value.
RETINS:
	; Return swapping the shadow registers
	RETFIE    ; FSR shadow registers are broken!

PSECT porttext4,local,class=CODE,reloc=2

;
; _xPortStartScheduler
;
GLOBAL _xPortStartScheduler
SIGNAT _xPortStartScheduler, 0x59
_xPortStartScheduler:
	CALL _prvSetupTimerInterrupt
	BANKSEL( PIR0 )
	BSF BANKMASK( PIR0 ), PIR0_SWIF_POSN, b
 	BSF INTCON0, INTCON0_GIEH_POSN, a
	GOTO $


PSECT porttext5,local,class=CODE,reloc=2

;
; __reentrant StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
; This function is written such that the current software stack may be located within the region pointed to by pxTopOfStack
;
GLOBAL _pxPortInitialiseStack
SIGNAT _pxPortInitialiseStack, 0x307A
_pxPortInitialiseStack:
	; low( pvParameters )
	; high( pvParameters )
	; low( pxCode )
	; high( pxCode )
	; low( pxTopOfStack )
	; high( pxTopOfStack )
	; <- FSR1
	MOVFF	POSTDEC1, FSR2H
	MOVFF	POSTDEC1, FSR2H	; high( pxTopOfStack )
	MOVFF	POSTDEC1, FSR2L	; low( pxTopOfStack )
	
	MOVFF	POSTDEC1, btemp + 1	; high( pxCode )
	MOVFF	POSTDEC1, btemp		; low( pxCode )

	; Push task function argument
	MOVLW	-1
	MOVFF	PLUSW1, POSTINC2	; low( pvParameters )
	MOVFF	POSTDEC1, POSTINC2	; high( pvParameters ); FSR1 now points to pvParameters (i.e. stack has been restored)
	MOVWF	POSTINC2, f, a		; space (SP must point to the next available byte for xc8 software stacks)
	;MOVWF	POSTDEC1, w, a

	; Task context
	MOVLW	INTCON0_GIEH_MASK | INTCON0_GIEL_MASK | INTCON0_IPEN_MASK
	MOVWF	POSTINC2, f, a	; INTCON0
	CLRF	WREG, a
	MOVWF	POSTINC2, f, a	; STATUS
	MOVWF	POSTINC2, f, a	; WREG
	MOVWF	POSTINC2, f, a	; BSR
	MOVWF	POSTINC2, f, a	; PCLATH
	MOVWF	POSTINC2, f, a	; PCLATU
	MOVWF	POSTINC2, f, a	; FSR0L
	MOVWF	POSTINC2, f, a	; FSR0H
	MOVWF	POSTINC2, f, a	; FSR2L
	MOVWF	POSTINC2, f, a	; FSR2H
	MOVWF	POSTINC2, f, a	; PRODL
	MOVWF	POSTINC2, f, a	; PRODH
	MOVWF	POSTINC2, f, a	; TABLAT
	MOVWF	POSTINC2, f, a	; TBLPTRUL (apparently must be initialized to 0!)
	MOVWF	POSTINC2, f, a	; TBLPTRUH
	MOVWF	POSTINC2, f, a	; TBLPTRU
	MOVWF	POSTINC2, f, a	; STATUS_CSHAD
	MOVWF	POSTINC2, f, a	; WREG_CSHAD
	MOVWF	POSTINC2, f, a	; BSR_CSHAD
	
	; Initialize btemp
	REPT	configTEMP_SIZE
		MOVWF	POSTINC2, f, a
	ENDM

	; Initialize critical nesting
	; WREG = 0
	MOVWF	POSTINC2, f, a

	; Start address
	MOVFF	btemp, POSTINC2		; TOSL
	MOVFF	btemp + 1, POSTINC2	; TOSH
	; TOSU is always null on PIC18F2xK83 chips

	MOVLW	1
	MOVWF	POSTINC2, f, a	; Number of addresses (only start address)

	; Return the current top of stack
	MOVFF	FSR2L, btemp		; low( pxTopOfStack )
	MOVFF	FSR2H, btemp + 1	; high( pxTopOfStack )
	RETURN
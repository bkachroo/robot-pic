	list P=PIC16F877, F=INHX8M, C=160, N=80, ST=OFF, MM=OFF, R=DEC
	include "P16F877.INC"
	__config ( _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_OFF & _CPD_OFF )
	errorlevel -302		;ignore error when storing to bank1

	; Variables
	TIMECNT	equ	0x20	; variable for counter
	MINDIST	equ	0x0F	; variable for distance, currently 0.3 V
	
	org 0x0000
 	goto	Init
	org 0x0004
	movwf 	temp_w
	movf 	STATUS,w
	movwf  	temp_status
	
	;btfsc 	INTCON,INTF
	;call	ISR_Lit	
	;bcf 	INTCON,INTF
	
	movf 	temp_status
	movwf 	STATUS
	swapf	temp_w,f
	swapf	temp_w,w
	retfie


	cblock 0x21
	temp_w
	temp_status

	endc
	
Init	bsf	STATUS, RP0	; bank 1
	movlw	B'00001110'	; config ADCON1 to use RA0
	movwf	ADCON1
	clrf	TRISE		; config PORTE output
	movlw	B'00000011'	; config PORTD sensors
	movwf	TRISD
	movlw  	B'11111111'
	movwf  	TRISB   	;Set PORTB as input
	bsf	INTCON,GIE	;allows global interrupt
	bsf	INTCON,INTE	;enable RB0 interrupt
	bcf	STATUS, RP0	; bank 0
	
; Main Program
ADStart	call	AD_Conv		; call AD subroutine
	btfss	PORTA, RA1	; if RA1 is set clear B
	goto	Nexter
	movlw	B'00000000'
	movwf	PORTB
Nexter	goto	ADStart		; repeat
	;movwf	PORTB		; display 8-bit result on B
	
	movlw	MINDIST		; move constant and compare to ADRESH
	subwf	ADRESH, W
	btfsc	STATUS,Z
	goto	ADStart		; if equal do it again
	btfss	STATUS,C
	goto	Small_Enough
	bcf	PORTE, 0	; empty PORTE if ADRESH is larger
	goto	ADStart		; repeat process
Small_Enough
	bsf	PORTE, 0	; flash RE0 is ADRESH is smaller
	goto ADStart		; repeat process
	
EndLP	goto	EndLP		; inf loop, just in case
	
; AD Conversion Routine
AD_Conv	movlw	B'10000001'	; config ADCON0
	movwf	ADCON0
	call	Tim20		; wait acquisition time
	bsf	ADCON0, GO	; start conversion
	
Wait	btfsc	ADCON0, GO	; wait for conversion
	goto	Wait		; poll DONE bit
	;movf	ADRESH, W	; move 8-bit high to w
	return
	
; Time Delay
Tim20	movlw	0x084
	movwf	TIMECNT

TimeLP	decfsz	TIMECNT, F	; wait the 400 cycles = 20 us
	goto	TimeLP
	nop
	return
	
	end
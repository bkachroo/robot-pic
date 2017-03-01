 
	list P=PIC16F877, F=INHX8M, C=160, N=80, ST=OFF, MM=OFF, R=DEC
	include "P16F877.INC"
	__config ( _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_OFF & _CPD_OFF )
	errorlevel -302		;ignore error when storing to bank1

	
	org 0x0000
 	goto	Mainline

	org 0x0004 
	; normally save context here, but don't need to
	goto Int_Handler
	
Mainline
	; initialize stuff
	bsf	STATUS, RP0 ; bank 1
	bsf	TRISB, 0    ; Set RB0 as input
	bsf	INTCON, GIE ; enable global interrupts
	bsf	INTCON, INTE	; enable RB0 interrupt
	; since RA1 is a AD input, we need to switch to digital
	;movlw	0x07	    ; setting for all digital
	;movwf	ADCON1	    ; change settings, also bank 1
	bcf	TRISA, 1    ; Set RA1 as output
	bcf	STATUS, RP0 ; bank 0
	; now loop
Mainloop
	goto Mainloop	    ; wait for interrupt
	
Int_Handler
	btfss	PORTB, 0    ; if RB0 is high
	goto	If_Low
If_High	bsf	PORTA, 1    ; set RA1 high
	goto	Done
If_Low	bcf	PORTA, 1    ; else set RA1 low
Done	retfie		    ; return from interrupt
	END
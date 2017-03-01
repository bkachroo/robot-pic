 
	list P=PIC16F877, F=INHX8M, C=160, N=80, ST=OFF, MM=OFF, R=DEC
	include "P16F877.INC"
	__config ( _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_OFF & _CPD_OFF )
	errorlevel -302		;ignore error when storing to bank1

	
	org 0x0000
 	goto	Mainline

	org 0x0004 
	; interrupts, don't know what to put
	
Mainline
	; initialize stuff
	bsf	STATUS, RP0 ; bank 1
	bsf	TRISA, 0    ; Set RA0 as input
	; since RA0 is a AD input, we need to switch to digital
	movlw	0x07	    ; setting for all digital
	movwf	ADCON1	    ; change settings, also bank 1
	bcf	TRISA, 1    ; Set RA1 as output
	bcf	STATUS, RP0 ; bank 0
	; now loop
Mainloop
	btfsc	PORTA, 0    ; if RA0 is low
	goto	If_High
If_Low	bcf	PORTA, 1    ; clear output RA1
	goto	Done
If_High	bsf	PORTA, 1    ; else set output RA1
Done	goto	Mainloop    ; just keep looping
	
	END
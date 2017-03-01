list P=PIC16F877, F=INHX8M, C=160, N=80, ST=OFF, MM=OFF, R=DEC
	include "P16F877.INC"
	__config ( _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_OFF & _CPD_OFF )
	errorlevel -302		;ignore error when storing to bank1
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Variables
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; first define sensors
#define	HEIGHT	PORTA, 0    ; barrel height input, 0 = low, 1 = high
#define	STRIP0	PORTA, 1    ; strip height inputs, 00 = empty, 01 = middle,
#define	STRIP1	PORTA, 2    ; 1x = full
#define	BARREL	PORTB, 0    ; Barrel proximity sensor, interrupts on rising edge
#define	COL	PORTB, 4    ; Column proximity sensor, interrupts
	; define motor outputs
#define LEFTE	PORTC, 0    ; Left base motor enable bit
#define LEFTD	PORTC, 1    ; Left base motor direction bit (forward/reverse)
#define RIGHTE	PORTC, 2    ; Right base motor enable bit
#define	RIGHTD	PORTC, 3    ; Right base motor direction bit
#define	ARME	PORTC, 4    ; Arm motor enable bit
#define ARMD	PORTC, 5    ; Arm motor direction bit
	; constants for timing
ARMTIME	equ	D'23'	    ; Arm timer constant: 0.0655*23 = 1.51 sec
T1PADL	equ	0x6A	    ; Padding constant so TMR1 has 0.5 sec period, low
T1PADH	equ	0x67	    ; high byte
	;variables for timing
T0CNT	equ	0x20	    ; location of TMR0 high-level count register
T1CNTL	equ	0x21	    ; location of TRM1 high-level count register, low
T1CNTH	equ	0x22	    ; location of high byte
	; variables for interrupts
TEMP_W	    equ	0x23
TEMP_STATUS equ	0x24
	; variables for barrel data
SPEED	equ	D'3'	    ; speed of motion in cm/sec
TEMP1	equ	0x25
TEMP2	equ	0x26
NUM_B	equ	0x27	    ; SET THIS!!
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Vectors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	org 0x0000
 	goto	Init	    
	org 0x0004
	movwf 	TEMP_W		; save context
	movf 	STATUS,W
	movwf  	TEMP_STATUS
	
	call	ISR	
	
	movf 	TEMP_STATUS	; restore context
	movwf 	STATUS
	movf	TEMP_W
	retfie


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
	; INITIALIZATION
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; note, interrupts can't be initialized yet because they would
	; interefere with standby polling (RB4=COL is also keypad)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Initialize Ports
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Init	bsf	STATUS, RP0	; go to bank 1
	; Initialize inputs
	movlw	B'11111111'	; set PORTB to input (keypad)
	movwf	TRISB
	movlw	B'11111111'	; set PORTA to input (sensors)
	movwf	TRISA
	movlw	B'00000111'	; set PORTA to digital input
	movwf	ADCON1
	; Initialize outputs
	movlw	B'00000000'	; set PORTC to output (motors)
	movwf	TRISC
	movlw	B'00000000'
	movwf	TRISD
	bcf	STATUS, RP0	; go to bank 0
	clrf	PORTC
	clrf	PORTD

	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Initialize Interrupts/Timers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
In_Int	bsf	STATUS, RP0	; go to bank 1
	movlw	B'01000111'	; interrupt only on rising edge of BARREL and
	movwf	OPTION_REG	; set TMR0 to overflow period = 0.0655 sec
	bcf	STATUS, RP0
	movlw	B'10011000'	; enable only BARREL and COL interrupts, none
	movwf	INTCON		; from timers
	; note: even though timer interrupts are not on, their flags are still
	; set on overflow
	movlw	T1PADL		; put initial pad in TMR1 so it has the exact
	movwf	TMR1L		; right period
	movlw	T1PADH
	movwf	TMR1H
	movlw	B'00110001'	; set TMR1 to overflow period = 0.5 sec
	movwf	T1CON		; and start TMR1
	clrf	T1CNTL
	clrf	T1CNTH
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
	; Main Loop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; note: This works by waiting for interrupts while polling the
	; timer overflow flag, PIR1, TMR1IF, and restarting the timer with
	; the constant padding
Main	btfss	PIR1, TMR1IF	; poll TMR1 overflow flag
	goto	Main		; if not loop
Reset_Timer
	; note: adding the padding instead of just moving it is important
	; because the barrel interrupt might allow the timer to overflow 
	; and increment to an appreciable value, so wiping would underestimate
	; time elapsed. All assuming the timer continue to counts after 
	; overflow. As long as the time elapsed during the barrel subroutine
	; is less than the timer period minus padding, even an interrupt during
	; this reset routine won't mess up the timing.
	bcf	PIR1, TMR1IF	; clear the interrupt flag
	incf	T1CNTL, F	; increment high-level count low byte
	btfsc	STATUS, Z	; if it overflows
	incf	T1CNTH, F	; increment the high byte
	movlw	T1PADL		; get padding low byte
        addwf	TMR1L		; add to Timer1 low byte
	btfsc   STATUS,C        ; Check for carry
        incf	TMR1H, F	; if so add to Timer1 high byte
	movlw	T1PADH		; get high byte
	addwf	TMR1H		; add to Timer1 high byte
        goto	Main		; back to waiting

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Barrel Subroutine
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	
Barrel_Sub  ; basically take a photo through the sensors and deposit the data
	bcf	INTCON, INTF	; clear the external interrupt flag bit
	bcf	STATUS, IRP	; making sure indirect addressing is bank 0
	incf	NUM_B, F	; increment the number of barrels
	movlw	NUM_B		; put the literal initial address in FSR
	movwf	FSR
	rlf	NUM_B, W	; add the number of barrels*2 to FSR
	addwf	FSR, F
; Compute Distance ;;;;;;;;;;;;;;;;;;;;;;
	movf	T1CNTH, W	; all to divide by 8 and fit into one byte
	movwf	TEMP1
	movf	T1CNTL, W
	movwf	TEMP2
	bcf	STATUS, C
	rrf	TEMP1, F
	rrf	TEMP2, F
	rrf	TEMP1, F
	rrf	TEMP2, F
	rrf	TEMP1, F
	rrf	TEMP2, W	; ends with time in secs in W
	; Multiplication Routine 
	movwf	INDF		; put time into low byte
	movlw	SPEED		; get cm/sec into W
	incf	FSR, F
	clrf	INDF

	clrf	TEMP1
	bsf	TEMP1,3
	decf	FSR, F
	rrf	INDF,F
Mult_Loop
	incf	FSR, F
	btfsc	STATUS, C
	addwf	INDF,F
	
	rrf	INDF,F
	decf	FSR, F
	rrf	INDF,F

	decfsz TEMP1
	goto	Mult_Loop
	incf	FSR, F	    ; exit at high byte
; Read and Save Sensors ;;;;;;;;;;;;;;;;;
	movf	PORTA, W    ; get sensors
	andlw	0x0F	    ; avoid any weird connections
	swapf	W	    ; put the results in the top 4 bits of the high
	iorwf	INDF, F	    ; register for this barrel
	return		    ; done barrel subroutine
	
Column_Sub
ISR	btfsc	INTCON, INTF	; if interrupt came from barrel detection
	call	Barrel_Sub	; call the barrel subroutine
	btfsc	INTCON, RBIF	; if interrupt came from column
	call	Column_Sub	; call the column subroutine
	return			; there shouldn't be another reason
	
	end

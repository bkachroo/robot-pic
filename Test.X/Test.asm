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
#define	BARREL	PORTA, 3    ; Barrel proximity sensor, interrupts on rising edge
#define	COL	PORTA, 5    ; Column proximity sensor, interrupts
	; define motor outputs
#define LEFTE	PORTC, 0    ; Left base motor enable bit
#define LEFTD	PORTC, 1    ; Left base motor direction bit (forward/reverse)
#define RIGHTE	PORTC, 2    ; Right base motor enable bit
#define	RIGHTD	PORTC, 3    ; Right base motor direction bit
#define	ARME	PORTC, 4    ; Arm motor enable bit
#define ARMD	PORTC, 6    ; Arm motor direction bit (RC5 is weird)
	; define LCD outputs
#define RS	PORTD, 2
#define	E	PORTD, 3
	; constants for timing
ARMTIME	equ	D'87'	    ; Arm timer in constant: 0.0262*73 = 1.9 sec
ARMTIME2 equ	D'90'	    ; Out-constant
ARMTIME3 equ	D'6'	    ; Just the real arm time*3 for the optime
COLTIME	equ	D'26'	    ; Column pass time constant: 0.125*32=4sec (T1)
			    ; the rest are all T0
TURNTIME equ	D'0'	    ; First turn constant for exit 0.0262*0= 0 sec
TURNTIME2 equ	D'0'	    ; real turn time*2 for optime use
REV1TIME equ	D'0'	    ; 
REV2TIME equ	D'1'	    ; Constant for exit reverse = 0.0262*256*8 = 54 sec
REV2TIME2 equ	D'1'
REVTIMES equ	D'0'	    ; real sum of rev times for optime
T1PADL	equ	0x6A	    ; Padding constant so TMR1 has 0.5 sec period, low
T1PADH	equ	0x67	    ; high byte
	;variables for timing
T0CNT	equ	0x20	    ; location of TMR0 high-level count register
T1CNTL	equ	0x21	    ; location of TRM1 high-level count register, low
T1CNTH	equ	0x22	    ; location of high byte
	; variables for interrupts
TEMP_W	    equ	0x23
TEMP_STATUS equ	0x24
	; variables for LCD
COM	equ	0x25	    ; buffer for instruction
DAT	equ	0x26	    ; buffer for data
d1	equ	0x27
d2	equ	0x28
COUNTER	equ	0x29
	; variables for barrel data
SPEED	equ	D'6'	    ; speed of motion in cm/sec
MAXL	equ	B'10000000'	    ; max time = run time (60) in sec * 8
MAXH	equ	B'00000000'
DIST_PAD equ	D'10'	    ; difference between barrel sensor and start line
			    ; = 20 cm - 10 cm the barrel radius
TEMP1	equ	0x30	    ; a useful second working register
TEMP2	equ	0x31
TEMP3	equ	0x32
TEMP4	equ	0x33
TEMP5	equ	0x34
FLAGGER	equ	0x35	    ; flag bit register
#define	FLAG_B	FLAGGER, 0
#define	COL_EN	FLAGGER, 1
COUNTB	equ	0x36
NUM_TALL equ	0x37	    ; number of tall barrels
NUM_B	equ	0x38	    ; SET THIS!!
	
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
	
	movf 	TEMP_STATUS, W	; restore context
	movwf 	STATUS
	movf	TEMP_W, W
	retfie
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; TABLES ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
FirstMessage
	addwf	PCL, F
	dt	"Press 1 to start.", 0
EndMessage
	addwf	PCL, F
	dt	"Run Complete.", 0
Optime
	addwf	PCL, F
	dt	"Time: ", 0

Seconds
	addwf	PCL, F
	dt	"secs", 0

Bar_Num
	addwf	PCL, F
	dt	"Barrels: ", 0

Tall_B_Num
	addwf	PCL, F
	dt	"Tall: ", 0

Short_B_Num
	addwf	PCL, F
	dt	"Short: ", 0

Distance
	addwf	PCL, F
	dt	"", 0

Centi
	addwf	PCL, F
	dt	"cm: ", 0

Tall
	addwf	PCL, F
	dt	"T, ", 0

Short	
	addwf	PCL, F
	dt	"S, ", 0

Full
	addwf	PCL, F
	dt	"Full", 0

Middle	
	addwf	PCL, F
	dt	"Half", 0

Empty	
	addwf	PCL, F
	dt	"Empty", 0
All_Done
	addwf	PCL, F
	dt	"All done.", 0

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; MACROS ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Display	macro	Message		; this macro works through a table to LCD
	local	loop_
	local	end_
	clrf	COUNTER		; as a counter
	clrw
loop_	movf	COUNTER, W
	call	Message
	xorlw	B'00000000'
	btfsc	STATUS, Z
	goto	end_
	call	WR_DATA
	incf	COUNTER
	goto	loop_
end_	
	endm

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
	movlw	B'11111111'	; set PORTA to input (sensors)
	movwf	TRISA
	movlw	B'11111111'	; set PORTB to input (keypad)
	movwf	TRISB
	movlw	B'00000111'	; set PORTA to digital input
	movwf	ADCON1
	; Initialize outputs
	movlw	B'00000000'	; set PORTC to output (motors)
	movwf	TRISC
	movlw	B'00000000'	; set PORTD to output (LCD)
	movwf	TRISD
	bcf	STATUS, RP0	; go to bank 0
	bcf	INTCON, GIE	
	clrf	PORTC		; clear to prevent motors starting by accident
	clrf	NUM_TALL
	clrf	NUM_B
	clrf	T1CNTL
	clrf	T1CNTH
	; Send Initial LCD Message
	movlw	B'00110011'
	call	WR_INS
	movlw	B'00110010'
	call	WR_INS
	movlw	B'00101000'
	call	WR_INS
	movlw	B'00001100'
	call	WR_INS
	movlw	B'00000110'
	call	WR_INS
	movlw	B'00000001'
	call	WR_INS
	Display	FirstMessage
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Standby
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	call Wait_K
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Initialize Interrupts/Timers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
In_Int	
	bsf	STATUS, RP0	; go to bank 1
	movlw	B'00000111'	; interrupt only on falling edge of BARREL and
	movwf	OPTION_REG	; set TMR0 to overflow period = 0.0262 sec
	bcf	STATUS, RP0	; 
	movlw	B'00000000'	; disable interrupts
	movwf	INTCON		; from timers
	; note: even though timer interrupts are not on, their flags are still
	; set on overflow
	movlw	T1PADL		; put initial pad in TMR1 so it has the exact
	movwf	TMR1L		; right period
	movlw	T1PADH
	movwf	TMR1H
	movlw	B'00110001'	; set TMR1 to overflow period = 0.125 sec
	movwf	T1CON		; and start TMR1
	bsf	COL_EN		; enable column detection

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Start Motors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Start_M	movlw	B'00000101'	; mask for PORTC, enable base motors and move
	movwf	PORTC		; forward
	movf	NUM_B, W
	movwf	PORTD
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; MAIN ROUTINES ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
	; Main Loop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; note: This works by waiting for interrupts while polling the
	; timer overflow flag, PIR1, TMR1IF, and restarting the timer with
	; the constant padding
Main	btfss	PIR1, TMR1IF	; poll TMR1 overflow flag
	goto	Main		; if not overflowed repeat
	call	Overflow_Timer1	; reset timer
	btfss	BARREL		; check if barrel is detected
	goto	Barrel_Check
	bcf	FLAG_B		; if not remove any flags
	goto	Column_Check	; and check for column
Barrel_Check
	btfss	FLAG_B		; check if currently on barrel
	call	Barrel_Sub	; if there is a barrel get information
	goto	Main
Column_Check
	btfss	COL_EN		; check if a column is detected
	goto	Main
	btfss	COL
	call	Column_Sub	; if so goto column routine
	goto	Main		; if not repeat
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Exit Routine
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Disable all interrupts (they might misfire)
Exit_R	
	; stop, retract arm
	bcf	T1CON, TMR1ON	; stop timer1
	call	Reset_Timer0	; reset timer0 before using
	movlw	TURNTIME		; set timer constant
	movwf	T0CNT
E4Loop
	call	Poll_Timer0	; get it running, delays 0.0262s
	decfsz	T0CNT
	goto	E4Loop		; loops until constant finishes
	
	clrf	PORTC		; stop motors
	bsf	ARMD
	bsf	ARME		; start arm
	call	Reset_Timer0	; reset timer0 before using
	movlw	ARMTIME		; set timer constant
	movwf	T0CNT
E1Loop
	call	Poll_Timer0	; get it running, delays 0.0262s
	decfsz	T0CNT
	goto	E1Loop		; loops until constant finishes
	
	movlw	B'00001111'	; run in reverse until the end
	movwf	PORTC
	call	Reset_Timer0	; reset timer0 before using
	movlw	REV2TIME2	; set high-level timer constant
	movwf	TEMP2
E2Loop
	movlw	REV2TIME	; set timer constant
	movwf	T0CNT
E3Loop
	call	Poll_Timer0	; get it running, delays 0.0262s
	decfsz	T0CNT
	goto	E3Loop		; loops until constant finishes
	decfsz	TEMP2
	goto	E2Loop		; double loop
	clrf	PORTC		; stop moving
	; and just continue to the Display segment
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Display Loop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Initialize the LCD ;;;;;;;;;;;;;;;;;;
	movlw	B'00110011'
	call	WR_INS
	movlw	B'00110010'
	call	WR_INS
	movlw	B'00101000'
	call	WR_INS
	movlw	B'00001100'
	call	WR_INS
	movlw	B'00000110'
	call	WR_INS
	movlw	B'00000001'
	call	WR_INS

; Display Message ;;;;;;;;;;;;;;;;;;;;;;
	movlw	B'00000001'	; clear LCD
	Display EndMessage
	call Wait_K
	movlw	B'00000001'	; clear LCD
	call	WR_INS
	; display the operation time
	; take total T1 time in seconds
	movf	T1CNTH, W	; all to divide by 8 and fit into one byte
	movwf	TEMP2
	movf	T1CNTL, W
	movwf	TEMP1
	bcf	STATUS, C
	rrf	TEMP2, F
	rrf	TEMP1, F
	rrf	TEMP2, F
	rrf	TEMP1, F
	rrf	TEMP2, F
	rrf	TEMP1, F	; ends with time in secs
	movlw	ARMTIME3	; add stationary times and reversing times
	addwf	TEMP1, F	; to get total
	movlw	TURNTIME2
	addwf	TEMP1, F
	movlw	REVTIMES
	addwf	TEMP1, F	; done adding
	clrf	TEMP2		; should be clear anyway
	call	bin2dec999	; convert to ASCII digits (3)
	Display Optime		; write 'Operation Time'
	movf	TEMP5, W	; write from greatest to least to LCD
	call	WR_DATA
	movf	TEMP4, W
	call	WR_DATA
	movf	TEMP3, W
	call	WR_DATA
	Display Seconds		; add units
	call Wait_K
	; display the number of barrels
	movlw	B'00000001'	; clear LCD
	call	WR_INS
	movf	NUM_B, W	; convert barrel number to ASCII
	movwf	TEMP1
	call	bin2dec999
	Display Bar_Num		; write 'Number of Barrels'
	movf	TEMP4, W
	call	WR_DATA
	movf	TEMP3, W
	call	WR_DATA		; put barrel number
	call	Wait_K
	; display the number of tall and short barrels
	movlw	B'00000001'	; clear LCD
	call	WR_INS
	movf	NUM_TALL, W	; convert barrel number to ASCII
	movwf	TEMP1
	call	bin2dec999
	Display Tall_B_Num
	movf	TEMP4, W
	call	WR_DATA
	movf	TEMP3, W
	call	WR_DATA		; put barrel number
	call	Wait_K
	movlw	B'00000001'	; clear LCD
	call	WR_INS
	movf	NUM_B, W	; convert barrel number to ASCII, after subtract
	movwf	TEMP1
	movf	NUM_TALL, W
	subwf	TEMP1, F
	call	bin2dec999
	Display Short_B_Num
	movf	TEMP4, W
	call	WR_DATA
	movf	TEMP3, W
	call	WR_DATA		; put barrel number
	call	Wait_K
	; display the barrel data: distance, type, height
	movf	NUM_B, F	; Test NUM_B for zero
	btfsc	STATUS, Z	; if so, finish up
	goto	DDone
	movlw	NUM_B		; put the literal initial address in FSR
	movwf	FSR
	incf	FSR, F		; go to first entry
	movf	NUM_B, W
	movwf	COUNTB		; start counter with N
Disp_Loop
	movlw	B'00000001'	; clear LCD
	call	WR_INS
	incf	FSR, F
	movf	INDF, W		; convert barrel number to ASCII, low
	movwf	TEMP1
	incf	FSR, F		; high
	movf	INDF, W
	movwf	TEMP2
	movlw	B'00001111'	; get rid of sensor data
	andwf	TEMP2, F
	call	bin2dec999
	;Display Distance
	movf	TEMP5, W
	call	WR_DATA
	movf	TEMP4, W
	call	WR_DATA
	movf	TEMP3, W
	call	WR_DATA		; put distance
	Display Centi
	btfss	INDF, 4		; read barrel type
	goto	DShort		; if short, say short
	Display Tall		; else say tall
	btfsc	INTCON, 7	; known clear, just skips
DShort	Display Short
	btfsc	INDF, 6		; read barrel strip height
	goto	DFull		; display appropriate height
	btfss	INDF, 5
	goto	DEmpty
	Display Middle
	goto	LSTOP
DEmpty	Display Empty
	goto	LSTOP
DFull	Display Full
LSTOP	call	Wait_K		; standby
	decfsz	COUNTB, F	; when counter reaches zero exit
	goto	Disp_Loop
DDone	
	movlw	B'00000001'	; clear LCD
	call	WR_INS
	Display All_Done	; That's all, folks.
	call Wait_K
	sleep
Endloop
	goto Endloop
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; SUBROUTINES ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; General
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	; Standby
Wait_K	btfss	PORTB, 1	; check for keypad signal (any key pressed)
	goto	Wait_K		; if nothing loop
	return	

	; Timers
Overflow_Timer1
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
	;;; Check if timer counter = max count
	movf	T1CNTL, W
	movwf	PORTD
	movlw	MAXH
	xorwf	T1CNTH, W	; subtract max high from timer counter high
	btfss	STATUS, Z	; if they are different who cares
	return
	;movlw	MAXL
	;xorwf	T1CNTL, W	; substract max low from timer counter low
	;btfsc	STATUS, Z	; if they are the same goto exit routine
	goto	Exit_R 
	movlw	B'00000011'
	xorwf	NUM_B, W
	btfss	STATUS, Z
	;;;
        return		; back to waiting
	goto	Exit_R

Reset_Timer0
	bcf	INTCON, T0IF	; clear overflow
	clrf	TMR0		; set to zero, no padding
	return
Poll_Timer0
	btfss	INTCON, T0IF	; check timer0 overflow
	goto Poll_Timer0	; if yes go to reset, if no loop
	goto Reset_Timer0
Overflow_Timer0
	bcf	INTCON, T0IF	; clear interrupt flag
	return
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Barrel Subroutine
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
Barrel_Sub  ; basically take a photo through the sensors and deposit the data
	bsf	FLAG_B		; set barrel flag
	bcf	STATUS, IRP	; making sure indirect addressing is bank 0
	incf	NUM_B, F	; increment the number of barrels
	movlw	NUM_B		; put the literal initial address in FSR
	movwf	FSR
	rlf	NUM_B, W	; add the number of barrels*2 to FSR
	addwf	FSR, F
	movf	NUM_B, W
	movwf	PORTD
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
	; Now subtract the distance between start line and barrel detect
	movlw	DIST_PAD    ; get the padding
	subwf	INDF, F	    ; subtract from low byte
	btfss	STATUS, C   ; if negative decrease larger byte
	goto	Decrease_It
	incf	FSR, F	    ; exit at high byte
	goto	Sensing
Decrease_It
	incf	FSR, F
	decf	INDF, F
; Read and Save Sensors ;;;;;;;;;;;;;;;;;
Sensing
	movlw	0x0F
	andwf	INDF, F
	movf	PORTA, W    ; get sensors
	andlw	0x0F	    ; avoid any weird connections
	movwf	TEMP5
	swapf	TEMP5, W    ; put the results in the top 4 bits of the high
	iorwf	INDF, F	    ; register for this barrel
	btfsc	HEIGHT	    ; if barrel is tall
	incf	NUM_TALL, F ; add to tall barrel tally
	return		    ; done barrel subroutine
	
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; Column Subroutine
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Column_Sub
	bcf	COL_EN		; disable further interrupts from col
	bcf	T1CON, TMR1ON	; stop timer1
	clrf	PORTC		; stop motors
	bsf	ARMD
	bsf	ARME		; start arm
	call	Reset_Timer0	; reset timer0 before using
	movlw	ARMTIME		; set timer constant
	movwf	T0CNT
A1Loop
	call	Poll_Timer0	; get it running, delays 0.0262s
	decfsz	T0CNT
	goto	A1Loop		; loops until constant finishes
	movlw	B'00000101'	; start moving again
	movwf	PORTC
	movlw	COLTIME		; second time constant
	movwf	T0CNT		; still use the timer0 counter for this
	bsf	T1CON, TMR1ON
A2Loop
	btfss	PIR1, TMR1IF	; poll timer1 (some variance, max 0.125s)
	goto	A2Loop
	call	Overflow_Timer1	; natural timer1 increase
	decfsz	T0CNT		; until high-level constant runs down
	goto	A2Loop
	bcf	T1CON, TMR1ON
	movlw	B'00010000'	; put arm in reverse and stop motors
	movwf	PORTC
	call	Reset_Timer0
	movlw	ARMTIME2	; set timer constant
	movwf	T0CNT
A3Loop
	call	Poll_Timer0	; get it running, delays 0.0262s
	decfsz	T0CNT
	goto	A3Loop		; loops until constant finishes
	bsf	INTCON, INTE
	movlw	B'00000101'	; start moving again
	movwf	PORTC
	bsf	T1CON, TMR1ON	; start timer1 again
	return			; done column subroutine
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; LCD Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
; LCD Interfacing ;;;;;;;;;;;;;;;;;;
WR_INS	bcf	RS		; writes instructions to LCD, like clear
	movwf	COM
	andlw	0xF0
	movwf	PORTD
	bsf	E
	call	delay
	bcf	E
	swapf	COM, W
	andlw	0xF0
	movwf	PORTD
	bsf	E
	call	delay
	bcf	E
	call	delay
	return

WR_DATA	bsf	RS		; for writing ASCII to LCD
	movwf	DAT
	movf	DAT, W
	andlw	0xF0
	addlw	4
	movwf	PORTD
	bsf	E
	call	delay
	bcf	E
	swapf	DAT, W
	andlw	0xF0
	addlw	4
	movwf	PORTD
	bsf	E
	call	delay
	bcf	E
	return

delay
	movlw	0x4E		; for use in LCD subroutines
	movwf	d1
	movlw	0x14		; modified this to make it faster
	movwf	d2
delay_0
	decfsz	d1, F
	goto	$+2
	decfsz	d2, F
	goto	delay_0
	
	goto	$+1
	nop
	return

; LCD Calculations ;;;;;;;;;;;;;;;;;;
#define	NumL	TEMP1	; define variables for this subroutine, reusability
#define	NumH	TEMP2
#define	Ones	TEMP3
#define	Tens	TEMP4
#define	Hund	TEMP5

bin2dec999		; binary to ascii conversion <999
        movf NumH, w
        addlw 241
        addwf NumH, w
        movwf Hund      ;b_2 = 2a_2 - 15

        addwf Hund, w
        addwf Hund, w
        addlw 253
        movwf Tens
        swapf NumL, w
        andlw 0x0F
        addwf Tens, f
        addwf Tens, f   ;b_1 = 6a_2 + 2a_1 - 48

        addwf NumH, w
        sublw 251
        movwf Ones
        addwf Ones, f
        addwf Ones, f
        addwf Ones, f
        movf NumL, w
        andlw 0x0F
        addwf Ones, f   ;b_0 = a_0 - 4(a_2 + a_1) - 20

        movlw 10
bin2dec999a                     ;9 cycles max
        addwf Ones, f
        decf Tens, f
        btfss	STATUS, C
         goto bin2dec999a

bin2dec999b                     ;6 cycles max
        addwf Tens, f
        decf Hund, f
        btfss	STATUS, C
         goto bin2dec999b

bin2dec999c                     ;3 cycles max
        addwf Hund, f
        btfss	STATUS, C
         goto bin2dec999c

	movlw	B'00110000'
	iorwf	Hund, F		; my addition to make it actual ASCII
	iorwf	Tens, F
	iorwf	Ones, F
        return

	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; INTERRUPT HANDLER
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ISR	
	return			; there shouldn't be another reason
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; End of Program
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	end
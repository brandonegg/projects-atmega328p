.include "m328Pdef.inc"

.equ	zSize = 16

.def	loopCt = R20
.def    displayVal = R16
.def	state = R21		; 0 = incr, 1 = decr, 2 = reset = -1 (assuming "dec state" can bring state to -1)
.def	timer1 = R22		; counts number of 5 ms delay_long loops
.def	timer2 = R23		; may or may not need, depending on implementation

.cseg
.org 0
pArr:	.db	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71

; configure PB0-PB3 as output pins
sbi	DDRB,0	; PB0 is output (for SER)
sbi	DDRB,1	; PB1 is output (for RCLK)
sbi	DDRB,2	; PB2 is output (for SRCLK)
cbi	DDRB,3	; PB3 is input (for button)

reset:
	ldi state, 0			; set state to 0 (increment)
	ldi displayVal, 0x3F		; set pattern to 0
	ldi timer1, 0x01		; set timer1 to 1 sec ;0xcd

init_counter:
	ldi	ZL,LOW(2*pArr)		; initialize Z pointer
	ldi	ZH,HIGH(2*pArr)		; to pmem array address
	ldi	loopCt, zSize

; display a digit
count_up:
	lpm displayVal, Z+	; load pattern to display
	rcall display		; call display subroutine
	dec loopCt			; decrement loop count
	rcall button_handler ; await button press event
	tst loopCt           ; test if loopCt register is 0
	breq init_counter    ; if loopCt 0, reinit pointer and loopCt

	rjmp count_up
count_down:
	lpm displayVal, Z	; load pattern to display
	dec ZL				; decrement lower Z reg
	rcall display		; call display subroutine
	dec loopCt			; decrement loop count
	rcall button_handler ; await button press event
	tst loopCt           ; test if loopCt register is 0
	breq init_counter    ; if loopCt 0, reinit pointer and loopCt

	rjmp count_down

display:
	; backup used registers on stack
	push R16
	push R17
	in R17, SREG
	push R17
	
	ldi R17, 8 ; test DP bit
	rol R16 ; rotate left through Carry
	sbrc state,0	; skip if not in increment mode
	cbi PORTB,0  ; set SER to 0
	ldi R17, 7 ; loop --> test other 7 bits
loop:
	rol R16 ; rotate left through Carry
	brcs set_ser_in_1 ; branch if Carry is set
	cbi PORTB,0  ; set SER to 0
	
	rjmp end
set_ser_in_1:
	sbi PORTB,0  ; set SER to 1
end:
	sbi PORTB,2
	cbi PORTB,2  ; make SRCLK pulse
	
	dec R17
	brne loop
	
	sbi PORTB,1
	cbi PORTB,1  ; make RCLK pulse
	
	; restore regs from stack
	pop R17
	out SREG, R17
	pop R17
	pop R16
	
	ret

button_handler:
button_down:
	sbic PINB,3
	rjmp button_handler
button_loop:
	rcall delay_long	; 5 ms delay
	dec timer1
	brne button_up
	rcall switch_mode	; switch at 1 sec
button_up:
	sbis PINB,3
	rjmp button_loop
	ret

switch_mode:
	sbrc state,2		; skip unless 0100 or 0101 or 0110 or 0111
	rjmp reset
	ori state, 0x04		; (0000 -> 0100) or (0001 -> 0101)
	sbrs state,0
	rjmp switch_dec
	inc state		; 0100 -> 0101
	ret
switch_dec:
	dec state		; 0101 -> 0100
	ret

.equ count = 0x0096		; assign a 16-bit value to symbol "count"

delay_long:
	ldi R28, low(count)	  	; R29:R28  <-- load a 16-bit value into counter register for outer loop
	ldi R29, high(count);
d1:
	ldi   R27, 0xff		    	; R27 <-- load a 8-bit value into counter register for inner loop
d2:
	nop							; 1 clk
	dec   R27            		; R27 <-- R27 - 1, 1 clk
	brne  d2							; 1 clk if R27 = 0x00, else 2 clk
	sbiw R29:R28, 1					; R29:R28 <-- R29:R28 - 1
	brne d1								; 1 clk if R29:R28 = 0x0000, else 2 clk
	ret							; return
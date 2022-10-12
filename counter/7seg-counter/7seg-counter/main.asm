;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Assembly Language file for lab 2 (Embedded Systems)
; Fall 2022, The University of Iowa.
; Brandon Egger
; 9/25/2022
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.include "m328Pdef.inc"

; Constants
.equ	zSize = 16            ; Size of array Z pointer points to
.equ    oneSecTime = 0x7D     ; 2 sec in hex (2/8ms = 125)
.equ    twoSecTime = 0xFA     ; 1 sec in hex (1/8ms = 250)
.equ count = 0x007D		      ; delay_long delay - 8ms

; Register references
.def	loopCt = R20          ; the relative offset position of the counter array pointer
.def	state = R21		      ; 0 = incr, 1 = decr, 2 = reset = -1 (assuming "dec state" can bring state to -1)
.def	timer1 = R22          ; button hold register counter (value = R22 * 8ms, each loop is 8ms long - provided in count)
.def    displayVal = R16      ; Display value to be outputted register

.cseg
.org 0
; 7-Segment display values
segDisplayArr:	.db	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71

ldi state, 0

; configure PB0-PB3 as output pins
sbi   DDRB,0	; PB0 is output (for SER)
sbi   DDRB,1	; PB1 is output (for RCLK)
sbi   DDRB,2	; PB2 is output (for SRCLK)
cbi	  DDRB,3    ; PB3 is input (for button)

init_counter:
	ldi	ZL,LOW(2*segDisplayArr)		; initialize Z pointer
	ldi	ZH,HIGH(2*segDisplayArr)

	ldi loopCt, zSize               ; initialize loopCt to size of array

count_up:
	lpm displayVal, Z+   ; load Z pointer value to display, and increment
	rcall display		 ; call display subroutine
	dec	loopCt			 ; decrement loop count
	rcall button_handler ; await button press event
	tst loopCt           ; test if loopCt register is 0 (meaning fully iterated through number array)
	breq init_counter    ; if loopCt 0, reinit pointer and loopCt

	rjmp count_up

inc_loop_count:            ; Designed to prevent and overflow (going past size of array) of the loopCt when incrementing
	sbiw Z, 1
	inc loopCt
	cpi loopCt, 17
	brsh init_dec_counter  ; Exceeds size of array, so loop back to beginning of count down (top of array)
	ret

init_dec_counter:
	ldi	ZL,LOW(2*segDisplayArr)		; initialize Z pointer
	ldi	ZH,HIGH(2*segDisplayArr)

	adiw Z, zSize-1              ; Set Z pointer to end of array (add size of array)
	ldi loopCt, 0

count_down:
	lpm displayVal, Z
	rcall inc_loop_count
	ori displayVal, (1<<7)	; Set MSB high causing the '.' on 7seg to be lit
	rcall display			; call display subroutine
	rcall button_handler	; await button press event

	rjmp count_down

button_handler:
	clr timer1              ; make sure timer register is 0
button_down:
	sbic PINB,3             ; loop until button has been pressed down
	rjmp button_down
button_up:
	rcall delay_long		; call 8ms delay for each loop
	cpi timer1, 0xff        ; Hold the timer at max value 255 to prevent overflow
	brsh button_up_continue
	inc timer1

button_up_continue:
	sbis PINB,3             ; While button is still being held down, continue looping over delay and incrementing timer
	rjmp button_up

	cpi timer1, oneSecTime
	brsh switch_mode        ; if timer1 exceeds 1 second, we need to handle mode switching
	
	ret                     ; If no mode switch necessary, routine to previous routine (count_down or count_up)

; Below handles state switching after button press
switch_mode:
	cpi timer1, twoSecTime ; Check if press time was greater than 2 sec
	brsh mode_reset		   ; and if so, branch to reset subroutine

	tst state              ; check if state is 0
	breq mode_countdown    ; 0 state implies addition, branch to countdown mode switch subroutine

	ldi state, 0           ; Above state != 0, thus subtraction mode, so handle transition to addition
	
	adiw Z, 1			   ; Adjust Z pointer offset from previous count_down routine
	rjmp count_up          ; return to count up functionality

mode_countdown:
	inc state              ; Set z = 1
	dec loopCt             ; realign loopCt
	rcall inc_loop_count
	rjmp count_down

mode_reset:
	ldi state, 0		   ; Set state to addition
	rjmp init_counter      ; reinit the counter

; Displays patterns to 7-segment-display
display:
   ; backup used registers on stack
   push R16
   push R17
   in R17, SREG
   push R17
   
   ldi R17, 8 ; loop --> test all 8 bits
loop:
   rol R16			 ; rotate left through Carry
   BRCS set_ser_in_1 ; branch if Carry is set
   cbi PORTB,0		 ; insert code to set SER to 0
   
   rjmp end
set_ser_in_1:
	sbi PORTB,0		 ; insert code to set SER to 1
end:
	sbi PORTB,2
	cbi PORTB,2		 ; insert code to make SRCLK pulse
   
   dec R17
   brne loop
   
   sbi PORTB,1
   cbi PORTB,1		 ; insert code to make RCLK pulse
   
   ; restore regs from stack
   pop R17
   out SREG, R17
   pop R17
   pop R16
   
   ret

; Produces small delay for timing button press (based on count value)
delay_long:
	ldi r28, low(count)	  	; r31:r30  <-- load a 16-bit value into counter register for outer loop
	ldi r29, high(count);
d1:
	ldi   r27, 0xff		    ; r29 <-- load a 8-bit value into counter register for inner loop
d2:
	nop						; no operation
	dec   r27            	; r29 <-- r29 - 1
	brne  d2				; branch to d2 if result is not "0"
	sbiw r29:r28, 1			; r31:r30 <-- r31:r30 - 1
	brne d1					; branch to d1 if result is not "0"
	ret						; return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Assembly Language file for lab 3 (Embedded Systems)
; Fall 2022, The University of Iowa.
; Brandon Egger
; 9/25/2022
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.include "m328Pdef.inc"

; Constants
.equ	zSize = 16            ; Size of array Z pointer points to
.equ    codeH = 0x5E          ; Upper 8-bits of our code
.equ    codeL = 0x9F          ; Lower 8-bits of our code

; Register references
.def	zOffset = R20         ; the relative offset position of the counter array pointer
.def    displayVal   = R16    ; Display value to be outputted register
.def	inputState   = R17    ; For storing all inputs (bits 2,1,0 = pinA, pinB, button)
.def    debInState   = r18    ; Used ONLY to store current debounced state of pinD
.def    inputCodePtr = r25
.def    tmp1         = r19    ; tmp registers, can't assume they are safe outside subroutine
.def    tmp2         = r21
.def    tmp3         = r22
.def    tmp4         = r23
.def    tmp5         = r24

; NOTE: - X pointer is used for storing 3 second timing,
;       - Y pointer is used for storing code input

.cseg
.org 0
; 7-Segment display values
segDisplayArr:	.db	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71

sbi DDRB,0	; PB0 is output (for SER)
sbi DDRB,1	; PB1 is output (for RCLK)
sbi DDRB,2	; PB2 is output (for SRCLK)
sbi DDRB,5  ; Set status L to output
cbi	DDRD,2  ; PD2 is input (for B of RPG)
cbi	DDRD,3  ; PD3 is input (for A of RPG)
cbi	DDRD,4  ; PD4 push-button
sbi PORTD,2 ; pull-ups
sbi PORTD,3
sbi PORTD,4

rjmp init

count_up:
   cpi zOffset, zSize-1
   breq load_displayval
   inc zOffset			 ; decrement loop count
   rjmp load_displayval

count_down:
   tst zOffset
   breq load_displayval
   dec zOffset
   rjmp load_displayval

load_displayval:
   ldi	ZL,LOW(2*segDisplayArr)  ; initialize Z pointer
   ldi	ZH,HIGH(2*segDisplayArr)
   mov tmp4, zOffset
   add ZL, tmp4
   clr tmp4
   adc ZH, tmp4                  ; Add the ZOffset to our pointer
   lpm displayVal, Z             ; Load the value at start of segDisplayArr + ZOffset
   rcall display
   rjmp input_listener

pshbtn_routine:
   clr XL                        ; Clear X register used to store count.
   clr XH                        ; Clear X register used to count how many times button has been pressed.
   adiw X, 1
   rcall wait_for_button_release

wait_for_button_release:
   rcall load_input_state
   cpi XL, 0xFF            ; Check if lower 8 bits of word are about to overflow
   breq check_if_overflow
increment_button_count:
   adiw X, 1
check_button_release:
   sbrc debInState, 0
   rjmp push_btn_released       ; Branches to subroutine when button has been released
   rjmp wait_for_button_release

check_if_overflow:
   cpi XH, 0xFF                 ; If upper 8 bits overflow, don't increment word.
   breq check_button_release
   rjmp increment_button_count  ; Otherwise increment button count

init:
   cbi PORTB,5                    ; initialize status L to 0
   ldi	ZL, LOW(2*segDisplayArr)  ; initialize Z pointer
   ldi	ZH, HIGH(2*segDisplayArr)
   ldi	YL, 0	                  ; initialize code input word
   ldi	YH, 0
   ldi zOffset, 0                 ; initialize loopCt to size of array
   ldi inputCodePtr, 0
   
   ldi displayVal, (1<<6)         ; Display a -
   rcall display
   rjmp wait_for_rotate

; Wait for rotation before performing normal logic
wait_for_rotate:
   rcall load_input_state
   mov tmp2, debInState
   lsr tmp2
   cpi tmp2, 0b10        ; CW
   breq init_cw_routine  ; Start at 0, we skip increment step
   cpi tmp2, 0b01        ; CCW
   breq ccw_routine      ; no need to skip as it already is bounded by 0
   rjmp wait_for_rotate  ; loop until rotation received

push_btn_released:
   cpi XH, 0x00                 ; If high register 0, 3 sec condition wont be met.
   breq check_for_1sec_interval ; Only compare XL
   rjmp check_for_3sec_interval

check_for_1sec_interval:
   cpi XL, 0xF5                 ; 245, (1s/4.096ms (the time of one debounce timer)
   brlo input_selected          ; If less than 1 second
   rjmp input_listener          ; If > 1 second but < 3 seconds

check_for_3sec_interval:    ; Check if total X is > 3 seconds (0x02DC)
   cpi XH, 0x02
   brlo input_listener      ; If XH is less 0x02, we already know its not > 3 s
   cpi XH, 0x03
   brsh init                ; If XH >= 0x03, we already know it is greater, no need to check XL
check_for_3sec_interval_xl: ; For handling when XH = 0x02
   cpi XL, 0xDC
   brsh init                ; >= 3 seconds
   rjmp input_listener      ; < 3 seconds

correct_code:
   sbi PORTB,5               ; sets status L to 1
   ldi tmp4, 244             ; 4 second delay counter
start_4sec_loop_timer:
   rcall start_sixteen_timer
foursec_loop:
   in tmp5, TIFR0
   sbrs tmp5, TOV0           ; Check if timer is done
   rjmp foursec_loop

   dec tmp4
   cpi tmp4, 0
   breq init                 ; escape loop once 4 seconds up (244 16ms delays occured)
   rjmp start_4sec_loop_timer

incorrect_code:
   ldi displayVal, (1<<3)    ; Display _ character
   rcall display
   ldi XH, 0x02
   ldi XL, 0x62
start_10sec_loop_timer:
   rcall start_sixteen_timer
tensec_loop:
   in tmp5, TIFR0
   sbrs tmp5, TOV0            ; Check if timer is done
   rjmp tensec_loop

   sbiw X, 1
   cpi XL, 0
   breq check_high            ; check if high bit exceeds value first
   rjmp start_10sec_loop_timer
check_high:
   cpi XH, 0
   breq init
   rjmp start_10sec_loop_timer

input_listener:
   rcall load_input_state ; Loads values of each input into inputState and debInState
   mov tmp2, debInState
   lsr tmp2               ; RPG state stored in bits 2,1 of debInState

   cpi tmp2, 0b10         ; CW
   breq cw_routine
   cpi tmp2, 0b01         ; ccw
   breq ccw_routine

   sbrs debInState, 0     ; bit 0 is psh btn debounced
   rjmp pshbtn_routine

   rjmp input_listener

ccw_routine:
   rcall wait_for_return
   rjmp count_down

init_cw_routine:
   rcall wait_for_return
   rjmp load_displayval

cw_routine:
   rcall wait_for_return
   rjmp count_up

; Called when push button pressed for <= 1 second. Stores zOffset into next byte of Y
input_selected:
   cpi inputCodePtr, 2 ; On the 3rd operation, move YL to YH
   breq move_yl_to_yh
shift_and_add:
   lsl YL
   lsl YL
   lsl YL
   lsl YL              ; Shift four times 
   or YL, zOffset 
   inc inputCodePtr
check_if_four_inputs:
   cpi inputCodePtr, 4
   breq check_if_code_correct
   rjmp input_listener

check_if_code_correct:
   cpi YH, codeH
   brne incorrect_code ; YH was incorrect, code is incorrect
   cpi YL, codeL
   brne incorrect_code ; YL was incorrect, code is incorrect
   rjmp correct_code

move_yl_to_yh:
   mov YH, YL
   rjmp shift_and_add

wait_for_return:
   rcall load_input_state

   mov tmp2, debInState
   lsr tmp2

   ldi tmp1, 0b11          ; 11 is both channels off. Returned to 'idle' rotation state
   cpse tmp2, tmp1         ; Check if our debounce rpgState is correct
   rjmp wait_for_return
   ret

; Handles debouncing and loading of input state
load_input_state:
   rcall start_fourms_timer
   ldi tmp1, 0
   ldi tmp2, 0
   ldi tmp3, 0
load_input_timer_loop:
   in inputState, pind
   inc tmp4                     ; Keeps track of how many total times we collect data
   andi inputState, 0b00011100  ; We only care about pin 3,4,5. 0b channel A, channel B, button
   sbrc inputState, 4           ; push button
   inc tmp3
   sbrc inputState, 3           ; channel A
   inc tmp2
   sbrc inputState, 2           ; channel B
   inc tmp1

   in tmp5, TIFR0
   sbrs tmp5, TOV0              ; Check if timer is done
   rjmp load_input_timer_loop
find_most_likely_bits:
   clr debInState
   lsr tmp4                     ; Divide register by 2. All register counts greater than half the total cycles are valid inputs
   cp tmp1, tmp4
   brsh set_in_state_chanB
find_most_likely_bits_chanA:
   cp tmp2, tmp4
   brsh set_in_state_chanA
find_most_likely_bit_pshbtn:
   cp tmp3, tmp4
   brsh set_in_state_pshbtn
   ret

set_in_state_chanB:
   sbr debInState, 4
   rjmp find_most_likely_bits_chanA

set_in_state_chanA:
   sbr debInState, 2
   rjmp find_most_likely_bit_pshbtn

set_in_state_pshbtn:
   sbr debInState, 1
   ret

; initializes 4.096ms timer
start_fourms_timer:
   ldi tmp2, 0x00
   out TCCR0B, tmp2  ; stop timer 0
   in tmp2, TIFR0
   sbr tmp2, 1<<TOV0 ; TOV0 is # of bit its stored at. So we shift 1 that many bytes
   out TIFR0, tmp2   ; clear overflow flag

   ldi tmp2, 0
   ldi tmp1, 0b100   ; load configuration
   out TCNT0, tmp2   ; load to 5ms start point
   out TCCR0B, tmp1  ; Load config (starts timer)
   ret

; initializes 16.096 ms timer
start_sixteen_timer:
   ldi tmp2, 0x00
   out TCCR0B, tmp2  ; stop timer 0
   in tmp2, TIFR0
   sbr tmp2, 1<<TOV0 ; TOV0 is # of bit its stored at. So we shift 1 that many bytes
   out TIFR0, tmp2   ; clear overflow flag

   ldi tmp2, 0
   ldi tmp1, 0b101 ; load configuration
   out TCNT0, tmp2  ; load to 5ms start point
   out TCCR0B, tmp1  ; Load config (starts timer)
   ret

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

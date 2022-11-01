;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Assembly Language file for lab 4 (Embedded Systems)
; Fall 2022, The University of Iowa.
; Gustav Champlin, Brandon Egger
; 10/14/2022
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.include "m328Pdef.inc"

; MACROS
.listmac

.macro settimer2			; use timer2 for delays; settimer2 has 12 words (sts = 2) and takes 14 clk
	push	tmp1		; back up tmrOffset on stack
	ldi	tmp1, 0
	sts	TCCR2B, tmp1	; stop timer (TCCR2B is at 0xB1)
	in	tmp1, TIFR2
	sbr	tmp1, 1<<TOV2	; TOV2 is # of bit its stored at. So we shift 1 that many bytes
	out	TIFR2, tmp1	; clear overflow flag

	pop	tmp1			; restore tmrOffset from stack
	sts	TCNT2, tmp1	; load timer with offset (TCNT2 is at 0xB2)
	sts	TCCR2B, tmrConfig	; configure timer with prescaler
.endmacro
; END MACROS

.cseg
; INTERRUPTS
.org 0x00
rjmp	0x100

.org 0x0A
PCINT2_INT:  ; Configured to be called when PD6 goes high.
	inc rpmIntCount
	reti

.org 0x16
rjmp	TIM1_COMPA

.org 0x1C
rjmp	TIM0_COMPA

.org 0x40
TIM0_COMPA:
	sbi	PORTD,5
	reti

.org 0x60
TIM1_COMPA:               ; 16-bit timer, occurs every 0.5 seconds
    mov rpm, rpmIntCount
	clr rpmIntCount
	rjmp rpm_interrupt_helper

; END INTERRUPTS
.org 0x100

; Constants
.equ	pwmTOP    = 99		; TOP value for PWM timer --> 20 kHz with prescaler 2
.equ    rpm_delay = 0x7A12  ; 0.5s/(6.25E-8 * 256) = 31250 - 0.5 seconds for 16-second rpm timer
LCD_init_routine: .db 0x33,0x32,0x28,0x01,0x0c,0x06 ;Initialization routine called at start of application

; TESTING:
sbi DDRB,5  ; Set status L to output

; Setup pin change interrupt
cbi	DDRD, 6  ; PD6 fan rpm counter input
sbi	PORTD, 6 ; PD6 fan rpm counter, pull-up

ldi tmp1, (1 << PCINT22) ; Enable PCINT22 for pcint2 - enables 16-bit
sts PCMSK2, tmp1
ldi tmp1, (1 << PCIE2)
sts PCICR, tmp1
; end of pin change interrupt setup

; Display strings
dc_str:           .db "DC = ",0x00
fan_str:          .db "Fan: ",0x00
fan_ok:           .db "RPM OK ",0x00
fan_low:          .db "low RPM",0x00
fan_stopped:      .db "stopped",0x00
fan_off:          .db "OFF  ", 0x00

.equ init_nibble = 6

; Register references
.def	inputState  = r17    ; For storing all inputs (bits 2,1,0 = pinA, pinB, button)
.def    debInState  = r22    ; Used ONLY to store current debounced state of pinD
.def    tmrConfig   = r19    ; Used to set a timer's prescaler value (and PWM info if needed)
.def    lcdRefresh  = r20	 ; Used to load an 8-bit offset into a timer
.def	dutyCycle	= r21	 ; Used to change when (in each cycle) PWM output is changed
.def    rpmIntCount = r29
.def    rpm         = r24
.def    tmp1        = r18
.def    tmp2        = r23
.def    tmp3        = r27
.def    tmp4        = r28
.def    tmp5        = r19
.def    letter      = r25
.def    command     = r26

; INPUTS
cbi	DDRD,2  ; PD2 is input (for B of RPG)
cbi	DDRD,3  ; PD3 is input (for A of RPG)
cbi	DDRD,4  ; PD4 push-button
sbi PORTD,2 ; pull-up resistors
sbi PORTD,3
sbi PORTD,4
; LCD OUTPUTS
sbi DDRC,0  ; PC0 -> D4 of LCD (output)
sbi DDRC,1  ; PC1 -> D5 of LCD (output)
sbi DDRC,2  ; PC1 -> D6 of LCD (output)
sbi DDRC,3  ; PC1 -> D7 of LCD (output)

sbi DDRB,0  ; PB0 -> RS of LCD (output)
sbi DDRB,1  ; PB1 -> E of LCD (output)
; OTHER
sbi	DDRD,5	; Set OC0A as output to enable Compare Match Output A for timer0

; Initialize outputs
cbi PORTB, 1 ; E to 0
cbi PORTB, 0 ; RS to 0

sei				  ; set global interrupt enable
ldi	dutyCycle, 50 ; Init to dutycycle 50%

;***************************************************************************
; Main initialization routine
;***************************************************************************
; WAITING 0.1 seconds before beginning (required by LCD)
ldi r16, 7  ; 16MHz -> .1 / (6.25E-8 * 1024 * 256) ~= 7

delay_100ms:
   ldi tmp2, 0x00
   sts TCCR2B, tmp2  ; stop timer 0
   in tmp2, TIFR2
   sbr tmp2, 1<<TOV2 ; TOV0 is # of bit its stored at. So we shift 1 that many bytes
   out TIFR2, tmp2   ; clear overflow flag

   ldi tmp2, 0     ;
   ldi tmp1, 0b101   ; load configuration
   sts TCNT2, tmp2   ; load to 0 start point
   sts TCCR2B, tmp1  ; Load config (starts timer)
lcd_startup_wait:
   in tmp2, TIFR2
   sbrs tmp2, TOV2             ; Wait until timer done
   rjmp lcd_startup_wait

   dec r16
   tst r16                     ; Do this 7 times
   breq init_lcd
   rjmp delay_100ms
; WAITING COMPLETE, BEGIN INITIALIZING
init_lcd:
   rcall set_command_mode
   ldi	ZL,LOW(2*LCD_init_routine)  ; initialize Z pointer
   ldi	ZH,HIGH(2*LCD_init_routine)
   ldi r16, init_nibble             ; Keep track so we know when we're done sending commands
init_lcd_loop:
   lpm command, Z+
   rcall send_command
   dec r16
   tst r16
   breq main
   rjmp init_lcd_loop
; LCD INITIALIZATION COMPLETE

init_duty_cycle:
	ldi	tmrConfig, 0b00101010	; contains settings for PWM
setup_timer0:
    push tmp1
	ldi	tmp1, 0
	out	TCCR0B, tmp1	; stop timer
	;in	tmrOffset, TIFR0	; can be removed if ISR doesn't use TOV
	;sbr	tmrOffset, 1<<TOV0	; can be removed if ISR doesn't use TOV
	;out	TIFR0, tmrOffset	; clear overflow flag; can be removed if ISR doesn't use TOV
	ldi	tmp1, 0b10		; 0b10 enables Compare Match A, 0b100 enables Compare Match B
	sts	TIMSK0, tmp1	; enable interrupt at Compare Match A
	ldi	tmp1, pwmTOP
	out	OCR0A, tmp1	; load TOP value for PWM timer
	
	out	OCR0B, dutyCycle	; load Output Compare value for PWM timer
	push	tmrConfig		; back up tmrConfig on stack
	andi	tmrConfig, 0x0F	; isolate WGM0(2) and CS0(2:0)
	out	TCCR0B, tmrConfig	; configure PWM timer with prescaler

	pop	tmrConfig			; restore tmrConfig from stack
	andi	tmrConfig, 0x30	; isolate COM0B(1:0)
	ori	tmrConfig, 0b11		; add WGM0(1:0) which should always be 0b11
	out	TCCR0A, tmrConfig	; further configure PWM timer
	; PWM should automatically operate fan
	pop tmp1
	ret

init_rpm_timer:
   ldi tmp1, high(rpm_delay)
   sts OCR1AH, tmp1
   ldi tmp1, low(rpm_delay)
   sts OCR1AL, tmp1

   ldi tmp1, (1<<OCIE1A)
   sts TIMSK1, tmp1                              ; Output Compare A match interrupt enable, see TIM1_COMPA

   ldi tmp1, 0x00                                ; Configure CTC mode with clk_io/256
   sts TCCR1A, tmp1
   ldi tmp1, (1<<WGM12) | (1<<CS12)  ; Start timer clk_io/256
   sts TCCR1B, tmp1

init_timer0:
	rcall init_duty_cycle
rjmp input_loop

rpm_interrupt_helper:    ; Called at end of 16-bit RPM timer. Resets timer and refreshes LCD fan info
   push tmp1
   push tmp2
   push tmp3
   push tmp4
   push tmp5
   in tmp1, SREG
   push tmp1

   ldi tmp1, high(rpm_delay) ; Init 16-bit timer
   sts OCR1AH, tmp1
   ldi tmp1, low(rpm_delay)
   sts OCR1AL, tmp1

   rcall display_update_dc
   rcall display_update_rpm

   pop tmp1
   out SREG, tmp1
   pop tmp5
   pop tmp4
   pop tmp3
   pop tmp2
   pop tmp1
rpm_int_return:
   reti

;***************************************************************************
; Main rotuine after initialization has occured
;***************************************************************************
main:
   rcall display_lcd_constants
   rjmp init_rpm_timer
input_loop:
   rcall load_input_state
   rcall handle_input_state
   rjmp input_loop

;***************************************************************************
; Input listeners/Debounce
;***************************************************************************
; initializes 4.096ms timer
start_debounce_timer:
   ldi tmp2, 0x00
   sts TCCR2B, tmp2  ; stop timer 0
   in tmp2, TIFR2
   sbr tmp2, 1<<TOV2 ; TOV0 is # of bit its stored at. So we shift 1 that many bytes
   out TIFR2, tmp2   ; clear overflow flag

   ldi tmp2, 0
   ldi tmp1, 0b010   ; load configuration
   sts TCNT2, tmp2   ; load to 5ms start point
   sts TCCR2B, tmp1  ; Load config (starts timer)
   ret

; Handles debouncing and loading of input state
load_input_state:
   rcall start_debounce_timer
   ldi tmp1, 0
   ldi tmp2, 0
   ldi tmp3, 0
   ldi tmp4, 0
load_input_timer_loop:
   in inputState, pind
   inc tmp4                     ; Keeps track of how many total times we collect data
   sbrc inputState, 3           ; channel A
   inc tmp2
   sbrc inputState, 2           ; channel B
   inc tmp1
   sbrc inputState, 4           ; push button
   inc tmp3

   in tmp5, TIFR2
   sbrs tmp5, TOV2              ; Check if timer is done
   rjmp load_input_timer_loop
find_most_likely_bits:
   clr debInState
   lsr tmp4                     ; Divide register by 2. All register counts greater than half the total cycles are valid inputs
   cp tmp1, tmp4
   brsh set_deb_chanB
check_chanA:
   cp tmp2, tmp4
   brsh set_deb_chanA
check_pshbtn:
   cp tmp3, tmp4
   brsh set_pshbtn
   ret
set_deb_chanA:
   ori debInState, 2
   rjmp check_pshbtn
set_deb_chanB:
   ori debInState, 4
   rjmp check_chanA
set_pshbtn:
   ori debInState, 1
   ret

handle_input_state:
   andi debInState, 0b00000111
   sbrs debInState, 0
   rjmp turned_off_state

   lsr debInState
   cpi debInState, 0b10
   breq rpg_rotate_right
   cpi debInState, 0b01
   breq rpg_rotate_left
   ret

turned_off_state:
   rcall wait_for_release
   ldi tmrConfig, 0
   sts TCCR1B, tmrConfig
   mov rpm, dutyCycle
   ldi dutyCycle, 0
   rcall init_duty_cycle

   ldi command, 0b11000101 ; Set address to 0x45
   rcall send_command
   rcall display_fan_stopped

   rcall display_dc_off
turned_off_loop:
   rcall load_input_state
   andi debInState, 0b00000001
   sbrs debInState, 0
   rjmp turn_on

   rjmp turned_off_loop
turn_on:
   rcall wait_for_release
   mov dutyCycle, rpm
   rjmp init_rpm_timer

wait_for_release:
   rcall load_input_state
   andi debInState, 0b00000001
   sbrc debInState, 0
   ret

   rjmp wait_for_release

wait_for_rotate_complete:
   rcall load_input_state
   andi debInState, 0b00000111
   lsr debInState
   cpi debInState, 0b11                 ; Check if debounced input is 11, aka returned to complete rotation
   brne wait_for_rotate_complete
   ret

rpg_rotate_left:
   rcall wait_for_rotate_complete
   rcall dec_duty_cycle
   rjmp init_timer0

rpg_rotate_right:
   rcall wait_for_rotate_complete
   rcall inc_duty_cycle
   rjmp init_timer0

; Incrementing and Decrementing duty cycle
dec_duty_cycle:
   cpi dutyCycle, 0
   brne count_down_dc
   ret
count_down_dc:
   dec dutyCycle
   ret

inc_duty_cycle:
   cpi dutyCycle, 100
   brne count_up_dc
   ret
count_up_dc:
   inc dutyCycle
   ret

;***************************************************************************
; Commonly used timers (all utilizing timer2)
;***************************************************************************
delay_5ms: ; 5 ms timer
   rcall set_5ms
   rjmp timer_loop
set_5ms:
   push tmp1
   push tmp2

   ldi tmp2, 0x00
   sts TCCR2B, tmp2  ; stop timer 0
   in tmp2, TIFR2
   sbr tmp2, 1<<TOV2 ; TOV0 is # of bit its stored at. So we shift 1 that many bytes
   out TIFR2, tmp2   ; clear overflow flag

   ldi tmp2, 170     ; 256 - (5E-3/(6.25E-8*8*1024)) - round up
   ldi tmp1, 0b111   ; load configuration
   sts TCNT2, tmp2   ; load to 5ms start point
   sts TCCR2B, tmp1  ; Load config (starts timer)

   pop tmp2
   pop tmp1
   ret

delay_230ns: ; 230 ns timer
   push tmp1
   push tmp2

   ldi tmp2, 0x00
   sts TCCR2B, tmp2  ; stop timer 0
   in tmp2, TIFR2
   sbr tmp2, 1<<TOV2 ; TOV0 is # of bit its stored at. So we shift 1 that many bytes
   out TIFR2, tmp2   ; clear overflow flag

   ldi tmp2, 250     ; 256 - ((1E-9/6.25E-8) * 230) - round up
   ldi tmp1, 0b001   ; load configuration
   sts TCNT2, tmp2   ; load to 5ms start point
   sts TCCR2B, tmp1  ; Load config (starts timer)

   pop tmp2
   pop tmp1
   rjmp timer_loop
timer_loop:
   in tmp2, TIFR2
   sbrs tmp2, TOV2   ; Wait until timer done
   rjmp timer_loop
   ret

delay_100u: ; 5 ms timer
   rcall set_100u
   rjmp timer_loop
set_100u:          ; 100us timer
   push tmp1
   push tmp2

   ldi tmp2, 0x00
   sts TCCR2B, tmp2  ; stop timer 0
   in tmp2, TIFR2
   sbr tmp2, 1<<TOV2 ; TOV0 is # of bit its stored at. So we shift 1 that many bytes
   out TIFR2, tmp2   ; clear overflow flag

   ldi tmp2, 50      ; 256 - (100E-6/(6.25E-8*8))
   ldi tmp1, 0b010   ; load configuration
   sts TCNT2, tmp2   ; load to 5ms start point
   sts TCCR2B, tmp1  ; Load config (starts timer)

   pop tmp2
   pop tmp1
   ret

;***************************************************************************
; The following is the main LCD display subroutine. Each step handles a
; different portion of the display functionality.
;***************************************************************************
display_lcd_constants:
   rcall display_upper_const ; Displays DC = 
   rcall display_lower_const ; Displays Fan = 
   rcall display_update_dc   ; Sets the duty cycle
   rcall display_update_rpm  ; Displays rpm fan status
   ret

display_upper_const:
   ldi command, 0b00000010 ; Send cursor home
   rcall send_command      

   ldi	ZL,LOW(2*dc_str)   ; initialize Z pointer
   ldi	ZH,HIGH(2*dc_str)  ; to upper string base
   rcall display_letters
   ret

display_lower_const:
   ldi command, 0b11000000 ; Send to first cell of second row
   rcall send_command

   ldi	ZL,LOW(2*fan_str)   ; initialize Z pointer
   ldi	ZH,HIGH(2*fan_str)  ; to upper string base
   rcall display_letters
   ret

display_dc_off:
   ldi command, 0b10000101 ; Set address to 0x05
   rcall send_command

   ldi	ZL,LOW(2*fan_off)   ; initialize Z pointer
   ldi	ZH,HIGH(2*fan_off)  ; to upper string base
   rcall display_letters
   ret

display_update_dc:
   ldi command, 0b10000101 ; Set address to 0x05
   rcall send_command

   mov letter, dutyCycle   ; Copy duty cycle over to letter register
   rcall display_numbers
   ldi letter, 0x25
   rcall display_letter    ; Display percent
   ret

display_update_rpm:
   ldi command, 0b11000101 ; Set address to 0x45
   rcall send_command

   tst rpm
   breq display_fan_stopped
   cpi rpm, 40              ; 2400 rpm/60 = 40rps
   brlo display_fan_low     ; display low rpm text

   ldi	ZL,LOW(2*fan_ok)   ; initialize Z pointer
   ldi	ZH,HIGH(2*fan_ok)  ; to upper string base
   rcall display_letters
   ret
display_fan_low:
   ldi	ZL,LOW(2*fan_low)   ; initialize Z pointer
   ldi	ZH,HIGH(2*fan_low)  ; to upper string base
   rcall display_letters
   ret
display_fan_stopped:
   ldi	ZL,LOW(2*fan_stopped)   ; initialize Z pointer
   ldi	ZH,HIGH(2*fan_stopped)  ; to upper string base
   rcall display_letters
   ret

;***************************************************************************
; The following subroutines define commonly used LCD commands.
;***************************************************************************
pulse_e:             ; Pulses E of LCD
   sbi PORTB, 1
   rcall delay_230ns ; Requires 230ns wait between high/low
   cbi PORTB, 1
   ret

set_character_mode:
   sbi PORTB, 0
   ret

set_command_mode: ; Sets LCD to command mode (RS bit cleared)
   cbi PORTB, 0
   ret

shift_cursor_right:
   ldi command, 0b00010100
   rcall send_command
   ret

send_command:              ; Generic routine to push whatever command is sitting in command register to LCD
   rcall set_command_mode
   swap command
   out PORTC, command
   rcall pulse_e
   rcall delay_5ms
   swap command
   out PORTC, command
   rcall pulse_e
   rcall delay_5ms
   rcall set_character_mode
   ret

;***************************************************************************
; The following subroutines are designed for taking letters and outputting
; them to the display.
;
; These routines do not handle where the letters will go, this is not their
; responsibility.
;
; display_letters takes the Z pointer (which should be initialized to a
; a series of words terminated with 0x00) and will display them in order.
;
; display_letter takes a letter stored in the 'letter' register and outputs
; this letter to the LCD.
;***************************************************************************
; This will always display numbers with 3 digits, aka 100 shows as 100, 50 returns 50.0% and 0 displays 0.00%
display_numbers:             ; uses value stored in letter as 'number'
   push tmp4
   ldi tmp4, 3               ; Used to keep track of places remaining
   rcall shift_cursor_right  ; Supports numbers of 3 digits (0-100% is all we need)
   rcall shift_cursor_right
   ldi command, 0x04         ; Change to decrement 
   rcall send_command
   rcall set_character_mode
display_numbers_loop:
   mov r16, letter           ; set dividend - in this case letter is number we are producing
   ldi r17, 0x0A             ; set divisor
   rcall div8u

   mov letter, r15
   ori letter, 0x30
      
   dec tmp4
   rcall display_letter
   cpi r16, 0x0A
   brlo disp_number_return
   mov letter, r16
   rjmp display_numbers_loop
disp_number_return:
   mov letter, r16
   ori letter, 0x30
   dec tmp4
   rcall display_letter
add_remaining_spaces:
   cpi tmp4, 0
   breq undo_display_shift_mode
   dec tmp4
   ldi letter, 0b00100000
   rcall display_letter
   rjmp add_remaining_spaces
undo_display_shift_mode:
   ldi command, 0x06            ; Change display mode 
   rcall send_command
   rcall shift_cursor_right  ; Supports numbers of 3 digits (0-100% is all we need)
   rcall shift_cursor_right
   rcall shift_cursor_right
   rcall shift_cursor_right
   rcall set_character_mode
   pop tmp4
   ret

display_letters:            ; Uses z pointer to display letters until 0x00 is reached
   rcall set_character_mode
   lpm letter, Z+
   tst letter
   breq display_return      ; Return once end of string encountered
   rcall display_letter
   rjmp display_letters
display_letter:
   swap letter 
   out   PORTC,letter       ; Send upper nibble
   rcall pulse_e            ; Strobe Enable line 
   rcall delay_100u         ; wait
   swap letter    
   out   PORTC,letter       ; Send lower nibble
   rcall pulse_e            ; Strobe Enable line 
   rcall delay_100u
   ret
display_return:
   ret

;***************************************************************************
;*
;* "div8u" - 8/8 Bit Unsigned Division
;*
;* This subroutine divides the two register variables "dd8u" (dividend) and 
;* "dv8u" (divisor). The result is placed in "dres8u" and the remainder in
;* "drem8u".
;*  
;* Number of words	:14
;* Number of cycles	:97
;* Low registers used	:1 (drem8u)
;* High registers used  :3 (dres8u/dd8u,dv8u,dcnt8u)
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem8u	=r15		;remainder
.def	dres8u	=r16		;result
.def	dd8u	=r16		;dividend
.def	dv8u	=r17		;divisor
.def	dcnt8u	=r18		;loop counter

;***** Code

div8u:	sub	drem8u,drem8u	;clear remainder and carry
	ldi	dcnt8u,9	;init loop counter
d8u_1:	rol	dd8u		;shift left dividend
	dec	dcnt8u		;decrement counter
	brne	d8u_2		;if done
	ret			;    return
d8u_2:	rol	drem8u		;shift dividend into remainder
	sub	drem8u,dv8u	;remainder = remainder - divisor
	brcc	d8u_3		;if result negative
	add	drem8u,dv8u	;    restore remainder
	clc			;    clear carry to be shifted into result
	rjmp	d8u_1		;else
d8u_3:	sec			;    set carry to be shifted into result
	rjmp	d8u_1
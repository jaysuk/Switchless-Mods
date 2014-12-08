#include <p12f629.inc>

; -----------------------------------------------------------------------
;   NES "In-game reset" (IGR) controller
;
;   Copyright (C) 2013 by Peter Bartmann <peter.bartmann@gmx.de>
;
;   This program is free software; you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation; version 2 of the License only.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program; if not, write to the Free Software
;   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
;
; -----------------------------------------------------------------------
;
;   This program is designed to run on a PIC 12F629 microcontroller connected
;   to the controller port and NES main board. It allows an NES to be reset
;   via a standard controller.
;
;   pin configuration: (controller port pin) [Mainboard pin]
;                                ,------_-----.
;           +5V (7) [CIC Pin 16] |1          8| GND (1) [CIC Pin 15]
;   Reset in/out (-) [CIC Pin 7] |2  GP5 GP0 7| serial data in (4) [U7 74HC368 Pin 2]
;              not connected out |3  GP4 GP1 6| latch in (3)[CPU Pin 39]
;                         GND in |4  GP3 GP2 5| clk in (2) [CPU Pin 36]
;                                `------------'
;
;   As the internal oscillator is used, you should connect a capacitor of about 100nF between
;   Pin 1 (Vdd/+5V) and Pin 8 (Vss/GND) as close as possible to the PIC. This esures best
;   operation. Pins 3 and 4 are not used, but it is highly recommend to connect them to to
;   a voltage level of either Vss or Vcc. 
;
;   The voltage level on pin 4 gives the reset type. At the famicom system the reset is low
;   active, at the nintendo entertainment system high active.
;   - pin 4 connected to GND: reset is low active
;   - pin 4 connected to +5V: reset is high active
;
;   controller pin numbering
;   ========================
;
;        _______________
;       |               \     (1) - GND          (5) - not connected
;       | (5) (6) (7)    \    (2) - clk          (6) - not connected
;       | (4) (3) (2) (1) |   (3) - latch        (7) - Power, +5V 
;       |_________________|   (4) - serial data
;
;
;   key mapping  : Start+Select+A+B
;   stream data  : 0x0f
;   functionality: Reset, i.e., the console is simply resetted
;
; -----------------------------------------------------------------------
; Configuration bits: adapt to your setup and needs
    __CONFIG _INTRC_OSC_NOCLKOUT & _WDT_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF

; -----------------------------------------------------------------------
PROG CODE

start
        banksel	GPIO 	;Bank 0
        clrf    GPIO
        movlw   0x07            ; GPIO2..0 are digital I/O (not connected to comparator)
        movwf   CMCON
        movlw   0x10            ; enable interrupts: INTE
        movwf   INTCON
        banksel TRISIO	;Bank 1
        call 3FFh ;Get the cal value
        movwf OSCCAL ;Calibrate
        movlw   0x2f            ; in out in in in in
        movwf   TRISIO
        movlw   0x00            ; no pullups
        movwf   WPU
        movlw   0x80            ; global pullup disable, use rising_edge on GPIO2
;        movlw   0xc0            ; global pullup disable, use falling_edge on GPIO2
        movwf   OPTION_REG
        banksel	GPIO 	;Bank 0
	
idle
	clrf	0x40
	movlw   0x07
    movwf	0x41
	bcf     STATUS, C	; clear carry

waitforlatch_h; wait for "latch" to become high
    movfw   GPIO
	btfss	W, 1
	goto	waitforlatch_h  

read_data_after_latch
;	movf	GPIO, w 	;read data bit
	andlw	0x01
	iorwf	0x40, f		;put in data reg

wait_for_clk
    bcf     INTCON, INTF
    sleep

read_data_after_clk
	rlf     0x40, f		;shift
	movf	GPIO, w 	;read data bit
	andlw	0x01
	iorwf	0x40, f		;put in data reg
	decfsz	0x41		;data read done?
	goto	wait_for_clk
	

checkkeys
; we now have the 8 data bits in 0x40
	movlw	0x0f
	xorwf	0x40, w
	btfss	STATUS, Z
	goto	idle

doreset
	btfss	GPIO, 5
	goto 	doreset_high
	
doreset_low
    banksel	TRISIO 	;Bank 1
    bcf     TRISIO, 5
    banksel	GPIO 	;Bank 0
	bcf     GPIO, 5
	movlw	0x02
	call	superlongwait
	bsf     GPIO, 5
    banksel	TRISIO 	;Bank 1
	bsf     TRISIO, 5
    banksel	GPIO 	;Bank 0
	goto	idle
	
doreset_high
    banksel	TRISIO 	;Bank 1
    bcf     TRISIO, 5
    banksel	GPIO 	;Bank 0
	bsf     GPIO, 5
	movlw	0x02
	call	superlongwait
	bcf     GPIO, 5
    banksel	TRISIO 	;Bank 1
	bsf     TRISIO, 5
    banksel	GPIO 	;Bank 0
	goto	idle
	
; --------wait: 3*(W-1)+7 cycles (including call+return). W=0 -> 256!--------
wait
        movwf   0x4f
wait0   decfsz  0x4f, f
        goto    wait0
        goto	longwait1

; --------wait long: 8+(3*(w-1))+(772*w). W=0 -> 256!--------
longwait
        movwf   0x4e
        clrw
longwait0
        goto	wait
longwait1
        decfsz  0x4e, f
        goto    longwait0
        return

; --------wait extra long: 8+(3*(w-1))+(198405*w).
superlongwait
	movwf	0x4d
	clrw
superlongwait0
	call	longwait
	decfsz	0x4d, f
	goto	superlongwait0
	return
	
END

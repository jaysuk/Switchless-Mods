#include <p16f630.inc>
processor p16f630

;	Sega Mega Drive/Genesis switchless mod
;	d4s 2010-10-31 matt@dforce3000.de
;
;	Pressing Mega Drive reset switch cycles through three available modes indicated by Status LED color:
;		-50Hz/en	(Europe, green)
;		-60Hz/en	(USA, orange)
;		-60Hz/jp	(Japan, red)
;
;	Both active-high and active-low reset consoles are supported by sensing the reset line level
;	on startup.
;	Indicator LED is optional, but strongly recommended.
;	See corresponding PDF file for connection points on different pcb versions
;
;	Tested working on the following pcb versions:	
;		-IC BD M5 PAL VA4		(PAL early Mega Drive Mk1, active-high reset)
;		-IC BD M5 PAL			(PAL late Mega Drive Mk1, active-low reset)
;		-PC BD MD2 VA1.8 PAL	(PAL Mega Drive Mk2, active-low reset)
;
;	Untested on american, japanese or Mk3 consoles, but should work in theory.
;	Please report your experience with any pcb versions not listed here.
;
;   pin configuration: 
;
;                       ,-----_-----.
;                   +5V |1        14| GND
;                       |2  A5 A0 13| RESET_IN, /RESET_IN (Reset Button)
;                       |3  A4 A1 12| 
;                       |4  A3 A2 11| 
;        (red) LED_OUT2 |5  C5 C0 10| LANGUAGE_OUT (en/jp)
;        (grn) LED_OUT1 |6  C4 C1  9| VIDEOMODE_OUT (50/60Hz)
;                       |7  C3 C2  8| RESET_OUT, /RESET_OUT (VDP reset line)
;                       `-----------'
;
;	(/)RESET_IN				-	Reset Button Input (needs pulldown resistor (5 KOhm) on active-high reset consoles). *
;	LANGUAGE_OUT			-	English(high)/Japanese(low), JP1/JP2 on Mega Drive Mk1, C65/C68 on Genesis 3. **
;	VIDEOMODE_OUT			-	60Hz(high)/50Hz(low), JP3/JP4 on Mega Drive Mk1, C63/C64 on Genesis 3. **
;	(/)RESET_OUT			-	Reset output to videochip. *
:	LED_OUT1, LED_OUT2		-	Use 3-pin dual-color (red/green) LED here. Connect common base of LED to GND via 220 Ohm resistor.
;
;	* Cut reset line between reset button and videochip. Be sure to leave pulldown resistor connected to reset button on active-high reset units.
;	** Make sure to remove/cut any connection on set jumpers. Mk2 consoles don't have any jumpers, you'll have to cut lines here.
;
;	If you experience noise problems, you may add a capacitor near the PIC between the +5V and GND lines. I never had any, though.
;
;	Parts needed:
;		-Microchip PIC 16f630
;		-Some wire
;		-Dual-color LED				(optional)
;		-220 Ohm resistor			(optional)
;		-100nF Ceramic Capacitor	(optional)
;
; -----------------------------------------------------------------------
    __CONFIG _INTRC_OSC_NOCLKOUT & _WDT_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF

;port a
RESET_BUTTON equ 0

;port c
LANGUAGE equ 0
VIDEOMODE equ 1
RESET_OUT equ 2
LED1 equ 4
LED2 equ 5

;bitflags for byte ops
RESET_BUTTON_BIT	equ 1 << RESET_BUTTON
LED1_BIT			equ 1 << LED1
LED2_BIT			equ 1 << LED2
LANGUAGE_BIT		equ 1 << LANGUAGE
RESET_OUT_BIT		equ 1 << RESET_OUT
VIDEOMODE_BIT		equ 1 << VIDEOMODE

;variables
varBlock udata_shr
	modeCounter		res 1
	outBuffer		res 1
	waitBuff		res 2
	buttonStatus	res 1
	buff			res 1
	resetInversion	res 1

;code start
boot code 0
	goto main

main
	call init
	call senseResetInversion
	call clearReset
	call loadModeFromEEPROM
	call setLEDs
	call applySettings

mainLoop
	call getResetButtonState
	btfss buttonStatus,1			;wait for press on reset button
		goto mainLoop

	call waitLong
	call getResetButtonState
	btfsc buttonStatus,1			;if release occured, reset console
		goto changeSettingsLoop
	call doReset
	goto mainLoop

changeSettingsLoop
	call cycleModeCounter
	call setLEDs
	call waitLong
	call getResetButtonState
	btfsc buttonStatus,1			;cycle modes while reset button is held down
		goto changeSettingsLoop
	call applySettings				;set mode and save to EEPROM if button released
	call saveModeToEEPROM
	goto mainLoop

applySettings
	movfw PORTC
	movwf outBuffer
	bcf outBuffer, LANGUAGE
	bcf outBuffer, VIDEOMODE
	btfsc modeCounter, 0
		bsf outBuffer, LANGUAGE
	btfsc modeCounter, 1
		bsf outBuffer, VIDEOMODE
	movfw outBuffer
	movwf PORTC
	return

setLEDs
	movfw PORTC
	movwf outBuffer
	bcf outBuffer, LED1
	bcf outBuffer, LED2
	btfsc modeCounter, 0
		bsf outBuffer, LED1
	btfsc modeCounter, 1
		bsf outBuffer, LED2
	movfw outBuffer
	movwf PORTC
	return

clearLEDs
	bcf PORTC, LED1
	bcf PORTC, LED2
	return

loadModeFromEEPROM
	banksel EEADR					;fetch current mode from EEPROM
	clrf    EEADR					;address 0
	bsf     EECON1, RD
	movfw   EEDAT
	banksel PORTA
	andlw 3							;compensate for invalid modes(initial run or bad EEPROM)
	btfsc	STATUS,Z
		movlw 1
	movwf	modeCounter
	return

saveModeToEEPROM
	movfw	modeCounter
	banksel	EEADR					;save to EEPROM. note: banksels take two cycles each!
	movwf	EEDAT
	bsf	EECON1,WREN
	movlw	0x55
	movwf	EECON2
	movlw	0xaa
	movwf	EECON2
	bsf		EECON1, WR
	banksel	PORTA
	return

cycleModeCounter
	decfsz modeCounter
		goto dontResetModeCount
	movlw 3							;mode counter underflowed, reset to maximum
	movwf modeCounter

dontResetModeCount
	movfw modeCounter				;clip to maximum of 3 modes(+1 underflow detect)
	andlw 3
	movwf modeCounter
	return

getResetButtonState
	clrf buttonStatus
	call getResetInvertedPort
	btfss buff, RESET_BUTTON
		return
	call wait						;debounce reset button
	call getResetInvertedPort
	btfss buff, RESET_BUTTON
		return
	bsf buttonStatus,1
	return

senseResetInversion
	clrf resetInversion				;invert reset in/out depending on initial reset line state
	btfsc PORTA, RESET_BUTTON
		bsf resetInversion, 1
	return

getResetInvertedPort
	movfw PORTA
	btfsc resetInversion, 1
		xorlw RESET_BUTTON_BIT
	movwf buff
	return

doReset
	call setReset
	call waitLong
	call clearReset
	return

setReset
	btfsc resetInversion, 1
		goto setLowReset
	goto setHighReset

clearReset
	btfsc resetInversion, 1
		goto setHighReset
	goto setLowReset

setHighReset
	bsf PORTC, RESET_OUT
	return

setLowReset
	bcf PORTC, RESET_OUT
	return

init
	bcf 	STATUS, RP0				;bank 0 regs
	clrf	PORTA					;clear ports
	clrf	PORTC
	movlw	0x07					;a0/a2 digital 
	movwf	CMCON
	clrf	INTCON					;disable all interrupts, disable port a pullups

	bsf 	STATUS, RP0				;bank 1 regs
	clrf	TRISA
	bsf		TRISA, RESET_BUTTON
	clrf	TRISC
	movlw	0b111111				;maximum frequency
	movwf	OSCCAL
	clrf	WPUA
	bsf		WPUA, RESET_BUTTON		;weak pullup on reset button
	clrf	OPTION_REG				
	bcf		OPTION_REG, NOT_RAPU	;enable pullups on port a

	bcf 	STATUS, RP0				;return to bank 0 
	bcf		INTCON, RAIF			;clear irq on change flag
	return

waitLong
	clrf	waitBuff+1
waitLongLoop
		call wait
		decfsz	waitBuff+1, f
		goto	waitLongLoop
	return	

wait
	clrf	waitBuff
waitLoop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		decfsz	waitBuff, f
		goto	waitLoop
	return

end


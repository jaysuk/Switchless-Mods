#include <p12f635.inc>

; -----------------------------------------------------------------------
;   NES "In-game reset" (IGR) controller
;
;   Copyright (C) 2015 by Peter Bartmann <borti4938@gmx.de>
;
; -----------------------------------------------------------------------
;
;   This program is designed to run on a PIC 16F630 microcontroller connected
;   to the controller port and NES main board. It allows an NES to be reset
;   via a standard controller.
;
;   pin configuration: (controller port pin) [Mainboard pin/pad]
;
;                                         ,----_-----.
;                    +5V (7) [CIC Pin 16] |1        8| GND (1) [CIC Pin 15]
;   Reset - out (-) [CPU Pin 3/CIC Pin 7] |2  A5 A0 7| serial data in (4) [U7 74HC368 Pin 2]
;                                    n.c. |3  A4 A1 6| latch in (3) [CPU Pin 39]
;                 Resettype -  in (*) [*] |4  A3 A2 5| clk in (2) [CPU Pin 36]
;                                         `----------'
;
;   As the internal oscillator is used, you should connect a capacitor of about 100nF between
;   Pin 1 (Vdd/+5V) and Pin 8 (Vss/GND) as close as possible to the PIC. This esures best
;   operation.
;
;   * Resettype:
;     - Pin 4 set tide to Vcc = low-active reset
;       (e.g. Famicom -> Pin 2 is connected to CPU Pin 3)
;     - Pin 4 set tide to GND = high-active reset
;       (e.g. US-NES, consoles with CIC -> Pin 2 is connected to CIC Pin 7)
;
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
;   key mapping: Start + Select +                            (stream data)
;   =============================
;   A + B       Reset                                           0x0f
;   (nothing)   Pressed for ~2s -> perform Longreset (3s)       0xcf
;
; -----------------------------------------------------------------------

; -----------------------------------------------------------------------
; Configuration bits: adapt to your setup and needs

    __CONFIG _INTRC_OSC_NOCLKOUT & _IESO_OFF & _WDT_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOD_OFF

; -----------------------------------------------------------------------
; macros and definitions

M_movff macro   fromReg, toReg  ; move filereg to filereg
        movfw   fromReg
        movwf   toReg
        endm

M_movpf macro   fromPORT, toReg ; move PORTx to filereg
        movfw   fromPORT
        andlw   0x3f
        movwf   toReg
        endm

M_movlf macro   literal, toReg  ; move literal to filereg
        movlw   literal
        movwf   toReg
        endm

M_beff  macro   compReg1, compReg2, branch  ; branch if two fileregs are equal
        movfw   compReg1
        xorwf   compReg2, w
        btfsc   STATUS, Z
        goto    branch
        endm

M_bepf  macro   compPORT, compReg, branch   ; brach if PORTx equals compReg (ignoring bit 6 and 7)
        movfw   compPORT
        xorwf   compReg, w
        andlw   0x3f
        btfsc   STATUS, Z
        goto    branch
        endm

M_belf  macro   literal, compReg, branch  ; branch if a literal is stored in filereg
        movlw   literal
        xorwf   compReg, w
        btfsc   STATUS, Z
        goto    branch
        endm

M_celf  macro   literal, compReg, call_func  ; call if a literal is stored in filereg
        movlw   literal
        xorwf   compReg, w
        btfsc   STATUS, Z
        call    call_func
        endm

M_delay_x05ms   macro   literal ; delay about literal x 05ms
                movlw   literal
                movwf   reg_repetition_cnt
                call    delay_x05ms
                endm

; -----------------------------------------------------------------------

CTRL_DATA   EQU 0
CTRL_LATCH  EQU 1
CTRL_CLK    EQU 2
RESET_TYPE  EQU 3
N_C         EQU 4
RESET_OUT   EQU 5

reg_ctrl_data       EQU 0x41
reg_ctrl_read_ready EQU 0x42
reg_overflow_cnt    EQU 0x43
reg_repetition_cnt  EQU 0x44
reg_current_mode    EQU 0x50
reg_previous_mode   EQU 0x51
reg_reset_type      EQU 0x60
reg_ctrl_reset      EQU 0x61

bit_reset_type          EQU RESET_OUT
code_reset_low_active   EQU (0<<bit_reset_type)  ; 0x00
code_reset_high_active  EQU (1<<bit_reset_type)  ; 0x20

bit_ctrl_reset_perform_long EQU 5
bit_ctrl_reset_flag         EQU 7

delay_05ms_t0_overflows EQU 0x14    ; prescaler T0 set to 1:2 @ 8MHz
repetitions_045ms       EQU 0x09
repetitions_200ms       EQU 0x28
repetitions_300ms       EQU 0x3c
repetitions_1000ms      EQU 0xc8

; -----------------------------------------------------------------------
; buttons

BUTTON_A    EQU 7
BUTTON_B    EQU 6
BUTTON_SL   EQU 5
BUTTON_ST   EQU 4
DPAD_UP     EQU 3
DPAD_DW     EQU 2
DPAD_LE     EQU 1
DPAD_RI     EQU 0

; -----------------------------------------------------------------------

; code memory
 org    0x0000
    clrf    STATUS      ; 00h Page 0, Bank 0
    nop                 ; 01h
    nop                 ; 02h
    goto    start       ; 03h begin program / Initializing


; --------ISR--------
 org    0x0004  ; jump here on interrupt with GIE set
read_Button_A   ; button A can be read (nearly) immediately
    M_movlf ((1<<INTE)^(1<<RAIE)), INTCON
    nop
    nop
    btfsc   GPIO, CTRL_DATA
    bsf     reg_ctrl_data, BUTTON_A
postwait_Button_A
    btfss   INTCON, INTF
    goto    postwait_Button_A

prewait_Button_B
    btfss   GPIO, CTRL_CLK
    goto    prewait_Button_B
    bcf     INTCON, INTF
    bcf     INTCON, RAIF      ; from now on, no IOC at the data latch shall appear
store_Button_B
    btfsc   GPIO, CTRL_DATA
    bsf     reg_ctrl_data, BUTTON_B
postwait_Button_B
    btfss   INTCON, INTF
    goto    postwait_Button_B

prewait_Button_Sl
    btfss   GPIO, CTRL_CLK
    goto    prewait_Button_Sl
    bcf     INTCON, INTF
    nop
store_Button_Sl
    btfsc   GPIO, CTRL_DATA
    bsf     reg_ctrl_data, BUTTON_SL
postwait_Button_Sl
    btfss   INTCON, INTF
    goto    postwait_Button_Sl

prewait_Button_St
    btfss   GPIO, CTRL_CLK
    goto    prewait_Button_St
    bcf     INTCON, INTF
    nop
store_Button_St
    btfsc   GPIO, CTRL_DATA
    bsf     reg_ctrl_data, BUTTON_ST
postwait_Button_St
    btfss   INTCON, INTF
    goto    postwait_Button_St

prewait_DPad_Up
    btfss   GPIO, CTRL_CLK
    goto    prewait_DPad_Up
    bcf     INTCON, INTF
    nop
store_DPad_Up
    btfsc   GPIO, CTRL_DATA
    bsf     reg_ctrl_data, DPAD_UP
postwait_DPad_Up
    btfss   INTCON, INTF
    goto    postwait_DPad_Up

prewait_DPad_Dw
    btfss   GPIO, CTRL_CLK
    goto    prewait_DPad_Dw
    bcf     INTCON, INTF
    nop
store_Button_DW
    btfsc   GPIO, CTRL_DATA
    bsf     reg_ctrl_data, DPAD_DW
postwait_DPad_Dw
    btfss   INTCON, INTF
    goto    postwait_DPad_Dw

prewait_DPad_Le
    btfss   GPIO, CTRL_CLK
    goto    prewait_DPad_Le
    bcf     INTCON, INTF
    nop
store_DPad_Le
    btfsc   GPIO, CTRL_DATA
    bsf     reg_ctrl_data, DPAD_LE
postwait_DPad_Le
    btfss   INTCON, INTF
    goto    postwait_DPad_Le

prewait_DPad_Ri
    btfss   GPIO, CTRL_CLK
    goto    prewait_DPad_Ri
    bcf     INTCON, INTF
    nop
store_DPad_Ri
    btfsc   GPIO, CTRL_DATA
    bsf     reg_ctrl_data, DPAD_RI
postwait_DPad_Ri
    btfss   INTCON, INTF
    goto    postwait_DPad_Ri


check_controller_read
    btfsc   INTCON, RAIF            ; another IOC on data latch appeared
    goto    invalid_controller_read ; -> if yes, invalid read
    bsf	    reg_ctrl_read_ready, 0  ; -> if no, indicate a valid read is stored
    return                          ; return with GIE still unset (GIE set again on demand)

invalid_controller_read
    clrf    INTCON
    clrf    reg_ctrl_data
    clrf    reg_ctrl_read_ready
    bsf	    INTCON, RAIE
    retfie                      ; return with GIE set


; --------IDLE loops--------
idle_prepare
    clrf    INTCON
    clrf    reg_ctrl_data
    clrf    reg_ctrl_read_ready

    btfsc   GPIO, CTRL_LATCH
    call    read_Button_A                 ; go go go
    M_movlf ((1<<GIE)^(1<<RAIE)), INTCON  ; set GIE, only react on RAIF

idle_loop
    btfsc   reg_ctrl_read_ready, 0
    goto    checkkeys
    goto    idle_loop


; --------controller routines--------
checkkeys
    clrf    INTCON
    M_belf  0x0f, reg_ctrl_data, ctrl_reset     ; Start+Select+A+B
    btfsc   reg_ctrl_reset, bit_ctrl_reset_flag ; Start+Select+A+B previously detected?
    goto    doreset                             ; if yes, perform a reset
    goto    idle_prepare

ctrl_reset
    btfss reg_ctrl_reset, bit_ctrl_reset_flag
    goto  first_ctrl_reset
    btfsc reg_ctrl_reset, bit_ctrl_reset_perform_long
    goto  dolongreset
    M_delay_x05ms   repetitions_045ms
    incf  reg_ctrl_reset, 1
    goto  idle_prepare

first_ctrl_reset
    clrf    reg_ctrl_reset
    bsf     reg_ctrl_reset, bit_ctrl_reset_flag
    M_delay_x05ms   repetitions_045ms
    goto    idle_prepare

doreset
    banksel TRISIO            ; Bank 1
    bcf     TRISIO, RESET_OUT
    banksel GPIO              ; Bank 0
    M_movff reg_reset_type, GPIO
    M_delay_x05ms   repetitions_300ms
    goto    release_reset

dolongreset
    banksel TRISIO            ; Bank 1
    bcf     TRISIO, RESET_OUT
    banksel GPIO              ; Bank 0
    M_movff reg_reset_type, GPIO
    M_delay_x05ms   repetitions_1000ms
    M_delay_x05ms   repetitions_1000ms
    M_delay_x05ms   repetitions_1000ms

release_reset
    movfw   reg_reset_type
    xorlw   (1<<RESET_OUT)    ; invert to release
    movwf   GPIO
    banksel TRISIO            ; Bank 1
    bsf     TRISIO, RESET_OUT
    banksel GPIO              ; Bank 0
    clrf    reg_ctrl_reset
    goto    idle_prepare


; --------delay calls--------
delay_05ms
    clrf    TMR0                ; start timer (operation clears prescaler of T0)
    banksel TRISIO
    movfw   OPTION_REG
    andlw   0xf0
    movwf   OPTION_REG
    banksel GPIO
    M_movlf delay_05ms_t0_overflows, reg_overflow_cnt
    M_movlf (1<<T0IE), INTCON   ; enable timer interrupt

delay_05ms_loop_pre
    bcf     INTCON, T0IF

delay_05ms_loop
    btfss   INTCON, T0IF
    goto    delay_05ms_loop
    decfsz  reg_overflow_cnt, 1
    goto    delay_05ms_loop_pre
    clrf    INTCON              ; disable timer interrupt
    return

delay_x05ms
    call    delay_05ms
    decfsz  reg_repetition_cnt, 1
    goto    delay_x05ms
    return


; --------initialization--------
start
    clrf    GPIO
    M_movlf 0x07, CMCON0        ; GPIO2..0 are digital I/O (not connected to comparator)
    clrf    INTCON              ; INTCON set on demand during program
    banksel TRISIO
    M_movlf 0x70, OSCCON        ; use 8MHz internal clock (internal clock set on config)
    M_movlf 0x3f, TRISIO        ; in in in in in in
    M_movlf (1<<N_C), WPUDA     ; pullup at unused pin
    M_movlf 0x02, IOCA          ; IOC on DATA_LATCH
    clrf    OPTION_REG          ; global pullup enable, use falling edge for interrupt at GPIO2, prescaler T0 1:2
    banksel GPIO

detect_reset_type
    clrf    reg_reset_type
    movlw   code_reset_high_active
    btfss   GPIO, RESET_TYPE         ; jump next instruction for low-active reset
    movwf   reg_reset_type

init_end
    clrf    reg_ctrl_reset  ; clear this reg here just in case
    goto    idle_prepare

; -----------------------------------------------------------------------
theend
    END
; ------------------------------------------------------------------------
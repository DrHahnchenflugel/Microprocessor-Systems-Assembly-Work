;*****************************************************************
;* This program demonstrates the hardware timer
;* on the eebot
;*****************************************************************
; export symbols
            XDEF Entry            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point

; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

; variable/data section

            ORG   $3850
 ; Insert here your data definition.


; code section  
            ORG   $4000            

;***********************************************************
;* Timer Alams *
;************************************************************
;definitions
OneSec        EQU     23 ; 1 second delay (at 23Hz)
TwoSec        EQU     46 ; 2 second delay (at 23Hz)
LCD_DAT       EQU     PORTB ; LCD data port, bits - PB7,...,PB0
LCD_CNTR      EQU     PTJ ; LCD control port, bits - PJ7(E),PJ6(RS)
LCD_E         EQU     $80 ; LCD E-signal pin
LCD_RS        EQU     $40 ; LCD RS-signal pin

;variable/data section
        ORG           $3850     ; Where our TOF counter register lives
TOF_COUNTER   RMB     1 ; The timer, incremented at 23Hz
AT_DEMO       RMB     1 ; The alarm time for this demo

;code section
        ORG $4000 ; Where the code starts

Entry:
_Startup:
        LDS     #$4000 ; initialize the stack pointer
        JSR     initLCD ; initialize the LCD
        JSR     clrLCD ; clear LCD & home cursor
        JSR     ENABLE_TOF ; Jump to TOF initialization
        CLI           ; Enable global interrupt
        LDAA    #'A' ; Display A (for 1 sec)
        JSR     putcLCD ; --"--
        LDAA    TOF_COUNTER ; Initialize the alarm time
        ADDA    #OneSec ; by adding on the 1 sec delay
        STAA    AT_DEMO ; and save it in the alarm
        
CHK_DELAY_1 LDAA TOF_COUNTER ; If the current time
        CMPA    AT_DEMO ; equals the alarm time
        BEQ     A1 ; then display B
        BRA     CHK_DELAY_1 ; and check the alarm again
     
A1      LDAA    #'B' ; Display B (for 2 sec)
        JSR     putcLCD ; --"--
        LDAA    AT_DEMO ; Initialize the alarm time
        ADDA    #TwoSec ; by adding on the 2 sec delay
        STAA    AT_DEMO ; and save it in the alarm
    
CHK_DELAY_2 LDAA TOF_COUNTER ; If the current time
        CMPA    AT_DEMO ; equals the alarm time
        BEQ     A2 ; then display C
        BRA     CHK_DELAY_2 ; and check the alarm again
 
A2      LDAA    #'C' ; Display C (forever)
        JSR     putcLCD ; --"--
        SWI

; Subroutine section
initLCD
        BSET DDRB, %11111111      ;Set Pins PB7..0 for OUTPUT
        BSET DDRJ, %11000000      ;Set Pins PJ7, PJ6 for OUTPUT
        
        LDY  #2000                ;Wait for LCD to be ready
        JSR del_50us            ;delay for command to complete
        
        LDAA #$28                 ;Set 4-bit data, 2-line display
        JSR cmd2LCD               ;Send ACCA -> LCD
        
        LDAA #$0C                 ;Set Display ON, Cursor OFF, Blink OFF
        JSR cmd2LCD               ;Send ACCA -> LCD
        
        LDAA #$06                 ;Move cursor right after entering a character
        JSR cmd2LCD               ;Send ACCA -> LCD
        
        RTS
        
clrLCD
        LDAA #$01                 ;Clear cursor and return to home position
        JSR cmd2LCD               ;Send ACCA -> LCD
        
        LDY #$40                  ;Wait until clear cursor cmd is complete
        JSR del_50us
        
        RTS

del_50us
        PSHX
eloop:  LDX   #30
iloop:  PSHA
        PULA
        
        PSHA
        PULA        
        PSHA
        PULA        
        PSHA
        PULA
        
                
        PSHA
        PULA
        NOP
        NOP
        DBNE X,iloop
        DBNE Y,eloop
        PULX
        RTS

 
cmd2LCD
        BCLR LCD_CNTR, LCD_RS  ;Select LCD Instruction Register (RS)
        JSR dataMov               ;Send LCD Data
        
        RTS
        
putsLCD
        LDAA 1,X+
        BEQ  donePS
        JSR  putcLCD
        BRA  putsLCD

donePS  RTS

putcLCD
        BSET LCD_CNTR, LCD_RS
        JSR  dataMov
        
        RTS


dataMov
        BSET LCD_CNTR, LCD_E  ;Set the LCD E signal to HI
        STAA LCD_DAT             ;Send the upper 4 bits of data to LCD
        BCLR LCD_CNTR, LCD_E  ;Set the LCD E signal to LO to complete the write oper
        

        
        LDY #1                     ;Delay, adding this completes the oper
        JSR del_50us
        
        RTS
  
ENABLE_TOF 
        LDAA #%10000000
        STAA TSCR1 ; Enable TCNT
        STAA TFLG2 ; Clear TOF
        LDAA #%10000100 ; Enable TOI and select prescale factor equal to 16
        STAA TSCR2
        RTS
;************************************************************

TOF_ISR 
        INC TOF_COUNTER
        LDAA #%10000000 ; Clear
        STAA TFLG2 ; TOF
        RTI
        
;************************************************************

DISABLE_TOF 
        LDAA #%00000100 ; Disable TOI and leave prescale factor at 16
        STAA TSCR2
        RTS
        
;************************************************************
;* Interrupt Vectors *
;************************************************************

        ORG $FFFE
        DC.W Entry ; Reset Vector

        ORG $FFDE
        DC.W TOF_ISR ; Timer Overflow Interrupt Vector
;*****************************************************************
;* This program demonstrates the use of the eebot's features:    
;* - Reading from the Bumpers
;* - Reading from the potentiometer
;* - Writing to the LCD
;* - Using the ADC to convert the potentiometer signal to
;    a value usable by the digital controller
;*****************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point

; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

; variable/data section
            ORG   $3850
; Definitions
LCD_DATA    EQU   PORTB           ;LCD DATA port, bits - PB7..PB0
LCD_CONTROL EQU   PTJ             ;LCD CONTROL port, bits - PE7 (RS), PE4(E)
LCD_E       EQU   $80             ;LCD E-signal port on $80
LCD_RS      EQU   $40             ;LCD RS-signal port on $40

; used in INT2BCD 
TEN_THOUS         RMB       1            ;10,000 digit
THOUSANDS         RMB       1            ;1,000 digit
HUNDREDS          RMB       1            ;100 digit
TENS              RMB       1            ;10 digit
UNITS             RMB       1            ;1 digit
BCD_SPARE         RMB       10           ;Extra space for decimal point, string terminator
NO_BLANK          RMB       1            ;Used in 'leading zero' blanking by BCD2ASC

; code section  
            ORG   $4000            
Entry:
_Startup:
            LDS   #$4000               ; initialize the stack pointer
            JSR   initADC              ; initialize ADC
            JSR   initLCD              ; initialize LCD
            
            JSR   clrLCD               ; clear LCD and home cursor
            
            LDX   #msg1                ; load msg1 to ACCX
            JSR   putsLCD              ; send msg1 to LCD
            
            LDAA  #$C0                 ; move LCD cursor to the second row
            JSR   cmd2LCD              ; send cmd to LCD
            
            LDX   #msg2                ; load msg2 to ACCX
            JSR   putsLCD              ; send msg1 to LCD

lbl         
            MOVB  #$90,ATDCTL5         ; r.just., unsign., sing.conv., mult., cl0, start conv.
            BRCLR ATDSTAT0,$80,*       ; wait until the conversion sequence is complete
            
            LDAA  ATDDR4L              ; Load the ch4 result into ACCA
            LDAB  #30                  ; ACCB = 39
            MUL                        ; ACCD = 1st result x 39
            ADDD  #600               ; ACCD = 1st result x 30 + 600
            
            JSR   int2BCD
            JSR   BCD2ASC
            
            LDAA  #$8F                 ; Move LCD cursor to first row, end of msg1
            JSR   cmd2LCD              ; send cmd to LCD
            
            LDAA  TEN_THOUS            ; Output the TEN_THOUS ASCII character
            JSR   putcLCD
            
            LDAA  THOUSANDS            ; Output the THOUSANDS ASCII character
            JSR   putcLCD
            
            LDAA  #'.'
            JSR   putcLCD
                          
            LDAA  HUNDREDS            ; Output the HUNDREDS ASCII character
            JSR   putcLCD
            
            LDAA  #$CF                    ; Move LCD cursor to the 2nd row, end of msg2
            JSR   cmd2LCD
            
            BRCLR  PORTAD0,%00000100,bowON
            LDAA    #' '               ; Output ' ' if bow switch OFF
            BRA   bowOFF
bowON       LDAA    #'B'               ; Output 'B' if bow switch ON
bowOFF      JSR     putcLCD            ; Send bow state to LCD

            LDAA  #'|'
            JSR   putcLCD              ; ... output a vertical line character in ASCII
            
              BRCLR  PORTAD0,%00001000,sternON
              LDAA    #' '               ; Output ' ' if stern switch OFF
              BRA   sternOFF
sternON       LDAA    #'S'               ; Output 'S' if stern switch ON
sternOFF      JSR     putcLCD            ; Send stern state to LCD
              JMP     lbl

msg1        dc.b  "Battery volt ",0
msg2        dc.b  "Bumper status ",0


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
        BCLR LCD_CONTROL, LCD_RS  ;Select LCD Instruction Register (RS)
        JSR dataMov               ;Send LCD Data
        
        RTS
        
putsLCD
        LDAA 1,X+
        BEQ  donePS
        JSR  putcLCD
        BRA  putsLCD

donePS  RTS

putcLCD
        BSET LCD_CONTROL, LCD_RS
        JSR  dataMov
        
        RTS


dataMov
        BSET LCD_CONTROL, LCD_E  ;Set the LCD E signal to HI
        STAA LCD_DATA             ;Send the upper 4 bits of data to LCD
        BCLR LCD_CONTROL, LCD_E  ;Set the LCD E signal to LO to complete the write oper
        
        ; Working in 4 bits, shift A reg 4 bits left to send them
        LSLA
        LSLA
        LSLA
        LSLA
  
        BSET LCD_CONTROL, LCD_E   ;Set the LCD E signal to HI
        STAA LCD_DATA              ;Send the lower 4 bits of data to the LCD
        BCLR LCD_CONTROL, LCD_E   ;Set the LCD E signal to LO to complete the write oper
        
        LDY #1                     ;Delay, adding this completes the oper
        JSR del_50us
        
        RTS
  
;int2BCD ...
;BCD2ASC ...
initADC 
        MOVB #$C0,ATDCTL2 ;power up ADC, select fast flag clear
        JSR del_50us ;wait for 50 us
        MOVB #$00,ATDCTL3 ;8 conversions in a sequence
        MOVB #$85,ATDCTL4 ;res=8, conv-clks=2, prescal=12
        BSET ATDDIEN,$0C ;configure pins AN03,AN02 as digital inputs
        RTS


;**************************************************************
;*                 Routines                                   *
;**************************************************************

;* file ref b16todec.asm
;*
;***********************************************************************
;*****************************************************************
;* Integer to BCD Conversion Routine
;* This routine converts a 16 bit binary number in .D into
;* BCD digits in BCD_BUFFER.
;* Peter Hiscocks
;* Algorithm:
;* Because the IDIV (Integer Division) instruction is available on
;* the HCS12, we can determine the decimal digits by repeatedly
;* dividing the binary number by ten: the remainder each time is
;* a decimal digit. Conceptually, what we are doing is shifting
;* the decimal number one place to the right past the decimal
;* point with each divide operation. The remainder must be
;* a decimal digit between 0 and 9, because we divided by 10.
;* The algorithm terminates when the quotient has become zero.
;* Bug note: XGDX does not set any condition codes, so test for
;* quotient zero must be done explicitly with CPX.
;* Data structure:
;* BCD_BUFFER EQU * The following registers are the BCD buffer area
;* TEN_THOUS RMB 1 10,000 digit, max size for 16 bit binary
;* THOUSANDS RMB 1 1,000 digit
;* HUNDREDS RMB 1 100 digit
;* TENS RMB 1 10 digit
;* UNITS RMB 1 1 digit
;* BCD_SPARE RMB 2 Extra space for decimal point and string terminator

int2BCD       XGDX                      ;Save the binary number into .X
              LDAA        #0            ;Clear the BCD_BUFFER
              STAA        TEN_THOUS
              STAA        THOUSANDS
              STAA        HUNDREDS
              STAA        TENS
              STAA        UNITS
              STAA        BCD_SPARE
              STAA        BCD_SPARE+1
;*
              CPX         #0            ;Check for a zero input
              BEQ         CON_EXIT      ;and if so, exit
;*
              XGDX                      ;Not zero, get the binary number back to .D as dividend
              LDX         #10           ;Setup 10 (Decimal!) as the divisor
              IDIV                      ;Divide: Quotient is now in .X, remainder in .D
              STAB        UNITS         ;Store remainder
              CPX         #0            ;If quotient is zero,
              BEQ         CON_EXIT      ;then exit
;*
              XGDX                      ;else swap first quotient back into .D
              LDX         #10           ;and setup for another divide by 10
              IDIV
              STAB        TENS
              CPX         #0
              BEQ         CON_EXIT
;*
              XGDX                      ;Swap quotient back into .D
              LDX         #10           ;and setup for another divide by 10
              IDIV
              STAB        HUNDREDS
              CPX         #0
              BEQ         CON_EXIT
;*
              XGDX                      ;Swap quotient back into .D
              LDX         #10           ;and setup for another divide by 10
              IDIV
              STAB        THOUSANDS
              CPX         #0
              BEQ         CON_EXIT
;*
              XGDX                      ;Swap quotient back into .D
              LDX         #10           ;and setup for another divide by 10
              IDIV
              STAB        TEN_THOUS
;*

CON_EXIT      RTS                       ;We’re done the conversion

;* file ref: bcdtoasc.asm
;*
;***********************************************************************
;****************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag ’NO_BLANK’ starts cleared and is set once a non-zero
;* digit has been detected.
;* The ’units’ digit is never blanked, even if it and all the
;* preceding digits are zero.
;* Peter Hiscocks
BCD2ASC       LDAA        #0            ;Initialize the blanking flag
              STAA        NO_BLANK      
;*
C_TTHOU       LDAA        TEN_THOUS     ;Check the ’ten_thousands’ digit
              ORAA        NO_BLANK
              BNE         NOT_BLANK1
;*
ISBLANK1      LDAA        #' '            ;It’s blank
              STAA        TEN_THOUS       ;so store a space
              BRA         C_THOU          ;and check the ’thousands’ digit
;*
NOT_BLANK1    LDAA        TEN_THOUS       ;Get the ’ten_thousands’ digit
              ORAA        #$30            ;Convert to ascii
              STAA        TEN_THOUS
              LDAA        #$1             ;Signal that we have seen a ’non-blank’ digit
              STAA        NO_BLANK
;*
C_THOU        LDAA        THOUSANDS       ;Check the thousands digit for blankness
              ORAA        NO_BLANK        ;If it’s blank and ’no-blank’ is still zero
              BNE         NOT_BLANK2
;*
ISBLANK2      LDAA        #' '            ;Thousands digit is blank
              STAA        THOUSANDS       ;so store a space
              BRA         C_HUNS          ;and; check the hundreds digit
;*
NOT_BLANK2    LDAA        THOUSANDS       ;(similar to ’ten_thousands’ case)
              ORAA        #$30
              STAA        THOUSANDS
              LDAA        #$1
              STAA        NO_BLANK
;*
C_HUNS        LDAA        HUNDREDS        ;Check the hundreds digit for blankness
              ORAA        NO_BLANK        ;If it’s blank and ’no-blank’ is still zero
              BNE         NOT_BLANK3
;*
ISBLANK3      LDAA        #' '            ;Hundreds digit is blank
              STAA        HUNDREDS        ;so store a space
              BRA         C_TENS          ;and check the tens digit
;*
NOT_BLANK3    LDAA        HUNDREDS        ;(similar to ’ten_thousands’ case)
              ORAA        #$30
              STAA        HUNDREDS
              LDAA        #$1
              STAA        NO_BLANK
;*
C_TENS        LDAA        TENS            ;Check the tens digit for blankness
              ORAA        NO_BLANK        ;If it’s blank and ’no-blank’ is still zero
              BNE         NOT_BLANK4
;*
ISBLANK4      LDAA        #' '            ;Tens digit is blank
              STAA        TENS            ;so store a space
              BRA         C_UNITS         ;and check the units digit
;*
NOT_BLANK4    LDAA        TENS            ;(similar to ’ten_thousands’ case)
              ORAA        #$30
              STAA        TENS
;*
C_UNITS       LDAA        UNITS           ;No blank check necessary, convert to ascii.
              ORAA        #$30
              STAA        UNITS
;*
              RTS                         ;We’re done

;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector

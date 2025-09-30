;*****************************************************************
;* This program writes a NULL-terminated string (msg) to an LCD. *
;*                                                               *
;* Intended to be used with the HCS12D's MC9S12DG128B derivative *
;* and the EvalH1 Trainer Board.                                 *
;*                                                               *
;* Victor Lelikov                                                *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

ROMStart    EQU  $4000  ; absolute address to place my code/constant data

; variable/data section

            ORG RAMStart
LCD_DATA     EQU   PTS    ; LCD data port S, pins PS7..4
LCD_CONTROL  EQU   PORTE  ; LCD control port E. Pins PE7 (RS), PE4 (EN)
LCD_EN       EQU   $10    ; LCD enable signal, pin PE4
LCD_RS       EQU   $80    ; LCD registry select signal, pin PE7

; code section
            ORG   ROMStart
            


Entry:
; Setup code
_Startup:
            LDS   #RAMEnd+1       ; initialize the stack pointer

            CLI                   
            
            JSR   lcdInit         ; initialise 

msg         dc.b "Hello World",0  ; Set message to "Hello World"


MainLoop  
            JSR   lcdClear        ; clear LCD
            
            LDX   #msg             ; Load msg to ACCX
            JSR   lcdWriteString  ; Write String @ ACCX -> LCD 
            
            LDAA  $3000           ; Load contents at 3000 -> ACCA  
            JSR   leftHLF         ; convert left half of A into ASCII
            STAA  $6000           ; Store the ascii byte into mem1 ($6000)
            
            LDAA  $3000           ; Load contents at 3000 -> ACCA  
            JSR   rightHLF         ; convert right half of A into ASCII
            STAA  $6001           ; Store the ascii byte into mem2 ($6001)
            
            LDAA  $3001           ; Load contents at 3001 -> ACCA  
            JSR   leftHLF         ; convert left half of A into ASCII
            STAA  $6002           ; Store the ascii byte into mem3 ($6002)
            
            LDAA  $3001           ; Load contents at 3001 -> ACCA  
            JSR   rightHLF        ; convert right half of A into ASCII
            STAA  $6003           ; Store the ascii byte into mem4 ($6003)
            
            LDAA  #$00            ; Load 0000 into A (NULL char)
            STAA  $6004           ; Store string termination character 00 into mem5 ($6004)
            
            LDX   #$6000          ; Load the 4 ASCII characters 
            JSR   lcdWriteString  ; Output the 4 ASCII characters
            
            LDY   #$0002          ; Delay = 1s
            JSR   delay_50us
            
            BRA   MainLoop        ; Force branch to MainLoop      
            
;**************************************************************
;*                       Subroutines                          *
;**************************************************************

; ********************** General Purpose **********************
; ********************* Internal Commands *********************
; delay_50us - delay for {Y} x 50us. E-clk - 41,67ns.
delay_50us:   
            PSHX                    ;2 E-clk
eloop:      LDX   #30               ;2 E-clk -
iloop:      PSHA                    ;2 E-clk |
            PULA                    ;3 E-clk |
            NOP                     ;1 E-clk |
            NOP                     ;1 E-clk |
            NOP                     ;1 E-clk |
            NOP                     ;1 E-clk |
            NOP                     ;1 E-clk |
            NOP                     ;1 E-clk |
            NOP                     ;1 E-clk |
            NOP                     ;1 E-clk |
            PSHA                    ;2 E-clk | 50us
            PULA                    ;3 E-clk |
            NOP                     ;1 E-clk |
            NOP                     ;1 E-clk |
            DBNE  X,iloop           ;3 E-clk -
            DBNE  Y,eloop           ;3 E-clk
            PULX                    ;3 E-clk
            
            RTS                     ;5 E-clk, Return

; ************************ BIN_2_ASCII ************************            
; leftHLF 
leftHLF     
            LSRA                    ; shift data to right
            LSRA
            LSRA
            LSRA

; rightHLF  
rightHLF 
            ANDA  #$0F ; mask top half
            ADDA  #$30 ; convert to ascii
            CMPA  #$39
            BLE   out ; jump if 0-9
            ADDA  #$07 ; convert to hex A-F
out   RTS            

; **************************** LCD ****************************
; ********************* Internal Commands *********************
; sendData - Send data stored in ACCA to the LCD Instruction or Data Reg, depending on RS
sendData
          BSET  LCD_CONTROL, LCD_EN ; Set the LCD ENABLE signal to HI
          STAA  LCD_DATA            ; Send the upper 4 bits of data to LCD (ACCA[0..4] -> LCD_DATA)
          BCLR  LCD_CONTROL, LCD_EN ; Set the LCD ENABLE signal to LO to complete the write oper
          
          ; Shift the A reg 4 bits left to send lower 4 bits
          LSLA                      ; Match the lower 4 bits with the LCD data pins 
          LSLA                      ; Match the lower 4 bits with the LCD data pins
          LSLA                      ; Match the lower 4 bits with the LCD data pins
          LSLA                      ; Match the lower 4 bits with the LCD data pins
          
          BSET  LCD_CONTROL, LCD_EN ; Set the LCD ENABLE signal to HI
          STAA  LCD_DATA            ; Send the lower 4 bits of data to the LCD (ACCA[0..4] -> LCD_DATA)
          BCLR  LCD_CONTROL, LCD_EN ; Set the LCD ENABLE signal to LO to complete the write oper
          
          LDY   #1                  ; Delay, adding this completes the internal oper. for most instructions
          JSR   delay_50us
          
          RTS                       ; Return
          

; sendToLCD - Send command stored in ACCA to the LCD
sendToLCD
          BCLR LCD_CONTROL, LCD_RS ; Select LCD Instruction Registor (IR)
          JSR sendData              ; Send LCD Data
          
          RTS                       ; Return

                          
; **************************** LCD ****************************
; ****************** External (API) Commands ******************
; lcdInit - Initialise the LCD
; 4-bit data width, 2 line display, 
; Turn backlight on, cursor and blinking off
; Shift cursor right.

lcdInit
          BSET DDRS, %11110000      ; Set Pins PS7..4 for OUTPUT
          BSET DDRE, %10010000      ; Set Pins PE7, PE4 for OUTPUT
          
          LDY #2000                 ; wait for LCD to be ready
          JSR delay_50us            ; delay 50 us, for command to complete
          
          LDAA #$28                 ; Set 4-bit data, 2-line display
          JSR sendToLCD             ; Send ACCA -> LCD
          
          LDAA #$0C                 ; Display ON, Cursor OFF, Blink OFF
          JSR sendToLCD             ; Send ACCA -> LCD
          
          LDAA #$06                 ; Move cursor RIGHT after entering a character
          JSR sendToLCD             ; Send ACCA -> LCD

; lcdClear - Clear display and home cursor
lcdClear
          LDAA #$01                 ; Clear cursor and return to home position      
          JSR sendToLCD             ; Send ACCA -> LCD
          
          LDY  #$40                 ; Wait until "clear cursor" cmd is complete
          JSR delay_50us
          
          RTS                       ; Return
 
; lcdWriteString - Write a NULL-terminated string pointed to by X to LCD
lcdWriteString
          LDAA 1,X+                 ; Get 1 character from string
          BEQ  donePS               ; Break to donePS if at NULL character (ACCA = 0)
          JSR  lcdWriteChar         ; Else, Write character to LCD
          BRA  lcdWriteString       ; Recursively call self on remaining string
          
donePS    RTS                       ; Return

; lcdWriteChar - Write a char from ACCA -> LCD
lcdWriteChar
          BSET  LCD_CONTROL, LCD_RS; Select the LCD Data Register (DR)
          JSR   sendData            ; Send data to Data Register (DR)
          
          RTS                       ; Return

; INTERRUPT VECTORS
        ORG   $FFFE
        DC.W   Entry

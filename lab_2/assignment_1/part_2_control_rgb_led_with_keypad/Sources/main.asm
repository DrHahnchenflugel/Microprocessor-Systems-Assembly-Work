;*****************************************************************
;* This program reads the 3 Least Sig. Bits obtained from the    *
;* keypad, and uses their values (0-255) to output to the RGB LED*
;*                                                               *
;* Intended to be used with the HCS12D's MC9S12DG128B derivative *
;* and the EvalH1 Trainer Board.                                 *
;*                                                               *
;* Original code: Peter Hiscocks Adapted for lab: Victor Lelikov *
;*****************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

ROMStart    EQU  $4000  ; absolute address to place my code/constant data

; variable/data section

            ORG RAMStart


; code section
            ORG   ROMStart


Entry:
; Setup code
_Startup:
            LDS   #RAMEnd+1       ; initialize the stack pointer

            CLI                   ; enable interrupts
                                  
                                  ; %ABCDEFGH = ABCDEFGH_2 (binary)
            BSET  DDRP, %11111111       ; Configure port P for output (R,G,B controlled by PP0, PP1, PP2, respectively)
            BSET  DDRE, %00010000       ; Configure pin PE4 for output (Data Direction of PE4 = OUTPUT)
            BCLR  PORTE, %00010000      ; Clear PE4 -> PE4 = 0. This sets the keypad to be enabled (OE = active LO)
            
Loop:
            LDAA  PTS                   ; Read a key code into ACCA
            
            ; Shift the ACCA 4 bits right. This comes from us taking the 4 upper bits of the PTS.
            LSRA                        ; Shift ACCA right
            LSRA                        ; Shift ACCA right
            LSRA                        ; Shift ACCA right
            LSRA                        ; Shift ACCA right
            STAA  PTP                   ; [val of] ACCA -> PTP (Output ACCA -> LED2)
            BRA   Loop                  ; Force branch back to Loop
            
            

;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector

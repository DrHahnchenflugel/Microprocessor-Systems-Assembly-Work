;*****************************************************************
;* This program drives a buzzer with a frequency of ~726Hz.      *
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
                                  
            BSET  DDRP,%11111111  ; Set port P to OUTPUT
            LDAA  #%10000000      ; %1000 0000 -> ACCA (Set to drive PP7 [buzzer])

; Loop Code
MainLoop:
            STAA  PTP             ; ACCA -> PTP (Drive PP7 [buzzer])
            LDX   #$1FFF          ; Loop counter (ACCX) = 1FFF_16 = 8191_10

Delay:
            DEX                   ; Decrement the loop counter (ACCX)
            BNE   Delay           ; Loop to Delay until loop counter (ACCX) equals 0 
            ; This delay takes 4 E-CLK cycles per loop. 4*8191 loops * 42ns/loop = 1376088 ns.
            
            EORA  #%10000000      ; XOR MSB of ACCA with 1, inverting it.
            BRA   MainLoop        ; Force branch back to MainLoop
            
            

;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector

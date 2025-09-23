;*****************************************************************
;* This program obtains a value from the SW1 Dip Switch bar and  *
;* displays the values of the switches on the LED1 LED bar.      *
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
                                  
            LDAA #$FF             ; ACCA = $FF_16 (255_10)  
            STAA DDRH             ; [val of] ACCA -> DDRH. Configure PORT H for output
            STAA PERT             ; [val of] ACCA -> PERT. Configure PULLUP Resistors on Port T to EN

; Loop code
Loop:
            LDAA PTT              ; [val of] PTT -> ACCA. Read value of Port T (SW1, switch bar)
            STAA PTH              ; [val of] ACCA -> PTH. Store value of ACCA to Port H (LED1, LED bar)
            BRA  Loop             ; Force BRANCH back to Loop
            
            

;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector

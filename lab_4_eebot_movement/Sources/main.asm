;*****************************************************************
;* This program demonstrates motor control and hardware timers
;* on the eebot
;*****************************************************************
; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point

; Include derivative-specific definitions 
		INCLUDE 'derivative.inc' 

; variable/data section

            ORG   $3850
 ; Insert here your data definition.


; code section  
            ORG   $4000            
Entry:
_Startup:
            LDS   #$4000               ; initialize the stack pointer
            JSR   MOTORINIT

;************************************************************
;* PT. A)           MotorControl                            *
;************************************************************
MOTORINIT
             BSET DDRA,%00000011
             BSET DDRT,%00110000
             JSR STARFWD
             JSR PORTFWD
             JSR STARON
             JSR PORTON
             JSR STARREV
             JSR PORTREV
             JSR STAROFF
             JSR PORTOFF
             BRA *

STARON 
             LDAA PTT
             ORAA #%00100000
             STAA PTT
             RTS
STAROFF 
             LDAA PTT
             ANDA #%11011111
             STAA PTT
             RTS
 
PORTON 
             LDAA PTT
             ORAA #%00010000
             STAA PTT
             RTS
PORTOFF 
             LDAA PTT
             ANDA #%11101111
             STAA PTT
             RTS

STARFWD 
             LDAA PORTA
             ANDA #%11111101
             STAA PORTA
             RTS
 
STARREV 
             LDAA PORTA
             ORAA #%00000010
             STAA PORTA
             RTS
             
 
PORTFWD 
             LDAA PORTA
             ANDA #%11111110
             STAA PORTA
             RTS
 
PORTREV 
             LDAA PORTA
             ORAA #%00000001
             STAA PORTA
             RTS


;**************************************************************
;*                 Interrupt Vectors                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector

;***************************************************************************************************
;* FINAL PROJECT: EEBOT MAZE RUNNER                                                                *
;* VICTOR LELIKOV                                                                                  *
;* F2025                                                                                           *
;* This program implements a maze-running robot using 6 LDR sensors to follow lines                *
;* and detect intersections, and bumpers to detect collisions.                                     *
;* The robot records its path through the maze and can retrace its steps.                          *
;*                                                                                                 *
;* 4 of the 6 LDRs are used for intersection detection                                             *
;* The other 2 LDRs are used for line following/alignment                                          *
;***************************************************************************************************
              XDEF Entry, _Startup ;
              ABSENTRY Entry ; for absolute assembly: mark
              INCLUDE "derivative.inc"

; MADE FOR ROBOT No.31671
;***************************************************************************************************
;                                         EQUATES SECTION
;***************************************************************************************************

; Liquid Crystal Display Equates
;-------------------------------
CLEAR_HOME    EQU   $01                   ; Clear the display and home the cursor
INTERFACE     EQU   $38                   ; 8 bit interface, two line display
CURSOR_OFF    EQU   $0C                   ; Display on, cursor off
SHIFT_OFF     EQU   $06                   ; Address increments, no character shift
LCD_SEC_LINE  EQU   64                    ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD Addresses
; -------------
LCD_CNTR      EQU   PTJ                   ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT       EQU   PORTB                 ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E         EQU   $80                   ; LCD E-signal pin
LCD_RS        EQU   $40                   ; LCD RS-signal pin

; Other LCD codes
; -------------
NULL          EQU   00                    ; The string ?null terminator?
CR            EQU   $0D                   ; ?Carriage Return? character
SPACE         EQU   ' '                   ; The ?space? character

; Timers
; -------------
T_LEFT        EQU   8                     ; Time variable to store length taken for the larger left turn 
T_RIGHT       EQU   8                     ; Time variable to store length taken for the larger right turn
T_SPEED_ON    EQU   1                     ; TOF ticks for 50% PWM speed control
T_TURN_AROUND EQU   9999                  ; Time variable to turn around 180 degrees (run four times)
T_TURN_90     EQU   9000                  ; Time variable to turn 90 degrees (run twice)
T_FWD_OVER_INTERSECTION EQU 9999          ; Time variable to drive forward over an intersection (run twice)

; States for robot
;-----------------
START         EQU   0
FWD           EQU   1
ALL_STOP      EQU   2
LEFT_TRN      EQU   3                     ; Larger of the left turns
RIGHT_TRN     EQU   4                     ; Larger of the right turns
REV_TRN       EQU   5
LEFT_ALIGN    EQU   6                     ; Smaller left turn for alignment
RIGHT_ALIGN   EQU   7                     ; Smaller right turn for alignment

; Intersections
;----------------
MAX_INTERS    EQU   8                     ; Maximum number of intersections to store

; Direction constants
; The robot always starts facing north
;-----------------
DIR_N         EQU   0   
DIR_E         EQU   1
DIR_S         EQU   2
DIR_W         EQU   3
DIR_NONE      EQU   4

; Drive Modes
; The modes determine how the robot behaves at intersections
; In 'search' mode, the robot explores new paths
; In 'back' mode, the robot goes back to the previous intersection after hitting a wall
; In 'reverse' mode, the robot retraces its path back to the start from the end of the maze
; In 'traceback' mode, the robot goes back to the start after completing the maze, based off the recorded path
;-----------------
MODE_SEARCH   EQU   0
MODE_BACK     EQU   1
MODE_REVERSE  EQU   2
MODE_TRACEBACK EQU  3

;***************************************************************************************************
;                                             DATA SECTION
;***************************************************************************************************
              ORG   $3800

; Base values
; These were calculated by taking the midpoint of the 
; sensor readings on black and white surfaces
;----------------
BASE_LINE     FCB   $85
BASE_BOW      FCB   $A5
BASE_PORT     FCB   $8F
BASE_MID      FCB   $4F
BASE_STBD     FCB   $8E

; Variance values
; These were determined initially by the midpoint of the distance between base and black/white readings
; Then, adjusted for better performance through testing
; A larger variance will make the robot more "sensitive" to changes in sensor readings
;----------------
LINE_VARIANCE           FCB   $27           
BOW_VARIANCE            FCB   $13           
PORT_VARIANCE           FCB   $1E
MID_VARIANCE            FCB   $1A
STARBOARD_VARIANCE      FCB   $1F

; Display values
;----------------
TOP_LINE      FCC   'V:                 '       ; Top line of display
              FCB   NULL                        ; null-terminated

BOT_LINE      FCC   'S:                '        ; Bottom line of display
              FCB   NULL                        ; null-terminated

CLEAR_LINE    FCC   '                  '        ; Clear the line of display
              FCB   NULL                        ; null-terminated

TEMP          RMB   1                           ; Temporary location, used in sensors

; Storage registers
; Initialised to base values.
;----------------
SENSOR_LINE   FCB   $85                     
SENSOR_BOW    FCB   $A5                     
SENSOR_PORT   FCB   $8F
SENSOR_MID    FCB   $4F
SENSOR_STBD   FCB   $8E
SENSOR_NUM    RMB   1                           ; Sensor number being read

;***************************************************************************************************
;                                         VARIABLES SECTION
;***************************************************************************************************
              ORG   $3850                   
; TOF variables
; Used to store values for timed operations
;----------------
TOF_COUNTER   dc.b  0                       ; The timer, incremented at 23Hz
CRNT_STATE    dc.b  0                       ; Current state
T_TURN        ds.b  1                       ; Time to stop turning
T_FWD         ds.b  1                       ; Time to stop driving over intersection
T_SPD_CNTRL   ds.b  1                       ; Time to control drive speed

; Conversion digits
; Used in INT2BCD and BCD2ASC
;----------------
TEN_THOUS     ds.b  1                       ; 10,000 digit
THOUSANDS     ds.b  1                       ; 1,000 digit
HUNDREDS      ds.b  1                       ; 100 digit
TENS          ds.b  1                       ; 10 digit
UNITS         ds.b  1                       ; 1 digit
NO_BLANK      ds.b  1                       ; Used in 'leading zero' blanking by BCD2ASC

; MISC
;----------------
HEX_TABLE     FCC   '0123456789ABCDEF'      ; Table for converting values
BCD_SPARE     RMB   2                       ; Spare BCD storage   
LAST_PWM      dc.b  0                       ; Last time PWM motor control was updated

; Sensor detection states
; Stores whether each sensor is on black (1) or white (0)
; ---------------
; Intersection pattern LDRs
DETECTED_BOW     RMB   1
DETECTED_PORT    RMB   1
DETECTED_STBD    RMB   1
DETECTED_MID     RMB   1
; Line following LDRs
DETECTED_LINE_LEFT  RMB   1
DETECTED_LINE_RIGHT RMB   1

; Intersection variables
;----------------
HEADING       RMB   1                       ; Current heading of the robot (0..3 = N,E,S,W)
DRIVE_MODE    RMB   1                       ; Current drive mode of the robot (search, back, reverse, traceback)
INTERSECT_NUM RMB   1                       ; Current intersection number (0..MAX_INTERS-1). At first intersection, n=1

; Path storage arrays
;----------------
PATH_DIR      RMB   MAX_INTERS              ; Array to store path directions at intersections when going fwd. Only correct exit directions are stored.  
REV_PATH_DIR  RMB   MAX_INTERS              ; Array to store path directions at intersection when robot arrives at each intersection (flipped in reverse mode)
                                            ;     I.e., robot arrives at intersection 1 going North, North is recorded then flipped to South going back.


;***************************************************************************************************
;                                         CODE SECTION
;***************************************************************************************************
              ORG   $4000
; Initialisation Section
;----------------
Entry:
_Startup:

              LDS   #$4000                 ; Initialise the stack pointer
              CLI                          ; Enable interrupts
              
              JSR   INIT                   ; Initialise ports
              JSR   openADC                ; Initialise the ATD - TODO, this might not be needed due to initAD call. This has a different definition. (4 on AN1, certain prescale)
              JSR   initLCD                ; Initialise the LCD
              JSR   CLR_LCD_BUF            ; Write 'space' characters to the LCD buffer
              
              BSET  DDRA,%00000011         ; STAR_DIR, PORT_DIR
              BSET  DDRT,%00110000         ; STAR_SPEED, PORT_SPEED
              
              JSR   initAD                 ; Initialise ATD converter - 8 conversion sequence, different prescale to openADC, AN2/AN3 are digital inputs
              JSR   initLCD                ; Initialise the LCD
              
              JSR   SENSOR_FLAGS_INIT      ; Initialise sensor detected flags to 0

              ; Initialise robot variables to default states
              LDAA #0                   ; All initial values equal 0
              STAA HEADING              ; Initial heading - North
              STAA INTERSECT_NUM        ; Initial intersection number - 0
              STAA DRIVE_MODE           ; Initial drive mode - Search
              
              LDAA  #START              ; Enter START state
              STAA  CRNT_STATE          ; ``
              
              ; Initialise values in path arrays to DIR_NONE
              ; Fwd path direction
              LDX   #PATH_DIR           
              LDAA  #DIR_NONE
              STAA  0,X
              STAA  1,X
              STAA  2,X
              STAA  3,X
              STAA  4,X
              STAA  5,X
              STAA  6,X
              STAA  7,X
              
              ; Reverse path direction
              LDX   #REV_PATH_DIR
              LDAA  #DIR_NONE
              STAA  0,X
              STAA  1,X
              STAA  2,X
              STAA  3,X
              STAA  4,X
              STAA  5,X
              STAA  6,X
              STAA  7,X

              ; Stop wheels, in case the robot ended previous code while moving
              JSR   INIT_STOP
              JSR   ENABLE_TOF             ; Initialise TOF interrupt timer
              
              ; Initialise PWM control time to now.
              LDAA  TOF_COUNTER
              STAA  LAST_PWM

; MAIN Section
;----------------              
MAIN
              ; Sensors
              JSR   G_LEDS_ON              ; Enable guider LEDs
              JSR   READ_SENSORS           ; Read guider sensors
              JSR   G_LEDS_OFF             ; Disable the guider LEDs
              JSR   SENSOR_UPDATE          ; Update sensor detected flags
              
              ; Update display
              JSR   UPDT_DISPL
              
              ; State machine dispatcher
              LDAA  CRNT_STATE
              JSR   DISPATCHER

              ; Update PWM to control motors on/off
              JSR   PWM_UPDATE

              ; Repeat main loop
              BRA   MAIN

; Data Section - LUTs for displaying robot states and headings
;----------------   
; Robot states
tab           dc.b  "waiting",0
              dc.b  "forward",0
              dc.b  "all_stp",0
              dc.b  "lft_trn",0
              dc.b  "rgt_trn",0
              dc.b  "rev_trn",0
              dc.b  "lft_tmd",0
              dc.b  "rgt_tmd",0
; Robot headings
tab_heading   dc.b  'N','E','S','W','X' 
; Robot driving modes (search, back, reverse) TODO: add traceback mode
tab_drivemode dc.b  'S','B','R'

;***************************************************************************************************
;                                       SUBROUTINE SECTION
;***************************************************************************************************

; DISPATCHER subroutine
; Called each main loop. Checks current state and calls appropriate state subroutine.
; Important note:
; There are 2 types of line following turns: larger turns (LEFT/RIGHT_TRN)
; and smaller, alignment turns (LEFT/RIGHT_ALIGN)
; The larger turns are used when the robot is off by a larger amount, as detected by the port/stbd sensors (off line)
; The smaller alignment turns are used when the robot is mostly on the line, but slightly off-center, as detected by the line sensors
;----------------
DISPATCHER        
VERIFY_START      CMPA  #START                                ; Verify if the robot's state is START
                  BNE   VERIFY_FORWARD                        ; If not: move to FORWARD state validation
                  JSR   START_ST                              ; Else:   jump to START state
                  RTS

VERIFY_FORWARD    CMPA  #FWD                                  ; Verify if the robot's state is FORWARD
                  BNE   VERIFY_REV_TRN                        ; If not: move to REVERSE TURN state validation
                  JSR   FWD_ST                                ; Else:   jump to FORWARD state
                  RTS

VERIFY_REV_TRN    CMPA  #REV_TRN                              ; Verify if the robot's state is REV_TURN
                  BNE   VERIFY_STOP                           ; If not: move to LEFT_ALIGN state validation
                  JSR   REV_TRN_ST                            ; Else:   jump to REVERSE TURN state
                  RTS

VERIFY_STOP       CMPA  #ALL_STOP                             ; Verify if the robot's state is ALL_STOP
                  BNE   VERIFY_LEFT_TRN                       ; If not: move to LEFT_TURN state validation
                  JSR   ALL_STOP_ST                           ; Else:   jump to ALL_STOP state
                  RTS

VERIFY_LEFT_TRN   CMPA  #LEFT_TRN                             ; Verify if the robot's state is LEFT_TURN
                  BNE   VERIFY_LEFT_ALIGN                     ; If not: move to LEFT_ALIGN state validation
                  JSR   LEFT                                  ; Else:   jump to LEFT turn state
                  RTS

VERIFY_LEFT_ALIGN CMPA  #LEFT_ALIGN                           ; Verify if the robot's state is LEFT_ALIGN
                  BNE   VERIFY_RIGHT_TRN                      ; If not: move to RIGHT_ALIGN state validation
                  JSR   LEFT_ALIGN_DONE                       ; Else:   jump to LEFT_ALIGN_DONE state
                  RTS

VERIFY_RIGHT_TRN  CMPA  #RIGHT_TRN                            ; Verify if the robot's state is RIGHT_TURN
                  BNE   VERIFY_RIGHT_ALIGN                    ; If not: move to REV_TURN state validation
                  JSR   RIGHT                                 ; Else:   jump to RIGHT turn state
                  RTS

VERIFY_RIGHT_ALIGN CMPA  #RIGHT_ALIGN                         ; Verify if the robot's state is RIGHT_ALIGN
                   BNE   DISPATCH_EXIT                        ; If not: exit dispatcher (invalid state)
                   JSR   RIGHT_ALIGN_DONE                     ; Else:   jump to RIGHT_ALIGN_DONE state
                   RTS

DISPATCH_EXIT      RTS ;INVALID STATE


; PWM_UPDATE
; Called each main loop.
; Toggles motor enable bits on PTT every T_SPEED_ON TOF ticks.
; 50% duty, applies to all states (FWD, turns, reverse, etc.)
;------------------------------------------------------------------
PWM_UPDATE:
                  LDAA  CRNT_STATE
                  CMPA  #ALL_STOP
                  BEQ   PWM_EXIT        ; no PWM in full stop
                  CMPA  #START
                  BEQ   PWM_EXIT        ; and maybe not while waiting to start

                  ;LDAA  TOF_COUNTER
                  ;CMPA  T_SPD_CNTRL
                  ;BLO   PWM_EXIT

                  LDAA  TOF_COUNTER
                  SUBA  LAST_PWM
                  CMPA  #T_SPEED_ON
                  BLO   PWM_EXIT

                  LDAA  DDRT
                  EORA  #%00110000
                  STAA  DDRT

                  LDAA  TOF_COUNTER
                  STAA  LAST_PWM
                  ;STAA  T_SPD_CNTRL

PWM_EXIT:
                  RTS


; SENSOR_UPDATE
; Called each loop
; Reads all 5 sensors, checks states, stores values in DETECTED_*
; Values too high are "on black", too low are "on white"
;------------------------------------------------------------------
SENSOR_UPDATE
; CHECK BOW SENSOR A - TOO HIGH = BLACK, TOO LOW = WHITE
; Base + Variance < Sensor = WHITE
; Base + Variance > Sensor = BLACK
CHECK_BOW       LDAA BASE_BOW
                ADDA BOW_VARIANCE
                CMPA SENSOR_BOW
                BLO  BOW_ON_BLACK       ; If first condition met, branch to BLACK
                BHI  BOW_ON_WHITE       ; If second condition met, branch to WHITE
; If sensor on white, set detected flag to 0
BOW_ON_WHITE
                MOVB #0, DETECTED_BOW
                BRA  CHECK_MID
; If sensor on black, set detected flag to 1
BOW_ON_BLACK
                MOVB #1, DETECTED_BOW
                BRA  CHECK_MID

; CHECK MID SENSOR C - TOO HIGH = BLACK, TOO LOW = WHITE
; Base + Variance < Sensor = WHITE
; Base + Variance > Sensor = BLACK
CHECK_MID       LDAA BASE_MID
                ADDA MID_VARIANCE
                CMPA SENSOR_MID
                BLO  MID_ON_BLACK       ; If first condition met, branch to BLACK
                BHI  MID_ON_WHITE       ; If second condition met, branch to WHITE
; If sensor on white, set detected flag to 0
MID_ON_WHITE
                MOVB #0, DETECTED_MID
                BRA  CHECK_PORT
; If sensor on black, set detected flag to 1
MID_ON_BLACK
                MOVB #1, DETECTED_MID
                BRA  CHECK_PORT

; CHECK PORT SENSOR B - TOO HIGH = BLACK, TOO LOW = WHITE
; Base + Variance < Sensor = WHITE
; Base + Variance > Sensor = BLACK
CHECK_PORT      LDAA BASE_PORT
                ADDA PORT_VARIANCE
                CMPA SENSOR_PORT
                BLO  PORT_ON_BLACK      ; If first condition met, branch to BLACK
                BHI  PORT_ON_WHITE      ; If second condition met, branch to WHITE
; If sensor on white, set detected flag to 0
PORT_ON_WHITE
                MOVB #0, DETECTED_PORT
                BRA  CHECK_STBD
; If sensor on black, set detected flag to 1
PORT_ON_BLACK
                MOVB #1, DETECTED_PORT
                BRA  CHECK_STBD

; CHECK STBD SENSOR D - TOO HIGH = BLACK, TOO LOW = WHITE
; Base + Variance < Sensor = WHITE
; Base + Variance > Sensor = BLACK
CHECK_STBD      LDAA BASE_STBD
                ADDA STARBOARD_VARIANCE
                CMPA SENSOR_STBD
                BLO  STBD_ON_BLACK      ; If first condition met, branch to BLACK
                BHI  STBD_ON_WHITE      ; If second condition met, branch to WHITE
; If sensor on white, set detected flag to 0
STBD_ON_WHITE
                MOVB #0, DETECTED_STBD
                BRA  CHECK_LINE
; If sensor on black, set detected flag to 1
STBD_ON_BLACK
                MOVB #1, DETECTED_STBD
                BRA  CHECK_LINE

; CHECK LINE DIFF PAIR E/F
;   SENSOR E is on the left
;   SENSOR F is on the right
;   TOO HIGH = F on black (too far left)
;   TOO LOW  = E on black (too far right)
;   ELSE     = no significant difference
; WE SAY:
;   VAL < BASE - VAR = TOO LEFT
;   VAL > BASE + VAR = TOO RIGHT
CHECK_LINE      LDAA BASE_LINE
                ADDA LINE_VARIANCE
                CMPA SENSOR_LINE

                ; Based on position of sensor F, go to different checks
                BLO  F_ON_BLACK      
                BHS  F_ON_WHITE
; If F on black, set right detected flag to 1, left to 0
F_ON_BLACK
                MOVB #1, DETECTED_LINE_RIGHT
                MOVB #0, DETECTED_LINE_LEFT
                BRA  DONE_SENSOR_UPDATE
; If F on white, set right detected flag to 0, check left
F_ON_WHITE
                LDAA BASE_LINE
                SUBA LINE_VARIANCE
                CMPA SENSOR_LINE

                ; Based on position of sensor E, go to different checks
                BLS  E_ON_WHITE
                BHI  E_ON_BLACK
; If E on black, set left detected flag to 1, right to 0
E_ON_BLACK
                MOVB #0, DETECTED_LINE_RIGHT
                MOVB #1, DETECTED_LINE_LEFT
                BRA  DONE_SENSOR_UPDATE
; If E on white, set both detected flags to 0
E_ON_WHITE
                MOVB #0, DETECTED_LINE_RIGHT
                MOVB #0, DETECTED_LINE_LEFT
                BRA     DONE_SENSOR_UPDATE

DONE_SENSOR_UPDATE
                RTS

; MOVEMENT STATES
; START STATE
; Called each loop from DISPATCHER based on CRNT_STATE
;------------------------------------------------------------------
START_ST          BRSET   PORTAD0, $04, RELEASE         ; If front bumper is not pressed, go to RELEASE to check rear bumper
                  LDY     #9999                       ; Else, start driving forward after debouncing delay     
                  JSR     del_50us                      ; ``
                  JSR     INIT_FWD                      ; ``
                  MOVB    #FWD, CRNT_STATE              ; Set current state to FWD

RELEASE           ; SKIP FWD PATH FOR DEBUGGING
                  BRSET   PORTAD0, $08, RELEASE_REAR    ; If rear bumper is also not pressed, go to RELEASE_REAR to exit and back to main loop
                  ; Else, rear bumper is pressed and we start robot from end of maze.
                  ; This serves as a debugging tool to test reverse mode without having to complete the maze each time.
                  ; It does so by setting the robot as it would be set at the end of the maze, with a pre-defined path.
                  ; THIS PATH WILL NEED UPDATING IF A NEW MAZE IS USED.
                  ; ---------- PATH_DIR = 4,1,1,0,3,0,3,0 ----------
                  MOVB    #4, PATH_DIR
                  MOVB    #1, PATH_DIR+1
                  MOVB    #1, PATH_DIR+2
                  MOVB    #0, PATH_DIR+3
                  MOVB    #3, PATH_DIR+4
                  MOVB    #0, PATH_DIR+5
                  MOVB    #3, PATH_DIR+6
                  MOVB    #0, PATH_DIR+7

                  ; ---------- REV_PATH_DIR = 4,0,1,1,0,3,0,3 ----------
                  MOVB    #4, REV_PATH_DIR
                  MOVB    #0, REV_PATH_DIR+1
                  MOVB    #1, REV_PATH_DIR+2
                  MOVB    #1, REV_PATH_DIR+3
                  MOVB    #0, REV_PATH_DIR+4
                  MOVB    #3, REV_PATH_DIR+5
                  MOVB    #0, REV_PATH_DIR+6
                  MOVB    #3, REV_PATH_DIR+7
                  
                  ; Set robot variables as at end of maze
                  MOVB    #MODE_REVERSE, DRIVE_MODE
                  MOVB    #START, CRNT_STATE
                  MOVB    #7, INTERSECT_NUM
                  MOVB    #DIR_S, HEADING 
                  
                  ; Set robot wheels on, initialise stop state
                  BSET    DDRT, %00110000
                  JSR     INIT_STOP                           
                  
                  BRA     RELEASE_REAR      
RELEASE_REAR      ; This comes at the end of both paths, goes back to main loop
                  RTS   

; FORWARD MOVEMENT STATE
; This state is responsible for line following and intersection detection
; The intersection traversal logic follows an 'always-left' strategy
; If a left turn is not possible, it tries straight.
; Upon failure (hitting a wall), it will turn around and try the leftmost available direction again.
;------------------------------------------------------------------
FWD_ST            BRSET   PORTAD0, $04, NO_FWD_BUMP           ; If bow bumper is not pressed, go to NO_FWD_BUMP to check rear bumper
                  LDAA    #MODE_BACK                          ; Else, if bow bumper is pressed, robot hit a wall. 
                  STAA    DRIVE_MODE                          ; So, set drive mode to BACK
                  JSR     UPDT_DISPL                          ; And update the display
                  JSR     INIT_REV                            ; Start reversing away from wall for space to turn around
                  LDY     #6000                             ; And reverse for some time
                  JSR     del_50us                            ; ``
                  MOVB    #REV_TRN, CRNT_STATE                ; Then, set current state to REV_TRN
                  LBRA    EXIT                                ; And exit.

NO_FWD_BUMP       BRSET   PORTAD0, $08, NO_FWD_READ_BUMP_T    ; If rear bumper is not pressed, go to NO_FWD_READ_BUMP_T to continue non-bumper logic
                  BRA     YES_REAR_BUMP                       ; Else, rear bumper is pressed. Go to YES_REAR_BUMP to handle logic.

NO_FWD_READ_BUMP_T  LBRA   NO_FWD_REAR_BUMP                   ; If no bumpers are pressed, go to NO_FWD_REAR_BUMP to continue non-bumper logic

;**************************************
; REAR BUMPER PRESSED LOGIC
YES_REAR_BUMP                                                 ; If rear bumper is pressed, robot is at end of maze in either direction
                  LDAA    DRIVE_MODE                          ; Check robot drive mode
                  
                  CMPA    #MODE_REVERSE                       ; If robot is in reverse mode, i.e., going back to start from end of maze
                  BEQ     IS_REVERSE                          ; Branch to IS_REVERSE to handle logic
                  
                  CMPA    #MODE_TRACEBACK                     ; If robot is in traceback mode, i.e., going back to start after completing reverse of maze
                  BEQ     IS_TRACEBACK                        ; Branch to IS_TRACEBACK to handle logic
                  
                  MOVB    #MODE_REVERSE, DRIVE_MODE           ; Else, robot is in search or back mode, i.e., going fwd through maze for the first time
                  MOVB    #START, CRNT_STATE                  ; Set robot to reverse mode and start state
                  
                  BSET    DDRT, %00110000                     ; Then, turn wheels on
                  JSR     INIT_RIGHT                          ; And start turning right to turn around and face back into the maze

                  LDY     #T_TURN_AROUND                      ; Begin 180 degree turn by running 5 times the T_TURN_AROUND delay
                  JSR     del_50us                            ; ``
                  LDY     #T_TURN_AROUND                      ; ``
                  JSR     del_50us                            ; ``
                  LDY     #T_TURN_AROUND                      ; ``
                  JSR     del_50us                            ; ``
                  LDY     #T_TURN_AROUND                      ; ``
                  JSR     del_50us                            ; ``
                  LDY     #T_TURN_AROUND                      ; ``
                  JSR     del_50us                            ; ``
                  
                  JSR     INIT_STOP                           ; End backwards turn by entering STOP state
                  
                  LDAA    HEADING                             ; Update heading - flip 180 degrees
                  EORA    #2                                  ; XORing by 0010 will flip N<->S and E<->W (0000 <->0010 and 0001 <->0011)
                  STAA    HEADING                             ; Store updated heading
                  LBRA    EXIT                                ; And exit
                  
IS_REVERSE                                                    ; If rear bumper is pressed and robot is in reverse mode
                  MOVB    #MODE_TRACEBACK, DRIVE_MODE         ; Set drive mode to TRACEBACK
                  MOVB    #START, CRNT_STATE                  ; Set state to START
                  BSET    DDRT, %00110000                     ; Turn wheels on
                  JSR     INIT_RIGHT                          ; Start turning right to turn around and face back into the maze

                  LDY     #T_TURN_AROUND                      ; Begin 180 degree turn by running 5 times the T_TURN_AROUND delay
                  JSR     del_50us                            ; ``
                  LDY     #T_TURN_AROUND                      ; ``
                  JSR     del_50us                            ; ``
                  LDY     #T_TURN_AROUND                      ; ``
                  JSR     del_50us                            ; ``
                  LDY     #T_TURN_AROUND                      ; ``
                  JSR     del_50us                            ; ``
                  LDY     #T_TURN_AROUND                      ; ``
                  JSR     del_50us                            ; ``
                  
                  JSR     INIT_STOP                           ; End backwards turn by entering STOP state
                  
                  LDAA    HEADING                             ; Update heading - flip 180 degrees
                  EORA    #2                                  ; XORing by 0010 will flip N<->S and E<->W (0000 <->0010 and 0001 <->0011)
                  STAA    HEADING                             ; Store updated heading
                  LBRA    EXIT                                ; And exit
                  
IS_TRACEBACK                                                  ; If rear bumper is pressed and robot is in traceback mode
                  JSR     INIT_STOP                           ; Stop robot
                  MOVB    #ALL_STOP, CRNT_STATE               ; Set state to ALL_STOP
                  LBRA    EXIT                                ; And exit. Program ends here

; NEITHER FWD NOR REAR BUMPERS PRESSED LOGIC
NO_FWD_REAR_BUMP
                  ; Check for misalignment first
                  LDAA     DETECTED_MID                       ; If middle sensor of robot not on black, then robot not on line 
                  CMPA     #0                                 ; (i.e., misaligned)
                  LBEQ     NOT_ALIGNED                        ; So, go to NOT_ALIGNED to handle misalignment


;**************************************
; Else, robot is aligned on the line 
; Check for intersections
; If an intersection is detected, based off drive mode, update heading and path array
NO_MISALIGNMENT
                  ; Check if robot PORT sensor is on black (left intersection)
                  LDAA    DETECTED_PORT
                  CMPA    #0
                  LBEQ    NO_LEFT_INTERSECTION                ; If port sensor not detecting line, go to NO_LEFT_INTERSECTION to check starboard

PORT_INTERSECTION                                             ; Else, there is an intersection. 
                  LDAA    DRIVE_MODE                          ; Load the DRIVE_MODE and check which mode the robot is in
                  CMPA    #MODE_SEARCH                        ; Robot is in search mode
                  BEQ    PORT_INTERSECTION_SEARCH
                  
                  CMPA    #MODE_REVERSE                       ; Robot is in reverse mode
                  LBEQ     REVERSE_MODE_INTERSECTION
                  
                  CMPA    #MODE_TRACEBACK                     ; Robot is in traceback mode
                  LBEQ     TRACEBACK_MODE_INTERSECTION
                  

; Port intersection in back mode (no chg to heading or array)
; This happens when robot is going back to previous intersection after hitting a wall
; Thus, a new intersection is not reached, and intersection number is not incremented
; Also, entry heading (for way back) does not change in the memory array.
; However, heading is still updated to reflect the left turn.
PORT_INTERSECTION_BACK
                  LDAA    HEADING                             ; Load current heading       
                  SUBA    #1                                  ; Update heading to reflect left turn
                  ANDA    #3                                  ; Handle wraparound using AND
                  STAA    HEADING                             ; Store updated heading
                  BRA     DONE_HEADING_LEFT

; Port intersection in search mode
; This happens when a robot is exploring the maze for the first time and bumps into an intersection where it can turn left
; Thus, intersection number is incremented, and entry heading (for way back) is stored in the memory array
; Also, heading is updated to reflect the left turn.
PORT_INTERSECTION_SEARCH
                  ; Increment intersection number
                  INC     INTERSECT_NUM

                  ; Store the current, entry heading into reverse path array
                  LDX     #REV_PATH_DIR
                  LDAB    INTERSECT_NUM
                  ABX
                  LDAA    HEADING
                  STAA    0,X
                  
                  ; Update heading to reflect left turn
                  LDAA    HEADING                             ; Load current heading 
                  SUBA    #1                                  ; Update heading to reflect left turn
                  ANDA    #3                                  ; Handle wraparound using AND
                  STAA    HEADING                             ; Store updated heading
                  BRA     DONE_HEADING_LEFT

                  
; Port intersection setup completed
; Turn left 90 degrees, update drive mode to search, then store heading in array
; Array position will be corresponding to whether intersection is search or back mode
DONE_HEADING_LEFT
                  ; Store the heading into array
                  ; The position in the array is based off intersection number, which is correctly incremented based off state
                  LDX     #PATH_DIR
                  LDAB    INTERSECT_NUM
                  ABX
                  STAA    0,X

                  ; Set mode to search
                  ; This should happen regardless of previous mode, as robot is now exploring new path
                  MOVB    #MODE_SEARCH, DRIVE_MODE

                  ; Turn left 90 degrees
                  JSR     INIT_90_TURN
                  LBRA    TURN_LEFT_90_DEGREES

;**************************************
; NO PORT INTERSECTION - CHECK STARBOARD
; If starboard is detected, by problem statement, there is also a fwd path. drive fwd over the intersection
; and based off drive mode, update heading and path array
NO_LEFT_INTERSECTION
                  ; check stbd
                  LDAA    DETECTED_STBD
                  CMPA    #0
                  LBEQ    NO_INTERSECTION

                  ; MIGHT BE an intersection there - drive fwd a little bit and check for black on left side
                  ; if no black on left side - drive straight fwd
                  ; if black on left side - its a port turn, goto left intersection.
                  ; This condition came to be after testing showed that starboard sensor sometimes
                  ; detected false positives due to slight misalignment.
                  JSR     INIT_FWD              ; Drive fwd a little
                  LDY     #60                   ; ``
                  JSR     del_50us              ; ``

                  ; Update values
                  JSR   G_LEDS_ON               ; Enable the guider LEDs
                  JSR   READ_SENSORS            ; Read the 5 guider sensors
                  JSR   G_LEDS_OFF              ; Disable the guider LEDs
                  JSR   SENSOR_UPDATE           ; Update sensor detected flags
                  
                  ; Check if port sensor detects black
                  LDAA  DETECTED_PORT
                  CMPA  #1
                  LBEQ  PORT_INTERSECTION       ; If port sensor detects black, go to PORT_INTERSECTION to handle logic         
                  
                  ; Else, starboard intersection only
                  LDAA    DRIVE_MODE                    ; Load the DRIVE_MODE and check which mode the robot is in
                  
                  CMPA    #MODE_SEARCH                  ; Robot is in search mode
                  LBEQ    STBD_INTERSECTION_SEARCH
                  
                  CMPA    #MODE_REVERSE                 ; Robot is in reverse mode
                  LBEQ    REVERSE_MODE_INTERSECTION
                  
                  CMPA    #MODE_TRACEBACK               ; Robot is in traceback mode
                  LBEQ    TRACEBACK_MODE_INTERSECTION

; Starboard intersection in back mode 
; Heading and array not updated in back mode
; This happens when robot is going back to previous intersection after hitting a wall
STBD_INTERSECTION_BACK
                  LBRA     DONE_HEADING_RIGHT

; Starboard intersection in search mode
; Intersection number incremented, heading stays the same, and entry heading stored in array
STBD_INTERSECTION_SEARCH
                  ; Increment intersection number
                  INC     INTERSECT_NUM
                  
                  ; Store the current, entry heading into reverse path array
                  LDX     #REV_PATH_DIR
                  LDAB    INTERSECT_NUM
                  ABX
                  LDAA    HEADING
                  STAA    0,X
                  
                  LBRA     DONE_HEADING_RIGHT
                  
; Starboard intersection setup completed
; Move forward over intersection, update drive mode to search, then store heading in array
; Array position will be corresponding to whether intersection is search or back mode
DONE_HEADING_RIGHT
                  ; store the heading into array
                  LDAA    HEADING
                  LDX     #PATH_DIR
                  LDAB    INTERSECT_NUM
                  ABX
                  STAA    0,X

                  ; Set mode to search
                  LDAA    #MODE_SEARCH
                  STAA    DRIVE_MODE

                  ; Drive fwd over intersection
                  JSR     INIT_FWD_OVER_INTERSECTION
                  LBRA     GO_FWD_OVER_INTERSECTION

; No intersections seen
; Check alignment of robot on line it's meant to be following
NO_INTERSECTION
                  ; If right line on black (too far left), go to Check Right Align
                  LDAA    DETECTED_LINE_RIGHT
                  CMPA    #1
                  LBEQ    CHECK_RIGHT_ALIGN

                  ; If left line on black (too far right), go to Check Left Align
                  LDAA    DETECTED_LINE_LEFT
                  CMPA    #1
                  LBEQ    CHECK_LEFT_ALIGN

                  ;Exit condition
                  LBRA    EXIT

;**************************************
; Intersection seen in reverse mode
; In reverse mode, it doesn't matter which side intersection is detected
; Since we already know the directions in which the robot should be moving when it reaches each intersection.
REVERSE_MODE_INTERSECTION
                  ; Load the reverse path directory array at the index of INTERSECT_NUM
                  LDX    #REV_PATH_DIR
                  LDAB   INTERSECT_NUM
                  ABX
                  LDAA   0,X ;load reverse path direction for current intersection
                  
                  EORA   #2 ;XOR the direction with %0010. This will flip the direction, seeing as each entry
                            ;is the entry direction when the robot was going in the 'forwards' direction through
                            ;the maze. Thus, we must reverse the 'forwards' direction for the inverse path through the maze.
                  
                  ; Take the current heading and subtract it from the desired heading (stored in ACCA
                  ; For instance, if the robot is heading N (%0000) and the desired heading is E (%0001), the difference is %0001
                  ; We call this the 'direction delta' - it will range from 0 (same direction) to 3 (turn 90* CCW)
                  LDAB  HEADING         ; B = current
                  SBA                   ; A = desired - current  (can be -3..+3)
                  ANDA  #$03            ; A = (desired - current) mod 4  => 0..3  
                                         
                  ; Decrement the intersection number, as we get closer to the start of the maze.
                  DEC   INTERSECT_NUM

                  ; Compare the direction deltas
                  ; Check if delta is 0. If so,
                  ; Keep the same direction
                  CMPA  #0                              ; Check if delta is 0
                  BNE   NOT_FWD_REV_MODE                ; Else, go to check for if delta is 1
                  JSR   INIT_FWD_OVER_INTERSECTION
                  LBRA  GO_FWD_OVER_INTERSECTION

                  ; Check if delta is 1. If so,
NOT_FWD_REV_MODE  ; Turn 90* CW (Right)
                  CMPA  #1                      ; Check if delta is 1
                  BNE   NOT_RIGHT_REV_MODE      ; Else, delta is 3.
                  
                  ; Update heading - increment by 1, keep only the 2 LSBs.
                  LDAA    HEADING
                  INCA
                  ANDA    #$03
                  STAA    HEADING
                  
                  ; Turn right 90 degrees.
                  JSR     INIT_90_TURN_RIGHT
                  LBRA    TURN_RIGHT_90_DEGREES
        
                  ; Delta is 3, as it is impossible for delta to be 2.
                  ; We must turn left 90 degrees and update the heading
NOT_RIGHT_REV_MODE
                  ; Update heading - decrement by 1, keep only the 2 LSBs
                  LDAA    HEADING
                  DECA
                  ANDA    #$03
                  STAA    HEADING
                  
                  ; Turn left 90 degrees.
                  JSR     INIT_90_TURN
                  LBRA    TURN_LEFT_90_DEGREES

;**************************************
; Intersection seen in traceback mode
; In traceback mode, it doesn't matter which side intersection is detected
; Since we already know the directions in which the robot should be moving when it reaches each intersection.
; In this mode, we start at the beginning and drive through the whole maze with the memorized path.
TRACEBACK_MODE_INTERSECTION
                  ; Load the reverse path directory array at the index of INTERSECT_NUM
                  LDX     #PATH_DIR
                  LDAB    INTERSECT_NUM
                  ABX
                  
                  ; Take the current heading and subtract it from the desired heading (stored in ACCA
                  ; For instance, if the robot is heading N (%0000) and the desired heading is E (%0001), the difference is %0001
                  ; We call this the 'direction delta' - it will range from 0 (same direction) to 3 (turn 90* CCW)                 
                  LDAA    0,X           ;load reverse path direction for current intersection
                  
                  LDAB    HEADING       ; B = current
                  SBA                   ; A = desired - current  (can be -3..+3)
                  ANDA    #$03          ; A = (desired - current) mod 4  => 0..3
                  
                  ; Increment the intersection number, as we get closer to the end of the maze.
                  INC     INTERSECT_NUM
                  
                  ; Compare the direction deltas
                  ; Check if delta is 0. If so,
                  ; Keep the same direction
                  CMPA  #0                              ; Check if delta is 0
                  BNE   NOT_FW_TB_MODE                  ; Else, go to check for if delta is 1
                  JSR   INIT_FWD_OVER_INTERSECTION
                  LBRA  GO_FWD_OVER_INTERSECTION

                  ; Check if delta is 1. If so,
NOT_FW_TB_MODE    ; Turn right 90 degrees
                  CMPA  #1                              ; Check if delta is 1
                  BNE   NOT_RIGHT_TB_MODE               ; Else, go to check for if delta is 3
                  
                  ; Update heading - increment by 1, keep only the 2 LSBs.
                  LDAA    HEADING
                  INCA
                  ANDA    #3
                  STAA    HEADING
                  
                  ; Turn right 90 degrees.
                  JSR     INIT_90_TURN_RIGHT
                  LBRA    TURN_RIGHT_90_DEGREES

                  ; Delta is 3, as it is impossible for delta to be 2.
                  ; We must turn left 90 degrees and update the heading
NOT_RIGHT_TB_MODE
                  CMPA  #3
                  ; desired is 1 step counter-clockwise (-1 mod 4)
                  ; Turn left 90 degrees

                  ; Update heading - decrement by 1, keep only the 2 LSBs
                  LDAA    HEADING
                  SUBA    #1
                  ANDA    #3
                  STAA    HEADING

                  ; Turn left 90 degrees.
                  JSR     INIT_90_TURN
                  LBRA    TURN_LEFT_90_DEGREES
                  
                  
;**************************************
; NOT ALIGNED TO LINE 
; The robot is not on the line (the mid LDR doesnt detect the line)
; We determine which side detects the line (port or stbd)
; Based on that, we execute a larger sized (partial, rather than aligning) left or right turn
NOT_ALIGNED
                  ; Check port side - if that's the problem, execute partial left turn
                  LDAA    DETECTED_PORT
                  CMPA    #1
                  LBEQ    PARTIAL_LEFT_TRN

NO_PORT           ; Check starboard side - if that's the problem, execute partial right turn
                  LDAA    DETECTED_STBD
                  CMPA    #1
                  LBEQ    PARTIAL_RIGHT_TRN

NO_BOW            ; Neither port nor startboard - check bow.
                  ; If bow also on white, lost (exit)
                  ; If bow on black, do a partial right turn to re-align (chosen arbitrarily - if robot is here, it's in a bad state anyway)
                  LDAA    DETECTED_BOW
                  CMPA    #1
                  LBNE    EXIT
                  LBEQ    PARTIAL_RIGHT_TRN

; MOVEMENT/TURNING SUB-BRANCHES
;------------------------------------------------------------------
; EXECUTE PARTIAL TURNS - LEFT SIDE
PARTIAL_LEFT_TRN
                  ; Delay for a fraction of a second to allow the robot to move a little bit more forward
                  ; This was determined through trial and error
                  LDY     #60
                  jsr     del_50us
                  
                  ; Initialise the left turn
                  JSR     INIT_LEFT

                  ; Update current display state
                  MOVB    #LEFT_TRN, CRNT_STATE
                  JSR     UPDT_DISPL

                  ; DELAY (Allow time for turn to take effect)
                  ; Again, this was determined through trial and error
                  ;LDY     #600
                  LDY     #60
                  JSR     del_50us

                  ; DONE
                  LBRA     EXIT

; EXECUTE ALIGN TURN TO THE LEFT
; The align turn is like a partial turn with no delay
; Thus, the turn is much smaller in magnitude and better for when the robot is on the line.
CHECK_LEFT_ALIGN
                  JSR     INIT_LEFT                     ; Initialise left turn
                  MOVB    #LEFT_ALIGN, CRNT_STATE       ; Update current state
                  LBRA     EXIT

;------------------------------------------------------------------

; EXECUTE PARTIAL TURNS - RIGHT SIDE
PARTIAL_RIGHT_TRN
                  ; Delay for a fraction of a second to allow the robot to move a little bit more forward
                  ; This was determined through trial and error
                  LDY     #60
                  JSR     del_50us

                  ; Initialise the right turn
                  JSR     INIT_RIGHT

                  ; Update current display state
                  MOVB    #RIGHT_TRN, CRNT_STATE
                  JSR     UPDT_DISPL

                  ; DELAY (Allow time for turn to take effect)
                  ; Again, this was determined through trial and error
                  ;LDY     #600
                  LDY     #60
                  JSR     del_50us

                  ; DONE
                  LBRA     EXIT

; EXECUTE ALIGN TURN TO THE RIGHT
; The align turn is like a partial turn with no delay
; Thus, the turn is much smaller in magnitude and better for when the robot is on the line.
CHECK_RIGHT_ALIGN
                  JSR     INIT_RIGHT                     ; Initialise right turn
                  MOVB    #RIGHT_ALIGN, CRNT_STATE       ; Update current state
                  LBRA     EXIT


; ALIGNMENT SUBROUTINES
; The LEFT & RIGHT subroutines are based off the bow sensor's position, and are used for major alignments
; The LEFT/RIGHT_ALIGN_DONE subroutines are run once and reset the state to fwd, so the robot moves forward again
; before checking for a misalignment.
;------------------------------------------------------------------
; Turn left until bow sensor is on the line
LEFT
                  LDAA    DETECTED_BOW          ; Load the bow state  
                  CMPA    #1                    ; Check bow state - if it's 1, then the alignment is done (bow is now on line)
                  BEQ     LEFT_ALIGN_DONE       ; ``
                  LBNE    EXIT                  ; Else, not done - exit subroutine.

; left alignment is done
LEFT_ALIGN_DONE
                  MOVB    #FWD, CRNT_STATE      ; Update robot state to be FWD
                  JSR     INIT_FWD              ; Initialise FWD movement
                  LBRA    EXIT                  ; Exit subroutine

; Turn right until bow sensor is on the line
RIGHT
                  LDAA    DETECTED_BOW          ; Load the bow state
                  CMPA    #1                    ; Check bow state - if it's 1, then the alignment is done (bow is now on line)
                  BEQ     RIGHT_ALIGN_DONE      ; ``
                  LBNE    EXIT                  ; Else, not done - exit subroutine

; right alignment is done
RIGHT_ALIGN_DONE  MOVB    #FWD, CRNT_STATE      ; Update robot state to be FWD
                  JSR     INIT_FWD              ; Initialise FWD movement
                  LBRA     EXIT                 ; Exit subroutine

; OTHER MOVEMENT STATES
; These states are time-measured movements, used either in 90- or 180-degree turns, along with
; the 'drive through intersection' movement. The latter is used to drive over an intersection when a fwd path is detected.
;------------------------------------------------------------------
; Reverse turn
; This executes a ~180* turn to the right using a hardcoded delay.
;---------------- 
REV_TRN_ST
                  BSET    DDRT, %00110000       ; Enable motor movement
                  JSR     INIT_RIGHT            ; Initialise motors moving right

                  LDY     #T_TURN_AROUND        ; Turn around based on the T_TURN_AROUND value
                  JSR     del_50us              ; This was determined through trial and error for this specific robot.
                  LDY     #T_TURN_AROUND        ; ``
                  JSR     del_50us              ; ``
                  LDY     #T_TURN_AROUND        ; ``
                  JSR     del_50us              ; ``
                  LDY     #T_TURN_AROUND        ; ``
                  JSR     del_50us              ; ``
                  LDY     #T_TURN_AROUND        ; ``
                  JSR     del_50us              ; ``
                  
                  MOVB    #FWD, CRNT_STATE      ; Set the current state to FWD 
                  JSR     INIT_FWD              ; Initialise FWD movement

                  ; Swap the heading of the robot. If heading N, flip to S, etc.
                  LDAA    HEADING               ; Load heading
                  EORA    #2                    ; XOR by 0010, flip heading
                  STAA    HEADING               ; Store heading
                  LBRA    EXIT                  ; Exit subroutine

; All stop
; This is the end of the program and will disable the motors, as well as loop repeatedly in the all stop state.
;---------------- 
ALL_STOP_ST
                  BCLR    DDRT, %00110000       ; Disable motors
                  LBRA    EXIT                  ; Exit subroutine

; 90 Degree turn left
; This executes a ~90* turn to the left using a hardcoded delay
; This is only used in return and traceback mode. 
; The robot's heading is handled externally.
;---------------- 
TURN_LEFT_90_DEGREES
                  BSET    DDRT, %00110000       ; Enable motor movement
                  JSR     INIT_FWD              ; Initialise motors moving fwd
                  
                  LDY     #T_TURN_90            ; Drive forward a short while, so that
                  JSR     del_50us              ; the wheels (and pivot point of the robot) are
                  LDY     #T_TURN_90            ; aligned with where the sensors detected the intersection
                  JSR     del_50us              ; ``
                  
                  JSR     INIT_LEFT             ; Initialise motors moving left

                  LDY     #T_TURN_90            ; Execute a left turn for 90 degrees.
                  JSR     del_50us              ; This is based on the T_TURN_90 value.
                  LDY     #T_TURN_90            ; This was determined through trial and error for this specific robot.
                  JSR     del_50us              ; ``
                  LDY     #T_TURN_90            ; ``
                  JSR     del_50us              ; ``
                  
                  MOVB    #FWD, CRNT_STATE      ; Set the current state to be FWD
                  JSR     INIT_FWD              ; Initialise forward movement
                  BRA     EXIT                  ; Exit subroutine


; 90 Degree turn right
; This executes a ~90* turn to the right using a hardcoded delay
; This is only used in return and traceback mode. 
; The robot's heading is handled externally.
;---------------- 
TURN_RIGHT_90_DEGREES
                  BSET    DDRT, %00110000       ; Enable motor movement
                  JSR     INIT_FWD              ; Initialise motors moving fwd

                  LDY     #T_TURN_90            ; Drive forward a short while, so that
                  JSR     del_50us              ; the wheels (and pivot point of the robot) are
                  LDY     #T_TURN_90            ; aligned with where the sensors detected the intersection
                  JSR     del_50us              ; ``
                  
                  JSR     INIT_RIGHT            ; Initialise motors moving right

                  LDY     #T_TURN_90            ; Execute a right turn for 90 degrees.
                  JSR     del_50us              ; This is based on the T_TURN_90 value.
                  LDY     #T_TURN_90            ; This was determined through trial and error for this specific robot.
                  JSR     del_50us              ; ``
                  LDY     #T_TURN_90            ; ``
                  JSR     del_50us              ; ``
                  
                  MOVB    #FWD, CRNT_STATE      ; Set the current state to be FWD
                  JSR     INIT_FWD              ; Initialise forward movement
                  BRA     EXIT                  ; Exit subroutine

; Go forward over intersection
; This executes a forward drive using a hardcoded delay
; This is only used in return and traceback mode.
;---------------- 
GO_FWD_OVER_INTERSECTION
                  BSET    DDRT, %00110000       ; Enable motor movement
                  JSR     INIT_FWD              ; Initialise motors moving fwd

                  LDY     #T_FWD_OVER_INTERSECTION      ; Drive forward a short while
                  JSR     del_50us                      ; This is based on the T_FWD_OVER_INTERSECTION value
                  LDY     #T_FWD_OVER_INTERSECTION      ; which has been determined through trial and error for this specific robot.
                  JSR     del_50us                      ; ``
                  
                  MOVB    #FWD, CRNT_STATE      ; Set the current state to be FWD
                  JSR     INIT_FWD              ; Initialise forward movement
                  BRA     EXIT                  ; Exit subroutine

; EXIT - END OF SUBROUTINE
;---------------- 
EXIT
                  RTS


;***************************************************************************************************
;                                 INITIALISATION SUBROUTINE SECTION
;***************************************************************************************************

; Initialise 
; Run once @ beginning of pgm - set all detected values to 0.
;---------------- 
SENSOR_FLAGS_INIT:
        MOVB  #0, DETECTED_BOW
        MOVB  #0, DETECTED_PORT
        MOVB  #0, DETECTED_STBD
        MOVB  #0, DETECTED_MID
        MOVB  #0, DETECTED_LINE_LEFT
        MOVB  #0, DETECTED_LINE_RIGHT
        RTS

; Initialise right turn
;---------------- 
INIT_RIGHT        BSET    PORTA,%00000010           ; Set right motor to turn BACK
                  BCLR    PORTA,%00000001           ; Set left motor to turn FWD
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  LDAA    TOF_COUNTER               ; Mark the right turn time T_TURN
                  ADDA    #T_RIGHT                  ; ``
                  STAA    T_TURN                    ; ``
                  RTS

; Initialise left turn
;---------------- 
INIT_LEFT         BSET    PORTA,%00000001           ; Set left motor to turn BACK
                  BCLR    PORTA,%00000010           ; Set right motor to turn FWD
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  LDAA    TOF_COUNTER               ; Mark the right turn time T_TURN
                  ADDA    #T_LEFT                   ; ``
                  STAA    T_TURN                    ; ``
                  RTS

; Initialise forward movement
;---------------- 
INIT_FWD          BCLR    PORTA, %00000011          ; Set FWD direction for both motors
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  RTS

; Initialise reverse movement
;---------------- 
INIT_REV          BSET PORTA,%00000011              ; Set REV direction for both motors
                  BSET PTT,%00110000                ; Turn on the drive motors
                  RTS

; Initialise reverse turn
;---------------- 
INIT_REV_TURN     BSET    PORTA,%00000010           ; Set right motor to turn BACK
                  BCLR    PORTA,%00000001           ; Set left motor to turn FWD
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  RTS

; Initialize 90 degree turn (left)
;---------------- 
INIT_90_TURN      BSET    PORTA,%00000001           ; Set left motor to turn BACK
                  BCLR    PORTA,%00000010           ; Set right motor to turn FWD
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  RTS


; Initialize 90 degree turn right
;---------------- 
INIT_90_TURN_RIGHT      BSET    PORTA,%00000010         ; Set right motor to turn BACK
                        BCLR    PORTA,%00000001         ; Set left motor to turn FWD
                        BSET    PTT, %00110000          ; Turn on the drive motors
                        RTS

; Initialize forward over intersection
;---------------- 
INIT_FWD_OVER_INTERSECTION
                        JSR     INIT_FWD                ; Same process as initialising fwd movement
                        RTS                             ; This subroutine was created for ease of reading and coding

; Initialize stop (all motors off)
;---------------- 
INIT_STOP         BCLR    PTT, %00110000                ; Turn off the drive motors
                  RTS


; Initialise Sensors, Motors, etc.
; Set ports to be inputs/outputs, as needed.
;---------------- 
INIT              BSET   DDRA,$FF       ; PORTA > OUTPUT
                  BCLR   DDRAD,$FF      ; PORTAD > INPUT
                  BSET   DDRB,$FF       ; PORTB > OUTPUT
                  BSET   DDRJ,$C0       ; Pins 6,7 of PTJ > OUTPUTS
                  RTS


; Initialise ADC
;---------------- 
openADC           MOVB   #$80,ATDCTL2   ; Turn on ADC
                  LDY    #1             ; Wait for 50 us for ADC to be ready
                  JSR    del_50us       ; ``
                  MOVB   #$20,ATDCTL3   ; 4 conversions on channel AN1
                  MOVB   #$97,ATDCTL4   ; 8-bit res, prescale = 48
                  RTS

; Initialise ADC
; This routine writes 'SPACE' characters into the LCD display
; to prepare it for a new display mode.
;---------------- 
CLR_LCD_BUF       LDX   #CLEAR_LINE
                  LDY   #TOP_LINE
                  JSR   STRCPY

CLB_SECOND        LDX   #CLEAR_LINE
                  LDY   #BOT_LINE
                  JSR   STRCPY

CLB_EXIT          RTS

; String Copy
; Copies a null-terminated string from address in REG_X to address in REG_Y
; X contains starting address of null-terminated string
; Y contains first address of destination
;---------------- 
STRCPY            PSHX            ; Protect the registers used
                  PSHY
                  PSHA

STRCPY_LOOP       LDAA 0,X        ; Get a source character
                  STAA 0,Y        ; Copy it to the destination
                  BEQ STRCPY_EXIT ; If it was the null, then exit
                  INX             ; Else increment the pointers
                  INY
                  BRA STRCPY_LOOP ; and do it again

STRCPY_EXIT       PULA            ; Restore the registers
                  PULY
                  PULX
                  RTS

; -------------------------------------------------------------------------------------------------
;                                   Guider LEDs ON                                                 |
; This routine enables the guider LEDs so that readings of the sensor                              |
; correspond to the illuminated situation.                                                         |
; Passed: Nothing                                                                                  |
; Returns: Nothing                                                                                 |
; Side: PORTA bit 5 is changed                                                                     |
G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5                                                 |
                  RTS                                                                             ;|


; -------------------------------------------------------------------------------------------------
;                                   Guider LEDs OFF                                                |
; This routine disables the guider LEDs. Readings of the sensor                                    |
; correspond to the ?ambient lighting? situation.                                                  |
; Passed: Nothing                                                                                  |
; Returns: Nothing                                                                                 |
; Side: PORTA bit 5 is changed                                                                     |
G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5                                               |
                  RTS                                                                             ;|


; -------------------------------------------------------------------------------------------------
;                               Read Sensors

READ_SENSORS      CLR   SENSOR_NUM     ; Select sensor number 0
                  LDX   #SENSOR_LINE   ; Point at the start of the sensor array

RS_MAIN_LOOP      LDAA  SENSOR_NUM     ; Select the correct sensor input
                  JSR   SELECT_SENSOR  ; on the hardware
                  LDY   #400         ; 20 ms delay to allow the
                  JSR   del_50us       ; sensor to stabilize
                  LDAA  #%10000001     ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  LDAA  ATDDR0L        ; A/D conversion is complete in ATDDR0L
                  STAA  0,X            ; so copy it to the sensor register
                  CPX   #SENSOR_STBD   ; If this is the last reading
                  BEQ   RS_EXIT        ; Then exit
                  INC   SENSOR_NUM     ; Else, increment the sensor number
                  INX                  ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP   ; and do it again

RS_EXIT           RTS


; -------------------------------------------------------------------------------------------------
;                               Select Sensor
; -------------------------------------------------------------------------------------------------
SELECT_SENSOR     PSHA                ; Save the sensor number for the moment
                  LDAA PORTA          ; Clear the sensor selection bits to zeros
                  ANDA #%11100011
                  STAA TEMP           ; and save it into TEMP
                  PULA                ; Get the sensor number
                  ASLA                ; Shift the selection number left, twice
                  ASLA
                  ANDA #%00011100     ; Clear irrelevant bit positions
                  ORAA TEMP           ; OR it into the sensor bit positions
                  STAA PORTA          ; Update the hardware
                  RTS


;***************************************************************************************************
;                                               DISPLAY SECTION
;***************************************************************************************************
; Positional values for the display
;---------------- 
DP_FRONT_SENSOR   EQU TOP_LINE+14
DP_LINE_SENSOR    EQU TOP_LINE+17
DP_VOLTAGE_TEN_THOU       EQU TOP_LINE+2
DP_VOLTAGE_THOU           EQU DP_VOLTAGE_TEN_THOU+1
DP_VOLTAGE_DECIMAL        EQU DP_VOLTAGE_TEN_THOU+2
DP_VOLTAGE_HUNDREDS       EQU DP_VOLTAGE_TEN_THOU+3
DP_HEADING        EQU DP_VOLTAGE_HUNDREDS+4
DP_DRIVEMODE      EQU DP_HEADING+1
DP_INTERSECT_NUM  EQU DP_DRIVEMODE+1

DP_PORT_SENSOR    EQU BOT_LINE+12
DP_MID_SENSOR     EQU BOT_LINE+15
DP_STBD_SENSOR    EQU BOT_LINE+18
DP_STATE          EQU BOT_LINE+2
DP_STATE_END      EQU DP_STATE+7

; Update display
; This shows the Voltage, as well as the robot heading, drive mode, intersection number, 
;       and values of the bow and line sensors, in that order on line 1.
; Along with the robot state, and the values of the port, mid, and stbd sensors, in that
;       order, on line 2.
;---------------- 
UPDT_DISPL
                  JSR   clrLCD                  ; Clear LCD
                  
                  LDAA    DRIVE_MODE            ; Based on the robot drive mode, the robot will either display the abovementioned info
                  CMPA    #MODE_REVERSE         ; Or, if the robot is in reverse mode, before starting, the list of detected intersections
                  BNE    NORMAL_DISP_OP         ; for debugging purposes. Compare the drive mode, and if it equals REVERSE AND the 
                  LDAA    CRNT_STATE            ; current state equals START, then go to UPD_DISP_INTERSECTIONS
                  CMPA    #START                ; ``
                  BNE    NORMAL_DISP_OP         ; Else, go to normal display operations

                  LBRA  UPD_DISP_INTERSECTIONS  
                  
; Normal display operations
NORMAL_DISP_OP    
; FIRST LINE        
;---------------- 
                        ; Show the "V:"
                  LDAA    #'V'
                  LDAB    #':'
                  LDX     #TOP_LINE
                  STAA    0,X
                  LDX     #TOP_LINE
                  STAB    1,X

                        ; Read the voltage
                  MOVB    #$90,ATDCTL5    ; R-just., uns., sing. conv., mult., ch=0, start
                  BRCLR   ATDSTAT0,$80,*  ; Wait until the conver. seq. is complete
                  LDAA    ATDDR0L         ; Load the ch0 result - battery volt - into A
                  LDAB    #39             ;AccB = 39
                  MUL                     ;AccD = 1st result x 39
                  ADDD    #600          ;AccD = 1st result x 39 + 600
                  JSR     int2BCD         ; Convert value to BCD
                  JSR     BCD2ASC         ; Convert result to ASCII

                        ; Display voltage
                  LDAB     TEN_THOUS            ; Show first digit
                  LDX     #DP_VOLTAGE_TEN_THOU
                  STAB     0,X

                  LDAB     THOUSANDS            ; Show second digit
                  LDX     #DP_VOLTAGE_THOU
                  STAB     0,X

                  LDAB     #'.'                 ; Show decimal point
                  LDX      #DP_VOLTAGE_DECIMAL  
                  STAB     0,X

                  LDAB     HUNDREDS             ; Show digit after decimal
                  LDX     #DP_VOLTAGE_HUNDREDS
                  STAB     0,X

                        ; Display heading
                  LDX      #tab_heading         ; Load heading LUT
                  LDAB     HEADING              ; Add current heading value to LUT
                  ABX                           ; ``
                  LDAA     0,X                  ; Load position for heading
                  LDY      #DP_HEADING          ; ``
                  STAA     0,Y                  ; Store heading char to that position

                        ; Display drive mode
                  LDX      #tab_drivemode       ; Load drive mode LUT
                  LDAB     DRIVE_MODE           ; Add current drive mode value to LUT
                  ABX                           ; ``
                  LDAA     0,X                  ; Load position for drive mode
                  LDY      #DP_DRIVEMODE        ; ``
                  STAA     0,Y                  ; Store drive mode char to that position

                        ; Display intersection number
                  LDAA     INTERSECT_NUM        ; Load intersection number
                  ADDA     #$30                 ; Convert to ASCII char
                  LDY      #DP_INTERSECT_NUM    ; Load position for intersection number
                  STAA     0,Y                  ; Store intersection number to that position

                        ; Display sensor values
                  LDAA  SENSOR_BOW        ; Get the BOW sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_FRONT_SENSOR  ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there

                  LDAA  SENSOR_LINE       ; Get the LINE sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_LINE_SENSOR   ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there

; SECOND LINE
;---------------- 
                        ; Show the "S:"
                  LDAA    #'S'
                  LDAB    #':'
                  LDX     #BOT_LINE
                  STAA    0,X
                  LDX     #BOT_LINE
                  STAB    1,X

                        ; Display the state
                  LDAB    CRNT_STATE      ; Display current state
                  LSLB                    ; "
                  LSLB                    ; "
                  LSLB                    ; "
                  LDX     #tab            ; "
                  ABX                     ; "
                  LDY     #DP_STATE       ; "
                  JSR     STRCPY          ; "

                        ; Display a space between the state and other values
                  LDAB     #' '
                  LDX      #DP_STATE_END
                  STAB     0,X

                        ; Display sensor values
                  LDAA  SENSOR_PORT        ; Get the PORT sensor value
                  JSR   BIN2ASC            ; Convert to ascii string in D
                  LDX   #DP_PORT_SENSOR    ; Point to the LCD buffer position
                  STD   0,X                ; and write the 2 ascii digits there

                  LDAA  SENSOR_MID         ; Get the MID sensor value
                  JSR   BIN2ASC            ; Convert to ascii string in D
                  LDX   #DP_MID_SENSOR     ; Point to the LCD buffer position
                  STD   0,X                ; and write the 2 ascii digits there

                  LDAA  SENSOR_STBD        ; Get the STBD sensor value
                  JSR   BIN2ASC            ; Convert to ascii string in D
                  LDX   #DP_STBD_SENSOR    ; Point to the LCD buffer position
                  STD   0,X                ; and write the 2 ascii digits there

                        ; Write lines to the LCD

                  LDAA    #CLEAR_HOME   ; Move LCD cursor to the 1st row
                  JSR     cmd2LCD
                  LDY     #40           ; Wait until command is complete
                  JSR     del_50us
                  JSR     CLR_LCD_BUF   ; Separately clear LCD Buffer before displaying
                  LDY     #40                  ; Wait until command is complete
                  JSR     del_50us
                  LDX     #TOP_LINE     ; Load top line
                  JSR     putsLCD       ; And store it on the LCD

                  LDAA    #$C0          ; Move LCD cursor to the 2nd row
                  JSR     cmd2LCD
                  LDY     #40           ; Wait until command is complete
                  JSR     del_50us
                  LDX     #BOT_LINE     ; Load bottom line
                  JSR     putsLCD       ; And store it on the LCD

                  RTS


; Path display operations
; This displays all the PATH values on line 1 and the RETURN PATH values on line 2
DP_P_LABEL       EQU  TOP_LINE       ; 'P' at col 0, ':' at col 1
DP_R_LABEL       EQU  BOT_LINE       ; 'R' at col 0, ':' at col 1

DP_P_PATH_START  EQU  TOP_LINE+2     ; first direction on row 1
DP_R_PATH_START  EQU  BOT_LINE+2     ; first direction on row 2

; Update display for the intersections
UPD_DISP_INTERSECTIONS
        JSR     SHOW_PATHS           ; Build intersection strings
        LDAA    #CLEAR_HOME          ; Move LCD cursor to the 1st row
        JSR     cmd2LCD
        LDY     #40                  ; Wait until command is complete
        JSR     del_50us
        JSR     CLR_LCD_BUF          ; Separately clear LCD Buffer before displaying
        LDY     #40                  ; Wait until command is complete
        JSR     del_50us

        LDX     #TOP_LINE            ; Load top line
        JSR     putsLCD              ; and store it on the LCD

        LDAA    #$C0                 ; Move LCD cursor to the 2nd row
        JSR     cmd2LCD
        LDY     #40                  ; Wait until command is complete
        JSR     del_50us
        
        LDX     #BOT_LINE            ; Load bottom line
        JSR     putsLCD              ; And store it on the LCD

        RTS

; DISP_PATH_BUF
; In:
;   Y -> array of direction codes (0..4)
;   X -> destination buffer (e.g. DP_P_PATH_START)
;   B = number of elements
; Uses:
;   tab_heading: db 'N','E','S','W','X'
; Clobbers: A, B, X, Y
;------------------------------------------------------------
DISP_PATH_BUF
DPB_LOOP
        CMPB    #0
        BEQ     DPB_DONE

        ; A = code (0..4) from array
        LDAA    0,Y
        INY

        ; Map code via tab_heading using A as offset
        ; Need B temporarily as index, so save loop counter
        PSHB                ; save count

        TAB                 ; B = code (0..4)
        PSHX
        LDX     #tab_heading
        ABX                 ; X = tab_heading + B
        LDAA    0,X         ; A = 'N'/'E'/'S'/'W'/'X'
        PULX
        PULB                ; restore count to B

        ; Store letter into buffer
        STAA    0,X
        INX

        ; One element consumed
        DECB
        BEQ     DPB_DONE    ; no trailing space after last one

        ; Store space between entries
        LDAA    #' '
        STAA    0,X
        INX

        BRA     DPB_LOOP

DPB_DONE
        RTS
        
; SHOW_PATHS
; Builds:
;   Row 1: "P:" + PATH_DIR entries as N/E/S/W separated by spaces
;   Row 2: "R:" + REV_PATH_DIR entries as N/E/S/W separated by spaces
; Uses INTERSECT_NUM (0-based index of last valid entry)
;------------------------------------------------------------
SHOW_PATHS
        ; ---------------- Row 1: "P:" label ----------------
        LDX     #DP_P_LABEL
        LDAA    #'P'
        STAA    0,X
        LDAA    #':'
        STAA    1,X

        ; PATH_DIR content -> TOP_LINE starting at col 2
        LDX     #DP_P_PATH_START     ; dest buffer (row 1 path start)
        LDY     #PATH_DIR            ; source array

        LDAA    INTERSECT_NUM        ; 0..MAX_INTERS-1
        ;INCA                         ; count = last_index + 1
        TAB                          ; B = count

        JSR     DISP_PATH_BUF

        ; ---------------- Row 2: "R:" label ----------------
        LDX     #DP_R_LABEL
        LDAA    #'R'
        STAA    0,X
        LDAA    #':'
        STAA    1,X

        ; REV_PATH_DIR content -> BOT_LINE starting at col 2
        LDX     #DP_R_PATH_START
        LDY     #REV_PATH_DIR

        LDAA    INTERSECT_NUM
        INCA
        TAB

        JSR     DISP_PATH_BUF

        RTS


;***************************************************************************************************
;                                       TOF SUBROUTINE SECTION
;***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1           ; Enable TCNT
                  STAA    TFLG2           ; Clear TOF
                  LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS

TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000      ; Clear
                  STAA    TFLG2           ; TOF
                  RTI

;***************************************************************************************************
;                                   UTILITY SUBROUTINE SECTION
;***************************************************************************************************
; Initialise LCD
;---------------- 
initLCD:          BSET    DDRB,%11111111  ; configure pins PS7,PS6,PS5,PS4 for output
                  BSET    DDRJ,%11000000  ; configure pins PE7,PE4 for output
                  LDY     #2000         ; wait for LCD to be ready
                  JSR     del_50us        ; ``
                  LDAA    #$28            ; set 4-bit data, 2-line display
                  JSR     cmd2LCD         ; ``
                  LDAA    #$0C            ; display on, cursor off, blinking off
                  JSR     cmd2LCD         ; ``
                  LDAA    #$06            ; move cursor right after entering a character
                  JSR     cmd2LCD         ; ``
                  RTS

; Clear LCD
;---------------- 
clrLCD:           LDAA  #$01                    ; clear cursor, return to home position
                  JSR   cmd2LCD                 ; ``
                  LDY   #40                     ; wait until cmd is complete
                  JSR   del_50us                ; ``
                  RTS

; Delay 50 us * Y
;---------------- 
del_50us          PSHX                   ; (2 E-clk) Protect X register
eloop             LDX   #300           ; (2 E-clk) Initialise inner loop count
iloop             NOP                    ; (1 E-clk) NOP
                  DBNE X,iloop           ; (3 E-clk) If the inner count != 0, loop again
                  DBNE Y,eloop           ; (3 E-clk) If the outer count != 0, loop again
                  PULX                   ; (3 E-clk) Restore X register
                  RTS                    ; (5 E-clk) 

; Send a command to the LCD
;---------------- 
cmd2LCD:          BCLR  LCD_CNTR, LCD_RS ; select the LCD instruction
                  JSR   dataMov          ; send data to IR
                  RTS

; Send a null-terminated string to the LCD
;---------------- 
putsLCD:          LDAA  1,X+             ; get one character from  string
                  BEQ   donePS           ; get NULL character
                  JSR   putcLCD
                  BRA   putsLCD

donePS            RTS

; Send a character to the LCD
;---------------- 
putcLCD:          BSET  LCD_CNTR, LCD_RS  ; select the LCD data register (DR)c
                  JSR   dataMov           ; send data to DR
                  RTS

; Send data to the LCD
;---------------- 
dataMov:          BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the upper 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LSLA                    ; match the lower 4 bits with LCD data pins
                  LSLA                    ; ""
                  LSLA                    ; ""
                  LSLA                    ; ""
                  BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the lower 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LDY   #1                ; adding this delay allows
                  JSR   del_50us          ; completion of most instructions
                  RTS

; Initialise ADC
;---------------- 
initAD            MOVB  #$C0,ATDCTL2      ;power up AD, select fast flag clear
                  JSR   del_50us          ;wait for 50 us
                  MOVB  #$00,ATDCTL3      ;8 conversions in a sequence
                  MOVB  #$85,ATDCTL4      ;res=8, conv-clks=2, prescal=12
                  BSET  ATDDIEN,$0C       ;configure pins AN03,AN02 as digital inputs
                  RTS

; Convert integer to BCD
;---------------- 
int2BCD           XGDX                    ; Save binary number into X
                  LDAA #0                 ; Clear BCD_BUFFER
                  STAA TEN_THOUS
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                  CPX #0                  ; Check for zero input
                  BEQ CON_EXIT            ; and if so, exit
                  XGDX                    ; Not zero, get the binary number back to D as dividend
                  LDX #10                 ; Setup 10 (Decimal!) as the divisor
                  IDIV                    ; Divide Quotient is now in X, remainder in D
                  STAB UNITS              ; Store remainder
                  CPX #0                  ; If quotient is zero,
                  BEQ CON_EXIT            ; then exit
                  XGDX                    ; else swap first quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TENS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TEN_THOUS

CON_EXIT          RTS                     ; Were done the conversion

LCD_POS_CRSR      ORAA #%10000000         ; Set the high bit of the control word
                  JSR cmd2LCD             ; and set the cursor address
                  RTS

; Convert binary to ASCII
;---------------- 
BIN2ASC               PSHA               ; Save a copy of the input number
                      TAB
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the LSnibble
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX
                      LDAA 0,X            ; Get the LSnibble character
                      PULB                ; Retrieve the input number into ACCB
                      PSHA                ; and push the LSnibble character in its place
                      RORB                ; Move the upper nibble of the input number
                      RORB                ;  into the lower nibble position.
                      RORB
                      RORB
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the MSnibble
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX
                      LDAA 0,X            ; Get the MSnibble character into ACCA
                      PULB                ; Retrieve the LSnibble character into ACCB
                      RTS

;***************************************************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag ?NO_BLANK? starts cleared and is set once a non-zero
;* digit has been detected.
;* The ?units? digit is never blanked, even if it and all the
;* preceding digits are zero.
BCD2ASC           LDAA    #0            ; Initialize the blanking flag
                  STAA    NO_BLANK

C_TTHOU           LDAA    TEN_THOUS     ; Check... (6 KB left)
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK1

ISBLANK1          LDAA    #' '          ; It's blank
                  STAA    TEN_THOUS     ; so store a space
                  BRA     C_THOU        ; and check the thousands digit

NOT_BLANK1        LDAA    TEN_THOUS     ; Get the ten_thousands digit
                  ORAA    #$30          ; Convert to ascii
                  STAA    TEN_THOUS
                  LDAA    #$1           ; Signal that we have seen a non-blank digit
                  STAA    NO_BLANK

C_THOU            LDAA    THOUSANDS     ; Check the thousands digit for blankness
                  ORAA    NO_BLANK      ; If it?s blank and no-blank is still zero
                  BNE     NOT_BLANK2

ISBLANK2          LDAA    #' '          ; Thousands digit is blank
                  STAA    THOUSANDS     ; so store a space
                  BRA     C_HUNS        ; and check the hundreds digit

NOT_BLANK2        LDAA    THOUSANDS     ; (similar to ten_thousands case)
                  ORAA    #$30
                  STAA    THOUSANDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_HUNS            LDAA    HUNDREDS      ; Check the hundreds digit for blankness
                  ORAA    NO_BLANK      ; If it's blank and no-blank is still zero
                  BNE     NOT_BLANK3

ISBLANK3          LDAA    #' '          ; Hundreds digit is blank
                  STAA    HUNDREDS       ; so store a space
                  BRA     C_TENS          ; and check the tens digit

NOT_BLANK3        LDAA    HUNDREDS          ; (similar to ten_thousands case)
                  ORAA    #$30
                  STAA    HUNDREDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_TENS            LDAA    TENS          ; Check the tens digit for blankness
                  ORAA    NO_BLANK      ; If it?s blank and no-blank is still zero
                  BNE     NOT_BLANK4

ISBLANK4          LDAA    #' '          ; Tens digit is blank
                  STAA    TENS          ; so store a space
                  BRA     C_UNITS       ; and check the units digit

NOT_BLANK4        LDAA    TENS          ; (similar to ten_thousands case)
                  ORAA    #$30
                  STAA    TENS

C_UNITS           LDAA    UNITS         ; No blank check necessary, convert to ascii.
                  ORAA    #$30
                  STAA    UNITS
                  RTS                 ; We?re done

;***************************************************************************************************
;***************************************************************************************************
;*                                Interrupt Vectors                                                *
;***************************************************************************************************
                  ORG     $FFFE
                  DC.W    Entry ; Reset Vector
                  ORG     $FFDE
                  DC.W    TOF_ISR ; Timer Overflow Interrupt Vector

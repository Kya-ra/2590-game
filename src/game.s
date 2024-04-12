# PASTE LINK TO TEAM VIDEO BELOW
#
# https://media.heanet.ie/page/e5892fb5219847fc94f58f274f7a1d77

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global  SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "./src/definitions.s"

  .equ    BLINK_PERIOD, 100

  .section .text

Main:
  PUSH  {R4-R12,LR}

  @
  @ Prepare GPIO Port E Pin 9 for output (LED LD3)
  @ We'll blink LED LD3 (the orange LED)
  @

  @ Enable GPIO port E by enabling its clock
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ Configure LD3 for output
  @   by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @   (by BIClearing then ORRing)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...

  BIC     R5, #(0b11<<(LD3_PIN*2))    @ Clear LED 3 bits
  ORR     R5, #(0b01<<(LD3_PIN*2))    @ write 01 to bits to set active

  BIC     R5, #(0b11<<(LD4_PIN*2))    @ Clear LED 4 bits
  ORR     R5, #(0b01<<(LD4_PIN*2))    @ write 01 to bits to set active
  
  BIC     R5, #(0b11<<(LD5_PIN*2))    @ Clear LED 5 bits
  ORR     R5, #(0b01<<(LD5_PIN*2))    @ write 01 to bits to set active

  BIC     R5, #(0b11<<(LD6_PIN*2))    @ Clear LED 6 bits
  ORR     R5, #(0b01<<(LD6_PIN*2))    @ write 01 to bits to set active

  BIC     R5, #(0b11<<(LD7_PIN*2))    @ Clear LED 7 bits
  ORR     R5, #(0b01<<(LD7_PIN*2))    @ write 01 to bits to set active

  BIC     R5, #(0b11<<(LD8_PIN*2))    @ Clear LED 8 bits
  ORR     R5, #(0b01<<(LD8_PIN*2))    @ write 01 to bits to set active
  
  BIC     R5, #(0b11<<(LD9_PIN*2))    @ Clear LED 9 bits
  ORR     R5, #(0b01<<(LD9_PIN*2))    @ write 01 to bits to set active

  BIC     R5, #(0b11<<(LD10_PIN*2))    @ Clear LED 10 bits
  ORR     R5, #(0b01<<(LD10_PIN*2))    @ write 01 to bits to set active

  STR     R5, [R4]                    @ Write 

  @ Initialise the first countdown
  LDR     R4, =blink_countdown
  LDR     R5, =blink_period
  LDR     R5, [R5]
  @ LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  

  @ LDR     R4, =blink_countdown
  @ LDR     R5, =blink_period
  @ STR     R5, [R4]  

  @ Configure SysTick Timer to generate an interrupt every 1ms

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 

  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1


  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @ We'll count the number of times the button is pressed
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  LDR     R4, =SYSCFG_EXTIICR1 @ Load systen configuration to EXTIICR register
  LDR     R5, [R4]            @ Load External Interrupt control
  BIC     R5, R5, #0b1111     @ Clear bits 3..0 of External Interrupt control
  STR     R5, [R4]            @ store new result in SYSCFG_EXTIICR1 

  @ Enable (unmask) interrupts on external interrupt Line0
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on Line0
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt #6 (external interrupt Line0)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]

  

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  B     Idle_Loop
  
End_Main:
  POP   {R4-R12,PC}


@ random subroutine
@ Generates a random number between a range using Linear-feedback shift register
@ 
@ Sources:
@   https://en.wikipedia.org/wiki/Linear-feedback_shift_register
@   https://www.youtube.com/watch?v=Ks1pw1X22y4
@
@ Paramaters:
@   R0: range - the range of numbers generated
@   R1: seed - the seed the prng uses
@
@ Return:
@   R0: random_number - the random number
random:
  PUSH    {R4-R8,LR}
  @ Save range-1
  SUB     R7, R0, #1  @ tempRange = range - 1
  MOV     R8, R1      @ tempSeed = seed
  @ Get amount of bits
  CMP     R0, #0               @ if range >= 0 {
  BGT     .LgreaterThanZero
  MOV     R0, #0               @ range = 0
  B       .LendRandom          @ }
.LgreaterThanZero:
  MOV     R5, #1               @ else { 
  MOV     R1, 0b1              @  numBits = 1; tempMax = 1;
.LbitGetterLoop:               @  
  CMP     R7, R1               @  for tempRange < ; tempMax -- {
  BLS     .LendBitGetterLoop   @ 
  LSL     R1, #1               @  tempMax = tempMax * 2;
  ORR     R1, #1               @  
  ADD     R5, #1               @  numBits++
  B       .LbitGetterLoop      @ } }
.LendBitGetterLoop:
  @ Load seed
  MOV     R1, R8               @ seed = tempSeed
  MOV     R6, #0               @ bitcount = 0;
  MOV     R0, #0               @ MaxRange = 0;
.LprnBitLoop:
  CMP     R6, R5
  BEQ     .LcheckIfBelow
  @ Compute the taps          tap number:
  @ tap 31:
  AND     R2, R1, 0x2
  LSR     R2, #1  @ 1 bit
  @ tap 6:
  AND     R3, R1, 0x2000000
  LSR     R3, #25 @ 25 bits
  EOR     R2, R2, R3
  @ tap 5:
  AND     R3, R1, 0x4000000
  LSR     R3, #26 @ 26 bits
  EOR     R2, R2, R3
  @ tap 1:
  AND     R3, R1, 0x40000000
  LSR     R3, #30 @ 30 bits
  EOR     R2, R2, R3
  @ add the new number to the end of the seed
  LSL     R2, R2, #31
  LSR     R1, #1
  ORR     R1, R1, R2
  @ add to the output number
  LSL     R0, #1
  AND     R3, R1, 0b1
  ORR     R0, R3
  ADD     R6, #1
  B       .LprnBitLoop
.LcheckIfBelow:
  CMP     R0, R7
  BLS     .LendRandom
  MOV     R6, #0
  MOV     R0, #0
  B       .LprnBitLoop
.LendRandom:
  POP     {R4-R8,PC}


@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4-R12, LR}

  LDR   R4, =tick       @ tickAddress = tickAddresslocation
  LDR   R5, [R4]        @ tickNum = tickAddress[0]
  CMP   R5, #0xffffffff @ if tickNum ==  Max { tickNum = 0 }
  BNE   .LnotMax        @ 
  LDR   R5, =0          
.LnotMax:
  ADD   R5, #1          @ tickNumm ++
  STR   R5, [R4]        @ tickAddress = ticknum


  LDR   R4, =blink_countdown        @ 
  LDR   R5, [R4]                    @ countdown = blink_countdown
.LdontInit:
  CMP   R5, #0                      @if (countdown != 0) {
  BEQ   .LelseFire                  @

  SUB   R5, R5, #1                  @   countdown--;
  STR   R5, [R4]                    @   blink_countDown = countdown

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {

@ Set up for game to begin
  LDR R9, =game_ready
  LDR R9, [R9]
  CMP R9, #1
  BNE .LendIfDelay

@ Inits starting LED
  LDR     R6, =led_position         @ startingLEDPos = led_position
  LDR     R3, [R6]
  LDR     R7, =led_state            @ startingLEDState = led_state
  LDR     R8, [R7]

  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, R3                    @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =blink_countdown      @  countLength = blink_count
  LDR     R5, =blink_period         @  countdown = blink_period;
  LDR     R5, [R5]
  STR     R5, [R4]                  @

  @ Checks if game should be active and resets if a problem arises
  LDR     R9, =game_active          @ isActive = game_active
  LDR     R9, [R9]                  @
  CMP     R9, #0                    @ if isActive == false{start Countdown for next game beginning}
  BEQ     .LstartCountdown
  CMP     R8, #0                    @ if (isActive && startLEDOn = false ) { reset  }
  BEQ     .LnoReset

  @ Checks if Game Should Be reversed and Runs result
  LDR     R9, =play_direction      @ isReversed = play_direction
  LDR     R9, [R9]   
  CMP     R9, #0                   @ if isReversed = true{reverseGame() } 
  BEQ     .LreversedGame


@ Normal function for LED movement
  LSL     R3, R3, #1               @ currentLEDPos += 1 (LSL moves to next LED)
  CMP     R3, #0x8000              @ if currentLEDPos >= LED10 {reset to LED 3}
  BLE     .LnoReset                @ LEDOff()
  MOV     R3, #0x100
  B       .LnoReset                @ LEDOff()
  
.LreversedGame:
@ Reverse function for LED movement
  LSR     R3, R3, #1              @ currentLEDPos += 1 (LSL moves to next LED)
  CMP     R3, #0x100              @ if currentLEDPos < LED3 {reset to LED 10}
  BGE     .LnoReset               @ LEDOff()
  MOV     R3, #0x8000

.LnoReset:
@ Turns previously lit LED off
  RSB     R8, R8, #1              @ currentLEDOn = false
  STR     R8, [R7]                @ led_state = currentLEDOn
  STR     R3, [R6]                @ led_position = currentLEDPos
  B       .LendIfDelay

.LstartCountdown:
@ Counts down to start of game 
  LDR     R8, =game_start_count   @ startCountLen = game_start_count
  LDR     R9, [R8]                
  SUB     R9, R9, #1              @ startCountLen -= 1
  STR     R9, [R8]                @ game_start_count = startCountLen
  CMP     R9, #1                  @ if startCountLen != 1 {endGame()}
  BNE     .LendIfDelay
  LDR     R8, =game_active        @ else {gameActive = game_active}
  STR     R9, [R8]                @

.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @


  @ Return from interrupt handler
  POP     {R4-R12, PC}

@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH    {R4-R12,LR}

  LDR     R4, =button_count           @ count = count + 1
  LDR     R5, [R4]                    @
  ADD     R5, R5, #1                  @
  STR     R5, [R4]                    @

  LDR     R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV     R5, #(1<<0)                 @
  STR     R5, [R4]                    @
  
  LDR     R9, =game_ready             @ gameReadyCheck = gameReadyLocation
  LDR     R10, [R9]                   @ isGameReadyReg = gameReadyCheck
  CMP     R10, #0                     @ if isGameReadyTemp == 0 {generateRandomNum()}
  BEQ     .LgenerateRandoms

  LDR     R6, =led_position           @ tempLED = LED_PositionLocation
  LDR     R7, [R6]                    @ LEDValue = tempLED[0]
  LDR     R8, =win_led                @ tempValue = win_LEDLocation
  LDR     R8, [R8]                    @ tempValue = tempValue[0]
  CMP     R7, R8                      @ if tempValue != LEDValue (fail)
  BNE     .Lfail
  MOV     R7, #0x800                  @ Light Loss LED
  STR     R7, [R6]                    @ Store LED positon
  B       .Lexit
.Lfail:
  MOV     R7, #0x200                  @ Light Win LED
  STR     R7, [R6]                    @ Store LED position
.Lexit:
  MOV     R9, #0                      @ gameReadyCheck = false;
  LDR     R10, =game_active           @ gameActiveTemp = gameActiveLocation
  STR     R9, [R10]                   @ store gameActiveCheck in memory
  B     .LendSub

.LgenerateRandoms:
  @initalize count
  MOV   R0, #50                       @ RNGrange = 50
  LDR   R2, =tick                     @ tickNumTemp = tickNumMem
  LDR   R1, [R2]                      @ seed = tickNumTenp
  BL    random                        @ RNG(range, seed )
  ADD   R0, R0, #20                   @ randomNum += 20
  LDR   R1, =blink_period             
  STR   R0, [R1]
  @initalize win_led
  MOV     R0, #8                      @ range = 8
  LDR     R1, =tick                   
  LDR     R1, [R1]                    @ seed = numticks
  BL      random                      @ random(range, seed)
  @correct to 8-15 range reqd
  ADD     R0, #8                      @ randomNum += 8
  MOV     R1, #1                      @ Win_LEDLocation = 1 
  LSL     R1, R0                      @ Win_LEDLocation = 2 ^ randomNum (= LED (randomNum))
  LDR     R2, =win_led                @ Win_LEDMemTemp = Win_LEDMem
  STR     R1, [R2]                    @ Win_LEDMemTemp[0] = Win_LEDLocatiom
  LDR     R2, =led_position           @ Win_LEDMemTemp = Win_LEDPosMem
  STR     R1, [R2]                    @ Win_LEDMemTemp[0] = Win_LEDLocation

  @initialize play_direction
  MOV     R0, #2                      @ range = 2
  LDR     R1, =tick                    
  LDR     R1, [R1]                    @ seed = numTicks
  BL      random                      @ random(range, seed)
  LDR     R1, =play_direction         @ PlayDirectionTemp = PlayDirectionTempMem
  STR     R0, [R1]                    @ PlatDirectionMem[0] = randomNum 

  MOV     R10, #1                     @ gameReadyTemp = true
  LDR     R9, =game_ready             @
  STR     R10, [R9]                   @ gameReadyFlag = gameReadyTemp
.LendSub:

  @ Return from interrupt handler
  POP     {R4-R12,PC}


  .section .data
  
button_count:
  .space 4

blink_countdown:
  .space 4

led_position:
  .space 4

led_state:
  .word 0x0

win_led:
  .space 4

game_active:
  .word 0x0

game_ready:
  .word 0x0

play_direction:
  .space 4

game_start_count:
  .word 0xd
  @this number must be odd
blink_period:
  .word 100
  @ .equ    BLINK_PERIOD, 100

tick:
  .word 0

  .end
# PASTE LINK TO TEAM VIDEO BELOW
#
#

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global  SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "./src/definitions.s"

  .equ    BLINK_PERIOD, 150

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

  BIC     R5, #(0b11<<(LD3_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))    @ write 01 to bits 

  BIC     R5, #(0b11<<(LD4_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD4_PIN*2))    @ write 01 to bits 
  
  BIC     R5, #(0b11<<(LD5_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD5_PIN*2))    @ write 01 to bits 

  BIC     R5, #(0b11<<(LD6_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD6_PIN*2))    @ write 01 to bits 

  BIC     R5, #(0b11<<(LD7_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD7_PIN*2))    @ write 01 to bits 

  BIC     R5, #(0b11<<(LD8_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD8_PIN*2))    @ write 01 to bits 
  
  BIC     R5, #(0b11<<(LD9_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD9_PIN*2))    @ write 01 to bits 

  BIC     R5, #(0b11<<(LD10_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD10_PIN*2))    @ write 01 to bits 

  STR     R5, [R4]                    @ Write 

  @ Initialise the first countdown

  LDR     R4, =blink_countdown
  LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  

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
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

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

  @initalize win_led
  MOV R0, #8
  BL random
  @correct to 8-15 range reqd
  ADD R0, #8
  MOV R1, #1
  LSL R1, R0
  LDR R2, =win_led
  STR R1, [R2]
  LDR R2, =led_position
  STR R1, [R2]

  @initialize play_direction
  MOV R0, #2
  BL random
  LDR R1, =play_direction
  STR R0, [R1]

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
@
@ Return:
@   R0: random_number - the random number
random:
  PUSH    {R4-R7,LR}
  @ Save range-1
  SUB     R7, R0, #1
  @ Get amount of bits
  CMP     R0, #0
  BGT     .LgreaterThanZero
  MOV     R0, #0
  B       .LendRandom
.LgreaterThanZero:
  MOV     R5, #1
  MOV     R1, 0b1
.LbitGetterLoop:
  CMP     R7, R1
  BLS     .LendBitGetterLoop     
  LSL     R1, #1
  ORR     R1, #1
  ADD     R5, #1
  B       .LbitGetterLoop
.LendBitGetterLoop:
  @ Load seed
  LDR     R4, =random_seed
  LDR     R1, [R4]
  MOV     R6, #0
  MOV     R0, #0
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
  BLS     .LendRandomAndSave
  MOV     R6, #0
  MOV     R0, #0
  B       .LprnBitLoop
.LendRandomAndSave:
  @ Save seed
  STR     R1, [R4]
.LendRandom:
  POP     {R4-R7,PC}


@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4-R12, LR}

  LDR   R4, =blink_countdown        @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @

  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {

  LDR     R6, =led_position
  LDR     R3, [R6]
  LDR     R7, =led_state
  LDR     R8, [R7]

  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, R3                    @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =blink_countdown      @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD         @
  STR     R5, [R4]                  @

  LDR R9, =game_active
  LDR R9, [R9]
  CMP R9, #0
  BEQ .LendIfDelay
  CMP R8, #0
  BEQ .LnoReset

  LDR R9, =play_direction
  LDR R9, [R9]
  CMP R9, #0
  BEQ .LreversedGame

  LSL R3, R3, #1
  CMP R3, #0x8000
  BLE .LnoReset
  MOV R3, #0x100
  B .LnoReset
  
.LreversedGame:
  LSR R3, R3, #1
  CMP R3, #0x100
  BGE .LnoReset
  MOV R3, #0x8000

  .LnoReset:
  RSB R8, R8, #1
  STR R8, [R7]
  STR R3, [R6]

.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4-R12, PC}



@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R4-R12,LR}

  LDR   R4, =button_count           @ count = count + 1
  LDR   R5, [R4]                    @
  ADD   R5, R5, #1                  @
  STR   R5, [R4]                    @

  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @

  LDR R6, =led_position
  LDR R7, [R6]
  LDR R8, =win_led
  LDR R8, [R8]
  CMP R7, R8
  BNE .Lfail
  MOV R7, #0x800
  STR R7, [R6]
  B .Lexit
  .Lfail:
  MOV R7, #0x200
  STR R7, [R6]
  .Lexit:
  MOV R9, #0
  LDR R10, =game_active
  STR R9, [R10]
  @ Return from interrupt handler
  POP  {R4-R12,PC}

  .section .data
  
button_count:
  .space  4

blink_countdown:
  .space  4

led_position:
  .space 4

led_state:
  .word 0x0

win_led:
  .space 4

game_active:
  .word 0x1

play_direction:
  .space 4

random_seed:
  .word 0xca660da9

  .end
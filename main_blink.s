	.syntax	unified
	.arch   armv8-m.main
    .thumb

    .list

    SCS_BASE     = (0xE000E000UL)          /*!< System Control Space Base Address */
    ITM_BASE     = (0xE0000000UL)          /*!< ITM Base Address */
    DWT_BASE     = (0xE0001000UL)          /*!< DWT Base Address */
    TPIU_BASE    = (0xE0040000UL)          /*!< TPIU Base Address */
    DCB_BASE     = (0xE000EDF0UL)          /*!< DCB Base Address */
    DIB_BASE     = (0xE000EFB0UL)          /*!< DIB Base Address */
    SysTick_BASE = (SCS_BASE +  0x0010UL)  /*!< SysTick Base Address */
    NVIC_BASE    = (SCS_BASE +  0x0100UL)  /*!< NVIC Base Address */
    SCB_BASE     = (SCS_BASE +  0x0D00UL)  /*!< System Control Block Base Address */

    SYSCON0_BASE = 0x40000000 // SYSCON0 base address: 4000_0000h
    GPIO_BASE = 0x40096000

    GPIO_PDDR_OFFSET = 0x54 // Port direction register
    GPIO_PDOR_OFFSET = 0x44 // Port Output data Register
    GPIO_PSOR_OFFSET = 0x44 // Port Output SET Register
    GPIO_PCOR_OFFSET = 0x48 // Port Output Clear Register
    GPIO_PTOR_OFFSET = 0x4c // Toggle output

    // ==========================================================================
    // The Main entry _main_blink()
    // ==========================================================================
	.section .text
    .align 4
    .global _main_blink
    .thumb_func

_main_blink:
    bl _initLed
    bl _initSysTick

.loop:
    bl .delay
    b .loop

    .align 4
.delay:
    ldr r12,=10*30000

.del_loop:

    subs r12,#1
    bne .del_loop
    blx lr

    // ==========================================================================
    // initSysTick() code
    // ==========================================================================
	.section .text
    .align 4
    .thumb_func

_initSysTick:
    push {r0,r1,lr}
    ldr r0,=SysTick_BASE

    eors r1,r1
    str r1,[r0,0x0] // CTRL
    str r1,[r0,0x8] // VAL

    ldr r1,=48000000 / 1000
    sub r1,#1
    str r1,[r0,0x4] // LOAD

    mov r1,#(1<<0) | (1<<1) | (1<<2)
    str r1,[r0,0x0] // CTRL

    /*
        typedef struct
        {
          __IOM uint32_t CTRL;   // Offset: 0x000 (R/W)  SysTick Control and Status Register
          __IOM uint32_t LOAD;   // Offset: 0x004 (R/W)  SysTick Reload Value Register
          __IOM uint32_t VAL;    // Offset: 0x008 (R/W)  SysTick Current Value Register
          __IM  uint32_t CALIB;  // Offset: 0x00C (R/ )  SysTick Calibration Register
        } SysTick_Type;
    */

    pop {r0,r1,pc}

    // ==========================================================================
    // initLed()
    // ==========================================================================
	.section .text
    .align 4
    .thumb_func
_initLed:

    ldr r0,=SYSCON0_BASE
    ldr r1,[r0, 0x200] // AHBCLKCTRL0
    orr r1,#(0b11<<19)
    str r1,[r0, 0x200] // AHBCLKCTRL0

// • Red LED connects to target MCU pin P0_10
// • Green LED connects to target MCU pin P0_27
// • Blue LED connects to target MCU pin P1_2

    // =================================================================================
    // RED and GREEN Led (@GPIO0)
    // =================================================================================
    ldr r0,=(GPIO_BASE + 0x0000)

    ldr r2,=(( 1u<<10 ) | ( 1u<<27 ))

    ldr r1,[r0, GPIO_PDOR_OFFSET] // Set Output data register
    orr r1,r2
    str r1,[r0, GPIO_PDOR_OFFSET] // Set Output data register

    ldr r1,[r0, GPIO_PDDR_OFFSET] // Set Output direction Register
    orr r1,r2
    str r1,[r0, GPIO_PDDR_OFFSET] // Set Output direction Register

    // =================================================================================
    // Blue Led (@GPIO1)
    // =================================================================================
    ldr r0,=(GPIO_BASE +  0x2000)

    mov r2,#(1u<<2)

    ldr r1,[r0, GPIO_PDOR_OFFSET] // Set Output data register
    orr r1,r2
    str r1,[r0, GPIO_PDOR_OFFSET] // Set Output data register

    ldr r1,[r0, GPIO_PDDR_OFFSET] // Set Output direction Register
    orr r1,r2
    str r1,[r0, GPIO_PDDR_OFFSET] // Set Output direction Register

    blx lr

    // ==========================================================================
    // The Systick Handler
    // ==========================================================================
	.section .text
    .align 4
    .global SysTick_Handler
    .thumb_func

    .extern ledCnt

SysTick_Handler:
    push {r0,r1,r2,r3,lr}

    ldr r3,=ledCnt
    ldr r2,[r3]
    cmp r2,#0
    beq _doBlink
    subs r2,#1

    b _exitSystickISR
_doBlink:
    mov r2,#20-1

    ldr r0,=GPIO_BASE
    mov r1,#(1u<<10)
    str r1,[r0, GPIO_PTOR_OFFSET] // Toggle

_exitSystickISR:
    str r2,[r3]

    pop {r0,r1,r2,r3,pc}

    // ==========================================================================
    // Store the ledCnt in the BSS section
    // ==========================================================================
    .section .bss
    .align 4
    .global ledCnt

ledCnt:
    .ds 256

    .end

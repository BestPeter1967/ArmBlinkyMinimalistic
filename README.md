## Minemalistic Blinky in arm assembly

**Author:**

Hans-Peter Beständig
Kühbachstr. 8
**81543 Munich**

Germany

mailto: hdusel@tangerine-soft.de

## Rationale

This is a Minimalistic Project to run Blinky on a [FRDM-MCXN947](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXN947) evaluation board.

#### About the Target Hardware

The  [FRDM-MCXN947](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXN947) contains a [N947](https://www.nxp.com/part/MCXN947VDFT#/) that is an arm Cortex-M33 MCU. 

This MCU belongs to the NXP [MCXN947 MCU Family](https://www.nxp.com/products/MCX-N94-N54-N53-N52-N24)

- The [MCXNx4x Product Data Sheet](https://www.nxp.com/docs/en/data-sheet/MCXNx4xDS.pdf)

## Projects scope

This little project is solely written in **arm assembly code** since this is the only way to be minimalistic

It uses a GCC Compiler-toolchain (its asm and linker) to build (assemble) the code into an *armabi elf* executable.

The project exists of 4 files:

- The **main** "blinky-Code": `main_blink.s`
- a **Startup code**: `mcxn947_startup.s`
- a **Makefile** to build the code
- ...and finally this `README.md`;-)

## How to build the Project
To build the Project you'll need a [GCC Crosscompiler Toolchain](https://wiki.osdev.org/GCC_Cross-Compiler) that is capable to generate target code for a [arm Cortex-M33 MCU](https://developer.arm.com/Processors/Cortex-M33).

It is assumed that your development PC already has installed this toolchain and the path variable is set accordingly.

Furthermore the PC needs an istallation of (GNU) Make

**Open a Console `cd` to the project and type `make`**

~~~shell
$ make
arm-none-eabi-as -a=obj/mcxn947_startup.o.lst mcxn947_startup.s -o obj/mcxn947_startup.o -mcpu=cortex-m33
arm-none-eabi-ld -Ttext 0 -Tdata 0x20000000 -nostdlib obj/mcxn947_startup.o obj/main_blink.o -o main_blink.elf --print-memory-usage -Map main_blink.elf.map
Memory region         Used Size  Region Size  %age Used
#arm-none-eabi-objcopy main_blink.elf -O ihex main_blink.hex
~~~

Afterward you'll get an elf binary `main_blink.elf`, a matching Intel HEX File (that you may use to flash the target) `main_blink.hex` and a build frsgments **linker map** file `main_blink.elf.map`

## Appendix 

## ### Overview

#### `main_blink.s`

~~~asm
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
~~~


#### `mcxn947_startup.s`

~~~asm
/* ==============================================================================
 * Startup-Code for a NXP mcxn947 MCU
 * see: https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXN947
 * ============================================================================== */
	.syntax	unified
	.arch   armv8-m.main
    .list


    RAMSTART = 0x20000000ul
    RAMSIZE =  32*1024
    RAMEND = RAMSTART + RAMSIZE

    STACKTOP = RAMEND

    .org 0x00000000

	.section .text

.macro ISR_DEFAULT label
    .global \label
    .weak \label
    .thumb_set \label,Default_Handler
.endm

/*
 * The common arm Cortex-M Interrupt Vector Table!
 * This is the minimum Table for a arm Cortex MCU!
 */
vectors:
    .long   STACKTOP /* Initial SP Value */
    .long   Reset_Handler   /* RESET Vector */
    .long   NMI_Handler
    .long   HardFault_Handler
    .long   MemoryManagement_Handler
    .long   BusFault_Handler
    .long   UsageFault_Handler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SVC_Handler
    .long   DebugMon_Handler
    .long   0                           /*Reserved */
    .long   PendSV_Handler
    .long   SysTick_Handler

    .ds.l 138,Default_Handler /* This chip has 138 custom Interrupts */

    /* ==========================================================================
     * The Reset Handler: Here the Party begins!
     * This is the Reset Vector which is the second entry of vectors (see above)
     * ========================================================================== */
    .thumb
    .section .text
    .align

    .global _start
    .thumb_set _start,Reset_Handler
    .thumb_func

Reset_Handler:
    .extern _main_blink
    mrs r0, msp
    sub r0,#512
    msr psp,r0

    bl _enableCache
    bl _enableFPU
    bl _initBSS

    cpsie I // Enable Interrupts

    bl _main_blink
    b . // Stay here forever!

    // ==========================================================================
    // Install the Default handlers for all ISR.
    // since these are weak linked custom code may override them on demand!
    // ==========================================================================
    ISR_DEFAULT NMI_Handler
    ISR_DEFAULT HardFault_Handler
    ISR_DEFAULT MemoryManagement_Handler
    ISR_DEFAULT BusFault_Handler
    ISR_DEFAULT UsageFault_Handler
    ISR_DEFAULT SVC_Handler
    ISR_DEFAULT DebugMon_Handler
    ISR_DEFAULT PendSV_Handler
    ISR_DEFAULT SysTick_Handler

    // ==========================================================================
    // The default Handler for all unitialized vectors (weak linked)
    // ==========================================================================
    .align 4
    .thumb_func

Default_Handler:
    bkpt #0 /* Breakpoint */
    b .

    // ==========================================================================
    // The code that initaializes the BSS Section.
    // This is filled with 0 bytes. This is called by the Reset_Handler.
    // ==========================================================================
    .align 4
    .thumb_func

_initBSS:
    ldr r1,=__bss_start__
    ldr r2,=__bss_end__
    eors r0,r0

_initBSS_loop:
    cmp r1,r2
    bge _initBSS_loop_exit
    str r0,[r1], #4
    b _initBSS_loop

_initBSS_loop_exit:
    blx lr

    // ==========================================================================
    // Enable the FPU (actually we do not need it in this example yet)
    // ==========================================================================
    .align 4
    .thumb_func
_enableFPU:
    CPACR   =  0xe000ed88
    ldr     r0,  =CPACR          // Read CPACR
    ldr     r1, [r0]             // Set bits 20-23 to enable CP10 and CP11 coprocessors
    orr     r1, r1, #(0xf << 20)
    str     r1, [r0]             // Write back the modified value to the CPACR
    dsb
    isb                          // Reset pipeline now the FPU is enabled.
    blx lr

    // ==========================================================================
    // The NXP MCX Enable the CPU Caches.
    // ==========================================================================
    .align 4
    .thumb_func
_enableCache:
    SYSCON_BASE_ADDRESS   =  0x40000000
    NVM_CTRL_OFFSET   =  0x400

    ldr     r0,  =SYSCON_BASE_ADDRESS
    ldr     r1, [r0, NVM_CTRL_OFFSET]             // Set bits 20-23 to enable CP10 and CP11 coprocessors
    // Clear Flash Cache
    orr     r2, r1, #1<<5 // Clear Flash Cache
    str     r2, [r0, NVM_CTRL_OFFSET]             // Write back the modified value to the NVM_CTRL

    // Enable data and instruction Cache
    bic     r1, #0x1f // Enable data and instruction cache as well speculative execution
    str     r1, [r0, NVM_CTRL_OFFSET]             // Write back the modified value to the CPACR

    dsb
    isb                          // Reset pipeline now the Cache is enabled.
    blx lr

    .end

~~~

#### The Makefile

~~~makefile
OBJDIR = obj

${OBJDIR}/%.o : %.s
	@mkdir -p $(@D)
	arm-none-eabi-as -a=$@.lst $< -o $@ -mcpu=cortex-m33

main_blink.elf : ${OBJDIR}/main_blink.o ${OBJDIR}/mcxn947_startup.o
	arm-none-eabi-ld -Ttext 0 -Tdata 0x20000000 -nostdlib ${OBJDIR}/mcxn947_startup.o ${OBJDIR}/main_blink.o -o $@ --print-memory-usage -Map $@.map
	#arm-none-eabi-objcopy $@ -O ihex main_blink.hex

${OBJDIR}/main_blink.o : main_blink.s
${OBJDIR}/mcxn947_startup.o : mcxn947_startup.s


clean :
	rm -rf ${OBJDIR}/*.o

clobber : clean
	rm -rf *.lst *.out *.elf *.hex *.map

~~~


/* ==============================================================================
 * Startup-Code for a NXP mcxn947 MCU
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

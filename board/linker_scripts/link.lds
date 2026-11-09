/*
 * STM32H7 memory layout for GNU ld
 * bernard.xiong 2009-10-14
 */

/* Program Entry, set to mark it as "used" and avoid gc */
MEMORY
{
    ROM (rx) : ORIGIN = 0x08000000, LENGTH = 1024k /* 1024KB flash */
    RAM (rw) : ORIGIN = 0x24000000, LENGTH =  320k /* 320K sram */
    DTCM (rw) : ORIGIN = 0x20000000, LENGTH = 128k
    DMA_RAM (rw) : ORIGIN = 0x30000000, LENGTH = 32k
}
ENTRY(Reset_Handler)
_system_stack_size = 0x1000;

SECTIONS
{
    .logger_stack (NOLOAD) : ALIGN(8)
    {
        *(.logger_stack)
    } > DTCM
    ASSERT(SIZEOF(.logger_stack) <= 4k, "Flight logger stack exceeds 4 KiB")

    .logger_queue (NOLOAD) : ALIGN(8)
    {
        *(.logger_queue)
    } > DTCM
    ASSERT(SIZEOF(.logger_queue) <= 1k, "Flight logger queue exceeds 1 KiB")

    .flash_work (NOLOAD) : ALIGN(8)
    {
        *(.flash_work)
    } > DTCM
    ASSERT(SIZEOF(.flash_work) <= 4k, "Flight NOR work area exceeds 4 KiB")

    .estimator_cores (NOLOAD) : ALIGN(8)
    {
        *(.estimator_cores)
    } > DTCM
    ASSERT(SIZEOF(.estimator_cores) <= 30k, "Flight estimator cores exceed 30 KiB")

    .device_dma (NOLOAD) : ALIGN(32)
    {
        __device_dma_start = .;
        *(.device_dma)
        . = ALIGN(32);
        __device_dma_end = .;
    } > DMA_RAM
    ASSERT(SIZEOF(.device_dma) <= 12k, "Flight DMA arena exceeds 12 KiB")
    .text :
    {
        . = ALIGN(4);
        _stext = .;
        KEEP(*(.isr_vector))            /* Startup code */

        . = ALIGN(4);
        *(.text)                        /* remaining code */
        *(.text.*)                      /* remaining code */
        *(.rodata)                      /* read-only data (constants) */
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        *(.gnu.linkonce.t*)

        /* section information for finsh shell */
        . = ALIGN(4);
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;

        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;

        . = ALIGN(4);
        __rt_utest_tc_tab_start = .;
        KEEP(*(UtestTcTab))
        __rt_utest_tc_tab_end = .;

        /* section information for initial. */
        . = ALIGN(4);
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;

        . = ALIGN(4);

        PROVIDE(__ctors_start__ = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        PROVIDE(__ctors_end__ = .);

        . = ALIGN(4);

        _etext = .;
    } > ROM = 0

    /* .ARM.exidx is sorted, so has to go in its own output section.  */
    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > ROM
    __exidx_end = .;

    /* Initialize RAM from the load image in ROM at startup. */
    .data :
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _sdata = . ;

        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d*)

        PROVIDE(__dtors_start__ = .);
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        PROVIDE(__dtors_end__ = .);

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _edata = . ;
    } > RAM AT > ROM
    _sidata = LOADADDR(.data);

    .stack :
    {
        . = ALIGN(4);
        _sstack = .;
        . = . + _system_stack_size;
        . = ALIGN(4);
        _estack = .;
    } >RAM

    __bss_start = .;
    .bss :
    {
        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;

        *(.bss)
        *(.bss.*)
        *(COMMON)

        . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _ebss = . ;

        *(.bss.init)
    } > RAM
    __bss_end = .;

    _end = .;
    ASSERT(ORIGIN(RAM) + LENGTH(RAM) - _end >= 64k,
           "Flight AXI SRAM static headroom is below 64 KiB")

    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment       0 : { *(.comment) }
    /* DWARF debug sections.
     * Symbols in the DWARF debugging sections are relative to the beginning
     * of the section so we begin them at 0.  */
    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
}

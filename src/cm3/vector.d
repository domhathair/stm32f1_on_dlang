/**
 * Authors: Alexander Chepkov
 *
 * License: Distributed under the
 *      $(LINK2 http://www.boost.org/LICENSE_1_0.txt, Boost Software License 1.0).
 *    (See accompanying file LICENSE)
 */

module cm3.vector;

public import ldc.attributes;
import std.meta : Alias;

version (cm3):
extern (C):
@nogc:
nothrow:

alias funcp_t = void function();
__gshared funcp_t /*  */
__preinit_array_start, __preinit_array_end,
__init_array_start, __init_array_end,
__fini_array_start, __fini_array_end;

__gshared size_t /*  */
__data_loadaddr,
__text,
__etext,
__data,
__edata,
__ebss,
__end,
__stack;

static assert(size_t.sizeof == uint.sizeof);
static assert(ubyte.sizeof == 1);
static assert(ushort.sizeof == 2);
static assert(uint.sizeof == 4);
static assert(ulong.sizeof == 8);

@weak int init() => 0;
@weak void main();
@weak void blocking_isr() => panic("The system encountered a critical error");
@weak void null_handler() => cast(void)0;
@weak void panic(string msg) {
    while (1) {
    }
}

@weak void reset_handler() {
    size_t* src, dest;
    funcp_t* fp;

    /* copy .data section to SRAM */
    for (src = &__data_loadaddr, dest = &__data; dest < &__edata; src++, dest++)
        *dest = *src;

    /* .bss section initialization */
    while (dest < &__ebss)
        *dest++ = 0;

    /* Ensure 8-byte alignment of stack pointer on interrupts
	 * Enabled by default on most Cortex-M parts, but not M3 r1 */
    alias SCB_CCR = Alias!(cast(size_t*)0xE000ED14);
    *SCB_CCR |= (1 << 9);

    /* Constructors. */
    for (fp = &__preinit_array_start; fp < &__preinit_array_end; fp++)
        (*fp)();

    for (fp = &__init_array_start; fp < &__init_array_end; fp++)
        (*fp)();

    /* Call the application's entry point. */
    if (init() == 0)
        main();

    /* Destructors. */
    for (fp = &__fini_array_start; fp < &__fini_array_end; fp++) {
        (*fp)();
    }

    while (1) {
    }
}

@weak void __aeabi_read_tp() => cast(void)0;
@weak void __aeabi_unwind_cpp_pr0() => cast(void)0;
@weak void __aeabi_unwind_cpp_pr1() => cast(void)0;
@weak void __tls_get_addr() => cast(void)0;
@weak void _d_arraybounds(string file, uint line) => cast(void)0;
@weak void _d_arraybounds_index(string file, uint line, uint index, uint length) => cast(void)0;
@weak void _d_assert(string file, uint line) => panic("Assertion failed");
@weak noreturn __assert(const(char)* exp, const(char)* file, int line) {
    while (1) {
    }
}

enum CM3 : size_t {
    RESET = 0,
    NMI = 1,
    HARD_FAULT = 2,
    MEMORY_FAULT = 3,
    BUS_FAULT = 4,
    USAGE_FAULT = 5,
    SV_CALL = 10,
    DEBUG_MONITOR = 11,
    PEND_SV = 13,
    SYS_TICK = 14
}

private enum SHIFT = CM3.max + 1;

enum NVIC : size_t {
    WWDG = SHIFT,
    PVD,
    TAMPER,
    RTC,
    FLASH,
    RCC,
    EXTI0,
    EXTI1,
    EXTI2,
    EXTI3,
    EXTI4,
    DMA1_CHANNEL1,
    DMA1_CHANNEL2,
    DMA1_CHANNEL3,
    DMA1_CHANNEL4,
    DMA1_CHANNEL5,
    DMA1_CHANNEL6,
    DMA1_CHANNEL7,
    ADC1_2,
    USB_HP_CAN_TX,
    USB_LP_CAN_RX0,
    CAN_RX1,
    CAN_SCE,
    EXTI9_5,
    TIM1_BRK,
    TIM1_UP,
    TIM1_TRG_COM,
    TIM1_CC,
    TIM2,
    TIM3,
    TIM4,
    I2C1_EV,
    I2C1_ER,
    I2C2_EV,
    I2C2_ER,
    SPI1,
    SPI2,
    USART1,
    USART2,
    USART3,
    EXTI15_10,
    RTC_ALARM,
    USB_WAKEUP,
    TIM8_BRK,
    TIM8_UP,
    TIM8_TRG_COM,
    TIM8_CC,
    ADC3,
    FSMC,
    SDIO,
    TIM5,
    SPI3,
    UART4,
    UART5,
    TIM6,
    TIM7,
    DMA2_CHANNEL1,
    DMA2_CHANNEL2,
    DMA2_CHANNEL3,
    DMA2_CHANNEL4,
    DMA2_CHANNEL5,
    ETH,
    ETH_WKUP,
    CAN2_TX,
    CAN2_RX0,
    CAN2_RX1,
    CAN2_SCE,
    OTG_FS
}

private string generate_irq(T, size_t shift)() {
    import std.conv : to;

    alias members = __traits(allMembers, T);
    debug pragma(msg, members);

    string irq_str = "enum IRQ : size_t {\n";
    foreach (i, member; members) {
        enum ov = mixin("T." ~ member).to!size_t;
        enum sv = ov - shift;
        irq_str ~= member ~ " = " ~ to!string(sv) ~ ",\n";
    }
    irq_str ~= "}\n";

    return irq_str;
}

private string generate_isr(T, string postfix, string returns)() {
    alias members = __traits(allMembers, T);
    debug pragma(msg, members);

    string isr_str;
    foreach (member; members) {
        char[] lower_member = member.dup;
        foreach (ref c; lower_member)
            if (c >= 'A' && c <= 'Z')
                c = cast(char)(c + ('a' - 'A'));
        isr_str ~= "@weak void " ~ lower_member ~ "_" ~ postfix ~ "() => " ~ returns ~ "_" ~ postfix ~ "();\n";
    }

    return isr_str;
}

static mixin(generate_irq!(NVIC, SHIFT));
static mixin(generate_isr!(CM3, "handler", "null"));
static mixin(generate_isr!(NVIC, "isr", "blocking"));

struct vector_table_entry_t {
    size_t* initial_sp_value;
    immutable void function()[NVIC.max + 1] isr_vectors;
}

private string generate_vectors(T, string postfix)() {
    alias members = __traits(allMembers, T);

    string vector_str;
    foreach (member; members) {
        char[] lower_member = member.dup;
        foreach (ref c; lower_member)
            if (c >= 'A' && c <= 'Z')
                c = cast(char)(c + ('a' - 'A'));
        vector_str ~= T.stringof ~ "." ~ member ~ ": &" ~ lower_member ~ "_" ~ postfix ~ ",\n";
    }

    return vector_str;
}

static mixin(
    "@(section(\".vectors\")) static const vector_table_entry_t vector_table = {\n" ~
        "initial_sp_value: &__stack,\n" ~
        "isr_vectors: [\n" ~ generate_vectors!(CM3, "handler") ~ generate_vectors!(NVIC, "isr") ~
        "]};\n");

static assert(vector_table.sizeof == 0x150);

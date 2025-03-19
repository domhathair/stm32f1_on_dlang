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
@weak void blocking_handler() => panic("The system encountered a critical error");
@weak void null_handler() => cast(void)0;
@weak void panic(string mesg) {
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
	MEMORY_MANAGE_FAULT = 3,
	BUS_FAULT = 4,
	USAGE_FAULT = 5,
	SV_CALL = 10,
	DEBUG_MONITOR = 11,
	PEND_SV = 13,
	SYS_TICK = 14
}

@weak void nmi_handler() => null_handler();
@weak void hard_fault_handler() => null_handler();
@weak void mem_manage_handler() => null_handler();
@weak void bus_fault_handler() => null_handler();
@weak void usage_fault_handler() => null_handler();
@weak void sv_call_handler() => null_handler();
@weak void debug_monitor_handler() => null_handler();
@weak void pend_sv_handler() => null_handler();
@weak void sys_tick_handler() => null_handler();

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
	DMA2_CHANNEL4_5,
	DMA2_CHANNEL5,
	ETH,
	ETH_WKUP,
	CAN2_TX,
	CAN2_RX0,
	CAN2_RX1,
	CAN2_SCE,
	OTG_FS
}

private static string generate_shifted_enum(T, string name, string op, size_t shift)() {
	import std.conv : to;

	alias members = __traits(allMembers, T);
	debug pragma(msg, members);

	string enum_str = "enum " ~ name ~ " : size_t {\n";

	foreach (i, member; members) {
		enum ov = mixin("T." ~ member).to!size_t;
		mixin("enum sv = ov " ~ op ~ " shift;");
		enum_str ~= "	" ~ member ~ " = " ~ to!string(sv) ~ ",\n";
	}
	enum_str ~= "}\n";

	return enum_str;
}

mixin(generate_shifted_enum!(NVIC, "IRQ", "-", SHIFT));

struct vector_table_entry_t {
	size_t* initial_sp_value;
	immutable void function()[NVIC.max + 1] isr_vectors;
}

@weak void wwdg_isr() => blocking_handler();
@weak void pvd_isr() => blocking_handler();
@weak void tamper_isr() => blocking_handler();
@weak void rtc_isr() => blocking_handler();
@weak void flash_isr() => blocking_handler();
@weak void rcc_isr() => blocking_handler();
@weak void exti0_isr() => blocking_handler();
@weak void exti1_isr() => blocking_handler();
@weak void exti2_isr() => blocking_handler();
@weak void exti3_isr() => blocking_handler();
@weak void exti4_isr() => blocking_handler();
@weak void dma1_channel1_isr() => blocking_handler();
@weak void dma1_channel2_isr() => blocking_handler();
@weak void dma1_channel3_isr() => blocking_handler();
@weak void dma1_channel4_isr() => blocking_handler();
@weak void dma1_channel5_isr() => blocking_handler();
@weak void dma1_channel6_isr() => blocking_handler();
@weak void dma1_channel7_isr() => blocking_handler();
@weak void adc1_2_isr() => blocking_handler();
@weak void usb_hp_can_tx_isr() => blocking_handler();
@weak void usb_lp_can_rx0_isr() => blocking_handler();
@weak void can_rx1_isr() => blocking_handler();
@weak void can_sce_isr() => blocking_handler();
@weak void exti9_5_isr() => blocking_handler();
@weak void tim1_brk_isr() => blocking_handler();
@weak void tim1_up_isr() => blocking_handler();
@weak void tim1_trg_com_isr() => blocking_handler();
@weak void tim1_cc_isr() => blocking_handler();
@weak void tim2_isr() => blocking_handler();
@weak void tim3_isr() => blocking_handler();
@weak void tim4_isr() => blocking_handler();
@weak void i2c1_ev_isr() => blocking_handler();
@weak void i2c1_er_isr() => blocking_handler();
@weak void i2c2_ev_isr() => blocking_handler();
@weak void i2c2_er_isr() => blocking_handler();
@weak void spi1_isr() => blocking_handler();
@weak void spi2_isr() => blocking_handler();
@weak void usart1_isr() => blocking_handler();
@weak void usart2_isr() => blocking_handler();
@weak void usart3_isr() => blocking_handler();
@weak void exti15_10_isr() => blocking_handler();
@weak void rtc_alarm_isr() => blocking_handler();
@weak void usb_wakeup_isr() => blocking_handler();
@weak void tim8_brk_isr() => blocking_handler();
@weak void tim8_up_isr() => blocking_handler();
@weak void tim8_trg_com_isr() => blocking_handler();
@weak void tim8_cc_isr() => blocking_handler();
@weak void adc3_isr() => blocking_handler();
@weak void fsmc_isr() => blocking_handler();
@weak void sdio_isr() => blocking_handler();
@weak void tim5_isr() => blocking_handler();
@weak void spi3_isr() => blocking_handler();
@weak void uart4_isr() => blocking_handler();
@weak void uart5_isr() => blocking_handler();
@weak void tim6_isr() => blocking_handler();
@weak void tim7_isr() => blocking_handler();
@weak void dma2_channel1_isr() => blocking_handler();
@weak void dma2_channel2_isr() => blocking_handler();
@weak void dma2_channel3_isr() => blocking_handler();
@weak void dma2_channel4_5_isr() => blocking_handler();
@weak void dma2_channel5_isr() => blocking_handler();
@weak void eth_isr() => blocking_handler();
@weak void eth_wkup_isr() => blocking_handler();
@weak void can2_tx_isr() => blocking_handler();
@weak void can2_rx0_isr() => blocking_handler();
@weak void can2_rx1_isr() => blocking_handler();
@weak void can2_sce_isr() => blocking_handler();
@weak void otg_fs_isr() => blocking_handler();

@(section(".vectors")) static const vector_table_entry_t vector_table =
{
	initial_sp_value: &__stack,
	isr_vectors: [
		CM3.RESET: &reset_handler,
		CM3.NMI: &nmi_handler,
		CM3.HARD_FAULT: &hard_fault_handler,
		CM3.MEMORY_MANAGE_FAULT: &mem_manage_handler,
		CM3.BUS_FAULT: &bus_fault_handler,
		CM3.USAGE_FAULT: &usage_fault_handler,
		CM3.SV_CALL: &sv_call_handler,
		CM3.DEBUG_MONITOR: &debug_monitor_handler,
		CM3.PEND_SV: &pend_sv_handler,
		CM3.SYS_TICK: &sys_tick_handler,

		NVIC.WWDG: &wwdg_isr,
		NVIC.PVD: &pvd_isr,
		NVIC.TAMPER: &tamper_isr,
		NVIC.RTC: &rtc_isr,
		NVIC.FLASH: &flash_isr,
		NVIC.RCC: &rcc_isr,
		NVIC.EXTI0: &exti0_isr,
		NVIC.EXTI1: &exti1_isr,
		NVIC.EXTI2: &exti2_isr,
		NVIC.EXTI3: &exti3_isr,
		NVIC.EXTI4: &exti4_isr,
		NVIC.DMA1_CHANNEL1: &dma1_channel1_isr,
		NVIC.DMA1_CHANNEL2: &dma1_channel2_isr,
		NVIC.DMA1_CHANNEL3: &dma1_channel3_isr,
		NVIC.DMA1_CHANNEL4: &dma1_channel4_isr,
		NVIC.DMA1_CHANNEL5: &dma1_channel5_isr,
		NVIC.DMA1_CHANNEL6: &dma1_channel6_isr,
		NVIC.DMA1_CHANNEL7: &dma1_channel7_isr,
		NVIC.ADC1_2: &adc1_2_isr,
		NVIC.USB_HP_CAN_TX: &usb_hp_can_tx_isr,
		NVIC.USB_LP_CAN_RX0: &usb_lp_can_rx0_isr,
		NVIC.CAN_RX1: &can_rx1_isr,
		NVIC.CAN_SCE: &can_sce_isr,
		NVIC.EXTI9_5: &exti9_5_isr,
		NVIC.TIM1_BRK: &tim1_brk_isr,
		NVIC.TIM1_UP: &tim1_up_isr,
		NVIC.TIM1_TRG_COM: &tim1_trg_com_isr,
		NVIC.TIM1_CC: &tim1_cc_isr,
		NVIC.TIM2: &tim2_isr,
		NVIC.TIM3: &tim3_isr,
		NVIC.TIM4: &tim4_isr,
		NVIC.I2C1_EV: &i2c1_ev_isr,
		NVIC.I2C1_ER: &i2c1_er_isr,
		NVIC.I2C2_EV: &i2c2_ev_isr,
		NVIC.I2C2_ER: &i2c2_er_isr,
		NVIC.SPI1: &spi1_isr,
		NVIC.SPI2: &spi2_isr,
		NVIC.USART1: &usart1_isr,
		NVIC.USART2: &usart2_isr,
		NVIC.USART3: &usart3_isr,
		NVIC.EXTI15_10: &exti15_10_isr,
		NVIC.RTC_ALARM: &rtc_alarm_isr,
		NVIC.USB_WAKEUP: &usb_wakeup_isr,
		NVIC.TIM8_BRK: &tim8_brk_isr,
		NVIC.TIM8_UP: &tim8_up_isr,
		NVIC.TIM8_TRG_COM: &tim8_trg_com_isr,
		NVIC.TIM8_CC: &tim8_cc_isr,
		NVIC.ADC3: &adc3_isr,
		NVIC.FSMC: &fsmc_isr,
		NVIC.SDIO: &sdio_isr,
		NVIC.TIM5: &tim5_isr,
		NVIC.SPI3: &spi3_isr,
		NVIC.UART4: &uart4_isr,
		NVIC.UART5: &uart5_isr,
		NVIC.TIM6: &tim6_isr,
		NVIC.TIM7: &tim7_isr,
		NVIC.DMA2_CHANNEL1: &dma2_channel1_isr,
		NVIC.DMA2_CHANNEL2: &dma2_channel2_isr,
		NVIC.DMA2_CHANNEL3: &dma2_channel3_isr,
		NVIC.DMA2_CHANNEL4_5: &dma2_channel4_5_isr,
		NVIC.DMA2_CHANNEL5: &dma2_channel5_isr,
		NVIC.ETH: &eth_isr,
		NVIC.ETH_WKUP: &eth_wkup_isr,
		NVIC.CAN2_TX: &can2_tx_isr,
		NVIC.CAN2_RX0: &can2_rx0_isr,
		NVIC.CAN2_RX1: &can2_rx1_isr,
		NVIC.CAN2_SCE: &can2_sce_isr,
		NVIC.OTG_FS: &otg_fs_isr,
	]
};

static assert(vector_table.sizeof == 0x150);

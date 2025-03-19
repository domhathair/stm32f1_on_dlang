import cm3;
import stm32.f1x;

extern (C):
@nogc:
nothrow:

int init() {
    /* bsr - scan for bit starting from MSB, bsf - from LSB */
    RCC.APB2ENR.bset(
        RCC_APB2ENR_IOPAEN.bsr,
        RCC_APB2ENR_USART1EN.bsr
    );

    enum PA9_SHIFT = 4;
    GPIOA.CRH &= ~(0xF << GPIO_CRH_MODE9.bsf);
    GPIOA.CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9;

    USART1.CR1.bclr(USART_CR1_UE.bsr);
    USART1.CR1.bset(USART_CR1_TE.bsr);

    enum BAUD = 115_200;
    USART1.BRR = (HSI_VALUE + BAUD / 2) / BAUD;

    USART1.CR1.bset(USART_CR1_UE.bsr);

    SYS_TICK.CTRL = 0;

    SYS_TICK.LOAD = HSI_VALUE - 1; /* 1 second */

    SYS_TICK.VAL = 0;

    SYS_TICK.CTRL.bset(
        SYS_TICK_CTRL_CLKSOURCE.bsr,
        SYS_TICK_CTRL_TICKINT.bsr,
        SYS_TICK_CTRL_ENABLE.bsr
    );

    return 0;
}

void main() {
    __gshared string[] utf8 = [
        "\r\nHello world from D on STM32F1!\r\n",
        "Привет, мир! Это программа на D для STM32F1!\r\n"
    ];

    foreach (s; utf8)
        s.print;
}

@property void print(T)(T s) {
    if (s is null)
        return;

    foreach (c; s) {
        while (!USART1.SR.bget(USART_SR_TXE.bsf)) {
        }
        USART1.DR = c;
    }
}

version = stdc;

version (stdc) {
    void sys_tick_handler() {
        import cm3.core.stdc;

        __gshared size_t times = 0;
        string fmt = "\rSeconds passed:";

        @property auto length(char[] s = null) => snprintf(s.ptr, s.length, "%s %lu", fmt.ptr, times);
        T[] allocate(T)(size_t nmem) => (cast(T*)calloc(nmem, T.sizeof))[0 .. nmem];

        char[] s = allocate!char(length + 1);
        if (s is null)
            return;
        scope (exit)
            free(s.ptr);

        s[0 .. length(s)].print, times++;
    }
} else {
    void sys_tick_handler() {
        __gshared size_t times = 0;
        /**
        * There is no memory allocator here, 
        * so we will write right at the beginning 
        * of the improvised heap 
        */
        char[] s = (cast(char*)&__end)[0 .. count(times) + 1];
        "\rSeconds passed: ".print;
        s.to!char(times++).print;
    }

    size_t count(size_t n, size_t base = 10) {
        size_t count = 0;

        if (n == 0)
            return 1;

        while (n > 0)
            n /= base, count++;

        return count;
    }

    T[] reverse(T)(T[] s) {
        size_t i = 0, j = s.length;

        j -= 1;
        for (i = 0; i < j; i++, j--) {
            T c = s[i];
            s[i] = s[j];
            s[j] = c;
        }

        return s;
    }

    T[] to(T)(T[] s, size_t n) {
        size_t i = 0;

        if (n == 0) {
            s[i++] = '0';
            s[i] = '\0';
            return s;
        }

        while (n != 0) {
            size_t digit = n % 10;
            s[i++] = cast(T)(digit + '0');
            n /= 10;
        }
        s[i] = '\0';

        return reverse(s);
    }
}

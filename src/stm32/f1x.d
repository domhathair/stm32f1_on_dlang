/**
 ******************************************************************************
 * Released into the public domain.
 * This work is free: you can redistribute it and/or modify it under the terms of
 * Creative Commons Zero license v1.0
 * 
 * This work is licensed under the Creative Commons Zero 1.0 United States License.
 * To view a copy of this license, visit http://creativecommons.org/publicdomain/zero/1.0/
 * or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco,
 * California, 94105, USA.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
 ******************************************************************************
 */

module stm32.f1x;

version (stm32f1x) :
extern (C):
@nogc:
nothrow:

enum HSE_VALUE = cast(uint)8_000_000; /* Value of the External oscillator in Hz */
enum HSE_STARTUP_TIMEOUT = cast(ushort)0x0500; /* Time out for HSE start up */
enum HSI_VALUE = cast(uint)8_000_000; /* Value of the Internal oscillator in Hz*/

enum __MPU_PRESENT = 0; /* Other STM32 devices does not provide an MPU */
enum __CM3_REV = 0x0101; /* Core Revision r1p1 */
enum __NVIC_PRIO_BITS = 4; /* STM32 uses 4 Bits for the Priority Levels */

/**
 * System Timer
 */
struct sys_tick_t {
    uint CTRL;
    uint LOAD;
    uint VAL;
    uint CALIB;
}

/**
 * Analog to Digital Converter
 */
struct adc_t {
    uint SR;
    uint CR1;
    uint CR2;
    uint SMPR1;
    uint SMPR2;
    uint JOFR1;
    uint JOFR2;
    uint JOFR3;
    uint JOFR4;
    uint HTR;
    uint LTR;
    uint SQR1;
    uint SQR2;
    uint SQR3;
    uint JSQR;
    uint JDR1;
    uint JDR2;
    uint JDR3;
    uint JDR4;
    uint DR;
}

/**
 * Backup Registers
 */
struct bkp_t {
    private const uint __reserved0;
    ushort DR1;
    private const ushort __reserved1;
    ushort DR2;
    private const ushort __reserved2;
    ushort DR3;
    private const ushort __reserved3;
    ushort DR4;
    private const ushort __reserved4;
    ushort DR5;
    private const ushort __reserved5;
    ushort DR6;
    private const ushort __reserved6;
    ushort DR7;
    private const ushort __reserved7;
    ushort DR8;
    private const ushort __reserved8;
    ushort DR9;
    private const ushort __reserved9;
    ushort DR10;
    private const ushort __reserved10;
    ushort RTCCR;
    private const ushort __reserved11;
    ushort CR;
    private const ushort __reserved12;
    ushort CSR;
    private const ushort[5] __reserved13;
    ushort DR11;
    private const ushort __reserved14;
    ushort DR12;
    private const ushort __reserved15;
    ushort DR13;
    private const ushort __reserved16;
    ushort DR14;
    private const ushort __reserved17;
    ushort DR15;
    private const ushort __reserved18;
    ushort DR16;
    private const ushort __reserved19;
    ushort DR17;
    private const ushort __reserved20;
    ushort DR18;
    private const ushort __reserved21;
    ushort DR19;
    private const ushort __reserved22;
    ushort DR20;
    private const ushort __reserved23;
    ushort DR21;
    private const ushort __reserved24;
    ushort DR22;
    private const ushort __reserved25;
    ushort DR23;
    private const ushort __reserved26;
    ushort DR24;
    private const ushort __reserved27;
    ushort DR25;
    private const ushort __reserved28;
    ushort DR26;
    private const ushort __reserved29;
    ushort DR27;
    private const ushort __reserved30;
    ushort DR28;
    private const ushort __reserved31;
    ushort DR29;
    private const ushort __reserved32;
    ushort DR30;
    private const ushort __reserved33;
    ushort DR31;
    private const ushort __reserved34;
    ushort DR32;
    private const ushort __reserved35;
    ushort DR33;
    private const ushort __reserved36;
    ushort DR34;
    private const ushort __reserved37;
    ushort DR35;
    private const ushort __reserved38;
    ushort DR36;
    private const ushort __reserved39;
    ushort DR37;
    private const ushort __reserved40;
    ushort DR38;
    private const ushort __reserved41;
    ushort DR39;
    private const ushort __reserved42;
    ushort DR40;
    private const ushort __reserved43;
    ushort DR41;
    private const ushort __reserved44;
    ushort DR42;
    private const ushort __reserved45;
}

/**
 * Controller Area Network TxMailBox
 */
struct can_tx_mail_box_t {
    uint TIR;
    uint TDTR;
    uint TDLR;
    uint TDHR;
}

/**
 * Controller Area Network FIFOMailBox
 */
struct can_fifo_mail_box_t {
    uint RIR;
    uint RDTR;
    uint RDLR;
    uint RDHR;
}

/**
 * Controller Area Network FilterRegister
 */
struct can_filter_register_t {
    uint FR1;
    uint FR2;
}

/**
 * Controller Area Network
 */

struct can_t {
    uint MCR;
    uint MSR;
    uint TSR;
    uint RF0R;
    uint RF1R;
    uint IER;
    uint ESR;
    uint BTR;
    private const uint[88] __reserved0;
    can_tx_mail_box_t[3] S_TX_MAIL_BOX;
    can_fifo_mail_box_t[2] S_FIFO_MAIL_BOX;
    private const uint[12] __reserved1;
    uint FMR;
    uint FM1R;
    private const uint __reserved2;
    uint FS1R;
    private const uint __reserved3;
    uint FFA1R;
    private const uint __reserved4;
    uint FA1R;
    private const uint[8] __reserved5;

    can_filter_register_t[28] S_FILTER_REGISTER;
}

/**
 * Consumer Electronics Control (CEC)
 */
struct cec_t {
    uint CFGR;
    uint OAR;
    uint PRES;
    uint ESR;
    uint CSR;
    uint TXD;
    uint RXD;
}

/**
 * CRC calculation unit
 */
struct crc_t {
    uint DR;
    ubyte IDR;
    private const ubyte __reserved0;
    private const ushort __reserved1;
    uint CR;
}

/**
 * Digital to Analog Converter
 */
struct dac_t {
    uint CR;
    uint SWTRIGR;
    uint DHR12R1;
    uint DHR12L1;
    uint DHR8R1;
    uint DHR12R2;
    uint DHR12L2;
    uint DHR8R2;
    uint DHR12RD;
    uint DHR12LD;
    uint DHR8RD;
    uint DOR1;
    uint DOR2;
}

/**
 * Debug MCU
 */
struct dbgmcu_t {
    uint IDCODE;
    uint CR;
}

/**
 * DMA Controller
 */
struct dma_channel_t {
    uint CCR;
    uint CNDTR;
    uint CPAR;
    uint CMAR;
}

struct dma_t {
    uint ISR;
    uint IFCR;
}

/**
 * Ethernet MAC
 */
struct eth_t {
    uint MACCR;
    uint MACFFR;
    uint MACHTHR;
    uint MACHTLR;
    uint MACMIIAR;
    uint MACMIIDR;
    uint MACFCR;
    uint MACVLANTR;
    private const uint[2] __reserved0;
    uint MACRWUFFR;
    uint MACPMTCSR;
    private const uint[2] __reserved1;
    uint MACSR;
    uint MACIMR;
    uint MACA0HR;
    uint MACA0LR;
    uint MACA1HR;
    uint MACA1LR;
    uint MACA2HR;
    uint MACA2LR;
    uint MACA3HR;
    uint MACA3LR;
    private const uint[40] __reserved2;
    uint MMCCR;
    uint MMCRIR;
    uint MMCTIR;
    uint MMCRIMR;
    uint MMCTIMR;
    private const uint[14] __reserved3;
    uint MMCTGFSCCR;
    uint MMCTGFMSCCR;
    private const uint[5] __reserved4;
    uint MMCTGFCR;
    private const uint[10] __reserved5;
    uint MMCRFCECR;
    uint MMCRFAECR;
    private const uint[10] __reserved6;
    uint MMCRGUFCR;
    private const uint[334] __reserved7;
    uint PTPTSCR;
    uint PTPSSIR;
    uint PTPTSHR;
    uint PTPTSLR;
    uint PTPTSHUR;
    uint PTPTSLUR;
    uint PTPTSAR;
    uint PTPTTHR;
    uint PTPTTLR;
    private const uint[567] __reserved8;
    uint DMABMR;
    uint DMATPDR;
    uint DMARPDR;
    uint DMARDLAR;
    uint DMATDLAR;
    uint DMASR;
    uint DMAOMR;
    uint DMAIER;
    uint DMAMFBOCR;
    private const uint[9] __reserved9;
    uint DMACHTDR;
    uint DMACHRDR;
    uint DMACHTBAR;
    uint DMACHRBAR;
}

/**
 * External Interrupt/Event Controller
 */
struct exti_t {
    uint IMR;
    uint EMR;
    uint RTSR;
    uint FTSR;
    uint SWIER;
    uint PR;
}

/**
 * FLASH Registers
 */
struct flash_t {
    uint ACR;
    uint KEYR;
    uint OPTKEYR;
    uint SR;
    uint CR;
    uint AR;
    private const uint __reserved;
    uint OBR;
    uint WRPR;
}

/**
 * Option Bytes Registers
 */
struct ob_t {
    ushort RDP;
    ushort USER;
    ushort Data0;
    ushort Data1;
    ushort WRP0;
    ushort WRP1;
    ushort WRP2;
    ushort WRP3;
}

/**
 * Flexible Static Memory Controller
 */
struct fsmc_bank1_t {
    uint[8] BTCR;
}

/**
 * Flexible Static Memory Controller Bank1E
 */
struct fsmc_bank1e_t {
    uint[7] BWTR;
}

/**
 * Flexible Static Memory Controller Bank2
 */
struct fsmc_bank2_t {
    uint PCR2;
    uint SR2;
    uint PMEM2;
    uint PATT2;
    private const uint __reserved0;
    uint ECCR2;
}

/**
 * Flexible Static Memory Controller Bank3
 */
struct fsmc_bank3_t {
    uint PCR3;
    uint SR3;
    uint PMEM3;
    uint PATT3;
    private const uint __reserved0;
    uint ECCR3;
}

/**
 * Flexible Static Memory Controller Bank4
 */
struct fsmc_bank4_t {
    uint PCR4;
    uint SR4;
    uint PMEM4;
    uint PATT4;
    uint PIO4;
}

/**
 * General Purpose I/O
 */
struct gpio_t {
    uint CRL;
    uint CRH;
    uint IDR;
    uint ODR;
    uint BSRR;
    uint BRR;
    uint LCKR;
}

/**
 * Alternate Function I/O
 */
struct afio_t {
    uint EVCR;
    uint MAPR;
    uint[4] EXTICR;
    private const uint __reserved0;
    uint MAPR2;
}

/**
 * Inter Integrated Circuit Interface
 */
struct i2c_t {
    ushort CR1;
    private const ushort __reserved0;
    ushort CR2;
    private const ushort __reserved1;
    ushort OAR1;
    private const ushort __reserved2;
    ushort OAR2;
    private const ushort __reserved3;
    ushort DR;
    private const ushort __reserved4;
    ushort SR1;
    private const ushort __reserved5;
    ushort SR2;
    private const ushort __reserved6;
    ushort CCR;
    private const ushort __reserved7;
    ushort TRISE;
    private const ushort __reserved8;
}

/**
 * Independent WATCHDOG
 */
struct iwdg_t {
    uint KR;
    uint PR;
    uint RLR;
    uint SR;
}

/**
 * Power Control
 */
struct pwr_t {
    uint CR;
    uint CSR;
}

/**
 * Reset and Clock Control
 */
struct rcc_t {
    uint CR;
    uint CFGR;
    uint CIR;
    uint APB2RSTR;
    uint APB1RSTR;
    uint AHBENR;
    uint APB2ENR;
    uint APB1ENR;
    uint BDCR;
    uint CSR;

    uint AHBRSTR;
    uint CFGR2;
}

/**
 * Real-Time Clock
 */
struct rtc_t {
    ushort CRH;
    private const ushort __reserved0;
    ushort CRL;
    private const ushort __reserved1;
    ushort PRLH;
    private const ushort __reserved2;
    ushort PRLL;
    private const ushort __reserved3;
    ushort DIVH;
    private const ushort __reserved4;
    ushort DIVL;
    private const ushort __reserved5;
    ushort CNTH;
    private const ushort __reserved6;
    ushort CNTL;
    private const ushort __reserved7;
    ushort ALRH;
    private const ushort __reserved8;
    ushort ALRL;
    private const ushort __reserved9;
}

/**
 * SD host Interface
 */
struct sdio_t {
    uint POWER;
    uint CLKCR;
    uint ARG;
    uint CMD;
    const uint RESPCMD;
    const uint RESP1;
    const uint RESP2;
    const uint RESP3;
    const uint RESP4;
    uint DTIMER;
    uint DLEN;
    uint DCTRL;
    const uint DCOUNT;
    const uint STA;
    uint ICR;
    uint MASK;
    private const uint[2] __reserved0;
    const uint FIFOCNT;
    private const uint[13] __reserved1;
    uint FIFO;
}

/**
 * Serial Peripheral Interface
 */
struct spi_t {
    ushort CR1;
    private const ushort __reserved0;
    ushort CR2;
    private const ushort __reserved1;
    ushort SR;
    private const ushort __reserved2;
    ushort DR;
    private const ushort __reserved3;
    ushort CRCPR;
    private const ushort __reserved4;
    ushort RXCRCR;
    private const ushort __reserved5;
    ushort TXCRCR;
    private const ushort __reserved6;
    ushort I2SCFGR;
    private const ushort __reserved7;
    ushort I2SPR;
    private const ushort __reserved8;
}

/**
 * TIM
 */
struct tim_t {
    ushort CR1;
    private const ushort __reserved0;
    ushort CR2;
    private const ushort __reserved1;
    ushort SMCR;
    private const ushort __reserved2;
    ushort DIER;
    private const ushort __reserved3;
    ushort SR;
    private const ushort __reserved4;
    ushort EGR;
    private const ushort __reserved5;
    ushort CCMR1;
    private const ushort __reserved6;
    ushort CCMR2;
    private const ushort __reserved7;
    ushort CCER;
    private const ushort __reserved8;
    ushort CNT;
    private const ushort __reserved9;
    ushort PSC;
    private const ushort __reserved10;
    ushort ARR;
    private const ushort __reserved11;
    ushort RCR;
    private const ushort __reserved12;
    ushort CCR1;
    private const ushort __reserved13;
    ushort CCR2;
    private const ushort __reserved14;
    ushort CCR3;
    private const ushort __reserved15;
    ushort CCR4;
    private const ushort __reserved16;
    ushort BDTR;
    private const ushort __reserved17;
    ushort DCR;
    private const ushort __reserved18;
    ushort DMAR;
    private const ushort __reserved19;
}

/**
 * Universal Synchronous Asynchronous Receiver Transmitter
 */
struct usart_t {
    ushort SR;
    private const ushort __reserved0;
    ushort DR; /* 9 bits */
    private const ushort __reserved1;
    ushort BRR;
    private const ushort __reserved2;
    ushort CR1;
    private const ushort __reserved3;
    ushort CR2;
    private const ushort __reserved4;
    ushort CR3;
    private const ushort __reserved5;
    ushort GTPR;
    private const ushort __reserved6;
}

/**
 * Window WATCHDOG
 */
struct wwdg_t {
    uint CR;
    uint CFR;
    uint SR;
}

/* Memory map */
struct BASE {
    enum SCS = cast(uint)0xE000E000; /* System Control Space Base Address */

    enum FLASH = cast(uint)0x08000000; /* FLASH base address in the alias region */
    enum SRAM = cast(uint)0x20000000; /* SRAM base address in the alias region */
    enum PERIPH = cast(uint)0x40000000; /* Peripheral base address in the alias region */

    enum SRAM_BB = cast(uint)0x22000000; /* SRAM base address in the bit-band region */
    enum PERIPH_BB = cast(uint)0x42000000; /* Peripheral base address in the bit-band region */

    enum FSMC_R = cast(uint)0xA0000000; /* FSMC registers base address */

    enum SYS_TICK = SCS + 0x0010;

    enum APB1 = PERIPH;
    enum APB2 = PERIPH + 0x10000;
    enum AHB = PERIPH + 0x20000;

    enum TIM2 = APB1 + 0x0000;
    enum TIM3 = APB1 + 0x0400;
    enum TIM4 = APB1 + 0x0800;
    enum TIM5 = APB1 + 0x0C00;
    enum TIM6 = APB1 + 0x1000;
    enum TIM7 = APB1 + 0x1400;
    enum TIM12 = APB1 + 0x1800;
    enum TIM13 = APB1 + 0x1C00;
    enum TIM14 = APB1 + 0x2000;
    enum RTC = APB1 + 0x2800;
    enum WWDG = APB1 + 0x2C00;
    enum IWDG = APB1 + 0x3000;
    enum SPI2 = APB1 + 0x3800;
    enum SPI3 = APB1 + 0x3C00;
    enum USART2 = APB1 + 0x4400;
    enum USART3 = APB1 + 0x4800;
    enum UART4 = APB1 + 0x4C00;
    enum UART5 = APB1 + 0x5000;
    enum I2C1 = APB1 + 0x5400;
    enum I2C2 = APB1 + 0x5800;
    enum CAN1 = APB1 + 0x6400;
    enum CAN2 = APB1 + 0x6800;
    enum BKP = APB1 + 0x6C00;
    enum PWR = APB1 + 0x7000;
    enum DAC = APB1 + 0x7400;
    enum CEC = APB1 + 0x7800;

    enum AFIO = APB2 + 0x0000;
    enum EXTI = APB2 + 0x0400;
    enum GPIOA = APB2 + 0x0800;
    enum GPIOB = APB2 + 0x0C00;
    enum GPIOC = APB2 + 0x1000;
    enum GPIOD = APB2 + 0x1400;
    enum GPIOE = APB2 + 0x1800;
    enum GPIOF = APB2 + 0x1C00;
    enum GPIOG = APB2 + 0x2000;
    enum ADC1 = APB2 + 0x2400;
    enum ADC2 = APB2 + 0x2800;
    enum TIM1 = APB2 + 0x2C00;
    enum SPI1 = APB2 + 0x3000;
    enum TIM8 = APB2 + 0x3400;
    enum USART1 = APB2 + 0x3800;
    enum ADC3 = APB2 + 0x3C00;
    enum TIM15 = APB2 + 0x4000;
    enum TIM16 = APB2 + 0x4400;
    enum TIM17 = APB2 + 0x4800;
    enum TIM9 = APB2 + 0x4C00;
    enum TIM10 = APB2 + 0x5000;
    enum TIM11 = APB2 + 0x5400;

    enum SDIO = PERIPH + 0x18000;

    struct DMA1 {
        enum __this = AHB + 0x0000;
        alias __this this;

        enum CHANNEL1 = AHB + 0x0008;
        enum CHANNEL2 = AHB + 0x001C;
        enum CHANNEL3 = AHB + 0x0030;
        enum CHANNEL4 = AHB + 0x0044;
        enum CHANNEL5 = AHB + 0x0058;
        enum CHANNEL6 = AHB + 0x006C;
        enum CHANNEL7 = AHB + 0x0080;
    }

    struct DMA2 {
        enum __this = AHB + 0x0400;
        alias __this this;

        enum CHANNEL1 = AHB + 0x0408;
        enum CHANNEL2 = AHB + 0x041C;
        enum CHANNEL3 = AHB + 0x0430;
        enum CHANNEL4 = AHB + 0x0444;
        enum CHANNEL5 = AHB + 0x0458;
    }

    enum RCC = AHB + 0x1000;
    enum CRC = AHB + 0x3000;

    enum FLASH_R = AHB + 0x2000; /* Flash registers base address */
    enum OB = cast(uint)0x1FFFF800; /* Flash Option Bytes base address */

    struct ETH {
        enum __this = AHB + 0x8000;
        alias __this this;

        enum MAC = ETH;
        enum MMC = ETH + 0x0100;
        enum PTP = ETH + 0x0700;
        enum DMA = ETH + 0x1000;
    }

    struct FSMC {
        enum BANK1_R = FSMC_R + 0x0000; /* FSMC Bank1 registers base address */
        enum BANK1E_R = FSMC_R + 0x0104; /* FSMC Bank1E registers base address */
        enum BANK2_R = FSMC_R + 0x0060; /* FSMC Bank2 registers base address */
        enum BANK3_R = FSMC_R + 0x0080; /* FSMC Bank3 registers base address */
        enum BANK4_R = FSMC_R + 0x00A0; /* FSMC Bank4 registers base address */
    }

    enum DBGMCU = cast(uint)0xE0042000; /* Debug MCU registers base address */
}

/**
 * Peripheral declaration
 */
enum SYS_TICK = cast(sys_tick_t*)BASE.SYS_TICK;
enum TIM2 = cast(tim_t*)BASE.TIM2;
enum TIM3 = cast(tim_t*)BASE.TIM3;
enum TIM4 = cast(tim_t*)BASE.TIM4;
enum TIM5 = cast(tim_t*)BASE.TIM5;
enum TIM6 = cast(tim_t*)BASE.TIM6;
enum TIM7 = cast(tim_t*)BASE.TIM7;
enum TIM12 = cast(tim_t*)BASE.TIM12;
enum TIM13 = cast(tim_t*)BASE.TIM13;
enum TIM14 = cast(tim_t*)BASE.TIM14;
enum RTC = cast(rtc_t*)BASE.RTC;
enum WWDG = cast(wwdg_t*)BASE.WWDG;
enum IWDG = cast(iwdg_t*)BASE.IWDG;
enum SPI2 = cast(spi_t*)BASE.SPI2;
enum SPI3 = cast(spi_t*)BASE.SPI3;
enum USART2 = cast(usart_t*)BASE.USART2;
enum USART3 = cast(usart_t*)BASE.USART3;
enum UART4 = cast(usart_t*)BASE.UART4;
enum UART5 = cast(usart_t*)BASE.UART5;
enum I2C1 = cast(i2c_t*)BASE.I2C1;
enum I2C2 = cast(i2c_t*)BASE.I2C2;
enum CAN1 = cast(can_t*)BASE.CAN1;
enum CAN2 = cast(can_t*)BASE.CAN2;
enum BKP = cast(bkp_t*)BASE.BKP;
enum PWR = cast(pwr_t*)BASE.PWR;
enum DAC = cast(dac_t*)BASE.DAC;
enum CEC = cast(cec_t*)BASE.CEC;
enum AFIO = cast(afio_t*)BASE.AFIO;
enum EXTI = cast(exti_t*)BASE.EXTI;
enum GPIOA = cast(gpio_t*)BASE.GPIOA;
enum GPIOB = cast(gpio_t*)BASE.GPIOB;
enum GPIOC = cast(gpio_t*)BASE.GPIOC;
enum GPIOD = cast(gpio_t*)BASE.GPIOD;
enum GPIOE = cast(gpio_t*)BASE.GPIOE;
enum GPIOF = cast(gpio_t*)BASE.GPIOF;
enum GPIOG = cast(gpio_t*)BASE.GPIOG;
enum ADC1 = cast(adc_t*)BASE.ADC1;
enum ADC2 = cast(adc_t*)BASE.ADC2;
enum TIM1 = cast(tim_t*)BASE.TIM1;
enum STM32_SPI1 = cast(spi_t*)BASE.SPI1;
enum TIM8 = cast(tim_t*)BASE.TIM8;
enum USART1 = cast(usart_t*)BASE.USART1;
enum ADC3 = cast(adc_t*)BASE.ADC3;
enum TIM15 = cast(tim_t*)BASE.TIM15;
enum TIM16 = cast(tim_t*)BASE.TIM16;
enum TIM17 = cast(tim_t*)BASE.TIM17;
enum TIM9 = cast(tim_t*)BASE.TIM9;
enum TIM10 = cast(tim_t*)BASE.TIM10;
enum TIM11 = cast(tim_t*)BASE.TIM11;
enum SDIO = cast(sdio_t*)BASE.SDIO;
struct DMA1 {
    enum __this = cast(dma_t*)BASE.DMA1;
    alias __this this;

    enum CHANNEL1 = cast(dma_channel_t*)BASE.DMA1.CHANNEL1;
    enum CHANNEL2 = cast(dma_channel_t*)BASE.DMA1.CHANNEL2;
    enum CHANNEL3 = cast(dma_channel_t*)BASE.DMA1.CHANNEL3;
    enum CHANNEL4 = cast(dma_channel_t*)BASE.DMA1.CHANNEL4;
    enum CHANNEL5 = cast(dma_channel_t*)BASE.DMA1.CHANNEL5;
    enum CHANNEL6 = cast(dma_channel_t*)BASE.DMA1.CHANNEL6;
    enum CHANNEL7 = cast(dma_channel_t*)BASE.DMA1.CHANNEL7;
}

struct DMA2 {
    enum __this = cast(dma_t*)BASE.DMA2;
    alias __this this;

    enum CHANNEL1 = cast(dma_channel_t*)BASE.DMA2.CHANNEL1;
    enum CHANNEL2 = cast(dma_channel_t*)BASE.DMA2.CHANNEL2;
    enum CHANNEL3 = cast(dma_channel_t*)BASE.DMA2.CHANNEL3;
    enum CHANNEL4 = cast(dma_channel_t*)BASE.DMA2.CHANNEL4;
    enum CHANNEL5 = cast(dma_channel_t*)BASE.DMA2.CHANNEL5;

}

enum RCC = cast(rcc_t*)BASE.RCC;
enum CRC = cast(crc_t*)BASE.CRC;
enum FLASH = cast(flash_t*)BASE.FLASH_R;
enum OB = cast(ob_t*)BASE.OB;
enum ETH = cast(eth_t*)BASE.ETH;
struct FSMC {
    enum BANK1 = cast(fsmc_bank1_t*)BASE.FSMC.BANK1_R;
    enum BANK1E = cast(fsmc_bank1e_t*)BASE.FSMC.BANK1E_R;
    enum BANK2 = cast(fsmc_bank2_t*)BASE.FSMC.BANK2_R;
    enum BANK3 = cast(fsmc_bank3_t*)BASE.FSMC.BANK3_R;
    enum BANK4 = cast(fsmc_bank4_t*)BASE.FSMC.BANK4_R;
}

enum DBGMCU = cast(dbgmcu_t*)BASE.DBGMCU;

/**
 * Peripheral Registers Bits Definition
 */

enum CRC_DR_DR = cast(uint)0xFFFFFFFF; /* Data register bits */

enum CRC_IDR_IDR = cast(ubyte)0xFF; /* General-purpose 8-bit data register bits */

enum CRC_CR_RESET = cast(ubyte)0x01; /* RESET bit */

/* Bit definition for PWR_CR register */
enum PWR_CR_LPDS = cast(ushort)0x0001; /* Low-Power Deepsleep */
enum PWR_CR_PDDS = cast(ushort)0x0002; /* Power Down Deepsleep */
enum PWR_CR_CWUF = cast(ushort)0x0004; /* Clear Wakeup Flag */
enum PWR_CR_CSBF = cast(ushort)0x0008; /* Clear Standby Flag */
enum PWR_CR_PVDE = cast(ushort)0x0010; /* Power Voltage Detector Enable */

/* PVD level configuration */
enum PWR_CR_PLS = cast(ushort)0x00E0; /* PLS[2:0] bits (PVD Level Selection; */
enum PWR_CR_PLS_0 = cast(ushort)0x0020; /* Bit 0 */
enum PWR_CR_PLS_1 = cast(ushort)0x0040; /* Bit 1 */
enum PWR_CR_PLS_2 = cast(ushort)0x0080; /* Bit 2 */
enum PWR_CR_PLS_2V2 = cast(ushort)0x0000; /* PVD level 2.2V */
enum PWR_CR_PLS_2V3 = cast(ushort)0x0020; /* PVD level 2.3V */
enum PWR_CR_PLS_2V4 = cast(ushort)0x0040; /* PVD level 2.4V */
enum PWR_CR_PLS_2V5 = cast(ushort)0x0060; /* PVD level 2.5V */
enum PWR_CR_PLS_2V6 = cast(ushort)0x0080; /* PVD level 2.6V */
enum PWR_CR_PLS_2V7 = cast(ushort)0x00A0; /* PVD level 2.7V */
enum PWR_CR_PLS_2V8 = cast(ushort)0x00C0; /* PVD level 2.8V */
enum PWR_CR_PLS_2V9 = cast(ushort)0x00E0; /* PVD level 2.9V */

enum PWR_CR_DBP = cast(ushort)0x0100; /* Disable Backup Domain write protection */

/* Bit definition for PWR_CSR register */
enum PWR_CSR_WUF = cast(ushort)0x0001; /* Wakeup Flag */
enum PWR_CSR_SBF = cast(ushort)0x0002; /* Standby Flag */
enum PWR_CSR_PVDO = cast(ushort)0x0004; /* PVD Output */
enum PWR_CSR_EWUP = cast(ushort)0x0100; /* Enable WKUP pin */

/* Bit definition for BKP_RTCCR register */
enum BKP_RTCCR_CAL = cast(ushort)0x007F; /* Calibration value */
enum BKP_RTCCR_CCO = cast(ushort)0x0080; /* Calibration Clock Output */
enum BKP_RTCCR_ASOE = cast(ushort)0x0100; /* Alarm or Second Output Enable */
enum BKP_RTCCR_ASOS = cast(ushort)0x0200; /* Alarm or Second Output Selection */

/* Bit definition for BKP_CR register */
enum BKP_CR_TPE = cast(ubyte)0x01; /* TAMPER pin enable */
enum BKP_CR_TPAL = cast(ubyte)0x02; /* TAMPER pin active level */

/* Bit definition for BKP_CSR register */
enum BKP_CSR_CTE = cast(ushort)0x0001; /* Clear Tamper event */
enum BKP_CSR_CTI = cast(ushort)0x0002; /* Clear Tamper Interrupt */
enum BKP_CSR_TPIE = cast(ushort)0x0004; /* TAMPER Pin interrupt enable */
enum BKP_CSR_TEF = cast(ushort)0x0100; /* Tamper Event Flag */
enum BKP_CSR_TIF = cast(ushort)0x0200; /* Tamper Interrupt Flag */

/* Bit definition for RCC_CR register */
enum RCC_CR_HSION = cast(uint)0x00000001; /* Internal High Speed clock enable */
enum RCC_CR_HSIRDY = cast(uint)0x00000002; /* Internal High Speed clock ready flag */
enum RCC_CR_HSITRIM = cast(uint)0x000000F8; /* Internal High Speed clock trimming */
enum RCC_CR_HSICAL = cast(uint)0x0000FF00; /* Internal High Speed clock Calibration */
enum RCC_CR_HSEON = cast(uint)0x00010000; /* External High Speed clock enable */
enum RCC_CR_HSERDY = cast(uint)0x00020000; /* External High Speed clock ready flag */
enum RCC_CR_HSEBYP = cast(uint)0x00040000; /* External High Speed clock Bypass */
enum RCC_CR_CSSON = cast(uint)0x00080000; /* Clock Security System enable */
enum RCC_CR_PLLON = cast(uint)0x01000000; /* PLL enable */
enum RCC_CR_PLLRDY = cast(uint)0x02000000; /* PLL clock ready flag */
enum RCC_CR_PLL2ON = cast(uint)0x04000000; /* PLL2 enable */
enum RCC_CR_PLL2RDY = cast(uint)0x08000000; /* PLL2 clock ready flag */
enum RCC_CR_PLL3ON = cast(uint)0x10000000; /* PLL3 enable */
enum RCC_CR_PLL3RDY = cast(uint)0x20000000; /* PLL3 clock ready flag */

/* Bit definition for RCC_CFGR register */
/* SW configuration */
enum RCC_CFGR_SW = cast(uint)0x00000003; /* SW[1:0] bits (System clock Switch; */
enum RCC_CFGR_SW_0 = cast(uint)0x00000001; /* Bit 0 */
enum RCC_CFGR_SW_1 = cast(uint)0x00000002; /* Bit 1 */
enum RCC_CFGR_SW_HSI = cast(uint)0x00000000; /* HSI selected as system clock */
enum RCC_CFGR_SW_HSE = cast(uint)0x00000001; /* HSE selected as system clock */
enum RCC_CFGR_SW_PLL = cast(uint)0x00000002; /* PLL selected as system clock */

/* SWS configuration */
enum RCC_CFGR_SWS = cast(uint)0x0000000C; /* SWS[1:0] bits (System Clock Switch Status; */
enum RCC_CFGR_SWS_0 = cast(uint)0x00000004; /* Bit 0 */
enum RCC_CFGR_SWS_1 = cast(uint)0x00000008; /* Bit 1 */
enum RCC_CFGR_SWS_HSI = cast(uint)0x00000000; /* HSI oscillator used as system clock */
enum RCC_CFGR_SWS_HSE = cast(uint)0x00000004; /* HSE oscillator used as system clock */
enum RCC_CFGR_SWS_PLL = cast(uint)0x00000008; /* PLL used as system clock */

/* HPRE configuration */
enum RCC_CFGR_HPRE = cast(uint)0x000000F0; /* HPRE[3:0] bits (AHB prescaler; */
enum RCC_CFGR_HPRE_0 = cast(uint)0x00000010; /* Bit 0 */
enum RCC_CFGR_HPRE_1 = cast(uint)0x00000020; /* Bit 1 */
enum RCC_CFGR_HPRE_2 = cast(uint)0x00000040; /* Bit 2 */
enum RCC_CFGR_HPRE_3 = cast(uint)0x00000080; /* Bit 3 */
enum RCC_CFGR_HPRE_DIV1 = cast(uint)0x00000000; /* SYSCLK not divided */
enum RCC_CFGR_HPRE_DIV2 = cast(uint)0x00000080; /* SYSCLK divided by 2 */
enum RCC_CFGR_HPRE_DIV4 = cast(uint)0x00000090; /* SYSCLK divided by 4 */
enum RCC_CFGR_HPRE_DIV8 = cast(uint)0x000000A0; /* SYSCLK divided by 8 */
enum RCC_CFGR_HPRE_DIV16 = cast(uint)0x000000B0; /* SYSCLK divided by 16 */
enum RCC_CFGR_HPRE_DIV64 = cast(uint)0x000000C0; /* SYSCLK divided by 64 */
enum RCC_CFGR_HPRE_DIV128 = cast(uint)0x000000D0; /* SYSCLK divided by 128 */
enum RCC_CFGR_HPRE_DIV256 = cast(uint)0x000000E0; /* SYSCLK divided by 256 */
enum RCC_CFGR_HPRE_DIV512 = cast(uint)0x000000F0; /* SYSCLK divided by 512 */

/* PPRE1 configuration */
enum RCC_CFGR_PPRE1 = cast(uint)0x00000700; /* PRE1[2:0] bits (APB1 prescaler; */
enum RCC_CFGR_PPRE1_0 = cast(uint)0x00000100; /* Bit 0 */
enum RCC_CFGR_PPRE1_1 = cast(uint)0x00000200; /* Bit 1 */
enum RCC_CFGR_PPRE1_2 = cast(uint)0x00000400; /* Bit 2 */
enum RCC_CFGR_PPRE1_DIV1 = cast(uint)0x00000000; /* HCLK not divided */
enum RCC_CFGR_PPRE1_DIV2 = cast(uint)0x00000400; /* HCLK divided by 2 */
enum RCC_CFGR_PPRE1_DIV4 = cast(uint)0x00000500; /* HCLK divided by 4 */
enum RCC_CFGR_PPRE1_DIV8 = cast(uint)0x00000600; /* HCLK divided by 8 */
enum RCC_CFGR_PPRE1_DIV16 = cast(uint)0x00000700; /* HCLK divided by 16 */

/* PPRE2 configuration */
enum RCC_CFGR_PPRE2 = cast(uint)0x00003800; /* PRE2[2:0] bits (APB2 prescaler; */
enum RCC_CFGR_PPRE2_0 = cast(uint)0x00000800; /* Bit 0 */
enum RCC_CFGR_PPRE2_1 = cast(uint)0x00001000; /* Bit 1 */
enum RCC_CFGR_PPRE2_2 = cast(uint)0x00002000; /* Bit 2 */
enum RCC_CFGR_PPRE2_DIV1 = cast(uint)0x00000000; /* HCLK not divided */
enum RCC_CFGR_PPRE2_DIV2 = cast(uint)0x00002000; /* HCLK divided by 2 */
enum RCC_CFGR_PPRE2_DIV4 = cast(uint)0x00002800; /* HCLK divided by 4 */
enum RCC_CFGR_PPRE2_DIV8 = cast(uint)0x00003000; /* HCLK divided by 8 */
enum RCC_CFGR_PPRE2_DIV16 = cast(uint)0x00003800; /* HCLK divided by 16 */

/* ADCPPRE configuration */
enum RCC_CFGR_ADCPRE = cast(uint)0x0000C000; /* ADCPRE[1:0] bits (ADC prescaler; */
enum RCC_CFGR_ADCPRE_0 = cast(uint)0x00004000; /* Bit 0 */
enum RCC_CFGR_ADCPRE_1 = cast(uint)0x00008000; /* Bit 1 */
enum RCC_CFGR_ADCPRE_DIV2 = cast(uint)0x00000000; /* PCLK2 divided by 2 */
enum RCC_CFGR_ADCPRE_DIV4 = cast(uint)0x00004000; /* PCLK2 divided by 4 */
enum RCC_CFGR_ADCPRE_DIV6 = cast(uint)0x00008000; /* PCLK2 divided by 6 */
enum RCC_CFGR_ADCPRE_DIV8 = cast(uint)0x0000C000; /* PCLK2 divided by 8 */
enum RCC_CFGR_PLLSRC = cast(uint)0x00010000; /* PLL entry clock source */
enum RCC_CFGR_PLLXTPRE = cast(uint)0x00020000; /* HSE divider for PLL entry */

/* PLLMUL configuration */
enum RCC_CFGR_PLLMULL = cast(uint)0x003C0000; /* PLLMUL[3:0] bits (PLL multiplication factor; */
enum RCC_CFGR_PLLMULL_0 = cast(uint)0x00040000; /* Bit 0 */
enum RCC_CFGR_PLLMULL_1 = cast(uint)0x00080000; /* Bit 1 */
enum RCC_CFGR_PLLMULL_2 = cast(uint)0x00100000; /* Bit 2 */
enum RCC_CFGR_PLLMULL_3 = cast(uint)0x00200000; /* Bit 3 */
enum RCC_CFGR_PLLSRC_HSI_DIV2 = cast(uint)0x00000000; /* HSI clock divided by 2 selected as PLL entry clock source */
enum RCC_CFGR_PLLSRC_PREDIV1 = cast(uint)0x00010000; /* PREDIV1 clock selected as PLL entry clock source */
enum RCC_CFGR_OTGFSPRE = cast(uint)0x00400000; /* USB OTG FS prescaler */

/* MCO configuration */
enum RCC_CFGR_MCO = cast(uint)0x0F000000; /* MCO[3:0] bits (Microcontroller Clock Output; */
enum RCC_CFGR_MCO_0 = cast(uint)0x01000000; /* Bit 0 */
enum RCC_CFGR_MCO_1 = cast(uint)0x02000000; /* Bit 1 */
enum RCC_CFGR_MCO_2 = cast(uint)0x04000000; /* Bit 2 */
enum RCC_CFGR_MCO_3 = cast(uint)0x08000000; /* Bit 3 */
enum RCC_CFGR_MCO_NOCLOCK = cast(uint)0x00000000; /* No clock */
enum RCC_CFGR_MCO_SYSCLK = cast(uint)0x04000000; /* System clock selected as MCO source */
enum RCC_CFGR_MCO_HSI = cast(uint)0x05000000; /* HSI clock selected as MCO source */
enum RCC_CFGR_MCO_HSE = cast(uint)0x06000000; /* HSE clock selected as MCO source */
enum RCC_CFGR_MCO_PLLCLK_DIV2 = cast(uint)0x07000000; /* PLL clock divided by 2 selected as MCO source */
enum RCC_CFGR_MCO_PLL2CLK = cast(uint)0x08000000; /* PLL2 clock selected as MCO source*/
enum RCC_CFGR_MCO_PLL3CLK_DIV2 = cast(uint)0x09000000; /* PLL3 clock divided by 2 selected as MCO source*/
enum RCC_CFGR_MCO_EXT_HSE = cast(uint)0x0A000000; /* XT1 external 3-25 MHz oscillator clock selected as MCO source */
enum RCC_CFGR_MCO_PLL3CLK = cast(uint)0x0B000000; /* PLL3 clock selected as MCO source */

enum RCC_CFGR_PLLXTPRE_PREDIV1 = cast(uint)0x00000000; /* PREDIV1 clock not divided for PLL entry */
enum RCC_CFGR_PLLXTPRE_PREDIV1_DIV2 = cast(uint)0x00020000; /* PREDIV1 clock divided by 2 for PLL entry */
enum RCC_CFGR_PLLSRC_HSE = cast(uint)0x00010000; /* HSE clock selected as PLL entry clock source */
enum RCC_CFGR_PLLXTPRE_HSE = cast(uint)0x00000000; /* HSE clock not divided for PLL entry */
enum RCC_CFGR_PLLXTPRE_HSE_DIV2 = cast(uint)0x00020000; /* HSE clock divided by 2 for PLL entry */
enum RCC_CFGR_PLLMULL2 = cast(uint)0x00000000; /* PLL input clock*2 */
enum RCC_CFGR_PLLMULL3 = cast(uint)0x00040000; /* PLL input clock*3 */
enum RCC_CFGR_PLLMULL4 = cast(uint)0x00080000; /* PLL input clock*4 */
enum RCC_CFGR_PLLMULL5 = cast(uint)0x000C0000; /* PLL input clock*5 */
enum RCC_CFGR_PLLMULL6 = cast(uint)0x00100000; /* PLL input clock*6 */
enum RCC_CFGR_PLLMULL7 = cast(uint)0x00140000; /* PLL input clock*7 */
enum RCC_CFGR_PLLMULL8 = cast(uint)0x00180000; /* PLL input clock*8 */
enum RCC_CFGR_PLLMULL9 = cast(uint)0x001C0000; /* PLL input clock*9 */
enum RCC_CFGR_PLLMULL10 = cast(uint)0x00200000; /* PLL input clock10 */
enum RCC_CFGR_PLLMULL11 = cast(uint)0x00240000; /* PLL input clock*11 */
enum RCC_CFGR_PLLMULL12 = cast(uint)0x00280000; /* PLL input clock*12 */
enum RCC_CFGR_PLLMULL13 = cast(uint)0x002C0000; /* PLL input clock*13 */
enum RCC_CFGR_PLLMULL14 = cast(uint)0x00300000; /* PLL input clock*14 */
enum RCC_CFGR_PLLMULL15 = cast(uint)0x00340000; /* PLL input clock*15 */
enum RCC_CFGR_PLLMULL6_5 = cast(uint)0x00340000; /* PLL input clock*6.5 */
enum RCC_CFGR_PLLMULL16 = cast(uint)0x00380000; /* PLL input clock*16 */
enum RCC_CFGR_USBPRE = cast(uint)0x00400000; /* USB Device prescaler */

/* Bit definition for RCC_CIR register */
enum RCC_CIR_LSIRDYF = cast(uint)0x00000001; /* LSI Ready Interrupt flag */
enum RCC_CIR_LSERDYF = cast(uint)0x00000002; /* LSE Ready Interrupt flag */
enum RCC_CIR_HSIRDYF = cast(uint)0x00000004; /* HSI Ready Interrupt flag */
enum RCC_CIR_HSERDYF = cast(uint)0x00000008; /* HSE Ready Interrupt flag */
enum RCC_CIR_PLLRDYF = cast(uint)0x00000010; /* PLL Ready Interrupt flag */
enum RCC_CIR_CSSF = cast(uint)0x00000080; /* Clock Security System Interrupt flag */
enum RCC_CIR_LSIRDYIE = cast(uint)0x00000100; /* LSI Ready Interrupt Enable */
enum RCC_CIR_LSERDYIE = cast(uint)0x00000200; /* LSE Ready Interrupt Enable */
enum RCC_CIR_HSIRDYIE = cast(uint)0x00000400; /* HSI Ready Interrupt Enable */
enum RCC_CIR_HSERDYIE = cast(uint)0x00000800; /* HSE Ready Interrupt Enable */
enum RCC_CIR_PLLRDYIE = cast(uint)0x00001000; /* PLL Ready Interrupt Enable */
enum RCC_CIR_LSIRDYC = cast(uint)0x00010000; /* LSI Ready Interrupt Clear */
enum RCC_CIR_LSERDYC = cast(uint)0x00020000; /* LSE Ready Interrupt Clear */
enum RCC_CIR_HSIRDYC = cast(uint)0x00040000; /* HSI Ready Interrupt Clear */
enum RCC_CIR_HSERDYC = cast(uint)0x00080000; /* HSE Ready Interrupt Clear */
enum RCC_CIR_PLLRDYC = cast(uint)0x00100000; /* PLL Ready Interrupt Clear */
enum RCC_CIR_CSSC = cast(uint)0x00800000; /* Clock Security System Interrupt Clear */
enum RCC_CIR_PLL2RDYF = cast(uint)0x00000020; /* PLL2 Ready Interrupt flag */
enum RCC_CIR_PLL3RDYF = cast(uint)0x00000040; /* PLL3 Ready Interrupt flag */
enum RCC_CIR_PLL2RDYIE = cast(uint)0x00002000; /* PLL2 Ready Interrupt Enable */
enum RCC_CIR_PLL3RDYIE = cast(uint)0x00004000; /* PLL3 Ready Interrupt Enable */
enum RCC_CIR_PLL2RDYC = cast(uint)0x00200000; /* PLL2 Ready Interrupt Clear */
enum RCC_CIR_PLL3RDYC = cast(uint)0x00400000; /* PLL3 Ready Interrupt Clear */

/* Bit definition for RCC_APB2RSTR register */
enum RCC_APB2RSTR_AFIORST = cast(uint)0x00000001; /* Alternate Function I/O reset */
enum RCC_APB2RSTR_IOPARST = cast(uint)0x00000004; /* I/O port A reset */
enum RCC_APB2RSTR_IOPBRST = cast(uint)0x00000008; /* I/O port B reset */
enum RCC_APB2RSTR_IOPCRST = cast(uint)0x00000010; /* I/O port C reset */
enum RCC_APB2RSTR_IOPDRST = cast(uint)0x00000020; /* I/O port D reset */
enum RCC_APB2RSTR_ADC1RST = cast(uint)0x00000200; /* ADC 1 interface reset */
enum RCC_APB2RSTR_ADC2RST = cast(uint)0x00000400; /* ADC 2 interface reset */
enum RCC_APB2RSTR_TIM1RST = cast(uint)0x00000800; /* TIM1 Timer reset */
enum RCC_APB2RSTR_SPI1RST = cast(uint)0x00001000; /* SPI 1 reset */
enum RCC_APB2RSTR_USART1RST = cast(uint)0x00004000; /* USART1 reset */
enum RCC_APB2RSTR_TIM15RST = cast(uint)0x00010000; /* TIM15 Timer reset */
enum RCC_APB2RSTR_TIM16RST = cast(uint)0x00020000; /* TIM16 Timer reset */
enum RCC_APB2RSTR_TIM17RST = cast(uint)0x00040000; /* TIM17 Timer reset */
enum RCC_APB2RSTR_IOPERST = cast(uint)0x00000040; /* I/O port E reset */
enum RCC_APB2RSTR_IOPFRST = cast(uint)0x00000080; /* I/O port F reset */
enum RCC_APB2RSTR_IOPGRST = cast(uint)0x00000100; /* I/O port G reset */
enum RCC_APB2RSTR_TIM8RST = cast(uint)0x00002000; /* TIM8 Timer reset */
enum RCC_APB2RSTR_ADC3RST = cast(uint)0x00008000; /* ADC3 interface reset */
enum RCC_APB2RSTR_TIM9RST = cast(uint)0x00080000; /* TIM9 Timer reset */
enum RCC_APB2RSTR_TIM10RST = cast(uint)0x00100000; /* TIM10 Timer reset */
enum RCC_APB2RSTR_TIM11RST = cast(uint)0x00200000; /* TIM11 Timer reset */

/* Bit definition for RCC_APB1RSTR register */
enum RCC_APB1RSTR_TIM2RST = cast(uint)0x00000001; /* Timer 2 reset */
enum RCC_APB1RSTR_TIM3RST = cast(uint)0x00000002; /* Timer 3 reset */
enum RCC_APB1RSTR_WWDGRST = cast(uint)0x00000800; /* Window Watchdog reset */
enum RCC_APB1RSTR_USART2RST = cast(uint)0x00020000; /* USART 2 reset */
enum RCC_APB1RSTR_I2C1RST = cast(uint)0x00200000; /* I2C 1 reset */
enum RCC_APB1RSTR_CAN1RST = cast(uint)0x02000000; /* CAN1 reset */
enum RCC_APB1RSTR_BKPRST = cast(uint)0x08000000; /* Backup interface reset */
enum RCC_APB1RSTR_PWRRST = cast(uint)0x10000000; /* Power interface reset */
enum RCC_APB1RSTR_TIM4RST = cast(uint)0x00000004; /* Timer 4 reset */
enum RCC_APB1RSTR_SPI2RST = cast(uint)0x00004000; /* SPI 2 reset */
enum RCC_APB1RSTR_USART3RST = cast(uint)0x00040000; /* USART 3 reset */
enum RCC_APB1RSTR_I2C2RST = cast(uint)0x00400000; /* I2C 2 reset */
enum RCC_APB1RSTR_USBRST = cast(uint)0x00800000; /* USB Device reset */
enum RCC_APB1RSTR_TIM5RST = cast(uint)0x00000008; /* Timer 5 reset */
enum RCC_APB1RSTR_TIM6RST = cast(uint)0x00000010; /* Timer 6 reset */
enum RCC_APB1RSTR_TIM7RST = cast(uint)0x00000020; /* Timer 7 reset */
enum RCC_APB1RSTR_SPI3RST = cast(uint)0x00008000; /* SPI 3 reset */
enum RCC_APB1RSTR_UART4RST = cast(uint)0x00080000; /* UART 4 reset */
enum RCC_APB1RSTR_UART5RST = cast(uint)0x00100000; /* UART 5 reset */
enum RCC_APB1RSTR_DACRST = cast(uint)0x20000000; /* DAC interface reset */
enum RCC_APB1RSTR_CECRST = cast(uint)0x40000000; /* CEC interface reset */
enum RCC_APB1RSTR_CAN2RST = cast(uint)0x04000000; /* CAN2 reset */
enum RCC_APB1RSTR_TIM12RST = cast(uint)0x00000040; /* TIM12 Timer reset */
enum RCC_APB1RSTR_TIM13RST = cast(uint)0x00000080; /* TIM13 Timer reset */
enum RCC_APB1RSTR_TIM14RST = cast(uint)0x00000100; /* TIM14 Timer reset */

/* Bit definition for RCC_AHBENR register */
enum RCC_AHBENR_DMA1EN = cast(ushort)0x0001; /* DMA1 clock enable */
enum RCC_AHBENR_SRAMEN = cast(ushort)0x0004; /* SRAM interface clock enable */
enum RCC_AHBENR_FLITFEN = cast(ushort)0x0010; /* FLITF clock enable */
enum RCC_AHBENR_CRCEN = cast(ushort)0x0040; /* CRC clock enable */
enum RCC_AHBENR_DMA2EN = cast(ushort)0x0002; /* DMA2 clock enable */
enum RCC_AHBENR_FSMCEN = cast(ushort)0x0100; /* FSMC clock enable */
enum RCC_AHBENR_SDIOEN = cast(ushort)0x0400; /* SDIO clock enable */
enum RCC_AHBENR_OTGFSEN = cast(uint)0x00001000; /* USB OTG FS clock enable */
enum RCC_AHBENR_ETHMACEN = cast(uint)0x00004000; /* ETHERNET MAC clock enable */
enum RCC_AHBENR_ETHMACTXEN = cast(uint)0x00008000; /* ETHERNET MAC Tx clock enable */
enum RCC_AHBENR_ETHMACRXEN = cast(uint)0x00010000; /* ETHERNET MAC Rx clock enable */

/* Bit definition for RCC_APB2ENR register */
enum RCC_APB2ENR_AFIOEN = cast(uint)0x00000001; /* Alternate Function I/O clock enable */
enum RCC_APB2ENR_IOPAEN = cast(uint)0x00000004; /* I/O port A clock enable */
enum RCC_APB2ENR_IOPBEN = cast(uint)0x00000008; /* I/O port B clock enable */
enum RCC_APB2ENR_IOPCEN = cast(uint)0x00000010; /* I/O port C clock enable */
enum RCC_APB2ENR_IOPDEN = cast(uint)0x00000020; /* I/O port D clock enable */
enum RCC_APB2ENR_ADC1EN = cast(uint)0x00000200; /* ADC 1 interface clock enable */
enum RCC_APB2ENR_ADC2EN = cast(uint)0x00000400; /* ADC 2 interface clock enable */
enum RCC_APB2ENR_TIM1EN = cast(uint)0x00000800; /* TIM1 Timer clock enable */
enum RCC_APB2ENR_SPI1EN = cast(uint)0x00001000; /* SPI 1 clock enable */
enum RCC_APB2ENR_USART1EN = cast(uint)0x00004000; /* USART1 clock enable */
enum RCC_APB2ENR_TIM15EN = cast(uint)0x00010000; /* TIM15 Timer clock enable */
enum RCC_APB2ENR_TIM16EN = cast(uint)0x00020000; /* TIM16 Timer clock enable */
enum RCC_APB2ENR_TIM17EN = cast(uint)0x00040000; /* TIM17 Timer clock enable */
enum RCC_APB2ENR_IOPEEN = cast(uint)0x00000040; /* I/O port E clock enable */
enum RCC_APB2ENR_IOPFEN = cast(uint)0x00000080; /* I/O port F clock enable */
enum RCC_APB2ENR_IOPGEN = cast(uint)0x00000100; /* I/O port G clock enable */
enum RCC_APB2ENR_TIM8EN = cast(uint)0x00002000; /* TIM8 Timer clock enable */
enum RCC_APB2ENR_ADC3EN = cast(uint)0x00008000; /* DMA1 clock enable */
enum RCC_APB2ENR_TIM9EN = cast(uint)0x00080000; /* TIM9 Timer clock enable */
enum RCC_APB2ENR_TIM10EN = cast(uint)0x00100000; /* TIM10 Timer clock enable */
enum RCC_APB2ENR_TIM11EN = cast(uint)0x00200000; /* TIM11 Timer clock enable */

/* Bit definition for RCC_APB1ENR register */
enum RCC_APB1ENR_TIM2EN = cast(uint)0x00000001; /* Timer 2 clock enabled*/
enum RCC_APB1ENR_TIM3EN = cast(uint)0x00000002; /* Timer 3 clock enable */
enum RCC_APB1ENR_WWDGEN = cast(uint)0x00000800; /* Window Watchdog clock enable */
enum RCC_APB1ENR_USART2EN = cast(uint)0x00020000; /* USART 2 clock enable */
enum RCC_APB1ENR_I2C1EN = cast(uint)0x00200000; /* I2C 1 clock enable */
enum RCC_APB1ENR_CAN1EN = cast(uint)0x02000000; /* CAN1 clock enable */
enum RCC_APB1ENR_BKPEN = cast(uint)0x08000000; /* Backup interface clock enable */
enum RCC_APB1ENR_PWREN = cast(uint)0x10000000; /* Power interface clock enable */
enum RCC_APB1ENR_TIM4EN = cast(uint)0x00000004; /* Timer 4 clock enable */
enum RCC_APB1ENR_SPI2EN = cast(uint)0x00004000; /* SPI 2 clock enable */
enum RCC_APB1ENR_USART3EN = cast(uint)0x00040000; /* USART 3 clock enable */
enum RCC_APB1ENR_I2C2EN = cast(uint)0x00400000; /* I2C 2 clock enable */
enum RCC_APB1ENR_USBEN = cast(uint)0x00800000; /* USB Device clock enable */
enum RCC_APB1ENR_TIM5EN = cast(uint)0x00000008; /* Timer 5 clock enable */
enum RCC_APB1ENR_TIM6EN = cast(uint)0x00000010; /* Timer 6 clock enable */
enum RCC_APB1ENR_TIM7EN = cast(uint)0x00000020; /* Timer 7 clock enable */
enum RCC_APB1ENR_SPI3EN = cast(uint)0x00008000; /* SPI 3 clock enable */
enum RCC_APB1ENR_UART4EN = cast(uint)0x00080000; /* UART 4 clock enable */
enum RCC_APB1ENR_UART5EN = cast(uint)0x00100000; /* UART 5 clock enable */
enum RCC_APB1ENR_DACEN = cast(uint)0x20000000; /* DAC interface clock enable */
enum RCC_APB1ENR_CECEN = cast(uint)0x40000000; /* CEC interface clock enable */
enum RCC_APB1ENR_TIM12EN = cast(uint)0x00000040; /* TIM12 Timer clock enable */
enum RCC_APB1ENR_TIM13EN = cast(uint)0x00000080; /* TIM13 Timer clock enable */
enum RCC_APB1ENR_TIM14EN = cast(uint)0x00000100; /* TIM14 Timer clock enable */
enum RCC_APB1ENR_CAN2EN = cast(uint)0x04000000; /* CAN2 clock enable */

/* Bit definition for RCC_BDCR register */
enum RCC_BDCR_LSEON = cast(uint)0x00000001; /* External Low Speed oscillator enable */
enum RCC_BDCR_LSERDY = cast(uint)0x00000002; /* External Low Speed oscillator Ready */
enum RCC_BDCR_LSEBYP = cast(uint)0x00000004; /* External Low Speed oscillator Bypass */

enum RCC_BDCR_RTCSEL = cast(uint)0x00000300; /* RTCSEL[1:0] bits (RTC clock source selection; */
enum RCC_BDCR_RTCSEL_0 = cast(uint)0x00000100; /* Bit 0 */
enum RCC_BDCR_RTCSEL_1 = cast(uint)0x00000200; /* Bit 1 */

/* RTC congiguration */
enum RCC_BDCR_RTCSEL_NOCLOCK = cast(uint)0x00000000; /* No clock */
enum RCC_BDCR_RTCSEL_LSE = cast(uint)0x00000100; /* LSE oscillator clock used as RTC clock */
enum RCC_BDCR_RTCSEL_LSI = cast(uint)0x00000200; /* LSI oscillator clock used as RTC clock */
enum RCC_BDCR_RTCSEL_HSE = cast(uint)0x00000300; /* HSE oscillator clock divided by 128 used as RTC clock */

enum RCC_BDCR_RTCEN = cast(uint)0x00008000; /* RTC clock enable */
enum RCC_BDCR_BDRST = cast(uint)0x00010000; /* Backup domain software reset */

/* Bit definition for RCC_CSR register */
enum RCC_CSR_LSION = cast(uint)0x00000001; /* Internal Low Speed oscillator enable */
enum RCC_CSR_LSIRDY = cast(uint)0x00000002; /* Internal Low Speed oscillator Ready */
enum RCC_CSR_RMVF = cast(uint)0x01000000; /* Remove reset flag */
enum RCC_CSR_PINRSTF = cast(uint)0x04000000; /* PIN reset flag */
enum RCC_CSR_PORRSTF = cast(uint)0x08000000; /* POR/PDR reset flag */
enum RCC_CSR_SFTRSTF = cast(uint)0x10000000; /* Software Reset flag */
enum RCC_CSR_IWDGRSTF = cast(uint)0x20000000; /* Independent Watchdog reset flag */
enum RCC_CSR_WWDGRSTF = cast(uint)0x40000000; /* Window watchdog reset flag */
enum RCC_CSR_LPWRRSTF = cast(uint)0x80000000; /* Low-Power reset flag */

/* Bit definition for RCC_AHBRSTR register */
enum RCC_AHBRSTR_OTGFSRST = cast(uint)0x00001000; /* USB OTG FS reset */
enum RCC_AHBRSTR_ETHMACRST = cast(uint)0x00004000; /* ETHERNET MAC reset */

/* Bit definition for RCC_CFGR2 register */
/* PREDIV1 configuration */
enum RCC_CFGR2_PREDIV1 = cast(uint)0x0000000F; /* PREDIV1[3:0] bits */
enum RCC_CFGR2_PREDIV1_0 = cast(uint)0x00000001; /* Bit 0 */
enum RCC_CFGR2_PREDIV1_1 = cast(uint)0x00000002; /* Bit 1 */
enum RCC_CFGR2_PREDIV1_2 = cast(uint)0x00000004; /* Bit 2 */
enum RCC_CFGR2_PREDIV1_3 = cast(uint)0x00000008; /* Bit 3 */

enum RCC_CFGR2_PREDIV1_DIV1 = cast(uint)0x00000000; /* PREDIV1 input clock not divided */
enum RCC_CFGR2_PREDIV1_DIV2 = cast(uint)0x00000001; /* PREDIV1 input clock divided by 2 */
enum RCC_CFGR2_PREDIV1_DIV3 = cast(uint)0x00000002; /* PREDIV1 input clock divided by 3 */
enum RCC_CFGR2_PREDIV1_DIV4 = cast(uint)0x00000003; /* PREDIV1 input clock divided by 4 */
enum RCC_CFGR2_PREDIV1_DIV5 = cast(uint)0x00000004; /* PREDIV1 input clock divided by 5 */
enum RCC_CFGR2_PREDIV1_DIV6 = cast(uint)0x00000005; /* PREDIV1 input clock divided by 6 */
enum RCC_CFGR2_PREDIV1_DIV7 = cast(uint)0x00000006; /* PREDIV1 input clock divided by 7 */
enum RCC_CFGR2_PREDIV1_DIV8 = cast(uint)0x00000007; /* PREDIV1 input clock divided by 8 */
enum RCC_CFGR2_PREDIV1_DIV9 = cast(uint)0x00000008; /* PREDIV1 input clock divided by 9 */
enum RCC_CFGR2_PREDIV1_DIV10 = cast(uint)0x00000009; /* PREDIV1 input clock divided by 10 */
enum RCC_CFGR2_PREDIV1_DIV11 = cast(uint)0x0000000A; /* PREDIV1 input clock divided by 11 */
enum RCC_CFGR2_PREDIV1_DIV12 = cast(uint)0x0000000B; /* PREDIV1 input clock divided by 12 */
enum RCC_CFGR2_PREDIV1_DIV13 = cast(uint)0x0000000C; /* PREDIV1 input clock divided by 13 */
enum RCC_CFGR2_PREDIV1_DIV14 = cast(uint)0x0000000D; /* PREDIV1 input clock divided by 14 */
enum RCC_CFGR2_PREDIV1_DIV15 = cast(uint)0x0000000E; /* PREDIV1 input clock divided by 15 */
enum RCC_CFGR2_PREDIV1_DIV16 = cast(uint)0x0000000F; /* PREDIV1 input clock divided by 16 */

/* PREDIV2 configuration */
enum RCC_CFGR2_PREDIV2 = cast(uint)0x000000F0; /* PREDIV2[3:0] bits */
enum RCC_CFGR2_PREDIV2_0 = cast(uint)0x00000010; /* Bit 0 */
enum RCC_CFGR2_PREDIV2_1 = cast(uint)0x00000020; /* Bit 1 */
enum RCC_CFGR2_PREDIV2_2 = cast(uint)0x00000040; /* Bit 2 */
enum RCC_CFGR2_PREDIV2_3 = cast(uint)0x00000080; /* Bit 3 */

enum RCC_CFGR2_PREDIV2_DIV1 = cast(uint)0x00000000; /* PREDIV2 input clock not divided */
enum RCC_CFGR2_PREDIV2_DIV2 = cast(uint)0x00000010; /* PREDIV2 input clock divided by 2 */
enum RCC_CFGR2_PREDIV2_DIV3 = cast(uint)0x00000020; /* PREDIV2 input clock divided by 3 */
enum RCC_CFGR2_PREDIV2_DIV4 = cast(uint)0x00000030; /* PREDIV2 input clock divided by 4 */
enum RCC_CFGR2_PREDIV2_DIV5 = cast(uint)0x00000040; /* PREDIV2 input clock divided by 5 */
enum RCC_CFGR2_PREDIV2_DIV6 = cast(uint)0x00000050; /* PREDIV2 input clock divided by 6 */
enum RCC_CFGR2_PREDIV2_DIV7 = cast(uint)0x00000060; /* PREDIV2 input clock divided by 7 */
enum RCC_CFGR2_PREDIV2_DIV8 = cast(uint)0x00000070; /* PREDIV2 input clock divided by 8 */
enum RCC_CFGR2_PREDIV2_DIV9 = cast(uint)0x00000080; /* PREDIV2 input clock divided by 9 */
enum RCC_CFGR2_PREDIV2_DIV10 = cast(uint)0x00000090; /* PREDIV2 input clock divided by 10 */
enum RCC_CFGR2_PREDIV2_DIV11 = cast(uint)0x000000A0; /* PREDIV2 input clock divided by 11 */
enum RCC_CFGR2_PREDIV2_DIV12 = cast(uint)0x000000B0; /* PREDIV2 input clock divided by 12 */
enum RCC_CFGR2_PREDIV2_DIV13 = cast(uint)0x000000C0; /* PREDIV2 input clock divided by 13 */
enum RCC_CFGR2_PREDIV2_DIV14 = cast(uint)0x000000D0; /* PREDIV2 input clock divided by 14 */
enum RCC_CFGR2_PREDIV2_DIV15 = cast(uint)0x000000E0; /* PREDIV2 input clock divided by 15 */
enum RCC_CFGR2_PREDIV2_DIV16 = cast(uint)0x000000F0; /* PREDIV2 input clock divided by 16 */

/* PLL2MUL configuration */
enum RCC_CFGR2_PLL2MUL = cast(uint)0x00000F00; /* PLL2MUL[3:0] bits */
enum RCC_CFGR2_PLL2MUL_0 = cast(uint)0x00000100; /* Bit 0 */
enum RCC_CFGR2_PLL2MUL_1 = cast(uint)0x00000200; /* Bit 1 */
enum RCC_CFGR2_PLL2MUL_2 = cast(uint)0x00000400; /* Bit 2 */
enum RCC_CFGR2_PLL2MUL_3 = cast(uint)0x00000800; /* Bit 3 */

enum RCC_CFGR2_PLL2MUL8 = cast(uint)0x00000600; /* PLL2 input clock * 8 */
enum RCC_CFGR2_PLL2MUL9 = cast(uint)0x00000700; /* PLL2 input clock * 9 */
enum RCC_CFGR2_PLL2MUL10 = cast(uint)0x00000800; /* PLL2 input clock * 10 */
enum RCC_CFGR2_PLL2MUL11 = cast(uint)0x00000900; /* PLL2 input clock * 11 */
enum RCC_CFGR2_PLL2MUL12 = cast(uint)0x00000A00; /* PLL2 input clock * 12 */
enum RCC_CFGR2_PLL2MUL13 = cast(uint)0x00000B00; /* PLL2 input clock * 13 */
enum RCC_CFGR2_PLL2MUL14 = cast(uint)0x00000C00; /* PLL2 input clock * 14 */
enum RCC_CFGR2_PLL2MUL16 = cast(uint)0x00000E00; /* PLL2 input clock * 16 */
enum RCC_CFGR2_PLL2MUL20 = cast(uint)0x00000F00; /* PLL2 input clock * 20 */

/* PLL3MUL configuration */
enum RCC_CFGR2_PLL3MUL = cast(uint)0x0000F000; /* PLL3MUL[3:0] bits */
enum RCC_CFGR2_PLL3MUL_0 = cast(uint)0x00001000; /* Bit 0 */
enum RCC_CFGR2_PLL3MUL_1 = cast(uint)0x00002000; /* Bit 1 */
enum RCC_CFGR2_PLL3MUL_2 = cast(uint)0x00004000; /* Bit 2 */
enum RCC_CFGR2_PLL3MUL_3 = cast(uint)0x00008000; /* Bit 3 */

enum RCC_CFGR2_PLL3MUL8 = cast(uint)0x00006000; /* PLL3 input clock * 8 */
enum RCC_CFGR2_PLL3MUL9 = cast(uint)0x00007000; /* PLL3 input clock * 9 */
enum RCC_CFGR2_PLL3MUL10 = cast(uint)0x00008000; /* PLL3 input clock * 10 */
enum RCC_CFGR2_PLL3MUL11 = cast(uint)0x00009000; /* PLL3 input clock * 11 */
enum RCC_CFGR2_PLL3MUL12 = cast(uint)0x0000A000; /* PLL3 input clock * 12 */
enum RCC_CFGR2_PLL3MUL13 = cast(uint)0x0000B000; /* PLL3 input clock * 13 */
enum RCC_CFGR2_PLL3MUL14 = cast(uint)0x0000C000; /* PLL3 input clock * 14 */
enum RCC_CFGR2_PLL3MUL16 = cast(uint)0x0000E000; /* PLL3 input clock * 16 */
enum RCC_CFGR2_PLL3MUL20 = cast(uint)0x0000F000; /* PLL3 input clock * 20 */

enum RCC_CFGR2_PREDIV1SRC = cast(uint)0x00010000; /* PREDIV1 entry clock source */
enum RCC_CFGR2_PREDIV1SRC_PLL2 = cast(uint)0x00010000; /* PLL2 selected as PREDIV1 entry clock source */
enum RCC_CFGR2_PREDIV1SRC_HSE = cast(uint)0x00000000; /* HSE selected as PREDIV1 entry clock source */
enum RCC_CFGR2_I2S2SRC = cast(uint)0x00020000; /* I2S2 entry clock source */
enum RCC_CFGR2_I2S3SRC = cast(uint)0x00040000; /* I2S3 clock source */

/* Bit definition for GPIO_CRL register */
enum GPIO_CRL_MODE = cast(uint)0x33333333; /* Port x mode bits */

enum GPIO_CRL_MODE0 = cast(uint)0x00000003; /* MODE0[1:0] bits (Port x mode bits, pin 0; */
enum GPIO_CRL_MODE0_0 = cast(uint)0x00000001; /* Bit 0 */
enum GPIO_CRL_MODE0_1 = cast(uint)0x00000002; /* Bit 1 */

enum GPIO_CRL_MODE1 = cast(uint)0x00000030; /* MODE1[1:0] bits (Port x mode bits, pin 1; */
enum GPIO_CRL_MODE1_0 = cast(uint)0x00000010; /* Bit 0 */
enum GPIO_CRL_MODE1_1 = cast(uint)0x00000020; /* Bit 1 */

enum GPIO_CRL_MODE2 = cast(uint)0x00000300; /* MODE2[1:0] bits (Port x mode bits, pin 2; */
enum GPIO_CRL_MODE2_0 = cast(uint)0x00000100; /* Bit 0 */
enum GPIO_CRL_MODE2_1 = cast(uint)0x00000200; /* Bit 1 */

enum GPIO_CRL_MODE3 = cast(uint)0x00003000; /* MODE3[1:0] bits (Port x mode bits, pin 3; */
enum GPIO_CRL_MODE3_0 = cast(uint)0x00001000; /* Bit 0 */
enum GPIO_CRL_MODE3_1 = cast(uint)0x00002000; /* Bit 1 */

enum GPIO_CRL_MODE4 = cast(uint)0x00030000; /* MODE4[1:0] bits (Port x mode bits, pin 4; */
enum GPIO_CRL_MODE4_0 = cast(uint)0x00010000; /* Bit 0 */
enum GPIO_CRL_MODE4_1 = cast(uint)0x00020000; /* Bit 1 */

enum GPIO_CRL_MODE5 = cast(uint)0x00300000; /* MODE5[1:0] bits (Port x mode bits, pin 5; */
enum GPIO_CRL_MODE5_0 = cast(uint)0x00100000; /* Bit 0 */
enum GPIO_CRL_MODE5_1 = cast(uint)0x00200000; /* Bit 1 */

enum GPIO_CRL_MODE6 = cast(uint)0x03000000; /* MODE6[1:0] bits (Port x mode bits, pin 6; */
enum GPIO_CRL_MODE6_0 = cast(uint)0x01000000; /* Bit 0 */
enum GPIO_CRL_MODE6_1 = cast(uint)0x02000000; /* Bit 1 */

enum GPIO_CRL_MODE7 = cast(uint)0x30000000; /* MODE7[1:0] bits (Port x mode bits, pin 7; */
enum GPIO_CRL_MODE7_0 = cast(uint)0x10000000; /* Bit 0 */
enum GPIO_CRL_MODE7_1 = cast(uint)0x20000000; /* Bit 1 */

enum GPIO_CRL_CNF = cast(uint)0xCCCCCCCC; /* Port x configuration bits */

enum GPIO_CRL_CNF0 = cast(uint)0x0000000C; /* CNF0[1:0] bits (Port x configuration bits, pin 0; */
enum GPIO_CRL_CNF0_0 = cast(uint)0x00000004; /* Bit 0 */
enum GPIO_CRL_CNF0_1 = cast(uint)0x00000008; /* Bit 1 */

enum GPIO_CRL_CNF1 = cast(uint)0x000000C0; /* CNF1[1:0] bits (Port x configuration bits, pin 1; */
enum GPIO_CRL_CNF1_0 = cast(uint)0x00000040; /* Bit 0 */
enum GPIO_CRL_CNF1_1 = cast(uint)0x00000080; /* Bit 1 */

enum GPIO_CRL_CNF2 = cast(uint)0x00000C00; /* CNF2[1:0] bits (Port x configuration bits, pin 2; */
enum GPIO_CRL_CNF2_0 = cast(uint)0x00000400; /* Bit 0 */
enum GPIO_CRL_CNF2_1 = cast(uint)0x00000800; /* Bit 1 */

enum GPIO_CRL_CNF3 = cast(uint)0x0000C000; /* CNF3[1:0] bits (Port x configuration bits, pin 3; */
enum GPIO_CRL_CNF3_0 = cast(uint)0x00004000; /* Bit 0 */
enum GPIO_CRL_CNF3_1 = cast(uint)0x00008000; /* Bit 1 */

enum GPIO_CRL_CNF4 = cast(uint)0x000C0000; /* CNF4[1:0] bits (Port x configuration bits, pin 4; */
enum GPIO_CRL_CNF4_0 = cast(uint)0x00040000; /* Bit 0 */
enum GPIO_CRL_CNF4_1 = cast(uint)0x00080000; /* Bit 1 */

enum GPIO_CRL_CNF5 = cast(uint)0x00C00000; /* CNF5[1:0] bits (Port x configuration bits, pin 5; */
enum GPIO_CRL_CNF5_0 = cast(uint)0x00400000; /* Bit 0 */
enum GPIO_CRL_CNF5_1 = cast(uint)0x00800000; /* Bit 1 */

enum GPIO_CRL_CNF6 = cast(uint)0x0C000000; /* CNF6[1:0] bits (Port x configuration bits, pin 6; */
enum GPIO_CRL_CNF6_0 = cast(uint)0x04000000; /* Bit 0 */
enum GPIO_CRL_CNF6_1 = cast(uint)0x08000000; /* Bit 1 */

enum GPIO_CRL_CNF7 = cast(uint)0xC0000000; /* CNF7[1:0] bits (Port x configuration bits, pin 7; */
enum GPIO_CRL_CNF7_0 = cast(uint)0x40000000; /* Bit 0 */
enum GPIO_CRL_CNF7_1 = cast(uint)0x80000000; /* Bit 1 */

/* Bit definition for GPIO_CRH register */
enum GPIO_CRH_MODE = cast(uint)0x33333333; /* Port x mode bits */

enum GPIO_CRH_MODE8 = cast(uint)0x00000003; /* MODE8[1:0] bits (Port x mode bits, pin 8; */
enum GPIO_CRH_MODE8_0 = cast(uint)0x00000001; /* Bit 0 */
enum GPIO_CRH_MODE8_1 = cast(uint)0x00000002; /* Bit 1 */

enum GPIO_CRH_MODE9 = cast(uint)0x00000030; /* MODE9[1:0] bits (Port x mode bits, pin 9; */
enum GPIO_CRH_MODE9_0 = cast(uint)0x00000010; /* Bit 0 */
enum GPIO_CRH_MODE9_1 = cast(uint)0x00000020; /* Bit 1 */

enum GPIO_CRH_MODE10 = cast(uint)0x00000300; /* MODE10[1:0] bits (Port x mode bits, pin 10; */
enum GPIO_CRH_MODE10_0 = cast(uint)0x00000100; /* Bit 0 */
enum GPIO_CRH_MODE10_1 = cast(uint)0x00000200; /* Bit 1 */

enum GPIO_CRH_MODE11 = cast(uint)0x00003000; /* MODE11[1:0] bits (Port x mode bits, pin 11; */
enum GPIO_CRH_MODE11_0 = cast(uint)0x00001000; /* Bit 0 */
enum GPIO_CRH_MODE11_1 = cast(uint)0x00002000; /* Bit 1 */

enum GPIO_CRH_MODE12 = cast(uint)0x00030000; /* MODE12[1:0] bits (Port x mode bits, pin 12; */
enum GPIO_CRH_MODE12_0 = cast(uint)0x00010000; /* Bit 0 */
enum GPIO_CRH_MODE12_1 = cast(uint)0x00020000; /* Bit 1 */

enum GPIO_CRH_MODE13 = cast(uint)0x00300000; /* MODE13[1:0] bits (Port x mode bits, pin 13; */
enum GPIO_CRH_MODE13_0 = cast(uint)0x00100000; /* Bit 0 */
enum GPIO_CRH_MODE13_1 = cast(uint)0x00200000; /* Bit 1 */

enum GPIO_CRH_MODE14 = cast(uint)0x03000000; /* MODE14[1:0] bits (Port x mode bits, pin 14; */
enum GPIO_CRH_MODE14_0 = cast(uint)0x01000000; /* Bit 0 */
enum GPIO_CRH_MODE14_1 = cast(uint)0x02000000; /* Bit 1 */

enum GPIO_CRH_MODE15 = cast(uint)0x30000000; /* MODE15[1:0] bits (Port x mode bits, pin 15; */
enum GPIO_CRH_MODE15_0 = cast(uint)0x10000000; /* Bit 0 */
enum GPIO_CRH_MODE15_1 = cast(uint)0x20000000; /* Bit 1 */

enum GPIO_CRH_CNF = cast(uint)0xCCCCCCCC; /* Port x configuration bits */

enum GPIO_CRH_CNF8 = cast(uint)0x0000000C; /* CNF8[1:0] bits (Port x configuration bits, pin 8; */
enum GPIO_CRH_CNF8_0 = cast(uint)0x00000004; /* Bit 0 */
enum GPIO_CRH_CNF8_1 = cast(uint)0x00000008; /* Bit 1 */

enum GPIO_CRH_CNF9 = cast(uint)0x000000C0; /* CNF9[1:0] bits (Port x configuration bits, pin 9; */
enum GPIO_CRH_CNF9_0 = cast(uint)0x00000040; /* Bit 0 */
enum GPIO_CRH_CNF9_1 = cast(uint)0x00000080; /* Bit 1 */

enum GPIO_CRH_CNF10 = cast(uint)0x00000C00; /* CNF10[1:0] bits (Port x configuration bits, pin 10; */
enum GPIO_CRH_CNF10_0 = cast(uint)0x00000400; /* Bit 0 */
enum GPIO_CRH_CNF10_1 = cast(uint)0x00000800; /* Bit 1 */

enum GPIO_CRH_CNF11 = cast(uint)0x0000C000; /* CNF11[1:0] bits (Port x configuration bits, pin 11; */
enum GPIO_CRH_CNF11_0 = cast(uint)0x00004000; /* Bit 0 */
enum GPIO_CRH_CNF11_1 = cast(uint)0x00008000; /* Bit 1 */

enum GPIO_CRH_CNF12 = cast(uint)0x000C0000; /* CNF12[1:0] bits (Port x configuration bits, pin 12; */
enum GPIO_CRH_CNF12_0 = cast(uint)0x00040000; /* Bit 0 */
enum GPIO_CRH_CNF12_1 = cast(uint)0x00080000; /* Bit 1 */

enum GPIO_CRH_CNF13 = cast(uint)0x00C00000; /* CNF13[1:0] bits (Port x configuration bits, pin 13; */
enum GPIO_CRH_CNF13_0 = cast(uint)0x00400000; /* Bit 0 */
enum GPIO_CRH_CNF13_1 = cast(uint)0x00800000; /* Bit 1 */

enum GPIO_CRH_CNF14 = cast(uint)0x0C000000; /* CNF14[1:0] bits (Port x configuration bits, pin 14; */
enum GPIO_CRH_CNF14_0 = cast(uint)0x04000000; /* Bit 0 */
enum GPIO_CRH_CNF14_1 = cast(uint)0x08000000; /* Bit 1 */

enum GPIO_CRH_CNF15 = cast(uint)0xC0000000; /* CNF15[1:0] bits (Port x configuration bits, pin 15; */
enum GPIO_CRH_CNF15_0 = cast(uint)0x40000000; /* Bit 0 */
enum GPIO_CRH_CNF15_1 = cast(uint)0x80000000; /* Bit 1 */

/* Bit definition for GPIO_IDR register */
enum GPIO_IDR_IDR0 = cast(ushort)0x0001; /* Port input data, bit 0 */
enum GPIO_IDR_IDR1 = cast(ushort)0x0002; /* Port input data, bit 1 */
enum GPIO_IDR_IDR2 = cast(ushort)0x0004; /* Port input data, bit 2 */
enum GPIO_IDR_IDR3 = cast(ushort)0x0008; /* Port input data, bit 3 */
enum GPIO_IDR_IDR4 = cast(ushort)0x0010; /* Port input data, bit 4 */
enum GPIO_IDR_IDR5 = cast(ushort)0x0020; /* Port input data, bit 5 */
enum GPIO_IDR_IDR6 = cast(ushort)0x0040; /* Port input data, bit 6 */
enum GPIO_IDR_IDR7 = cast(ushort)0x0080; /* Port input data, bit 7 */
enum GPIO_IDR_IDR8 = cast(ushort)0x0100; /* Port input data, bit 8 */
enum GPIO_IDR_IDR9 = cast(ushort)0x0200; /* Port input data, bit 9 */
enum GPIO_IDR_IDR10 = cast(ushort)0x0400; /* Port input data, bit 10 */
enum GPIO_IDR_IDR11 = cast(ushort)0x0800; /* Port input data, bit 11 */
enum GPIO_IDR_IDR12 = cast(ushort)0x1000; /* Port input data, bit 12 */
enum GPIO_IDR_IDR13 = cast(ushort)0x2000; /* Port input data, bit 13 */
enum GPIO_IDR_IDR14 = cast(ushort)0x4000; /* Port input data, bit 14 */
enum GPIO_IDR_IDR15 = cast(ushort)0x8000; /* Port input data, bit 15 */

/* Bit definition for GPIO_ODR register */
enum GPIO_ODR_ODR0 = cast(ushort)0x0001; /* Port output data, bit 0 */
enum GPIO_ODR_ODR1 = cast(ushort)0x0002; /* Port output data, bit 1 */
enum GPIO_ODR_ODR2 = cast(ushort)0x0004; /* Port output data, bit 2 */
enum GPIO_ODR_ODR3 = cast(ushort)0x0008; /* Port output data, bit 3 */
enum GPIO_ODR_ODR4 = cast(ushort)0x0010; /* Port output data, bit 4 */
enum GPIO_ODR_ODR5 = cast(ushort)0x0020; /* Port output data, bit 5 */
enum GPIO_ODR_ODR6 = cast(ushort)0x0040; /* Port output data, bit 6 */
enum GPIO_ODR_ODR7 = cast(ushort)0x0080; /* Port output data, bit 7 */
enum GPIO_ODR_ODR8 = cast(ushort)0x0100; /* Port output data, bit 8 */
enum GPIO_ODR_ODR9 = cast(ushort)0x0200; /* Port output data, bit 9 */
enum GPIO_ODR_ODR10 = cast(ushort)0x0400; /* Port output data, bit 10 */
enum GPIO_ODR_ODR11 = cast(ushort)0x0800; /* Port output data, bit 11 */
enum GPIO_ODR_ODR12 = cast(ushort)0x1000; /* Port output data, bit 12 */
enum GPIO_ODR_ODR13 = cast(ushort)0x2000; /* Port output data, bit 13 */
enum GPIO_ODR_ODR14 = cast(ushort)0x4000; /* Port output data, bit 14 */
enum GPIO_ODR_ODR15 = cast(ushort)0x8000; /* Port output data, bit 15 */

/* Bit definition for GPIO_BSRR register */
enum GPIO_BSRR_BS0 = cast(uint)0x00000001; /* Port x Set bit 0 */
enum GPIO_BSRR_BS1 = cast(uint)0x00000002; /* Port x Set bit 1 */
enum GPIO_BSRR_BS2 = cast(uint)0x00000004; /* Port x Set bit 2 */
enum GPIO_BSRR_BS3 = cast(uint)0x00000008; /* Port x Set bit 3 */
enum GPIO_BSRR_BS4 = cast(uint)0x00000010; /* Port x Set bit 4 */
enum GPIO_BSRR_BS5 = cast(uint)0x00000020; /* Port x Set bit 5 */
enum GPIO_BSRR_BS6 = cast(uint)0x00000040; /* Port x Set bit 6 */
enum GPIO_BSRR_BS7 = cast(uint)0x00000080; /* Port x Set bit 7 */
enum GPIO_BSRR_BS8 = cast(uint)0x00000100; /* Port x Set bit 8 */
enum GPIO_BSRR_BS9 = cast(uint)0x00000200; /* Port x Set bit 9 */
enum GPIO_BSRR_BS10 = cast(uint)0x00000400; /* Port x Set bit 10 */
enum GPIO_BSRR_BS11 = cast(uint)0x00000800; /* Port x Set bit 11 */
enum GPIO_BSRR_BS12 = cast(uint)0x00001000; /* Port x Set bit 12 */
enum GPIO_BSRR_BS13 = cast(uint)0x00002000; /* Port x Set bit 13 */
enum GPIO_BSRR_BS14 = cast(uint)0x00004000; /* Port x Set bit 14 */
enum GPIO_BSRR_BS15 = cast(uint)0x00008000; /* Port x Set bit 15 */

enum GPIO_BSRR_BR0 = cast(uint)0x00010000; /* Port x Reset bit 0 */
enum GPIO_BSRR_BR1 = cast(uint)0x00020000; /* Port x Reset bit 1 */
enum GPIO_BSRR_BR2 = cast(uint)0x00040000; /* Port x Reset bit 2 */
enum GPIO_BSRR_BR3 = cast(uint)0x00080000; /* Port x Reset bit 3 */
enum GPIO_BSRR_BR4 = cast(uint)0x00100000; /* Port x Reset bit 4 */
enum GPIO_BSRR_BR5 = cast(uint)0x00200000; /* Port x Reset bit 5 */
enum GPIO_BSRR_BR6 = cast(uint)0x00400000; /* Port x Reset bit 6 */
enum GPIO_BSRR_BR7 = cast(uint)0x00800000; /* Port x Reset bit 7 */
enum GPIO_BSRR_BR8 = cast(uint)0x01000000; /* Port x Reset bit 8 */
enum GPIO_BSRR_BR9 = cast(uint)0x02000000; /* Port x Reset bit 9 */
enum GPIO_BSRR_BR10 = cast(uint)0x04000000; /* Port x Reset bit 10 */
enum GPIO_BSRR_BR11 = cast(uint)0x08000000; /* Port x Reset bit 11 */
enum GPIO_BSRR_BR12 = cast(uint)0x10000000; /* Port x Reset bit 12 */
enum GPIO_BSRR_BR13 = cast(uint)0x20000000; /* Port x Reset bit 13 */
enum GPIO_BSRR_BR14 = cast(uint)0x40000000; /* Port x Reset bit 14 */
enum GPIO_BSRR_BR15 = cast(uint)0x80000000; /* Port x Reset bit 15 */

/* Bit definition for GPIO_BRR register */
enum GPIO_BRR_BR0 = cast(ushort)0x0001; /* Port x Reset bit 0 */
enum GPIO_BRR_BR1 = cast(ushort)0x0002; /* Port x Reset bit 1 */
enum GPIO_BRR_BR2 = cast(ushort)0x0004; /* Port x Reset bit 2 */
enum GPIO_BRR_BR3 = cast(ushort)0x0008; /* Port x Reset bit 3 */
enum GPIO_BRR_BR4 = cast(ushort)0x0010; /* Port x Reset bit 4 */
enum GPIO_BRR_BR5 = cast(ushort)0x0020; /* Port x Reset bit 5 */
enum GPIO_BRR_BR6 = cast(ushort)0x0040; /* Port x Reset bit 6 */
enum GPIO_BRR_BR7 = cast(ushort)0x0080; /* Port x Reset bit 7 */
enum GPIO_BRR_BR8 = cast(ushort)0x0100; /* Port x Reset bit 8 */
enum GPIO_BRR_BR9 = cast(ushort)0x0200; /* Port x Reset bit 9 */
enum GPIO_BRR_BR10 = cast(ushort)0x0400; /* Port x Reset bit 10 */
enum GPIO_BRR_BR11 = cast(ushort)0x0800; /* Port x Reset bit 11 */
enum GPIO_BRR_BR12 = cast(ushort)0x1000; /* Port x Reset bit 12 */
enum GPIO_BRR_BR13 = cast(ushort)0x2000; /* Port x Reset bit 13 */
enum GPIO_BRR_BR14 = cast(ushort)0x4000; /* Port x Reset bit 14 */
enum GPIO_BRR_BR15 = cast(ushort)0x8000; /* Port x Reset bit 15 */

/* Bit definition for GPIO_LCKR register */
enum GPIO_LCKR_LCK0 = cast(uint)0x00000001; /* Port x Lock bit 0 */
enum GPIO_LCKR_LCK1 = cast(uint)0x00000002; /* Port x Lock bit 1 */
enum GPIO_LCKR_LCK2 = cast(uint)0x00000004; /* Port x Lock bit 2 */
enum GPIO_LCKR_LCK3 = cast(uint)0x00000008; /* Port x Lock bit 3 */
enum GPIO_LCKR_LCK4 = cast(uint)0x00000010; /* Port x Lock bit 4 */
enum GPIO_LCKR_LCK5 = cast(uint)0x00000020; /* Port x Lock bit 5 */
enum GPIO_LCKR_LCK6 = cast(uint)0x00000040; /* Port x Lock bit 6 */
enum GPIO_LCKR_LCK7 = cast(uint)0x00000080; /* Port x Lock bit 7 */
enum GPIO_LCKR_LCK8 = cast(uint)0x00000100; /* Port x Lock bit 8 */
enum GPIO_LCKR_LCK9 = cast(uint)0x00000200; /* Port x Lock bit 9 */
enum GPIO_LCKR_LCK10 = cast(uint)0x00000400; /* Port x Lock bit 10 */
enum GPIO_LCKR_LCK11 = cast(uint)0x00000800; /* Port x Lock bit 11 */
enum GPIO_LCKR_LCK12 = cast(uint)0x00001000; /* Port x Lock bit 12 */
enum GPIO_LCKR_LCK13 = cast(uint)0x00002000; /* Port x Lock bit 13 */
enum GPIO_LCKR_LCK14 = cast(uint)0x00004000; /* Port x Lock bit 14 */
enum GPIO_LCKR_LCK15 = cast(uint)0x00008000; /* Port x Lock bit 15 */
enum GPIO_LCKR_LCKK = cast(uint)0x00010000; /* Lock key */

/* Bit definition for AFIO_EVCR register */
enum AFIO_EVCR_PIN = cast(ubyte)0x0F; /* PIN[3:0] bits (Pin selection; */
enum AFIO_EVCR_PIN_0 = cast(ubyte)0x01; /* Bit 0 */
enum AFIO_EVCR_PIN_1 = cast(ubyte)0x02; /* Bit 1 */
enum AFIO_EVCR_PIN_2 = cast(ubyte)0x04; /* Bit 2 */
enum AFIO_EVCR_PIN_3 = cast(ubyte)0x08; /* Bit 3 */

/* PIN configuration */
enum AFIO_EVCR_PIN_PX0 = cast(ubyte)0x00; /* Pin 0 selected */
enum AFIO_EVCR_PIN_PX1 = cast(ubyte)0x01; /* Pin 1 selected */
enum AFIO_EVCR_PIN_PX2 = cast(ubyte)0x02; /* Pin 2 selected */
enum AFIO_EVCR_PIN_PX3 = cast(ubyte)0x03; /* Pin 3 selected */
enum AFIO_EVCR_PIN_PX4 = cast(ubyte)0x04; /* Pin 4 selected */
enum AFIO_EVCR_PIN_PX5 = cast(ubyte)0x05; /* Pin 5 selected */
enum AFIO_EVCR_PIN_PX6 = cast(ubyte)0x06; /* Pin 6 selected */
enum AFIO_EVCR_PIN_PX7 = cast(ubyte)0x07; /* Pin 7 selected */
enum AFIO_EVCR_PIN_PX8 = cast(ubyte)0x08; /* Pin 8 selected */
enum AFIO_EVCR_PIN_PX9 = cast(ubyte)0x09; /* Pin 9 selected */
enum AFIO_EVCR_PIN_PX10 = cast(ubyte)0x0A; /* Pin 10 selected */
enum AFIO_EVCR_PIN_PX11 = cast(ubyte)0x0B; /* Pin 11 selected */
enum AFIO_EVCR_PIN_PX12 = cast(ubyte)0x0C; /* Pin 12 selected */
enum AFIO_EVCR_PIN_PX13 = cast(ubyte)0x0D; /* Pin 13 selected */
enum AFIO_EVCR_PIN_PX14 = cast(ubyte)0x0E; /* Pin 14 selected */
enum AFIO_EVCR_PIN_PX15 = cast(ubyte)0x0F; /* Pin 15 selected */

enum AFIO_EVCR_PORT = cast(ubyte)0x70; /* PORT[2:0] bits (Port selection; */
enum AFIO_EVCR_PORT_0 = cast(ubyte)0x10; /* Bit 0 */
enum AFIO_EVCR_PORT_1 = cast(ubyte)0x20; /* Bit 1 */
enum AFIO_EVCR_PORT_2 = cast(ubyte)0x40; /* Bit 2 */

/* PORT configuration */
enum AFIO_EVCR_PORT_PA = cast(ubyte)0x00; /* Port A selected */
enum AFIO_EVCR_PORT_PB = cast(ubyte)0x10; /* Port B selected */
enum AFIO_EVCR_PORT_PC = cast(ubyte)0x20; /* Port C selected */
enum AFIO_EVCR_PORT_PD = cast(ubyte)0x30; /* Port D selected */
enum AFIO_EVCR_PORT_PE = cast(ubyte)0x40; /* Port E selected */

enum AFIO_EVCR_EVOE = cast(ubyte)0x80; /* Event Output Enable */

/* Bit definition for AFIO_MAPR register */
enum AFIO_MAPR_SPI1_REMAP = cast(uint)0x00000001; /* SPI1 remapping */
enum AFIO_MAPR_I2C1_REMAP = cast(uint)0x00000002; /* I2C1 remapping */
enum AFIO_MAPR_USART1_REMAP = cast(uint)0x00000004; /* USART1 remapping */
enum AFIO_MAPR_USART2_REMAP = cast(uint)0x00000008; /* USART2 remapping */

enum AFIO_MAPR_USART3_REMAP = cast(uint)0x00000030; /* USART3_REMAP[1:0] bits (USART3 remapping; */
enum AFIO_MAPR_USART3_REMAP_0 = cast(uint)0x00000010; /* Bit 0 */
enum AFIO_MAPR_USART3_REMAP_1 = cast(uint)0x00000020; /* Bit 1 */

/* USART3_REMAP configuration */
enum AFIO_MAPR_USART3_REMAP_NOREMAP = cast(uint)0x00000000; /* No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14; */
enum AFIO_MAPR_USART3_REMAP_PARTIALREMAP = cast(uint)0x00000010; /* Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14; */
enum AFIO_MAPR_USART3_REMAP_FULLREMAP = cast(uint)0x00000030; /* Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12; */

enum AFIO_MAPR_TIM1_REMAP = cast(uint)0x000000C0; /* TIM1_REMAP[1:0] bits (TIM1 remapping; */
enum AFIO_MAPR_TIM1_REMAP_0 = cast(uint)0x00000040; /* Bit 0 */
enum AFIO_MAPR_TIM1_REMAP_1 = cast(uint)0x00000080; /* Bit 1 */

/* TIM1_REMAP configuration */
enum AFIO_MAPR_TIM1_REMAP_NOREMAP = cast(uint)0x00000000; /* No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15; */
enum AFIO_MAPR_TIM1_REMAP_PARTIALREMAP = cast(uint)0x00000040; /* Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1; */
enum AFIO_MAPR_TIM1_REMAP_FULLREMAP = cast(uint)0x000000C0; /* Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12; */

enum AFIO_MAPR_TIM2_REMAP = cast(uint)0x00000300; /* TIM2_REMAP[1:0] bits (TIM2 remapping; */
enum AFIO_MAPR_TIM2_REMAP_0 = cast(uint)0x00000100; /* Bit 0 */
enum AFIO_MAPR_TIM2_REMAP_1 = cast(uint)0x00000200; /* Bit 1 */

/* TIM2_REMAP configuration */
enum AFIO_MAPR_TIM2_REMAP_NOREMAP = cast(uint)0x00000000; /* No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3; */
enum AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1 = cast(uint)0x00000100; /* Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3; */
enum AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2 = cast(uint)0x00000200; /* Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11; */
enum AFIO_MAPR_TIM2_REMAP_FULLREMAP = cast(uint)0x00000300; /* Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11; */

enum AFIO_MAPR_TIM3_REMAP = cast(uint)0x00000C00; /* TIM3_REMAP[1:0] bits (TIM3 remapping; */
enum AFIO_MAPR_TIM3_REMAP_0 = cast(uint)0x00000400; /* Bit 0 */
enum AFIO_MAPR_TIM3_REMAP_1 = cast(uint)0x00000800; /* Bit 1 */

/* TIM3_REMAP configuration */
enum AFIO_MAPR_TIM3_REMAP_NOREMAP = cast(uint)0x00000000; /* No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1; */
enum AFIO_MAPR_TIM3_REMAP_PARTIALREMAP = cast(uint)0x00000800; /* Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1; */
enum AFIO_MAPR_TIM3_REMAP_FULLREMAP = cast(uint)0x00000C00; /* Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9; */

enum AFIO_MAPR_TIM4_REMAP = cast(uint)0x00001000; /* TIM4_REMAP bit (TIM4 remapping; */

enum AFIO_MAPR_CAN_REMAP = cast(uint)0x00006000; /* CAN_REMAP[1:0] bits (CAN Alternate function remapping; */
enum AFIO_MAPR_CAN_REMAP_0 = cast(uint)0x00002000; /* Bit 0 */
enum AFIO_MAPR_CAN_REMAP_1 = cast(uint)0x00004000; /* Bit 1 */

/* CAN_REMAP configuration */
enum AFIO_MAPR_CAN_REMAP_REMAP1 = cast(uint)0x00000000; /* CANRX mapped to PA11, CANTX mapped to PA12 */
enum AFIO_MAPR_CAN_REMAP_REMAP2 = cast(uint)0x00004000; /* CANRX mapped to PB8, CANTX mapped to PB9 */
enum AFIO_MAPR_CAN_REMAP_REMAP3 = cast(uint)0x00006000; /* CANRX mapped to PD0, CANTX mapped to PD1 */

enum AFIO_MAPR_PD01_REMAP = cast(uint)0x00008000; /* Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
enum AFIO_MAPR_TIM5CH4_IREMAP = cast(uint)0x00010000; /* TIM5 Channel4 Internal Remap */
enum AFIO_MAPR_ADC1_ETRGINJ_REMAP = cast(uint)0x00020000; /* ADC 1 External Trigger Injected Conversion remapping */
enum AFIO_MAPR_ADC1_ETRGREG_REMAP = cast(uint)0x00040000; /* ADC 1 External Trigger Regular Conversion remapping */
enum AFIO_MAPR_ADC2_ETRGINJ_REMAP = cast(uint)0x00080000; /* ADC 2 External Trigger Injected Conversion remapping */
enum AFIO_MAPR_ADC2_ETRGREG_REMAP = cast(uint)0x00100000; /* ADC 2 External Trigger Regular Conversion remapping */

/* SWJ_CFG configuration */
enum AFIO_MAPR_SWJ_CFG = cast(uint)0x07000000; /* SWJ_CFG[2:0] bits (Serial1 Wire JTAG configuration; */
enum AFIO_MAPR_SWJ_CFG_0 = cast(uint)0x01000000; /* Bit 0 */
enum AFIO_MAPR_SWJ_CFG_1 = cast(uint)0x02000000; /* Bit 1 */
enum AFIO_MAPR_SWJ_CFG_2 = cast(uint)0x04000000; /* Bit 2 */

enum AFIO_MAPR_SWJ_CFG_RESET = cast(uint)0x00000000; /* Full SWJ (JTAG-DP + SW-DP; : Reset State */
enum AFIO_MAPR_SWJ_CFG_NOJNTRST = cast(uint)0x01000000; /* Full SWJ (JTAG-DP + SW-DP; but without JNTRST */
enum AFIO_MAPR_SWJ_CFG_JTAGDISABLE = cast(uint)0x02000000; /* JTAG-DP Disabled and SW-DP Enabled */
enum AFIO_MAPR_SWJ_CFG_DISABLE = cast(uint)0x04000000; /* JTAG-DP Disabled and SW-DP Disabled */

/* ETH_REMAP configuration */
enum AFIO_MAPR_ETH_REMAP = cast(uint)0x00200000; /* SPI3_REMAP bit (Ethernet MAC I/O remapping; */

/* CAN2_REMAP configuration */
enum AFIO_MAPR_CAN2_REMAP = cast(uint)0x00400000; /* CAN2_REMAP bit (CAN2 I/O remapping; */

/* MII_RMII_SEL configuration */
enum AFIO_MAPR_MII_RMII_SEL = cast(uint)0x00800000; /* MII_RMII_SEL bit (Ethernet MII or RMII selection; */

/* SPI3_REMAP configuration */
enum AFIO_MAPR_SPI3_REMAP = cast(uint)0x10000000; /* SPI3_REMAP bit (SPI3 remapping; */

/* TIM2ITR1_IREMAP configuration */
enum AFIO_MAPR_TIM2ITR1_IREMAP = cast(uint)0x20000000; /* TIM2ITR1_IREMAP bit (TIM2 internal trigger 1 remapping; */

/* PTP_PPS_REMAP configuration */
enum AFIO_MAPR_PTP_PPS_REMAP = cast(uint)0x40000000; /* PTP_PPS_REMAP bit (Ethernet PTP PPS remapping; */

/* Bit definition for AFIO_EXTICR1 register */
enum AFIO_EXTICR1_EXTI0 = cast(ushort)0x000F; /* EXTI 0 configuration */
enum AFIO_EXTICR1_EXTI1 = cast(ushort)0x00F0; /* EXTI 1 configuration */
enum AFIO_EXTICR1_EXTI2 = cast(ushort)0x0F00; /* EXTI 2 configuration */
enum AFIO_EXTICR1_EXTI3 = cast(ushort)0xF000; /* EXTI 3 configuration */

/* EXTI0 configuration */
enum AFIO_EXTICR1_EXTI0_PA = cast(ushort)0x0000; /* PA[0] pin */
enum AFIO_EXTICR1_EXTI0_PB = cast(ushort)0x0001; /* PB[0] pin */
enum AFIO_EXTICR1_EXTI0_PC = cast(ushort)0x0002; /* PC[0] pin */
enum AFIO_EXTICR1_EXTI0_PD = cast(ushort)0x0003; /* PD[0] pin */
enum AFIO_EXTICR1_EXTI0_PE = cast(ushort)0x0004; /* PE[0] pin */
enum AFIO_EXTICR1_EXTI0_PF = cast(ushort)0x0005; /* PF[0] pin */
enum AFIO_EXTICR1_EXTI0_PG = cast(ushort)0x0006; /* PG[0] pin */

/* EXTI1 configuration */
enum AFIO_EXTICR1_EXTI1_PA = cast(ushort)0x0000; /* PA[1] pin */
enum AFIO_EXTICR1_EXTI1_PB = cast(ushort)0x0010; /* PB[1] pin */
enum AFIO_EXTICR1_EXTI1_PC = cast(ushort)0x0020; /* PC[1] pin */
enum AFIO_EXTICR1_EXTI1_PD = cast(ushort)0x0030; /* PD[1] pin */
enum AFIO_EXTICR1_EXTI1_PE = cast(ushort)0x0040; /* PE[1] pin */
enum AFIO_EXTICR1_EXTI1_PF = cast(ushort)0x0050; /* PF[1] pin */
enum AFIO_EXTICR1_EXTI1_PG = cast(ushort)0x0060; /* PG[1] pin */

/* EXTI2 configuration */
enum AFIO_EXTICR1_EXTI2_PA = cast(ushort)0x0000; /* PA[2] pin */
enum AFIO_EXTICR1_EXTI2_PB = cast(ushort)0x0100; /* PB[2] pin */
enum AFIO_EXTICR1_EXTI2_PC = cast(ushort)0x0200; /* PC[2] pin */
enum AFIO_EXTICR1_EXTI2_PD = cast(ushort)0x0300; /* PD[2] pin */
enum AFIO_EXTICR1_EXTI2_PE = cast(ushort)0x0400; /* PE[2] pin */
enum AFIO_EXTICR1_EXTI2_PF = cast(ushort)0x0500; /* PF[2] pin */
enum AFIO_EXTICR1_EXTI2_PG = cast(ushort)0x0600; /* PG[2] pin */

/* EXTI3 configuration */
enum AFIO_EXTICR1_EXTI3_PA = cast(ushort)0x0000; /* PA[3] pin */
enum AFIO_EXTICR1_EXTI3_PB = cast(ushort)0x1000; /* PB[3] pin */
enum AFIO_EXTICR1_EXTI3_PC = cast(ushort)0x2000; /* PC[3] pin */
enum AFIO_EXTICR1_EXTI3_PD = cast(ushort)0x3000; /* PD[3] pin */
enum AFIO_EXTICR1_EXTI3_PE = cast(ushort)0x4000; /* PE[3] pin */
enum AFIO_EXTICR1_EXTI3_PF = cast(ushort)0x5000; /* PF[3] pin */
enum AFIO_EXTICR1_EXTI3_PG = cast(ushort)0x6000; /* PG[3] pin */

/* Bit definition for AFIO_EXTICR2 register */
enum AFIO_EXTICR2_EXTI4 = cast(ushort)0x000F; /* EXTI 4 configuration */
enum AFIO_EXTICR2_EXTI5 = cast(ushort)0x00F0; /* EXTI 5 configuration */
enum AFIO_EXTICR2_EXTI6 = cast(ushort)0x0F00; /* EXTI 6 configuration */
enum AFIO_EXTICR2_EXTI7 = cast(ushort)0xF000; /* EXTI 7 configuration */

/* EXTI4 configuration */
enum AFIO_EXTICR2_EXTI4_PA = cast(ushort)0x0000; /* PA[4] pin */
enum AFIO_EXTICR2_EXTI4_PB = cast(ushort)0x0001; /* PB[4] pin */
enum AFIO_EXTICR2_EXTI4_PC = cast(ushort)0x0002; /* PC[4] pin */
enum AFIO_EXTICR2_EXTI4_PD = cast(ushort)0x0003; /* PD[4] pin */
enum AFIO_EXTICR2_EXTI4_PE = cast(ushort)0x0004; /* PE[4] pin */
enum AFIO_EXTICR2_EXTI4_PF = cast(ushort)0x0005; /* PF[4] pin */
enum AFIO_EXTICR2_EXTI4_PG = cast(ushort)0x0006; /* PG[4] pin */

/* EXTI5 configuration */
enum AFIO_EXTICR2_EXTI5_PA = cast(ushort)0x0000; /* PA[5] pin */
enum AFIO_EXTICR2_EXTI5_PB = cast(ushort)0x0010; /* PB[5] pin */
enum AFIO_EXTICR2_EXTI5_PC = cast(ushort)0x0020; /* PC[5] pin */
enum AFIO_EXTICR2_EXTI5_PD = cast(ushort)0x0030; /* PD[5] pin */
enum AFIO_EXTICR2_EXTI5_PE = cast(ushort)0x0040; /* PE[5] pin */
enum AFIO_EXTICR2_EXTI5_PF = cast(ushort)0x0050; /* PF[5] pin */
enum AFIO_EXTICR2_EXTI5_PG = cast(ushort)0x0060; /* PG[5] pin */

/* EXTI6 configuration */
enum AFIO_EXTICR2_EXTI6_PA = cast(ushort)0x0000; /* PA[6] pin */
enum AFIO_EXTICR2_EXTI6_PB = cast(ushort)0x0100; /* PB[6] pin */
enum AFIO_EXTICR2_EXTI6_PC = cast(ushort)0x0200; /* PC[6] pin */
enum AFIO_EXTICR2_EXTI6_PD = cast(ushort)0x0300; /* PD[6] pin */
enum AFIO_EXTICR2_EXTI6_PE = cast(ushort)0x0400; /* PE[6] pin */
enum AFIO_EXTICR2_EXTI6_PF = cast(ushort)0x0500; /* PF[6] pin */
enum AFIO_EXTICR2_EXTI6_PG = cast(ushort)0x0600; /* PG[6] pin */

/* EXTI7 configuration */
enum AFIO_EXTICR2_EXTI7_PA = cast(ushort)0x0000; /* PA[7] pin */
enum AFIO_EXTICR2_EXTI7_PB = cast(ushort)0x1000; /* PB[7] pin */
enum AFIO_EXTICR2_EXTI7_PC = cast(ushort)0x2000; /* PC[7] pin */
enum AFIO_EXTICR2_EXTI7_PD = cast(ushort)0x3000; /* PD[7] pin */
enum AFIO_EXTICR2_EXTI7_PE = cast(ushort)0x4000; /* PE[7] pin */
enum AFIO_EXTICR2_EXTI7_PF = cast(ushort)0x5000; /* PF[7] pin */
enum AFIO_EXTICR2_EXTI7_PG = cast(ushort)0x6000; /* PG[7] pin */

/* Bit definition for AFIO_EXTICR3 register */
enum AFIO_EXTICR3_EXTI8 = cast(ushort)0x000F; /* EXTI 8 configuration */
enum AFIO_EXTICR3_EXTI9 = cast(ushort)0x00F0; /* EXTI 9 configuration */
enum AFIO_EXTICR3_EXTI10 = cast(ushort)0x0F00; /* EXTI 10 configuration */
enum AFIO_EXTICR3_EXTI11 = cast(ushort)0xF000; /* EXTI 11 configuration */

/* EXTI8 configuration */
enum AFIO_EXTICR3_EXTI8_PA = cast(ushort)0x0000; /* PA[8] pin */
enum AFIO_EXTICR3_EXTI8_PB = cast(ushort)0x0001; /* PB[8] pin */
enum AFIO_EXTICR3_EXTI8_PC = cast(ushort)0x0002; /* PC[8] pin */
enum AFIO_EXTICR3_EXTI8_PD = cast(ushort)0x0003; /* PD[8] pin */
enum AFIO_EXTICR3_EXTI8_PE = cast(ushort)0x0004; /* PE[8] pin */
enum AFIO_EXTICR3_EXTI8_PF = cast(ushort)0x0005; /* PF[8] pin */
enum AFIO_EXTICR3_EXTI8_PG = cast(ushort)0x0006; /* PG[8] pin */

/* EXTI9 configuration */
enum AFIO_EXTICR3_EXTI9_PA = cast(ushort)0x0000; /* PA[9] pin */
enum AFIO_EXTICR3_EXTI9_PB = cast(ushort)0x0010; /* PB[9] pin */
enum AFIO_EXTICR3_EXTI9_PC = cast(ushort)0x0020; /* PC[9] pin */
enum AFIO_EXTICR3_EXTI9_PD = cast(ushort)0x0030; /* PD[9] pin */
enum AFIO_EXTICR3_EXTI9_PE = cast(ushort)0x0040; /* PE[9] pin */
enum AFIO_EXTICR3_EXTI9_PF = cast(ushort)0x0050; /* PF[9] pin */
enum AFIO_EXTICR3_EXTI9_PG = cast(ushort)0x0060; /* PG[9] pin */

/* EXTI10 configuration */
enum AFIO_EXTICR3_EXTI10_PA = cast(ushort)0x0000; /* PA[10] pin */
enum AFIO_EXTICR3_EXTI10_PB = cast(ushort)0x0100; /* PB[10] pin */
enum AFIO_EXTICR3_EXTI10_PC = cast(ushort)0x0200; /* PC[10] pin */
enum AFIO_EXTICR3_EXTI10_PD = cast(ushort)0x0300; /* PD[10] pin */
enum AFIO_EXTICR3_EXTI10_PE = cast(ushort)0x0400; /* PE[10] pin */
enum AFIO_EXTICR3_EXTI10_PF = cast(ushort)0x0500; /* PF[10] pin */
enum AFIO_EXTICR3_EXTI10_PG = cast(ushort)0x0600; /* PG[10] pin */

/* EXTI11 configuration */
enum AFIO_EXTICR3_EXTI11_PA = cast(ushort)0x0000; /* PA[11] pin */
enum AFIO_EXTICR3_EXTI11_PB = cast(ushort)0x1000; /* PB[11] pin */
enum AFIO_EXTICR3_EXTI11_PC = cast(ushort)0x2000; /* PC[11] pin */
enum AFIO_EXTICR3_EXTI11_PD = cast(ushort)0x3000; /* PD[11] pin */
enum AFIO_EXTICR3_EXTI11_PE = cast(ushort)0x4000; /* PE[11] pin */
enum AFIO_EXTICR3_EXTI11_PF = cast(ushort)0x5000; /* PF[11] pin */
enum AFIO_EXTICR3_EXTI11_PG = cast(ushort)0x6000; /* PG[11] pin */

/* Bit definition for AFIO_EXTICR4 register */
enum AFIO_EXTICR4_EXTI12 = cast(ushort)0x000F; /* EXTI 12 configuration */
enum AFIO_EXTICR4_EXTI13 = cast(ushort)0x00F0; /* EXTI 13 configuration */
enum AFIO_EXTICR4_EXTI14 = cast(ushort)0x0F00; /* EXTI 14 configuration */
enum AFIO_EXTICR4_EXTI15 = cast(ushort)0xF000; /* EXTI 15 configuration */

/* EXTI12 configuration */
enum AFIO_EXTICR4_EXTI12_PA = cast(ushort)0x0000; /* PA[12] pin */
enum AFIO_EXTICR4_EXTI12_PB = cast(ushort)0x0001; /* PB[12] pin */
enum AFIO_EXTICR4_EXTI12_PC = cast(ushort)0x0002; /* PC[12] pin */
enum AFIO_EXTICR4_EXTI12_PD = cast(ushort)0x0003; /* PD[12] pin */
enum AFIO_EXTICR4_EXTI12_PE = cast(ushort)0x0004; /* PE[12] pin */
enum AFIO_EXTICR4_EXTI12_PF = cast(ushort)0x0005; /* PF[12] pin */
enum AFIO_EXTICR4_EXTI12_PG = cast(ushort)0x0006; /* PG[12] pin */

/* EXTI13 configuration */
enum AFIO_EXTICR4_EXTI13_PA = cast(ushort)0x0000; /* PA[13] pin */
enum AFIO_EXTICR4_EXTI13_PB = cast(ushort)0x0010; /* PB[13] pin */
enum AFIO_EXTICR4_EXTI13_PC = cast(ushort)0x0020; /* PC[13] pin */
enum AFIO_EXTICR4_EXTI13_PD = cast(ushort)0x0030; /* PD[13] pin */
enum AFIO_EXTICR4_EXTI13_PE = cast(ushort)0x0040; /* PE[13] pin */
enum AFIO_EXTICR4_EXTI13_PF = cast(ushort)0x0050; /* PF[13] pin */
enum AFIO_EXTICR4_EXTI13_PG = cast(ushort)0x0060; /* PG[13] pin */

/* EXTI14 configuration */
enum AFIO_EXTICR4_EXTI14_PA = cast(ushort)0x0000; /* PA[14] pin */
enum AFIO_EXTICR4_EXTI14_PB = cast(ushort)0x0100; /* PB[14] pin */
enum AFIO_EXTICR4_EXTI14_PC = cast(ushort)0x0200; /* PC[14] pin */
enum AFIO_EXTICR4_EXTI14_PD = cast(ushort)0x0300; /* PD[14] pin */
enum AFIO_EXTICR4_EXTI14_PE = cast(ushort)0x0400; /* PE[14] pin */
enum AFIO_EXTICR4_EXTI14_PF = cast(ushort)0x0500; /* PF[14] pin */
enum AFIO_EXTICR4_EXTI14_PG = cast(ushort)0x0600; /* PG[14] pin */

/* EXTI15 configuration */
enum AFIO_EXTICR4_EXTI15_PA = cast(ushort)0x0000; /* PA[15] pin */
enum AFIO_EXTICR4_EXTI15_PB = cast(ushort)0x1000; /* PB[15] pin */
enum AFIO_EXTICR4_EXTI15_PC = cast(ushort)0x2000; /* PC[15] pin */
enum AFIO_EXTICR4_EXTI15_PD = cast(ushort)0x3000; /* PD[15] pin */
enum AFIO_EXTICR4_EXTI15_PE = cast(ushort)0x4000; /* PE[15] pin */
enum AFIO_EXTICR4_EXTI15_PF = cast(ushort)0x5000; /* PF[15] pin */
enum AFIO_EXTICR4_EXTI15_PG = cast(ushort)0x6000; /* PG[15] pin */

/* Bit definition for AFIO_MAPR2 register */
enum AFIO_MAPR2_TIM15_REMAP = cast(uint)0x00000001; /* TIM15 remapping */
enum AFIO_MAPR2_TIM16_REMAP = cast(uint)0x00000002; /* TIM16 remapping */
enum AFIO_MAPR2_TIM17_REMAP = cast(uint)0x00000004; /* TIM17 remapping */
enum AFIO_MAPR2_CEC_REMAP = cast(uint)0x00000008; /* CEC remapping */
enum AFIO_MAPR2_TIM1_DMA_REMAP = cast(uint)0x00000010; /* TIM1_DMA remapping */
enum AFIO_MAPR2_TIM13_REMAP = cast(uint)0x00000100; /* TIM13 remapping */
enum AFIO_MAPR2_TIM14_REMAP = cast(uint)0x00000200; /* TIM14 remapping */
enum AFIO_MAPR2_FSMC_NADV_REMAP = cast(uint)0x00000400; /* FSMC NADV remapping */
enum AFIO_MAPR2_TIM67_DAC_DMA_REMAP = cast(uint)0x00000800; /* TIM6/TIM7 and DAC DMA remapping */
enum AFIO_MAPR2_TIM12_REMAP = cast(uint)0x00001000; /* TIM12 remapping */
enum AFIO_MAPR2_MISC_REMAP = cast(uint)0x00002000; /* Miscellaneous remapping */
enum AFIO_MAPR2_TIM9_REMAP = cast(uint)0x00000020; /* TIM9 remapping */
enum AFIO_MAPR2_TIM10_REMAP = cast(uint)0x00000040; /* TIM10 remapping */
enum AFIO_MAPR2_TIM11_REMAP = cast(uint)0x00000080; /* TIM11 remapping */

/* Bit definition for SYS_TICK_CTRL register */
enum SYS_TICK_CTRL_ENABLE = cast(uint)0x00000001; /* Counter enable */
enum SYS_TICK_CTRL_TICKINT = cast(uint)0x00000002; /* Counting down to 0 pends the SYS_TICK handler */
enum SYS_TICK_CTRL_CLKSOURCE = cast(uint)0x00000004; /* Clock source */
enum SYS_TICK_CTRL_COUNTFLAG = cast(uint)0x00010000; /* Count Flag */

/* Bit definition for SYS_TICK_LOAD register */
enum SYS_TICK_LOAD_RELOAD = cast(uint)0x00FFFFFF; /* Value to load into the SYS_TICK Current Value Register when the counter reaches 0 */

/* Bit definition for SYS_TICK_VAL register */
enum SYS_TICK_VAL_CURRENT = cast(uint)0x00FFFFFF; /* Current value at the time the register is accessed */

/* Bit definition for SYS_TICK_CALIB register */
enum SYS_TICK_CALIB_TENMS = cast(uint)0x00FFFFFF; /* Reload value to use for 10ms timing */
enum SYS_TICK_CALIB_SKEW = cast(uint)0x40000000; /* Calibration value is not exactly 10 ms */
enum SYS_TICK_CALIB_NOREF = cast(uint)0x80000000; /* The reference clock is not provided */

/* Bit definition for NVIC_ISER register */
enum NVIC_ISER_SETENA = cast(uint)0xFFFFFFFF; /* Interrupt set enable bits */
enum NVIC_ISER_SETENA_0 = cast(uint)0x00000001; /* bit 0 */
enum NVIC_ISER_SETENA_1 = cast(uint)0x00000002; /* bit 1 */
enum NVIC_ISER_SETENA_2 = cast(uint)0x00000004; /* bit 2 */
enum NVIC_ISER_SETENA_3 = cast(uint)0x00000008; /* bit 3 */
enum NVIC_ISER_SETENA_4 = cast(uint)0x00000010; /* bit 4 */
enum NVIC_ISER_SETENA_5 = cast(uint)0x00000020; /* bit 5 */
enum NVIC_ISER_SETENA_6 = cast(uint)0x00000040; /* bit 6 */
enum NVIC_ISER_SETENA_7 = cast(uint)0x00000080; /* bit 7 */
enum NVIC_ISER_SETENA_8 = cast(uint)0x00000100; /* bit 8 */
enum NVIC_ISER_SETENA_9 = cast(uint)0x00000200; /* bit 9 */
enum NVIC_ISER_SETENA_10 = cast(uint)0x00000400; /* bit 10 */
enum NVIC_ISER_SETENA_11 = cast(uint)0x00000800; /* bit 11 */
enum NVIC_ISER_SETENA_12 = cast(uint)0x00001000; /* bit 12 */
enum NVIC_ISER_SETENA_13 = cast(uint)0x00002000; /* bit 13 */
enum NVIC_ISER_SETENA_14 = cast(uint)0x00004000; /* bit 14 */
enum NVIC_ISER_SETENA_15 = cast(uint)0x00008000; /* bit 15 */
enum NVIC_ISER_SETENA_16 = cast(uint)0x00010000; /* bit 16 */
enum NVIC_ISER_SETENA_17 = cast(uint)0x00020000; /* bit 17 */
enum NVIC_ISER_SETENA_18 = cast(uint)0x00040000; /* bit 18 */
enum NVIC_ISER_SETENA_19 = cast(uint)0x00080000; /* bit 19 */
enum NVIC_ISER_SETENA_20 = cast(uint)0x00100000; /* bit 20 */
enum NVIC_ISER_SETENA_21 = cast(uint)0x00200000; /* bit 21 */
enum NVIC_ISER_SETENA_22 = cast(uint)0x00400000; /* bit 22 */
enum NVIC_ISER_SETENA_23 = cast(uint)0x00800000; /* bit 23 */
enum NVIC_ISER_SETENA_24 = cast(uint)0x01000000; /* bit 24 */
enum NVIC_ISER_SETENA_25 = cast(uint)0x02000000; /* bit 25 */
enum NVIC_ISER_SETENA_26 = cast(uint)0x04000000; /* bit 26 */
enum NVIC_ISER_SETENA_27 = cast(uint)0x08000000; /* bit 27 */
enum NVIC_ISER_SETENA_28 = cast(uint)0x10000000; /* bit 28 */
enum NVIC_ISER_SETENA_29 = cast(uint)0x20000000; /* bit 29 */
enum NVIC_ISER_SETENA_30 = cast(uint)0x40000000; /* bit 30 */
enum NVIC_ISER_SETENA_31 = cast(uint)0x80000000; /* bit 31 */

/* Bit definition for NVIC_ICER register */
enum NVIC_ICER_CLRENA = cast(uint)0xFFFFFFFF; /* Interrupt clear-enable bits */
enum NVIC_ICER_CLRENA_0 = cast(uint)0x00000001; /* bit 0 */
enum NVIC_ICER_CLRENA_1 = cast(uint)0x00000002; /* bit 1 */
enum NVIC_ICER_CLRENA_2 = cast(uint)0x00000004; /* bit 2 */
enum NVIC_ICER_CLRENA_3 = cast(uint)0x00000008; /* bit 3 */
enum NVIC_ICER_CLRENA_4 = cast(uint)0x00000010; /* bit 4 */
enum NVIC_ICER_CLRENA_5 = cast(uint)0x00000020; /* bit 5 */
enum NVIC_ICER_CLRENA_6 = cast(uint)0x00000040; /* bit 6 */
enum NVIC_ICER_CLRENA_7 = cast(uint)0x00000080; /* bit 7 */
enum NVIC_ICER_CLRENA_8 = cast(uint)0x00000100; /* bit 8 */
enum NVIC_ICER_CLRENA_9 = cast(uint)0x00000200; /* bit 9 */
enum NVIC_ICER_CLRENA_10 = cast(uint)0x00000400; /* bit 10 */
enum NVIC_ICER_CLRENA_11 = cast(uint)0x00000800; /* bit 11 */
enum NVIC_ICER_CLRENA_12 = cast(uint)0x00001000; /* bit 12 */
enum NVIC_ICER_CLRENA_13 = cast(uint)0x00002000; /* bit 13 */
enum NVIC_ICER_CLRENA_14 = cast(uint)0x00004000; /* bit 14 */
enum NVIC_ICER_CLRENA_15 = cast(uint)0x00008000; /* bit 15 */
enum NVIC_ICER_CLRENA_16 = cast(uint)0x00010000; /* bit 16 */
enum NVIC_ICER_CLRENA_17 = cast(uint)0x00020000; /* bit 17 */
enum NVIC_ICER_CLRENA_18 = cast(uint)0x00040000; /* bit 18 */
enum NVIC_ICER_CLRENA_19 = cast(uint)0x00080000; /* bit 19 */
enum NVIC_ICER_CLRENA_20 = cast(uint)0x00100000; /* bit 20 */
enum NVIC_ICER_CLRENA_21 = cast(uint)0x00200000; /* bit 21 */
enum NVIC_ICER_CLRENA_22 = cast(uint)0x00400000; /* bit 22 */
enum NVIC_ICER_CLRENA_23 = cast(uint)0x00800000; /* bit 23 */
enum NVIC_ICER_CLRENA_24 = cast(uint)0x01000000; /* bit 24 */
enum NVIC_ICER_CLRENA_25 = cast(uint)0x02000000; /* bit 25 */
enum NVIC_ICER_CLRENA_26 = cast(uint)0x04000000; /* bit 26 */
enum NVIC_ICER_CLRENA_27 = cast(uint)0x08000000; /* bit 27 */
enum NVIC_ICER_CLRENA_28 = cast(uint)0x10000000; /* bit 28 */
enum NVIC_ICER_CLRENA_29 = cast(uint)0x20000000; /* bit 29 */
enum NVIC_ICER_CLRENA_30 = cast(uint)0x40000000; /* bit 30 */
enum NVIC_ICER_CLRENA_31 = cast(uint)0x80000000; /* bit 31 */

/* Bit definition for NVIC_ISPR register */
enum NVIC_ISPR_SETPEND = cast(uint)0xFFFFFFFF; /* Interrupt set-pending bits */
enum NVIC_ISPR_SETPEND_0 = cast(uint)0x00000001; /* bit 0 */
enum NVIC_ISPR_SETPEND_1 = cast(uint)0x00000002; /* bit 1 */
enum NVIC_ISPR_SETPEND_2 = cast(uint)0x00000004; /* bit 2 */
enum NVIC_ISPR_SETPEND_3 = cast(uint)0x00000008; /* bit 3 */
enum NVIC_ISPR_SETPEND_4 = cast(uint)0x00000010; /* bit 4 */
enum NVIC_ISPR_SETPEND_5 = cast(uint)0x00000020; /* bit 5 */
enum NVIC_ISPR_SETPEND_6 = cast(uint)0x00000040; /* bit 6 */
enum NVIC_ISPR_SETPEND_7 = cast(uint)0x00000080; /* bit 7 */
enum NVIC_ISPR_SETPEND_8 = cast(uint)0x00000100; /* bit 8 */
enum NVIC_ISPR_SETPEND_9 = cast(uint)0x00000200; /* bit 9 */
enum NVIC_ISPR_SETPEND_10 = cast(uint)0x00000400; /* bit 10 */
enum NVIC_ISPR_SETPEND_11 = cast(uint)0x00000800; /* bit 11 */
enum NVIC_ISPR_SETPEND_12 = cast(uint)0x00001000; /* bit 12 */
enum NVIC_ISPR_SETPEND_13 = cast(uint)0x00002000; /* bit 13 */
enum NVIC_ISPR_SETPEND_14 = cast(uint)0x00004000; /* bit 14 */
enum NVIC_ISPR_SETPEND_15 = cast(uint)0x00008000; /* bit 15 */
enum NVIC_ISPR_SETPEND_16 = cast(uint)0x00010000; /* bit 16 */
enum NVIC_ISPR_SETPEND_17 = cast(uint)0x00020000; /* bit 17 */
enum NVIC_ISPR_SETPEND_18 = cast(uint)0x00040000; /* bit 18 */
enum NVIC_ISPR_SETPEND_19 = cast(uint)0x00080000; /* bit 19 */
enum NVIC_ISPR_SETPEND_20 = cast(uint)0x00100000; /* bit 20 */
enum NVIC_ISPR_SETPEND_21 = cast(uint)0x00200000; /* bit 21 */
enum NVIC_ISPR_SETPEND_22 = cast(uint)0x00400000; /* bit 22 */
enum NVIC_ISPR_SETPEND_23 = cast(uint)0x00800000; /* bit 23 */
enum NVIC_ISPR_SETPEND_24 = cast(uint)0x01000000; /* bit 24 */
enum NVIC_ISPR_SETPEND_25 = cast(uint)0x02000000; /* bit 25 */
enum NVIC_ISPR_SETPEND_26 = cast(uint)0x04000000; /* bit 26 */
enum NVIC_ISPR_SETPEND_27 = cast(uint)0x08000000; /* bit 27 */
enum NVIC_ISPR_SETPEND_28 = cast(uint)0x10000000; /* bit 28 */
enum NVIC_ISPR_SETPEND_29 = cast(uint)0x20000000; /* bit 29 */
enum NVIC_ISPR_SETPEND_30 = cast(uint)0x40000000; /* bit 30 */
enum NVIC_ISPR_SETPEND_31 = cast(uint)0x80000000; /* bit 31 */

/* Bit definition for NVIC_ICPR register */
enum NVIC_ICPR_CLRPEND = cast(uint)0xFFFFFFFF; /* Interrupt clear-pending bits */
enum NVIC_ICPR_CLRPEND_0 = cast(uint)0x00000001; /* bit 0 */
enum NVIC_ICPR_CLRPEND_1 = cast(uint)0x00000002; /* bit 1 */
enum NVIC_ICPR_CLRPEND_2 = cast(uint)0x00000004; /* bit 2 */
enum NVIC_ICPR_CLRPEND_3 = cast(uint)0x00000008; /* bit 3 */
enum NVIC_ICPR_CLRPEND_4 = cast(uint)0x00000010; /* bit 4 */
enum NVIC_ICPR_CLRPEND_5 = cast(uint)0x00000020; /* bit 5 */
enum NVIC_ICPR_CLRPEND_6 = cast(uint)0x00000040; /* bit 6 */
enum NVIC_ICPR_CLRPEND_7 = cast(uint)0x00000080; /* bit 7 */
enum NVIC_ICPR_CLRPEND_8 = cast(uint)0x00000100; /* bit 8 */
enum NVIC_ICPR_CLRPEND_9 = cast(uint)0x00000200; /* bit 9 */
enum NVIC_ICPR_CLRPEND_10 = cast(uint)0x00000400; /* bit 10 */
enum NVIC_ICPR_CLRPEND_11 = cast(uint)0x00000800; /* bit 11 */
enum NVIC_ICPR_CLRPEND_12 = cast(uint)0x00001000; /* bit 12 */
enum NVIC_ICPR_CLRPEND_13 = cast(uint)0x00002000; /* bit 13 */
enum NVIC_ICPR_CLRPEND_14 = cast(uint)0x00004000; /* bit 14 */
enum NVIC_ICPR_CLRPEND_15 = cast(uint)0x00008000; /* bit 15 */
enum NVIC_ICPR_CLRPEND_16 = cast(uint)0x00010000; /* bit 16 */
enum NVIC_ICPR_CLRPEND_17 = cast(uint)0x00020000; /* bit 17 */
enum NVIC_ICPR_CLRPEND_18 = cast(uint)0x00040000; /* bit 18 */
enum NVIC_ICPR_CLRPEND_19 = cast(uint)0x00080000; /* bit 19 */
enum NVIC_ICPR_CLRPEND_20 = cast(uint)0x00100000; /* bit 20 */
enum NVIC_ICPR_CLRPEND_21 = cast(uint)0x00200000; /* bit 21 */
enum NVIC_ICPR_CLRPEND_22 = cast(uint)0x00400000; /* bit 22 */
enum NVIC_ICPR_CLRPEND_23 = cast(uint)0x00800000; /* bit 23 */
enum NVIC_ICPR_CLRPEND_24 = cast(uint)0x01000000; /* bit 24 */
enum NVIC_ICPR_CLRPEND_25 = cast(uint)0x02000000; /* bit 25 */
enum NVIC_ICPR_CLRPEND_26 = cast(uint)0x04000000; /* bit 26 */
enum NVIC_ICPR_CLRPEND_27 = cast(uint)0x08000000; /* bit 27 */
enum NVIC_ICPR_CLRPEND_28 = cast(uint)0x10000000; /* bit 28 */
enum NVIC_ICPR_CLRPEND_29 = cast(uint)0x20000000; /* bit 29 */
enum NVIC_ICPR_CLRPEND_30 = cast(uint)0x40000000; /* bit 30 */
enum NVIC_ICPR_CLRPEND_31 = cast(uint)0x80000000; /* bit 31 */

/* Bit definition for NVIC_IABR register */
enum NVIC_IABR_ACTIVE = cast(uint)0xFFFFFFFF; /* Interrupt active flags */
enum NVIC_IABR_ACTIVE_0 = cast(uint)0x00000001; /* bit 0 */
enum NVIC_IABR_ACTIVE_1 = cast(uint)0x00000002; /* bit 1 */
enum NVIC_IABR_ACTIVE_2 = cast(uint)0x00000004; /* bit 2 */
enum NVIC_IABR_ACTIVE_3 = cast(uint)0x00000008; /* bit 3 */
enum NVIC_IABR_ACTIVE_4 = cast(uint)0x00000010; /* bit 4 */
enum NVIC_IABR_ACTIVE_5 = cast(uint)0x00000020; /* bit 5 */
enum NVIC_IABR_ACTIVE_6 = cast(uint)0x00000040; /* bit 6 */
enum NVIC_IABR_ACTIVE_7 = cast(uint)0x00000080; /* bit 7 */
enum NVIC_IABR_ACTIVE_8 = cast(uint)0x00000100; /* bit 8 */
enum NVIC_IABR_ACTIVE_9 = cast(uint)0x00000200; /* bit 9 */
enum NVIC_IABR_ACTIVE_10 = cast(uint)0x00000400; /* bit 10 */
enum NVIC_IABR_ACTIVE_11 = cast(uint)0x00000800; /* bit 11 */
enum NVIC_IABR_ACTIVE_12 = cast(uint)0x00001000; /* bit 12 */
enum NVIC_IABR_ACTIVE_13 = cast(uint)0x00002000; /* bit 13 */
enum NVIC_IABR_ACTIVE_14 = cast(uint)0x00004000; /* bit 14 */
enum NVIC_IABR_ACTIVE_15 = cast(uint)0x00008000; /* bit 15 */
enum NVIC_IABR_ACTIVE_16 = cast(uint)0x00010000; /* bit 16 */
enum NVIC_IABR_ACTIVE_17 = cast(uint)0x00020000; /* bit 17 */
enum NVIC_IABR_ACTIVE_18 = cast(uint)0x00040000; /* bit 18 */
enum NVIC_IABR_ACTIVE_19 = cast(uint)0x00080000; /* bit 19 */
enum NVIC_IABR_ACTIVE_20 = cast(uint)0x00100000; /* bit 20 */
enum NVIC_IABR_ACTIVE_21 = cast(uint)0x00200000; /* bit 21 */
enum NVIC_IABR_ACTIVE_22 = cast(uint)0x00400000; /* bit 22 */
enum NVIC_IABR_ACTIVE_23 = cast(uint)0x00800000; /* bit 23 */
enum NVIC_IABR_ACTIVE_24 = cast(uint)0x01000000; /* bit 24 */
enum NVIC_IABR_ACTIVE_25 = cast(uint)0x02000000; /* bit 25 */
enum NVIC_IABR_ACTIVE_26 = cast(uint)0x04000000; /* bit 26 */
enum NVIC_IABR_ACTIVE_27 = cast(uint)0x08000000; /* bit 27 */
enum NVIC_IABR_ACTIVE_28 = cast(uint)0x10000000; /* bit 28 */
enum NVIC_IABR_ACTIVE_29 = cast(uint)0x20000000; /* bit 29 */
enum NVIC_IABR_ACTIVE_30 = cast(uint)0x40000000; /* bit 30 */
enum NVIC_IABR_ACTIVE_31 = cast(uint)0x80000000; /* bit 31 */

/* Bit definition for NVIC_PRI0 register */
enum NVIC_IPR0_PRI_0 = cast(uint)0x000000FF; /* Priority of interrupt 0 */
enum NVIC_IPR0_PRI_1 = cast(uint)0x0000FF00; /* Priority of interrupt 1 */
enum NVIC_IPR0_PRI_2 = cast(uint)0x00FF0000; /* Priority of interrupt 2 */
enum NVIC_IPR0_PRI_3 = cast(uint)0xFF000000; /* Priority of interrupt 3 */

/* Bit definition for NVIC_PRI1 register */
enum NVIC_IPR1_PRI_4 = cast(uint)0x000000FF; /* Priority of interrupt 4 */
enum NVIC_IPR1_PRI_5 = cast(uint)0x0000FF00; /* Priority of interrupt 5 */
enum NVIC_IPR1_PRI_6 = cast(uint)0x00FF0000; /* Priority of interrupt 6 */
enum NVIC_IPR1_PRI_7 = cast(uint)0xFF000000; /* Priority of interrupt 7 */

/* Bit definition for NVIC_PRI2 register */
enum NVIC_IPR2_PRI_8 = cast(uint)0x000000FF; /* Priority of interrupt 8 */
enum NVIC_IPR2_PRI_9 = cast(uint)0x0000FF00; /* Priority of interrupt 9 */
enum NVIC_IPR2_PRI_10 = cast(uint)0x00FF0000; /* Priority of interrupt 10 */
enum NVIC_IPR2_PRI_11 = cast(uint)0xFF000000; /* Priority of interrupt 11 */

/* Bit definition for NVIC_PRI3 register */
enum NVIC_IPR3_PRI_12 = cast(uint)0x000000FF; /* Priority of interrupt 12 */
enum NVIC_IPR3_PRI_13 = cast(uint)0x0000FF00; /* Priority of interrupt 13 */
enum NVIC_IPR3_PRI_14 = cast(uint)0x00FF0000; /* Priority of interrupt 14 */
enum NVIC_IPR3_PRI_15 = cast(uint)0xFF000000; /* Priority of interrupt 15 */

/* Bit definition for NVIC_PRI4 register */
enum NVIC_IPR4_PRI_16 = cast(uint)0x000000FF; /* Priority of interrupt 16 */
enum NVIC_IPR4_PRI_17 = cast(uint)0x0000FF00; /* Priority of interrupt 17 */
enum NVIC_IPR4_PRI_18 = cast(uint)0x00FF0000; /* Priority of interrupt 18 */
enum NVIC_IPR4_PRI_19 = cast(uint)0xFF000000; /* Priority of interrupt 19 */

/* Bit definition for NVIC_PRI5 register */
enum NVIC_IPR5_PRI_20 = cast(uint)0x000000FF; /* Priority of interrupt 20 */
enum NVIC_IPR5_PRI_21 = cast(uint)0x0000FF00; /* Priority of interrupt 21 */
enum NVIC_IPR5_PRI_22 = cast(uint)0x00FF0000; /* Priority of interrupt 22 */
enum NVIC_IPR5_PRI_23 = cast(uint)0xFF000000; /* Priority of interrupt 23 */

/* Bit definition for NVIC_PRI6 register */
enum NVIC_IPR6_PRI_24 = cast(uint)0x000000FF; /* Priority of interrupt 24 */
enum NVIC_IPR6_PRI_25 = cast(uint)0x0000FF00; /* Priority of interrupt 25 */
enum NVIC_IPR6_PRI_26 = cast(uint)0x00FF0000; /* Priority of interrupt 26 */
enum NVIC_IPR6_PRI_27 = cast(uint)0xFF000000; /* Priority of interrupt 27 */

/* Bit definition for NVIC_PRI7 register */
enum NVIC_IPR7_PRI_28 = cast(uint)0x000000FF; /* Priority of interrupt 28 */
enum NVIC_IPR7_PRI_29 = cast(uint)0x0000FF00; /* Priority of interrupt 29 */
enum NVIC_IPR7_PRI_30 = cast(uint)0x00FF0000; /* Priority of interrupt 30 */
enum NVIC_IPR7_PRI_31 = cast(uint)0xFF000000; /* Priority of interrupt 31 */

/* Bit definition for SCB_CPUID register */
enum SCB_CPUID_REVISION = cast(uint)0x0000000F; /* Implementation defined revision number */
enum SCB_CPUID_PARTNO = cast(uint)0x0000FFF0; /* Number of processor within family */
enum SCB_CPUID_CONSTANT = cast(uint)0x000F0000; /* Reads as 0x0F */
enum SCB_CPUID_VARIANT = cast(uint)0x00F00000; /* Implementation defined variant number */
enum SCB_CPUID_IMPLEMENTER = cast(uint)0xFF000000; /* Implementer code. ARM is 0x41 */

/* Bit definition for SCB_ICSR register */
enum SCB_ICSR_VECTACTIVE = cast(uint)0x000001FF; /* Active ISR number field */
enum SCB_ICSR_RETTOBASE = cast(uint)0x00000800; /* All active exceptions minus the IPSR_current_exception yields the empty set */
enum SCB_ICSR_VECTPENDING = cast(uint)0x003FF000; /* Pending ISR number field */
enum SCB_ICSR_ISRPENDING = cast(uint)0x00400000; /* Interrupt pending flag */
enum SCB_ICSR_ISRPREEMPT = cast(uint)0x00800000; /* It indicates that a pending interrupt becomes active in the next running cycle */
enum SCB_ICSR_PENDSTCLR = cast(uint)0x02000000; /* Clear pending SYS_TICK bit */
enum SCB_ICSR_PENDSTSET = cast(uint)0x04000000; /* Set pending SYS_TICK bit */
enum SCB_ICSR_PENDSVCLR = cast(uint)0x08000000; /* Clear pending pendSV bit */
enum SCB_ICSR_PENDSVSET = cast(uint)0x10000000; /* Set pending pendSV bit */
enum SCB_ICSR_NMIPENDSET = cast(uint)0x80000000; /* Set pending NMI bit */

/* Bit definition for SCB_VTOR register */
enum SCB_VTOR_TBLOFF = cast(uint)0x1FFFFF80; /* Vector table base offset field */
enum SCB_VTOR_TBLBASE = cast(uint)0x20000000; /* Table base in code(0; or RAM(1; */

/* Bit definition for SCB_AIRCR register */
enum SCB_AIRCR_VECTRESET = cast(uint)0x00000001; /* System Reset bit */
enum SCB_AIRCR_VECTCLRACTIVE = cast(uint)0x00000002; /* Clear active vector bit */
enum SCB_AIRCR_SYSRESETREQ = cast(uint)0x00000004; /* Requests chip control logic to generate a reset */

enum SCB_AIRCR_PRIGROUP = cast(uint)0x00000700; /* PRIGROUP[2:0] bits (Priority group; */
enum SCB_AIRCR_PRIGROUP_0 = cast(uint)0x00000100; /* Bit 0 */
enum SCB_AIRCR_PRIGROUP_1 = cast(uint)0x00000200; /* Bit 1 */
enum SCB_AIRCR_PRIGROUP_2 = cast(uint)0x00000400; /* Bit 2 */

/* prority group configuration */
enum SCB_AIRCR_PRIGROUP0 = cast(uint)0x00000000; /* Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority; */
enum SCB_AIRCR_PRIGROUP1 = cast(uint)0x00000100; /* Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority; */
enum SCB_AIRCR_PRIGROUP2 = cast(uint)0x00000200; /* Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority; */
enum SCB_AIRCR_PRIGROUP3 = cast(uint)0x00000300; /* Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority; */
enum SCB_AIRCR_PRIGROUP4 = cast(uint)0x00000400; /* Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority; */
enum SCB_AIRCR_PRIGROUP5 = cast(uint)0x00000500; /* Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority; */
enum SCB_AIRCR_PRIGROUP6 = cast(uint)0x00000600; /* Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority; */
enum SCB_AIRCR_PRIGROUP7 = cast(uint)0x00000700; /* Priority group=7 (no pre-emption priority, 8 bits of subpriority; */

enum SCB_AIRCR_ENDIANESS = cast(uint)0x00008000; /* Data endianness bit */
enum SCB_AIRCR_VECTKEY = cast(uint)0xFFFF0000; /* Register key (VECTKEY; - Reads as 0xFA05 (VECTKEYSTAT; */

/* Bit definition for SCB_SCR register */
enum SCB_SCR_SLEEPONEXIT = cast(ubyte)0x02; /* Sleep on exit bit */
enum SCB_SCR_SLEEPDEEP = cast(ubyte)0x04; /* Sleep deep bit */
enum SCB_SCR_SEVONPEND = cast(ubyte)0x10; /* Wake up from WFE */

/* Bit definition for SCB_CCR register */
enum SCB_CCR_NONBASETHRDENA = cast(ushort)0x0001; /* Thread mode can be entered from any level in Handler mode by controlled return value */
enum SCB_CCR_USERSETMPEND = cast(ushort)0x0002; /* Enables user code to write the Software Trigger Interrupt register to trigger (pend; a Main exception */
enum SCB_CCR_UNALIGN_TRP = cast(ushort)0x0008; /* Trap for unaligned access */
enum SCB_CCR_DIV_0_TRP = cast(ushort)0x0010; /* Trap on DIVide by 0 */
enum SCB_CCR_BFHFNMIGN = cast(ushort)0x0100; /* Handlers running at priority -1 and -2 */
enum SCB_CCR_STKALIGN = cast(ushort)0x0200; /* On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned */

/* Bit definition for SCB_SHPR register */
enum SCB_SHPR_PRI_N = cast(uint)0x000000FF; /* Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor */
enum SCB_SHPR_PRI_N1 = cast(uint)0x0000FF00; /* Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved */
enum SCB_SHPR_PRI_N2 = cast(uint)0x00FF0000; /* Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV */
enum SCB_SHPR_PRI_N3 = cast(uint)0xFF000000; /* Priority of system handler 7,11, and 15. Reserved, SVCall and SYS_TICK */

/* Bit definition for SCB_SHCSR register */
enum SCB_SHCSR_MEMFAULTACT = cast(uint)0x00000001; /* MemManage is active */
enum SCB_SHCSR_BUSFAULTACT = cast(uint)0x00000002; /* BusFault is active */
enum SCB_SHCSR_USGFAULTACT = cast(uint)0x00000008; /* UsageFault is active */
enum SCB_SHCSR_SVCALLACT = cast(uint)0x00000080; /* SVCall is active */
enum SCB_SHCSR_MONITORACT = cast(uint)0x00000100; /* Monitor is active */
enum SCB_SHCSR_PENDSVACT = cast(uint)0x00000400; /* PendSV is active */
enum SCB_SHCSR_SYSTICKACT = cast(uint)0x00000800; /* SYS_TICK is active */
enum SCB_SHCSR_USGFAULTPENDED = cast(uint)0x00001000; /* Usage Fault is pended */
enum SCB_SHCSR_MEMFAULTPENDED = cast(uint)0x00002000; /* MemManage is pended */
enum SCB_SHCSR_BUSFAULTPENDED = cast(uint)0x00004000; /* Bus Fault is pended */
enum SCB_SHCSR_SVCALLPENDED = cast(uint)0x00008000; /* SVCall is pended */
enum SCB_SHCSR_MEMFAULTENA = cast(uint)0x00010000; /* MemManage enable */
enum SCB_SHCSR_BUSFAULTENA = cast(uint)0x00020000; /* Bus Fault enable */
enum SCB_SHCSR_USGFAULTENA = cast(uint)0x00040000; /* UsageFault enable */

/* Bit definition for SCB_CFSR register */
/* MFSR */
enum SCB_CFSR_IACCVIOL = cast(uint)0x00000001; /* Instruction access violation */
enum SCB_CFSR_DACCVIOL = cast(uint)0x00000002; /* Data access violation */
enum SCB_CFSR_MUNSTKERR = cast(uint)0x00000008; /* Unstacking error */
enum SCB_CFSR_MSTKERR = cast(uint)0x00000010; /* Stacking error */
enum SCB_CFSR_MMARVALID = cast(uint)0x00000080; /* Memory Manage Address Register address valid flag */
/* BFSR */
enum SCB_CFSR_IBUSERR = cast(uint)0x00000100; /* Instruction bus error flag */
enum SCB_CFSR_PRECISERR = cast(uint)0x00000200; /* Precise data bus error */
enum SCB_CFSR_IMPRECISERR = cast(uint)0x00000400; /* Imprecise data bus error */
enum SCB_CFSR_UNSTKERR = cast(uint)0x00000800; /* Unstacking error */
enum SCB_CFSR_STKERR = cast(uint)0x00001000; /* Stacking error */
enum SCB_CFSR_BFARVALID = cast(uint)0x00008000; /* Bus Fault Address Register address valid flag */
/* UFSR */
enum SCB_CFSR_UNDEFINSTR = cast(uint)0x00010000; /* The processor attempt to execute an undefined instruction */
enum SCB_CFSR_INVSTATE = cast(uint)0x00020000; /* Invalid combination of EPSR and instruction */
enum SCB_CFSR_INVPC = cast(uint)0x00040000; /* Attempt to load EXC_RETURN into pc illegally */
enum SCB_CFSR_NOCP = cast(uint)0x00080000; /* Attempt to use a coprocessor instruction */
enum SCB_CFSR_UNALIGNED = cast(uint)0x01000000; /* Fault occurs when there is an attempt to make an unaligned memory access */
enum SCB_CFSR_DIVBYZERO = cast(uint)0x02000000; /* Fault occurs when SDIV or DIV instruction is used with a divisor of 0 */

/* Bit definition for SCB_HFSR register */
enum SCB_HFSR_VECTTBL = cast(uint)0x00000002; /* Fault occurs because of vector table read on exception processing */
enum SCB_HFSR_FORCED = cast(uint)0x40000000; /* Hard Fault activated when a configurable Fault was received and cannot activate */
enum SCB_HFSR_DEBUGEVT = cast(uint)0x80000000; /* Fault related to debug */

/* Bit definition for SCB_DFSR register */
enum SCB_DFSR_HALTED = cast(ubyte)0x01; /* Halt request flag */
enum SCB_DFSR_BKPT = cast(ubyte)0x02; /* BKPT flag */
enum SCB_DFSR_DWTTRAP = cast(ubyte)0x04; /* Data Watchpoint and Trace (DWT; flag */
enum SCB_DFSR_VCATCH = cast(ubyte)0x08; /* Vector catch flag */
enum SCB_DFSR_EXTERNAL = cast(ubyte)0x10; /* External debug request flag */

/* Bit definition for SCB_MMFAR register */
enum SCB_MMFAR_ADDRESS = cast(uint)0xFFFFFFFF; /* Mem Manage fault address field */

/* Bit definition for SCB_BFAR register */
enum SCB_BFAR_ADDRESS = cast(uint)0xFFFFFFFF; /* Bus fault address field */

/* Bit definition for SCB_afsr register */
enum SCB_AFSR_IMPDEF = cast(uint)0xFFFFFFFF; /* Implementation defined */

/* Bit definition for EXTI_IMR register */
enum EXTI_IMR_MR0 = cast(uint)0x00000001; /* Interrupt Mask on line 0 */
enum EXTI_IMR_MR1 = cast(uint)0x00000002; /* Interrupt Mask on line 1 */
enum EXTI_IMR_MR2 = cast(uint)0x00000004; /* Interrupt Mask on line 2 */
enum EXTI_IMR_MR3 = cast(uint)0x00000008; /* Interrupt Mask on line 3 */
enum EXTI_IMR_MR4 = cast(uint)0x00000010; /* Interrupt Mask on line 4 */
enum EXTI_IMR_MR5 = cast(uint)0x00000020; /* Interrupt Mask on line 5 */
enum EXTI_IMR_MR6 = cast(uint)0x00000040; /* Interrupt Mask on line 6 */
enum EXTI_IMR_MR7 = cast(uint)0x00000080; /* Interrupt Mask on line 7 */
enum EXTI_IMR_MR8 = cast(uint)0x00000100; /* Interrupt Mask on line 8 */
enum EXTI_IMR_MR9 = cast(uint)0x00000200; /* Interrupt Mask on line 9 */
enum EXTI_IMR_MR10 = cast(uint)0x00000400; /* Interrupt Mask on line 10 */
enum EXTI_IMR_MR11 = cast(uint)0x00000800; /* Interrupt Mask on line 11 */
enum EXTI_IMR_MR12 = cast(uint)0x00001000; /* Interrupt Mask on line 12 */
enum EXTI_IMR_MR13 = cast(uint)0x00002000; /* Interrupt Mask on line 13 */
enum EXTI_IMR_MR14 = cast(uint)0x00004000; /* Interrupt Mask on line 14 */
enum EXTI_IMR_MR15 = cast(uint)0x00008000; /* Interrupt Mask on line 15 */
enum EXTI_IMR_MR16 = cast(uint)0x00010000; /* Interrupt Mask on line 16 */
enum EXTI_IMR_MR17 = cast(uint)0x00020000; /* Interrupt Mask on line 17 */
enum EXTI_IMR_MR18 = cast(uint)0x00040000; /* Interrupt Mask on line 18 */
enum EXTI_IMR_MR19 = cast(uint)0x00080000; /* Interrupt Mask on line 19 */

/* Bit definition for EXTI_EMR register */
enum EXTI_EMR_MR0 = cast(uint)0x00000001; /* Event Mask on line 0 */
enum EXTI_EMR_MR1 = cast(uint)0x00000002; /* Event Mask on line 1 */
enum EXTI_EMR_MR2 = cast(uint)0x00000004; /* Event Mask on line 2 */
enum EXTI_EMR_MR3 = cast(uint)0x00000008; /* Event Mask on line 3 */
enum EXTI_EMR_MR4 = cast(uint)0x00000010; /* Event Mask on line 4 */
enum EXTI_EMR_MR5 = cast(uint)0x00000020; /* Event Mask on line 5 */
enum EXTI_EMR_MR6 = cast(uint)0x00000040; /* Event Mask on line 6 */
enum EXTI_EMR_MR7 = cast(uint)0x00000080; /* Event Mask on line 7 */
enum EXTI_EMR_MR8 = cast(uint)0x00000100; /* Event Mask on line 8 */
enum EXTI_EMR_MR9 = cast(uint)0x00000200; /* Event Mask on line 9 */
enum EXTI_EMR_MR10 = cast(uint)0x00000400; /* Event Mask on line 10 */
enum EXTI_EMR_MR11 = cast(uint)0x00000800; /* Event Mask on line 11 */
enum EXTI_EMR_MR12 = cast(uint)0x00001000; /* Event Mask on line 12 */
enum EXTI_EMR_MR13 = cast(uint)0x00002000; /* Event Mask on line 13 */
enum EXTI_EMR_MR14 = cast(uint)0x00004000; /* Event Mask on line 14 */
enum EXTI_EMR_MR15 = cast(uint)0x00008000; /* Event Mask on line 15 */
enum EXTI_EMR_MR16 = cast(uint)0x00010000; /* Event Mask on line 16 */
enum EXTI_EMR_MR17 = cast(uint)0x00020000; /* Event Mask on line 17 */
enum EXTI_EMR_MR18 = cast(uint)0x00040000; /* Event Mask on line 18 */
enum EXTI_EMR_MR19 = cast(uint)0x00080000; /* Event Mask on line 19 */

/* Bit definition for EXTI_RTSR register */
enum EXTI_RTSR_TR0 = cast(uint)0x00000001; /* Rising trigger event configuration bit of line 0 */
enum EXTI_RTSR_TR1 = cast(uint)0x00000002; /* Rising trigger event configuration bit of line 1 */
enum EXTI_RTSR_TR2 = cast(uint)0x00000004; /* Rising trigger event configuration bit of line 2 */
enum EXTI_RTSR_TR3 = cast(uint)0x00000008; /* Rising trigger event configuration bit of line 3 */
enum EXTI_RTSR_TR4 = cast(uint)0x00000010; /* Rising trigger event configuration bit of line 4 */
enum EXTI_RTSR_TR5 = cast(uint)0x00000020; /* Rising trigger event configuration bit of line 5 */
enum EXTI_RTSR_TR6 = cast(uint)0x00000040; /* Rising trigger event configuration bit of line 6 */
enum EXTI_RTSR_TR7 = cast(uint)0x00000080; /* Rising trigger event configuration bit of line 7 */
enum EXTI_RTSR_TR8 = cast(uint)0x00000100; /* Rising trigger event configuration bit of line 8 */
enum EXTI_RTSR_TR9 = cast(uint)0x00000200; /* Rising trigger event configuration bit of line 9 */
enum EXTI_RTSR_TR10 = cast(uint)0x00000400; /* Rising trigger event configuration bit of line 10 */
enum EXTI_RTSR_TR11 = cast(uint)0x00000800; /* Rising trigger event configuration bit of line 11 */
enum EXTI_RTSR_TR12 = cast(uint)0x00001000; /* Rising trigger event configuration bit of line 12 */
enum EXTI_RTSR_TR13 = cast(uint)0x00002000; /* Rising trigger event configuration bit of line 13 */
enum EXTI_RTSR_TR14 = cast(uint)0x00004000; /* Rising trigger event configuration bit of line 14 */
enum EXTI_RTSR_TR15 = cast(uint)0x00008000; /* Rising trigger event configuration bit of line 15 */
enum EXTI_RTSR_TR16 = cast(uint)0x00010000; /* Rising trigger event configuration bit of line 16 */
enum EXTI_RTSR_TR17 = cast(uint)0x00020000; /* Rising trigger event configuration bit of line 17 */
enum EXTI_RTSR_TR18 = cast(uint)0x00040000; /* Rising trigger event configuration bit of line 18 */
enum EXTI_RTSR_TR19 = cast(uint)0x00080000; /* Rising trigger event configuration bit of line 19 */

/* Bit definition for EXTI_FTSR register */
enum EXTI_FTSR_TR0 = cast(uint)0x00000001; /* Falling trigger event configuration bit of line 0 */
enum EXTI_FTSR_TR1 = cast(uint)0x00000002; /* Falling trigger event configuration bit of line 1 */
enum EXTI_FTSR_TR2 = cast(uint)0x00000004; /* Falling trigger event configuration bit of line 2 */
enum EXTI_FTSR_TR3 = cast(uint)0x00000008; /* Falling trigger event configuration bit of line 3 */
enum EXTI_FTSR_TR4 = cast(uint)0x00000010; /* Falling trigger event configuration bit of line 4 */
enum EXTI_FTSR_TR5 = cast(uint)0x00000020; /* Falling trigger event configuration bit of line 5 */
enum EXTI_FTSR_TR6 = cast(uint)0x00000040; /* Falling trigger event configuration bit of line 6 */
enum EXTI_FTSR_TR7 = cast(uint)0x00000080; /* Falling trigger event configuration bit of line 7 */
enum EXTI_FTSR_TR8 = cast(uint)0x00000100; /* Falling trigger event configuration bit of line 8 */
enum EXTI_FTSR_TR9 = cast(uint)0x00000200; /* Falling trigger event configuration bit of line 9 */
enum EXTI_FTSR_TR10 = cast(uint)0x00000400; /* Falling trigger event configuration bit of line 10 */
enum EXTI_FTSR_TR11 = cast(uint)0x00000800; /* Falling trigger event configuration bit of line 11 */
enum EXTI_FTSR_TR12 = cast(uint)0x00001000; /* Falling trigger event configuration bit of line 12 */
enum EXTI_FTSR_TR13 = cast(uint)0x00002000; /* Falling trigger event configuration bit of line 13 */
enum EXTI_FTSR_TR14 = cast(uint)0x00004000; /* Falling trigger event configuration bit of line 14 */
enum EXTI_FTSR_TR15 = cast(uint)0x00008000; /* Falling trigger event configuration bit of line 15 */
enum EXTI_FTSR_TR16 = cast(uint)0x00010000; /* Falling trigger event configuration bit of line 16 */
enum EXTI_FTSR_TR17 = cast(uint)0x00020000; /* Falling trigger event configuration bit of line 17 */
enum EXTI_FTSR_TR18 = cast(uint)0x00040000; /* Falling trigger event configuration bit of line 18 */
enum EXTI_FTSR_TR19 = cast(uint)0x00080000; /* Falling trigger event configuration bit of line 19 */

/* Bit definition for EXTI_SWIER register */
enum EXTI_SWIER_SWIER0 = cast(uint)0x00000001; /* Software Interrupt on line 0 */
enum EXTI_SWIER_SWIER1 = cast(uint)0x00000002; /* Software Interrupt on line 1 */
enum EXTI_SWIER_SWIER2 = cast(uint)0x00000004; /* Software Interrupt on line 2 */
enum EXTI_SWIER_SWIER3 = cast(uint)0x00000008; /* Software Interrupt on line 3 */
enum EXTI_SWIER_SWIER4 = cast(uint)0x00000010; /* Software Interrupt on line 4 */
enum EXTI_SWIER_SWIER5 = cast(uint)0x00000020; /* Software Interrupt on line 5 */
enum EXTI_SWIER_SWIER6 = cast(uint)0x00000040; /* Software Interrupt on line 6 */
enum EXTI_SWIER_SWIER7 = cast(uint)0x00000080; /* Software Interrupt on line 7 */
enum EXTI_SWIER_SWIER8 = cast(uint)0x00000100; /* Software Interrupt on line 8 */
enum EXTI_SWIER_SWIER9 = cast(uint)0x00000200; /* Software Interrupt on line 9 */
enum EXTI_SWIER_SWIER10 = cast(uint)0x00000400; /* Software Interrupt on line 10 */
enum EXTI_SWIER_SWIER11 = cast(uint)0x00000800; /* Software Interrupt on line 11 */
enum EXTI_SWIER_SWIER12 = cast(uint)0x00001000; /* Software Interrupt on line 12 */
enum EXTI_SWIER_SWIER13 = cast(uint)0x00002000; /* Software Interrupt on line 13 */
enum EXTI_SWIER_SWIER14 = cast(uint)0x00004000; /* Software Interrupt on line 14 */
enum EXTI_SWIER_SWIER15 = cast(uint)0x00008000; /* Software Interrupt on line 15 */
enum EXTI_SWIER_SWIER16 = cast(uint)0x00010000; /* Software Interrupt on line 16 */
enum EXTI_SWIER_SWIER17 = cast(uint)0x00020000; /* Software Interrupt on line 17 */
enum EXTI_SWIER_SWIER18 = cast(uint)0x00040000; /* Software Interrupt on line 18 */
enum EXTI_SWIER_SWIER19 = cast(uint)0x00080000; /* Software Interrupt on line 19 */

/* Bit definition for EXTI_PR register */
enum EXTI_PR_PR0 = cast(uint)0x00000001; /* Pending bit for line 0 */
enum EXTI_PR_PR1 = cast(uint)0x00000002; /* Pending bit for line 1 */
enum EXTI_PR_PR2 = cast(uint)0x00000004; /* Pending bit for line 2 */
enum EXTI_PR_PR3 = cast(uint)0x00000008; /* Pending bit for line 3 */
enum EXTI_PR_PR4 = cast(uint)0x00000010; /* Pending bit for line 4 */
enum EXTI_PR_PR5 = cast(uint)0x00000020; /* Pending bit for line 5 */
enum EXTI_PR_PR6 = cast(uint)0x00000040; /* Pending bit for line 6 */
enum EXTI_PR_PR7 = cast(uint)0x00000080; /* Pending bit for line 7 */
enum EXTI_PR_PR8 = cast(uint)0x00000100; /* Pending bit for line 8 */
enum EXTI_PR_PR9 = cast(uint)0x00000200; /* Pending bit for line 9 */
enum EXTI_PR_PR10 = cast(uint)0x00000400; /* Pending bit for line 10 */
enum EXTI_PR_PR11 = cast(uint)0x00000800; /* Pending bit for line 11 */
enum EXTI_PR_PR12 = cast(uint)0x00001000; /* Pending bit for line 12 */
enum EXTI_PR_PR13 = cast(uint)0x00002000; /* Pending bit for line 13 */
enum EXTI_PR_PR14 = cast(uint)0x00004000; /* Pending bit for line 14 */
enum EXTI_PR_PR15 = cast(uint)0x00008000; /* Pending bit for line 15 */
enum EXTI_PR_PR16 = cast(uint)0x00010000; /* Pending bit for line 16 */
enum EXTI_PR_PR17 = cast(uint)0x00020000; /* Pending bit for line 17 */
enum EXTI_PR_PR18 = cast(uint)0x00040000; /* Pending bit for line 18 */
enum EXTI_PR_PR19 = cast(uint)0x00080000; /* Pending bit for line 19 */

/* Bit definition for DMA_ISR register */
enum DMA_ISR_GIF1 = cast(uint)0x00000001; /* Channel 1 Global interrupt flag */
enum DMA_ISR_TCIF1 = cast(uint)0x00000002; /* Channel 1 Transfer Complete flag */
enum DMA_ISR_HTIF1 = cast(uint)0x00000004; /* Channel 1 Half Transfer flag */
enum DMA_ISR_TEIF1 = cast(uint)0x00000008; /* Channel 1 Transfer Error flag */
enum DMA_ISR_GIF2 = cast(uint)0x00000010; /* Channel 2 Global interrupt flag */
enum DMA_ISR_TCIF2 = cast(uint)0x00000020; /* Channel 2 Transfer Complete flag */
enum DMA_ISR_HTIF2 = cast(uint)0x00000040; /* Channel 2 Half Transfer flag */
enum DMA_ISR_TEIF2 = cast(uint)0x00000080; /* Channel 2 Transfer Error flag */
enum DMA_ISR_GIF3 = cast(uint)0x00000100; /* Channel 3 Global interrupt flag */
enum DMA_ISR_TCIF3 = cast(uint)0x00000200; /* Channel 3 Transfer Complete flag */
enum DMA_ISR_HTIF3 = cast(uint)0x00000400; /* Channel 3 Half Transfer flag */
enum DMA_ISR_TEIF3 = cast(uint)0x00000800; /* Channel 3 Transfer Error flag */
enum DMA_ISR_GIF4 = cast(uint)0x00001000; /* Channel 4 Global interrupt flag */
enum DMA_ISR_TCIF4 = cast(uint)0x00002000; /* Channel 4 Transfer Complete flag */
enum DMA_ISR_HTIF4 = cast(uint)0x00004000; /* Channel 4 Half Transfer flag */
enum DMA_ISR_TEIF4 = cast(uint)0x00008000; /* Channel 4 Transfer Error flag */
enum DMA_ISR_GIF5 = cast(uint)0x00010000; /* Channel 5 Global interrupt flag */
enum DMA_ISR_TCIF5 = cast(uint)0x00020000; /* Channel 5 Transfer Complete flag */
enum DMA_ISR_HTIF5 = cast(uint)0x00040000; /* Channel 5 Half Transfer flag */
enum DMA_ISR_TEIF5 = cast(uint)0x00080000; /* Channel 5 Transfer Error flag */
enum DMA_ISR_GIF6 = cast(uint)0x00100000; /* Channel 6 Global interrupt flag */
enum DMA_ISR_TCIF6 = cast(uint)0x00200000; /* Channel 6 Transfer Complete flag */
enum DMA_ISR_HTIF6 = cast(uint)0x00400000; /* Channel 6 Half Transfer flag */
enum DMA_ISR_TEIF6 = cast(uint)0x00800000; /* Channel 6 Transfer Error flag */
enum DMA_ISR_GIF7 = cast(uint)0x01000000; /* Channel 7 Global interrupt flag */
enum DMA_ISR_TCIF7 = cast(uint)0x02000000; /* Channel 7 Transfer Complete flag */
enum DMA_ISR_HTIF7 = cast(uint)0x04000000; /* Channel 7 Half Transfer flag */
enum DMA_ISR_TEIF7 = cast(uint)0x08000000; /* Channel 7 Transfer Error flag */

/* Bit definition for DMA_IFCR register */
enum DMA_IFCR_CGIF1 = cast(uint)0x00000001; /* Channel 1 Global interrupt clear */
enum DMA_IFCR_CTCIF1 = cast(uint)0x00000002; /* Channel 1 Transfer Complete clear */
enum DMA_IFCR_CHTIF1 = cast(uint)0x00000004; /* Channel 1 Half Transfer clear */
enum DMA_IFCR_CTEIF1 = cast(uint)0x00000008; /* Channel 1 Transfer Error clear */
enum DMA_IFCR_CGIF2 = cast(uint)0x00000010; /* Channel 2 Global interrupt clear */
enum DMA_IFCR_CTCIF2 = cast(uint)0x00000020; /* Channel 2 Transfer Complete clear */
enum DMA_IFCR_CHTIF2 = cast(uint)0x00000040; /* Channel 2 Half Transfer clear */
enum DMA_IFCR_CTEIF2 = cast(uint)0x00000080; /* Channel 2 Transfer Error clear */
enum DMA_IFCR_CGIF3 = cast(uint)0x00000100; /* Channel 3 Global interrupt clear */
enum DMA_IFCR_CTCIF3 = cast(uint)0x00000200; /* Channel 3 Transfer Complete clear */
enum DMA_IFCR_CHTIF3 = cast(uint)0x00000400; /* Channel 3 Half Transfer clear */
enum DMA_IFCR_CTEIF3 = cast(uint)0x00000800; /* Channel 3 Transfer Error clear */
enum DMA_IFCR_CGIF4 = cast(uint)0x00001000; /* Channel 4 Global interrupt clear */
enum DMA_IFCR_CTCIF4 = cast(uint)0x00002000; /* Channel 4 Transfer Complete clear */
enum DMA_IFCR_CHTIF4 = cast(uint)0x00004000; /* Channel 4 Half Transfer clear */
enum DMA_IFCR_CTEIF4 = cast(uint)0x00008000; /* Channel 4 Transfer Error clear */
enum DMA_IFCR_CGIF5 = cast(uint)0x00010000; /* Channel 5 Global interrupt clear */
enum DMA_IFCR_CTCIF5 = cast(uint)0x00020000; /* Channel 5 Transfer Complete clear */
enum DMA_IFCR_CHTIF5 = cast(uint)0x00040000; /* Channel 5 Half Transfer clear */
enum DMA_IFCR_CTEIF5 = cast(uint)0x00080000; /* Channel 5 Transfer Error clear */
enum DMA_IFCR_CGIF6 = cast(uint)0x00100000; /* Channel 6 Global interrupt clear */
enum DMA_IFCR_CTCIF6 = cast(uint)0x00200000; /* Channel 6 Transfer Complete clear */
enum DMA_IFCR_CHTIF6 = cast(uint)0x00400000; /* Channel 6 Half Transfer clear */
enum DMA_IFCR_CTEIF6 = cast(uint)0x00800000; /* Channel 6 Transfer Error clear */
enum DMA_IFCR_CGIF7 = cast(uint)0x01000000; /* Channel 7 Global interrupt clear */
enum DMA_IFCR_CTCIF7 = cast(uint)0x02000000; /* Channel 7 Transfer Complete clear */
enum DMA_IFCR_CHTIF7 = cast(uint)0x04000000; /* Channel 7 Half Transfer clear */
enum DMA_IFCR_CTEIF7 = cast(uint)0x08000000; /* Channel 7 Transfer Error clear */

/* Bit definition for DMA_CCR1 register */
enum DMA_CCR1_EN = cast(ushort)0x0001; /* Channel enable*/
enum DMA_CCR1_TCIE = cast(ushort)0x0002; /* Transfer complete interrupt enable */
enum DMA_CCR1_HTIE = cast(ushort)0x0004; /* Half Transfer interrupt enable */
enum DMA_CCR1_TEIE = cast(ushort)0x0008; /* Transfer error interrupt enable */
enum DMA_CCR1_DIR = cast(ushort)0x0010; /* Data transfer direction */
enum DMA_CCR1_CIRC = cast(ushort)0x0020; /* Circular mode */
enum DMA_CCR1_PINC = cast(ushort)0x0040; /* Peripheral increment mode */
enum DMA_CCR1_MINC = cast(ushort)0x0080; /* Memory increment mode */

enum DMA_CCR1_PSIZE = cast(ushort)0x0300; /* PSIZE[1:0] bits (Peripheral size; */
enum DMA_CCR1_PSIZE_0 = cast(ushort)0x0100; /* Bit 0 */
enum DMA_CCR1_PSIZE_1 = cast(ushort)0x0200; /* Bit 1 */

enum DMA_CCR1_MSIZE = cast(ushort)0x0C00; /* MSIZE[1:0] bits (Memory size; */
enum DMA_CCR1_MSIZE_0 = cast(ushort)0x0400; /* Bit 0 */
enum DMA_CCR1_MSIZE_1 = cast(ushort)0x0800; /* Bit 1 */

enum DMA_CCR1_PL = cast(ushort)0x3000; /* PL[1:0] bits(Channel Priority level; */
enum DMA_CCR1_PL_0 = cast(ushort)0x1000; /* Bit 0 */
enum DMA_CCR1_PL_1 = cast(ushort)0x2000; /* Bit 1 */

enum DMA_CCR1_MEM2MEM = cast(ushort)0x4000; /* Memory to memory mode */

/* Bit definition for DMA_CCR2 register */
enum DMA_CCR2_EN = cast(ushort)0x0001; /* Channel enable */
enum DMA_CCR2_TCIE = cast(ushort)0x0002; /* Transfer complete interrupt enable */
enum DMA_CCR2_HTIE = cast(ushort)0x0004; /* Half Transfer interrupt enable */
enum DMA_CCR2_TEIE = cast(ushort)0x0008; /* Transfer error interrupt enable */
enum DMA_CCR2_DIR = cast(ushort)0x0010; /* Data transfer direction */
enum DMA_CCR2_CIRC = cast(ushort)0x0020; /* Circular mode */
enum DMA_CCR2_PINC = cast(ushort)0x0040; /* Peripheral increment mode */
enum DMA_CCR2_MINC = cast(ushort)0x0080; /* Memory increment mode */

enum DMA_CCR2_PSIZE = cast(ushort)0x0300; /* PSIZE[1:0] bits (Peripheral size; */
enum DMA_CCR2_PSIZE_0 = cast(ushort)0x0100; /* Bit 0 */
enum DMA_CCR2_PSIZE_1 = cast(ushort)0x0200; /* Bit 1 */

enum DMA_CCR2_MSIZE = cast(ushort)0x0C00; /* MSIZE[1:0] bits (Memory size; */
enum DMA_CCR2_MSIZE_0 = cast(ushort)0x0400; /* Bit 0 */
enum DMA_CCR2_MSIZE_1 = cast(ushort)0x0800; /* Bit 1 */

enum DMA_CCR2_PL = cast(ushort)0x3000; /* PL[1:0] bits (Channel Priority level; */
enum DMA_CCR2_PL_0 = cast(ushort)0x1000; /* Bit 0 */
enum DMA_CCR2_PL_1 = cast(ushort)0x2000; /* Bit 1 */

enum DMA_CCR2_MEM2MEM = cast(ushort)0x4000; /* Memory to memory mode */

/* Bit definition for DMA_CCR3 register */
enum DMA_CCR3_EN = cast(ushort)0x0001; /* Channel enable */
enum DMA_CCR3_TCIE = cast(ushort)0x0002; /* Transfer complete interrupt enable */
enum DMA_CCR3_HTIE = cast(ushort)0x0004; /* Half Transfer interrupt enable */
enum DMA_CCR3_TEIE = cast(ushort)0x0008; /* Transfer error interrupt enable */
enum DMA_CCR3_DIR = cast(ushort)0x0010; /* Data transfer direction */
enum DMA_CCR3_CIRC = cast(ushort)0x0020; /* Circular mode */
enum DMA_CCR3_PINC = cast(ushort)0x0040; /* Peripheral increment mode */
enum DMA_CCR3_MINC = cast(ushort)0x0080; /* Memory increment mode */

enum DMA_CCR3_PSIZE = cast(ushort)0x0300; /* PSIZE[1:0] bits (Peripheral size; */
enum DMA_CCR3_PSIZE_0 = cast(ushort)0x0100; /* Bit 0 */
enum DMA_CCR3_PSIZE_1 = cast(ushort)0x0200; /* Bit 1 */

enum DMA_CCR3_MSIZE = cast(ushort)0x0C00; /* MSIZE[1:0] bits (Memory size; */
enum DMA_CCR3_MSIZE_0 = cast(ushort)0x0400; /* Bit 0 */
enum DMA_CCR3_MSIZE_1 = cast(ushort)0x0800; /* Bit 1 */

enum DMA_CCR3_PL = cast(ushort)0x3000; /* PL[1:0] bits (Channel Priority level; */
enum DMA_CCR3_PL_0 = cast(ushort)0x1000; /* Bit 0 */
enum DMA_CCR3_PL_1 = cast(ushort)0x2000; /* Bit 1 */

enum DMA_CCR3_MEM2MEM = cast(ushort)0x4000; /* Memory to memory mode */

/* Bit definition for DMA_CCR4 register */
enum DMA_CCR4_EN = cast(ushort)0x0001; /* Channel enable */
enum DMA_CCR4_TCIE = cast(ushort)0x0002; /* Transfer complete interrupt enable */
enum DMA_CCR4_HTIE = cast(ushort)0x0004; /* Half Transfer interrupt enable */
enum DMA_CCR4_TEIE = cast(ushort)0x0008; /* Transfer error interrupt enable */
enum DMA_CCR4_DIR = cast(ushort)0x0010; /* Data transfer direction */
enum DMA_CCR4_CIRC = cast(ushort)0x0020; /* Circular mode */
enum DMA_CCR4_PINC = cast(ushort)0x0040; /* Peripheral increment mode */
enum DMA_CCR4_MINC = cast(ushort)0x0080; /* Memory increment mode */

enum DMA_CCR4_PSIZE = cast(ushort)0x0300; /* PSIZE[1:0] bits (Peripheral size; */
enum DMA_CCR4_PSIZE_0 = cast(ushort)0x0100; /* Bit 0 */
enum DMA_CCR4_PSIZE_1 = cast(ushort)0x0200; /* Bit 1 */

enum DMA_CCR4_MSIZE = cast(ushort)0x0C00; /* MSIZE[1:0] bits (Memory size; */
enum DMA_CCR4_MSIZE_0 = cast(ushort)0x0400; /* Bit 0 */
enum DMA_CCR4_MSIZE_1 = cast(ushort)0x0800; /* Bit 1 */

enum DMA_CCR4_PL = cast(ushort)0x3000; /* PL[1:0] bits (Channel Priority level; */
enum DMA_CCR4_PL_0 = cast(ushort)0x1000; /* Bit 0 */
enum DMA_CCR4_PL_1 = cast(ushort)0x2000; /* Bit 1 */

enum DMA_CCR4_MEM2MEM = cast(ushort)0x4000; /* Memory to memory mode */

/* Bit definition for DMA_CCR5 register */
enum DMA_CCR5_EN = cast(ushort)0x0001; /* Channel enable */
enum DMA_CCR5_TCIE = cast(ushort)0x0002; /* Transfer complete interrupt enable */
enum DMA_CCR5_HTIE = cast(ushort)0x0004; /* Half Transfer interrupt enable */
enum DMA_CCR5_TEIE = cast(ushort)0x0008; /* Transfer error interrupt enable */
enum DMA_CCR5_DIR = cast(ushort)0x0010; /* Data transfer direction */
enum DMA_CCR5_CIRC = cast(ushort)0x0020; /* Circular mode */
enum DMA_CCR5_PINC = cast(ushort)0x0040; /* Peripheral increment mode */
enum DMA_CCR5_MINC = cast(ushort)0x0080; /* Memory increment mode */

enum DMA_CCR5_PSIZE = cast(ushort)0x0300; /* PSIZE[1:0] bits (Peripheral size; */
enum DMA_CCR5_PSIZE_0 = cast(ushort)0x0100; /* Bit 0 */
enum DMA_CCR5_PSIZE_1 = cast(ushort)0x0200; /* Bit 1 */

enum DMA_CCR5_MSIZE = cast(ushort)0x0C00; /* MSIZE[1:0] bits (Memory size; */
enum DMA_CCR5_MSIZE_0 = cast(ushort)0x0400; /* Bit 0 */
enum DMA_CCR5_MSIZE_1 = cast(ushort)0x0800; /* Bit 1 */

enum DMA_CCR5_PL = cast(ushort)0x3000; /* PL[1:0] bits (Channel Priority level; */
enum DMA_CCR5_PL_0 = cast(ushort)0x1000; /* Bit 0 */
enum DMA_CCR5_PL_1 = cast(ushort)0x2000; /* Bit 1 */

enum DMA_CCR5_MEM2MEM = cast(ushort)0x4000; /* Memory to memory mode enable */

/* Bit definition for DMA_CCR6 register */
enum DMA_CCR6_EN = cast(ushort)0x0001; /* Channel enable */
enum DMA_CCR6_TCIE = cast(ushort)0x0002; /* Transfer complete interrupt enable */
enum DMA_CCR6_HTIE = cast(ushort)0x0004; /* Half Transfer interrupt enable */
enum DMA_CCR6_TEIE = cast(ushort)0x0008; /* Transfer error interrupt enable */
enum DMA_CCR6_DIR = cast(ushort)0x0010; /* Data transfer direction */
enum DMA_CCR6_CIRC = cast(ushort)0x0020; /* Circular mode */
enum DMA_CCR6_PINC = cast(ushort)0x0040; /* Peripheral increment mode */
enum DMA_CCR6_MINC = cast(ushort)0x0080; /* Memory increment mode */

enum DMA_CCR6_PSIZE = cast(ushort)0x0300; /* PSIZE[1:0] bits (Peripheral size; */
enum DMA_CCR6_PSIZE_0 = cast(ushort)0x0100; /* Bit 0 */
enum DMA_CCR6_PSIZE_1 = cast(ushort)0x0200; /* Bit 1 */

enum DMA_CCR6_MSIZE = cast(ushort)0x0C00; /* MSIZE[1:0] bits (Memory size; */
enum DMA_CCR6_MSIZE_0 = cast(ushort)0x0400; /* Bit 0 */
enum DMA_CCR6_MSIZE_1 = cast(ushort)0x0800; /* Bit 1 */

enum DMA_CCR6_PL = cast(ushort)0x3000; /* PL[1:0] bits (Channel Priority level; */
enum DMA_CCR6_PL_0 = cast(ushort)0x1000; /* Bit 0 */
enum DMA_CCR6_PL_1 = cast(ushort)0x2000; /* Bit 1 */

enum DMA_CCR6_MEM2MEM = cast(ushort)0x4000; /* Memory to memory mode */

/* Bit definition for DMA_CCR7 register */
enum DMA_CCR7_EN = cast(ushort)0x0001; /* Channel enable */
enum DMA_CCR7_TCIE = cast(ushort)0x0002; /* Transfer complete interrupt enable */
enum DMA_CCR7_HTIE = cast(ushort)0x0004; /* Half Transfer interrupt enable */
enum DMA_CCR7_TEIE = cast(ushort)0x0008; /* Transfer error interrupt enable */
enum DMA_CCR7_DIR = cast(ushort)0x0010; /* Data transfer direction */
enum DMA_CCR7_CIRC = cast(ushort)0x0020; /* Circular mode */
enum DMA_CCR7_PINC = cast(ushort)0x0040; /* Peripheral increment mode */
enum DMA_CCR7_MINC = cast(ushort)0x0080; /* Memory increment mode */

enum DMA_CCR7_PSIZE = cast(ushort)0x0300; /* PSIZE[1:0] bits (Peripheral size; */
enum DMA_CCR7_PSIZE_0 = cast(ushort)0x0100; /* Bit 0 */
enum DMA_CCR7_PSIZE_1 = cast(ushort)0x0200; /* Bit 1 */

enum DMA_CCR7_MSIZE = cast(ushort)0x0C00; /* MSIZE[1:0] bits (Memory size; */
enum DMA_CCR7_MSIZE_0 = cast(ushort)0x0400; /* Bit 0 */
enum DMA_CCR7_MSIZE_1 = cast(ushort)0x0800; /* Bit 1 */

enum DMA_CCR7_PL = cast(ushort)0x3000; /* PL[1:0] bits (Channel Priority level; */
enum DMA_CCR7_PL_0 = cast(ushort)0x1000; /* Bit 0 */
enum DMA_CCR7_PL_1 = cast(ushort)0x2000; /* Bit 1 */

enum DMA_CCR7_MEM2MEM = cast(ushort)0x4000; /* Memory to memory mode enable */

/* Bit definition for DMA_CNDTR1 register */
enum DMA_CNDTR1_NDT = cast(ushort)0xFFFF; /* Number of data to Transfer */

/* Bit definition for DMA_CNDTR2 register */
enum DMA_CNDTR2_NDT = cast(ushort)0xFFFF; /* Number of data to Transfer */

/* Bit definition for DMA_CNDTR3 register */
enum DMA_CNDTR3_NDT = cast(ushort)0xFFFF; /* Number of data to Transfer */

/* Bit definition for DMA_CNDTR4 register */
enum DMA_CNDTR4_NDT = cast(ushort)0xFFFF; /* Number of data to Transfer */

/* Bit definition for DMA_CNDTR5 register */
enum DMA_CNDTR5_NDT = cast(ushort)0xFFFF; /* Number of data to Transfer */

/* Bit definition for DMA_CNDTR6 register */
enum DMA_CNDTR6_NDT = cast(ushort)0xFFFF; /* Number of data to Transfer */

/* Bit definition for DMA_CNDTR7 register */
enum DMA_CNDTR7_NDT = cast(ushort)0xFFFF; /* Number of data to Transfer */

/* Bit definition for DMA_CPAR1 register */
enum DMA_CPAR1_PA = cast(uint)0xFFFFFFFF; /* Peripheral Address */

/* Bit definition for DMA_CPAR2 register */
enum DMA_CPAR2_PA = cast(uint)0xFFFFFFFF; /* Peripheral Address */

/* Bit definition for DMA_CPAR3 register */
enum DMA_CPAR3_PA = cast(uint)0xFFFFFFFF; /* Peripheral Address */

/* Bit definition for DMA_CPAR4 register */
enum DMA_CPAR4_PA = cast(uint)0xFFFFFFFF; /* Peripheral Address */

/* Bit definition for DMA_CPAR5 register */
enum DMA_CPAR5_PA = cast(uint)0xFFFFFFFF; /* Peripheral Address */

/* Bit definition for DMA_CPAR6 register */
enum DMA_CPAR6_PA = cast(uint)0xFFFFFFFF; /* Peripheral Address */

/* Bit definition for DMA_CPAR7 register */
enum DMA_CPAR7_PA = cast(uint)0xFFFFFFFF; /* Peripheral Address */

/* Bit definition for DMA_CMAR1 register */
enum DMA_CMAR1_MA = cast(uint)0xFFFFFFFF; /* Memory Address */

/* Bit definition for DMA_CMAR2 register */
enum DMA_CMAR2_MA = cast(uint)0xFFFFFFFF; /* Memory Address */

/* Bit definition for DMA_CMAR3 register */
enum DMA_CMAR3_MA = cast(uint)0xFFFFFFFF; /* Memory Address */

/* Bit definition for DMA_CMAR4 register */
enum DMA_CMAR4_MA = cast(uint)0xFFFFFFFF; /* Memory Address */

/* Bit definition for DMA_CMAR5 register */
enum DMA_CMAR5_MA = cast(uint)0xFFFFFFFF; /* Memory Address */

/* Bit definition for DMA_CMAR6 register */
enum DMA_CMAR6_MA = cast(uint)0xFFFFFFFF; /* Memory Address */

/* Bit definition for DMA_CMAR7 register */
enum DMA_CMAR7_MA = cast(uint)0xFFFFFFFF; /* Memory Address */

/* Bit definition for ADC_SR register */
enum ADC_SR_AWD = cast(ubyte)0x01; /* Analog watchdog flag */
enum ADC_SR_EOC = cast(ubyte)0x02; /* End of conversion */
enum ADC_SR_JEOC = cast(ubyte)0x04; /* Injected channel end of conversion */
enum ADC_SR_JSTRT = cast(ubyte)0x08; /* Injected channel Start flag */
enum ADC_SR_STRT = cast(ubyte)0x10; /* Regular channel Start flag */

/* Bit definition for ADC_CR1 register */
enum ADC_CR1_AWDCH = cast(uint)0x0000001F; /* AWDCH[4:0] bits (Analog watchdog channel select bits; */
enum ADC_CR1_AWDCH_0 = cast(uint)0x00000001; /* Bit 0 */
enum ADC_CR1_AWDCH_1 = cast(uint)0x00000002; /* Bit 1 */
enum ADC_CR1_AWDCH_2 = cast(uint)0x00000004; /* Bit 2 */
enum ADC_CR1_AWDCH_3 = cast(uint)0x00000008; /* Bit 3 */
enum ADC_CR1_AWDCH_4 = cast(uint)0x00000010; /* Bit 4 */

enum ADC_CR1_EOCIE = cast(uint)0x00000020; /* Interrupt enable for EOC */
enum ADC_CR1_AWDIE = cast(uint)0x00000040; /* Analog Watchdog interrupt enable */
enum ADC_CR1_JEOCIE = cast(uint)0x00000080; /* Interrupt enable for injected channels */
enum ADC_CR1_SCAN = cast(uint)0x00000100; /* Scan mode */
enum ADC_CR1_AWDSGL = cast(uint)0x00000200; /* Enable the watchdog on a single channel in scan mode */
enum ADC_CR1_JAUTO = cast(uint)0x00000400; /* Automatic injected group conversion */
enum ADC_CR1_DISCEN = cast(uint)0x00000800; /* Discontinuous mode on regular channels */
enum ADC_CR1_JDISCEN = cast(uint)0x00001000; /* Discontinuous mode on injected channels */

enum ADC_CR1_DISCNUM = cast(uint)0x0000E000; /* DISCNUM[2:0] bits (Discontinuous mode channel count; */
enum ADC_CR1_DISCNUM_0 = cast(uint)0x00002000; /* Bit 0 */
enum ADC_CR1_DISCNUM_1 = cast(uint)0x00004000; /* Bit 1 */
enum ADC_CR1_DISCNUM_2 = cast(uint)0x00008000; /* Bit 2 */

enum ADC_CR1_DUALMOD = cast(uint)0x000F0000; /* DUALMOD[3:0] bits (Dual mode selection; */
enum ADC_CR1_DUALMOD_0 = cast(uint)0x00010000; /* Bit 0 */
enum ADC_CR1_DUALMOD_1 = cast(uint)0x00020000; /* Bit 1 */
enum ADC_CR1_DUALMOD_2 = cast(uint)0x00040000; /* Bit 2 */
enum ADC_CR1_DUALMOD_3 = cast(uint)0x00080000; /* Bit 3 */

enum ADC_CR1_JAWDEN = cast(uint)0x00400000; /* Analog watchdog enable on injected channels */
enum ADC_CR1_AWDEN = cast(uint)0x00800000; /* Analog watchdog enable on regular channels */

/* Bit definition for ADC_CR2 register */
enum ADC_CR2_ADON = cast(uint)0x00000001; /* A/D Converter ON / OFF */
enum ADC_CR2_CONT = cast(uint)0x00000002; /* Continuous Conversion */
enum ADC_CR2_CAL = cast(uint)0x00000004; /* A/D Calibration */
enum ADC_CR2_RSTCAL = cast(uint)0x00000008; /* Reset Calibration */
enum ADC_CR2_DMA = cast(uint)0x00000100; /* Direct Memory access mode */
enum ADC_CR2_ALIGN = cast(uint)0x00000800; /* Data Alignment */

enum ADC_CR2_JEXTSEL = cast(uint)0x00007000; /* JEXTSEL[2:0] bits (External event select for injected group; */
enum ADC_CR2_JEXTSEL_0 = cast(uint)0x00001000; /* Bit 0 */
enum ADC_CR2_JEXTSEL_1 = cast(uint)0x00002000; /* Bit 1 */
enum ADC_CR2_JEXTSEL_2 = cast(uint)0x00004000; /* Bit 2 */

enum ADC_CR2_JEXTTRIG = cast(uint)0x00008000; /* External Trigger Conversion mode for injected channels */

enum ADC_CR2_EXTSEL = cast(uint)0x000E0000; /* EXTSEL[2:0] bits (External Event Select for regular group; */
enum ADC_CR2_EXTSEL_0 = cast(uint)0x00020000; /* Bit 0 */
enum ADC_CR2_EXTSEL_1 = cast(uint)0x00040000; /* Bit 1 */
enum ADC_CR2_EXTSEL_2 = cast(uint)0x00080000; /* Bit 2 */

enum ADC_CR2_EXTTRIG = cast(uint)0x00100000; /* External Trigger Conversion mode for regular channels */
enum ADC_CR2_JSWSTART = cast(uint)0x00200000; /* Start Conversion of injected channels */
enum ADC_CR2_SWSTART = cast(uint)0x00400000; /* Start Conversion of regular channels */
enum ADC_CR2_TSVREFE = cast(uint)0x00800000; /* Temperature Sensor and VREFINT Enable */

/* Bit definition for ADC_SMPR1 register */
enum ADC_SMPR1_SMP10 = cast(uint)0x00000007; /* SMP10[2:0] bits (Channel 10 Sample time selection; */
enum ADC_SMPR1_SMP10_0 = cast(uint)0x00000001; /* Bit 0 */
enum ADC_SMPR1_SMP10_1 = cast(uint)0x00000002; /* Bit 1 */
enum ADC_SMPR1_SMP10_2 = cast(uint)0x00000004; /* Bit 2 */

enum ADC_SMPR1_SMP11 = cast(uint)0x00000038; /* SMP11[2:0] bits (Channel 11 Sample time selection; */
enum ADC_SMPR1_SMP11_0 = cast(uint)0x00000008; /* Bit 0 */
enum ADC_SMPR1_SMP11_1 = cast(uint)0x00000010; /* Bit 1 */
enum ADC_SMPR1_SMP11_2 = cast(uint)0x00000020; /* Bit 2 */

enum ADC_SMPR1_SMP12 = cast(uint)0x000001C0; /* SMP12[2:0] bits (Channel 12 Sample time selection; */
enum ADC_SMPR1_SMP12_0 = cast(uint)0x00000040; /* Bit 0 */
enum ADC_SMPR1_SMP12_1 = cast(uint)0x00000080; /* Bit 1 */
enum ADC_SMPR1_SMP12_2 = cast(uint)0x00000100; /* Bit 2 */

enum ADC_SMPR1_SMP13 = cast(uint)0x00000E00; /* SMP13[2:0] bits (Channel 13 Sample time selection; */
enum ADC_SMPR1_SMP13_0 = cast(uint)0x00000200; /* Bit 0 */
enum ADC_SMPR1_SMP13_1 = cast(uint)0x00000400; /* Bit 1 */
enum ADC_SMPR1_SMP13_2 = cast(uint)0x00000800; /* Bit 2 */

enum ADC_SMPR1_SMP14 = cast(uint)0x00007000; /* SMP14[2:0] bits (Channel 14 Sample time selection; */
enum ADC_SMPR1_SMP14_0 = cast(uint)0x00001000; /* Bit 0 */
enum ADC_SMPR1_SMP14_1 = cast(uint)0x00002000; /* Bit 1 */
enum ADC_SMPR1_SMP14_2 = cast(uint)0x00004000; /* Bit 2 */

enum ADC_SMPR1_SMP15 = cast(uint)0x00038000; /* SMP15[2:0] bits (Channel 15 Sample time selection; */
enum ADC_SMPR1_SMP15_0 = cast(uint)0x00008000; /* Bit 0 */
enum ADC_SMPR1_SMP15_1 = cast(uint)0x00010000; /* Bit 1 */
enum ADC_SMPR1_SMP15_2 = cast(uint)0x00020000; /* Bit 2 */

enum ADC_SMPR1_SMP16 = cast(uint)0x001C0000; /* SMP16[2:0] bits (Channel 16 Sample time selection; */
enum ADC_SMPR1_SMP16_0 = cast(uint)0x00040000; /* Bit 0 */
enum ADC_SMPR1_SMP16_1 = cast(uint)0x00080000; /* Bit 1 */
enum ADC_SMPR1_SMP16_2 = cast(uint)0x00100000; /* Bit 2 */

enum ADC_SMPR1_SMP17 = cast(uint)0x00E00000; /* SMP17[2:0] bits (Channel 17 Sample time selection; */
enum ADC_SMPR1_SMP17_0 = cast(uint)0x00200000; /* Bit 0 */
enum ADC_SMPR1_SMP17_1 = cast(uint)0x00400000; /* Bit 1 */
enum ADC_SMPR1_SMP17_2 = cast(uint)0x00800000; /* Bit 2 */

/* Bit definition for ADC_SMPR2 register */
enum ADC_SMPR2_SMP0 = cast(uint)0x00000007; /* SMP0[2:0] bits (Channel 0 Sample time selection; */
enum ADC_SMPR2_SMP0_0 = cast(uint)0x00000001; /* Bit 0 */
enum ADC_SMPR2_SMP0_1 = cast(uint)0x00000002; /* Bit 1 */
enum ADC_SMPR2_SMP0_2 = cast(uint)0x00000004; /* Bit 2 */

enum ADC_SMPR2_SMP1 = cast(uint)0x00000038; /* SMP1[2:0] bits (Channel 1 Sample time selection; */
enum ADC_SMPR2_SMP1_0 = cast(uint)0x00000008; /* Bit 0 */
enum ADC_SMPR2_SMP1_1 = cast(uint)0x00000010; /* Bit 1 */
enum ADC_SMPR2_SMP1_2 = cast(uint)0x00000020; /* Bit 2 */

enum ADC_SMPR2_SMP2 = cast(uint)0x000001C0; /* SMP2[2:0] bits (Channel 2 Sample time selection; */
enum ADC_SMPR2_SMP2_0 = cast(uint)0x00000040; /* Bit 0 */
enum ADC_SMPR2_SMP2_1 = cast(uint)0x00000080; /* Bit 1 */
enum ADC_SMPR2_SMP2_2 = cast(uint)0x00000100; /* Bit 2 */

enum ADC_SMPR2_SMP3 = cast(uint)0x00000E00; /* SMP3[2:0] bits (Channel 3 Sample time selection; */
enum ADC_SMPR2_SMP3_0 = cast(uint)0x00000200; /* Bit 0 */
enum ADC_SMPR2_SMP3_1 = cast(uint)0x00000400; /* Bit 1 */
enum ADC_SMPR2_SMP3_2 = cast(uint)0x00000800; /* Bit 2 */

enum ADC_SMPR2_SMP4 = cast(uint)0x00007000; /* SMP4[2:0] bits (Channel 4 Sample time selection; */
enum ADC_SMPR2_SMP4_0 = cast(uint)0x00001000; /* Bit 0 */
enum ADC_SMPR2_SMP4_1 = cast(uint)0x00002000; /* Bit 1 */
enum ADC_SMPR2_SMP4_2 = cast(uint)0x00004000; /* Bit 2 */

enum ADC_SMPR2_SMP5 = cast(uint)0x00038000; /* SMP5[2:0] bits (Channel 5 Sample time selection; */
enum ADC_SMPR2_SMP5_0 = cast(uint)0x00008000; /* Bit 0 */
enum ADC_SMPR2_SMP5_1 = cast(uint)0x00010000; /* Bit 1 */
enum ADC_SMPR2_SMP5_2 = cast(uint)0x00020000; /* Bit 2 */

enum ADC_SMPR2_SMP6 = cast(uint)0x001C0000; /* SMP6[2:0] bits (Channel 6 Sample time selection; */
enum ADC_SMPR2_SMP6_0 = cast(uint)0x00040000; /* Bit 0 */
enum ADC_SMPR2_SMP6_1 = cast(uint)0x00080000; /* Bit 1 */
enum ADC_SMPR2_SMP6_2 = cast(uint)0x00100000; /* Bit 2 */

enum ADC_SMPR2_SMP7 = cast(uint)0x00E00000; /* SMP7[2:0] bits (Channel 7 Sample time selection; */
enum ADC_SMPR2_SMP7_0 = cast(uint)0x00200000; /* Bit 0 */
enum ADC_SMPR2_SMP7_1 = cast(uint)0x00400000; /* Bit 1 */
enum ADC_SMPR2_SMP7_2 = cast(uint)0x00800000; /* Bit 2 */

enum ADC_SMPR2_SMP8 = cast(uint)0x07000000; /* SMP8[2:0] bits (Channel 8 Sample time selection; */
enum ADC_SMPR2_SMP8_0 = cast(uint)0x01000000; /* Bit 0 */
enum ADC_SMPR2_SMP8_1 = cast(uint)0x02000000; /* Bit 1 */
enum ADC_SMPR2_SMP8_2 = cast(uint)0x04000000; /* Bit 2 */

enum ADC_SMPR2_SMP9 = cast(uint)0x38000000; /* SMP9[2:0] bits (Channel 9 Sample time selection; */
enum ADC_SMPR2_SMP9_0 = cast(uint)0x08000000; /* Bit 0 */
enum ADC_SMPR2_SMP9_1 = cast(uint)0x10000000; /* Bit 1 */
enum ADC_SMPR2_SMP9_2 = cast(uint)0x20000000; /* Bit 2 */

/* Bit definition for ADC_JOFR1 register */
enum ADC_JOFR1_JOFFSET1 = cast(ushort)0x0FFF; /* Data offset for injected channel 1 */

/* Bit definition for ADC_JOFR2 register */
enum ADC_JOFR2_JOFFSET2 = cast(ushort)0x0FFF; /* Data offset for injected channel 2 */

/* Bit definition for ADC_JOFR3 register */
enum ADC_JOFR3_JOFFSET3 = cast(ushort)0x0FFF; /* Data offset for injected channel 3 */

/* Bit definition for ADC_JOFR4 register */
enum ADC_JOFR4_JOFFSET4 = cast(ushort)0x0FFF; /* Data offset for injected channel 4 */

/* Bit definition for ADC_HTR register */
enum ADC_HTR_HT = cast(ushort)0x0FFF; /* Analog watchdog high threshold */

/* Bit definition for ADC_LTR register */
enum ADC_LTR_LT = cast(ushort)0x0FFF; /* Analog watchdog low threshold */

/* Bit definition for ADC_SQR1 register */
enum ADC_SQR1_SQ13 = cast(uint)0x0000001F; /* SQ13[4:0] bits (13th conversion in regular sequence; */
enum ADC_SQR1_SQ13_0 = cast(uint)0x00000001; /* Bit 0 */
enum ADC_SQR1_SQ13_1 = cast(uint)0x00000002; /* Bit 1 */
enum ADC_SQR1_SQ13_2 = cast(uint)0x00000004; /* Bit 2 */
enum ADC_SQR1_SQ13_3 = cast(uint)0x00000008; /* Bit 3 */
enum ADC_SQR1_SQ13_4 = cast(uint)0x00000010; /* Bit 4 */

enum ADC_SQR1_SQ14 = cast(uint)0x000003E0; /* SQ14[4:0] bits (14th conversion in regular sequence; */
enum ADC_SQR1_SQ14_0 = cast(uint)0x00000020; /* Bit 0 */
enum ADC_SQR1_SQ14_1 = cast(uint)0x00000040; /* Bit 1 */
enum ADC_SQR1_SQ14_2 = cast(uint)0x00000080; /* Bit 2 */
enum ADC_SQR1_SQ14_3 = cast(uint)0x00000100; /* Bit 3 */
enum ADC_SQR1_SQ14_4 = cast(uint)0x00000200; /* Bit 4 */

enum ADC_SQR1_SQ15 = cast(uint)0x00007C00; /* SQ15[4:0] bits (15th conversion in regular sequence; */
enum ADC_SQR1_SQ15_0 = cast(uint)0x00000400; /* Bit 0 */
enum ADC_SQR1_SQ15_1 = cast(uint)0x00000800; /* Bit 1 */
enum ADC_SQR1_SQ15_2 = cast(uint)0x00001000; /* Bit 2 */
enum ADC_SQR1_SQ15_3 = cast(uint)0x00002000; /* Bit 3 */
enum ADC_SQR1_SQ15_4 = cast(uint)0x00004000; /* Bit 4 */

enum ADC_SQR1_SQ16 = cast(uint)0x000F8000; /* SQ16[4:0] bits (16th conversion in regular sequence; */
enum ADC_SQR1_SQ16_0 = cast(uint)0x00008000; /* Bit 0 */
enum ADC_SQR1_SQ16_1 = cast(uint)0x00010000; /* Bit 1 */
enum ADC_SQR1_SQ16_2 = cast(uint)0x00020000; /* Bit 2 */
enum ADC_SQR1_SQ16_3 = cast(uint)0x00040000; /* Bit 3 */
enum ADC_SQR1_SQ16_4 = cast(uint)0x00080000; /* Bit 4 */

enum ADC_SQR1_L = cast(uint)0x00F00000; /* L[3:0] bits (Regular channel sequence length; */
enum ADC_SQR1_L_0 = cast(uint)0x00100000; /* Bit 0 */
enum ADC_SQR1_L_1 = cast(uint)0x00200000; /* Bit 1 */
enum ADC_SQR1_L_2 = cast(uint)0x00400000; /* Bit 2 */
enum ADC_SQR1_L_3 = cast(uint)0x00800000; /* Bit 3 */

/* Bit definition for ADC_SQR2 register */
enum ADC_SQR2_SQ7 = cast(uint)0x0000001F; /* SQ7[4:0] bits (7th conversion in regular sequence; */
enum ADC_SQR2_SQ7_0 = cast(uint)0x00000001; /* Bit 0 */
enum ADC_SQR2_SQ7_1 = cast(uint)0x00000002; /* Bit 1 */
enum ADC_SQR2_SQ7_2 = cast(uint)0x00000004; /* Bit 2 */
enum ADC_SQR2_SQ7_3 = cast(uint)0x00000008; /* Bit 3 */
enum ADC_SQR2_SQ7_4 = cast(uint)0x00000010; /* Bit 4 */

enum ADC_SQR2_SQ8 = cast(uint)0x000003E0; /* SQ8[4:0] bits (8th conversion in regular sequence; */
enum ADC_SQR2_SQ8_0 = cast(uint)0x00000020; /* Bit 0 */
enum ADC_SQR2_SQ8_1 = cast(uint)0x00000040; /* Bit 1 */
enum ADC_SQR2_SQ8_2 = cast(uint)0x00000080; /* Bit 2 */
enum ADC_SQR2_SQ8_3 = cast(uint)0x00000100; /* Bit 3 */
enum ADC_SQR2_SQ8_4 = cast(uint)0x00000200; /* Bit 4 */

enum ADC_SQR2_SQ9 = cast(uint)0x00007C00; /* SQ9[4:0] bits (9th conversion in regular sequence; */
enum ADC_SQR2_SQ9_0 = cast(uint)0x00000400; /* Bit 0 */
enum ADC_SQR2_SQ9_1 = cast(uint)0x00000800; /* Bit 1 */
enum ADC_SQR2_SQ9_2 = cast(uint)0x00001000; /* Bit 2 */
enum ADC_SQR2_SQ9_3 = cast(uint)0x00002000; /* Bit 3 */
enum ADC_SQR2_SQ9_4 = cast(uint)0x00004000; /* Bit 4 */

enum ADC_SQR2_SQ10 = cast(uint)0x000F8000; /* SQ10[4:0] bits (10th conversion in regular sequence; */
enum ADC_SQR2_SQ10_0 = cast(uint)0x00008000; /* Bit 0 */
enum ADC_SQR2_SQ10_1 = cast(uint)0x00010000; /* Bit 1 */
enum ADC_SQR2_SQ10_2 = cast(uint)0x00020000; /* Bit 2 */
enum ADC_SQR2_SQ10_3 = cast(uint)0x00040000; /* Bit 3 */
enum ADC_SQR2_SQ10_4 = cast(uint)0x00080000; /* Bit 4 */

enum ADC_SQR2_SQ11 = cast(uint)0x01F00000; /* SQ11[4:0] bits (11th conversion in regular sequence; */
enum ADC_SQR2_SQ11_0 = cast(uint)0x00100000; /* Bit 0 */
enum ADC_SQR2_SQ11_1 = cast(uint)0x00200000; /* Bit 1 */
enum ADC_SQR2_SQ11_2 = cast(uint)0x00400000; /* Bit 2 */
enum ADC_SQR2_SQ11_3 = cast(uint)0x00800000; /* Bit 3 */
enum ADC_SQR2_SQ11_4 = cast(uint)0x01000000; /* Bit 4 */

enum ADC_SQR2_SQ12 = cast(uint)0x3E000000; /* SQ12[4:0] bits (12th conversion in regular sequence; */
enum ADC_SQR2_SQ12_0 = cast(uint)0x02000000; /* Bit 0 */
enum ADC_SQR2_SQ12_1 = cast(uint)0x04000000; /* Bit 1 */
enum ADC_SQR2_SQ12_2 = cast(uint)0x08000000; /* Bit 2 */
enum ADC_SQR2_SQ12_3 = cast(uint)0x10000000; /* Bit 3 */
enum ADC_SQR2_SQ12_4 = cast(uint)0x20000000; /* Bit 4 */

/* Bit definition for ADC_SQR3 register */
enum ADC_SQR3_SQ1 = cast(uint)0x0000001F; /* SQ1[4:0] bits (1st conversion in regular sequence; */
enum ADC_SQR3_SQ1_0 = cast(uint)0x00000001; /* Bit 0 */
enum ADC_SQR3_SQ1_1 = cast(uint)0x00000002; /* Bit 1 */
enum ADC_SQR3_SQ1_2 = cast(uint)0x00000004; /* Bit 2 */
enum ADC_SQR3_SQ1_3 = cast(uint)0x00000008; /* Bit 3 */
enum ADC_SQR3_SQ1_4 = cast(uint)0x00000010; /* Bit 4 */

enum ADC_SQR3_SQ2 = cast(uint)0x000003E0; /* SQ2[4:0] bits (2nd conversion in regular sequence; */
enum ADC_SQR3_SQ2_0 = cast(uint)0x00000020; /* Bit 0 */
enum ADC_SQR3_SQ2_1 = cast(uint)0x00000040; /* Bit 1 */
enum ADC_SQR3_SQ2_2 = cast(uint)0x00000080; /* Bit 2 */
enum ADC_SQR3_SQ2_3 = cast(uint)0x00000100; /* Bit 3 */
enum ADC_SQR3_SQ2_4 = cast(uint)0x00000200; /* Bit 4 */

enum ADC_SQR3_SQ3 = cast(uint)0x00007C00; /* SQ3[4:0] bits (3rd conversion in regular sequence; */
enum ADC_SQR3_SQ3_0 = cast(uint)0x00000400; /* Bit 0 */
enum ADC_SQR3_SQ3_1 = cast(uint)0x00000800; /* Bit 1 */
enum ADC_SQR3_SQ3_2 = cast(uint)0x00001000; /* Bit 2 */
enum ADC_SQR3_SQ3_3 = cast(uint)0x00002000; /* Bit 3 */
enum ADC_SQR3_SQ3_4 = cast(uint)0x00004000; /* Bit 4 */

enum ADC_SQR3_SQ4 = cast(uint)0x000F8000; /* SQ4[4:0] bits (4th conversion in regular sequence; */
enum ADC_SQR3_SQ4_0 = cast(uint)0x00008000; /* Bit 0 */
enum ADC_SQR3_SQ4_1 = cast(uint)0x00010000; /* Bit 1 */
enum ADC_SQR3_SQ4_2 = cast(uint)0x00020000; /* Bit 2 */
enum ADC_SQR3_SQ4_3 = cast(uint)0x00040000; /* Bit 3 */
enum ADC_SQR3_SQ4_4 = cast(uint)0x00080000; /* Bit 4 */

enum ADC_SQR3_SQ5 = cast(uint)0x01F00000; /* SQ5[4:0] bits (5th conversion in regular sequence; */
enum ADC_SQR3_SQ5_0 = cast(uint)0x00100000; /* Bit 0 */
enum ADC_SQR3_SQ5_1 = cast(uint)0x00200000; /* Bit 1 */
enum ADC_SQR3_SQ5_2 = cast(uint)0x00400000; /* Bit 2 */
enum ADC_SQR3_SQ5_3 = cast(uint)0x00800000; /* Bit 3 */
enum ADC_SQR3_SQ5_4 = cast(uint)0x01000000; /* Bit 4 */

enum ADC_SQR3_SQ6 = cast(uint)0x3E000000; /* SQ6[4:0] bits (6th conversion in regular sequence; */
enum ADC_SQR3_SQ6_0 = cast(uint)0x02000000; /* Bit 0 */
enum ADC_SQR3_SQ6_1 = cast(uint)0x04000000; /* Bit 1 */
enum ADC_SQR3_SQ6_2 = cast(uint)0x08000000; /* Bit 2 */
enum ADC_SQR3_SQ6_3 = cast(uint)0x10000000; /* Bit 3 */
enum ADC_SQR3_SQ6_4 = cast(uint)0x20000000; /* Bit 4 */

/* Bit definition for ADC_JSQR register */
enum ADC_JSQR_JSQ1 = cast(uint)0x0000001F; /* JSQ1[4:0] bits (1st conversion in injected sequence; */
enum ADC_JSQR_JSQ1_0 = cast(uint)0x00000001; /* Bit 0 */
enum ADC_JSQR_JSQ1_1 = cast(uint)0x00000002; /* Bit 1 */
enum ADC_JSQR_JSQ1_2 = cast(uint)0x00000004; /* Bit 2 */
enum ADC_JSQR_JSQ1_3 = cast(uint)0x00000008; /* Bit 3 */
enum ADC_JSQR_JSQ1_4 = cast(uint)0x00000010; /* Bit 4 */

enum ADC_JSQR_JSQ2 = cast(uint)0x000003E0; /* JSQ2[4:0] bits (2nd conversion in injected sequence; */
enum ADC_JSQR_JSQ2_0 = cast(uint)0x00000020; /* Bit 0 */
enum ADC_JSQR_JSQ2_1 = cast(uint)0x00000040; /* Bit 1 */
enum ADC_JSQR_JSQ2_2 = cast(uint)0x00000080; /* Bit 2 */
enum ADC_JSQR_JSQ2_3 = cast(uint)0x00000100; /* Bit 3 */
enum ADC_JSQR_JSQ2_4 = cast(uint)0x00000200; /* Bit 4 */

enum ADC_JSQR_JSQ3 = cast(uint)0x00007C00; /* JSQ3[4:0] bits (3rd conversion in injected sequence; */
enum ADC_JSQR_JSQ3_0 = cast(uint)0x00000400; /* Bit 0 */
enum ADC_JSQR_JSQ3_1 = cast(uint)0x00000800; /* Bit 1 */
enum ADC_JSQR_JSQ3_2 = cast(uint)0x00001000; /* Bit 2 */
enum ADC_JSQR_JSQ3_3 = cast(uint)0x00002000; /* Bit 3 */
enum ADC_JSQR_JSQ3_4 = cast(uint)0x00004000; /* Bit 4 */

enum ADC_JSQR_JSQ4 = cast(uint)0x000F8000; /* JSQ4[4:0] bits (4th conversion in injected sequence; */
enum ADC_JSQR_JSQ4_0 = cast(uint)0x00008000; /* Bit 0 */
enum ADC_JSQR_JSQ4_1 = cast(uint)0x00010000; /* Bit 1 */
enum ADC_JSQR_JSQ4_2 = cast(uint)0x00020000; /* Bit 2 */
enum ADC_JSQR_JSQ4_3 = cast(uint)0x00040000; /* Bit 3 */
enum ADC_JSQR_JSQ4_4 = cast(uint)0x00080000; /* Bit 4 */

enum ADC_JSQR_JL = cast(uint)0x00300000; /* JL[1:0] bits (Injected Sequence length; */
enum ADC_JSQR_JL_0 = cast(uint)0x00100000; /* Bit 0 */
enum ADC_JSQR_JL_1 = cast(uint)0x00200000; /* Bit 1 */

/* Bit definition for ADC_JDR1 register */
enum ADC_JDR1_JDATA = cast(ushort)0xFFFF; /* Injected data */

/* Bit definition for ADC_JDR2 register */
enum ADC_JDR2_JDATA = cast(ushort)0xFFFF; /* Injected data */

/* Bit definition for ADC_JDR3 register */
enum ADC_JDR3_JDATA = cast(ushort)0xFFFF; /* Injected data */

/* Bit definition for ADC_JDR4 register */
enum ADC_JDR4_JDATA = cast(ushort)0xFFFF; /* Injected data */

/* Bit definition for ADC_DR register */
enum ADC_DR_DATA = cast(uint)0x0000FFFF; /* Regular data */
enum ADC_DR_ADC2DATA = cast(uint)0xFFFF0000; /* ADC2 data */

/* Bit definition for DAC_CR register */
enum DAC_CR_EN1 = cast(uint)0x00000001; /* DAC channel1 enable */
enum DAC_CR_BOFF1 = cast(uint)0x00000002; /* DAC channel1 output buffer disable */
enum DAC_CR_TEN1 = cast(uint)0x00000004; /* DAC channel1 Trigger enable */

enum DAC_CR_TSEL1 = cast(uint)0x00000038; /* TSEL1[2:0] (DAC channel1 Trigger selection; */
enum DAC_CR_TSEL1_0 = cast(uint)0x00000008; /* Bit 0 */
enum DAC_CR_TSEL1_1 = cast(uint)0x00000010; /* Bit 1 */
enum DAC_CR_TSEL1_2 = cast(uint)0x00000020; /* Bit 2 */

enum DAC_CR_WAVE1 = cast(uint)0x000000C0; /* WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable; */
enum DAC_CR_WAVE1_0 = cast(uint)0x00000040; /* Bit 0 */
enum DAC_CR_WAVE1_1 = cast(uint)0x00000080; /* Bit 1 */

enum DAC_CR_MAMP1 = cast(uint)0x00000F00; /* MAMP1[3:0] (DAC channel1 Mask/Amplitude selector; */
enum DAC_CR_MAMP1_0 = cast(uint)0x00000100; /* Bit 0 */
enum DAC_CR_MAMP1_1 = cast(uint)0x00000200; /* Bit 1 */
enum DAC_CR_MAMP1_2 = cast(uint)0x00000400; /* Bit 2 */
enum DAC_CR_MAMP1_3 = cast(uint)0x00000800; /* Bit 3 */

enum DAC_CR_DMAEN1 = cast(uint)0x00001000; /* DAC channel1 DMA enable */
enum DAC_CR_EN2 = cast(uint)0x00010000; /* DAC channel2 enable */
enum DAC_CR_BOFF2 = cast(uint)0x00020000; /* DAC channel2 output buffer disable */
enum DAC_CR_TEN2 = cast(uint)0x00040000; /* DAC channel2 Trigger enable */

enum DAC_CR_TSEL2 = cast(uint)0x00380000; /* TSEL2[2:0] (DAC channel2 Trigger selection; */
enum DAC_CR_TSEL2_0 = cast(uint)0x00080000; /* Bit 0 */
enum DAC_CR_TSEL2_1 = cast(uint)0x00100000; /* Bit 1 */
enum DAC_CR_TSEL2_2 = cast(uint)0x00200000; /* Bit 2 */

enum DAC_CR_WAVE2 = cast(uint)0x00C00000; /* WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable; */
enum DAC_CR_WAVE2_0 = cast(uint)0x00400000; /* Bit 0 */
enum DAC_CR_WAVE2_1 = cast(uint)0x00800000; /* Bit 1 */

enum DAC_CR_MAMP2 = cast(uint)0x0F000000; /* MAMP2[3:0] (DAC channel2 Mask/Amplitude selector; */
enum DAC_CR_MAMP2_0 = cast(uint)0x01000000; /* Bit 0 */
enum DAC_CR_MAMP2_1 = cast(uint)0x02000000; /* Bit 1 */
enum DAC_CR_MAMP2_2 = cast(uint)0x04000000; /* Bit 2 */
enum DAC_CR_MAMP2_3 = cast(uint)0x08000000; /* Bit 3 */

enum DAC_CR_DMAEN2 = cast(uint)0x10000000; /* DAC channel2 DMA enabled */

enum DAC_CR_DMAUDRIE1 = cast(uint)0x00002000; /* DAC channel1 DMA underrun interrupt enable */
enum DAC_CR_DMAUDRIE2 = cast(uint)0x20000000; /* DAC channel2 DMA underrun interrupt enable */

/* Bit definition for DAC_SWTRIGR register */
enum DAC_SWTRIGR_SWTRIG1 = cast(ubyte)0x01; /* DAC channel1 software trigger */
enum DAC_SWTRIGR_SWTRIG2 = cast(ubyte)0x02; /* DAC channel2 software trigger */

/* Bit definition for DAC_DHR12R1 register */
enum DAC_DHR12R1_DACC1DHR = cast(ushort)0x0FFF; /* DAC channel1 12-bit Right aligned data */

/* Bit definition for DAC_DHR12L1 register */
enum DAC_DHR12L1_DACC1DHR = cast(ushort)0xFFF0; /* DAC channel1 12-bit Left aligned data */

/* Bit definition for DAC_DHR8R1 register */
enum DAC_DHR8R1_DACC1DHR = cast(ubyte)0xFF; /* DAC channel1 8-bit Right aligned data */

/* Bit definition for DAC_DHR12R2 register */
enum DAC_DHR12R2_DACC2DHR = cast(ushort)0x0FFF; /* DAC channel2 12-bit Right aligned data */

/* Bit definition for DAC_DHR12L2 register */
enum DAC_DHR12L2_DACC2DHR = cast(ushort)0xFFF0; /* DAC channel2 12-bit Left aligned data */

/* Bit definition for DAC_DHR8R2 register */
enum DAC_DHR8R2_DACC2DHR = cast(ubyte)0xFF; /* DAC channel2 8-bit Right aligned data */

/* Bit definition for DAC_DHR12RD register */
enum DAC_DHR12RD_DACC1DHR = cast(uint)0x00000FFF; /* DAC channel1 12-bit Right aligned data */
enum DAC_DHR12RD_DACC2DHR = cast(uint)0x0FFF0000; /* DAC channel2 12-bit Right aligned data */

/* Bit definition for DAC_DHR12LD register */
enum DAC_DHR12LD_DACC1DHR = cast(uint)0x0000FFF0; /* DAC channel1 12-bit Left aligned data */
enum DAC_DHR12LD_DACC2DHR = cast(uint)0xFFF00000; /* DAC channel2 12-bit Left aligned data */

/* Bit definition for DAC_DHR8RD register */
enum DAC_DHR8RD_DACC1DHR = cast(ushort)0x00FF; /* DAC channel1 8-bit Right aligned data */
enum DAC_DHR8RD_DACC2DHR = cast(ushort)0xFF00; /* DAC channel2 8-bit Right aligned data */

/* Bit definition for DAC_DOR1 register */
enum DAC_DOR1_DACC1DOR = cast(ushort)0x0FFF; /* DAC channel1 data output */

/* Bit definition for DAC_DOR2 register */
enum DAC_DOR2_DACC2DOR = cast(ushort)0x0FFF; /* DAC channel2 data output */

/* Bit definition for DAC_SR register */
enum DAC_SR_DMAUDR1 = cast(uint)0x00002000; /* DAC channel1 DMA underrun flag */
enum DAC_SR_DMAUDR2 = cast(uint)0x20000000; /* DAC channel2 DMA underrun flag */

/* Bit definition for CEC_CFGR register */
enum CEC_CFGR_PE = cast(ushort)0x0001; /* Peripheral Enable */
enum CEC_CFGR_IE = cast(ushort)0x0002; /* Interrupt Enable */
enum CEC_CFGR_BTEM = cast(ushort)0x0004; /* Bit Timing Error Mode */
enum CEC_CFGR_BPEM = cast(ushort)0x0008; /* Bit Period Error Mode */

/* Bit definition for CEC_OAR register */
enum CEC_OAR_OA = cast(ushort)0x000F; /* OA[3:0]: Own Address */
enum CEC_OAR_OA_0 = cast(ushort)0x0001; /* Bit 0 */
enum CEC_OAR_OA_1 = cast(ushort)0x0002; /* Bit 1 */
enum CEC_OAR_OA_2 = cast(ushort)0x0004; /* Bit 2 */
enum CEC_OAR_OA_3 = cast(ushort)0x0008; /* Bit 3 */

/* Bit definition for CEC_PRES register */
enum CEC_PRES_PRES = cast(ushort)0x3FFF; /* Prescaler Counter Value */

/* Bit definition for CEC_ESR register */
enum CEC_ESR_BTE = cast(ushort)0x0001; /* Bit Timing Error */
enum CEC_ESR_BPE = cast(ushort)0x0002; /* Bit Period Error */
enum CEC_ESR_RBTFE = cast(ushort)0x0004; /* Rx Block Transfer Finished Error */
enum CEC_ESR_SBE = cast(ushort)0x0008; /* Start Bit Error */
enum CEC_ESR_ACKE = cast(ushort)0x0010; /* Block Acknowledge Error */
enum CEC_ESR_LINE = cast(ushort)0x0020; /* Line Error */
enum CEC_ESR_TBTFE = cast(ushort)0x0040; /* Tx Block Transfer Finished Error */

/* Bit definition for CEC_CSR register */
enum CEC_CSR_TSOM = cast(ushort)0x0001; /* Tx Start Of Message */
enum CEC_CSR_TEOM = cast(ushort)0x0002; /* Tx End Of Message */
enum CEC_CSR_TERR = cast(ushort)0x0004; /* Tx Error */
enum CEC_CSR_TBTRF = cast(ushort)0x0008; /* Tx Byte Transfer Request or Block Transfer Finished */
enum CEC_CSR_RSOM = cast(ushort)0x0010; /* Rx Start Of Message */
enum CEC_CSR_REOM = cast(ushort)0x0020; /* Rx End Of Message */
enum CEC_CSR_RERR = cast(ushort)0x0040; /* Rx Error */
enum CEC_CSR_RBTF = cast(ushort)0x0080; /* Rx Block Transfer Finished */

/* Bit definition for CEC_TXD register */
enum CEC_TXD_TXD = cast(ushort)0x00FF; /* Tx Data register */

/* Bit definition for CEC_RXD register */
enum CEC_RXD_RXD = cast(ushort)0x00FF; /* Rx Data register */

/* Bit definition for TIM_CR1 register */
enum TIM_CR1_CEN = cast(ushort)0x0001; /* Counter enable */
enum TIM_CR1_UDIS = cast(ushort)0x0002; /* Update disable */
enum TIM_CR1_URS = cast(ushort)0x0004; /* Update request source */
enum TIM_CR1_OPM = cast(ushort)0x0008; /* One pulse mode */
enum TIM_CR1_DIR = cast(ushort)0x0010; /* Direction */

enum TIM_CR1_CMS = cast(ushort)0x0060; /* CMS[1:0] bits (Center-aligned mode selection; */
enum TIM_CR1_CMS_0 = cast(ushort)0x0020; /* Bit 0 */
enum TIM_CR1_CMS_1 = cast(ushort)0x0040; /* Bit 1 */

enum TIM_CR1_ARPE = cast(ushort)0x0080; /* Auto-reload preload enable */

enum TIM_CR1_CKD = cast(ushort)0x0300; /* CKD[1:0] bits (clock division; */
enum TIM_CR1_CKD_0 = cast(ushort)0x0100; /* Bit 0 */
enum TIM_CR1_CKD_1 = cast(ushort)0x0200; /* Bit 1 */

/* Bit definition for TIM_CR2 register */
enum TIM_CR2_CCPC = cast(ushort)0x0001; /* Capture/Compare Preloaded Control */
enum TIM_CR2_CCUS = cast(ushort)0x0004; /* Capture/Compare Control Update Selection */
enum TIM_CR2_CCDS = cast(ushort)0x0008; /* Capture/Compare DMA Selection */

enum TIM_CR2_MMS = cast(ushort)0x0070; /* MMS[2:0] bits (Master Mode Selection; */
enum TIM_CR2_MMS_0 = cast(ushort)0x0010; /* Bit 0 */
enum TIM_CR2_MMS_1 = cast(ushort)0x0020; /* Bit 1 */
enum TIM_CR2_MMS_2 = cast(ushort)0x0040; /* Bit 2 */

enum TIM_CR2_TI1S = cast(ushort)0x0080; /* TI1 Selection */
enum TIM_CR2_OIS1 = cast(ushort)0x0100; /* Output Idle state 1 (OC1 output; */
enum TIM_CR2_OIS1N = cast(ushort)0x0200; /* Output Idle state 1 (OC1N output; */
enum TIM_CR2_OIS2 = cast(ushort)0x0400; /* Output Idle state 2 (OC2 output; */
enum TIM_CR2_OIS2N = cast(ushort)0x0800; /* Output Idle state 2 (OC2N output; */
enum TIM_CR2_OIS3 = cast(ushort)0x1000; /* Output Idle state 3 (OC3 output; */
enum TIM_CR2_OIS3N = cast(ushort)0x2000; /* Output Idle state 3 (OC3N output; */
enum TIM_CR2_OIS4 = cast(ushort)0x4000; /* Output Idle state 4 (OC4 output; */

/* Bit definition for TIM_SMCR register */
enum TIM_SMCR_SMS = cast(ushort)0x0007; /* SMS[2:0] bits (Slave mode selection; */
enum TIM_SMCR_SMS_0 = cast(ushort)0x0001; /* Bit 0 */
enum TIM_SMCR_SMS_1 = cast(ushort)0x0002; /* Bit 1 */
enum TIM_SMCR_SMS_2 = cast(ushort)0x0004; /* Bit 2 */

enum TIM_SMCR_TS = cast(ushort)0x0070; /* TS[2:0] bits (Trigger selection; */
enum TIM_SMCR_TS_0 = cast(ushort)0x0010; /* Bit 0 */
enum TIM_SMCR_TS_1 = cast(ushort)0x0020; /* Bit 1 */
enum TIM_SMCR_TS_2 = cast(ushort)0x0040; /* Bit 2 */

enum TIM_SMCR_MSM = cast(ushort)0x0080; /* Master/slave mode */

enum TIM_SMCR_ETF = cast(ushort)0x0F00; /* ETF[3:0] bits (External trigger filter; */
enum TIM_SMCR_ETF_0 = cast(ushort)0x0100; /* Bit 0 */
enum TIM_SMCR_ETF_1 = cast(ushort)0x0200; /* Bit 1 */
enum TIM_SMCR_ETF_2 = cast(ushort)0x0400; /* Bit 2 */
enum TIM_SMCR_ETF_3 = cast(ushort)0x0800; /* Bit 3 */

enum TIM_SMCR_ETPS = cast(ushort)0x3000; /* ETPS[1:0] bits (External trigger prescaler; */
enum TIM_SMCR_ETPS_0 = cast(ushort)0x1000; /* Bit 0 */
enum TIM_SMCR_ETPS_1 = cast(ushort)0x2000; /* Bit 1 */

enum TIM_SMCR_ECE = cast(ushort)0x4000; /* External clock enable */
enum TIM_SMCR_ETP = cast(ushort)0x8000; /* External trigger polarity */

/* Bit definition for TIM_DIER register */
enum TIM_DIER_UIE = cast(ushort)0x0001; /* Update interrupt enable */
enum TIM_DIER_CC1IE = cast(ushort)0x0002; /* Capture/Compare 1 interrupt enable */
enum TIM_DIER_CC2IE = cast(ushort)0x0004; /* Capture/Compare 2 interrupt enable */
enum TIM_DIER_CC3IE = cast(ushort)0x0008; /* Capture/Compare 3 interrupt enable */
enum TIM_DIER_CC4IE = cast(ushort)0x0010; /* Capture/Compare 4 interrupt enable */
enum TIM_DIER_COMIE = cast(ushort)0x0020; /* COM interrupt enable */
enum TIM_DIER_TIE = cast(ushort)0x0040; /* Trigger interrupt enable */
enum TIM_DIER_BIE = cast(ushort)0x0080; /* Break interrupt enable */
enum TIM_DIER_UDE = cast(ushort)0x0100; /* Update DMA request enable */
enum TIM_DIER_CC1DE = cast(ushort)0x0200; /* Capture/Compare 1 DMA request enable */
enum TIM_DIER_CC2DE = cast(ushort)0x0400; /* Capture/Compare 2 DMA request enable */
enum TIM_DIER_CC3DE = cast(ushort)0x0800; /* Capture/Compare 3 DMA request enable */
enum TIM_DIER_CC4DE = cast(ushort)0x1000; /* Capture/Compare 4 DMA request enable */
enum TIM_DIER_COMDE = cast(ushort)0x2000; /* COM DMA request enable */
enum TIM_DIER_TDE = cast(ushort)0x4000; /* Trigger DMA request enable */

/* Bit definition for TIM_SR register */
enum TIM_SR_UIF = cast(ushort)0x0001; /* Update interrupt Flag */
enum TIM_SR_CC1IF = cast(ushort)0x0002; /* Capture/Compare 1 interrupt Flag */
enum TIM_SR_CC2IF = cast(ushort)0x0004; /* Capture/Compare 2 interrupt Flag */
enum TIM_SR_CC3IF = cast(ushort)0x0008; /* Capture/Compare 3 interrupt Flag */
enum TIM_SR_CC4IF = cast(ushort)0x0010; /* Capture/Compare 4 interrupt Flag */
enum TIM_SR_COMIF = cast(ushort)0x0020; /* COM interrupt Flag */
enum TIM_SR_TIF = cast(ushort)0x0040; /* Trigger interrupt Flag */
enum TIM_SR_BIF = cast(ushort)0x0080; /* Break interrupt Flag */
enum TIM_SR_CC1OF = cast(ushort)0x0200; /* Capture/Compare 1 Overcapture Flag */
enum TIM_SR_CC2OF = cast(ushort)0x0400; /* Capture/Compare 2 Overcapture Flag */
enum TIM_SR_CC3OF = cast(ushort)0x0800; /* Capture/Compare 3 Overcapture Flag */
enum TIM_SR_CC4OF = cast(ushort)0x1000; /* Capture/Compare 4 Overcapture Flag */

/* Bit definition for TIM_EGR register */
enum TIM_EGR_UG = cast(ubyte)0x01; /* Update Generation */
enum TIM_EGR_CC1G = cast(ubyte)0x02; /* Capture/Compare 1 Generation */
enum TIM_EGR_CC2G = cast(ubyte)0x04; /* Capture/Compare 2 Generation */
enum TIM_EGR_CC3G = cast(ubyte)0x08; /* Capture/Compare 3 Generation */
enum TIM_EGR_CC4G = cast(ubyte)0x10; /* Capture/Compare 4 Generation */
enum TIM_EGR_COMG = cast(ubyte)0x20; /* Capture/Compare Control Update Generation */
enum TIM_EGR_TG = cast(ubyte)0x40; /* Trigger Generation */
enum TIM_EGR_BG = cast(ubyte)0x80; /* Break Generation */

/* Bit definition for TIM_CCMR1 register */
enum TIM_CCMR1_CC1S = cast(ushort)0x0003; /* CC1S[1:0] bits (Capture/Compare 1 Selection; */
enum TIM_CCMR1_CC1S_0 = cast(ushort)0x0001; /* Bit 0 */
enum TIM_CCMR1_CC1S_1 = cast(ushort)0x0002; /* Bit 1 */

enum TIM_CCMR1_OC1FE = cast(ushort)0x0004; /* Output Compare 1 Fast enable */
enum TIM_CCMR1_OC1PE = cast(ushort)0x0008; /* Output Compare 1 Preload enable */

enum TIM_CCMR1_OC1M = cast(ushort)0x0070; /* OC1M[2:0] bits (Output Compare 1 Mode; */
enum TIM_CCMR1_OC1M_0 = cast(ushort)0x0010; /* Bit 0 */
enum TIM_CCMR1_OC1M_1 = cast(ushort)0x0020; /* Bit 1 */
enum TIM_CCMR1_OC1M_2 = cast(ushort)0x0040; /* Bit 2 */

enum TIM_CCMR1_OC1CE = cast(ushort)0x0080; /* Output Compare 1Clear Enable */

enum TIM_CCMR1_CC2S = cast(ushort)0x0300; /* CC2S[1:0] bits (Capture/Compare 2 Selection; */
enum TIM_CCMR1_CC2S_0 = cast(ushort)0x0100; /* Bit 0 */
enum TIM_CCMR1_CC2S_1 = cast(ushort)0x0200; /* Bit 1 */

enum TIM_CCMR1_OC2FE = cast(ushort)0x0400; /* Output Compare 2 Fast enable */
enum TIM_CCMR1_OC2PE = cast(ushort)0x0800; /* Output Compare 2 Preload enable */

enum TIM_CCMR1_OC2M = cast(ushort)0x7000; /* OC2M[2:0] bits (Output Compare 2 Mode; */
enum TIM_CCMR1_OC2M_0 = cast(ushort)0x1000; /* Bit 0 */
enum TIM_CCMR1_OC2M_1 = cast(ushort)0x2000; /* Bit 1 */
enum TIM_CCMR1_OC2M_2 = cast(ushort)0x4000; /* Bit 2 */

enum TIM_CCMR1_OC2CE = cast(ushort)0x8000; /* Output Compare 2 Clear Enable */

enum TIM_CCMR1_IC1PSC = cast(ushort)0x000C; /* IC1PSC[1:0] bits (Input Capture 1 Prescaler; */
enum TIM_CCMR1_IC1PSC_0 = cast(ushort)0x0004; /* Bit 0 */
enum TIM_CCMR1_IC1PSC_1 = cast(ushort)0x0008; /* Bit 1 */

enum TIM_CCMR1_IC1F = cast(ushort)0x00F0; /* IC1F[3:0] bits (Input Capture 1 Filter; */
enum TIM_CCMR1_IC1F_0 = cast(ushort)0x0010; /* Bit 0 */
enum TIM_CCMR1_IC1F_1 = cast(ushort)0x0020; /* Bit 1 */
enum TIM_CCMR1_IC1F_2 = cast(ushort)0x0040; /* Bit 2 */
enum TIM_CCMR1_IC1F_3 = cast(ushort)0x0080; /* Bit 3 */

enum TIM_CCMR1_IC2PSC = cast(ushort)0x0C00; /* IC2PSC[1:0] bits (Input Capture 2 Prescaler; */
enum TIM_CCMR1_IC2PSC_0 = cast(ushort)0x0400; /* Bit 0 */
enum TIM_CCMR1_IC2PSC_1 = cast(ushort)0x0800; /* Bit 1 */

enum TIM_CCMR1_IC2F = cast(ushort)0xF000; /* IC2F[3:0] bits (Input Capture 2 Filter; */
enum TIM_CCMR1_IC2F_0 = cast(ushort)0x1000; /* Bit 0 */
enum TIM_CCMR1_IC2F_1 = cast(ushort)0x2000; /* Bit 1 */
enum TIM_CCMR1_IC2F_2 = cast(ushort)0x4000; /* Bit 2 */
enum TIM_CCMR1_IC2F_3 = cast(ushort)0x8000; /* Bit 3 */

/* Bit definition for TIM_CCMR2 register */
enum TIM_CCMR2_CC3S = cast(ushort)0x0003; /* CC3S[1:0] bits (Capture/Compare 3 Selection; */
enum TIM_CCMR2_CC3S_0 = cast(ushort)0x0001; /* Bit 0 */
enum TIM_CCMR2_CC3S_1 = cast(ushort)0x0002; /* Bit 1 */

enum TIM_CCMR2_OC3FE = cast(ushort)0x0004; /* Output Compare 3 Fast enable */
enum TIM_CCMR2_OC3PE = cast(ushort)0x0008; /* Output Compare 3 Preload enable */

enum TIM_CCMR2_OC3M = cast(ushort)0x0070; /* OC3M[2:0] bits (Output Compare 3 Mode; */
enum TIM_CCMR2_OC3M_0 = cast(ushort)0x0010; /* Bit 0 */
enum TIM_CCMR2_OC3M_1 = cast(ushort)0x0020; /* Bit 1 */
enum TIM_CCMR2_OC3M_2 = cast(ushort)0x0040; /* Bit 2 */

enum TIM_CCMR2_OC3CE = cast(ushort)0x0080; /* Output Compare 3 Clear Enable */

enum TIM_CCMR2_CC4S = cast(ushort)0x0300; /* CC4S[1:0] bits (Capture/Compare 4 Selection; */
enum TIM_CCMR2_CC4S_0 = cast(ushort)0x0100; /* Bit 0 */
enum TIM_CCMR2_CC4S_1 = cast(ushort)0x0200; /* Bit 1 */

enum TIM_CCMR2_OC4FE = cast(ushort)0x0400; /* Output Compare 4 Fast enable */
enum TIM_CCMR2_OC4PE = cast(ushort)0x0800; /* Output Compare 4 Preload enable */

enum TIM_CCMR2_OC4M = cast(ushort)0x7000; /* OC4M[2:0] bits (Output Compare 4 Mode; */
enum TIM_CCMR2_OC4M_0 = cast(ushort)0x1000; /* Bit 0 */
enum TIM_CCMR2_OC4M_1 = cast(ushort)0x2000; /* Bit 1 */
enum TIM_CCMR2_OC4M_2 = cast(ushort)0x4000; /* Bit 2 */

enum TIM_CCMR2_OC4CE = cast(ushort)0x8000; /* Output Compare 4 Clear Enable */

enum TIM_CCMR2_IC3PSC = cast(ushort)0x000C; /* IC3PSC[1:0] bits (Input Capture 3 Prescaler; */
enum TIM_CCMR2_IC3PSC_0 = cast(ushort)0x0004; /* Bit 0 */
enum TIM_CCMR2_IC3PSC_1 = cast(ushort)0x0008; /* Bit 1 */

enum TIM_CCMR2_IC3F = cast(ushort)0x00F0; /* IC3F[3:0] bits (Input Capture 3 Filter; */
enum TIM_CCMR2_IC3F_0 = cast(ushort)0x0010; /* Bit 0 */
enum TIM_CCMR2_IC3F_1 = cast(ushort)0x0020; /* Bit 1 */
enum TIM_CCMR2_IC3F_2 = cast(ushort)0x0040; /* Bit 2 */
enum TIM_CCMR2_IC3F_3 = cast(ushort)0x0080; /* Bit 3 */

enum TIM_CCMR2_IC4PSC = cast(ushort)0x0C00; /* IC4PSC[1:0] bits (Input Capture 4 Prescaler; */
enum TIM_CCMR2_IC4PSC_0 = cast(ushort)0x0400; /* Bit 0 */
enum TIM_CCMR2_IC4PSC_1 = cast(ushort)0x0800; /* Bit 1 */

enum TIM_CCMR2_IC4F = cast(ushort)0xF000; /* IC4F[3:0] bits (Input Capture 4 Filter; */
enum TIM_CCMR2_IC4F_0 = cast(ushort)0x1000; /* Bit 0 */
enum TIM_CCMR2_IC4F_1 = cast(ushort)0x2000; /* Bit 1 */
enum TIM_CCMR2_IC4F_2 = cast(ushort)0x4000; /* Bit 2 */
enum TIM_CCMR2_IC4F_3 = cast(ushort)0x8000; /* Bit 3 */

/* Bit definition for TIM_CCER register */
enum TIM_CCER_CC1E = cast(ushort)0x0001; /* Capture/Compare 1 output enable */
enum TIM_CCER_CC1P = cast(ushort)0x0002; /* Capture/Compare 1 output Polarity */
enum TIM_CCER_CC1NE = cast(ushort)0x0004; /* Capture/Compare 1 Complementary output enable */
enum TIM_CCER_CC1NP = cast(ushort)0x0008; /* Capture/Compare 1 Complementary output Polarity */
enum TIM_CCER_CC2E = cast(ushort)0x0010; /* Capture/Compare 2 output enable */
enum TIM_CCER_CC2P = cast(ushort)0x0020; /* Capture/Compare 2 output Polarity */
enum TIM_CCER_CC2NE = cast(ushort)0x0040; /* Capture/Compare 2 Complementary output enable */
enum TIM_CCER_CC2NP = cast(ushort)0x0080; /* Capture/Compare 2 Complementary output Polarity */
enum TIM_CCER_CC3E = cast(ushort)0x0100; /* Capture/Compare 3 output enable */
enum TIM_CCER_CC3P = cast(ushort)0x0200; /* Capture/Compare 3 output Polarity */
enum TIM_CCER_CC3NE = cast(ushort)0x0400; /* Capture/Compare 3 Complementary output enable */
enum TIM_CCER_CC3NP = cast(ushort)0x0800; /* Capture/Compare 3 Complementary output Polarity */
enum TIM_CCER_CC4E = cast(ushort)0x1000; /* Capture/Compare 4 output enable */
enum TIM_CCER_CC4P = cast(ushort)0x2000; /* Capture/Compare 4 output Polarity */
enum TIM_CCER_CC4NP = cast(ushort)0x8000; /* Capture/Compare 4 Complementary output Polarity */

/* Bit definition for TIM_CNT register */
enum TIM_CNT_CNT = cast(ushort)0xFFFF; /* Counter Value */

/* Bit definition for TIM_PSC register */
enum TIM_PSC_PSC = cast(ushort)0xFFFF; /* Prescaler Value */

/* Bit definition for TIM_ARR register */
enum TIM_ARR_ARR = cast(ushort)0xFFFF; /* actual auto-reload Value */

/* Bit definition for TIM_RCR register */
enum TIM_RCR_REP = cast(ubyte)0xFF; /* Repetition Counter Value */

/* Bit definition for TIM_CCR1 register */
enum TIM_CCR1_CCR1 = cast(ushort)0xFFFF; /* Capture/Compare 1 Value */

/* Bit definition for TIM_CCR2 register */
enum TIM_CCR2_CCR2 = cast(ushort)0xFFFF; /* Capture/Compare 2 Value */

/* Bit definition for TIM_CCR3 register */
enum TIM_CCR3_CCR3 = cast(ushort)0xFFFF; /* Capture/Compare 3 Value */

/* Bit definition for TIM_CCR4 register */
enum TIM_CCR4_CCR4 = cast(ushort)0xFFFF; /* Capture/Compare 4 Value */

/* Bit definition for TIM_BDTR register */
enum TIM_BDTR_DTG = cast(ushort)0x00FF; /* DTG[0:7] bits (Dead-Time Generator set-up; */
enum TIM_BDTR_DTG_0 = cast(ushort)0x0001; /* Bit 0 */
enum TIM_BDTR_DTG_1 = cast(ushort)0x0002; /* Bit 1 */
enum TIM_BDTR_DTG_2 = cast(ushort)0x0004; /* Bit 2 */
enum TIM_BDTR_DTG_3 = cast(ushort)0x0008; /* Bit 3 */
enum TIM_BDTR_DTG_4 = cast(ushort)0x0010; /* Bit 4 */
enum TIM_BDTR_DTG_5 = cast(ushort)0x0020; /* Bit 5 */
enum TIM_BDTR_DTG_6 = cast(ushort)0x0040; /* Bit 6 */
enum TIM_BDTR_DTG_7 = cast(ushort)0x0080; /* Bit 7 */

enum TIM_BDTR_LOCK = cast(ushort)0x0300; /* LOCK[1:0] bits (Lock Configuration; */
enum TIM_BDTR_LOCK_0 = cast(ushort)0x0100; /* Bit 0 */
enum TIM_BDTR_LOCK_1 = cast(ushort)0x0200; /* Bit 1 */

enum TIM_BDTR_OSSI = cast(ushort)0x0400; /* Off-State Selection for Idle mode */
enum TIM_BDTR_OSSR = cast(ushort)0x0800; /* Off-State Selection for Run mode */
enum TIM_BDTR_BKE = cast(ushort)0x1000; /* Break enable */
enum TIM_BDTR_BKP = cast(ushort)0x2000; /* Break Polarity */
enum TIM_BDTR_AOE = cast(ushort)0x4000; /* Automatic Output enable */
enum TIM_BDTR_MOE = cast(ushort)0x8000; /* Main Output enable */

/* Bit definition for TIM_DCR register */
enum TIM_DCR_DBA = cast(ushort)0x001F; /* DBA[4:0] bits (DMA Base Address; */
enum TIM_DCR_DBA_0 = cast(ushort)0x0001; /* Bit 0 */
enum TIM_DCR_DBA_1 = cast(ushort)0x0002; /* Bit 1 */
enum TIM_DCR_DBA_2 = cast(ushort)0x0004; /* Bit 2 */
enum TIM_DCR_DBA_3 = cast(ushort)0x0008; /* Bit 3 */
enum TIM_DCR_DBA_4 = cast(ushort)0x0010; /* Bit 4 */

enum TIM_DCR_DBL = cast(ushort)0x1F00; /* DBL[4:0] bits (DMA Burst Length; */
enum TIM_DCR_DBL_0 = cast(ushort)0x0100; /* Bit 0 */
enum TIM_DCR_DBL_1 = cast(ushort)0x0200; /* Bit 1 */
enum TIM_DCR_DBL_2 = cast(ushort)0x0400; /* Bit 2 */
enum TIM_DCR_DBL_3 = cast(ushort)0x0800; /* Bit 3 */
enum TIM_DCR_DBL_4 = cast(ushort)0x1000; /* Bit 4 */

/* Bit definition for TIM_DMAR register */
enum TIM_DMAR_DMAB = cast(ushort)0xFFFF; /* DMA register for burst accesses */

/* Bit definition for RTC_CRH register */
enum RTC_CRH_SECIE = cast(ubyte)0x01; /* Second Interrupt Enable */
enum RTC_CRH_ALRIE = cast(ubyte)0x02; /* Alarm Interrupt Enable */
enum RTC_CRH_OWIE = cast(ubyte)0x04; /* OverfloW Interrupt Enable */

/* Bit definition for RTC_CRL register */
enum RTC_CRL_SECF = cast(ubyte)0x01; /* Second Flag */
enum RTC_CRL_ALRF = cast(ubyte)0x02; /* Alarm Flag */
enum RTC_CRL_OWF = cast(ubyte)0x04; /* OverfloW Flag */
enum RTC_CRL_RSF = cast(ubyte)0x08; /* Registers Synchronized Flag */
enum RTC_CRL_CNF = cast(ubyte)0x10; /* Configuration Flag */
enum RTC_CRL_RTOFF = cast(ubyte)0x20; /* RTC operation OFF */

/* Bit definition for RTC_PRLH register */
enum RTC_PRLH_PRL = cast(ushort)0x000F; /* RTC Prescaler Reload Value High */

/* Bit definition for RTC_PRLL register */
enum RTC_PRLL_PRL = cast(ushort)0xFFFF; /* RTC Prescaler Reload Value Low */

/* Bit definition for RTC_DIVH register */
enum RTC_DIVH_RTC_DIV = cast(ushort)0x000F; /* RTC Clock DIVider High */

/* Bit definition for RTC_DIVL register */
enum RTC_DIVL_RTC_DIV = cast(ushort)0xFFFF; /* RTC Clock DIVider Low */

/* Bit definition for RTC_CNTH register */
enum RTC_CNTH_RTC_CNT = cast(ushort)0xFFFF; /* RTC Counter High */

/* Bit definition for RTC_CNTL register */
enum RTC_CNTL_RTC_CNT = cast(ushort)0xFFFF; /* RTC Counter Low */

/* Bit definition for RTC_ALRH register */
enum RTC_ALRH_RTC_ALR = cast(ushort)0xFFFF; /* RTC Alarm High */

/* Bit definition for RTC_ALRL register */
enum RTC_ALRL_RTC_ALR = cast(ushort)0xFFFF; /* RTC Alarm Low */

/* Bit definition for IWDG_KR register */
enum IWDG_KR_KEY = cast(ushort)0xFFFF; /* Key value (write only, read 0000h; */

/* Bit definition for IWDG_PR register */
enum IWDG_PR_PR = cast(ubyte)0x07; /* PR[2:0] (Prescaler divider; */
enum IWDG_PR_PR_0 = cast(ubyte)0x01; /* Bit 0 */
enum IWDG_PR_PR_1 = cast(ubyte)0x02; /* Bit 1 */
enum IWDG_PR_PR_2 = cast(ubyte)0x04; /* Bit 2 */

/* Bit definition for IWDG_RLR register */
enum IWDG_RLR_RL = cast(ushort)0x0FFF; /* Watchdog counter reload value */

/* Bit definition for IWDG_SR register */
enum IWDG_SR_PVU = cast(ubyte)0x01; /* Watchdog prescaler value update */
enum IWDG_SR_RVU = cast(ubyte)0x02; /* Watchdog counter reload value update */

/* Bit definition for WWDG_CR register */
enum WWDG_CR_T = cast(ubyte)0x7F; /* T[6:0] bits (7-Bit counter (MSB to LSB); */
enum WWDG_CR_T0 = cast(ubyte)0x01; /* Bit 0 */
enum WWDG_CR_T1 = cast(ubyte)0x02; /* Bit 1 */
enum WWDG_CR_T2 = cast(ubyte)0x04; /* Bit 2 */
enum WWDG_CR_T3 = cast(ubyte)0x08; /* Bit 3 */
enum WWDG_CR_T4 = cast(ubyte)0x10; /* Bit 4 */
enum WWDG_CR_T5 = cast(ubyte)0x20; /* Bit 5 */
enum WWDG_CR_T6 = cast(ubyte)0x40; /* Bit 6 */

enum WWDG_CR_WDGA = cast(ubyte)0x80; /* Activation bit */

/* Bit definition for WWDG_CFR register */
enum WWDG_CFR_W = cast(ushort)0x007F; /* W[6:0] bits (7-bit window value; */
enum WWDG_CFR_W0 = cast(ushort)0x0001; /* Bit 0 */
enum WWDG_CFR_W1 = cast(ushort)0x0002; /* Bit 1 */
enum WWDG_CFR_W2 = cast(ushort)0x0004; /* Bit 2 */
enum WWDG_CFR_W3 = cast(ushort)0x0008; /* Bit 3 */
enum WWDG_CFR_W4 = cast(ushort)0x0010; /* Bit 4 */
enum WWDG_CFR_W5 = cast(ushort)0x0020; /* Bit 5 */
enum WWDG_CFR_W6 = cast(ushort)0x0040; /* Bit 6 */

enum WWDG_CFR_WDGTB = cast(ushort)0x0180; /* WDGTB[1:0] bits (Timer Base; */
enum WWDG_CFR_WDGTB0 = cast(ushort)0x0080; /* Bit 0 */
enum WWDG_CFR_WDGTB1 = cast(ushort)0x0100; /* Bit 1 */

enum WWDG_CFR_EWI = cast(ushort)0x0200; /* Early Wakeup Interrupt */

/* Bit definition for WWDG_SR register */
enum WWDG_SR_EWIF = cast(ubyte)0x01; /* Early Wakeup Interrupt Flag */

/* Bit definition for FSMC_BCR1 register */
enum FSMC_BCR1_MBKEN = cast(uint)0x00000001; /* Memory bank enable bit */
enum FSMC_BCR1_MUXEN = cast(uint)0x00000002; /* Address/data multiplexing enable bit */

enum FSMC_BCR1_MTYP = cast(uint)0x0000000C; /* MTYP[1:0] bits (Memory type; */
enum FSMC_BCR1_MTYP_0 = cast(uint)0x00000004; /* Bit 0 */
enum FSMC_BCR1_MTYP_1 = cast(uint)0x00000008; /* Bit 1 */

enum FSMC_BCR1_MWID = cast(uint)0x00000030; /* MWID[1:0] bits (Memory data bus width; */
enum FSMC_BCR1_MWID_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BCR1_MWID_1 = cast(uint)0x00000020; /* Bit 1 */

enum FSMC_BCR1_FACCEN = cast(uint)0x00000040; /* Flash access enable */
enum FSMC_BCR1_BURSTEN = cast(uint)0x00000100; /* Burst enable bit */
enum FSMC_BCR1_WAITPOL = cast(uint)0x00000200; /* Wait signal polarity bit */
enum FSMC_BCR1_WRAPMOD = cast(uint)0x00000400; /* Wrapped burst mode support */
enum FSMC_BCR1_WAITCFG = cast(uint)0x00000800; /* Wait timing configuration */
enum FSMC_BCR1_WREN = cast(uint)0x00001000; /* Write enable bit */
enum FSMC_BCR1_WAITEN = cast(uint)0x00002000; /* Wait enable bit */
enum FSMC_BCR1_EXTMOD = cast(uint)0x00004000; /* Extended mode enable */
enum FSMC_BCR1_ASYNCWAIT = cast(uint)0x00008000; /* Asynchronous wait */
enum FSMC_BCR1_CBURSTRW = cast(uint)0x00080000; /* Write burst enable */

/* Bit definition for FSMC_BCR2 register */
enum FSMC_BCR2_MBKEN = cast(uint)0x00000001; /* Memory bank enable bit */
enum FSMC_BCR2_MUXEN = cast(uint)0x00000002; /* Address/data multiplexing enable bit */

enum FSMC_BCR2_MTYP = cast(uint)0x0000000C; /* MTYP[1:0] bits (Memory type; */
enum FSMC_BCR2_MTYP_0 = cast(uint)0x00000004; /* Bit 0 */
enum FSMC_BCR2_MTYP_1 = cast(uint)0x00000008; /* Bit 1 */

enum FSMC_BCR2_MWID = cast(uint)0x00000030; /* MWID[1:0] bits (Memory data bus width; */
enum FSMC_BCR2_MWID_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BCR2_MWID_1 = cast(uint)0x00000020; /* Bit 1 */

enum FSMC_BCR2_FACCEN = cast(uint)0x00000040; /* Flash access enable */
enum FSMC_BCR2_BURSTEN = cast(uint)0x00000100; /* Burst enable bit */
enum FSMC_BCR2_WAITPOL = cast(uint)0x00000200; /* Wait signal polarity bit */
enum FSMC_BCR2_WRAPMOD = cast(uint)0x00000400; /* Wrapped burst mode support */
enum FSMC_BCR2_WAITCFG = cast(uint)0x00000800; /* Wait timing configuration */
enum FSMC_BCR2_WREN = cast(uint)0x00001000; /* Write enable bit */
enum FSMC_BCR2_WAITEN = cast(uint)0x00002000; /* Wait enable bit */
enum FSMC_BCR2_EXTMOD = cast(uint)0x00004000; /* Extended mode enable */
enum FSMC_BCR2_ASYNCWAIT = cast(uint)0x00008000; /* Asynchronous wait */
enum FSMC_BCR2_CBURSTRW = cast(uint)0x00080000; /* Write burst enable */

/* Bit definition for FSMC_BCR3 register */
enum FSMC_BCR3_MBKEN = cast(uint)0x00000001; /* Memory bank enable bit */
enum FSMC_BCR3_MUXEN = cast(uint)0x00000002; /* Address/data multiplexing enable bit */

enum FSMC_BCR3_MTYP = cast(uint)0x0000000C; /* MTYP[1:0] bits (Memory type; */
enum FSMC_BCR3_MTYP_0 = cast(uint)0x00000004; /* Bit 0 */
enum FSMC_BCR3_MTYP_1 = cast(uint)0x00000008; /* Bit 1 */

enum FSMC_BCR3_MWID = cast(uint)0x00000030; /* MWID[1:0] bits (Memory data bus width; */
enum FSMC_BCR3_MWID_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BCR3_MWID_1 = cast(uint)0x00000020; /* Bit 1 */

enum FSMC_BCR3_FACCEN = cast(uint)0x00000040; /* Flash access enable */
enum FSMC_BCR3_BURSTEN = cast(uint)0x00000100; /* Burst enable bit */
enum FSMC_BCR3_WAITPOL = cast(uint)0x00000200; /* Wait signal polarity bit. */
enum FSMC_BCR3_WRAPMOD = cast(uint)0x00000400; /* Wrapped burst mode support */
enum FSMC_BCR3_WAITCFG = cast(uint)0x00000800; /* Wait timing configuration */
enum FSMC_BCR3_WREN = cast(uint)0x00001000; /* Write enable bit */
enum FSMC_BCR3_WAITEN = cast(uint)0x00002000; /* Wait enable bit */
enum FSMC_BCR3_EXTMOD = cast(uint)0x00004000; /* Extended mode enable */
enum FSMC_BCR3_ASYNCWAIT = cast(uint)0x00008000; /* Asynchronous wait */
enum FSMC_BCR3_CBURSTRW = cast(uint)0x00080000; /* Write burst enable */

/* Bit definition for FSMC_BCR4 register */
enum FSMC_BCR4_MBKEN = cast(uint)0x00000001; /* Memory bank enable bit */
enum FSMC_BCR4_MUXEN = cast(uint)0x00000002; /* Address/data multiplexing enable bit */

enum FSMC_BCR4_MTYP = cast(uint)0x0000000C; /* MTYP[1:0] bits (Memory type; */
enum FSMC_BCR4_MTYP_0 = cast(uint)0x00000004; /* Bit 0 */
enum FSMC_BCR4_MTYP_1 = cast(uint)0x00000008; /* Bit 1 */

enum FSMC_BCR4_MWID = cast(uint)0x00000030; /* MWID[1:0] bits (Memory data bus width; */
enum FSMC_BCR4_MWID_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BCR4_MWID_1 = cast(uint)0x00000020; /* Bit 1 */

enum FSMC_BCR4_FACCEN = cast(uint)0x00000040; /* Flash access enable */
enum FSMC_BCR4_BURSTEN = cast(uint)0x00000100; /* Burst enable bit */
enum FSMC_BCR4_WAITPOL = cast(uint)0x00000200; /* Wait signal polarity bit */
enum FSMC_BCR4_WRAPMOD = cast(uint)0x00000400; /* Wrapped burst mode support */
enum FSMC_BCR4_WAITCFG = cast(uint)0x00000800; /* Wait timing configuration */
enum FSMC_BCR4_WREN = cast(uint)0x00001000; /* Write enable bit */
enum FSMC_BCR4_WAITEN = cast(uint)0x00002000; /* Wait enable bit */
enum FSMC_BCR4_EXTMOD = cast(uint)0x00004000; /* Extended mode enable */
enum FSMC_BCR4_ASYNCWAIT = cast(uint)0x00008000; /* Asynchronous wait */
enum FSMC_BCR4_CBURSTRW = cast(uint)0x00080000; /* Write burst enable */

/* Bit definition for FSMC_BTR1 register */
enum FSMC_BTR1_ADDSET = cast(uint)0x0000000F; /* ADDSET[3:0] bits (Address setup phase duration; */
enum FSMC_BTR1_ADDSET_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_BTR1_ADDSET_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_BTR1_ADDSET_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_BTR1_ADDSET_3 = cast(uint)0x00000008; /* Bit 3 */

enum FSMC_BTR1_ADDHLD = cast(uint)0x000000F0; /* ADDHLD[3:0] bits (Address-hold phase duration; */
enum FSMC_BTR1_ADDHLD_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BTR1_ADDHLD_1 = cast(uint)0x00000020; /* Bit 1 */
enum FSMC_BTR1_ADDHLD_2 = cast(uint)0x00000040; /* Bit 2 */
enum FSMC_BTR1_ADDHLD_3 = cast(uint)0x00000080; /* Bit 3 */

enum FSMC_BTR1_DATAST = cast(uint)0x0000FF00; /* DATAST [3:0] bits (Data-phase duration; */
enum FSMC_BTR1_DATAST_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_BTR1_DATAST_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_BTR1_DATAST_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_BTR1_DATAST_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_BTR1_DATAST_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_BTR1_DATAST_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_BTR1_DATAST_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_BTR1_DATAST_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_BTR1_BUSTURN = cast(uint)0x000F0000; /* BUSTURN[3:0] bits (Bus turnaround phase duration; */
enum FSMC_BTR1_BUSTURN_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_BTR1_BUSTURN_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_BTR1_BUSTURN_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_BTR1_BUSTURN_3 = cast(uint)0x00080000; /* Bit 3 */

enum FSMC_BTR1_CLKDIV = cast(uint)0x00F00000; /* CLKDIV[3:0] bits (Clock divide ratio; */
enum FSMC_BTR1_CLKDIV_0 = cast(uint)0x00100000; /* Bit 0 */
enum FSMC_BTR1_CLKDIV_1 = cast(uint)0x00200000; /* Bit 1 */
enum FSMC_BTR1_CLKDIV_2 = cast(uint)0x00400000; /* Bit 2 */
enum FSMC_BTR1_CLKDIV_3 = cast(uint)0x00800000; /* Bit 3 */

enum FSMC_BTR1_DATLAT = cast(uint)0x0F000000; /* DATLA[3:0] bits (Data latency; */
enum FSMC_BTR1_DATLAT_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_BTR1_DATLAT_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_BTR1_DATLAT_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_BTR1_DATLAT_3 = cast(uint)0x08000000; /* Bit 3 */

enum FSMC_BTR1_ACCMOD = cast(uint)0x30000000; /* ACCMOD[1:0] bits (Access mode; */
enum FSMC_BTR1_ACCMOD_0 = cast(uint)0x10000000; /* Bit 0 */
enum FSMC_BTR1_ACCMOD_1 = cast(uint)0x20000000; /* Bit 1 */

/* Bit definition for FSMC_BTR2 register */
enum FSMC_BTR2_ADDSET = cast(uint)0x0000000F; /* ADDSET[3:0] bits (Address setup phase duration; */
enum FSMC_BTR2_ADDSET_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_BTR2_ADDSET_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_BTR2_ADDSET_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_BTR2_ADDSET_3 = cast(uint)0x00000008; /* Bit 3 */

enum FSMC_BTR2_ADDHLD = cast(uint)0x000000F0; /* ADDHLD[3:0] bits (Address-hold phase duration; */
enum FSMC_BTR2_ADDHLD_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BTR2_ADDHLD_1 = cast(uint)0x00000020; /* Bit 1 */
enum FSMC_BTR2_ADDHLD_2 = cast(uint)0x00000040; /* Bit 2 */
enum FSMC_BTR2_ADDHLD_3 = cast(uint)0x00000080; /* Bit 3 */

enum FSMC_BTR2_DATAST = cast(uint)0x0000FF00; /* DATAST [3:0] bits (Data-phase duration; */
enum FSMC_BTR2_DATAST_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_BTR2_DATAST_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_BTR2_DATAST_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_BTR2_DATAST_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_BTR2_DATAST_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_BTR2_DATAST_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_BTR2_DATAST_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_BTR2_DATAST_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_BTR2_BUSTURN = cast(uint)0x000F0000; /* BUSTURN[3:0] bits (Bus turnaround phase duration; */
enum FSMC_BTR2_BUSTURN_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_BTR2_BUSTURN_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_BTR2_BUSTURN_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_BTR2_BUSTURN_3 = cast(uint)0x00080000; /* Bit 3 */

enum FSMC_BTR2_CLKDIV = cast(uint)0x00F00000; /* CLKDIV[3:0] bits (Clock divide ratio; */
enum FSMC_BTR2_CLKDIV_0 = cast(uint)0x00100000; /* Bit 0 */
enum FSMC_BTR2_CLKDIV_1 = cast(uint)0x00200000; /* Bit 1 */
enum FSMC_BTR2_CLKDIV_2 = cast(uint)0x00400000; /* Bit 2 */
enum FSMC_BTR2_CLKDIV_3 = cast(uint)0x00800000; /* Bit 3 */

enum FSMC_BTR2_DATLAT = cast(uint)0x0F000000; /* DATLA[3:0] bits (Data latency; */
enum FSMC_BTR2_DATLAT_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_BTR2_DATLAT_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_BTR2_DATLAT_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_BTR2_DATLAT_3 = cast(uint)0x08000000; /* Bit 3 */

enum FSMC_BTR2_ACCMOD = cast(uint)0x30000000; /* ACCMOD[1:0] bits (Access mode; */
enum FSMC_BTR2_ACCMOD_0 = cast(uint)0x10000000; /* Bit 0 */
enum FSMC_BTR2_ACCMOD_1 = cast(uint)0x20000000; /* Bit 1 */

/* Bit definition for FSMC_BTR3 register */
enum FSMC_BTR3_ADDSET = cast(uint)0x0000000F; /* ADDSET[3:0] bits (Address setup phase duration; */
enum FSMC_BTR3_ADDSET_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_BTR3_ADDSET_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_BTR3_ADDSET_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_BTR3_ADDSET_3 = cast(uint)0x00000008; /* Bit 3 */

enum FSMC_BTR3_ADDHLD = cast(uint)0x000000F0; /* ADDHLD[3:0] bits (Address-hold phase duration; */
enum FSMC_BTR3_ADDHLD_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BTR3_ADDHLD_1 = cast(uint)0x00000020; /* Bit 1 */
enum FSMC_BTR3_ADDHLD_2 = cast(uint)0x00000040; /* Bit 2 */
enum FSMC_BTR3_ADDHLD_3 = cast(uint)0x00000080; /* Bit 3 */

enum FSMC_BTR3_DATAST = cast(uint)0x0000FF00; /* DATAST [3:0] bits (Data-phase duration; */
enum FSMC_BTR3_DATAST_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_BTR3_DATAST_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_BTR3_DATAST_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_BTR3_DATAST_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_BTR3_DATAST_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_BTR3_DATAST_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_BTR3_DATAST_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_BTR3_DATAST_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_BTR3_BUSTURN = cast(uint)0x000F0000; /* BUSTURN[3:0] bits (Bus turnaround phase duration; */
enum FSMC_BTR3_BUSTURN_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_BTR3_BUSTURN_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_BTR3_BUSTURN_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_BTR3_BUSTURN_3 = cast(uint)0x00080000; /* Bit 3 */

enum FSMC_BTR3_CLKDIV = cast(uint)0x00F00000; /* CLKDIV[3:0] bits (Clock divide ratio; */
enum FSMC_BTR3_CLKDIV_0 = cast(uint)0x00100000; /* Bit 0 */
enum FSMC_BTR3_CLKDIV_1 = cast(uint)0x00200000; /* Bit 1 */
enum FSMC_BTR3_CLKDIV_2 = cast(uint)0x00400000; /* Bit 2 */
enum FSMC_BTR3_CLKDIV_3 = cast(uint)0x00800000; /* Bit 3 */

enum FSMC_BTR3_DATLAT = cast(uint)0x0F000000; /* DATLA[3:0] bits (Data latency; */
enum FSMC_BTR3_DATLAT_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_BTR3_DATLAT_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_BTR3_DATLAT_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_BTR3_DATLAT_3 = cast(uint)0x08000000; /* Bit 3 */

enum FSMC_BTR3_ACCMOD = cast(uint)0x30000000; /* ACCMOD[1:0] bits (Access mode; */
enum FSMC_BTR3_ACCMOD_0 = cast(uint)0x10000000; /* Bit 0 */
enum FSMC_BTR3_ACCMOD_1 = cast(uint)0x20000000; /* Bit 1 */

/* Bit definition for FSMC_BTR4 register */
enum FSMC_BTR4_ADDSET = cast(uint)0x0000000F; /* ADDSET[3:0] bits (Address setup phase duration; */
enum FSMC_BTR4_ADDSET_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_BTR4_ADDSET_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_BTR4_ADDSET_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_BTR4_ADDSET_3 = cast(uint)0x00000008; /* Bit 3 */

enum FSMC_BTR4_ADDHLD = cast(uint)0x000000F0; /* ADDHLD[3:0] bits (Address-hold phase duration; */
enum FSMC_BTR4_ADDHLD_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BTR4_ADDHLD_1 = cast(uint)0x00000020; /* Bit 1 */
enum FSMC_BTR4_ADDHLD_2 = cast(uint)0x00000040; /* Bit 2 */
enum FSMC_BTR4_ADDHLD_3 = cast(uint)0x00000080; /* Bit 3 */

enum FSMC_BTR4_DATAST = cast(uint)0x0000FF00; /* DATAST [3:0] bits (Data-phase duration; */
enum FSMC_BTR4_DATAST_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_BTR4_DATAST_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_BTR4_DATAST_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_BTR4_DATAST_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_BTR4_DATAST_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_BTR4_DATAST_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_BTR4_DATAST_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_BTR4_DATAST_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_BTR4_BUSTURN = cast(uint)0x000F0000; /* BUSTURN[3:0] bits (Bus turnaround phase duration; */
enum FSMC_BTR4_BUSTURN_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_BTR4_BUSTURN_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_BTR4_BUSTURN_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_BTR4_BUSTURN_3 = cast(uint)0x00080000; /* Bit 3 */

enum FSMC_BTR4_CLKDIV = cast(uint)0x00F00000; /* CLKDIV[3:0] bits (Clock divide ratio; */
enum FSMC_BTR4_CLKDIV_0 = cast(uint)0x00100000; /* Bit 0 */
enum FSMC_BTR4_CLKDIV_1 = cast(uint)0x00200000; /* Bit 1 */
enum FSMC_BTR4_CLKDIV_2 = cast(uint)0x00400000; /* Bit 2 */
enum FSMC_BTR4_CLKDIV_3 = cast(uint)0x00800000; /* Bit 3 */

enum FSMC_BTR4_DATLAT = cast(uint)0x0F000000; /* DATLA[3:0] bits (Data latency; */
enum FSMC_BTR4_DATLAT_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_BTR4_DATLAT_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_BTR4_DATLAT_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_BTR4_DATLAT_3 = cast(uint)0x08000000; /* Bit 3 */

enum FSMC_BTR4_ACCMOD = cast(uint)0x30000000; /* ACCMOD[1:0] bits (Access mode; */
enum FSMC_BTR4_ACCMOD_0 = cast(uint)0x10000000; /* Bit 0 */
enum FSMC_BTR4_ACCMOD_1 = cast(uint)0x20000000; /* Bit 1 */

/* Bit definition for FSMC_BWTR1 register */
enum FSMC_BWTR1_ADDSET = cast(uint)0x0000000F; /* ADDSET[3:0] bits (Address setup phase duration; */
enum FSMC_BWTR1_ADDSET_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_BWTR1_ADDSET_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_BWTR1_ADDSET_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_BWTR1_ADDSET_3 = cast(uint)0x00000008; /* Bit 3 */

enum FSMC_BWTR1_ADDHLD = cast(uint)0x000000F0; /* ADDHLD[3:0] bits (Address-hold phase duration; */
enum FSMC_BWTR1_ADDHLD_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BWTR1_ADDHLD_1 = cast(uint)0x00000020; /* Bit 1 */
enum FSMC_BWTR1_ADDHLD_2 = cast(uint)0x00000040; /* Bit 2 */
enum FSMC_BWTR1_ADDHLD_3 = cast(uint)0x00000080; /* Bit 3 */

enum FSMC_BWTR1_DATAST = cast(uint)0x0000FF00; /* DATAST [3:0] bits (Data-phase duration; */
enum FSMC_BWTR1_DATAST_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_BWTR1_DATAST_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_BWTR1_DATAST_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_BWTR1_DATAST_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_BWTR1_DATAST_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_BWTR1_DATAST_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_BWTR1_DATAST_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_BWTR1_DATAST_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_BWTR1_CLKDIV = cast(uint)0x00F00000; /* CLKDIV[3:0] bits (Clock divide ratio; */
enum FSMC_BWTR1_CLKDIV_0 = cast(uint)0x00100000; /* Bit 0 */
enum FSMC_BWTR1_CLKDIV_1 = cast(uint)0x00200000; /* Bit 1 */
enum FSMC_BWTR1_CLKDIV_2 = cast(uint)0x00400000; /* Bit 2 */
enum FSMC_BWTR1_CLKDIV_3 = cast(uint)0x00800000; /* Bit 3 */

enum FSMC_BWTR1_DATLAT = cast(uint)0x0F000000; /* DATLA[3:0] bits (Data latency; */
enum FSMC_BWTR1_DATLAT_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_BWTR1_DATLAT_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_BWTR1_DATLAT_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_BWTR1_DATLAT_3 = cast(uint)0x08000000; /* Bit 3 */

enum FSMC_BWTR1_ACCMOD = cast(uint)0x30000000; /* ACCMOD[1:0] bits (Access mode; */
enum FSMC_BWTR1_ACCMOD_0 = cast(uint)0x10000000; /* Bit 0 */
enum FSMC_BWTR1_ACCMOD_1 = cast(uint)0x20000000; /* Bit 1 */

/* Bit definition for FSMC_BWTR2 register */
enum FSMC_BWTR2_ADDSET = cast(uint)0x0000000F; /* ADDSET[3:0] bits (Address setup phase duration; */
enum FSMC_BWTR2_ADDSET_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_BWTR2_ADDSET_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_BWTR2_ADDSET_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_BWTR2_ADDSET_3 = cast(uint)0x00000008; /* Bit 3 */

enum FSMC_BWTR2_ADDHLD = cast(uint)0x000000F0; /* ADDHLD[3:0] bits (Address-hold phase duration; */
enum FSMC_BWTR2_ADDHLD_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BWTR2_ADDHLD_1 = cast(uint)0x00000020; /* Bit 1 */
enum FSMC_BWTR2_ADDHLD_2 = cast(uint)0x00000040; /* Bit 2 */
enum FSMC_BWTR2_ADDHLD_3 = cast(uint)0x00000080; /* Bit 3 */

enum FSMC_BWTR2_DATAST = cast(uint)0x0000FF00; /* DATAST [3:0] bits (Data-phase duration; */
enum FSMC_BWTR2_DATAST_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_BWTR2_DATAST_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_BWTR2_DATAST_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_BWTR2_DATAST_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_BWTR2_DATAST_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_BWTR2_DATAST_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_BWTR2_DATAST_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_BWTR2_DATAST_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_BWTR2_CLKDIV = cast(uint)0x00F00000; /* CLKDIV[3:0] bits (Clock divide ratio; */
enum FSMC_BWTR2_CLKDIV_0 = cast(uint)0x00100000; /* Bit 0 */
enum FSMC_BWTR2_CLKDIV_1 = cast(uint)0x00200000; /* Bit 1*/
enum FSMC_BWTR2_CLKDIV_2 = cast(uint)0x00400000; /* Bit 2 */
enum FSMC_BWTR2_CLKDIV_3 = cast(uint)0x00800000; /* Bit 3 */

enum FSMC_BWTR2_DATLAT = cast(uint)0x0F000000; /* DATLA[3:0] bits (Data latency; */
enum FSMC_BWTR2_DATLAT_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_BWTR2_DATLAT_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_BWTR2_DATLAT_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_BWTR2_DATLAT_3 = cast(uint)0x08000000; /* Bit 3 */

enum FSMC_BWTR2_ACCMOD = cast(uint)0x30000000; /* ACCMOD[1:0] bits (Access mode; */
enum FSMC_BWTR2_ACCMOD_0 = cast(uint)0x10000000; /* Bit 0 */
enum FSMC_BWTR2_ACCMOD_1 = cast(uint)0x20000000; /* Bit 1 */

/* Bit definition for FSMC_BWTR3 register */
enum FSMC_BWTR3_ADDSET = cast(uint)0x0000000F; /* ADDSET[3:0] bits (Address setup phase duration; */
enum FSMC_BWTR3_ADDSET_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_BWTR3_ADDSET_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_BWTR3_ADDSET_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_BWTR3_ADDSET_3 = cast(uint)0x00000008; /* Bit 3 */

enum FSMC_BWTR3_ADDHLD = cast(uint)0x000000F0; /* ADDHLD[3:0] bits (Address-hold phase duration; */
enum FSMC_BWTR3_ADDHLD_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BWTR3_ADDHLD_1 = cast(uint)0x00000020; /* Bit 1 */
enum FSMC_BWTR3_ADDHLD_2 = cast(uint)0x00000040; /* Bit 2 */
enum FSMC_BWTR3_ADDHLD_3 = cast(uint)0x00000080; /* Bit 3 */

enum FSMC_BWTR3_DATAST = cast(uint)0x0000FF00; /* DATAST [3:0] bits (Data-phase duration; */
enum FSMC_BWTR3_DATAST_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_BWTR3_DATAST_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_BWTR3_DATAST_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_BWTR3_DATAST_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_BWTR3_DATAST_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_BWTR3_DATAST_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_BWTR3_DATAST_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_BWTR3_DATAST_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_BWTR3_CLKDIV = cast(uint)0x00F00000; /* CLKDIV[3:0] bits (Clock divide ratio; */
enum FSMC_BWTR3_CLKDIV_0 = cast(uint)0x00100000; /* Bit 0 */
enum FSMC_BWTR3_CLKDIV_1 = cast(uint)0x00200000; /* Bit 1 */
enum FSMC_BWTR3_CLKDIV_2 = cast(uint)0x00400000; /* Bit 2 */
enum FSMC_BWTR3_CLKDIV_3 = cast(uint)0x00800000; /* Bit 3 */

enum FSMC_BWTR3_DATLAT = cast(uint)0x0F000000; /* DATLA[3:0] bits (Data latency; */
enum FSMC_BWTR3_DATLAT_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_BWTR3_DATLAT_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_BWTR3_DATLAT_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_BWTR3_DATLAT_3 = cast(uint)0x08000000; /* Bit 3 */

enum FSMC_BWTR3_ACCMOD = cast(uint)0x30000000; /* ACCMOD[1:0] bits (Access mode; */
enum FSMC_BWTR3_ACCMOD_0 = cast(uint)0x10000000; /* Bit 0 */
enum FSMC_BWTR3_ACCMOD_1 = cast(uint)0x20000000; /* Bit 1 */

/* Bit definition for FSMC_BWTR4 register */
enum FSMC_BWTR4_ADDSET = cast(uint)0x0000000F; /* ADDSET[3:0] bits (Address setup phase duration; */
enum FSMC_BWTR4_ADDSET_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_BWTR4_ADDSET_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_BWTR4_ADDSET_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_BWTR4_ADDSET_3 = cast(uint)0x00000008; /* Bit 3 */

enum FSMC_BWTR4_ADDHLD = cast(uint)0x000000F0; /* ADDHLD[3:0] bits (Address-hold phase duration; */
enum FSMC_BWTR4_ADDHLD_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_BWTR4_ADDHLD_1 = cast(uint)0x00000020; /* Bit 1 */
enum FSMC_BWTR4_ADDHLD_2 = cast(uint)0x00000040; /* Bit 2 */
enum FSMC_BWTR4_ADDHLD_3 = cast(uint)0x00000080; /* Bit 3 */

enum FSMC_BWTR4_DATAST = cast(uint)0x0000FF00; /* DATAST [3:0] bits (Data-phase duration; */
enum FSMC_BWTR4_DATAST_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_BWTR4_DATAST_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_BWTR4_DATAST_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_BWTR4_DATAST_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_BWTR4_DATAST_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_BWTR4_DATAST_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_BWTR4_DATAST_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_BWTR4_DATAST_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_BWTR4_CLKDIV = cast(uint)0x00F00000; /* CLKDIV[3:0] bits (Clock divide ratio; */
enum FSMC_BWTR4_CLKDIV_0 = cast(uint)0x00100000; /* Bit 0 */
enum FSMC_BWTR4_CLKDIV_1 = cast(uint)0x00200000; /* Bit 1 */
enum FSMC_BWTR4_CLKDIV_2 = cast(uint)0x00400000; /* Bit 2 */
enum FSMC_BWTR4_CLKDIV_3 = cast(uint)0x00800000; /* Bit 3 */

enum FSMC_BWTR4_DATLAT = cast(uint)0x0F000000; /* DATLA[3:0] bits (Data latency; */
enum FSMC_BWTR4_DATLAT_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_BWTR4_DATLAT_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_BWTR4_DATLAT_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_BWTR4_DATLAT_3 = cast(uint)0x08000000; /* Bit 3 */

enum FSMC_BWTR4_ACCMOD = cast(uint)0x30000000; /* ACCMOD[1:0] bits (Access mode; */
enum FSMC_BWTR4_ACCMOD_0 = cast(uint)0x10000000; /* Bit 0 */
enum FSMC_BWTR4_ACCMOD_1 = cast(uint)0x20000000; /* Bit 1 */

/* Bit definition for FSMC_PCR2 register */
enum FSMC_PCR2_PWAITEN = cast(uint)0x00000002; /* Wait feature enable bit */
enum FSMC_PCR2_PBKEN = cast(uint)0x00000004; /* PC Card/NAND Flash memory bank enable bit */
enum FSMC_PCR2_PTYP = cast(uint)0x00000008; /* Memory type */

enum FSMC_PCR2_PWID = cast(uint)0x00000030; /* PWID[1:0] bits (NAND Flash databus width; */
enum FSMC_PCR2_PWID_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_PCR2_PWID_1 = cast(uint)0x00000020; /* Bit 1 */

enum FSMC_PCR2_ECCEN = cast(uint)0x00000040; /* ECC computation logic enable bit */

enum FSMC_PCR2_TCLR = cast(uint)0x00001E00; /* TCLR[3:0] bits (CLE to RE delay; */
enum FSMC_PCR2_TCLR_0 = cast(uint)0x00000200; /* Bit 0 */
enum FSMC_PCR2_TCLR_1 = cast(uint)0x00000400; /* Bit 1 */
enum FSMC_PCR2_TCLR_2 = cast(uint)0x00000800; /* Bit 2 */
enum FSMC_PCR2_TCLR_3 = cast(uint)0x00001000; /* Bit 3 */

enum FSMC_PCR2_TAR = cast(uint)0x0001E000; /* TAR[3:0] bits (ALE to RE delay; */
enum FSMC_PCR2_TAR_0 = cast(uint)0x00002000; /* Bit 0 */
enum FSMC_PCR2_TAR_1 = cast(uint)0x00004000; /* Bit 1 */
enum FSMC_PCR2_TAR_2 = cast(uint)0x00008000; /* Bit 2 */
enum FSMC_PCR2_TAR_3 = cast(uint)0x00010000; /* Bit 3 */

enum FSMC_PCR2_ECCPS = cast(uint)0x000E0000; /* ECCPS[1:0] bits (ECC page size; */
enum FSMC_PCR2_ECCPS_0 = cast(uint)0x00020000; /* Bit 0 */
enum FSMC_PCR2_ECCPS_1 = cast(uint)0x00040000; /* Bit 1 */
enum FSMC_PCR2_ECCPS_2 = cast(uint)0x00080000; /* Bit 2 */

/* Bit definition for FSMC_PCR3 register */
enum FSMC_PCR3_PWAITEN = cast(uint)0x00000002; /* Wait feature enable bit */
enum FSMC_PCR3_PBKEN = cast(uint)0x00000004; /* PC Card/NAND Flash memory bank enable bit */
enum FSMC_PCR3_PTYP = cast(uint)0x00000008; /* Memory type */

enum FSMC_PCR3_PWID = cast(uint)0x00000030; /* PWID[1:0] bits (NAND Flash databus width; */
enum FSMC_PCR3_PWID_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_PCR3_PWID_1 = cast(uint)0x00000020; /* Bit 1 */

enum FSMC_PCR3_ECCEN = cast(uint)0x00000040; /* ECC computation logic enable bit */

enum FSMC_PCR3_TCLR = cast(uint)0x00001E00; /* TCLR[3:0] bits (CLE to RE delay; */
enum FSMC_PCR3_TCLR_0 = cast(uint)0x00000200; /* Bit 0 */
enum FSMC_PCR3_TCLR_1 = cast(uint)0x00000400; /* Bit 1 */
enum FSMC_PCR3_TCLR_2 = cast(uint)0x00000800; /* Bit 2 */
enum FSMC_PCR3_TCLR_3 = cast(uint)0x00001000; /* Bit 3 */

enum FSMC_PCR3_TAR = cast(uint)0x0001E000; /* TAR[3:0] bits (ALE to RE delay; */
enum FSMC_PCR3_TAR_0 = cast(uint)0x00002000; /* Bit 0 */
enum FSMC_PCR3_TAR_1 = cast(uint)0x00004000; /* Bit 1 */
enum FSMC_PCR3_TAR_2 = cast(uint)0x00008000; /* Bit 2 */
enum FSMC_PCR3_TAR_3 = cast(uint)0x00010000; /* Bit 3 */

enum FSMC_PCR3_ECCPS = cast(uint)0x000E0000; /* ECCPS[2:0] bits (ECC page size; */
enum FSMC_PCR3_ECCPS_0 = cast(uint)0x00020000; /* Bit 0 */
enum FSMC_PCR3_ECCPS_1 = cast(uint)0x00040000; /* Bit 1 */
enum FSMC_PCR3_ECCPS_2 = cast(uint)0x00080000; /* Bit 2 */

/* Bit definition for FSMC_PCR4 register */
enum FSMC_PCR4_PWAITEN = cast(uint)0x00000002; /* Wait feature enable bit */
enum FSMC_PCR4_PBKEN = cast(uint)0x00000004; /* PC Card/NAND Flash memory bank enable bit */
enum FSMC_PCR4_PTYP = cast(uint)0x00000008; /* Memory type */

enum FSMC_PCR4_PWID = cast(uint)0x00000030; /* PWID[1:0] bits (NAND Flash databus width; */
enum FSMC_PCR4_PWID_0 = cast(uint)0x00000010; /* Bit 0 */
enum FSMC_PCR4_PWID_1 = cast(uint)0x00000020; /* Bit 1 */

enum FSMC_PCR4_ECCEN = cast(uint)0x00000040; /* ECC computation logic enable bit */

enum FSMC_PCR4_TCLR = cast(uint)0x00001E00; /* TCLR[3:0] bits (CLE to RE delay; */
enum FSMC_PCR4_TCLR_0 = cast(uint)0x00000200; /* Bit 0 */
enum FSMC_PCR4_TCLR_1 = cast(uint)0x00000400; /* Bit 1 */
enum FSMC_PCR4_TCLR_2 = cast(uint)0x00000800; /* Bit 2 */
enum FSMC_PCR4_TCLR_3 = cast(uint)0x00001000; /* Bit 3 */

enum FSMC_PCR4_TAR = cast(uint)0x0001E000; /* TAR[3:0] bits (ALE to RE delay; */
enum FSMC_PCR4_TAR_0 = cast(uint)0x00002000; /* Bit 0 */
enum FSMC_PCR4_TAR_1 = cast(uint)0x00004000; /* Bit 1 */
enum FSMC_PCR4_TAR_2 = cast(uint)0x00008000; /* Bit 2 */
enum FSMC_PCR4_TAR_3 = cast(uint)0x00010000; /* Bit 3 */

enum FSMC_PCR4_ECCPS = cast(uint)0x000E0000; /* ECCPS[2:0] bits (ECC page size; */
enum FSMC_PCR4_ECCPS_0 = cast(uint)0x00020000; /* Bit 0 */
enum FSMC_PCR4_ECCPS_1 = cast(uint)0x00040000; /* Bit 1 */
enum FSMC_PCR4_ECCPS_2 = cast(uint)0x00080000; /* Bit 2 */

/* Bit definition for FSMC_SR2 register */
enum FSMC_SR2_IRS = cast(ubyte)0x01; /* Interrupt Rising Edge status */
enum FSMC_SR2_ILS = cast(ubyte)0x02; /* Interrupt Level status */
enum FSMC_SR2_IFS = cast(ubyte)0x04; /* Interrupt Falling Edge status */
enum FSMC_SR2_IREN = cast(ubyte)0x08; /* Interrupt Rising Edge detection Enable bit */
enum FSMC_SR2_ILEN = cast(ubyte)0x10; /* Interrupt Level detection Enable bit */
enum FSMC_SR2_IFEN = cast(ubyte)0x20; /* Interrupt Falling Edge detection Enable bit */
enum FSMC_SR2_FEMPT = cast(ubyte)0x40; /* FIFO empty */

/* Bit definition for FSMC_SR3 register */
enum FSMC_SR3_IRS = cast(ubyte)0x01; /* Interrupt Rising Edge status */
enum FSMC_SR3_ILS = cast(ubyte)0x02; /* Interrupt Level status */
enum FSMC_SR3_IFS = cast(ubyte)0x04; /* Interrupt Falling Edge status */
enum FSMC_SR3_IREN = cast(ubyte)0x08; /* Interrupt Rising Edge detection Enable bit */
enum FSMC_SR3_ILEN = cast(ubyte)0x10; /* Interrupt Level detection Enable bit */
enum FSMC_SR3_IFEN = cast(ubyte)0x20; /* Interrupt Falling Edge detection Enable bit */
enum FSMC_SR3_FEMPT = cast(ubyte)0x40; /* FIFO empty */

/* Bit definition for FSMC_SR4 register */
enum FSMC_SR4_IRS = cast(ubyte)0x01; /* Interrupt Rising Edge status */
enum FSMC_SR4_ILS = cast(ubyte)0x02; /* Interrupt Level status */
enum FSMC_SR4_IFS = cast(ubyte)0x04; /* Interrupt Falling Edge status */
enum FSMC_SR4_IREN = cast(ubyte)0x08; /* Interrupt Rising Edge detection Enable bit */
enum FSMC_SR4_ILEN = cast(ubyte)0x10; /* Interrupt Level detection Enable bit */
enum FSMC_SR4_IFEN = cast(ubyte)0x20; /* Interrupt Falling Edge detection Enable bit */
enum FSMC_SR4_FEMPT = cast(ubyte)0x40; /* FIFO empty */

/* Bit definition for FSMC_PMEM2 register */
enum FSMC_PMEM2_MEMSET2 = cast(uint)0x000000FF; /* MEMSET2[7:0] bits (Common memory 2 setup time; */
enum FSMC_PMEM2_MEMSET2_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_PMEM2_MEMSET2_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_PMEM2_MEMSET2_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_PMEM2_MEMSET2_3 = cast(uint)0x00000008; /* Bit 3 */
enum FSMC_PMEM2_MEMSET2_4 = cast(uint)0x00000010; /* Bit 4 */
enum FSMC_PMEM2_MEMSET2_5 = cast(uint)0x00000020; /* Bit 5 */
enum FSMC_PMEM2_MEMSET2_6 = cast(uint)0x00000040; /* Bit 6 */
enum FSMC_PMEM2_MEMSET2_7 = cast(uint)0x00000080; /* Bit 7 */

enum FSMC_PMEM2_MEMWAIT2 = cast(uint)0x0000FF00; /* MEMWAIT2[7:0] bits (Common memory 2 wait time; */
enum FSMC_PMEM2_MEMWAIT2_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_PMEM2_MEMWAIT2_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_PMEM2_MEMWAIT2_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_PMEM2_MEMWAIT2_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_PMEM2_MEMWAIT2_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_PMEM2_MEMWAIT2_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_PMEM2_MEMWAIT2_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_PMEM2_MEMWAIT2_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_PMEM2_MEMHOLD2 = cast(uint)0x00FF0000; /* MEMHOLD2[7:0] bits (Common memory 2 hold time; */
enum FSMC_PMEM2_MEMHOLD2_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_PMEM2_MEMHOLD2_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_PMEM2_MEMHOLD2_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_PMEM2_MEMHOLD2_3 = cast(uint)0x00080000; /* Bit 3 */
enum FSMC_PMEM2_MEMHOLD2_4 = cast(uint)0x00100000; /* Bit 4 */
enum FSMC_PMEM2_MEMHOLD2_5 = cast(uint)0x00200000; /* Bit 5 */
enum FSMC_PMEM2_MEMHOLD2_6 = cast(uint)0x00400000; /* Bit 6 */
enum FSMC_PMEM2_MEMHOLD2_7 = cast(uint)0x00800000; /* Bit 7 */

enum FSMC_PMEM2_MEMHIZ2 = cast(uint)0xFF000000; /* MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time; */
enum FSMC_PMEM2_MEMHIZ2_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_PMEM2_MEMHIZ2_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_PMEM2_MEMHIZ2_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_PMEM2_MEMHIZ2_3 = cast(uint)0x08000000; /* Bit 3 */
enum FSMC_PMEM2_MEMHIZ2_4 = cast(uint)0x10000000; /* Bit 4 */
enum FSMC_PMEM2_MEMHIZ2_5 = cast(uint)0x20000000; /* Bit 5 */
enum FSMC_PMEM2_MEMHIZ2_6 = cast(uint)0x40000000; /* Bit 6 */
enum FSMC_PMEM2_MEMHIZ2_7 = cast(uint)0x80000000; /* Bit 7 */

/* Bit definition for FSMC_PMEM3 register */
enum FSMC_PMEM3_MEMSET3 = cast(uint)0x000000FF; /* MEMSET3[7:0] bits (Common memory 3 setup time; */
enum FSMC_PMEM3_MEMSET3_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_PMEM3_MEMSET3_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_PMEM3_MEMSET3_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_PMEM3_MEMSET3_3 = cast(uint)0x00000008; /* Bit 3 */
enum FSMC_PMEM3_MEMSET3_4 = cast(uint)0x00000010; /* Bit 4 */
enum FSMC_PMEM3_MEMSET3_5 = cast(uint)0x00000020; /* Bit 5 */
enum FSMC_PMEM3_MEMSET3_6 = cast(uint)0x00000040; /* Bit 6 */
enum FSMC_PMEM3_MEMSET3_7 = cast(uint)0x00000080; /* Bit 7 */

enum FSMC_PMEM3_MEMWAIT3 = cast(uint)0x0000FF00; /* MEMWAIT3[7:0] bits (Common memory 3 wait time; */
enum FSMC_PMEM3_MEMWAIT3_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_PMEM3_MEMWAIT3_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_PMEM3_MEMWAIT3_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_PMEM3_MEMWAIT3_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_PMEM3_MEMWAIT3_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_PMEM3_MEMWAIT3_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_PMEM3_MEMWAIT3_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_PMEM3_MEMWAIT3_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_PMEM3_MEMHOLD3 = cast(uint)0x00FF0000; /* MEMHOLD3[7:0] bits (Common memory 3 hold time; */
enum FSMC_PMEM3_MEMHOLD3_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_PMEM3_MEMHOLD3_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_PMEM3_MEMHOLD3_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_PMEM3_MEMHOLD3_3 = cast(uint)0x00080000; /* Bit 3 */
enum FSMC_PMEM3_MEMHOLD3_4 = cast(uint)0x00100000; /* Bit 4 */
enum FSMC_PMEM3_MEMHOLD3_5 = cast(uint)0x00200000; /* Bit 5 */
enum FSMC_PMEM3_MEMHOLD3_6 = cast(uint)0x00400000; /* Bit 6 */
enum FSMC_PMEM3_MEMHOLD3_7 = cast(uint)0x00800000; /* Bit 7 */

enum FSMC_PMEM3_MEMHIZ3 = cast(uint)0xFF000000; /* MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time; */
enum FSMC_PMEM3_MEMHIZ3_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_PMEM3_MEMHIZ3_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_PMEM3_MEMHIZ3_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_PMEM3_MEMHIZ3_3 = cast(uint)0x08000000; /* Bit 3 */
enum FSMC_PMEM3_MEMHIZ3_4 = cast(uint)0x10000000; /* Bit 4 */
enum FSMC_PMEM3_MEMHIZ3_5 = cast(uint)0x20000000; /* Bit 5 */
enum FSMC_PMEM3_MEMHIZ3_6 = cast(uint)0x40000000; /* Bit 6 */
enum FSMC_PMEM3_MEMHIZ3_7 = cast(uint)0x80000000; /* Bit 7 */

/* Bit definition for FSMC_PMEM4 register */
enum FSMC_PMEM4_MEMSET4 = cast(uint)0x000000FF; /* MEMSET4[7:0] bits (Common memory 4 setup time; */
enum FSMC_PMEM4_MEMSET4_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_PMEM4_MEMSET4_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_PMEM4_MEMSET4_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_PMEM4_MEMSET4_3 = cast(uint)0x00000008; /* Bit 3 */
enum FSMC_PMEM4_MEMSET4_4 = cast(uint)0x00000010; /* Bit 4 */
enum FSMC_PMEM4_MEMSET4_5 = cast(uint)0x00000020; /* Bit 5 */
enum FSMC_PMEM4_MEMSET4_6 = cast(uint)0x00000040; /* Bit 6 */
enum FSMC_PMEM4_MEMSET4_7 = cast(uint)0x00000080; /* Bit 7 */

enum FSMC_PMEM4_MEMWAIT4 = cast(uint)0x0000FF00; /* MEMWAIT4[7:0] bits (Common memory 4 wait time; */
enum FSMC_PMEM4_MEMWAIT4_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_PMEM4_MEMWAIT4_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_PMEM4_MEMWAIT4_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_PMEM4_MEMWAIT4_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_PMEM4_MEMWAIT4_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_PMEM4_MEMWAIT4_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_PMEM4_MEMWAIT4_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_PMEM4_MEMWAIT4_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_PMEM4_MEMHOLD4 = cast(uint)0x00FF0000; /* MEMHOLD4[7:0] bits (Common memory 4 hold time; */
enum FSMC_PMEM4_MEMHOLD4_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_PMEM4_MEMHOLD4_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_PMEM4_MEMHOLD4_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_PMEM4_MEMHOLD4_3 = cast(uint)0x00080000; /* Bit 3 */
enum FSMC_PMEM4_MEMHOLD4_4 = cast(uint)0x00100000; /* Bit 4 */
enum FSMC_PMEM4_MEMHOLD4_5 = cast(uint)0x00200000; /* Bit 5 */
enum FSMC_PMEM4_MEMHOLD4_6 = cast(uint)0x00400000; /* Bit 6 */
enum FSMC_PMEM4_MEMHOLD4_7 = cast(uint)0x00800000; /* Bit 7 */

enum FSMC_PMEM4_MEMHIZ4 = cast(uint)0xFF000000; /* MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time; */
enum FSMC_PMEM4_MEMHIZ4_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_PMEM4_MEMHIZ4_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_PMEM4_MEMHIZ4_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_PMEM4_MEMHIZ4_3 = cast(uint)0x08000000; /* Bit 3 */
enum FSMC_PMEM4_MEMHIZ4_4 = cast(uint)0x10000000; /* Bit 4 */
enum FSMC_PMEM4_MEMHIZ4_5 = cast(uint)0x20000000; /* Bit 5 */
enum FSMC_PMEM4_MEMHIZ4_6 = cast(uint)0x40000000; /* Bit 6 */
enum FSMC_PMEM4_MEMHIZ4_7 = cast(uint)0x80000000; /* Bit 7 */

/* Bit definition for FSMC_PATT2 register */
enum FSMC_PATT2_ATTSET2 = cast(uint)0x000000FF; /* ATTSET2[7:0] bits (Attribute memory 2 setup time; */
enum FSMC_PATT2_ATTSET2_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_PATT2_ATTSET2_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_PATT2_ATTSET2_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_PATT2_ATTSET2_3 = cast(uint)0x00000008; /* Bit 3 */
enum FSMC_PATT2_ATTSET2_4 = cast(uint)0x00000010; /* Bit 4 */
enum FSMC_PATT2_ATTSET2_5 = cast(uint)0x00000020; /* Bit 5 */
enum FSMC_PATT2_ATTSET2_6 = cast(uint)0x00000040; /* Bit 6 */
enum FSMC_PATT2_ATTSET2_7 = cast(uint)0x00000080; /* Bit 7 */

enum FSMC_PATT2_ATTWAIT2 = cast(uint)0x0000FF00; /* ATTWAIT2[7:0] bits (Attribute memory 2 wait time; */
enum FSMC_PATT2_ATTWAIT2_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_PATT2_ATTWAIT2_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_PATT2_ATTWAIT2_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_PATT2_ATTWAIT2_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_PATT2_ATTWAIT2_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_PATT2_ATTWAIT2_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_PATT2_ATTWAIT2_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_PATT2_ATTWAIT2_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_PATT2_ATTHOLD2 = cast(uint)0x00FF0000; /* ATTHOLD2[7:0] bits (Attribute memory 2 hold time; */
enum FSMC_PATT2_ATTHOLD2_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_PATT2_ATTHOLD2_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_PATT2_ATTHOLD2_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_PATT2_ATTHOLD2_3 = cast(uint)0x00080000; /* Bit 3 */
enum FSMC_PATT2_ATTHOLD2_4 = cast(uint)0x00100000; /* Bit 4 */
enum FSMC_PATT2_ATTHOLD2_5 = cast(uint)0x00200000; /* Bit 5 */
enum FSMC_PATT2_ATTHOLD2_6 = cast(uint)0x00400000; /* Bit 6 */
enum FSMC_PATT2_ATTHOLD2_7 = cast(uint)0x00800000; /* Bit 7 */

enum FSMC_PATT2_ATTHIZ2 = cast(uint)0xFF000000; /* ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time; */
enum FSMC_PATT2_ATTHIZ2_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_PATT2_ATTHIZ2_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_PATT2_ATTHIZ2_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_PATT2_ATTHIZ2_3 = cast(uint)0x08000000; /* Bit 3 */
enum FSMC_PATT2_ATTHIZ2_4 = cast(uint)0x10000000; /* Bit 4 */
enum FSMC_PATT2_ATTHIZ2_5 = cast(uint)0x20000000; /* Bit 5 */
enum FSMC_PATT2_ATTHIZ2_6 = cast(uint)0x40000000; /* Bit 6 */
enum FSMC_PATT2_ATTHIZ2_7 = cast(uint)0x80000000; /* Bit 7 */

/* Bit definition for FSMC_PATT3 register */
enum FSMC_PATT3_ATTSET3 = cast(uint)0x000000FF; /* ATTSET3[7:0] bits (Attribute memory 3 setup time; */
enum FSMC_PATT3_ATTSET3_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_PATT3_ATTSET3_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_PATT3_ATTSET3_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_PATT3_ATTSET3_3 = cast(uint)0x00000008; /* Bit 3 */
enum FSMC_PATT3_ATTSET3_4 = cast(uint)0x00000010; /* Bit 4 */
enum FSMC_PATT3_ATTSET3_5 = cast(uint)0x00000020; /* Bit 5 */
enum FSMC_PATT3_ATTSET3_6 = cast(uint)0x00000040; /* Bit 6 */
enum FSMC_PATT3_ATTSET3_7 = cast(uint)0x00000080; /* Bit 7 */

enum FSMC_PATT3_ATTWAIT3 = cast(uint)0x0000FF00; /* ATTWAIT3[7:0] bits (Attribute memory 3 wait time; */
enum FSMC_PATT3_ATTWAIT3_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_PATT3_ATTWAIT3_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_PATT3_ATTWAIT3_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_PATT3_ATTWAIT3_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_PATT3_ATTWAIT3_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_PATT3_ATTWAIT3_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_PATT3_ATTWAIT3_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_PATT3_ATTWAIT3_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_PATT3_ATTHOLD3 = cast(uint)0x00FF0000; /* ATTHOLD3[7:0] bits (Attribute memory 3 hold time; */
enum FSMC_PATT3_ATTHOLD3_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_PATT3_ATTHOLD3_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_PATT3_ATTHOLD3_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_PATT3_ATTHOLD3_3 = cast(uint)0x00080000; /* Bit 3 */
enum FSMC_PATT3_ATTHOLD3_4 = cast(uint)0x00100000; /* Bit 4 */
enum FSMC_PATT3_ATTHOLD3_5 = cast(uint)0x00200000; /* Bit 5 */
enum FSMC_PATT3_ATTHOLD3_6 = cast(uint)0x00400000; /* Bit 6 */
enum FSMC_PATT3_ATTHOLD3_7 = cast(uint)0x00800000; /* Bit 7 */

enum FSMC_PATT3_ATTHIZ3 = cast(uint)0xFF000000; /* ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time; */
enum FSMC_PATT3_ATTHIZ3_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_PATT3_ATTHIZ3_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_PATT3_ATTHIZ3_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_PATT3_ATTHIZ3_3 = cast(uint)0x08000000; /* Bit 3 */
enum FSMC_PATT3_ATTHIZ3_4 = cast(uint)0x10000000; /* Bit 4 */
enum FSMC_PATT3_ATTHIZ3_5 = cast(uint)0x20000000; /* Bit 5 */
enum FSMC_PATT3_ATTHIZ3_6 = cast(uint)0x40000000; /* Bit 6 */
enum FSMC_PATT3_ATTHIZ3_7 = cast(uint)0x80000000; /* Bit 7 */

/* Bit definition for FSMC_PATT4 register */
enum FSMC_PATT4_ATTSET4 = cast(uint)0x000000FF; /* ATTSET4[7:0] bits (Attribute memory 4 setup time; */
enum FSMC_PATT4_ATTSET4_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_PATT4_ATTSET4_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_PATT4_ATTSET4_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_PATT4_ATTSET4_3 = cast(uint)0x00000008; /* Bit 3 */
enum FSMC_PATT4_ATTSET4_4 = cast(uint)0x00000010; /* Bit 4 */
enum FSMC_PATT4_ATTSET4_5 = cast(uint)0x00000020; /* Bit 5 */
enum FSMC_PATT4_ATTSET4_6 = cast(uint)0x00000040; /* Bit 6 */
enum FSMC_PATT4_ATTSET4_7 = cast(uint)0x00000080; /* Bit 7 */

enum FSMC_PATT4_ATTWAIT4 = cast(uint)0x0000FF00; /* ATTWAIT4[7:0] bits (Attribute memory 4 wait time; */
enum FSMC_PATT4_ATTWAIT4_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_PATT4_ATTWAIT4_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_PATT4_ATTWAIT4_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_PATT4_ATTWAIT4_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_PATT4_ATTWAIT4_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_PATT4_ATTWAIT4_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_PATT4_ATTWAIT4_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_PATT4_ATTWAIT4_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_PATT4_ATTHOLD4 = cast(uint)0x00FF0000; /* ATTHOLD4[7:0] bits (Attribute memory 4 hold time; */
enum FSMC_PATT4_ATTHOLD4_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_PATT4_ATTHOLD4_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_PATT4_ATTHOLD4_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_PATT4_ATTHOLD4_3 = cast(uint)0x00080000; /* Bit 3 */
enum FSMC_PATT4_ATTHOLD4_4 = cast(uint)0x00100000; /* Bit 4 */
enum FSMC_PATT4_ATTHOLD4_5 = cast(uint)0x00200000; /* Bit 5 */
enum FSMC_PATT4_ATTHOLD4_6 = cast(uint)0x00400000; /* Bit 6 */
enum FSMC_PATT4_ATTHOLD4_7 = cast(uint)0x00800000; /* Bit 7 */

enum FSMC_PATT4_ATTHIZ4 = cast(uint)0xFF000000; /* ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time; */
enum FSMC_PATT4_ATTHIZ4_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_PATT4_ATTHIZ4_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_PATT4_ATTHIZ4_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_PATT4_ATTHIZ4_3 = cast(uint)0x08000000; /* Bit 3 */
enum FSMC_PATT4_ATTHIZ4_4 = cast(uint)0x10000000; /* Bit 4 */
enum FSMC_PATT4_ATTHIZ4_5 = cast(uint)0x20000000; /* Bit 5 */
enum FSMC_PATT4_ATTHIZ4_6 = cast(uint)0x40000000; /* Bit 6 */
enum FSMC_PATT4_ATTHIZ4_7 = cast(uint)0x80000000; /* Bit 7 */

/* Bit definition for FSMC_PIO4 register */
enum FSMC_PIO4_IOSET4 = cast(uint)0x000000FF; /* IOSET4[7:0] bits (I/O 4 setup time; */
enum FSMC_PIO4_IOSET4_0 = cast(uint)0x00000001; /* Bit 0 */
enum FSMC_PIO4_IOSET4_1 = cast(uint)0x00000002; /* Bit 1 */
enum FSMC_PIO4_IOSET4_2 = cast(uint)0x00000004; /* Bit 2 */
enum FSMC_PIO4_IOSET4_3 = cast(uint)0x00000008; /* Bit 3 */
enum FSMC_PIO4_IOSET4_4 = cast(uint)0x00000010; /* Bit 4 */
enum FSMC_PIO4_IOSET4_5 = cast(uint)0x00000020; /* Bit 5 */
enum FSMC_PIO4_IOSET4_6 = cast(uint)0x00000040; /* Bit 6 */
enum FSMC_PIO4_IOSET4_7 = cast(uint)0x00000080; /* Bit 7 */

enum FSMC_PIO4_IOWAIT4 = cast(uint)0x0000FF00; /* IOWAIT4[7:0] bits (I/O 4 wait time; */
enum FSMC_PIO4_IOWAIT4_0 = cast(uint)0x00000100; /* Bit 0 */
enum FSMC_PIO4_IOWAIT4_1 = cast(uint)0x00000200; /* Bit 1 */
enum FSMC_PIO4_IOWAIT4_2 = cast(uint)0x00000400; /* Bit 2 */
enum FSMC_PIO4_IOWAIT4_3 = cast(uint)0x00000800; /* Bit 3 */
enum FSMC_PIO4_IOWAIT4_4 = cast(uint)0x00001000; /* Bit 4 */
enum FSMC_PIO4_IOWAIT4_5 = cast(uint)0x00002000; /* Bit 5 */
enum FSMC_PIO4_IOWAIT4_6 = cast(uint)0x00004000; /* Bit 6 */
enum FSMC_PIO4_IOWAIT4_7 = cast(uint)0x00008000; /* Bit 7 */

enum FSMC_PIO4_IOHOLD4 = cast(uint)0x00FF0000; /* IOHOLD4[7:0] bits (I/O 4 hold time; */
enum FSMC_PIO4_IOHOLD4_0 = cast(uint)0x00010000; /* Bit 0 */
enum FSMC_PIO4_IOHOLD4_1 = cast(uint)0x00020000; /* Bit 1 */
enum FSMC_PIO4_IOHOLD4_2 = cast(uint)0x00040000; /* Bit 2 */
enum FSMC_PIO4_IOHOLD4_3 = cast(uint)0x00080000; /* Bit 3 */
enum FSMC_PIO4_IOHOLD4_4 = cast(uint)0x00100000; /* Bit 4 */
enum FSMC_PIO4_IOHOLD4_5 = cast(uint)0x00200000; /* Bit 5 */
enum FSMC_PIO4_IOHOLD4_6 = cast(uint)0x00400000; /* Bit 6 */
enum FSMC_PIO4_IOHOLD4_7 = cast(uint)0x00800000; /* Bit 7 */

enum FSMC_PIO4_IOHIZ4 = cast(uint)0xFF000000; /* IOHIZ4[7:0] bits (I/O 4 databus HiZ time; */
enum FSMC_PIO4_IOHIZ4_0 = cast(uint)0x01000000; /* Bit 0 */
enum FSMC_PIO4_IOHIZ4_1 = cast(uint)0x02000000; /* Bit 1 */
enum FSMC_PIO4_IOHIZ4_2 = cast(uint)0x04000000; /* Bit 2 */
enum FSMC_PIO4_IOHIZ4_3 = cast(uint)0x08000000; /* Bit 3 */
enum FSMC_PIO4_IOHIZ4_4 = cast(uint)0x10000000; /* Bit 4 */
enum FSMC_PIO4_IOHIZ4_5 = cast(uint)0x20000000; /* Bit 5 */
enum FSMC_PIO4_IOHIZ4_6 = cast(uint)0x40000000; /* Bit 6 */
enum FSMC_PIO4_IOHIZ4_7 = cast(uint)0x80000000; /* Bit 7 */

/* Bit definition for FSMC_ECCR2 register */
enum FSMC_ECCR2_ECC2 = cast(uint)0xFFFFFFFF; /* ECC result */

/* Bit definition for FSMC_ECCR3 register */
enum FSMC_ECCR3_ECC3 = cast(uint)0xFFFFFFFF; /* ECC result */

/* Bit definition for SDIO_POWER register */
enum SDIO_POWER_PWRCTRL = cast(ubyte)0x03; /* PWRCTRL[1:0] bits (Power supply control bits; */
enum SDIO_POWER_PWRCTRL_0 = cast(ubyte)0x01; /* Bit 0 */
enum SDIO_POWER_PWRCTRL_1 = cast(ubyte)0x02; /* Bit 1 */

/* Bit definition for SDIO_CLKCR register */
enum SDIO_CLKCR_CLKDIV = cast(ushort)0x00FF; /* Clock divide factor */
enum SDIO_CLKCR_CLKEN = cast(ushort)0x0100; /* Clock enable bit */
enum SDIO_CLKCR_PWRSAV = cast(ushort)0x0200; /* Power saving configuration bit */
enum SDIO_CLKCR_BYPASS = cast(ushort)0x0400; /* Clock divider bypass enable bit */

enum SDIO_CLKCR_WIDBUS = cast(ushort)0x1800; /* WIDBUS[1:0] bits (Wide bus mode enable bit; */
enum SDIO_CLKCR_WIDBUS_0 = cast(ushort)0x0800; /* Bit 0 */
enum SDIO_CLKCR_WIDBUS_1 = cast(ushort)0x1000; /* Bit 1 */

enum SDIO_CLKCR_NEGEDGE = cast(ushort)0x2000; /* SDIO_CK dephasing selection bit */
enum SDIO_CLKCR_HWFC_EN = cast(ushort)0x4000; /* HW Flow Control enable */

/* Bit definition for SDIO_ARG register */
enum SDIO_ARG_CMDARG = cast(uint)0xFFFFFFFF; /* Command argument */

/* Bit definition for SDIO_CMD register */
enum SDIO_CMD_CMDINDEX = cast(ushort)0x003F; /* Command Index */

enum SDIO_CMD_WAITRESP = cast(ushort)0x00C0; /* WAITRESP[1:0] bits (Wait for response bits; */
enum SDIO_CMD_WAITRESP_0 = cast(ushort)0x0040; /* Bit 0 */
enum SDIO_CMD_WAITRESP_1 = cast(ushort)0x0080; /* Bit 1 */

enum SDIO_CMD_WAITINT = cast(ushort)0x0100; /* CPSM Waits for Interrupt Request */
enum SDIO_CMD_WAITPEND = cast(ushort)0x0200; /* CPSM Waits for ends of data transfer (CmdPend internal signal; */
enum SDIO_CMD_CPSMEN = cast(ushort)0x0400; /* Command path state machine (CPSM; Enable bit */
enum SDIO_CMD_SDIOSUSPEND = cast(ushort)0x0800; /* SD I/O suspend command */
enum SDIO_CMD_ENCMDCOMPL = cast(ushort)0x1000; /* Enable CMD completion */
enum SDIO_CMD_NIEN = cast(ushort)0x2000; /* Not Interrupt Enable */
enum SDIO_CMD_CEATACMD = cast(ushort)0x4000; /* CE-ATA command */

/* Bit definition for SDIO_RESPCMD register */
enum SDIO_RESPCMD_RESPCMD = cast(ubyte)0x3F; /* Response command index */

/* Bit definition for SDIO_RESP0 register */
enum SDIO_RESP0_CARDSTATUS0 = cast(uint)0xFFFFFFFF; /* Card Status */

/* Bit definition for SDIO_RESP1 register */
enum SDIO_RESP1_CARDSTATUS1 = cast(uint)0xFFFFFFFF; /* Card Status */

/* Bit definition for SDIO_RESP2 register */
enum SDIO_RESP2_CARDSTATUS2 = cast(uint)0xFFFFFFFF; /* Card Status */

/* Bit definition for SDIO_RESP3 register */
enum SDIO_RESP3_CARDSTATUS3 = cast(uint)0xFFFFFFFF; /* Card Status */

/* Bit definition for SDIO_RESP4 register */
enum SDIO_RESP4_CARDSTATUS4 = cast(uint)0xFFFFFFFF; /* Card Status */

/* Bit definition for SDIO_DTIMER register */
enum SDIO_DTIMER_DATATIME = cast(uint)0xFFFFFFFF; /* Data timeout period. */

/* Bit definition for SDIO_DLEN register */
enum SDIO_DLEN_DATALENGTH = cast(uint)0x01FFFFFF; /* Data length value */

/* Bit definition for SDIO_DCTRL register */
enum SDIO_DCTRL_DTEN = cast(ushort)0x0001; /* Data transfer enabled bit */
enum SDIO_DCTRL_DTDIR = cast(ushort)0x0002; /* Data transfer direction selection */
enum SDIO_DCTRL_DTMODE = cast(ushort)0x0004; /* Data transfer mode selection */
enum SDIO_DCTRL_DMAEN = cast(ushort)0x0008; /* DMA enabled bit */

enum SDIO_DCTRL_DBLOCKSIZE = cast(ushort)0x00F0; /* DBLOCKSIZE[3:0] bits (Data block size; */
enum SDIO_DCTRL_DBLOCKSIZE_0 = cast(ushort)0x0010; /* Bit 0 */
enum SDIO_DCTRL_DBLOCKSIZE_1 = cast(ushort)0x0020; /* Bit 1 */
enum SDIO_DCTRL_DBLOCKSIZE_2 = cast(ushort)0x0040; /* Bit 2 */
enum SDIO_DCTRL_DBLOCKSIZE_3 = cast(ushort)0x0080; /* Bit 3 */

enum SDIO_DCTRL_RWSTART = cast(ushort)0x0100; /* Read wait start */
enum SDIO_DCTRL_RWSTOP = cast(ushort)0x0200; /* Read wait stop */
enum SDIO_DCTRL_RWMOD = cast(ushort)0x0400; /* Read wait mode */
enum SDIO_DCTRL_SDIOEN = cast(ushort)0x0800; /* SD I/O enable functions */

/* Bit definition for SDIO_DCOUNT register */
enum SDIO_DCOUNT_DATACOUNT = cast(uint)0x01FFFFFF; /* Data count value */

/* Bit definition for SDIO_STA register */
enum SDIO_STA_CCRCFAIL = cast(uint)0x00000001; /* Command response received (CRC check failed; */
enum SDIO_STA_DCRCFAIL = cast(uint)0x00000002; /* Data block sent/received (CRC check failed; */
enum SDIO_STA_CTIMEOUT = cast(uint)0x00000004; /* Command response timeout */
enum SDIO_STA_DTIMEOUT = cast(uint)0x00000008; /* Data timeout */
enum SDIO_STA_TXUNDERR = cast(uint)0x00000010; /* Transmit FIFO underrun error */
enum SDIO_STA_RXOVERR = cast(uint)0x00000020; /* Received FIFO overrun error */
enum SDIO_STA_CMDREND = cast(uint)0x00000040; /* Command response received (CRC check passed; */
enum SDIO_STA_CMDSENT = cast(uint)0x00000080; /* Command sent (no response required; */
enum SDIO_STA_DATAEND = cast(uint)0x00000100; /* Data end (data counter, SDIDCOUNT, is zero; */
enum SDIO_STA_STBITERR = cast(uint)0x00000200; /* Start bit not detected on all data signals in wide bus mode */
enum SDIO_STA_DBCKEND = cast(uint)0x00000400; /* Data block sent/received (CRC check passed; */
enum SDIO_STA_CMDACT = cast(uint)0x00000800; /* Command transfer in progress */
enum SDIO_STA_TXACT = cast(uint)0x00001000; /* Data transmit in progress */
enum SDIO_STA_RXACT = cast(uint)0x00002000; /* Data receive in progress */
enum SDIO_STA_TXFIFOHE = cast(uint)0x00004000; /* Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
enum SDIO_STA_RXFIFOHF = cast(uint)0x00008000; /* Receive FIFO Half Full: there are at least 8 words in the FIFO */
enum SDIO_STA_TXFIFOF = cast(uint)0x00010000; /* Transmit FIFO full */
enum SDIO_STA_RXFIFOF = cast(uint)0x00020000; /* Receive FIFO full */
enum SDIO_STA_TXFIFOE = cast(uint)0x00040000; /* Transmit FIFO empty */
enum SDIO_STA_RXFIFOE = cast(uint)0x00080000; /* Receive FIFO empty */
enum SDIO_STA_TXDAVL = cast(uint)0x00100000; /* Data available in transmit FIFO */
enum SDIO_STA_RXDAVL = cast(uint)0x00200000; /* Data available in receive FIFO */
enum SDIO_STA_SDIOIT = cast(uint)0x00400000; /* SDIO interrupt received */
enum SDIO_STA_CEATAEND = cast(uint)0x00800000; /* CE-ATA command completion signal received for CMD61 */

/* Bit definition for SDIO_ICR register */
enum SDIO_ICR_CCRCFAILC = cast(uint)0x00000001; /* CCRCFAIL flag clear bit */
enum SDIO_ICR_DCRCFAILC = cast(uint)0x00000002; /* DCRCFAIL flag clear bit */
enum SDIO_ICR_CTIMEOUTC = cast(uint)0x00000004; /* CTIMEOUT flag clear bit */
enum SDIO_ICR_DTIMEOUTC = cast(uint)0x00000008; /* DTIMEOUT flag clear bit */
enum SDIO_ICR_TXUNDERRC = cast(uint)0x00000010; /* TXUNDERR flag clear bit */
enum SDIO_ICR_RXOVERRC = cast(uint)0x00000020; /* RXOVERR flag clear bit */
enum SDIO_ICR_CMDRENDC = cast(uint)0x00000040; /* CMDREND flag clear bit */
enum SDIO_ICR_CMDSENTC = cast(uint)0x00000080; /* CMDSENT flag clear bit */
enum SDIO_ICR_DATAENDC = cast(uint)0x00000100; /* DATAEND flag clear bit */
enum SDIO_ICR_STBITERRC = cast(uint)0x00000200; /* STBITERR flag clear bit */
enum SDIO_ICR_DBCKENDC = cast(uint)0x00000400; /* DBCKEND flag clear bit */
enum SDIO_ICR_SDIOITC = cast(uint)0x00400000; /* SDIOIT flag clear bit */
enum SDIO_ICR_CEATAENDC = cast(uint)0x00800000; /* CEATAEND flag clear bit */

/* Bit definition for SDIO_MASK register */
enum SDIO_MASK_CCRCFAILIE = cast(uint)0x00000001; /* Command CRC Fail Interrupt Enable */
enum SDIO_MASK_DCRCFAILIE = cast(uint)0x00000002; /* Data CRC Fail Interrupt Enable */
enum SDIO_MASK_CTIMEOUTIE = cast(uint)0x00000004; /* Command TimeOut Interrupt Enable */
enum SDIO_MASK_DTIMEOUTIE = cast(uint)0x00000008; /* Data TimeOut Interrupt Enable */
enum SDIO_MASK_TXUNDERRIE = cast(uint)0x00000010; /* Tx FIFO UnderRun Error Interrupt Enable */
enum SDIO_MASK_RXOVERRIE = cast(uint)0x00000020; /* Rx FIFO OverRun Error Interrupt Enable */
enum SDIO_MASK_CMDRENDIE = cast(uint)0x00000040; /* Command Response Received Interrupt Enable */
enum SDIO_MASK_CMDSENTIE = cast(uint)0x00000080; /* Command Sent Interrupt Enable */
enum SDIO_MASK_DATAENDIE = cast(uint)0x00000100; /* Data End Interrupt Enable */
enum SDIO_MASK_STBITERRIE = cast(uint)0x00000200; /* Start Bit Error Interrupt Enable */
enum SDIO_MASK_DBCKENDIE = cast(uint)0x00000400; /* Data Block End Interrupt Enable */
enum SDIO_MASK_CMDACTIE = cast(uint)0x00000800; /* Command Acting Interrupt Enable */
enum SDIO_MASK_TXACTIE = cast(uint)0x00001000; /* Data Transmit Acting Interrupt Enable */
enum SDIO_MASK_RXACTIE = cast(uint)0x00002000; /* Data receive acting interrupt enabled */
enum SDIO_MASK_TXFIFOHEIE = cast(uint)0x00004000; /* Tx FIFO Half Empty interrupt Enable */
enum SDIO_MASK_RXFIFOHFIE = cast(uint)0x00008000; /* Rx FIFO Half Full interrupt Enable */
enum SDIO_MASK_TXFIFOFIE = cast(uint)0x00010000; /* Tx FIFO Full interrupt Enable */
enum SDIO_MASK_RXFIFOFIE = cast(uint)0x00020000; /* Rx FIFO Full interrupt Enable */
enum SDIO_MASK_TXFIFOEIE = cast(uint)0x00040000; /* Tx FIFO Empty interrupt Enable */
enum SDIO_MASK_RXFIFOEIE = cast(uint)0x00080000; /* Rx FIFO Empty interrupt Enable */
enum SDIO_MASK_TXDAVLIE = cast(uint)0x00100000; /* Data available in Tx FIFO interrupt Enable */
enum SDIO_MASK_RXDAVLIE = cast(uint)0x00200000; /* Data available in Rx FIFO interrupt Enable */
enum SDIO_MASK_SDIOITIE = cast(uint)0x00400000; /* SDIO Mode Interrupt Received interrupt Enable */
enum SDIO_MASK_CEATAENDIE = cast(uint)0x00800000; /* CE-ATA command completion signal received Interrupt Enable */

/* Bit definition for SDIO_FIFOCNT register */
enum SDIO_FIFOCNT_FIFOCOUNT = cast(uint)0x00FFFFFF; /* Remaining number of words to be written to or read from the FIFO */

/* Bit definition for SDIO_FIFO register */
enum SDIO_FIFO_FIFODATA = cast(uint)0xFFFFFFFF; /* Receive and transmit FIFO data */

/* Endpoint-specific registers */
/* Bit definition for USB_EP0R register */
enum USB_EP0R_EA = cast(ushort)0x000F; /* Endpoint Address */

enum USB_EP0R_STAT_TX = cast(ushort)0x0030; /* STAT_TX[1:0] bits (Status bits, for transmission transfers; */
enum USB_EP0R_STAT_TX_0 = cast(ushort)0x0010; /* Bit 0 */
enum USB_EP0R_STAT_TX_1 = cast(ushort)0x0020; /* Bit 1 */

enum USB_EP0R_DTOG_TX = cast(ushort)0x0040; /* Data Toggle, for transmission transfers */
enum USB_EP0R_CTR_TX = cast(ushort)0x0080; /* Correct Transfer for transmission */
enum USB_EP0R_EP_KIND = cast(ushort)0x0100; /* Endpoint Kind */

enum USB_EP0R_EP_TYPE = cast(ushort)0x0600; /* EP_TYPE[1:0] bits (Endpoint type; */
enum USB_EP0R_EP_TYPE_0 = cast(ushort)0x0200; /* Bit 0 */
enum USB_EP0R_EP_TYPE_1 = cast(ushort)0x0400; /* Bit 1 */

enum USB_EP0R_SETUP = cast(ushort)0x0800; /* Setup transaction completed */

enum USB_EP0R_STAT_RX = cast(ushort)0x3000; /* STAT_RX[1:0] bits (Status bits, for reception transfers; */
enum USB_EP0R_STAT_RX_0 = cast(ushort)0x1000; /* Bit 0 */
enum USB_EP0R_STAT_RX_1 = cast(ushort)0x2000; /* Bit 1 */

enum USB_EP0R_DTOG_RX = cast(ushort)0x4000; /* Data Toggle, for reception transfers */
enum USB_EP0R_CTR_RX = cast(ushort)0x8000; /* Correct Transfer for reception */

/* Bit definition for USB_EP1R register */
enum USB_EP1R_EA = cast(ushort)0x000F; /* Endpoint Address */

enum USB_EP1R_STAT_TX = cast(ushort)0x0030; /* STAT_TX[1:0] bits (Status bits, for transmission transfers; */
enum USB_EP1R_STAT_TX_0 = cast(ushort)0x0010; /* Bit 0 */
enum USB_EP1R_STAT_TX_1 = cast(ushort)0x0020; /* Bit 1 */

enum USB_EP1R_DTOG_TX = cast(ushort)0x0040; /* Data Toggle, for transmission transfers */
enum USB_EP1R_CTR_TX = cast(ushort)0x0080; /* Correct Transfer for transmission */
enum USB_EP1R_EP_KIND = cast(ushort)0x0100; /* Endpoint Kind */

enum USB_EP1R_EP_TYPE = cast(ushort)0x0600; /* EP_TYPE[1:0] bits (Endpoint type; */
enum USB_EP1R_EP_TYPE_0 = cast(ushort)0x0200; /* Bit 0 */
enum USB_EP1R_EP_TYPE_1 = cast(ushort)0x0400; /* Bit 1 */

enum USB_EP1R_SETUP = cast(ushort)0x0800; /* Setup transaction completed */

enum USB_EP1R_STAT_RX = cast(ushort)0x3000; /* STAT_RX[1:0] bits (Status bits, for reception transfers; */
enum USB_EP1R_STAT_RX_0 = cast(ushort)0x1000; /* Bit 0 */
enum USB_EP1R_STAT_RX_1 = cast(ushort)0x2000; /* Bit 1 */

enum USB_EP1R_DTOG_RX = cast(ushort)0x4000; /* Data Toggle, for reception transfers */
enum USB_EP1R_CTR_RX = cast(ushort)0x8000; /* Correct Transfer for reception */

/* Bit definition for USB_EP2R register */
enum USB_EP2R_EA = cast(ushort)0x000F; /* Endpoint Address */

enum USB_EP2R_STAT_TX = cast(ushort)0x0030; /* STAT_TX[1:0] bits (Status bits, for transmission transfers; */
enum USB_EP2R_STAT_TX_0 = cast(ushort)0x0010; /* Bit 0 */
enum USB_EP2R_STAT_TX_1 = cast(ushort)0x0020; /* Bit 1 */

enum USB_EP2R_DTOG_TX = cast(ushort)0x0040; /* Data Toggle, for transmission transfers */
enum USB_EP2R_CTR_TX = cast(ushort)0x0080; /* Correct Transfer for transmission */
enum USB_EP2R_EP_KIND = cast(ushort)0x0100; /* Endpoint Kind */

enum USB_EP2R_EP_TYPE = cast(ushort)0x0600; /* EP_TYPE[1:0] bits (Endpoint type; */
enum USB_EP2R_EP_TYPE_0 = cast(ushort)0x0200; /* Bit 0 */
enum USB_EP2R_EP_TYPE_1 = cast(ushort)0x0400; /* Bit 1 */

enum USB_EP2R_SETUP = cast(ushort)0x0800; /* Setup transaction completed */

enum USB_EP2R_STAT_RX = cast(ushort)0x3000; /* STAT_RX[1:0] bits (Status bits, for reception transfers; */
enum USB_EP2R_STAT_RX_0 = cast(ushort)0x1000; /* Bit 0 */
enum USB_EP2R_STAT_RX_1 = cast(ushort)0x2000; /* Bit 1 */

enum USB_EP2R_DTOG_RX = cast(ushort)0x4000; /* Data Toggle, for reception transfers */
enum USB_EP2R_CTR_RX = cast(ushort)0x8000; /* Correct Transfer for reception */

/* Bit definition for USB_EP3R register */
enum USB_EP3R_EA = cast(ushort)0x000F; /* Endpoint Address */

enum USB_EP3R_STAT_TX = cast(ushort)0x0030; /* STAT_TX[1:0] bits (Status bits, for transmission transfers; */
enum USB_EP3R_STAT_TX_0 = cast(ushort)0x0010; /* Bit 0 */
enum USB_EP3R_STAT_TX_1 = cast(ushort)0x0020; /* Bit 1 */

enum USB_EP3R_DTOG_TX = cast(ushort)0x0040; /* Data Toggle, for transmission transfers */
enum USB_EP3R_CTR_TX = cast(ushort)0x0080; /* Correct Transfer for transmission */
enum USB_EP3R_EP_KIND = cast(ushort)0x0100; /* Endpoint Kind */

enum USB_EP3R_EP_TYPE = cast(ushort)0x0600; /* EP_TYPE[1:0] bits (Endpoint type; */
enum USB_EP3R_EP_TYPE_0 = cast(ushort)0x0200; /* Bit 0 */
enum USB_EP3R_EP_TYPE_1 = cast(ushort)0x0400; /* Bit 1 */

enum USB_EP3R_SETUP = cast(ushort)0x0800; /* Setup transaction completed */

enum USB_EP3R_STAT_RX = cast(ushort)0x3000; /* STAT_RX[1:0] bits (Status bits, for reception transfers; */
enum USB_EP3R_STAT_RX_0 = cast(ushort)0x1000; /* Bit 0 */
enum USB_EP3R_STAT_RX_1 = cast(ushort)0x2000; /* Bit 1 */

enum USB_EP3R_DTOG_RX = cast(ushort)0x4000; /* Data Toggle, for reception transfers */
enum USB_EP3R_CTR_RX = cast(ushort)0x8000; /* Correct Transfer for reception */

/* Bit definition for USB_EP4R register */
enum USB_EP4R_EA = cast(ushort)0x000F; /* Endpoint Address */

enum USB_EP4R_STAT_TX = cast(ushort)0x0030; /* STAT_TX[1:0] bits (Status bits, for transmission transfers; */
enum USB_EP4R_STAT_TX_0 = cast(ushort)0x0010; /* Bit 0 */
enum USB_EP4R_STAT_TX_1 = cast(ushort)0x0020; /* Bit 1 */

enum USB_EP4R_DTOG_TX = cast(ushort)0x0040; /* Data Toggle, for transmission transfers */
enum USB_EP4R_CTR_TX = cast(ushort)0x0080; /* Correct Transfer for transmission */
enum USB_EP4R_EP_KIND = cast(ushort)0x0100; /* Endpoint Kind */

enum USB_EP4R_EP_TYPE = cast(ushort)0x0600; /* EP_TYPE[1:0] bits (Endpoint type; */
enum USB_EP4R_EP_TYPE_0 = cast(ushort)0x0200; /* Bit 0 */
enum USB_EP4R_EP_TYPE_1 = cast(ushort)0x0400; /* Bit 1 */

enum USB_EP4R_SETUP = cast(ushort)0x0800; /* Setup transaction completed */

enum USB_EP4R_STAT_RX = cast(ushort)0x3000; /* STAT_RX[1:0] bits (Status bits, for reception transfers; */
enum USB_EP4R_STAT_RX_0 = cast(ushort)0x1000; /* Bit 0 */
enum USB_EP4R_STAT_RX_1 = cast(ushort)0x2000; /* Bit 1 */

enum USB_EP4R_DTOG_RX = cast(ushort)0x4000; /* Data Toggle, for reception transfers */
enum USB_EP4R_CTR_RX = cast(ushort)0x8000; /* Correct Transfer for reception */

/* Bit definition for USB_EP5R register */
enum USB_EP5R_EA = cast(ushort)0x000F; /* Endpoint Address */

enum USB_EP5R_STAT_TX = cast(ushort)0x0030; /* STAT_TX[1:0] bits (Status bits, for transmission transfers; */
enum USB_EP5R_STAT_TX_0 = cast(ushort)0x0010; /* Bit 0 */
enum USB_EP5R_STAT_TX_1 = cast(ushort)0x0020; /* Bit 1 */

enum USB_EP5R_DTOG_TX = cast(ushort)0x0040; /* Data Toggle, for transmission transfers */
enum USB_EP5R_CTR_TX = cast(ushort)0x0080; /* Correct Transfer for transmission */
enum USB_EP5R_EP_KIND = cast(ushort)0x0100; /* Endpoint Kind */

enum USB_EP5R_EP_TYPE = cast(ushort)0x0600; /* EP_TYPE[1:0] bits (Endpoint type; */
enum USB_EP5R_EP_TYPE_0 = cast(ushort)0x0200; /* Bit 0 */
enum USB_EP5R_EP_TYPE_1 = cast(ushort)0x0400; /* Bit 1 */

enum USB_EP5R_SETUP = cast(ushort)0x0800; /* Setup transaction completed */

enum USB_EP5R_STAT_RX = cast(ushort)0x3000; /* STAT_RX[1:0] bits (Status bits, for reception transfers; */
enum USB_EP5R_STAT_RX_0 = cast(ushort)0x1000; /* Bit 0 */
enum USB_EP5R_STAT_RX_1 = cast(ushort)0x2000; /* Bit 1 */

enum USB_EP5R_DTOG_RX = cast(ushort)0x4000; /* Data Toggle, for reception transfers */
enum USB_EP5R_CTR_RX = cast(ushort)0x8000; /* Correct Transfer for reception */

/* Bit definition for USB_EP6R register */
enum USB_EP6R_EA = cast(ushort)0x000F; /* Endpoint Address */

enum USB_EP6R_STAT_TX = cast(ushort)0x0030; /* STAT_TX[1:0] bits (Status bits, for transmission transfers; */
enum USB_EP6R_STAT_TX_0 = cast(ushort)0x0010; /* Bit 0 */
enum USB_EP6R_STAT_TX_1 = cast(ushort)0x0020; /* Bit 1 */

enum USB_EP6R_DTOG_TX = cast(ushort)0x0040; /* Data Toggle, for transmission transfers */
enum USB_EP6R_CTR_TX = cast(ushort)0x0080; /* Correct Transfer for transmission */
enum USB_EP6R_EP_KIND = cast(ushort)0x0100; /* Endpoint Kind */

enum USB_EP6R_EP_TYPE = cast(ushort)0x0600; /* EP_TYPE[1:0] bits (Endpoint type; */
enum USB_EP6R_EP_TYPE_0 = cast(ushort)0x0200; /* Bit 0 */
enum USB_EP6R_EP_TYPE_1 = cast(ushort)0x0400; /* Bit 1 */

enum USB_EP6R_SETUP = cast(ushort)0x0800; /* Setup transaction completed */

enum USB_EP6R_STAT_RX = cast(ushort)0x3000; /* STAT_RX[1:0] bits (Status bits, for reception transfers; */
enum USB_EP6R_STAT_RX_0 = cast(ushort)0x1000; /* Bit 0 */
enum USB_EP6R_STAT_RX_1 = cast(ushort)0x2000; /* Bit 1 */

enum USB_EP6R_DTOG_RX = cast(ushort)0x4000; /* Data Toggle, for reception transfers */
enum USB_EP6R_CTR_RX = cast(ushort)0x8000; /* Correct Transfer for reception */

/* Bit definition for USB_EP7R register */
enum USB_EP7R_EA = cast(ushort)0x000F; /* Endpoint Address */

enum USB_EP7R_STAT_TX = cast(ushort)0x0030; /* STAT_TX[1:0] bits (Status bits, for transmission transfers; */
enum USB_EP7R_STAT_TX_0 = cast(ushort)0x0010; /* Bit 0 */
enum USB_EP7R_STAT_TX_1 = cast(ushort)0x0020; /* Bit 1 */

enum USB_EP7R_DTOG_TX = cast(ushort)0x0040; /* Data Toggle, for transmission transfers */
enum USB_EP7R_CTR_TX = cast(ushort)0x0080; /* Correct Transfer for transmission */
enum USB_EP7R_EP_KIND = cast(ushort)0x0100; /* Endpoint Kind */

enum USB_EP7R_EP_TYPE = cast(ushort)0x0600; /* EP_TYPE[1:0] bits (Endpoint type; */
enum USB_EP7R_EP_TYPE_0 = cast(ushort)0x0200; /* Bit 0 */
enum USB_EP7R_EP_TYPE_1 = cast(ushort)0x0400; /* Bit 1 */

enum USB_EP7R_SETUP = cast(ushort)0x0800; /* Setup transaction completed */

enum USB_EP7R_STAT_RX = cast(ushort)0x3000; /* STAT_RX[1:0] bits (Status bits, for reception transfers; */
enum USB_EP7R_STAT_RX_0 = cast(ushort)0x1000; /* Bit 0 */
enum USB_EP7R_STAT_RX_1 = cast(ushort)0x2000; /* Bit 1 */

enum USB_EP7R_DTOG_RX = cast(ushort)0x4000; /* Data Toggle, for reception transfers */
enum USB_EP7R_CTR_RX = cast(ushort)0x8000; /* Correct Transfer for reception */

/* Common registers */
/* Bit definition for USB_CNTR register */
enum USB_CNTR_FRES = cast(ushort)0x0001; /* Force USB Reset */
enum USB_CNTR_PDWN = cast(ushort)0x0002; /* Power down */
enum USB_CNTR_LP_MODE = cast(ushort)0x0004; /* Low-power mode */
enum USB_CNTR_FSUSP = cast(ushort)0x0008; /* Force suspend */
enum USB_CNTR_RESUME = cast(ushort)0x0010; /* Resume request */
enum USB_CNTR_ESOFM = cast(ushort)0x0100; /* Expected Start Of Frame Interrupt Mask */
enum USB_CNTR_SOFM = cast(ushort)0x0200; /* Start Of Frame Interrupt Mask */
enum USB_CNTR_RESETM = cast(ushort)0x0400; /* RESET Interrupt Mask */
enum USB_CNTR_SUSPM = cast(ushort)0x0800; /* Suspend mode Interrupt Mask */
enum USB_CNTR_WKUPM = cast(ushort)0x1000; /* Wakeup Interrupt Mask */
enum USB_CNTR_ERRM = cast(ushort)0x2000; /* Error Interrupt Mask */
enum USB_CNTR_PMAOVRM = cast(ushort)0x4000; /* Packet Memory Area Over / Underrun Interrupt Mask */
enum USB_CNTR_CTRM = cast(ushort)0x8000; /* Correct Transfer Interrupt Mask */

/* Bit definition for USB_ISTR register */
enum USB_ISTR_EP_ID = cast(ushort)0x000F; /* Endpoint Identifier */
enum USB_ISTR_DIR = cast(ushort)0x0010; /* Direction of transaction */
enum USB_ISTR_ESOF = cast(ushort)0x0100; /* Expected Start Of Frame */
enum USB_ISTR_SOF = cast(ushort)0x0200; /* Start Of Frame */
enum USB_ISTR_RESET = cast(ushort)0x0400; /* USB RESET request */
enum USB_ISTR_SUSP = cast(ushort)0x0800; /* Suspend mode request */
enum USB_ISTR_WKUP = cast(ushort)0x1000; /* Wake up */
enum USB_ISTR_ERR = cast(ushort)0x2000; /* Error */
enum USB_ISTR_PMAOVR = cast(ushort)0x4000; /* Packet Memory Area Over / Underrun */
enum USB_ISTR_CTR = cast(ushort)0x8000; /* Correct Transfer */

/* Bit definition for USB_FNR register */
enum USB_FNR_FN = cast(ushort)0x07FF; /* Frame Number */
enum USB_FNR_LSOF = cast(ushort)0x1800; /* Lost SOF */
enum USB_FNR_LCK = cast(ushort)0x2000; /* Locked */
enum USB_FNR_RXDM = cast(ushort)0x4000; /* Receive Data - Line Status */
enum USB_FNR_RXDP = cast(ushort)0x8000; /* Receive Data + Line Status */

/* Bit definition for USB_DADDR register */
enum USB_DADDR_ADD = cast(ubyte)0x7F; /* ADD[6:0] bits (Device Address; */
enum USB_DADDR_ADD0 = cast(ubyte)0x01; /* Bit 0 */
enum USB_DADDR_ADD1 = cast(ubyte)0x02; /* Bit 1 */
enum USB_DADDR_ADD2 = cast(ubyte)0x04; /* Bit 2 */
enum USB_DADDR_ADD3 = cast(ubyte)0x08; /* Bit 3 */
enum USB_DADDR_ADD4 = cast(ubyte)0x10; /* Bit 4 */
enum USB_DADDR_ADD5 = cast(ubyte)0x20; /* Bit 5 */
enum USB_DADDR_ADD6 = cast(ubyte)0x40; /* Bit 6 */

enum USB_DADDR_EF = cast(ubyte)0x80; /* Enable Function */

/* Bit definition for USB_BTABLE register */
enum USB_BTABLE_BTABLE = cast(ushort)0xFFF8; /* Buffer Table */

/* Buffer descriptor table */
/* Bit definition for USB_ADDR0_TX register */
enum USB_ADDR0_TX_ADDR0_TX = cast(ushort)0xFFFE; /* Transmission Buffer Address 0 */

/* Bit definition for USB_ADDR1_TX register */
enum USB_ADDR1_TX_ADDR1_TX = cast(ushort)0xFFFE; /* Transmission Buffer Address 1 */

/* Bit definition for USB_ADDR2_TX register */
enum USB_ADDR2_TX_ADDR2_TX = cast(ushort)0xFFFE; /* Transmission Buffer Address 2 */

/* Bit definition for USB_ADDR3_TX register */
enum USB_ADDR3_TX_ADDR3_TX = cast(ushort)0xFFFE; /* Transmission Buffer Address 3 */

/* Bit definition for USB_ADDR4_TX register */
enum USB_ADDR4_TX_ADDR4_TX = cast(ushort)0xFFFE; /* Transmission Buffer Address 4 */

/* Bit definition for USB_ADDR5_TX register */
enum USB_ADDR5_TX_ADDR5_TX = cast(ushort)0xFFFE; /* Transmission Buffer Address 5 */

/* Bit definition for USB_ADDR6_TX register */
enum USB_ADDR6_TX_ADDR6_TX = cast(ushort)0xFFFE; /* Transmission Buffer Address 6 */

/* Bit definition for USB_ADDR7_TX register */
enum USB_ADDR7_TX_ADDR7_TX = cast(ushort)0xFFFE; /* Transmission Buffer Address 7 */

/* Bit definition for USB_COUNT0_TX register */
enum USB_COUNT0_TX_COUNT0_TX = cast(ushort)0x03FF; /* Transmission Byte Count 0 */

/* Bit definition for USB_COUNT1_TX register */
enum USB_COUNT1_TX_COUNT1_TX = cast(ushort)0x03FF; /* Transmission Byte Count 1 */

/* Bit definition for USB_COUNT2_TX register */
enum USB_COUNT2_TX_COUNT2_TX = cast(ushort)0x03FF; /* Transmission Byte Count 2 */

/* Bit definition for USB_COUNT3_TX register */
enum USB_COUNT3_TX_COUNT3_TX = cast(ushort)0x03FF; /* Transmission Byte Count 3 */

/* Bit definition for USB_COUNT4_TX register */
enum USB_COUNT4_TX_COUNT4_TX = cast(ushort)0x03FF; /* Transmission Byte Count 4 */

/* Bit definition for USB_COUNT5_TX register */
enum USB_COUNT5_TX_COUNT5_TX = cast(ushort)0x03FF; /* Transmission Byte Count 5 */

/* Bit definition for USB_COUNT6_TX register */
enum USB_COUNT6_TX_COUNT6_TX = cast(ushort)0x03FF; /* Transmission Byte Count 6 */

/* Bit definition for USB_COUNT7_TX register */
enum USB_COUNT7_TX_COUNT7_TX = cast(ushort)0x03FF; /* Transmission Byte Count 7 */

/* Bit definition for USB_COUNT0_TX_0 register */
enum USB_COUNT0_TX_0_COUNT0_TX_0 = cast(uint)0x000003FF; /* Transmission Byte Count 0 (low) */

/* Bit definition for USB_COUNT0_TX_1 register */
enum USB_COUNT0_TX_1_COUNT0_TX_1 = cast(uint)0x03FF0000; /* Transmission Byte Count 0 (high) */

/* Bit definition for USB_COUNT1_TX_0 register */
enum USB_COUNT1_TX_0_COUNT1_TX_0 = cast(uint)0x000003FF; /* Transmission Byte Count 1 (low) */

/* Bit definition for USB_COUNT1_TX_1 register */
enum USB_COUNT1_TX_1_COUNT1_TX_1 = cast(uint)0x03FF0000; /* Transmission Byte Count 1 (high) */

/* Bit definition for USB_COUNT2_TX_0 register */
enum USB_COUNT2_TX_0_COUNT2_TX_0 = cast(uint)0x000003FF; /* Transmission Byte Count 2 (low) */

/* Bit definition for USB_COUNT2_TX_1 register */
enum USB_COUNT2_TX_1_COUNT2_TX_1 = cast(uint)0x03FF0000; /* Transmission Byte Count 2 (high) */

/* Bit definition for USB_COUNT3_TX_0 register */
enum USB_COUNT3_TX_0_COUNT3_TX_0 = cast(ushort)0x000003FF; /* Transmission Byte Count 3 (low) */

/* Bit definition for USB_COUNT3_TX_1 register */
enum USB_COUNT3_TX_1_COUNT3_TX_1 = cast(ushort)0x03FF0000; /* Transmission Byte Count 3 (high) */

/* Bit definition for USB_COUNT4_TX_0 register */
enum USB_COUNT4_TX_0_COUNT4_TX_0 = cast(uint)0x000003FF; /* Transmission Byte Count 4 (low) */

/* Bit definition for USB_COUNT4_TX_1 register */
enum USB_COUNT4_TX_1_COUNT4_TX_1 = cast(uint)0x03FF0000; /* Transmission Byte Count 4 (high) */

/* Bit definition for USB_COUNT5_TX_0 register */
enum USB_COUNT5_TX_0_COUNT5_TX_0 = cast(uint)0x000003FF; /* Transmission Byte Count 5 (low) */

/* Bit definition for USB_COUNT5_TX_1 register */
enum USB_COUNT5_TX_1_COUNT5_TX_1 = cast(uint)0x03FF0000; /* Transmission Byte Count 5 (high) */

/* Bit definition for USB_COUNT6_TX_0 register */
enum USB_COUNT6_TX_0_COUNT6_TX_0 = cast(uint)0x000003FF; /* Transmission Byte Count 6 (low) */

/* Bit definition for USB_COUNT6_TX_1 register */
enum USB_COUNT6_TX_1_COUNT6_TX_1 = cast(uint)0x03FF0000; /* Transmission Byte Count 6 (high) */

/* Bit definition for USB_COUNT7_TX_0 register */
enum USB_COUNT7_TX_0_COUNT7_TX_0 = cast(uint)0x000003FF; /* Transmission Byte Count 7 (low) */

/* Bit definition for USB_COUNT7_TX_1 register */
enum USB_COUNT7_TX_1_COUNT7_TX_1 = cast(uint)0x03FF0000; /* Transmission Byte Count 7 (high) */

/* Bit definition for USB_ADDR0_RX register */
enum USB_ADDR0_RX_ADDR0_RX = cast(ushort)0xFFFE; /* Reception Buffer Address 0 */

/* Bit definition for USB_ADDR1_RX register */
enum USB_ADDR1_RX_ADDR1_RX = cast(ushort)0xFFFE; /* Reception Buffer Address 1 */

/* Bit definition for USB_ADDR2_RX register */
enum USB_ADDR2_RX_ADDR2_RX = cast(ushort)0xFFFE; /* Reception Buffer Address 2 */

/* Bit definition for USB_ADDR3_RX register */
enum USB_ADDR3_RX_ADDR3_RX = cast(ushort)0xFFFE; /* Reception Buffer Address 3 */

/* Bit definition for USB_ADDR4_RX register */
enum USB_ADDR4_RX_ADDR4_RX = cast(ushort)0xFFFE; /* Reception Buffer Address 4 */

/* Bit definition for USB_ADDR5_RX register */
enum USB_ADDR5_RX_ADDR5_RX = cast(ushort)0xFFFE; /* Reception Buffer Address 5 */

/* Bit definition for USB_ADDR6_RX register */
enum USB_ADDR6_RX_ADDR6_RX = cast(ushort)0xFFFE; /* Reception Buffer Address 6 */

/* Bit definition for USB_ADDR7_RX register */
enum USB_ADDR7_RX_ADDR7_RX = cast(ushort)0xFFFE; /* Reception Buffer Address 7 */

/* Bit definition for USB_COUNT0_RX register */
enum USB_COUNT0_RX_COUNT0_RX = cast(ushort)0x03FF; /* Reception Byte Count */

enum USB_COUNT0_RX_NUM_BLOCK = cast(ushort)0x7C00; /* NUM_BLOCK[4:0] bits (Number of blocks; */
enum USB_COUNT0_RX_NUM_BLOCK_0 = cast(ushort)0x0400; /* Bit 0 */
enum USB_COUNT0_RX_NUM_BLOCK_1 = cast(ushort)0x0800; /* Bit 1 */
enum USB_COUNT0_RX_NUM_BLOCK_2 = cast(ushort)0x1000; /* Bit 2 */
enum USB_COUNT0_RX_NUM_BLOCK_3 = cast(ushort)0x2000; /* Bit 3 */
enum USB_COUNT0_RX_NUM_BLOCK_4 = cast(ushort)0x4000; /* Bit 4 */

enum USB_COUNT0_RX_BLSIZE = cast(ushort)0x8000; /* BLock SIZE */

/* Bit definition for USB_COUNT1_RX register */
enum USB_COUNT1_RX_COUNT1_RX = cast(ushort)0x03FF; /* Reception Byte Count */

enum USB_COUNT1_RX_NUM_BLOCK = cast(ushort)0x7C00; /* NUM_BLOCK[4:0] bits (Number of blocks; */
enum USB_COUNT1_RX_NUM_BLOCK_0 = cast(ushort)0x0400; /* Bit 0 */
enum USB_COUNT1_RX_NUM_BLOCK_1 = cast(ushort)0x0800; /* Bit 1 */
enum USB_COUNT1_RX_NUM_BLOCK_2 = cast(ushort)0x1000; /* Bit 2 */
enum USB_COUNT1_RX_NUM_BLOCK_3 = cast(ushort)0x2000; /* Bit 3 */
enum USB_COUNT1_RX_NUM_BLOCK_4 = cast(ushort)0x4000; /* Bit 4 */

enum USB_COUNT1_RX_BLSIZE = cast(ushort)0x8000; /* BLock SIZE */

/* Bit definition for USB_COUNT2_RX register */
enum USB_COUNT2_RX_COUNT2_RX = cast(ushort)0x03FF; /* Reception Byte Count */

enum USB_COUNT2_RX_NUM_BLOCK = cast(ushort)0x7C00; /* NUM_BLOCK[4:0] bits (Number of blocks; */
enum USB_COUNT2_RX_NUM_BLOCK_0 = cast(ushort)0x0400; /* Bit 0 */
enum USB_COUNT2_RX_NUM_BLOCK_1 = cast(ushort)0x0800; /* Bit 1 */
enum USB_COUNT2_RX_NUM_BLOCK_2 = cast(ushort)0x1000; /* Bit 2 */
enum USB_COUNT2_RX_NUM_BLOCK_3 = cast(ushort)0x2000; /* Bit 3 */
enum USB_COUNT2_RX_NUM_BLOCK_4 = cast(ushort)0x4000; /* Bit 4 */

enum USB_COUNT2_RX_BLSIZE = cast(ushort)0x8000; /* BLock SIZE */

/* Bit definition for USB_COUNT3_RX register */
enum USB_COUNT3_RX_COUNT3_RX = cast(ushort)0x03FF; /* Reception Byte Count */

enum USB_COUNT3_RX_NUM_BLOCK = cast(ushort)0x7C00; /* NUM_BLOCK[4:0] bits (Number of blocks; */
enum USB_COUNT3_RX_NUM_BLOCK_0 = cast(ushort)0x0400; /* Bit 0 */
enum USB_COUNT3_RX_NUM_BLOCK_1 = cast(ushort)0x0800; /* Bit 1 */
enum USB_COUNT3_RX_NUM_BLOCK_2 = cast(ushort)0x1000; /* Bit 2 */
enum USB_COUNT3_RX_NUM_BLOCK_3 = cast(ushort)0x2000; /* Bit 3 */
enum USB_COUNT3_RX_NUM_BLOCK_4 = cast(ushort)0x4000; /* Bit 4 */

enum USB_COUNT3_RX_BLSIZE = cast(ushort)0x8000; /* BLock SIZE */

/* Bit definition for USB_COUNT4_RX register */
enum USB_COUNT4_RX_COUNT4_RX = cast(ushort)0x03FF; /* Reception Byte Count */

enum USB_COUNT4_RX_NUM_BLOCK = cast(ushort)0x7C00; /* NUM_BLOCK[4:0] bits (Number of blocks; */
enum USB_COUNT4_RX_NUM_BLOCK_0 = cast(ushort)0x0400; /* Bit 0 */
enum USB_COUNT4_RX_NUM_BLOCK_1 = cast(ushort)0x0800; /* Bit 1 */
enum USB_COUNT4_RX_NUM_BLOCK_2 = cast(ushort)0x1000; /* Bit 2 */
enum USB_COUNT4_RX_NUM_BLOCK_3 = cast(ushort)0x2000; /* Bit 3 */
enum USB_COUNT4_RX_NUM_BLOCK_4 = cast(ushort)0x4000; /* Bit 4 */

enum USB_COUNT4_RX_BLSIZE = cast(ushort)0x8000; /* BLock SIZE */

/* Bit definition for USB_COUNT5_RX register */
enum USB_COUNT5_RX_COUNT5_RX = cast(ushort)0x03FF; /* Reception Byte Count */

enum USB_COUNT5_RX_NUM_BLOCK = cast(ushort)0x7C00; /* NUM_BLOCK[4:0] bits (Number of blocks; */
enum USB_COUNT5_RX_NUM_BLOCK_0 = cast(ushort)0x0400; /* Bit 0 */
enum USB_COUNT5_RX_NUM_BLOCK_1 = cast(ushort)0x0800; /* Bit 1 */
enum USB_COUNT5_RX_NUM_BLOCK_2 = cast(ushort)0x1000; /* Bit 2 */
enum USB_COUNT5_RX_NUM_BLOCK_3 = cast(ushort)0x2000; /* Bit 3 */
enum USB_COUNT5_RX_NUM_BLOCK_4 = cast(ushort)0x4000; /* Bit 4 */

enum USB_COUNT5_RX_BLSIZE = cast(ushort)0x8000; /* BLock SIZE */

/* Bit definition for USB_COUNT6_RX register */
enum USB_COUNT6_RX_COUNT6_RX = cast(ushort)0x03FF; /* Reception Byte Count */

enum USB_COUNT6_RX_NUM_BLOCK = cast(ushort)0x7C00; /* NUM_BLOCK[4:0] bits (Number of blocks; */
enum USB_COUNT6_RX_NUM_BLOCK_0 = cast(ushort)0x0400; /* Bit 0 */
enum USB_COUNT6_RX_NUM_BLOCK_1 = cast(ushort)0x0800; /* Bit 1 */
enum USB_COUNT6_RX_NUM_BLOCK_2 = cast(ushort)0x1000; /* Bit 2 */
enum USB_COUNT6_RX_NUM_BLOCK_3 = cast(ushort)0x2000; /* Bit 3 */
enum USB_COUNT6_RX_NUM_BLOCK_4 = cast(ushort)0x4000; /* Bit 4 */

enum USB_COUNT6_RX_BLSIZE = cast(ushort)0x8000; /* BLock SIZE */

/* Bit definition for USB_COUNT7_RX register */
enum USB_COUNT7_RX_COUNT7_RX = cast(ushort)0x03FF; /* Reception Byte Count */

enum USB_COUNT7_RX_NUM_BLOCK = cast(ushort)0x7C00; /* NUM_BLOCK[4:0] bits (Number of blocks; */
enum USB_COUNT7_RX_NUM_BLOCK_0 = cast(ushort)0x0400; /* Bit 0 */
enum USB_COUNT7_RX_NUM_BLOCK_1 = cast(ushort)0x0800; /* Bit 1 */
enum USB_COUNT7_RX_NUM_BLOCK_2 = cast(ushort)0x1000; /* Bit 2 */
enum USB_COUNT7_RX_NUM_BLOCK_3 = cast(ushort)0x2000; /* Bit 3 */
enum USB_COUNT7_RX_NUM_BLOCK_4 = cast(ushort)0x4000; /* Bit 4 */

enum USB_COUNT7_RX_BLSIZE = cast(ushort)0x8000; /* BLock SIZE */

/* Bit definition for USB_COUNT0_RX_0 register */
enum USB_COUNT0_RX_0_COUNT0_RX_0 = cast(uint)0x000003FF; /* Reception Byte Count (low) */

enum USB_COUNT0_RX_0_NUM_BLOCK_0 = cast(uint)0x00007C00; /* NUM_BLOCK_0[4:0] bits (Number of blocks; (low) */
enum USB_COUNT0_RX_0_NUM_BLOCK_0_0 = cast(uint)0x00000400; /* Bit 0 */
enum USB_COUNT0_RX_0_NUM_BLOCK_0_1 = cast(uint)0x00000800; /* Bit 1 */
enum USB_COUNT0_RX_0_NUM_BLOCK_0_2 = cast(uint)0x00001000; /* Bit 2 */
enum USB_COUNT0_RX_0_NUM_BLOCK_0_3 = cast(uint)0x00002000; /* Bit 3 */
enum USB_COUNT0_RX_0_NUM_BLOCK_0_4 = cast(uint)0x00004000; /* Bit 4 */

enum USB_COUNT0_RX_0_BLSIZE_0 = cast(uint)0x00008000; /* BLock SIZE (low) */

/* Bit definition for USB_COUNT0_RX_1 register */
enum USB_COUNT0_RX_1_COUNT0_RX_1 = cast(uint)0x03FF0000; /* Reception Byte Count (high) */

enum USB_COUNT0_RX_1_NUM_BLOCK_1 = cast(uint)0x7C000000; /* NUM_BLOCK_1[4:0] bits (Number of blocks; (high) */
enum USB_COUNT0_RX_1_NUM_BLOCK_1_0 = cast(uint)0x04000000; /* Bit 1 */
enum USB_COUNT0_RX_1_NUM_BLOCK_1_1 = cast(uint)0x08000000; /* Bit 1 */
enum USB_COUNT0_RX_1_NUM_BLOCK_1_2 = cast(uint)0x10000000; /* Bit 2 */
enum USB_COUNT0_RX_1_NUM_BLOCK_1_3 = cast(uint)0x20000000; /* Bit 3 */
enum USB_COUNT0_RX_1_NUM_BLOCK_1_4 = cast(uint)0x40000000; /* Bit 4 */

enum USB_COUNT0_RX_1_BLSIZE_1 = cast(uint)0x80000000; /* BLock SIZE (high) */

/* Bit definition for USB_COUNT1_RX_0 register */
enum USB_COUNT1_RX_0_COUNT1_RX_0 = cast(uint)0x000003FF; /* Reception Byte Count (low) */

enum USB_COUNT1_RX_0_NUM_BLOCK_0 = cast(uint)0x00007C00; /* NUM_BLOCK_0[4:0] bits (Number of blocks; (low) */
enum USB_COUNT1_RX_0_NUM_BLOCK_0_0 = cast(uint)0x00000400; /* Bit 0 */
enum USB_COUNT1_RX_0_NUM_BLOCK_0_1 = cast(uint)0x00000800; /* Bit 1 */
enum USB_COUNT1_RX_0_NUM_BLOCK_0_2 = cast(uint)0x00001000; /* Bit 2 */
enum USB_COUNT1_RX_0_NUM_BLOCK_0_3 = cast(uint)0x00002000; /* Bit 3 */
enum USB_COUNT1_RX_0_NUM_BLOCK_0_4 = cast(uint)0x00004000; /* Bit 4 */

enum USB_COUNT1_RX_0_BLSIZE_0 = cast(uint)0x00008000; /* BLock SIZE (low) */

/* Bit definition for USB_COUNT1_RX_1 register */
enum USB_COUNT1_RX_1_COUNT1_RX_1 = cast(uint)0x03FF0000; /* Reception Byte Count (high) */

enum USB_COUNT1_RX_1_NUM_BLOCK_1 = cast(uint)0x7C000000; /* NUM_BLOCK_1[4:0] bits (Number of blocks; (high) */
enum USB_COUNT1_RX_1_NUM_BLOCK_1_0 = cast(uint)0x04000000; /* Bit 0 */
enum USB_COUNT1_RX_1_NUM_BLOCK_1_1 = cast(uint)0x08000000; /* Bit 1 */
enum USB_COUNT1_RX_1_NUM_BLOCK_1_2 = cast(uint)0x10000000; /* Bit 2 */
enum USB_COUNT1_RX_1_NUM_BLOCK_1_3 = cast(uint)0x20000000; /* Bit 3 */
enum USB_COUNT1_RX_1_NUM_BLOCK_1_4 = cast(uint)0x40000000; /* Bit 4 */

enum USB_COUNT1_RX_1_BLSIZE_1 = cast(uint)0x80000000; /* BLock SIZE (high) */

/* Bit definition for USB_COUNT2_RX_0 register */
enum USB_COUNT2_RX_0_COUNT2_RX_0 = cast(uint)0x000003FF; /* Reception Byte Count (low) */

enum USB_COUNT2_RX_0_NUM_BLOCK_0 = cast(uint)0x00007C00; /* NUM_BLOCK_0[4:0] bits (Number of blocks; (low) */
enum USB_COUNT2_RX_0_NUM_BLOCK_0_0 = cast(uint)0x00000400; /* Bit 0 */
enum USB_COUNT2_RX_0_NUM_BLOCK_0_1 = cast(uint)0x00000800; /* Bit 1 */
enum USB_COUNT2_RX_0_NUM_BLOCK_0_2 = cast(uint)0x00001000; /* Bit 2 */
enum USB_COUNT2_RX_0_NUM_BLOCK_0_3 = cast(uint)0x00002000; /* Bit 3 */
enum USB_COUNT2_RX_0_NUM_BLOCK_0_4 = cast(uint)0x00004000; /* Bit 4 */

enum USB_COUNT2_RX_0_BLSIZE_0 = cast(uint)0x00008000; /* BLock SIZE (low) */

/* Bit definition for USB_COUNT2_RX_1 register */
enum USB_COUNT2_RX_1_COUNT2_RX_1 = cast(uint)0x03FF0000; /* Reception Byte Count (high) */

enum USB_COUNT2_RX_1_NUM_BLOCK_1 = cast(uint)0x7C000000; /* NUM_BLOCK_1[4:0] bits (Number of blocks; (high) */
enum USB_COUNT2_RX_1_NUM_BLOCK_1_0 = cast(uint)0x04000000; /* Bit 0 */
enum USB_COUNT2_RX_1_NUM_BLOCK_1_1 = cast(uint)0x08000000; /* Bit 1 */
enum USB_COUNT2_RX_1_NUM_BLOCK_1_2 = cast(uint)0x10000000; /* Bit 2 */
enum USB_COUNT2_RX_1_NUM_BLOCK_1_3 = cast(uint)0x20000000; /* Bit 3 */
enum USB_COUNT2_RX_1_NUM_BLOCK_1_4 = cast(uint)0x40000000; /* Bit 4 */

enum USB_COUNT2_RX_1_BLSIZE_1 = cast(uint)0x80000000; /* BLock SIZE (high) */

/* Bit definition for USB_COUNT3_RX_0 register */
enum USB_COUNT3_RX_0_COUNT3_RX_0 = cast(uint)0x000003FF; /* Reception Byte Count (low) */

enum USB_COUNT3_RX_0_NUM_BLOCK_0 = cast(uint)0x00007C00; /* NUM_BLOCK_0[4:0] bits (Number of blocks; (low) */
enum USB_COUNT3_RX_0_NUM_BLOCK_0_0 = cast(uint)0x00000400; /* Bit 0 */
enum USB_COUNT3_RX_0_NUM_BLOCK_0_1 = cast(uint)0x00000800; /* Bit 1 */
enum USB_COUNT3_RX_0_NUM_BLOCK_0_2 = cast(uint)0x00001000; /* Bit 2 */
enum USB_COUNT3_RX_0_NUM_BLOCK_0_3 = cast(uint)0x00002000; /* Bit 3 */
enum USB_COUNT3_RX_0_NUM_BLOCK_0_4 = cast(uint)0x00004000; /* Bit 4 */

enum USB_COUNT3_RX_0_BLSIZE_0 = cast(uint)0x00008000; /* BLock SIZE (low) */

/* Bit definition for USB_COUNT3_RX_1 register */
enum USB_COUNT3_RX_1_COUNT3_RX_1 = cast(uint)0x03FF0000; /* Reception Byte Count (high) */

enum USB_COUNT3_RX_1_NUM_BLOCK_1 = cast(uint)0x7C000000; /* NUM_BLOCK_1[4:0] bits (Number of blocks; (high) */
enum USB_COUNT3_RX_1_NUM_BLOCK_1_0 = cast(uint)0x04000000; /* Bit 0 */
enum USB_COUNT3_RX_1_NUM_BLOCK_1_1 = cast(uint)0x08000000; /* Bit 1 */
enum USB_COUNT3_RX_1_NUM_BLOCK_1_2 = cast(uint)0x10000000; /* Bit 2 */
enum USB_COUNT3_RX_1_NUM_BLOCK_1_3 = cast(uint)0x20000000; /* Bit 3 */
enum USB_COUNT3_RX_1_NUM_BLOCK_1_4 = cast(uint)0x40000000; /* Bit 4 */

enum USB_COUNT3_RX_1_BLSIZE_1 = cast(uint)0x80000000; /* BLock SIZE (high) */

/* Bit definition for USB_COUNT4_RX_0 register */
enum USB_COUNT4_RX_0_COUNT4_RX_0 = cast(uint)0x000003FF; /* Reception Byte Count (low) */

enum USB_COUNT4_RX_0_NUM_BLOCK_0 = cast(uint)0x00007C00; /* NUM_BLOCK_0[4:0] bits (Number of blocks; (low) */
enum USB_COUNT4_RX_0_NUM_BLOCK_0_0 = cast(uint)0x00000400; /* Bit 0 */
enum USB_COUNT4_RX_0_NUM_BLOCK_0_1 = cast(uint)0x00000800; /* Bit 1 */
enum USB_COUNT4_RX_0_NUM_BLOCK_0_2 = cast(uint)0x00001000; /* Bit 2 */
enum USB_COUNT4_RX_0_NUM_BLOCK_0_3 = cast(uint)0x00002000; /* Bit 3 */
enum USB_COUNT4_RX_0_NUM_BLOCK_0_4 = cast(uint)0x00004000; /* Bit 4 */

enum USB_COUNT4_RX_0_BLSIZE_0 = cast(uint)0x00008000; /* BLock SIZE (low) */

/* Bit definition for USB_COUNT4_RX_1 register */
enum USB_COUNT4_RX_1_COUNT4_RX_1 = cast(uint)0x03FF0000; /* Reception Byte Count (high) */

enum USB_COUNT4_RX_1_NUM_BLOCK_1 = cast(uint)0x7C000000; /* NUM_BLOCK_1[4:0] bits (Number of blocks; (high) */
enum USB_COUNT4_RX_1_NUM_BLOCK_1_0 = cast(uint)0x04000000; /* Bit 0 */
enum USB_COUNT4_RX_1_NUM_BLOCK_1_1 = cast(uint)0x08000000; /* Bit 1 */
enum USB_COUNT4_RX_1_NUM_BLOCK_1_2 = cast(uint)0x10000000; /* Bit 2 */
enum USB_COUNT4_RX_1_NUM_BLOCK_1_3 = cast(uint)0x20000000; /* Bit 3 */
enum USB_COUNT4_RX_1_NUM_BLOCK_1_4 = cast(uint)0x40000000; /* Bit 4 */

enum USB_COUNT4_RX_1_BLSIZE_1 = cast(uint)0x80000000; /* BLock SIZE (high) */

/* Bit definition for USB_COUNT5_RX_0 register */
enum USB_COUNT5_RX_0_COUNT5_RX_0 = cast(uint)0x000003FF; /* Reception Byte Count (low) */

enum USB_COUNT5_RX_0_NUM_BLOCK_0 = cast(uint)0x00007C00; /* NUM_BLOCK_0[4:0] bits (Number of blocks; (low) */
enum USB_COUNT5_RX_0_NUM_BLOCK_0_0 = cast(uint)0x00000400; /* Bit 0 */
enum USB_COUNT5_RX_0_NUM_BLOCK_0_1 = cast(uint)0x00000800; /* Bit 1 */
enum USB_COUNT5_RX_0_NUM_BLOCK_0_2 = cast(uint)0x00001000; /* Bit 2 */
enum USB_COUNT5_RX_0_NUM_BLOCK_0_3 = cast(uint)0x00002000; /* Bit 3 */
enum USB_COUNT5_RX_0_NUM_BLOCK_0_4 = cast(uint)0x00004000; /* Bit 4 */

enum USB_COUNT5_RX_0_BLSIZE_0 = cast(uint)0x00008000; /* BLock SIZE (low) */

/* Bit definition for USB_COUNT5_RX_1 register */
enum USB_COUNT5_RX_1_COUNT5_RX_1 = cast(uint)0x03FF0000; /* Reception Byte Count (high) */

enum USB_COUNT5_RX_1_NUM_BLOCK_1 = cast(uint)0x7C000000; /* NUM_BLOCK_1[4:0] bits (Number of blocks; (high) */
enum USB_COUNT5_RX_1_NUM_BLOCK_1_0 = cast(uint)0x04000000; /* Bit 0 */
enum USB_COUNT5_RX_1_NUM_BLOCK_1_1 = cast(uint)0x08000000; /* Bit 1 */
enum USB_COUNT5_RX_1_NUM_BLOCK_1_2 = cast(uint)0x10000000; /* Bit 2 */
enum USB_COUNT5_RX_1_NUM_BLOCK_1_3 = cast(uint)0x20000000; /* Bit 3 */
enum USB_COUNT5_RX_1_NUM_BLOCK_1_4 = cast(uint)0x40000000; /* Bit 4 */

enum USB_COUNT5_RX_1_BLSIZE_1 = cast(uint)0x80000000; /* BLock SIZE (high) */

/* Bit definition for USB_COUNT6_RX_0 register */
enum USB_COUNT6_RX_0_COUNT6_RX_0 = cast(uint)0x000003FF; /* Reception Byte Count (low) */

enum USB_COUNT6_RX_0_NUM_BLOCK_0 = cast(uint)0x00007C00; /* NUM_BLOCK_0[4:0] bits (Number of blocks; (low) */
enum USB_COUNT6_RX_0_NUM_BLOCK_0_0 = cast(uint)0x00000400; /* Bit 0 */
enum USB_COUNT6_RX_0_NUM_BLOCK_0_1 = cast(uint)0x00000800; /* Bit 1 */
enum USB_COUNT6_RX_0_NUM_BLOCK_0_2 = cast(uint)0x00001000; /* Bit 2 */
enum USB_COUNT6_RX_0_NUM_BLOCK_0_3 = cast(uint)0x00002000; /* Bit 3 */
enum USB_COUNT6_RX_0_NUM_BLOCK_0_4 = cast(uint)0x00004000; /* Bit 4 */

enum USB_COUNT6_RX_0_BLSIZE_0 = cast(uint)0x00008000; /* BLock SIZE (low) */

/* Bit definition for USB_COUNT6_RX_1 register */
enum USB_COUNT6_RX_1_COUNT6_RX_1 = cast(uint)0x03FF0000; /* Reception Byte Count (high) */

enum USB_COUNT6_RX_1_NUM_BLOCK_1 = cast(uint)0x7C000000; /* NUM_BLOCK_1[4:0] bits (Number of blocks; (high) */
enum USB_COUNT6_RX_1_NUM_BLOCK_1_0 = cast(uint)0x04000000; /* Bit 0 */
enum USB_COUNT6_RX_1_NUM_BLOCK_1_1 = cast(uint)0x08000000; /* Bit 1 */
enum USB_COUNT6_RX_1_NUM_BLOCK_1_2 = cast(uint)0x10000000; /* Bit 2 */
enum USB_COUNT6_RX_1_NUM_BLOCK_1_3 = cast(uint)0x20000000; /* Bit 3 */
enum USB_COUNT6_RX_1_NUM_BLOCK_1_4 = cast(uint)0x40000000; /* Bit 4 */

enum USB_COUNT6_RX_1_BLSIZE_1 = cast(uint)0x80000000; /* BLock SIZE (high) */

/* Bit definition for USB_COUNT7_RX_0 register */
enum USB_COUNT7_RX_0_COUNT7_RX_0 = cast(uint)0x000003FF; /* Reception Byte Count (low) */

enum USB_COUNT7_RX_0_NUM_BLOCK_0 = cast(uint)0x00007C00; /* NUM_BLOCK_0[4:0] bits (Number of blocks; (low) */
enum USB_COUNT7_RX_0_NUM_BLOCK_0_0 = cast(uint)0x00000400; /* Bit 0 */
enum USB_COUNT7_RX_0_NUM_BLOCK_0_1 = cast(uint)0x00000800; /* Bit 1 */
enum USB_COUNT7_RX_0_NUM_BLOCK_0_2 = cast(uint)0x00001000; /* Bit 2 */
enum USB_COUNT7_RX_0_NUM_BLOCK_0_3 = cast(uint)0x00002000; /* Bit 3 */
enum USB_COUNT7_RX_0_NUM_BLOCK_0_4 = cast(uint)0x00004000; /* Bit 4 */

enum USB_COUNT7_RX_0_BLSIZE_0 = cast(uint)0x00008000; /* BLock SIZE (low) */

/* Bit definition for USB_COUNT7_RX_1 register */
enum USB_COUNT7_RX_1_COUNT7_RX_1 = cast(uint)0x03FF0000; /* Reception Byte Count (high) */

enum USB_COUNT7_RX_1_NUM_BLOCK_1 = cast(uint)0x7C000000; /* NUM_BLOCK_1[4:0] bits (Number of blocks; (high) */
enum USB_COUNT7_RX_1_NUM_BLOCK_1_0 = cast(uint)0x04000000; /* Bit 0 */
enum USB_COUNT7_RX_1_NUM_BLOCK_1_1 = cast(uint)0x08000000; /* Bit 1 */
enum USB_COUNT7_RX_1_NUM_BLOCK_1_2 = cast(uint)0x10000000; /* Bit 2 */
enum USB_COUNT7_RX_1_NUM_BLOCK_1_3 = cast(uint)0x20000000; /* Bit 3 */
enum USB_COUNT7_RX_1_NUM_BLOCK_1_4 = cast(uint)0x40000000; /* Bit 4 */

enum USB_COUNT7_RX_1_BLSIZE_1 = cast(uint)0x80000000; /* BLock SIZE (high) */

/* CAN control and status registers */
/* Bit definition for CAN_MCR register */
enum CAN_MCR_INRQ = cast(ushort)0x0001; /* Initialization Request */
enum CAN_MCR_SLEEP = cast(ushort)0x0002; /* Sleep Mode Request */
enum CAN_MCR_TXFP = cast(ushort)0x0004; /* Transmit FIFO Priority */
enum CAN_MCR_RFLM = cast(ushort)0x0008; /* Receive FIFO Locked Mode */
enum CAN_MCR_NART = cast(ushort)0x0010; /* No Automatic Retransmission */
enum CAN_MCR_AWUM = cast(ushort)0x0020; /* Automatic Wakeup Mode */
enum CAN_MCR_ABOM = cast(ushort)0x0040; /* Automatic Bus-Off Management */
enum CAN_MCR_TTCM = cast(ushort)0x0080; /* Time Triggered Communication Mode */
enum CAN_MCR_RESET = cast(ushort)0x8000; /* CAN software master reset */

/* Bit definition for CAN_MSR register */
enum CAN_MSR_INAK = cast(ushort)0x0001; /* Initialization Acknowledge */
enum CAN_MSR_SLAK = cast(ushort)0x0002; /* Sleep Acknowledge */
enum CAN_MSR_ERRI = cast(ushort)0x0004; /* Error Interrupt */
enum CAN_MSR_WKUI = cast(ushort)0x0008; /* Wakeup Interrupt */
enum CAN_MSR_SLAKI = cast(ushort)0x0010; /* Sleep Acknowledge Interrupt */
enum CAN_MSR_TXM = cast(ushort)0x0100; /* Transmit Mode */
enum CAN_MSR_RXM = cast(ushort)0x0200; /* Receive Mode */
enum CAN_MSR_SAMP = cast(ushort)0x0400; /* Last Sample Point */
enum CAN_MSR_RX = cast(ushort)0x0800; /* CAN Rx Signal */

/* Bit definition for CAN_TSR register */
enum CAN_TSR_RQCP0 = cast(uint)0x00000001; /* Request Completed Mailbox0 */
enum CAN_TSR_TXOK0 = cast(uint)0x00000002; /* Transmission OK of Mailbox0 */
enum CAN_TSR_ALST0 = cast(uint)0x00000004; /* Arbitration Lost for Mailbox0 */
enum CAN_TSR_TERR0 = cast(uint)0x00000008; /* Transmission Error of Mailbox0 */
enum CAN_TSR_ABRQ0 = cast(uint)0x00000080; /* Abort Request for Mailbox0 */
enum CAN_TSR_RQCP1 = cast(uint)0x00000100; /* Request Completed Mailbox1 */
enum CAN_TSR_TXOK1 = cast(uint)0x00000200; /* Transmission OK of Mailbox1 */
enum CAN_TSR_ALST1 = cast(uint)0x00000400; /* Arbitration Lost for Mailbox1 */
enum CAN_TSR_TERR1 = cast(uint)0x00000800; /* Transmission Error of Mailbox1 */
enum CAN_TSR_ABRQ1 = cast(uint)0x00008000; /* Abort Request for Mailbox 1 */
enum CAN_TSR_RQCP2 = cast(uint)0x00010000; /* Request Completed Mailbox2 */
enum CAN_TSR_TXOK2 = cast(uint)0x00020000; /* Transmission OK of Mailbox 2 */
enum CAN_TSR_ALST2 = cast(uint)0x00040000; /* Arbitration Lost for mailbox 2 */
enum CAN_TSR_TERR2 = cast(uint)0x00080000; /* Transmission Error of Mailbox 2 */
enum CAN_TSR_ABRQ2 = cast(uint)0x00800000; /* Abort Request for Mailbox 2 */
enum CAN_TSR_CODE = cast(uint)0x03000000; /* Mailbox Code */

enum CAN_TSR_TME = cast(uint)0x1C000000; /* TME[2:0] bits */
enum CAN_TSR_TME0 = cast(uint)0x04000000; /* Transmit Mailbox 0 Empty */
enum CAN_TSR_TME1 = cast(uint)0x08000000; /* Transmit Mailbox 1 Empty */
enum CAN_TSR_TME2 = cast(uint)0x10000000; /* Transmit Mailbox 2 Empty */

enum CAN_TSR_LOW = cast(uint)0xE0000000; /* LOW[2:0] bits */
enum CAN_TSR_LOW0 = cast(uint)0x20000000; /* Lowest Priority Flag for Mailbox 0 */
enum CAN_TSR_LOW1 = cast(uint)0x40000000; /* Lowest Priority Flag for Mailbox 1 */
enum CAN_TSR_LOW2 = cast(uint)0x80000000; /* Lowest Priority Flag for Mailbox 2 */

/* Bit definition for CAN_RF0R register */
enum CAN_RF0R_FMP0 = cast(ubyte)0x03; /* FIFO 0 Message Pending */
enum CAN_RF0R_FULL0 = cast(ubyte)0x08; /* FIFO 0 Full */
enum CAN_RF0R_FOVR0 = cast(ubyte)0x10; /* FIFO 0 Overrun */
enum CAN_RF0R_RFOM0 = cast(ubyte)0x20; /* Release FIFO 0 Output Mailbox */

/* Bit definition for CAN_RF1R register */
enum CAN_RF1R_FMP1 = cast(ubyte)0x03; /* FIFO 1 Message Pending */
enum CAN_RF1R_FULL1 = cast(ubyte)0x08; /* FIFO 1 Full */
enum CAN_RF1R_FOVR1 = cast(ubyte)0x10; /* FIFO 1 Overrun */
enum CAN_RF1R_RFOM1 = cast(ubyte)0x20; /* Release FIFO 1 Output Mailbox */

/* Bit definition for CAN_IER register */
enum CAN_IER_TMEIE = cast(uint)0x00000001; /* Transmit Mailbox Empty Interrupt Enable */
enum CAN_IER_FMPIE0 = cast(uint)0x00000002; /* FIFO Message Pending Interrupt Enable */
enum CAN_IER_FFIE0 = cast(uint)0x00000004; /* FIFO Full Interrupt Enable */
enum CAN_IER_FOVIE0 = cast(uint)0x00000008; /* FIFO Overrun Interrupt Enable */
enum CAN_IER_FMPIE1 = cast(uint)0x00000010; /* FIFO Message Pending Interrupt Enable */
enum CAN_IER_FFIE1 = cast(uint)0x00000020; /* FIFO Full Interrupt Enable */
enum CAN_IER_FOVIE1 = cast(uint)0x00000040; /* FIFO Overrun Interrupt Enable */
enum CAN_IER_EWGIE = cast(uint)0x00000100; /* Error Warning Interrupt Enable */
enum CAN_IER_EPVIE = cast(uint)0x00000200; /* Error Passive Interrupt Enable */
enum CAN_IER_BOFIE = cast(uint)0x00000400; /* Bus-Off Interrupt Enable */
enum CAN_IER_LECIE = cast(uint)0x00000800; /* Last Error Code Interrupt Enable */
enum CAN_IER_ERRIE = cast(uint)0x00008000; /* Error Interrupt Enable */
enum CAN_IER_WKUIE = cast(uint)0x00010000; /* Wakeup Interrupt Enable */
enum CAN_IER_SLKIE = cast(uint)0x00020000; /* Sleep Interrupt Enable */

/* Bit definition for CAN_ESR register */
enum CAN_ESR_EWGF = cast(uint)0x00000001; /* Error Warning Flag */
enum CAN_ESR_EPVF = cast(uint)0x00000002; /* Error Passive Flag */
enum CAN_ESR_BOFF = cast(uint)0x00000004; /* Bus-Off Flag */

enum CAN_ESR_LEC = cast(uint)0x00000070; /* LEC[2:0] bits (Last Error Code; */
enum CAN_ESR_LEC_0 = cast(uint)0x00000010; /* Bit 0 */
enum CAN_ESR_LEC_1 = cast(uint)0x00000020; /* Bit 1 */
enum CAN_ESR_LEC_2 = cast(uint)0x00000040; /* Bit 2 */

enum CAN_ESR_TEC = cast(uint)0x00FF0000; /* Least significant byte of the 9-bit Transmit Error Counter */
enum CAN_ESR_REC = cast(uint)0xFF000000; /* Receive Error Counter */

/* Bit definition for CAN_BTR register */
enum CAN_BTR_BRP = cast(uint)0x000003FF; /* Baud Rate Prescaler */
enum CAN_BTR_TS1 = cast(uint)0x000F0000; /* Time Segment 1 */
enum CAN_BTR_TS2 = cast(uint)0x00700000; /* Time Segment 2 */
enum CAN_BTR_SJW = cast(uint)0x03000000; /* Resynchronization Jump Width */
enum CAN_BTR_LBKM = cast(uint)0x40000000; /* Loop Back Mode (Debug; */
enum CAN_BTR_SILM = cast(uint)0x80000000; /* Silent Mode */

/* Mailbox registers */
/* Bit definition for CAN_TI0R register */
enum CAN_TI0R_TXRQ = cast(uint)0x00000001; /* Transmit Mailbox Request */
enum CAN_TI0R_RTR = cast(uint)0x00000002; /* Remote Transmission Request */
enum CAN_TI0R_IDE = cast(uint)0x00000004; /* Identifier Extension */
enum CAN_TI0R_EXID = cast(uint)0x001FFFF8; /* Extended Identifier */
enum CAN_TI0R_STID = cast(uint)0xFFE00000; /* Standard Identifier or Extended Identifier */

/* Bit definition for CAN_TDT0R register */
enum CAN_TDT0R_DLC = cast(uint)0x0000000F; /* Data Length Code */
enum CAN_TDT0R_TGT = cast(uint)0x00000100; /* Transmit Global Time */
enum CAN_TDT0R_TIME = cast(uint)0xFFFF0000; /* Message Time Stamp */

/* Bit definition for CAN_TDL0R register */
enum CAN_TDL0R_DATA0 = cast(uint)0x000000FF; /* Data byte 0 */
enum CAN_TDL0R_DATA1 = cast(uint)0x0000FF00; /* Data byte 1 */
enum CAN_TDL0R_DATA2 = cast(uint)0x00FF0000; /* Data byte 2 */
enum CAN_TDL0R_DATA3 = cast(uint)0xFF000000; /* Data byte 3 */

/* Bit definition for CAN_TDH0R register */
enum CAN_TDH0R_DATA4 = cast(uint)0x000000FF; /* Data byte 4 */
enum CAN_TDH0R_DATA5 = cast(uint)0x0000FF00; /* Data byte 5 */
enum CAN_TDH0R_DATA6 = cast(uint)0x00FF0000; /* Data byte 6 */
enum CAN_TDH0R_DATA7 = cast(uint)0xFF000000; /* Data byte 7 */

/* Bit definition for CAN_TI1R register */
enum CAN_TI1R_TXRQ = cast(uint)0x00000001; /* Transmit Mailbox Request */
enum CAN_TI1R_RTR = cast(uint)0x00000002; /* Remote Transmission Request */
enum CAN_TI1R_IDE = cast(uint)0x00000004; /* Identifier Extension */
enum CAN_TI1R_EXID = cast(uint)0x001FFFF8; /* Extended Identifier */
enum CAN_TI1R_STID = cast(uint)0xFFE00000; /* Standard Identifier or Extended Identifier */

/* Bit definition for CAN_TDT1R register */
enum CAN_TDT1R_DLC = cast(uint)0x0000000F; /* Data Length Code */
enum CAN_TDT1R_TGT = cast(uint)0x00000100; /* Transmit Global Time */
enum CAN_TDT1R_TIME = cast(uint)0xFFFF0000; /* Message Time Stamp */

/* Bit definition for CAN_TDL1R register */
enum CAN_TDL1R_DATA0 = cast(uint)0x000000FF; /* Data byte 0 */
enum CAN_TDL1R_DATA1 = cast(uint)0x0000FF00; /* Data byte 1 */
enum CAN_TDL1R_DATA2 = cast(uint)0x00FF0000; /* Data byte 2 */
enum CAN_TDL1R_DATA3 = cast(uint)0xFF000000; /* Data byte 3 */

/* Bit definition for CAN_TDH1R register */
enum CAN_TDH1R_DATA4 = cast(uint)0x000000FF; /* Data byte 4 */
enum CAN_TDH1R_DATA5 = cast(uint)0x0000FF00; /* Data byte 5 */
enum CAN_TDH1R_DATA6 = cast(uint)0x00FF0000; /* Data byte 6 */
enum CAN_TDH1R_DATA7 = cast(uint)0xFF000000; /* Data byte 7 */

/* Bit definition for CAN_TI2R register */
enum CAN_TI2R_TXRQ = cast(uint)0x00000001; /* Transmit Mailbox Request */
enum CAN_TI2R_RTR = cast(uint)0x00000002; /* Remote Transmission Request */
enum CAN_TI2R_IDE = cast(uint)0x00000004; /* Identifier Extension */
enum CAN_TI2R_EXID = cast(uint)0x001FFFF8; /* Extended identifier */
enum CAN_TI2R_STID = cast(uint)0xFFE00000; /* Standard Identifier or Extended Identifier */

/* Bit definition for CAN_TDT2R register */
enum CAN_TDT2R_DLC = cast(uint)0x0000000F; /* Data Length Code */
enum CAN_TDT2R_TGT = cast(uint)0x00000100; /* Transmit Global Time */
enum CAN_TDT2R_TIME = cast(uint)0xFFFF0000; /* Message Time Stamp */

/* Bit definition for CAN_TDL2R register */
enum CAN_TDL2R_DATA0 = cast(uint)0x000000FF; /* Data byte 0 */
enum CAN_TDL2R_DATA1 = cast(uint)0x0000FF00; /* Data byte 1 */
enum CAN_TDL2R_DATA2 = cast(uint)0x00FF0000; /* Data byte 2 */
enum CAN_TDL2R_DATA3 = cast(uint)0xFF000000; /* Data byte 3 */

/* Bit definition for CAN_TDH2R register */
enum CAN_TDH2R_DATA4 = cast(uint)0x000000FF; /* Data byte 4 */
enum CAN_TDH2R_DATA5 = cast(uint)0x0000FF00; /* Data byte 5 */
enum CAN_TDH2R_DATA6 = cast(uint)0x00FF0000; /* Data byte 6 */
enum CAN_TDH2R_DATA7 = cast(uint)0xFF000000; /* Data byte 7 */

/* Bit definition for CAN_RI0R register */
enum CAN_RI0R_RTR = cast(uint)0x00000002; /* Remote Transmission Request */
enum CAN_RI0R_IDE = cast(uint)0x00000004; /* Identifier Extension */
enum CAN_RI0R_EXID = cast(uint)0x001FFFF8; /* Extended Identifier */
enum CAN_RI0R_STID = cast(uint)0xFFE00000; /* Standard Identifier or Extended Identifier */

/* Bit definition for CAN_RDT0R register */
enum CAN_RDT0R_DLC = cast(uint)0x0000000F; /* Data Length Code */
enum CAN_RDT0R_FMI = cast(uint)0x0000FF00; /* Filter Match Index */
enum CAN_RDT0R_TIME = cast(uint)0xFFFF0000; /* Message Time Stamp */

/* Bit definition for CAN_RDL0R register */
enum CAN_RDL0R_DATA0 = cast(uint)0x000000FF; /* Data byte 0 */
enum CAN_RDL0R_DATA1 = cast(uint)0x0000FF00; /* Data byte 1 */
enum CAN_RDL0R_DATA2 = cast(uint)0x00FF0000; /* Data byte 2 */
enum CAN_RDL0R_DATA3 = cast(uint)0xFF000000; /* Data byte 3 */

/* Bit definition for CAN_RDH0R register */
enum CAN_RDH0R_DATA4 = cast(uint)0x000000FF; /* Data byte 4 */
enum CAN_RDH0R_DATA5 = cast(uint)0x0000FF00; /* Data byte 5 */
enum CAN_RDH0R_DATA6 = cast(uint)0x00FF0000; /* Data byte 6 */
enum CAN_RDH0R_DATA7 = cast(uint)0xFF000000; /* Data byte 7 */

/* Bit definition for CAN_RI1R register */
enum CAN_RI1R_RTR = cast(uint)0x00000002; /* Remote Transmission Request */
enum CAN_RI1R_IDE = cast(uint)0x00000004; /* Identifier Extension */
enum CAN_RI1R_EXID = cast(uint)0x001FFFF8; /* Extended identifier */
enum CAN_RI1R_STID = cast(uint)0xFFE00000; /* Standard Identifier or Extended Identifier */

/* Bit definition for CAN_RDT1R register */
enum CAN_RDT1R_DLC = cast(uint)0x0000000F; /* Data Length Code */
enum CAN_RDT1R_FMI = cast(uint)0x0000FF00; /* Filter Match Index */
enum CAN_RDT1R_TIME = cast(uint)0xFFFF0000; /* Message Time Stamp */

/* Bit definition for CAN_RDL1R register */
enum CAN_RDL1R_DATA0 = cast(uint)0x000000FF; /* Data byte 0 */
enum CAN_RDL1R_DATA1 = cast(uint)0x0000FF00; /* Data byte 1 */
enum CAN_RDL1R_DATA2 = cast(uint)0x00FF0000; /* Data byte 2 */
enum CAN_RDL1R_DATA3 = cast(uint)0xFF000000; /* Data byte 3 */

/* Bit definition for CAN_RDH1R register */
enum CAN_RDH1R_DATA4 = cast(uint)0x000000FF; /* Data byte 4 */
enum CAN_RDH1R_DATA5 = cast(uint)0x0000FF00; /* Data byte 5 */
enum CAN_RDH1R_DATA6 = cast(uint)0x00FF0000; /* Data byte 6 */
enum CAN_RDH1R_DATA7 = cast(uint)0xFF000000; /* Data byte 7 */

/* CAN filter registers */
/* Bit definition for CAN_FMR register */
enum CAN_FMR_FINIT = cast(ubyte)0x01; /* Filter Init Mode */

/* Bit definition for CAN_FM1R register */
enum CAN_FM1R_FBM = cast(ushort)0x3FFF; /* Filter Mode */
enum CAN_FM1R_FBM0 = cast(ushort)0x0001; /* Filter Init Mode bit 0 */
enum CAN_FM1R_FBM1 = cast(ushort)0x0002; /* Filter Init Mode bit 1 */
enum CAN_FM1R_FBM2 = cast(ushort)0x0004; /* Filter Init Mode bit 2 */
enum CAN_FM1R_FBM3 = cast(ushort)0x0008; /* Filter Init Mode bit 3 */
enum CAN_FM1R_FBM4 = cast(ushort)0x0010; /* Filter Init Mode bit 4 */
enum CAN_FM1R_FBM5 = cast(ushort)0x0020; /* Filter Init Mode bit 5 */
enum CAN_FM1R_FBM6 = cast(ushort)0x0040; /* Filter Init Mode bit 6 */
enum CAN_FM1R_FBM7 = cast(ushort)0x0080; /* Filter Init Mode bit 7 */
enum CAN_FM1R_FBM8 = cast(ushort)0x0100; /* Filter Init Mode bit 8 */
enum CAN_FM1R_FBM9 = cast(ushort)0x0200; /* Filter Init Mode bit 9 */
enum CAN_FM1R_FBM10 = cast(ushort)0x0400; /* Filter Init Mode bit 10 */
enum CAN_FM1R_FBM11 = cast(ushort)0x0800; /* Filter Init Mode bit 11 */
enum CAN_FM1R_FBM12 = cast(ushort)0x1000; /* Filter Init Mode bit 12 */
enum CAN_FM1R_FBM13 = cast(ushort)0x2000; /* Filter Init Mode bit 13 */

/* Bit definition for CAN_FS1R register */
enum CAN_FS1R_FSC = cast(ushort)0x3FFF; /* Filter Scale Configuration */
enum CAN_FS1R_FSC0 = cast(ushort)0x0001; /* Filter Scale Configuration bit 0 */
enum CAN_FS1R_FSC1 = cast(ushort)0x0002; /* Filter Scale Configuration bit 1 */
enum CAN_FS1R_FSC2 = cast(ushort)0x0004; /* Filter Scale Configuration bit 2 */
enum CAN_FS1R_FSC3 = cast(ushort)0x0008; /* Filter Scale Configuration bit 3 */
enum CAN_FS1R_FSC4 = cast(ushort)0x0010; /* Filter Scale Configuration bit 4 */
enum CAN_FS1R_FSC5 = cast(ushort)0x0020; /* Filter Scale Configuration bit 5 */
enum CAN_FS1R_FSC6 = cast(ushort)0x0040; /* Filter Scale Configuration bit 6 */
enum CAN_FS1R_FSC7 = cast(ushort)0x0080; /* Filter Scale Configuration bit 7 */
enum CAN_FS1R_FSC8 = cast(ushort)0x0100; /* Filter Scale Configuration bit 8 */
enum CAN_FS1R_FSC9 = cast(ushort)0x0200; /* Filter Scale Configuration bit 9 */
enum CAN_FS1R_FSC10 = cast(ushort)0x0400; /* Filter Scale Configuration bit 10 */
enum CAN_FS1R_FSC11 = cast(ushort)0x0800; /* Filter Scale Configuration bit 11 */
enum CAN_FS1R_FSC12 = cast(ushort)0x1000; /* Filter Scale Configuration bit 12 */
enum CAN_FS1R_FSC13 = cast(ushort)0x2000; /* Filter Scale Configuration bit 13 */

/* Bit definition for CAN_FFA1R register */
enum CAN_FFA1R_FFA = cast(ushort)0x3FFF; /* Filter FIFO Assignment */
enum CAN_FFA1R_FFA0 = cast(ushort)0x0001; /* Filter FIFO Assignment for Filter 0 */
enum CAN_FFA1R_FFA1 = cast(ushort)0x0002; /* Filter FIFO Assignment for Filter 1 */
enum CAN_FFA1R_FFA2 = cast(ushort)0x0004; /* Filter FIFO Assignment for Filter 2 */
enum CAN_FFA1R_FFA3 = cast(ushort)0x0008; /* Filter FIFO Assignment for Filter 3 */
enum CAN_FFA1R_FFA4 = cast(ushort)0x0010; /* Filter FIFO Assignment for Filter 4 */
enum CAN_FFA1R_FFA5 = cast(ushort)0x0020; /* Filter FIFO Assignment for Filter 5 */
enum CAN_FFA1R_FFA6 = cast(ushort)0x0040; /* Filter FIFO Assignment for Filter 6 */
enum CAN_FFA1R_FFA7 = cast(ushort)0x0080; /* Filter FIFO Assignment for Filter 7 */
enum CAN_FFA1R_FFA8 = cast(ushort)0x0100; /* Filter FIFO Assignment for Filter 8 */
enum CAN_FFA1R_FFA9 = cast(ushort)0x0200; /* Filter FIFO Assignment for Filter 9 */
enum CAN_FFA1R_FFA10 = cast(ushort)0x0400; /* Filter FIFO Assignment for Filter 10 */
enum CAN_FFA1R_FFA11 = cast(ushort)0x0800; /* Filter FIFO Assignment for Filter 11 */
enum CAN_FFA1R_FFA12 = cast(ushort)0x1000; /* Filter FIFO Assignment for Filter 12 */
enum CAN_FFA1R_FFA13 = cast(ushort)0x2000; /* Filter FIFO Assignment for Filter 13 */

/* Bit definition for CAN_FA1R register */
enum CAN_FA1R_FACT = cast(ushort)0x3FFF; /* Filter Active */
enum CAN_FA1R_FACT0 = cast(ushort)0x0001; /* Filter 0 Active */
enum CAN_FA1R_FACT1 = cast(ushort)0x0002; /* Filter 1 Active */
enum CAN_FA1R_FACT2 = cast(ushort)0x0004; /* Filter 2 Active */
enum CAN_FA1R_FACT3 = cast(ushort)0x0008; /* Filter 3 Active */
enum CAN_FA1R_FACT4 = cast(ushort)0x0010; /* Filter 4 Active */
enum CAN_FA1R_FACT5 = cast(ushort)0x0020; /* Filter 5 Active */
enum CAN_FA1R_FACT6 = cast(ushort)0x0040; /* Filter 6 Active */
enum CAN_FA1R_FACT7 = cast(ushort)0x0080; /* Filter 7 Active */
enum CAN_FA1R_FACT8 = cast(ushort)0x0100; /* Filter 8 Active */
enum CAN_FA1R_FACT9 = cast(ushort)0x0200; /* Filter 9 Active */
enum CAN_FA1R_FACT10 = cast(ushort)0x0400; /* Filter 10 Active */
enum CAN_FA1R_FACT11 = cast(ushort)0x0800; /* Filter 11 Active */
enum CAN_FA1R_FACT12 = cast(ushort)0x1000; /* Filter 12 Active */
enum CAN_FA1R_FACT13 = cast(ushort)0x2000; /* Filter 13 Active */

/* Bit definition for CAN_F0R1 register */
enum CAN_F0R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F0R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F0R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F0R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F0R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F0R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F0R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F0R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F0R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F0R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F0R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F0R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F0R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F0R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F0R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F0R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F0R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F0R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F0R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F0R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F0R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F0R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F0R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F0R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F0R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F0R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F0R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F0R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F0R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F0R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F0R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F0R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F1R1 register */
enum CAN_F1R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F1R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F1R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F1R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F1R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F1R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F1R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F1R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F1R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F1R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F1R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F1R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F1R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F1R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F1R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F1R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F1R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F1R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F1R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F1R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F1R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F1R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F1R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F1R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F1R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F1R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F1R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F1R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F1R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F1R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F1R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F1R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F2R1 register */
enum CAN_F2R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F2R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F2R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F2R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F2R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F2R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F2R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F2R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F2R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F2R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F2R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F2R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F2R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F2R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F2R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F2R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F2R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F2R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F2R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F2R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F2R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F2R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F2R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F2R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F2R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F2R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F2R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F2R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F2R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F2R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F2R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F2R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F3R1 register */
enum CAN_F3R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F3R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F3R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F3R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F3R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F3R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F3R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F3R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F3R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F3R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F3R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F3R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F3R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F3R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F3R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F3R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F3R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F3R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F3R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F3R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F3R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F3R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F3R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F3R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F3R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F3R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F3R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F3R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F3R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F3R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F3R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F3R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F4R1 register */
enum CAN_F4R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F4R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F4R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F4R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F4R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F4R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F4R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F4R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F4R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F4R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F4R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F4R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F4R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F4R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F4R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F4R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F4R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F4R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F4R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F4R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F4R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F4R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F4R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F4R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F4R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F4R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F4R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F4R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F4R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F4R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F4R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F4R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F5R1 register */
enum CAN_F5R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F5R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F5R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F5R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F5R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F5R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F5R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F5R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F5R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F5R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F5R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F5R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F5R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F5R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F5R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F5R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F5R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F5R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F5R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F5R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F5R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F5R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F5R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F5R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F5R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F5R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F5R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F5R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F5R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F5R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F5R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F5R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F6R1 register */
enum CAN_F6R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F6R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F6R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F6R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F6R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F6R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F6R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F6R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F6R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F6R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F6R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F6R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F6R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F6R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F6R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F6R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F6R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F6R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F6R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F6R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F6R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F6R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F6R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F6R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F6R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F6R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F6R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F6R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F6R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F6R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F6R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F6R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F7R1 register */
enum CAN_F7R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F7R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F7R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F7R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F7R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F7R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F7R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F7R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F7R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F7R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F7R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F7R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F7R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F7R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F7R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F7R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F7R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F7R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F7R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F7R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F7R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F7R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F7R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F7R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F7R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F7R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F7R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F7R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F7R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F7R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F7R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F7R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F8R1 register */
enum CAN_F8R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F8R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F8R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F8R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F8R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F8R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F8R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F8R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F8R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F8R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F8R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F8R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F8R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F8R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F8R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F8R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F8R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F8R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F8R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F8R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F8R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F8R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F8R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F8R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F8R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F8R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F8R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F8R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F8R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F8R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F8R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F8R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F9R1 register */
enum CAN_F9R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F9R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F9R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F9R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F9R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F9R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F9R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F9R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F9R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F9R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F9R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F9R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F9R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F9R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F9R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F9R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F9R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F9R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F9R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F9R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F9R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F9R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F9R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F9R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F9R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F9R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F9R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F9R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F9R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F9R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F9R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F9R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F10R1 register */
enum CAN_F10R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F10R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F10R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F10R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F10R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F10R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F10R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F10R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F10R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F10R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F10R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F10R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F10R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F10R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F10R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F10R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F10R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F10R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F10R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F10R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F10R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F10R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F10R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F10R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F10R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F10R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F10R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F10R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F10R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F10R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F10R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F10R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F11R1 register */
enum CAN_F11R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F11R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F11R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F11R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F11R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F11R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F11R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F11R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F11R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F11R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F11R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F11R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F11R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F11R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F11R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F11R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F11R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F11R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F11R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F11R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F11R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F11R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F11R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F11R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F11R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F11R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F11R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F11R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F11R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F11R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F11R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F11R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F12R1 register */
enum CAN_F12R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F12R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F12R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F12R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F12R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F12R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F12R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F12R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F12R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F12R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F12R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F12R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F12R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F12R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F12R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F12R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F12R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F12R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F12R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F12R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F12R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F12R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F12R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F12R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F12R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F12R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F12R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F12R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F12R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F12R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F12R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F12R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F13R1 register */
enum CAN_F13R1_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F13R1_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F13R1_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F13R1_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F13R1_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F13R1_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F13R1_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F13R1_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F13R1_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F13R1_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F13R1_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F13R1_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F13R1_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F13R1_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F13R1_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F13R1_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F13R1_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F13R1_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F13R1_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F13R1_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F13R1_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F13R1_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F13R1_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F13R1_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F13R1_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F13R1_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F13R1_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F13R1_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F13R1_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F13R1_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F13R1_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F13R1_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F0R2 register */
enum CAN_F0R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F0R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F0R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F0R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F0R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F0R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F0R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F0R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F0R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F0R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F0R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F0R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F0R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F0R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F0R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F0R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F0R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F0R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F0R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F0R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F0R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F0R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F0R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F0R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F0R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F0R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F0R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F0R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F0R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F0R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F0R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F0R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F1R2 register */
enum CAN_F1R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F1R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F1R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F1R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F1R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F1R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F1R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F1R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F1R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F1R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F1R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F1R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F1R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F1R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F1R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F1R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F1R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F1R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F1R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F1R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F1R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F1R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F1R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F1R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F1R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F1R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F1R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F1R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F1R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F1R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F1R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F1R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F2R2 register */
enum CAN_F2R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F2R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F2R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F2R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F2R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F2R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F2R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F2R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F2R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F2R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F2R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F2R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F2R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F2R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F2R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F2R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F2R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F2R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F2R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F2R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F2R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F2R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F2R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F2R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F2R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F2R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F2R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F2R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F2R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F2R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F2R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F2R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F3R2 register */
enum CAN_F3R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F3R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F3R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F3R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F3R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F3R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F3R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F3R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F3R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F3R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F3R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F3R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F3R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F3R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F3R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F3R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F3R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F3R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F3R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F3R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F3R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F3R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F3R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F3R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F3R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F3R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F3R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F3R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F3R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F3R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F3R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F3R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F4R2 register */
enum CAN_F4R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F4R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F4R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F4R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F4R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F4R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F4R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F4R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F4R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F4R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F4R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F4R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F4R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F4R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F4R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F4R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F4R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F4R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F4R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F4R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F4R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F4R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F4R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F4R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F4R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F4R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F4R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F4R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F4R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F4R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F4R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F4R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F5R2 register */
enum CAN_F5R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F5R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F5R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F5R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F5R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F5R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F5R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F5R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F5R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F5R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F5R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F5R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F5R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F5R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F5R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F5R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F5R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F5R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F5R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F5R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F5R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F5R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F5R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F5R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F5R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F5R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F5R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F5R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F5R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F5R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F5R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F5R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F6R2 register */
enum CAN_F6R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F6R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F6R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F6R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F6R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F6R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F6R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F6R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F6R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F6R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F6R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F6R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F6R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F6R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F6R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F6R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F6R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F6R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F6R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F6R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F6R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F6R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F6R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F6R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F6R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F6R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F6R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F6R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F6R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F6R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F6R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F6R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F7R2 register */
enum CAN_F7R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F7R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F7R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F7R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F7R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F7R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F7R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F7R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F7R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F7R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F7R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F7R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F7R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F7R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F7R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F7R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F7R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F7R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F7R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F7R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F7R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F7R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F7R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F7R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F7R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F7R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F7R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F7R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F7R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F7R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F7R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F7R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F8R2 register */
enum CAN_F8R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F8R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F8R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F8R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F8R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F8R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F8R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F8R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F8R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F8R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F8R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F8R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F8R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F8R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F8R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F8R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F8R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F8R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F8R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F8R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F8R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F8R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F8R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F8R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F8R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F8R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F8R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F8R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F8R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F8R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F8R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F8R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F9R2 register */
enum CAN_F9R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F9R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F9R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F9R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F9R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F9R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F9R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F9R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F9R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F9R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F9R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F9R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F9R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F9R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F9R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F9R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F9R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F9R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F9R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F9R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F9R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F9R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F9R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F9R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F9R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F9R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F9R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F9R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F9R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F9R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F9R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F9R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F10R2 register */
enum CAN_F10R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F10R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F10R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F10R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F10R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F10R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F10R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F10R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F10R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F10R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F10R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F10R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F10R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F10R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F10R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F10R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F10R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F10R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F10R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F10R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F10R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F10R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F10R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F10R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F10R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F10R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F10R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F10R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F10R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F10R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F10R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F10R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F11R2 register */
enum CAN_F11R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F11R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F11R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F11R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F11R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F11R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F11R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F11R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F11R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F11R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F11R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F11R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F11R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F11R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F11R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F11R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F11R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F11R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F11R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F11R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F11R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F11R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F11R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F11R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F11R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F11R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F11R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F11R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F11R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F11R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F11R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F11R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F12R2 register */
enum CAN_F12R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F12R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F12R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F12R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F12R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F12R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F12R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F12R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F12R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F12R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F12R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F12R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F12R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F12R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F12R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F12R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F12R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F12R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F12R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F12R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F12R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F12R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F12R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F12R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F12R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F12R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F12R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F12R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F12R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F12R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F12R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F12R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for CAN_F13R2 register */
enum CAN_F13R2_FB0 = cast(uint)0x00000001; /* Filter bit 0 */
enum CAN_F13R2_FB1 = cast(uint)0x00000002; /* Filter bit 1 */
enum CAN_F13R2_FB2 = cast(uint)0x00000004; /* Filter bit 2 */
enum CAN_F13R2_FB3 = cast(uint)0x00000008; /* Filter bit 3 */
enum CAN_F13R2_FB4 = cast(uint)0x00000010; /* Filter bit 4 */
enum CAN_F13R2_FB5 = cast(uint)0x00000020; /* Filter bit 5 */
enum CAN_F13R2_FB6 = cast(uint)0x00000040; /* Filter bit 6 */
enum CAN_F13R2_FB7 = cast(uint)0x00000080; /* Filter bit 7 */
enum CAN_F13R2_FB8 = cast(uint)0x00000100; /* Filter bit 8 */
enum CAN_F13R2_FB9 = cast(uint)0x00000200; /* Filter bit 9 */
enum CAN_F13R2_FB10 = cast(uint)0x00000400; /* Filter bit 10 */
enum CAN_F13R2_FB11 = cast(uint)0x00000800; /* Filter bit 11 */
enum CAN_F13R2_FB12 = cast(uint)0x00001000; /* Filter bit 12 */
enum CAN_F13R2_FB13 = cast(uint)0x00002000; /* Filter bit 13 */
enum CAN_F13R2_FB14 = cast(uint)0x00004000; /* Filter bit 14 */
enum CAN_F13R2_FB15 = cast(uint)0x00008000; /* Filter bit 15 */
enum CAN_F13R2_FB16 = cast(uint)0x00010000; /* Filter bit 16 */
enum CAN_F13R2_FB17 = cast(uint)0x00020000; /* Filter bit 17 */
enum CAN_F13R2_FB18 = cast(uint)0x00040000; /* Filter bit 18 */
enum CAN_F13R2_FB19 = cast(uint)0x00080000; /* Filter bit 19 */
enum CAN_F13R2_FB20 = cast(uint)0x00100000; /* Filter bit 20 */
enum CAN_F13R2_FB21 = cast(uint)0x00200000; /* Filter bit 21 */
enum CAN_F13R2_FB22 = cast(uint)0x00400000; /* Filter bit 22 */
enum CAN_F13R2_FB23 = cast(uint)0x00800000; /* Filter bit 23 */
enum CAN_F13R2_FB24 = cast(uint)0x01000000; /* Filter bit 24 */
enum CAN_F13R2_FB25 = cast(uint)0x02000000; /* Filter bit 25 */
enum CAN_F13R2_FB26 = cast(uint)0x04000000; /* Filter bit 26 */
enum CAN_F13R2_FB27 = cast(uint)0x08000000; /* Filter bit 27 */
enum CAN_F13R2_FB28 = cast(uint)0x10000000; /* Filter bit 28 */
enum CAN_F13R2_FB29 = cast(uint)0x20000000; /* Filter bit 29 */
enum CAN_F13R2_FB30 = cast(uint)0x40000000; /* Filter bit 30 */
enum CAN_F13R2_FB31 = cast(uint)0x80000000; /* Filter bit 31 */

/* Bit definition for SPI_CR1 register */
enum SPI_CR1_CPHA = cast(ushort)0x0001; /* Clock Phase */
enum SPI_CR1_CPOL = cast(ushort)0x0002; /* Clock Polarity */
enum SPI_CR1_MSTR = cast(ushort)0x0004; /* Master Selection */

enum SPI_CR1_BR = cast(ushort)0x0038; /* BR[2:0] bits (Baud Rate Control; */
enum SPI_CR1_BR_0 = cast(ushort)0x0008; /* Bit 0 */
enum SPI_CR1_BR_1 = cast(ushort)0x0010; /* Bit 1 */
enum SPI_CR1_BR_2 = cast(ushort)0x0020; /* Bit 2 */

enum SPI_CR1_SPE = cast(ushort)0x0040; /* SPI Enable */
enum SPI_CR1_LSBFIRST = cast(ushort)0x0080; /* Frame Format */
enum SPI_CR1_SSI = cast(ushort)0x0100; /* Internal slave select */
enum SPI_CR1_SSM = cast(ushort)0x0200; /* Software slave management */
enum SPI_CR1_RXONLY = cast(ushort)0x0400; /* Receive only */
enum SPI_CR1_DFF = cast(ushort)0x0800; /* Data Frame Format */
enum SPI_CR1_CRCNEXT = cast(ushort)0x1000; /* Transmit CRC next */
enum SPI_CR1_CRCEN = cast(ushort)0x2000; /* Hardware CRC calculation enable */
enum SPI_CR1_BIDIOE = cast(ushort)0x4000; /* Output enable in bidirectional mode */
enum SPI_CR1_BIDIMODE = cast(ushort)0x8000; /* Bidirectional data mode enable */

/* Bit definition for SPI_CR2 register */
enum SPI_CR2_RXDMAEN = cast(ubyte)0x01; /* Rx Buffer DMA Enable */
enum SPI_CR2_TXDMAEN = cast(ubyte)0x02; /* Tx Buffer DMA Enable */
enum SPI_CR2_SSOE = cast(ubyte)0x04; /* SS Output Enable */
enum SPI_CR2_ERRIE = cast(ubyte)0x20; /* Error Interrupt Enable */
enum SPI_CR2_RXNEIE = cast(ubyte)0x40; /* RX buffer Not Empty Interrupt Enable */
enum SPI_CR2_TXEIE = cast(ubyte)0x80; /* Tx buffer Empty Interrupt Enable */

/* Bit definition for SPI_SR register */
enum SPI_SR_RXNE = cast(ubyte)0x01; /* Receive buffer Not Empty */
enum SPI_SR_TXE = cast(ubyte)0x02; /* Transmit buffer Empty */
enum SPI_SR_CHSIDE = cast(ubyte)0x04; /* Channel side */
enum SPI_SR_UDR = cast(ubyte)0x08; /* Underrun flag */
enum SPI_SR_CRCERR = cast(ubyte)0x10; /* CRC Error flag */
enum SPI_SR_MODF = cast(ubyte)0x20; /* Mode fault */
enum SPI_SR_OVR = cast(ubyte)0x40; /* Overrun flag */
enum SPI_SR_BSY = cast(ubyte)0x80; /* Busy flag */

/* Bit definition for SPI_DR register */
enum SPI_DR_DR = cast(ushort)0xFFFF; /* Data Register */

/* Bit definition for SPI_CRCPR register */
enum SPI_CRCPR_CRCPOLY = cast(ushort)0xFFFF; /* CRC polynomial register */

/* Bit definition for SPI_RXCRCR register */
enum SPI_RXCRCR_RXCRC = cast(ushort)0xFFFF; /* Rx CRC Register */

/* Bit definition for SPI_TXCRCR register */
enum SPI_TXCRCR_TXCRC = cast(ushort)0xFFFF; /* Tx CRC Register */

/* Bit definition for SPI_I2SCFGR register */
enum SPI_I2SCFGR_CHLEN = cast(ushort)0x0001; /* Channel length (number of bits per audio channel; */

enum SPI_I2SCFGR_DATLEN = cast(ushort)0x0006; /* DATLEN[1:0] bits (Data length to be transferred; */
enum SPI_I2SCFGR_DATLEN_0 = cast(ushort)0x0002; /* Bit 0 */
enum SPI_I2SCFGR_DATLEN_1 = cast(ushort)0x0004; /* Bit 1 */

enum SPI_I2SCFGR_CKPOL = cast(ushort)0x0008; /* steady state clock polarity */

enum SPI_I2SCFGR_I2SSTD = cast(ushort)0x0030; /* I2SSTD[1:0] bits (I2S standard selection; */
enum SPI_I2SCFGR_I2SSTD_0 = cast(ushort)0x0010; /* Bit 0 */
enum SPI_I2SCFGR_I2SSTD_1 = cast(ushort)0x0020; /* Bit 1 */

enum SPI_I2SCFGR_PCMSYNC = cast(ushort)0x0080; /* PCM frame synchronization */

enum SPI_I2SCFGR_I2SCFG = cast(ushort)0x0300; /* I2SCFG[1:0] bits (I2S configuration mode; */
enum SPI_I2SCFGR_I2SCFG_0 = cast(ushort)0x0100; /* Bit 0 */
enum SPI_I2SCFGR_I2SCFG_1 = cast(ushort)0x0200; /* Bit 1 */

enum SPI_I2SCFGR_I2SE = cast(ushort)0x0400; /* I2S Enable */
enum SPI_I2SCFGR_I2SMOD = cast(ushort)0x0800; /* I2S mode selection */

/* Bit definition for SPI_I2SPR register */
enum SPI_I2SPR_I2SDIV = cast(ushort)0x00FF; /* I2S Linear prescaler */
enum SPI_I2SPR_ODD = cast(ushort)0x0100; /* Odd factor for the prescaler */
enum SPI_I2SPR_MCKOE = cast(ushort)0x0200; /* Master Clock Output Enable */

/* Bit definition for I2C_CR1 register */
enum I2C_CR1_PE = cast(ushort)0x0001; /* Peripheral Enable */
enum I2C_CR1_SMBUS = cast(ushort)0x0002; /* SMBus Mode */
enum I2C_CR1_SMBTYPE = cast(ushort)0x0008; /* SMBus Type */
enum I2C_CR1_ENARP = cast(ushort)0x0010; /* ARP Enable */
enum I2C_CR1_ENPEC = cast(ushort)0x0020; /* PEC Enable */
enum I2C_CR1_ENGC = cast(ushort)0x0040; /* General Call Enable */
enum I2C_CR1_NOSTRETCH = cast(ushort)0x0080; /* Clock Stretching Disable (Slave mode; */
enum I2C_CR1_START = cast(ushort)0x0100; /* Start Generation */
enum I2C_CR1_STOP = cast(ushort)0x0200; /* Stop Generation */
enum I2C_CR1_ACK = cast(ushort)0x0400; /* Acknowledge Enable */
enum I2C_CR1_POS = cast(ushort)0x0800; /* Acknowledge/PEC Position (for data reception; */
enum I2C_CR1_PEC = cast(ushort)0x1000; /* Packet Error Checking */
enum I2C_CR1_ALERT = cast(ushort)0x2000; /* SMBus Alert */
enum I2C_CR1_SWRST = cast(ushort)0x8000; /* Software Reset */

/* Bit definition for I2C_CR2 register */
enum I2C_CR2_FREQ = cast(ushort)0x003F; /* FREQ[5:0] bits (Peripheral Clock Frequency; */
enum I2C_CR2_FREQ_0 = cast(ushort)0x0001; /* Bit 0 */
enum I2C_CR2_FREQ_1 = cast(ushort)0x0002; /* Bit 1 */
enum I2C_CR2_FREQ_2 = cast(ushort)0x0004; /* Bit 2 */
enum I2C_CR2_FREQ_3 = cast(ushort)0x0008; /* Bit 3 */
enum I2C_CR2_FREQ_4 = cast(ushort)0x0010; /* Bit 4 */
enum I2C_CR2_FREQ_5 = cast(ushort)0x0020; /* Bit 5 */

enum I2C_CR2_ITERREN = cast(ushort)0x0100; /* Error Interrupt Enable */
enum I2C_CR2_ITEVTEN = cast(ushort)0x0200; /* Event Interrupt Enable */
enum I2C_CR2_ITBUFEN = cast(ushort)0x0400; /* Buffer Interrupt Enable */
enum I2C_CR2_DMAEN = cast(ushort)0x0800; /* DMA Requests Enable */
enum I2C_CR2_LAST = cast(ushort)0x1000; /* DMA Last Transfer */

/* Bit definition for I2C_OAR1 register */
enum I2C_OAR1_ADD1_7 = cast(ushort)0x00FE; /* Interface Address */
enum I2C_OAR1_ADD8_9 = cast(ushort)0x0300; /* Interface Address */

enum I2C_OAR1_ADD0 = cast(ushort)0x0001; /* Bit 0 */
enum I2C_OAR1_ADD1 = cast(ushort)0x0002; /* Bit 1 */
enum I2C_OAR1_ADD2 = cast(ushort)0x0004; /* Bit 2 */
enum I2C_OAR1_ADD3 = cast(ushort)0x0008; /* Bit 3 */
enum I2C_OAR1_ADD4 = cast(ushort)0x0010; /* Bit 4 */
enum I2C_OAR1_ADD5 = cast(ushort)0x0020; /* Bit 5 */
enum I2C_OAR1_ADD6 = cast(ushort)0x0040; /* Bit 6 */
enum I2C_OAR1_ADD7 = cast(ushort)0x0080; /* Bit 7 */
enum I2C_OAR1_ADD8 = cast(ushort)0x0100; /* Bit 8 */
enum I2C_OAR1_ADD9 = cast(ushort)0x0200; /* Bit 9 */

enum I2C_OAR1_ADDMODE = cast(ushort)0x8000; /* Addressing Mode (Slave mode; */

/* Bit definition for I2C_OAR2 register */
enum I2C_OAR2_ENDUAL = cast(ubyte)0x01; /* Dual addressing mode enable */
enum I2C_OAR2_ADD2 = cast(ubyte)0xFE; /* Interface address */

/* Bit definition for I2C_DR register */
enum I2C_DR_DR = cast(ubyte)0xFF; /* 8-bit Data Register */

/* Bit definition for I2C_SR1 register */
enum I2C_SR1_SB = cast(ushort)0x0001; /* Start Bit (Master mode; */
enum I2C_SR1_ADDR = cast(ushort)0x0002; /* Address sent (master mode)/matched (slave mode; */
enum I2C_SR1_BTF = cast(ushort)0x0004; /* Byte Transfer Finished */
enum I2C_SR1_ADD10 = cast(ushort)0x0008; /* 10-bit header sent (Master mode; */
enum I2C_SR1_STOPF = cast(ushort)0x0010; /* Stop detection (Slave mode; */
enum I2C_SR1_RXNE = cast(ushort)0x0040; /* Data Register not Empty (receivers; */
enum I2C_SR1_TXE = cast(ushort)0x0080; /* Data Register Empty (transmitters; */
enum I2C_SR1_BERR = cast(ushort)0x0100; /* Bus Error */
enum I2C_SR1_ARLO = cast(ushort)0x0200; /* Arbitration Lost (master mode; */
enum I2C_SR1_AF = cast(ushort)0x0400; /* Acknowledge Failure */
enum I2C_SR1_OVR = cast(ushort)0x0800; /* Overrun/Underrun */
enum I2C_SR1_PECERR = cast(ushort)0x1000; /* PEC Error in reception */
enum I2C_SR1_TIMEOUT = cast(ushort)0x4000; /* Timeout or Tlow Error */
enum I2C_SR1_SMBALERT = cast(ushort)0x8000; /* SMBus Alert */

/* Bit definition for I2C_SR2 register */
enum I2C_SR2_MSL = cast(ushort)0x0001; /* Master/Slave */
enum I2C_SR2_BUSY = cast(ushort)0x0002; /* Bus Busy */
enum I2C_SR2_TRA = cast(ushort)0x0004; /* Transmitter/Receiver */
enum I2C_SR2_GENCALL = cast(ushort)0x0010; /* General Call Address (Slave mode; */
enum I2C_SR2_SMBDEFAULT = cast(ushort)0x0020; /* SMBus Device Default Address (Slave mode; */
enum I2C_SR2_SMBHOST = cast(ushort)0x0040; /* SMBus Host Header (Slave mode; */
enum I2C_SR2_DUALF = cast(ushort)0x0080; /* Dual Flag (Slave mode; */
enum I2C_SR2_PEC = cast(ushort)0xFF00; /* Packet Error Checking Register */

/* Bit definition for I2C_CCR register */
enum I2C_CCR_CCR = cast(ushort)0x0FFF; /* Clock Control Register in Fast/Standard mode (Master mode; */
enum I2C_CCR_DUTY = cast(ushort)0x4000; /* Fast Mode Duty Cycle */
enum I2C_CCR_FS = cast(ushort)0x8000; /* I2C Master Mode Selection */

/* Bit definition for I2C_TRISE register */
enum I2C_TRISE_TRISE = cast(ubyte)0x3F; /* Maximum Rise Time in Fast/Standard mode (Master mode; */

/* Bit definition for USART_SR register */
enum USART_SR_PE = cast(ushort)0x0001; /* Parity Error */
enum USART_SR_FE = cast(ushort)0x0002; /* Framing Error */
enum USART_SR_NE = cast(ushort)0x0004; /* Noise Error Flag */
enum USART_SR_ORE = cast(ushort)0x0008; /* OverRun Error */
enum USART_SR_IDLE = cast(ushort)0x0010; /* IDLE line detected */
enum USART_SR_RXNE = cast(ushort)0x0020; /* Read Data Register Not Empty */
enum USART_SR_TC = cast(ushort)0x0040; /* Transmission Complete */
enum USART_SR_TXE = cast(ushort)0x0080; /* Transmit Data Register Empty */
enum USART_SR_LBD = cast(ushort)0x0100; /* LIN Break Detection Flag */
enum USART_SR_CTS = cast(ushort)0x0200; /* CTS Flag */

/* Bit definition for USART_DR register */
enum USART_DR_DR = cast(ushort)0x01FF; /* Data value */

/* Bit definition for USART_BRR register */
enum USART_BRR_DIV_FRACTION = cast(ushort)0x000F; /* Fraction of USARTDIV */
enum USART_BRR_DIV_MANTISSA = cast(ushort)0xFFF0; /* Mantissa of USARTDIV */

/* Bit definition for USART_CR1 register */
enum USART_CR1_SBK = cast(ushort)0x0001; /* Send Break */
enum USART_CR1_RWU = cast(ushort)0x0002; /* Receiver wakeup */
enum USART_CR1_RE = cast(ushort)0x0004; /* Receiver Enable */
enum USART_CR1_TE = cast(ushort)0x0008; /* Transmitter Enable */
enum USART_CR1_IDLEIE = cast(ushort)0x0010; /* IDLE Interrupt Enable */
enum USART_CR1_RXNEIE = cast(ushort)0x0020; /* RXNE Interrupt Enable */
enum USART_CR1_TCIE = cast(ushort)0x0040; /* Transmission Complete Interrupt Enable */
enum USART_CR1_TXEIE = cast(ushort)0x0080; /* PE Interrupt Enable */
enum USART_CR1_PEIE = cast(ushort)0x0100; /* PE Interrupt Enable */
enum USART_CR1_PS = cast(ushort)0x0200; /* Parity Selection */
enum USART_CR1_PCE = cast(ushort)0x0400; /* Parity Control Enable */
enum USART_CR1_WAKE = cast(ushort)0x0800; /* Wakeup method */
enum USART_CR1_M = cast(ushort)0x1000; /* Word length */
enum USART_CR1_UE = cast(ushort)0x2000; /* USART Enable */
enum USART_CR1_OVER8 = cast(ushort)0x8000; /* USART Oversmapling 8-bits */

/* Bit definition for USART_CR2 register */
enum USART_CR2_ADD = cast(ushort)0x000F; /* Address of the USART node */
enum USART_CR2_LBDL = cast(ushort)0x0020; /* LIN Break Detection Length */
enum USART_CR2_LBDIE = cast(ushort)0x0040; /* LIN Break Detection Interrupt Enable */
enum USART_CR2_LBCL = cast(ushort)0x0100; /* Last Bit Clock pulse */
enum USART_CR2_CPHA = cast(ushort)0x0200; /* Clock Phase */
enum USART_CR2_CPOL = cast(ushort)0x0400; /* Clock Polarity */
enum USART_CR2_CLKEN = cast(ushort)0x0800; /* Clock Enable */

enum USART_CR2_STOP = cast(ushort)0x3000; /* STOP[1:0] bits (STOP bits; */
enum USART_CR2_STOP_0 = cast(ushort)0x1000; /* Bit 0 */
enum USART_CR2_STOP_1 = cast(ushort)0x2000; /* Bit 1 */

enum USART_CR2_LINEN = cast(ushort)0x4000; /* LIN mode enable */

/* Bit definition for USART_CR3 register */
enum USART_CR3_EIE = cast(ushort)0x0001; /* Error Interrupt Enable */
enum USART_CR3_IREN = cast(ushort)0x0002; /* IrDA mode Enable */
enum USART_CR3_IRLP = cast(ushort)0x0004; /* IrDA Low-Power */
enum USART_CR3_HDSEL = cast(ushort)0x0008; /* Half-Duplex Selection */
enum USART_CR3_NACK = cast(ushort)0x0010; /* Smartcard NACK enable */
enum USART_CR3_SCEN = cast(ushort)0x0020; /* Smartcard mode enable */
enum USART_CR3_DMAR = cast(ushort)0x0040; /* DMA Enable Receiver */
enum USART_CR3_DMAT = cast(ushort)0x0080; /* DMA Enable Transmitter */
enum USART_CR3_RTSE = cast(ushort)0x0100; /* RTS Enable */
enum USART_CR3_CTSE = cast(ushort)0x0200; /* CTS Enable */
enum USART_CR3_CTSIE = cast(ushort)0x0400; /* CTS Interrupt Enable */
enum USART_CR3_ONEBIT = cast(ushort)0x0800; /* One Bit method */

/* Bit definition for USART_GTPR register */
enum USART_GTPR_PSC = cast(ushort)0x00FF; /* PSC[7:0] bits (Prescaler value; */
enum USART_GTPR_PSC_0 = cast(ushort)0x0001; /* Bit 0 */
enum USART_GTPR_PSC_1 = cast(ushort)0x0002; /* Bit 1 */
enum USART_GTPR_PSC_2 = cast(ushort)0x0004; /* Bit 2 */
enum USART_GTPR_PSC_3 = cast(ushort)0x0008; /* Bit 3 */
enum USART_GTPR_PSC_4 = cast(ushort)0x0010; /* Bit 4 */
enum USART_GTPR_PSC_5 = cast(ushort)0x0020; /* Bit 5 */
enum USART_GTPR_PSC_6 = cast(ushort)0x0040; /* Bit 6 */
enum USART_GTPR_PSC_7 = cast(ushort)0x0080; /* Bit 7 */

enum USART_GTPR_GT = cast(ushort)0xFF00; /* Guard time value */

/* Bit definition for DBGMCU_IDCODE register */
enum DBGMCU_IDCODE_DEV_ID = cast(uint)0x00000FFF; /* Device Identifier */

enum DBGMCU_IDCODE_REV_ID = cast(uint)0xFFFF0000; /* REV_ID[15:0] bits (Revision Identifier; */
enum DBGMCU_IDCODE_REV_ID_0 = cast(uint)0x00010000; /* Bit 0 */
enum DBGMCU_IDCODE_REV_ID_1 = cast(uint)0x00020000; /* Bit 1 */
enum DBGMCU_IDCODE_REV_ID_2 = cast(uint)0x00040000; /* Bit 2 */
enum DBGMCU_IDCODE_REV_ID_3 = cast(uint)0x00080000; /* Bit 3 */
enum DBGMCU_IDCODE_REV_ID_4 = cast(uint)0x00100000; /* Bit 4 */
enum DBGMCU_IDCODE_REV_ID_5 = cast(uint)0x00200000; /* Bit 5 */
enum DBGMCU_IDCODE_REV_ID_6 = cast(uint)0x00400000; /* Bit 6 */
enum DBGMCU_IDCODE_REV_ID_7 = cast(uint)0x00800000; /* Bit 7 */
enum DBGMCU_IDCODE_REV_ID_8 = cast(uint)0x01000000; /* Bit 8 */
enum DBGMCU_IDCODE_REV_ID_9 = cast(uint)0x02000000; /* Bit 9 */
enum DBGMCU_IDCODE_REV_ID_10 = cast(uint)0x04000000; /* Bit 10 */
enum DBGMCU_IDCODE_REV_ID_11 = cast(uint)0x08000000; /* Bit 11 */
enum DBGMCU_IDCODE_REV_ID_12 = cast(uint)0x10000000; /* Bit 12 */
enum DBGMCU_IDCODE_REV_ID_13 = cast(uint)0x20000000; /* Bit 13 */
enum DBGMCU_IDCODE_REV_ID_14 = cast(uint)0x40000000; /* Bit 14 */
enum DBGMCU_IDCODE_REV_ID_15 = cast(uint)0x80000000; /* Bit 15 */

/* Bit definition for DBGMCU_CR register */
enum DBGMCU_CR_DBG_SLEEP = cast(uint)0x00000001; /* Debug Sleep Mode */
enum DBGMCU_CR_DBG_STOP = cast(uint)0x00000002; /* Debug Stop Mode */
enum DBGMCU_CR_DBG_STANDBY = cast(uint)0x00000004; /* Debug Standby mode */
enum DBGMCU_CR_TRACE_IOEN = cast(uint)0x00000020; /* Trace Pin Assignment Control */

enum DBGMCU_CR_TRACE_MODE = cast(uint)0x000000C0; /* TRACE_MODE[1:0] bits (Trace Pin Assignment Control; */
enum DBGMCU_CR_TRACE_MODE_0 = cast(uint)0x00000040; /* Bit 0 */
enum DBGMCU_CR_TRACE_MODE_1 = cast(uint)0x00000080; /* Bit 1 */

enum DBGMCU_CR_DBG_IWDG_STOP = cast(uint)0x00000100; /* Debug Independent Watchdog stopped when Core is halted */
enum DBGMCU_CR_DBG_WWDG_STOP = cast(uint)0x00000200; /* Debug Window Watchdog stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM1_STOP = cast(uint)0x00000400; /* TIM1 counter stopped when core is halted */
enum DBGMCU_CR_DBG_TIM2_STOP = cast(uint)0x00000800; /* TIM2 counter stopped when core is halted */
enum DBGMCU_CR_DBG_TIM3_STOP = cast(uint)0x00001000; /* TIM3 counter stopped when core is halted */
enum DBGMCU_CR_DBG_TIM4_STOP = cast(uint)0x00002000; /* TIM4 counter stopped when core is halted */
enum DBGMCU_CR_DBG_CAN1_STOP = cast(uint)0x00004000; /* Debug CAN1 stopped when Core is halted */
enum DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT = cast(uint)0x00008000; /* SMBUS timeout mode stopped when Core is halted */
enum DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT = cast(uint)0x00010000; /* SMBUS timeout mode stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM8_STOP = cast(uint)0x00020000; /* TIM8 counter stopped when core is halted */
enum DBGMCU_CR_DBG_TIM5_STOP = cast(uint)0x00040000; /* TIM5 counter stopped when core is halted */
enum DBGMCU_CR_DBG_TIM6_STOP = cast(uint)0x00080000; /* TIM6 counter stopped when core is halted */
enum DBGMCU_CR_DBG_TIM7_STOP = cast(uint)0x00100000; /* TIM7 counter stopped when core is halted */
enum DBGMCU_CR_DBG_CAN2_STOP = cast(uint)0x00200000; /* Debug CAN2 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM15_STOP = cast(uint)0x00400000; /* Debug TIM15 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM16_STOP = cast(uint)0x00800000; /* Debug TIM16 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM17_STOP = cast(uint)0x01000000; /* Debug TIM17 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM12_STOP = cast(uint)0x02000000; /* Debug TIM12 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM13_STOP = cast(uint)0x04000000; /* Debug TIM13 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM14_STOP = cast(uint)0x08000000; /* Debug TIM14 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM9_STOP = cast(uint)0x10000000; /* Debug TIM9 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM10_STOP = cast(uint)0x20000000; /* Debug TIM10 stopped when Core is halted */
enum DBGMCU_CR_DBG_TIM11_STOP = cast(uint)0x40000000; /* Debug TIM11 stopped when Core is halted */

/* Bit definition for FLASH_ACR register */
enum FLASH_ACR_LATENCY = cast(ubyte)0x03; /* LATENCY[2:0] bits (Latency; */
enum FLASH_ACR_LATENCY_0 = cast(ubyte)0x00; /* Bit 0 */
enum FLASH_ACR_LATENCY_1 = cast(ubyte)0x01; /* Bit 0 */
enum FLASH_ACR_LATENCY_2 = cast(ubyte)0x02; /* Bit 1 */

enum FLASH_ACR_HLFCYA = cast(ubyte)0x08; /* Flash Half Cycle Access Enable */
enum FLASH_ACR_PRFTBE = cast(ubyte)0x10; /* Prefetch Buffer Enable */
enum FLASH_ACR_PRFTBS = cast(ubyte)0x20; /* Prefetch Buffer Status */

/* Bit definition for FLASH_KEYR register */
enum FLASH_KEYR_FKEYR = cast(uint)0xFFFFFFFF; /* FPEC Key */

/* FLASH Keys */
enum RDP_KEY = cast(ushort)0x00A5;
enum FLASH_KEY1 = cast(uint)0x45670123;
enum FLASH_KEY2 = cast(uint)0xCDEF89AB; /* Bit definition for FLASH_OPTKEYR register */
enum FLASH_OPTKEYR_OPTKEYR = cast(uint)0xFFFFFFFF; /* Option Byte Key */

/* Bit definition for FLASH_SR register */
enum FLASH_SR_BSY = cast(ubyte)0x01; /* Busy */
enum FLASH_SR_PGERR = cast(ubyte)0x04; /* Programming Error */
enum FLASH_SR_WRPRTERR = cast(ubyte)0x10; /* Write Protection Error */
enum FLASH_SR_EOP = cast(ubyte)0x20; /* End of operation */

/* Bit definition for FLASH_CR register */
enum FLASH_CR_PG = cast(ushort)0x0001; /* Programming */
enum FLASH_CR_PER = cast(ushort)0x0002; /* Page Erase */
enum FLASH_CR_MER = cast(ushort)0x0004; /* Mass Erase */
enum FLASH_CR_OPTPG = cast(ushort)0x0010; /* Option Byte Programming */
enum FLASH_CR_OPTER = cast(ushort)0x0020; /* Option Byte Erase */
enum FLASH_CR_STRT = cast(ushort)0x0040; /* Start */
enum FLASH_CR_LOCK = cast(ushort)0x0080; /* Lock */
enum FLASH_CR_OPTWRE = cast(ushort)0x0200; /* Option Bytes Write Enable */
enum FLASH_CR_ERRIE = cast(ushort)0x0400; /* Error Interrupt Enable */
enum FLASH_CR_EOPIE = cast(ushort)0x1000; /* End of operation interrupt enable */

/* Bit definition for FLASH_AR register */
enum FLASH_AR_FAR = cast(uint)0xFFFFFFFF; /* Flash Address */

/* Bit definition for FLASH_OBR register */
enum FLASH_OBR_OPTERR = cast(ushort)0x0001; /* Option Byte Error */
enum FLASH_OBR_RDPRT = cast(ushort)0x0002; /* Read protection */

enum FLASH_OBR_USER = cast(ushort)0x03FC; /* User Option Bytes */
enum FLASH_OBR_WDG_SW = cast(ushort)0x0004; /* WDG_SW */
enum FLASH_OBR_N_RST_STOP = cast(ushort)0x0008; /* nRST_STOP */
enum FLASH_OBR_N_RST_STDBY = cast(ushort)0x0010; /* nRST_STDBY */
enum FLASH_OBR_BFB2 = cast(ushort)0x0020; /* BFB2 */

/* Bit definition for FLASH_WRPR register */
enum FLASH_WRPR_WRP = cast(uint)0xFFFFFFFF; /* Write Protect */

/* Bit definition for FLASH_RDP register */
enum FLASH_RDP_RDP = cast(uint)0x000000FF; /* Read protection option byte */
enum FLASH_RDP_N_RDP = cast(uint)0x0000FF00; /* Read protection complemented option byte */

/* Bit definition for FLASH_USER register */
enum FLASH_USER_USER = cast(uint)0x00FF0000; /* User option byte */
enum FLASH_USER_N_USER = cast(uint)0xFF000000; /* User complemented option byte */

/* Bit definition for FLASH_Data0 register */
enum FLASH_DATA0_DATA0 = cast(uint)0x000000FF; /* User data storage option byte */
enum FLASH_DATA0_N_DATA0 = cast(uint)0x0000FF00; /* User data storage complemented option byte */

/* Bit definition for FLASH_Data1 register */
enum FLASH_DATA1_DATA1 = cast(uint)0x00FF0000; /* User data storage option byte */
enum FLASH_DATA1_N_DATA1 = cast(uint)0xFF000000; /* User data storage complemented option byte */

/* Bit definition for FLASH_WRP0 register */
enum FLASH_WRP0_WRP0 = cast(uint)0x000000FF; /* Flash memory write protection option bytes */
enum FLASH_WRP0_N_WRP0 = cast(uint)0x0000FF00; /* Flash memory write protection complemented option bytes */

/* Bit definition for FLASH_WRP1 register */
enum FLASH_WRP1_WRP1 = cast(uint)0x00FF0000; /* Flash memory write protection option bytes */
enum FLASH_WRP1_N_WRP1 = cast(uint)0xFF000000; /* Flash memory write protection complemented option bytes */

/* Bit definition for FLASH_WRP2 register */
enum FLASH_WRP2_WRP2 = cast(uint)0x000000FF; /* Flash memory write protection option bytes */
enum FLASH_WRP2_N_WRP2 = cast(uint)0x0000FF00; /* Flash memory write protection complemented option bytes */

/* Bit definition for FLASH_WRP3 register */
enum FLASH_WRP3_WRP3 = cast(uint)0x00FF0000; /* Flash memory write protection option bytes */
enum FLASH_WRP3_N_WRP3 = cast(uint)0xFF000000; /* Flash memory write protection complemented option bytes */

/* Bit definition for Ethernet MAC Control Register register */
enum ETH_MACCR_WD = cast(uint)0x00800000; /* Watchdog disable */
enum ETH_MACCR_JD = cast(uint)0x00400000; /* Jabber disable */
enum ETH_MACCR_IFG = cast(uint)0x000E0000; /* Inter-frame gap */
enum ETH_MACCR_IFG_96_BIT = cast(uint)0x00000000; /* Minimum IFG between frames during transmission is 96Bit */
enum ETH_MACCR_IFG_88_BIT = cast(uint)0x00020000; /* Minimum IFG between frames during transmission is 88Bit */
enum ETH_MACCR_IFG_80_BIT = cast(uint)0x00040000; /* Minimum IFG between frames during transmission is 80Bit */
enum ETH_MACCR_IFG_72_BIT = cast(uint)0x00060000; /* Minimum IFG between frames during transmission is 72Bit */
enum ETH_MACCR_IFG_64_BIT = cast(uint)0x00080000; /* Minimum IFG between frames during transmission is 64Bit */
enum ETH_MACCR_IFG_56_BIT = cast(uint)0x000A0000; /* Minimum IFG between frames during transmission is 56Bit */
enum ETH_MACCR_IFG_48_BIT = cast(uint)0x000C0000; /* Minimum IFG between frames during transmission is 48Bit */
enum ETH_MACCR_IFG_40_BIT = cast(uint)0x000E0000; /* Minimum IFG between frames during transmission is 40Bit */
enum ETH_MACCR_CSD = cast(uint)0x00010000; /* Carrier sense disable (during transmission; */
enum ETH_MACCR_FES = cast(uint)0x00004000; /* Fast ethernet speed */
enum ETH_MACCR_ROD = cast(uint)0x00002000; /* Receive own disable */
enum ETH_MACCR_LM = cast(uint)0x00001000; /* loopback mode */
enum ETH_MACCR_DM = cast(uint)0x00000800; /* Duplex mode */
enum ETH_MACCR_IPCO = cast(uint)0x00000400; /* IP Checksum offload */
enum ETH_MACCR_RD = cast(uint)0x00000200; /* Retry disable */
enum ETH_MACCR_APCS = cast(uint)0x00000080; /* Automatic Pad/CRC stripping */
enum ETH_MACCR_BL = cast(uint)0x00000060; /* Back-off limit: random integer number (r; of slot time delays before rescheduling a transmission attempt during retries after a collision: 0 =< r <2^k */
enum ETH_MACCR_BL_10 = cast(uint)0x00000000; /* k = min (n, 10; */
enum ETH_MACCR_BL_8 = cast(uint)0x00000020; /* k = min (n, 8; */
enum ETH_MACCR_BL_4 = cast(uint)0x00000040; /* k = min (n, 4; */
enum ETH_MACCR_BL_1 = cast(uint)0x00000060; /* k = min (n, 1; */
enum ETH_MACCR_DC = cast(uint)0x00000010; /* Defferal check */
enum ETH_MACCR_TE = cast(uint)0x00000008; /* Transmitter enable */
enum ETH_MACCR_RE = cast(uint)0x00000004; /* Receiver enable */

/* Bit definition for Ethernet MAC Frame Filter Register */
enum ETH_MACFFR_RA = cast(uint)0x80000000; /* Receive all */
enum ETH_MACFFR_HPF = cast(uint)0x00000400; /* Hash or perfect filter */
enum ETH_MACFFR_SAF = cast(uint)0x00000200; /* Source address filter enable */
enum ETH_MACFFR_SAIF = cast(uint)0x00000100; /* SA inverse filtering */
enum ETH_MACFFR_PCF = cast(uint)0x000000C0; /* Pass control frames: 3 cases */
enum ETH_MACFFR_PCF_BLOCK_ALL = cast(uint)0x00000040; /* MAC filters all control frames from reaching the application */
enum ETH_MACFFR_PCF_FORWARD_ALL = cast(uint)0x00000080; /* MAC forwards all control frames to application even if they fail the Address Filter */
enum ETH_MACFFR_PCF_FORWARD_PASSED_ADDR_FILTER = cast(uint)0x000000C0; /* MAC forwards control frames that pass the Address Filter. */
enum ETH_MACFFR_BFD = cast(uint)0x00000020; /* Broadcast frame disable */
enum ETH_MACFFR_PAM = cast(uint)0x00000010; /* Pass all mutlicast */
enum ETH_MACFFR_DAIF = cast(uint)0x00000008; /* DA Inverse filtering */
enum ETH_MACFFR_HM = cast(uint)0x00000004; /* Hash multicast */
enum ETH_MACFFR_HU = cast(uint)0x00000002; /* Hash unicast */
enum ETH_MACFFR_PM = cast(uint)0x00000001; /* Promiscuous mode */

/* Bit definition for Ethernet MAC Hash Table High Register */
enum ETH_MACHTHR_HTH = cast(uint)0xFFFFFFFF; /* Hash table high */

/* Bit definition for Ethernet MAC Hash Table Low Register */
enum ETH_MACHTLR_HTL = cast(uint)0xFFFFFFFF; /* Hash table low */

/* Bit definition for Ethernet MAC MII Address Register */
enum ETH_MACMIIAR_PA = cast(uint)0x0000F800; /* Physical layer address */
enum ETH_MACMIIAR_MR = cast(uint)0x000007C0; /* MII register in the selected PHY */
enum ETH_MACMIIAR_CR = cast(uint)0x0000001C; /* CR clock range: 6 cases */
enum ETH_MACMIIAR_CR_DIV42 = cast(uint)0x00000000; /* HCLK:60-72 MHz; MDC clock= HCLK/42 */
enum ETH_MACMIIAR_CR_DIV16 = cast(uint)0x00000008; /* HCLK:20-35 MHz; MDC clock= HCLK/16 */
enum ETH_MACMIIAR_CR_DIV26 = cast(uint)0x0000000C; /* HCLK:35-60 MHz; MDC clock= HCLK/26 */
enum ETH_MACMIIAR_MW = cast(uint)0x00000002; /* MII write */
enum ETH_MACMIIAR_MB = cast(uint)0x00000001; /* MII busy */

/* Bit definition for Ethernet MAC MII Data Register */
enum ETH_MACMIIDR_MD = cast(uint)0x0000FFFF; /* MII data: read/write data from/to PHY */

/* Bit definition for Ethernet MAC Flow Control Register */
enum ETH_MACFCR_PT = cast(uint)0xFFFF0000; /* Pause time */
enum ETH_MACFCR_ZQPD = cast(uint)0x00000080; /* Zero-quanta pause disable */
enum ETH_MACFCR_PLT = cast(uint)0x00000030; /* Pause low threshold: 4 cases */
enum ETH_MACFCR_PLT_MINUS4 = cast(uint)0x00000000; /* Pause time minus 4 slot times */
enum ETH_MACFCR_PLT_MINUS28 = cast(uint)0x00000010; /* Pause time minus 28 slot times */
enum ETH_MACFCR_PLT_MINUS144 = cast(uint)0x00000020; /* Pause time minus 144 slot times */
enum ETH_MACFCR_PLT_MINUS256 = cast(uint)0x00000030; /* Pause time minus 256 slot times */
enum ETH_MACFCR_UPFD = cast(uint)0x00000008; /* Unicast pause frame detect */
enum ETH_MACFCR_RFCE = cast(uint)0x00000004; /* Receive flow control enable */
enum ETH_MACFCR_TFCE = cast(uint)0x00000002; /* Transmit flow control enable */
enum ETH_MACFCR_FCBBPA = cast(uint)0x00000001; /* Flow control busy/backpressure activate */

/* Bit definition for Ethernet MAC VLAN Tag Register */
enum ETH_MACVLANTR_VLANTC = cast(uint)0x00010000; /* 12-bit VLAN tag comparison */
enum ETH_MACVLANTR_VLANTI = cast(uint)0x0000FFFF; /* VLAN tag identifier (for receive frames; */

/* Bit definition for Ethernet MAC Remote Wake-UpFrame Filter Register */
enum ETH_MACRWUFFR_D = cast(uint)0xFFFFFFFF; /* Wake-up frame filter register data */
/**
 * Eight sequential Writes to this address (offset 0x28; will write all Wake-UpFrame Filter Registers.
 * Eight sequential Reads from this address (offset 0x28; will read all Wake-UpFrame Filter Registers. 
 * Wake-UpFrame Filter Reg0 : Filter 0 Byte Mask
 * Wake-UpFrame Filter Reg1 : Filter 1 Byte Mask
 * Wake-UpFrame Filter Reg2 : Filter 2 Byte Mask
 * Wake-UpFrame Filter Reg3 : Filter 3 Byte Mask
 * Wake-UpFrame Filter Reg4 : RSVD - Filter3 Command - RSVD - Filter2 Command -
 * RSVD - Filter1 Command - RSVD - Filter0 Command
 * Wake-UpFrame Filter Re5 : Filter3 Offset - Filter2 Offset - Filter1 Offset - Filter0 Offset
 * Wake-UpFrame Filter Re6 : Filter1 CRC16 - Filter0 CRC16
 * Wake-UpFrame Filter Re7 : Filter3 CRC16 - Filter2 CRC16 
 */

/* Bit definition for Ethernet MAC PMT Control and Status Register */
enum ETH_MACPMTCSR_WFFRPR = cast(uint)0x80000000; /* Wake-Up Frame Filter Register Pointer Reset */
enum ETH_MACPMTCSR_GU = cast(uint)0x00000200; /* Global Unicast */
enum ETH_MACPMTCSR_WFR = cast(uint)0x00000040; /* Wake-Up Frame Received */
enum ETH_MACPMTCSR_MPR = cast(uint)0x00000020; /* Magic Packet Received */
enum ETH_MACPMTCSR_WFE = cast(uint)0x00000004; /* Wake-Up Frame Enable */
enum ETH_MACPMTCSR_MPE = cast(uint)0x00000002; /* Magic Packet Enable */
enum ETH_MACPMTCSR_PD = cast(uint)0x00000001; /* Power Down */

/* Bit definition for Ethernet MAC Status Register */
enum ETH_MACSR_TSTS = cast(uint)0x00000200; /* Time stamp trigger status */
enum ETH_MACSR_MMCTS = cast(uint)0x00000040; /* MMC transmit status */
enum ETH_MACSR_MMMCRS = cast(uint)0x00000020; /* MMC receive status */
enum ETH_MACSR_MMCS = cast(uint)0x00000010; /* MMC status */
enum ETH_MACSR_PMTS = cast(uint)0x00000008; /* PMT status */

/* Bit definition for Ethernet MAC Interrupt Mask Register */
enum ETH_MACIMR_TSTIM = cast(uint)0x00000200; /* Time stamp trigger interrupt mask */
enum ETH_MACIMR_PMTIM = cast(uint)0x00000008; /* PMT interrupt mask */

/* Bit definition for Ethernet MAC Address0 High Register */
enum ETH_MACA0HR_MACA0H = cast(uint)0x0000FFFF; /* MAC address0 high */

/* Bit definition for Ethernet MAC Address0 Low Register */
enum ETH_MACA0LR_MACA0L = cast(uint)0xFFFFFFFF; /* MAC address0 low */

/* Bit definition for Ethernet MAC Address1 High Register */
enum ETH_MACA1HR_AE = cast(uint)0x80000000; /* Address enable */
enum ETH_MACA1HR_SA = cast(uint)0x40000000; /* Source address */
enum ETH_MACA1HR_MBC = cast(uint)0x3F000000; /* Mask byte control: bits to mask for comparison of the MAC Address bytes */
enum ETH_MACA1HR_MBC_H_BITS15_8 = cast(uint)0x20000000; /* Mask MAC Address high reg bits [15:8] */
enum ETH_MACA1HR_MBC_H_BITS7_0 = cast(uint)0x10000000; /* Mask MAC Address high reg bits [7:0] */
enum ETH_MACA1HR_MBC_L_BITS31_24 = cast(uint)0x08000000; /* Mask MAC Address low reg bits [31:24] */
enum ETH_MACA1HR_MBC_L_BITS23_16 = cast(uint)0x04000000; /* Mask MAC Address low reg bits [23:16] */
enum ETH_MACA1HR_MBC_L_BITS15_8 = cast(uint)0x02000000; /* Mask MAC Address low reg bits [15:8] */
enum ETH_MACA1HR_MBC_L_BITS7_0 = cast(uint)0x01000000; /* Mask MAC Address low reg bits [7:0] */
enum ETH_MACA1HR_MACA1H = cast(uint)0x0000FFFF; /* MAC address1 high */

/* Bit definition for Ethernet MAC Address1 Low Register */
enum ETH_MACA1LR_MACA1L = cast(uint)0xFFFFFFFF; /* MAC address1 low */

/* Bit definition for Ethernet MAC Address2 High Register */
enum ETH_MACA2HR_AE = cast(uint)0x80000000; /* Address enable */
enum ETH_MACA2HR_SA = cast(uint)0x40000000; /* Source address */
enum ETH_MACA2HR_MBC = cast(uint)0x3F000000; /* Mask byte control */
enum ETH_MACA2HR_MBC_H_BITS15_8 = cast(uint)0x20000000; /* Mask MAC Address high reg bits [15:8] */
enum ETH_MACA2HR_MBC_H_BITS7_0 = cast(uint)0x10000000; /* Mask MAC Address high reg bits [7:0] */
enum ETH_MACA2HR_MBC_L_BITS31_24 = cast(uint)0x08000000; /* Mask MAC Address low reg bits [31:24] */
enum ETH_MACA2HR_MBC_L_BITS23_16 = cast(uint)0x04000000; /* Mask MAC Address low reg bits [23:16] */
enum ETH_MACA2HR_MBC_L_BITS15_8 = cast(uint)0x02000000; /* Mask MAC Address low reg bits [15:8] */
enum ETH_MACA2HR_MBC_L_BITS7_0 = cast(uint)0x01000000; /* Mask MAC Address low reg bits [70] */
enum ETH_MACA2HR_MACA2H = cast(uint)0x0000FFFF; /* MAC address1 high */

/* Bit definition for Ethernet MAC Address2 Low Register */
enum ETH_MACA2LR_MACA2L = cast(uint)0xFFFFFFFF; /* MAC address2 low */

/* Bit definition for Ethernet MAC Address3 High Register */
enum ETH_MACA3HR_AE = cast(uint)0x80000000; /* Address enable */
enum ETH_MACA3HR_SA = cast(uint)0x40000000; /* Source address */
enum ETH_MACA3HR_MBC = cast(uint)0x3F000000; /* Mask byte control */
enum ETH_MACA3HR_MBC_H_BITS15_8 = cast(uint)0x20000000; /* Mask MAC Address high reg bits [15:8] */
enum ETH_MACA3HR_MBC_H_BITS7_0 = cast(uint)0x10000000; /* Mask MAC Address high reg bits [7:0] */
enum ETH_MACA3HR_MBC_L_BITS31_24 = cast(uint)0x08000000; /* Mask MAC Address low reg bits [31:24] */
enum ETH_MACA3HR_MBC_L_BITS23_16 = cast(uint)0x04000000; /* Mask MAC Address low reg bits [23:16] */
enum ETH_MACA3HR_MBC_L_BITS15_8 = cast(uint)0x02000000; /* Mask MAC Address low reg bits [15:8] */
enum ETH_MACA3HR_MBC_L_BITS7_0 = cast(uint)0x01000000; /* Mask MAC Address low reg bits [70] */
enum ETH_MACA3HR_MACA3H = cast(uint)0x0000FFFF; /* MAC address3 high */

/* Bit definition for Ethernet MAC Address3 Low Register */
enum ETH_MACA3LR_MACA3L = cast(uint)0xFFFFFFFF; /* MAC address3 low */

/* Bit definition for Ethernet MMC Contol Register */
enum ETH_MMCCR_MCF = cast(uint)0x00000008; /* MMC Counter Freeze */
enum ETH_MMCCR_ROR = cast(uint)0x00000004; /* Reset on Read */
enum ETH_MMCCR_CSR = cast(uint)0x00000002; /* Counter Stop Rollover */
enum ETH_MMCCR_CR = cast(uint)0x00000001; /* Counters Reset */

/* Bit definition for Ethernet MMC Receive Interrupt Register */
enum ETH_MMCRIR_RGUFS = cast(uint)0x00020000; /* Set when Rx good unicast frames counter reaches half the maximum value */
enum ETH_MMCRIR_RFAES = cast(uint)0x00000040; /* Set when Rx alignment error counter reaches half the maximum value */
enum ETH_MMCRIR_RFCES = cast(uint)0x00000020; /* Set when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Register */
enum ETH_MMCTIR_TGFS = cast(uint)0x00200000; /* Set when Tx good frame count counter reaches half the maximum value */
enum ETH_MMCTIR_TGFMSCS = cast(uint)0x00008000; /* Set when Tx good multi col counter reaches half the maximum value */
enum ETH_MMCTIR_TGFSCS = cast(uint)0x00004000; /* Set when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Receive Interrupt Mask Register */
enum ETH_MMCRIMR_RGUFM = cast(uint)0x00020000; /* Mask the interrupt when Rx good unicast frames counter reaches half the maximum value */
enum ETH_MMCRIMR_RFAEM = cast(uint)0x00000040; /* Mask the interrupt when when Rx alignment error counter reaches half the maximum value */
enum ETH_MMCRIMR_RFCEM = cast(uint)0x00000020; /* Mask the interrupt when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Mask Register */
enum ETH_MMCTIMR_TGFM = cast(uint)0x00200000; /* Mask the interrupt when Tx good frame count counter reaches half the maximum value */
enum ETH_MMCTIMR_TGFMSCM = cast(uint)0x00008000; /* Mask the interrupt when Tx good multi col counter reaches half the maximum value */
enum ETH_MMCTIMR_TGFSCM = cast(uint)0x00004000; /* Mask the interrupt when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmitted Good Frames after Single Collision Counter Register */
enum ETH_MMCTGFSCCR_TGFSCC = cast(uint)0xFFFFFFFF; /* Number of successfully transmitted frames after a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames after More than a Single Collision Counter Register */
enum ETH_MMCTGFMSCCR_TGFMSCC = cast(uint)0xFFFFFFFF; /* Number of successfully transmitted frames after more than a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames Counter Register */
enum ETH_MMCTGFCR_TGFC = cast(uint)0xFFFFFFFF; /* Number of good frames transmitted. */

/* Bit definition for Ethernet MMC Received Frames with CRC Error Counter Register */
enum ETH_MMCRFCECR_RFCEC = cast(uint)0xFFFFFFFF; /* Number of frames received with CRC error. */

/* Bit definition for Ethernet MMC Received Frames with Alignement Error Counter Register */
enum ETH_MMCRFAECR_RFAEC = cast(uint)0xFFFFFFFF; /* Number of frames received with alignment (dribble; error */

/* Bit definition for Ethernet MMC Received Good Unicast Frames Counter Register */
enum ETH_MMCRGUFCR_RGUFC = cast(uint)0xFFFFFFFF; /* Number of good unicast frames received. */

/* Bit definition for Ethernet PTP Time Stamp Contol Register */
enum ETH_PTPTSCR_TSARU = cast(uint)0x00000020; /* Addend register update */
enum ETH_PTPTSCR_TSITE = cast(uint)0x00000010; /* Time stamp interrupt trigger enable */
enum ETH_PTPTSCR_TSSTU = cast(uint)0x00000008; /* Time stamp update */
enum ETH_PTPTSCR_TSSTI = cast(uint)0x00000004; /* Time stamp initialize */
enum ETH_PTPTSCR_TSFCU = cast(uint)0x00000002; /* Time stamp fine or coarse update */
enum ETH_PTPTSCR_TSE = cast(uint)0x00000001; /* Time stamp enable */

/* Bit definition for Ethernet PTP Sub-Second Increment Register */
enum ETH_PTPSSIR_STSSI = cast(uint)0x000000FF; /* System time Sub-second increment value */

/* Bit definition for Ethernet PTP Time Stamp High Register */
enum ETH_PTPTSHR_STS = cast(uint)0xFFFFFFFF; /* System Time second */

/* Bit definition for Ethernet PTP Time Stamp Low Register */
enum ETH_PTPTSLR_STPNS = cast(uint)0x80000000; /* System Time Positive or negative time */
enum ETH_PTPTSLR_STSS = cast(uint)0x7FFFFFFF; /* System Time sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp High Update Register */
enum ETH_PTPTSHUR_TSUS = cast(uint)0xFFFFFFFF; /* Time stamp update seconds */

/* Bit definition for Ethernet PTP Time Stamp Low Update Register */
enum ETH_PTPTSLUR_TSUPNS = cast(uint)0x80000000; /* Time stamp update Positive or negative time */
enum ETH_PTPTSLUR_TSUSS = cast(uint)0x7FFFFFFF; /* Time stamp update sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp Addend Register */
enum ETH_PTPTSAR_TSA = cast(uint)0xFFFFFFFF; /* Time stamp addend */

/* Bit definition for Ethernet PTP Target Time High Register */
enum ETH_PTPTTHR_TTSH = cast(uint)0xFFFFFFFF; /* Target time stamp high */

/* Bit definition for Ethernet PTP Target Time Low Register */
enum ETH_PTPTTLR_TTSL = cast(uint)0xFFFFFFFF; /* Target time stamp low */

/* Bit definition for Ethernet DMA Bus Mode Register */
enum ETH_DMABMR_AAB = cast(uint)0x02000000; /* Address-Aligned beats */
enum ETH_DMABMR_FPM = cast(uint)0x01000000; /* 4xPBL mode */
enum ETH_DMABMR_USP = cast(uint)0x00800000; /* Use separate PBL */
enum ETH_DMABMR_RDP = cast(uint)0x007E0000; /* RxDMA PBL */
enum ETH_DMABMR_RDP_1BEAT = cast(uint)0x00020000; /* maximum number of beats to be transferred in one RxDMA transaction is 1 */
enum ETH_DMABMR_RDP_2BEAT = cast(uint)0x00040000; /* maximum number of beats to be transferred in one RxDMA transaction is 2 */
enum ETH_DMABMR_RDP_4BEAT = cast(uint)0x00080000; /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
enum ETH_DMABMR_RDP_8BEAT = cast(uint)0x00100000; /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
enum ETH_DMABMR_RDP_16BEAT = cast(uint)0x00200000; /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
enum ETH_DMABMR_RDP_32BEAT = cast(uint)0x00400000; /* maximum number of beats to be transferred in one RxDMA transaction is 32 */
enum ETH_DMABMR_RDP_4X_PBL_4BEAT = cast(uint)0x01020000; /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
enum ETH_DMABMR_RDP_4X_PBL_8BEAT = cast(uint)0x01040000; /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
enum ETH_DMABMR_RDP_4X_PBL_16BEAT = cast(uint)0x01080000; /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
enum ETH_DMABMR_RDP_4X_PBL_32BEAT = cast(uint)0x01100000; /* maximum number of beats to be transferred in one RxDMA transaction is 32 */
enum ETH_DMABMR_RDP_4X_PBL_64BEAT = cast(uint)0x01200000; /* maximum number of beats to be transferred in one RxDMA transaction is 64 */
enum ETH_DMABMR_RDP_4X_PBL_128BEAT = cast(uint)0x01400000; /* maximum number of beats to be transferred in one RxDMA transaction is 128 */
enum ETH_DMABMR_FB = cast(uint)0x00010000; /* Fixed Burst */
enum ETH_DMABMR_RTPR = cast(uint)0x0000C000; /* Rx Tx priority ratio */
enum ETH_DMABMR_RTPR_1_1 = cast(uint)0x00000000; /* Rx Tx priority ratio */
enum ETH_DMABMR_RTPR_2_1 = cast(uint)0x00004000; /* Rx Tx priority ratio */
enum ETH_DMABMR_RTPR_3_1 = cast(uint)0x00008000; /* Rx Tx priority ratio */
enum ETH_DMABMR_RTPR_4_1 = cast(uint)0x0000C000; /* Rx Tx priority ratio */
enum ETH_DMABMR_PBL = cast(uint)0x00003F00; /* Programmable burst length */
enum ETH_DMABMR_PBL_1BEAT = cast(uint)0x00000100; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 1 */
enum ETH_DMABMR_PBL_2BEAT = cast(uint)0x00000200; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 2 */
enum ETH_DMABMR_PBL_4BEAT = cast(uint)0x00000400; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 4 */
enum ETH_DMABMR_PBL_8BEAT = cast(uint)0x00000800; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 8 */
enum ETH_DMABMR_PBL_16BEAT = cast(uint)0x00001000; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 16 */
enum ETH_DMABMR_PBL_32BEAT = cast(uint)0x00002000; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 32 */
enum ETH_DMABMR_PBL_4X_PBL_4BEAT = cast(uint)0x01000100; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 4 */
enum ETH_DMABMR_PBL_4X_PBL_8BEAT = cast(uint)0x01000200; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 8 */
enum ETH_DMABMR_PBL_4X_PBL_16BEAT = cast(uint)0x01000400; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 16 */
enum ETH_DMABMR_PBL_4X_PBL_32BEAT = cast(uint)0x01000800; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 32 */
enum ETH_DMABMR_PBL_4X_PBL_64BEAT = cast(uint)0x01001000; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 64 */
enum ETH_DMABMR_PBL_4X_PBL_128BEAT = cast(uint)0x01002000; /* maximum number of beats to be transferred in one TxDMA (or both; transaction is 128 */
enum ETH_DMABMR_DSL = cast(uint)0x0000007C; /* Descriptor Skip Length */
enum ETH_DMABMR_DA = cast(uint)0x00000002; /* DMA arbitration scheme */
enum ETH_DMABMR_SR = cast(uint)0x00000001; /* Software reset */

/* Bit definition for Ethernet DMA Transmit Poll Demand Register */
enum ETH_DMATPDR_TPD = cast(uint)0xFFFFFFFF; /* Transmit poll demand */

/* Bit definition for Ethernet DMA Receive Poll Demand Register */
enum ETH_DMARPDR_RPD = cast(uint)0xFFFFFFFF; /* Receive poll demand */

/* Bit definition for Ethernet DMA Receive Descriptor List Address Register */
enum ETH_DMARDLAR_SRL = cast(uint)0xFFFFFFFF; /* Start of receive list */

/* Bit definition for Ethernet DMA Transmit Descriptor List Address Register */
enum ETH_DMATDLAR_STL = cast(uint)0xFFFFFFFF; /* Start of transmit list */

/* Bit definition for Ethernet DMA Status Register */
enum ETH_DMASR_TSTS = cast(uint)0x20000000; /* Time-stamp trigger status */
enum ETH_DMASR_PMTS = cast(uint)0x10000000; /* PMT status */
enum ETH_DMASR_MMCS = cast(uint)0x08000000; /* MMC status */
enum ETH_DMASR_EBS = cast(uint)0x03800000; /* Error bits status */
/* combination with EBS[2:0] for GetFlagStatus function */
enum ETH_DMASR_EBS_DESC_ACCESS = cast(uint)0x02000000; /* Error bits 0-data buffer, 1-desc. access */
enum ETH_DMASR_EBS_READ_TRANSF = cast(uint)0x01000000; /* Error bits 0-write trnsf, 1-read transfr */
enum ETH_DMASR_EBS_DATA_TRANSF_TX = cast(uint)0x00800000; /* Error bits 0-Rx DMA, 1-Tx DMA */
enum ETH_DMASR_TPS = cast(uint)0x00700000; /* Transmit process state */
enum ETH_DMASR_TPS_STOPPED = cast(uint)0x00000000; /* Stopped - Reset or Stop Tx Command issued */
enum ETH_DMASR_TPS_FETCHING = cast(uint)0x00100000; /* Running - fetching the Tx descriptor */
enum ETH_DMASR_TPS_WAITING = cast(uint)0x00200000; /* Running - waiting for status */
enum ETH_DMASR_TPS_READING = cast(uint)0x00300000; /* Running - reading the data from host memory */
enum ETH_DMASR_TPS_SUSPENDED = cast(uint)0x00600000; /* Suspended - Tx Descriptor unavailabe */
enum ETH_DMASR_TPS_CLOSING = cast(uint)0x00700000; /* Running - closing Rx descriptor */
enum ETH_DMASR_RPS = cast(uint)0x000E0000; /* Receive process state */
enum ETH_DMASR_RPS_STOPPED = cast(uint)0x00000000; /* Stopped - Reset or Stop Rx Command issued */
enum ETH_DMASR_RPS_FETCHING = cast(uint)0x00020000; /* Running - fetching the Rx descriptor */
enum ETH_DMASR_RPS_WAITING = cast(uint)0x00060000; /* Running - waiting for packet */
enum ETH_DMASR_RPS_SUSPENDED = cast(uint)0x00080000; /* Suspended - Rx Descriptor unavailable */
enum ETH_DMASR_RPS_CLOSING = cast(uint)0x000A0000; /* Running - closing descriptor */
enum ETH_DMASR_RPS_QUEUING = cast(uint)0x000E0000; /* Running - queuing the recieve frame into host memory */
enum ETH_DMASR_NIS = cast(uint)0x00010000; /* Normal interrupt summary */
enum ETH_DMASR_AIS = cast(uint)0x00008000; /* Abnormal interrupt summary */
enum ETH_DMASR_ERS = cast(uint)0x00004000; /* Early receive status */
enum ETH_DMASR_FBES = cast(uint)0x00002000; /* Fatal bus error status */
enum ETH_DMASR_ETS = cast(uint)0x00000400; /* Early transmit status */
enum ETH_DMASR_RWTS = cast(uint)0x00000200; /* Receive watchdog timeout status */
enum ETH_DMASR_RPSS = cast(uint)0x00000100; /* Receive process stopped status */
enum ETH_DMASR_RBUS = cast(uint)0x00000080; /* Receive buffer unavailable status */
enum ETH_DMASR_RS = cast(uint)0x00000040; /* Receive status */
enum ETH_DMASR_TUS = cast(uint)0x00000020; /* Transmit underflow status */
enum ETH_DMASR_ROS = cast(uint)0x00000010; /* Receive overflow status */
enum ETH_DMASR_TJTS = cast(uint)0x00000008; /* Transmit jabber timeout status */
enum ETH_DMASR_TBUS = cast(uint)0x00000004; /* Transmit buffer unavailable status */
enum ETH_DMASR_TPSS = cast(uint)0x00000002; /* Transmit process stopped status */
enum ETH_DMASR_TS = cast(uint)0x00000001; /* Transmit status */

/* Bit definition for Ethernet DMA Operation Mode Register */
enum ETH_DMAOMR_DTCEFD = cast(uint)0x04000000; /* Disable Dropping of TCP/IP checksum error frames */
enum ETH_DMAOMR_RSF = cast(uint)0x02000000; /* Receive store and forward */
enum ETH_DMAOMR_DFRF = cast(uint)0x01000000; /* Disable flushing of received frames */
enum ETH_DMAOMR_TSF = cast(uint)0x00200000; /* Transmit store and forward */
enum ETH_DMAOMR_FTF = cast(uint)0x00100000; /* Flush transmit FIFO */
enum ETH_DMAOMR_TTC = cast(uint)0x0001C000; /* Transmit threshold control */
enum ETH_DMAOMR_TTC_64_BYTES = cast(uint)0x00000000; /* threshold level of the MTL Transmit FIFO is 64 Bytes */
enum ETH_DMAOMR_TTC_128_BYTES = cast(uint)0x00004000; /* threshold level of the MTL Transmit FIFO is 128 Bytes */
enum ETH_DMAOMR_TTC_192_BYTES = cast(uint)0x00008000; /* threshold level of the MTL Transmit FIFO is 192 Bytes */
enum ETH_DMAOMR_TTC_256_BYTES = cast(uint)0x0000C000; /* threshold level of the MTL Transmit FIFO is 256 Bytes */
enum ETH_DMAOMR_TTC_40_BYTES = cast(uint)0x00010000; /* threshold level of the MTL Transmit FIFO is 40 Bytes */
enum ETH_DMAOMR_TTC_32_BYTES = cast(uint)0x00014000; /* threshold level of the MTL Transmit FIFO is 32 Bytes */
enum ETH_DMAOMR_TTC_24_BYTES = cast(uint)0x00018000; /* threshold level of the MTL Transmit FIFO is 24 Bytes */
enum ETH_DMAOMR_TTC_16_BYTES = cast(uint)0x0001C000; /* threshold level of the MTL Transmit FIFO is 16 Bytes */
enum ETH_DMAOMR_ST = cast(uint)0x00002000; /* Start/stop transmission command */
enum ETH_DMAOMR_FEF = cast(uint)0x00000080; /* Forward error frames */
enum ETH_DMAOMR_FUGF = cast(uint)0x00000040; /* Forward undersized good frames */
enum ETH_DMAOMR_RTC = cast(uint)0x00000018; /* receive threshold control */
enum ETH_DMAOMR_RTC_64_BYTES = cast(uint)0x00000000; /* threshold level of the MTL Receive FIFO is 64 Bytes */
enum ETH_DMAOMR_RTC_32_BYTES = cast(uint)0x00000008; /* threshold level of the MTL Receive FIFO is 32 Bytes */
enum ETH_DMAOMR_RTC_96_BYTES = cast(uint)0x00000010; /* threshold level of the MTL Receive FIFO is 96 Bytes */
enum ETH_DMAOMR_RTC_128_BYTES = cast(uint)0x00000018; /* threshold level of the MTL Receive FIFO is 128 Bytes */
enum ETH_DMAOMR_OSF = cast(uint)0x00000004; /* operate on second frame */
enum ETH_DMAOMR_SR = cast(uint)0x00000002; /* Start/stop receive */

/* Bit definition for Ethernet DMA Interrupt Enable Register */
enum ETH_DMAIER_NISE = cast(uint)0x00010000; /* Normal interrupt summary enable */
enum ETH_DMAIER_AISE = cast(uint)0x00008000; /* Abnormal interrupt summary enable */
enum ETH_DMAIER_ERIE = cast(uint)0x00004000; /* Early receive interrupt enable */
enum ETH_DMAIER_FBEIE = cast(uint)0x00002000; /* Fatal bus error interrupt enable */
enum ETH_DMAIER_ETIE = cast(uint)0x00000400; /* Early transmit interrupt enable */
enum ETH_DMAIER_RWTIE = cast(uint)0x00000200; /* Receive watchdog timeout interrupt enable */
enum ETH_DMAIER_RPSIE = cast(uint)0x00000100; /* Receive process stopped interrupt enable */
enum ETH_DMAIER_RBUIE = cast(uint)0x00000080; /* Receive buffer unavailable interrupt enable */
enum ETH_DMAIER_RIE = cast(uint)0x00000040; /* Receive interrupt enable */
enum ETH_DMAIER_TUIE = cast(uint)0x00000020; /* Transmit Underflow interrupt enable */
enum ETH_DMAIER_ROIE = cast(uint)0x00000010; /* Receive Overflow interrupt enable */
enum ETH_DMAIER_TJTIE = cast(uint)0x00000008; /* Transmit jabber timeout interrupt enable */
enum ETH_DMAIER_TBUIE = cast(uint)0x00000004; /* Transmit buffer unavailable interrupt enable */
enum ETH_DMAIER_TPSIE = cast(uint)0x00000002; /* Transmit process stopped interrupt enable */
enum ETH_DMAIER_TIE = cast(uint)0x00000001; /* Transmit interrupt enable */

/* Bit definition for Ethernet DMA Missed Frame and Buffer Overflow Counter Register */
enum ETH_DMAMFBOCR_OFOC = cast(uint)0x10000000; /* Overflow bit for FIFO overflow counter */
enum ETH_DMAMFBOCR_MFA = cast(uint)0x0FFE0000; /* Number of frames missed by the application */
enum ETH_DMAMFBOCR_OMFC = cast(uint)0x00010000; /* Overflow bit for missed frame counter */
enum ETH_DMAMFBOCR_MFC = cast(uint)0x0000FFFF; /* Number of frames missed by the controller */

/* Bit definition for Ethernet DMA Current Host Transmit Descriptor Register */
enum ETH_DMACHTDR_HTDAP = cast(uint)0xFFFFFFFF; /* Host transmit descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Descriptor Register */
enum ETH_DMACHRDR_HRDAP = cast(uint)0xFFFFFFFF; /* Host receive descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Transmit Buffer Address Register */
enum ETH_DMACHTBAR_HTBAP = cast(uint)0xFFFFFFFF; /* Host transmit buffer address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Buffer Address Register */
enum ETH_DMACHRBAR_HRBAP = cast(uint)0xFFFFFFFF; /* Host receive buffer address pointer */

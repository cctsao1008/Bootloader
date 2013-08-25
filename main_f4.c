/*
 * STM32F4 board support for the bootloader.
 *
 */

#include <stdlib.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/systick.h>

#ifdef BOARD_FC
#include <libopencm3/stm32/i2c.h>
#include "pca953x.h"
#endif

#include "bl.h"

/* we should know this, but we don't */
#ifndef SCB_CPACR
# define SCB_CPACR (*((volatile uint32_t *) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

/* flash parameters that we should not really know */
static struct {
    uint32_t    erase_code;
    unsigned    size;
} flash_sectors[] = {
    /* flash sector zero reserved for bootloader */
    { FLASH_SECTOR_1, 16 * 1024},
    { FLASH_SECTOR_2, 16 * 1024},
    { FLASH_SECTOR_3, 16 * 1024},
    { FLASH_SECTOR_4, 64 * 1024},
    { FLASH_SECTOR_5, 128 * 1024},
    { FLASH_SECTOR_6, 128 * 1024},
    { FLASH_SECTOR_7, 128 * 1024},
    { FLASH_SECTOR_8, 128 * 1024},
    { FLASH_SECTOR_9, 128 * 1024},
    { FLASH_SECTOR_10, 128 * 1024},
    { FLASH_SECTOR_11, 128 * 1024}
};
#define BOARD_FLASH_SECTORS (sizeof(flash_sectors) / sizeof(flash_sectors[0]))

#ifdef BOARD_FMU
# define BOARD_TYPE                 5

// Board OSC
# define OSC_FREQ                   24

// Board LED
# define BOARD_PIN_LED_ACTIVITY     GPIO15
# define BOARD_PIN_LED_BOOTLOADER   GPIO14
# define BOARD_PORT_LEDS            GPIOB
# define BOARD_CLOCK_LEDS           RCC_AHB1ENR_IOPBEN
# define BOARD_LED_ON               gpio_clear
# define BOARD_LED_OFF              gpio_set

// Board UART
# define BOARD_USART                USART1
# define BOARD_PORT_USART           GPIOB
# define BOARD_USART_CLOCK_REGISTER RCC_APB2ENR
# define BOARD_USART_CLOCK_BIT      RCC_APB2ENR_USART1EN
# define BOARD_PIN_TX               GPIO6
# define BOARD_PIN_RX               GPIO7
# define BOARD_CLOCK_USART_PINS     RCC_AHB1ENR_IOPBEN
# define BOARD_FUNC_USART           GPIO_AF7

// Board USB
# define BOARD_PORT_USB             GPIOA
# define BOARD_PIN_VBUS             GPIO9
#endif

#ifdef BOARD_FLOW
# define BOARD_TYPE                 6

// Board OSC
# define OSC_FREQ                   24

// Board LED
# define BOARD_PIN_LED_ACTIVITY     GPIO3
# define BOARD_PIN_LED_BOOTLOADER   GPIO2
# define BOARD_PORT_LEDS            GPIOE
# define BOARD_CLOCK_LEDS           RCC_AHB1ENR_IOPEEN
# define BOARD_LED_ON               gpio_clear
# define BOARD_LED_OFF              gpio_set

// Board UART
# define BOARD_USART_CLOCK_REGISTER RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT      RCC_APB1ENR_USART2EN
# define BOARD_USART                USART2
# define BOARD_PORT_USART           GPIOD
# define BOARD_PIN_TX               GPIO5
# define BOARD_PIN_RX               GPIO6
# define BOARD_CLOCK_USART_PINS     RCC_AHB1ENR_IOPDEN
# define BOARD_FUNC_USART           GPIO_AF7

// Board USB
# define BOARD_PORT_USB             GPIOA
# define BOARD_PIN_VBUS             GPIO9
#endif

#ifdef BOARD_DISCOVERY
# define BOARD_TYPE                 99

// Board OSC
# define OSC_FREQ                   8

// Board LED
# define BOARD_PIN_LED_ACTIVITY     GPIO12
# define BOARD_PIN_LED_BOOTLOADER   GPIO13
# define BOARD_PORT_LEDS            GPIOD
# define BOARD_CLOCK_LEDS           RCC_AHB1ENR_IOPDEN
# define BOARD_LED_ON               gpio_set
# define BOARD_LED_OFF              gpio_clear

// Board UART
# define BOARD_USART                USART2
# define BOARD_PORT_USART           GPIOA
# define BOARD_USART_CLOCK_REGISTER RCC_APB1ENR
# define BOARD_USART_CLOCK_BIT      RCC_APB1ENR_USART2EN
# define BOARD_PIN_TX               GPIO2
# define BOARD_PIN_RX               GPIO3
# define BOARD_CLOCK_USART_PINS     RCC_AHB1ENR_IOPAEN
# define BOARD_FUNC_USART           GPIO_AF7

// Board USB
# define BOARD_PORT_USB             GPIOA
# define BOARD_PIN_VBUS             GPIO9
#endif

#ifdef BOARD_FC
// Board OSC
# define BOARD_TYPE                 5
# define OSC_FREQ                   8

// Board LED
# define BOARD_PIN_LED_ACTIVITY     PCA9533_LED1 // Blue LED
# define BOARD_PIN_LED_BOOTLOADER   PCA9533_LED3 // Red LED
# define BOARD_PIN_LED_SYSPWRON     PCA9533_LED2 // Green LED

// Board UART
# define BOARD_USART                USART1
# define BOARD_PORT_USART           GPIOB
# define BOARD_USART_CLOCK_REGISTER RCC_APB2ENR
# define BOARD_USART_CLOCK_BIT      RCC_APB2ENR_USART1EN
# define BOARD_PIN_TX               GPIO6
# define BOARD_PIN_RX               GPIO7
# define BOARD_CLOCK_USART_PINS     RCC_AHB1ENR_IOPBEN
# define BOARD_FUNC_USART           GPIO_AF7

// Board I2C
# define BOARD_I2C                  I2C2
# define BOARD_PORT_I2C             GPIOB
# define BOARD_I2C_CLOCK_REGISTER   RCC_APB1ENR
# define BOARD_I2C_CLOCK_BIT        RCC_APB1ENR_I2C2EN
# define BOARD_PIN_SCL              GPIO10
# define BOARD_PIN_SDA              GPIO11
# define BOARD_CLOCK_I2C_PINS       RCC_AHB1ENR_IOPBEN
# define BOARD_FUNC_I2C             GPIO_AF4

// Board BEEP
# define BOARD_PORT_BEEP            GPIOB
# define BOARD_PIN_BEEP             GPIO12
# define BOARD_CLOCK_BEEP           RCC_AHB1ENR_IOPBEN

// Board USB
# define BOARD_PORT_USB             GPIOA
# define BOARD_PIN_VBUS             GPIO9
#endif

#ifdef INTERFACE_USART
# define BOARD_INTERFACE_CONFIG     (void *)BOARD_USART
#else
# define BOARD_INTERFACE_CONFIG     NULL
#endif

/* board definition */
struct boardinfo board_info = {
    .board_type = BOARD_TYPE,
    .board_rev  = 0,
    .fw_size    = APP_SIZE_MAX,

    .systick_mhz    = 168,
};

static void board_init(void);

/* standard clocking for all F4 boards */
static const clock_scale_t clock_setup =
{
#if defined(BOARD_FMU)
    .pllm = OSC_FREQ,
    .plln = 336,
    .pllp = 2,
    .pllq = 7,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    .power_save = 0,
    .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_5WS,
    .apb1_frequency = 42000000,
    .apb2_frequency = 84000000,
#elif defined(BOARD_FC)
    .pllm = OSC_FREQ,
    .plln = 336,
    .pllp = 2,
    .pllq = 7,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_4,
    .ppre2 = RCC_CFGR_PPRE_DIV_2,
    .power_save = 0,
    .flash_config = FLASH_ICE | FLASH_DCE | FLASH_LATENCY_5WS,
    .apb1_frequency = 42000000,
    .apb2_frequency = 84000000,
#endif
};

#ifdef BOARD_FC
i2c_device_t pca_i2c_dev =
{
    .i2c =
    {
        .clk =
        {
            .reg = &RCC_APB1ENR,
            .en = RCC_APB1ENR_I2C2EN,
        },
        .id = BOARD_I2C,
        .fast_mode = true,
        .auto_increment = true, // For PCA9533 only
    },
    
    .gpio =
    {
        .clk =
        {
            .reg = &RCC_AHB1ENR,
            .en = RCC_AHB1ENR_IOPBEN,
        },
        .port = BOARD_PORT_I2C,
        .pair = BOARD_PIN_SCL|BOARD_PIN_SDA,
        .mode_af = BOARD_FUNC_I2C,
    }
};
#endif

static void
board_init(void)
{
    /* Enable the FPU before we hit any FP instructions */
    SCB_CPACR |= ((3UL << 10*2) | (3UL << 11*2)); /* set CP10 Full Access and set CP11 Full Access */

    /* configure the clock for bootloader activity */
    rcc_clock_setup_hse_3v3(&clock_setup);

    /* start the timer system for I2C timeout used */
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);	/* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();
    
    #ifdef BOARD_FMU
    /* initialise LEDs */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, BOARD_CLOCK_LEDS);

    gpio_mode_setup(
        BOARD_PORT_LEDS, 
        GPIO_MODE_OUTPUT, 
        GPIO_PUPD_NONE,
        BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

    gpio_set_output_options(
        BOARD_PORT_LEDS,
        GPIO_OTYPE_PP,
        GPIO_OSPEED_2MHZ,
        BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);

        BOARD_LED_ON (BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER | BOARD_PIN_LED_ACTIVITY);
    #endif

    #ifdef BOARD_FC

    /* initialise system Beep */
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, BOARD_CLOCK_BEEP);

    gpio_mode_setup(
        BOARD_PORT_BEEP, 
        GPIO_MODE_OUTPUT, 
        GPIO_PUPD_NONE,
        BOARD_PIN_BEEP);

    gpio_set_output_options(
        BOARD_PORT_BEEP,
        GPIO_OTYPE_PP,
        GPIO_OSPEED_2MHZ,
        BOARD_PIN_BEEP);

    /* initialise LEDs */
    pca953x_init(&pca_i2c_dev);

    pca9533_set_led(PCA9533_LED1|PCA9533_LED2, PCA9533_LED_ON);

    /* system bootup beep */
    beep_on(40, 300); // (100,300)

    pca9533_set_led(PCA9533_LED1, PCA9533_LED_OFF);

    #endif

    /*  Common interface initialise */
    #ifdef INTERFACE_USB
    /* enable GPIO9 with a pulldown to sniff VBUS */
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    gpio_mode_setup(BOARD_PORT_USB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, BOARD_PIN_VBUS);
    #endif

    #ifdef INTERFACE_USART
    /* configure usart pins */
    rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
    gpio_mode_setup(BOARD_PORT_USART, GPIO_MODE_AF, GPIO_PUPD_NONE, BOARD_PIN_TX | BOARD_PIN_RX);
    gpio_set_af(BOARD_PORT_USART, BOARD_FUNC_USART, BOARD_PIN_TX | BOARD_PIN_RX);

    /* configure USART clock */
    rcc_peripheral_enable_clock(&BOARD_USART_CLOCK_REGISTER, BOARD_USART_CLOCK_BIT);
    #endif

}


unsigned
flash_func_sector_size(unsigned sector)
{
    if (sector < BOARD_FLASH_SECTORS)
        return flash_sectors[sector].size;
    return 0;
}

void
flash_func_erase_sector(unsigned sector)
{
    if (sector < BOARD_FLASH_SECTORS)
        flash_erase_sector(flash_sectors[sector].erase_code, FLASH_PROGRAM_X32);
}

void
flash_func_write_word(unsigned address, uint32_t word)
{
    flash_program_word(address + APP_LOAD_ADDRESS, word, FLASH_PROGRAM_X32);
}

uint32_t 
flash_func_read_word(unsigned address)
{
    return *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

void
led_on(unsigned led)
{
    switch (led) {
    case LED_ACTIVITY:
        #ifdef BOARD_FC
        pca9533_set_led(BOARD_PIN_LED_ACTIVITY, PCA9533_LED_ON);
        #else
        BOARD_LED_ON (BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
        #endif
        break;
    case LED_BOOTLOADER:
        #ifdef BOARD_FC
        pca9533_set_led(BOARD_PIN_LED_BOOTLOADER, PCA9533_LED_ON);
        pca9536_config_io(PCA9536_IO3, PCA9536_IO_O);
        #else
        BOARD_LED_ON (BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
        #endif
        break;
    }
}

void
led_off(unsigned led)
{
    switch (led) {
    case LED_ACTIVITY:
        #ifdef BOARD_FC
        pca9533_set_led(BOARD_PIN_LED_ACTIVITY, PCA9533_LED_OFF);
        #else
        BOARD_LED_OFF (BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
        #endif
        break;
    case LED_BOOTLOADER:
        #ifdef BOARD_FC
        pca9533_set_led(BOARD_PIN_LED_BOOTLOADER, PCA9533_LED_OFF);
        pca9536_config_io(PCA9536_IO3, PCA9536_IO_I);
        #else
        BOARD_LED_OFF (BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
        #endif
        break;
    }
}

void
led_toggle(unsigned led)
{ 
    #ifdef BOARD_FC
    u8 led_status = 0;
    #endif

    switch (led) {
    case LED_ACTIVITY:
        #ifdef BOARD_FC
        led_status = (pca_i2c_dev.pca_953x_tbl.pca9533->ls0.led1)^= (1 << 0);
        pca9533_set_led(BOARD_PIN_LED_ACTIVITY, led_status);
        #else
        gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_ACTIVITY);
        #endif
        break;
    case LED_BOOTLOADER:
        #ifdef BOARD_FC
        //led_bl_on ^= (1 << 0); // toggle bit0, IO3 of PCA9536, Red LED
        //pca9536_config_io(PCA9536_IO3, led_bl_on);
        led_status = (pca_i2c_dev.pca_953x_tbl.pca9533->ls0.led3)^= (1 << 0);
        pca9533_set_led(BOARD_PIN_LED_BOOTLOADER, led_status);
        pca9536_config_io(PCA9536_IO3, led_status);
        #else
        gpio_toggle(BOARD_PORT_LEDS, BOARD_PIN_LED_BOOTLOADER);
        #endif
        break;
    }
}

#ifdef BOARD_FC
#define MAX_BEEP_TD 2000 // limit the maximum duration to 2000 ms
void
beep_on(unsigned on_msec, unsigned off_msec)
{
    if(on_msec < MAX_BEEP_TD) 
        timer[TIMER_DELAY] = on_msec;
    else
        timer[TIMER_DELAY] = MAX_BEEP_TD;

    gpio_set(BOARD_PORT_BEEP, BOARD_PIN_BEEP);

    while(timer[TIMER_DELAY] > 0);

    gpio_clear(BOARD_PORT_BEEP, BOARD_PIN_BEEP);

    if(on_msec < MAX_BEEP_TD) // limit the maximum duration to 1000 ms
        timer[TIMER_DELAY] = off_msec;
    else
        timer[TIMER_DELAY] = MAX_BEEP_TD;

    while(timer[TIMER_DELAY] > 0);
}
#endif

int
main(void)
{
    unsigned timeout = 0;

    #ifdef BOARD_FC
    unsigned i = 0;
    #endif

    /* do board-specific initialisation */
    board_init();

    #ifdef INTERFACE_USB
    /* check for USB connection - if present, we will wait in the bootloader for a while */
    if (gpio_get(BOARD_PORT_USB, BOARD_PIN_VBUS) != 0)
    {
        timeout = BOOTLOADER_DELAY;
    }
    #endif

    #ifdef INTERFACE_USART
    /* XXX sniff for a USART connection to decide whether to wait in the bootloader */
    timeout = 0;
    #endif

    /* XXX we could look at the backup SRAM to check for stay-in-bootloader instructions */

    /* if we aren't expected to wait in the bootloader, try to boot immediately */
    if (timeout == 0) {
        /* try to boot immediately */
        jump_to_app();

        /* if we returned, there is no app; go to the bootloader and stay there */
        timeout = 0;
    }

    #ifdef BOARD_FC
    /* and beep for three times !! if we are using TMR-FC board */
    for(i = 0; i < 3 ; i++)
    {
       beep_on(20, 80); // beep on for 100ms (40,80)
    }
    #endif

    /* start the interface */
    cinit(BOARD_INTERFACE_CONFIG);

    while (1)
    {
        /* run the bootloader, possibly coming back after the timeout */
        bootloader(timeout);

        /* look to see if we can boot the app */
        jump_to_app();

        /* boot failed; stay in the bootloader forever next time */
        timeout = 0;
    }
}


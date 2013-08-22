/*
 *
 * NXP PCA953X Driver
 *
 */
 
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include "pca953x.h"

u8 auto_incresment = true, i2c_bus = 0;

PCA9533_MM pca9533_mm = {
    .input = 0x00,
    .psc0 = 0x00,
    .pwm0 = 0x00,
    .psc1 = 0x00,
    .pwm0 = 0x00,
    .led0 = LED_ON,
    .led1 = LED_ON,
    .led2 = LED_ON,
    .led3 = LED_ON,
} ;

PCA9536_MM pca9536_mm = {
    .input = 0x00,
    .output = 0x00,
    .polarity = 0x00,
    .config = 0x00
} ;

void i2c_write(u32 i2c, u8 addr, u8 reg, u8* data, u8 count);
void i2c_read(u32 i2c, u8 addr, u8 reg, u8* data, u8 count);
u32 i2c_setup(u32 i2c, u8 fast_mode);
void i2c_error(void);

u32 pca953x_init(PCA_I2C_DEVICE i2c_dev)
{
    u32 i2c = 0;
    i2c = i2c_setup(i2c_dev.i2c, i2c_dev.fast_mode);

    if(i2c == false)
        goto error ;

    i2c_write(i2c, PCA9533_ADDR, 0x0, (u8 *)&pca9533_mm, sizeof(PCA9533_MM));
    i2c_write(i2c, PCA9536_ADDR, 0x0, (u8 *)&pca9536_mm, sizeof(PCA9533_MM));

error :
    return i2c ;
}

void i2c_error(void)
{

}

u32 i2c_setup(u32 i2c, u8 fast_mode)
{
    if((i2c == I2C1) && (i2c_bus == 0))
    {
        /* Enable clocks for I2C1 and AFIO. */
        rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_I2C1EN);
        rcc_peripheral_enable_clock(&RCC_APB1LPENR, RCC_APB1LPENR_I2C1LPEN);
    }
    else if((i2c == I2C2) && (i2c_bus == 0))
    {
        rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_AHB1ENR_IOPBEN);
        /* Enable clocks for I2C2 and AFIO. */
        rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_I2C2EN);
        rcc_peripheral_enable_clock(&RCC_APB1LPENR, RCC_APB1LPENR_I2C2LPEN);

        gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
        gpio_set_af(GPIOB, GPIO_AF4, GPIO10 | GPIO11);
    }
    else
    {
        goto cleanup;
    }

    /* Disable the I2C before changing any configuration. */
    i2c_peripheral_disable(i2c);

    /* APB1 is running at 36MHz. */
    i2c_set_clock_frequency(i2c, I2C_CR2_FREQ_36MHZ);

    /* 400KHz - I2C Fast Mode */
    if(fast_mode)
        i2c_set_fast_mode(i2c);

    /*
        * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
        * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
        * Datasheet suggests 0x1e.
        */
    i2c_set_ccr(i2c, 0x1e);

    /*
        * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
        * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
        * Incremented by 1 -> 11.
        */
    i2c_set_trise(i2c, 0x0b);

    /* If everything is configured -> enable the peripheral. */
    i2c_peripheral_enable(i2c);

    i2c_bus = i2c;

cleanup:
    return i2c_bus ;
}

void i2c_write(u32 i2c, u8 addr, u8 reg, u8* data, u8 count)
{
	u32 reg32, i;

    // I2C Start
	/* Send START condition. */
	i2c_send_start(i2c);

    // I2C EV5
	/* Waiting for START is send and therefore switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

    // I2C Address
	/* Say to what address we want to talk to. */
	i2c_send_7bit_address(i2c, addr, I2C_WRITE);

    // I2C EV6
	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

    // I2C EV8 and Data
	/* Sending the data. */
	if((auto_incresment == true) && (addr = PCA9533_ADDR)) //Set Auto-Increment flag ?
	    reg |= AI_FLAG;

	i2c_send_data(i2c, reg); /* Sent register address that we want to talk to */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));

    for (i = 0; i < count; i++)
    {
        if(count == 1)
        {
            i2c_send_data(i2c, (u8)(*data));
            /* After the last byte we have to wait for TxE too. */
            while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
        }
        else
        {
            if((i+1) == count)
            {
                if(addr = PCA9536_ADDR)
                {
                    i2c_send_data(i2c, reg + i); /* Sent register address that we want to talk to */
	                while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	            }

                if(auto_incresment != true)
                {
                    i2c_send_data(i2c, reg + i); /* Sent register address that we want to talk to */
	                while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	            }

                i2c_send_data(i2c, (u8)(*(data + i)));  /* LSB */
                /* After the last byte we have to wait for TxE too. */
                while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));
            }
            else if(addr = PCA9533_ADDR)
            {
                if(auto_incresment != true)
                {
                    i2c_send_data(i2c, reg + i); /* Sent register address that we want to talk to */
	                while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	            }

                i2c_send_data(i2c, (u8)(*(data + i)));
                while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
            }
            else if(addr = PCA9536_ADDR)
            {
                i2c_send_data(i2c, reg + i); /* Sent register address that we want to talk to */
	            while (!(I2C_SR1(i2c) & I2C_SR1_BTF));

                i2c_send_data(i2c, (u8)(*(data + i)));
                while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
            }
            
        }

        /* Send STOP condition. */
        i2c_send_stop(i2c);
    }
}

void i2c_read(u32 i2c, u8 addr, u8 reg, u8* data, u8 count)
{
    #if 0
	u32 reg32;
	u16 temperature;

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	/* Yes, WRITE is correct - for selecting register in STTS75. */
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	i2c_send_data(i2c, 0x0); /* temperature register */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/*
	 * Now we transferred that we want to ACCESS the temperature register.
	 * Now we send another START condition (repeated START) and then
	 * transfer the destination but with flag READ.
	 */

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	i2c_send_7bit_address(i2c, sensor, I2C_READ); 

	/* 2-byte receive is a special case. See datasheet POS bit. */
	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Cleaning I2C_SR1_ACK. */
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;

	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	temperature = (u16)(I2C_DR(i2c) << 8); /* MSB */

	/*
	 * Yes they mean it: we have to generate the STOP condition before
	 * saving the 1st byte.
	 */
	I2C_CR1(i2c) |= I2C_CR1_STOP;

	temperature |= I2C_DR(i2c); /* LSB */

	/* Original state. */
	I2C_CR1(i2c) &= ~I2C_CR1_POS;

	return temperature;
    #endif
}


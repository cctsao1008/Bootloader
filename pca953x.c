/*
 *
 * NXP PCA953X Driver
 *
 */
 
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include "pca953x.h"

#define UNIT_US 6578
  
u8 i2c_write(u32 i2c, u8 addr, u8 reg, u8* data, u8 count);
u8 i2c_read(u32 i2c, u8 addr, u8 reg, u8* data, u8 count);
void i2c_setup(i2c_device_t* i2c_dev);
void i2c_error(void);

pca9533_t pca9533_tbl = {
    .input = 0x00,
    .psc0 = 0x00,
    .pwm0 = 0x00,
    .psc1 = 0x00,
    .pwm0 = 0x00,
    .ls0.led0 = PCA9533_LED_OFF,
    .ls0.led1 = PCA9533_LED_OFF,
    .ls0.led2 = PCA9533_LED_OFF,
    .ls0.led3 = PCA9533_LED_OFF,
};

pca9536_t pca9536_tbl = {
    .input = {0x00},
    .output = {0x00},
    .polarity = {0x00},
    .config.cx3 = 0x0,
};

pca_tbl_t pca_953x_tbl;
i2c_device_t* i2c_dev = 0;

#define TIMER_I2C	    4
extern volatile unsigned timer[];

pca_tbl_t* pca953x_init(i2c_device_t* dev)
{
    timer[TIMER_I2C] = 5000; // 5 secs
    while(timer[TIMER_I2C] > 0);

    i2c_setup(dev);

    i2c_write(i2c_dev->i2c.id, PCA9533_ADDR, PCA9533_REG_START, (u8 *)&pca9533_tbl, sizeof(pca9533_tbl)/sizeof(u8));
    i2c_write(i2c_dev->i2c.id, PCA9536_ADDR, PCA9536_REG_START, (u8 *)&pca9536_tbl, sizeof(pca9536_tbl)/sizeof(u8));

    // test code
    pca9533_set_peroid(PCA9533_REG_PSC0, 1000000);
    pca9533_set_peroid(PCA9533_REG_PSC1, 500000);
    pca9533_set_pwm(PCA9533_REG_PWM0, 50);
    pca9533_set_pwm(PCA9533_REG_PWM1, 50);
    pca9533_set_led(PCA9533_LED0, PCA9533_LED_PWM0);
    pca9533_set_led(PCA9533_LED1, PCA9533_LED_PWM1);
    pca9533_set_led(PCA9533_LED2, PCA9533_LED_OFF);
    pca9533_set_led(PCA9533_LED3, PCA9533_LED_ON);

    pca_953x_tbl.pca9533 = &pca9533_tbl;
    pca_953x_tbl.pca9536 = &pca9536_tbl;

    return &pca_953x_tbl;
}

u8 pca953x_update(u8 pca_id)
{
    u8 rc = false;

    if(i2c_dev == 0)
        goto cleanup;

    if(pca_id == PCA9533_ADDR)
    {
        if(!i2c_write(i2c_dev->i2c.id, PCA9533_ADDR, PCA9533_REG_START, (u8 *)&pca9533_tbl, sizeof(pca9533_tbl)/sizeof(u8)))
            goto cleanup;
    }
    else if(pca_id == PCA9536_ADDR)
    {
        if(!i2c_write(i2c_dev->i2c.id, PCA9536_ADDR, PCA9536_REG_START, (u8 *)&pca9536_tbl, sizeof(pca9536_tbl)/sizeof(u8)))
            goto cleanup;
    }

    rc = true;

cleanup:
    return rc;
}

u8 pca9533_set_peroid(u8 psc, u32 usec)
{
    u8 rc = 0, data = 0;

    if(usec < 6578)
        data = 0;

    if(usec > 1677390)
        data = 255;

    data = (u8)(usec/UNIT_US);

    if(psc == PCA9533_REG_PSC0)
    {
        if(!i2c_write(i2c_dev->i2c.id, PCA9533_ADDR, PCA9533_REG_PSC0, &data, 0x01))
            goto cleanup;

        // update table
        pca9533_tbl.psc0 = data;
    }
    
    if(psc == PCA9533_REG_PSC1)
    {
        if(!i2c_write(i2c_dev->i2c.id, PCA9533_ADDR, PCA9533_REG_PSC1, &data, 0x01))
            goto cleanup;

        // update table
        pca9533_tbl.psc1 = data;
    }

    rc = true;

cleanup:
    return rc;
}

u8 pca9533_set_pwm(u8 pwm, u32 duty)
{
    u8 rc = false, data = 0;

    if(duty < 0)
        data = 0;

    if(duty > 100)
        data = 255;

    data = (u8)(((float)duty/100)*256);

    if(pwm == PCA9533_REG_PWM0)
    {
        if(!i2c_write(i2c_dev->i2c.id, PCA9533_ADDR, PCA9533_REG_PWM0, &data, 0x01))
        goto cleanup;

        // update table
        pca9533_tbl.pwm0 = data;
    }
    
    if(pwm == PCA9533_REG_PWM1)
    {
        if(!i2c_write(i2c_dev->i2c.id, PCA9533_ADDR, PCA9533_REG_PWM1, &data, 0x01))
        goto cleanup;

        // update table
        pca9533_tbl.pwm1 = data;
    }

    rc = true;

cleanup:
    return rc;
}

u8 pca9533_set_led(u8 led, u32 mode)
{
    u8 rc = false;

    switch(led)
    {
        case PCA9533_LED0 :
            pca9533_tbl.ls0.led0 = mode;
            break;
        
        case PCA9533_LED1 :
            pca9533_tbl.ls0.led1 = mode;
            break;

        case PCA9533_LED2 :
            pca9533_tbl.ls0.led2 = mode;
            break;

        case PCA9533_LED3 :
            pca9533_tbl.ls0.led3 = mode;
            break;
    }

    if(!i2c_write(i2c_dev->i2c.id, PCA9533_ADDR, PCA9533_REG_LS0, (u8*)&(pca9533_tbl.ls0), 0x01))
        goto cleanup;

    rc = true;

cleanup:
    return rc;
}

u8 pca9536_config_io(u8 io, u8 set)
{
    u8 rc = false;

    switch(io)
    {
        case PCA9536_IO0 :
            pca9536_tbl.config.cx0 = set;
            break;
        
        case PCA9536_IO1 :
            pca9536_tbl.config.cx1 = set;
            break;

        case PCA9536_IO2 :
            pca9536_tbl.config.cx2 = set;
            break;

        case PCA9536_IO3 :
            pca9536_tbl.config.cx3 = set;
            break;
    }

    if(!i2c_write(i2c_dev->i2c.id, PCA9536_ADDR, PCA9536_REG_CONFIG, (u8*)&(pca9536_tbl.config), 0x01))
        goto cleanup;

    rc = true;

cleanup:
    return rc;
}

void i2c_error(void)
{

}

void i2c_setup(i2c_device_t* dev)
{   

    /* Enable clocks for GPIO. */
    rcc_peripheral_enable_clock(dev->i2c.clk.reg, dev->i2c.clk.en);

    /* Enable clocks for I2C2 and AFIO. */
    rcc_peripheral_enable_clock(dev->gpio.clk.reg, dev->gpio.clk.en);

    gpio_mode_setup(dev->gpio.port, GPIO_MODE_AF, GPIO_PUPD_NONE, dev->gpio.pair);
    gpio_set_output_options(dev->gpio.port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,  dev->gpio.pair);
    gpio_set_af(dev->gpio.port, GPIO_AF4, dev->gpio.pair);

    /* Disable the I2C before changing any configuration. */
    i2c_peripheral_disable(dev->i2c.id);

    /* APB1 is running at 42MHz. */
    i2c_set_clock_frequency(dev->i2c.id, I2C_CR2_FREQ_42MHZ);

    /* 400KHz - I2C Fast Mode */
    if(dev->i2c.fast_mode)
        i2c_set_fast_mode(dev->i2c.id);

    /*
        * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
        * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
        * Datasheet suggests 0x1e.
        */
    i2c_set_ccr(dev->i2c.id, 0xc034);

    /*
        * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
        * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
        * Incremented by 1 -> 11.
        */
    i2c_set_trise(dev->i2c.id, 0x0c);

    /* If everything is configured . enable the peripheral. */
    i2c_peripheral_enable(dev->i2c.id);

    i2c_dev = dev;

}

u8 i2c_write(u32 i2c, u8 addr, u8 reg, u8* data, u8 count)
{
    u8 rc = false;
	u32 i = 0;

    for(i = 0; i < count; i++)
    {
        timer[TIMER_I2C] = 10; // 10ms

        while((I2C_SR2(i2c) & I2C_SR2_BUSY))
        {
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    	}

        // I2C Start
        /* Send START condition. */
        i2c_send_start(i2c);

        // I2C EV5
    	/* Waiting for START is send and therefore switched to master mode. */
    	while (!((I2C_SR1(i2c) & I2C_SR1_SB) & (I2C_SR2(i2c) & (I2C_SR2_MSL|I2C_SR2_BUSY))))
    	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    	}

        // I2C Address
    	/* Say to what address we want to talk to. */
    	i2c_send_7bit_address(i2c, addr, I2C_WRITE);

        // I2C EV6
    	/* Waiting for address is transferred. */
    	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR))
    	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    	}

    	/* Cleaning ADDR condition sequence. */
	    I2C_SR2(i2c);

    	/* Sending the data. */
        i2c_send_data(i2c, reg + i); /* Sent register address that we want to talk to */

        // I2C EV8
    	while (!(I2C_SR1(i2c) & I2C_SR1_TxE))
    	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    	}

        // I2C EV8_2
        /* After the last byte we have to wait for I2C_SR1_BTF too. */
        i2c_send_data(i2c, (u8)(*(data + i)));
        while (!(I2C_SR1(i2c) & (I2C_SR1_TxE|I2C_SR1_BTF)));
        {
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    	}

        /* Send STOP condition. */
        i2c_send_stop(i2c);

    }

    rc = true;

cleanup:
    return rc;
}

u8 i2c_read(u32 i2c, u8 addr, u8 reg, u8* data, u8 count)
{
    u8 rc = 0;

    timer[TIMER_I2C] = 10; // 10ms

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB) & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    }

	/* Say to what address we want to talk to. */
	/* Yes, WRITE is correct - for selecting register in STTS75. */
	i2c_send_7bit_address(i2c, addr, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR))
	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    }

	/* Cleaning ADDR condition sequence. */
	I2C_SR2(i2c);

	i2c_send_data(i2c, 0x0);  /* Sent register address that we want to talk to */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)))
	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    }

	/*
	 * Now we transferred that we want to ACCESS the register.
	 * Now we send another START condition (repeated START) and then
	 * transfer the destination but with flag READ.
	 */

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB) & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    }

	/* Say to what address we want to talk to. */
	i2c_send_7bit_address(i2c, addr, I2C_READ); 

	/* 2-byte receive is a special case. See datasheet POS bit. */
	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR))
	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    }

	/* Cleaning ADDR condition sequence. */
	I2C_SR2(i2c);

	/* Cleaning I2C_SR1_ACK. */
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;

	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF))
	{
            if(timer[TIMER_I2C] == 0)
                goto cleanup;
    }

	*data = (u16)(I2C_DR(i2c) << 8); /* MSB */

	/*
	 * Yes they mean it: we have to generate the STOP condition before
	 * saving the 1st byte.
	 */
	I2C_CR1(i2c) |= I2C_CR1_STOP;

	*(data+1) |= I2C_DR(i2c); /* LSB */

	/* Original state. */
	I2C_CR1(i2c) &= ~I2C_CR1_POS;

cleanup:
    return rc;
}


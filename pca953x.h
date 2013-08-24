/*
 *
 * NXP PCA953X Driver header
 *
 */
 
#ifndef _PCA953X_H
#define _PCA953X_H
#include <stdint.h>

//4-bit I2C-bus LED dimmer
#define PCA9533_ADDR  0x62

#define PCA9533_REG_START      0x00
#define PCA9533_REG_INPUT      0x00
#define PCA9533_REG_PSC0       0x01
#define PCA9533_REG_PWM0       0x02
#define PCA9533_REG_PSC1       0x03
#define PCA9533_REG_PWM1       0x04
#define PCA9533_REG_LS0        0x05

#define PCA9533_LED_OFF        0x00
#define PCA9533_LED_ON         0x01
#define PCA9533_LED_PWM0       0x02
#define PCA9533_LED_PWM1       0x03

#define PCA9533_AI_FLAG        0x10

#define PCA9533_LED0           0x01
#define PCA9533_LED1           0x02
#define PCA9533_LED2           0x04
#define PCA9533_LED3           0x08

typedef struct _pca9533_t
{
    u8 input ;
    u8 psc0 ;
    u8 pwm0 ;
    u8 psc1 ;
    u8 pwm1 ;
    
    union
    {
    	u8 ls0 ;
    	struct
    	{
    		u8 led0 : 2 ;
        	u8 led1 : 2 ;
        	u8 led2 : 2 ;
        	u8 led3 : 2 ;
    	};
    }ls0 ;
}pca9533_t;

//4-bit I2C-bus and SMBus I/O port
#define PCA9536_ADDR  0x41

#define PCA9536_REG_START      0x00
#define PCA9536_REG_INPUT      0x00
#define PCA9536_REG_OUTPUT     0x01
#define PCA9536_REG_POLARITY   0x02
#define PCA9536_REG_CONFIG     0x03

#define PCA9536_IO0 0x00
#define PCA9536_IO1 0x01
#define PCA9536_IO2 0x02
#define PCA9536_IO3 0x03

typedef struct _pca9536_t
{
    union
    {
    	u8 input ;
    	struct
    	{
    		u8 ix0 : 1 ;
    		u8 ix1 : 1 ;
    		u8 ix2 : 1 ;
    		u8 ix3 : 1 ;
    	};
    }input;

    union
    {
    	u8 output ;
    	struct
    	{
    		u8 ox0 : 1 ;
    		u8 ox1 : 1 ;
    		u8 ox2 : 1 ;
    		u8 ox3 : 1 ;
    	};
    }output;

    union
    {
    	u8 polarity ;
    	struct
    	{
    		u8 nx0 : 1 ;
    		u8 nx1 : 1 ;
    		u8 nx2 : 1 ;
    		u8 nx3 : 1 ;
    	};
    }polarity;

    union
    {
    	u8 config ;
    	struct
    	{
    		u8 cx0 : 1 ;
    		u8 cx1 : 1 ;
    		u8 cx2 : 1 ;
    		u8 cx3 : 1 ;
    	};
    }config;
}pca9536_t;

typedef struct
{
    pca9533_t* pca9533;
    pca9536_t* pca9536;

}pca_tbl_t;

typedef struct _i2c_device_t
{
    struct
    {
        u8 locked;
        struct
        {
            volatile u32* reg;
            u32 en;
        }clk;
    
        u32 id;
        u8  fast_mode;
        u8  auto_increment; // For PCA9533 only
    }i2c;

    struct
    {
        struct
        {
            volatile u32* reg;
            u32 en;
        }clk;

        u32 port;
        u16 pair;
        u8 mode_af;
    }gpio;

    pca_tbl_t pca_953x_tbl;
}i2c_device_t;

u8 pca953x_init(i2c_device_t* dev);
u8 pca953x_update(u8 pca_id);
u8 pca9533_set_peroid(u8 psc, u32 msec);
u8 pca9533_set_pwm(u8 pwm, u32 duty);
u8 pca9533_set_led(u8 led, u32 mode);
u8 pca9536_config_io(u8 io, u8 set);
#endif  /* _PCA953X_H */

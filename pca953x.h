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

#define REG_INPUT      0x00
#define REG_PSC0       0x01
#define REG_PWM0       0x02
#define REG_PSC1       0x03
#define REG_PWM1       0x04
#define REG_LS0        0x05

#define LED_OFF        0x00
#define LED_ON         0x01
#define LED_PWM0       0x02
#define LED_PWM1       0x03

#define AI_FLAG        0x10
#define LED0           0x00
#define LED1           0x02
#define LED2           0x04
#define LED3           0x06

typedef struct tag_pca_i2c_device
{
    u32 i2c;
    u32 port;
    u16 mode_af;
    u16 gpio_scl;
    u16 gpio_sda;
    u8  fast_mode;
    u8  auto_increment; // For PCA9533 only
}PCA_I2C_DEVICE;

typedef union tag_pca9533
{
    u8 input ;
    u8 psc0 ;
    u8 pwm0 ;
    u8 psc1 ;
    u8 pwm1 ;
    u8 ls0 ;

    struct
    {
        u8 led0 : 2 ;
        u8 led1 : 2 ;
        u8 led2 : 2 ;
        u8 led3 : 2 ;
        u8      : 8 ;
        u8      : 8 ;
        u8      : 8 ;
        u8      : 8 ;
        u8      : 8 ;
    };
}PCA9533_MM;

//4-bit I2C-bus and SMBus I/O port
#define PCA9536_ADDR  0x41
#define REG_OUTPUT     0x01
#define REG_POLARITY   0x02
#define REG_CONFIG     0x03

typedef union tag_pca9536
{
    u8 input ;
    u8 output ;
    u8 polarity ;
    u8 config ;

    struct
    {
        u8 ix0 : 1 ;
        u8 ix1 : 1 ;
        u8 ix2 : 1 ;
        u8 ix3 : 1 ;
        u8     : 4 ;

        u8 ox0 : 1 ;
        u8 ox1 : 1 ;
        u8 ox2 : 1 ;
        u8 ox3 : 1 ;
        u8     : 4 ;

        u8 nx0 : 1 ;
        u8 nx1 : 1 ;
        u8 nx2 : 1 ;
        u8 nx3 : 1 ;
        u8     : 4 ;

        u8 cx0 : 1 ;
        u8 cx1 : 1 ;
        u8 cx2 : 1 ;
        u8 cx3 : 1 ;
        u8     : 4 ;
    };
}PCA9536_MM;

u32 pca953x_init(PCA_I2C_DEVICE i2c_dev);

#endif  /* _PCA953X_H */
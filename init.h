#ifndef _ECR8660_INIT_H_
#define _ECR8660_INIT_H_
#include "cs32f0xx.h"

void spi_gpio_init(void);
void gpio_config(void);
void spi_gpio_init(void);
void usart_config(void);
void time_config(void);
void adc_i2c_init(void);
void exti0_1_config(void);
void exti0_1_interrupt(enable_state_t enable);
unsigned char cs32_ecr8660_init(unsigned char number);
unsigned char flash_config(unsigned char number);
unsigned char board_config(unsigned char number);
void flash_addr_config(unsigned char number);

#endif 




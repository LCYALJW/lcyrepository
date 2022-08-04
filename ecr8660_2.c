#include "ecr8660_2.h"
#include "ECR8660.h"
#include "cs32f0xx_gpio.h"

#define SPI_2_SET()							 gpio_bits_set(GPIOA,GPIO_PIN_11)						//≈‰÷√ADC2


void ecr8660_2_init(void)
{
	SPI_2_SET();
}


void ecr8660_2_code_download(void)
{
	
}


void ecr8660_2_write(void)
{
	
}


void ecr8660_2_read(void)
{
	
}


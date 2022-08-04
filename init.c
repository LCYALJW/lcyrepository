#include <stdio.h>
#include "ECR8660.h"
#include "init.h"
#include "cs32f0xx_gpio.h"
#include "cs32f0xx_rcu.h"
#include "cs32f0xx_i2c.h"
#include "cs32f0xx_usart.h"
#include "cs32f0xx_conf.h"
#include "spi_adc.h"
#include "common.h"
#include "systick.h"
#include "command.h"
#include "i2c_ee.h"

BOARD_CONFIG_T board_imformation[BOARD_NUMBER];

AD_REG_T ad_reg_arr[] = {
	{0x20004000, 0x00020005},
	{0x20004004, 0x00050005},
	{0x20004028, 0x802f},
	{0x20004030, 0x8000},
	{0x20004008, 1500000},			//接收端	单位：KHZ
	//{0x20004018, 2700000},		//发送端	单位：KHZ
	{0x00201180, 0x22220001},
	{0x00201084, 0x2},
};

AD_REG_T ad_reg_arr_2[] = {
	{0x20004000, 0x00030505},
	{0x20004004, 0x00000000},
	{0x20004028, 0x802e},
	{0x20004030, 0x8023},
	{0x20004008, 400000},			//接收端	单位：KHZ
	{0x20004018, 490000},			//发送端	单位：KHZ
	{0x00201180, 0x22220001},
	{0x00201084, 0x2},
};


unsigned int ECR8660_Data = 0;
extern unsigned char AD1_lock_state;
extern unsigned char exist_state;
extern unsigned int data_value;
extern UART_PACK_T uart_data;
extern unsigned char tx_buf [20];
extern unsigned char rx_buf [80];
extern unsigned int scan_start_freq;
extern unsigned int scan_end_freq;


#define SPI_FLASH_CS_LOW()       gpio_bits_reset(GPIOA, GPIO_PIN_4)
#define SPI_FLASH_CS_HIGH()      gpio_bits_set(GPIOA, GPIO_PIN_4)
#define CLK_LOW()       				 gpio_bits_reset(GPIOA, GPIO_PIN_5)          
#define CLK_HIGH()       				 gpio_bits_set(GPIOA, GPIO_PIN_5)
#define MOSI_LOW()       				 gpio_bits_reset(GPIOA, GPIO_PIN_7)
#define MOSI_HIGH()       			 gpio_bits_set(GPIOA, GPIO_PIN_7)
#define MISO_READ() 						 gpio_input_data_bit_read(GPIOA,GPIO_PIN_6)


unsigned char REG_Config(unsigned char number, AD_REG_T* ad_reg,unsigned char reg_len)
{
	unsigned char i = 0;
	unsigned char ret = 0;
	//AD1配置
	if(!board_imformation[number].ad_lock_staet){
		//配置AD0 API
			for(i = 0; i < reg_len;i++){
				if(Time_Out(number));
				else ret = 1;
				ECR8660_write(number,ECR8660_ecternal_register, ad_reg[i].ad_reg, ad_reg[i].ad_reg_value);
				delay(10);
				if(ad_reg[i].ad_reg == 0x201180){
					delay(1000);
				};
			}
			
			delay(1000);
			printf("ADC%d init complate\r\n",number);
	}
	return ret;
}

/***********************************************************************************
函数功能：SPI GPIO口初始化函数
函数参数：无
函数返回：无
***********************************************************************************/
void spi_gpio_init(void)
{
    gpio_config_t  gpio_config_struct; 

    rcu_ahb_periph_clock_enable_ctrl(RCU_AHB_PERI_PORTA, ENABLE);  
		rcu_ahb_periph_clock_enable_ctrl(RCU_AHB_PERI_PORTB, ENABLE);
    
    // SPI GPIO Config
    gpio_config_struct.gpio_pin = GPIO_PIN_5|GPIO_PIN_7; //PA5 PA7
    gpio_config_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL; 
    gpio_init(GPIOA, &gpio_config_struct);    
	
		gpio_config_struct.gpio_pin = GPIO_PIN_6;  //  PA6
    gpio_config_struct.gpio_mode = GPIO_MODE_INPUT;  
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOA, &gpio_config_struct);
		
						
    gpio_config_struct.gpio_pin = GPIO_PIN_4;  //  SD_SPI_CS_PIN 
    gpio_config_struct.gpio_mode = GPIO_MODE_OUTPUT;  
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio_config_struct);	
			
    SPI_FLASH_CS_HIGH();
		CLK_LOW();
		MOSI_LOW();
}


/***********************************************************************************
函数功能：GPIO口初始化函数
函数参数：无
函数返回：无
***********************************************************************************/
void gpio_config(void)
{ 
    gpio_config_t  gpio_config_struct; 
    
    // Clock Config	
    rcu_ahb_periph_clock_enable_ctrl(RCU_AHB_PERI_PORTA, ENABLE);
		rcu_ahb_periph_clock_enable_ctrl(RCU_AHB_PERI_PORTB, ENABLE);
		rcu_ahb_periph_clock_enable_ctrl(RCU_AHB_PERI_PORTC, ENABLE);
	
		gpio_config_struct.gpio_pin = GPIO_PIN_9;	 				//LOCKING LED
    gpio_config_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOB, &gpio_config_struct);
	
		gpio_config_struct.gpio_pin = GPIO_PIN_13;	 			//RUNNING LED
    gpio_config_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOC, &gpio_config_struct);
		
		gpio_config_struct.gpio_pin = GPIO_PIN_10;	 			//FPGA_GPIO2
    gpio_config_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOB, &gpio_config_struct);
		
		gpio_config_struct.gpio_pin = GPIO_PIN_11;	 			//FPGA_GPIO1
    gpio_config_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOB, &gpio_config_struct);
		
		gpio_config_struct.gpio_pin = GPIO_PIN_3;	 			//FPGA select
    gpio_config_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOB, &gpio_config_struct);
		
		gpio_config_struct.gpio_pin = GPIO_PIN_4;	 			//WDOG_GPIO
    gpio_config_struct.gpio_mode = GPIO_MODE_OUTPUT;
    gpio_config_struct.gpio_speed = GPIO_SPEED_HIGH;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOB, &gpio_config_struct);
		
		gpio_bits_set(GPIOC,GPIO_PIN_13);
		gpio_bits_set(GPIOB,GPIO_PIN_4);
		delay(10);
		gpio_bits_reset(GPIOB,GPIO_PIN_4);
		
}


/***********************************************************************************
函数功能：USART初始化函数
函数参数：无
函数返回：无
***********************************************************************************/
void usart_config(void)
{ 
    usart_config_t usart_config_struct;
		nvic_config_t  nvic_config_struct;
    gpio_config_t  gpio_config_struct; 
    
    // Clock Config	
    rcu_ahb_periph_clock_enable_ctrl(RCU_AHB_PERI_PORTA, ENABLE);
    rcu_apb1_periph_clock_enable_ctrl(RCU_APB1_PERI_USART2, ENABLE);  
		rcu_apb2_periph_clock_enable_ctrl(RCU_APB2_PERI_USART1, ENABLE);
    
    // GPIO MF Config
    gpio_mf_config(GPIOA, GPIO_PIN_NUM2, GPIO_MF_SEL1);   
    gpio_mf_config(GPIOA, GPIO_PIN_NUM3, GPIO_MF_SEL1);
		gpio_mf_config(GPIOA, GPIO_PIN_NUM9, GPIO_MF_SEL1);   
    gpio_mf_config(GPIOA, GPIO_PIN_NUM10, GPIO_MF_SEL1);
        
    gpio_config_struct.gpio_pin = GPIO_PIN_2 | GPIO_PIN_3;	 //PA2,PA3 USART2
    gpio_config_struct.gpio_mode = GPIO_MODE_MF;
    gpio_config_struct.gpio_speed = GPIO_SPEED_MEDIUM;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOA, &gpio_config_struct);	
	
		gpio_config_struct.gpio_pin = GPIO_PIN_9 | GPIO_PIN_10;	 //PA9,PA10 USART1
    gpio_config_struct.gpio_mode = GPIO_MODE_MF;
    gpio_config_struct.gpio_speed = GPIO_SPEED_MEDIUM;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_PP;
    gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
    gpio_init(GPIOA, &gpio_config_struct);
	
    // USART1 Config	
    usart_def_init(USART1);
    usart_config_struct.usart_rate = 115200;
    usart_config_struct.data_width = USART_DATA_WIDTH_8;
    usart_config_struct.stop_bits = USART_STOP_BIT_1;
    usart_config_struct.usart_parity = USART_PARITY_NO;
    usart_config_struct.flow_control = USART_FLOW_CONTROL_NONE;
    usart_config_struct.usart_mode = USART_MODE_TX | USART_MODE_RX;
    usart_init(USART1, &usart_config_struct);
		
		// USART2 Config	
    usart_def_init(USART2);
    usart_config_struct.usart_rate = 115200;
    usart_config_struct.data_width = USART_DATA_WIDTH_8;
    usart_config_struct.stop_bits = USART_STOP_BIT_1;
    usart_config_struct.usart_parity = USART_PARITY_NO;
    usart_config_struct.flow_control = USART_FLOW_CONTROL_NONE;
    usart_config_struct.usart_mode = USART_MODE_TX | USART_MODE_RX;
    usart_init(USART2, &usart_config_struct);
		
		/* Enable the USART Interrupt */
		nvic_config_struct.nvic_IRQ_channel = IRQn_USART1;
		nvic_config_struct.nvic_channel_priority = 0;
		nvic_config_struct.nvic_enable_flag = ENABLE;
		nvic_init(&nvic_config_struct);

    usart_enable_ctrl(USART1,ENABLE);	
		usart_enable_ctrl(USART2,ENABLE);	
		
		usart_interrupt_config(USART1, USART_INT_RXNE, ENABLE); //Enable the USART Receive interrupt
}


/***********************************************************************************
函数功能：TIMER初始化函数
函数参数：无
函数返回：无
***********************************************************************************/
void time_config(void)
{
	//1S中断一次
	timer_config_t  timer_config_struct;
	nvic_config_t   nvic_config_struct;
	uint16_t divider = 0;

	rcu_apb1_periph_clock_enable_ctrl(RCU_APB1_PERI_TIM3, ENABLE); //TIM3 clock enable 
	
	/*TIM3 Configuration: Encoder mode1:
	TIM3 input clock (TIM3CLK) is set to APB1 clock (PCLK1).
	TIM3CLK= PCLK1 = HCLK = SystemCoreClock

	When TIM3 counter clock = 8 MHz,
	divider = (TIM3CLK / TIM3 counter clock) - 1 =  ((SystemCoreClock) /8MHz) - 1

	The encoder mode is encoder mode1: Counter counts up/down on TI2 rising edge 
	depending on TI1 level 
	The Autoreload value is set to 800, so the encoder round is 800 TIM counter clock.*/   

	divider = (uint16_t) ((SystemCoreClock ) / 64000) - 1; 

	// Time base configuration
	timer_config_struct.time_period = 63999;														//设置周期值
	timer_config_struct.time_divide = 0;
	timer_config_struct.clock_divide = 0;
	timer_config_struct.count_mode = TIM_COUNT_PATTERN_UP;
	tim_timer_config(TIM3, &timer_config_struct);	
	tim_pdiv_register_config(TIM3, divider, TIM_PDIV_MODE_IMMEDIATE);		//设置预分配系数

	//Enable the TIM3  Interrupt
	nvic_config_struct.nvic_IRQ_channel = IRQn_TIM3;
	nvic_config_struct.nvic_channel_priority = 0;
	nvic_config_struct.nvic_enable_flag = ENABLE;
	nvic_init(&nvic_config_struct);
	
	tim_interrupt_config(TIM3, TIM_INTR_UPDATE, ENABLE); // TIM Interrupts enable 
	
	tim_enable_ctrl(TIM3, ENABLE);
}


/***********************************************************************************
函数功能：I2C初始化函数
函数参数：无
函数返回：无
***********************************************************************************/
void adc_i2c_init(void)
{ 
    i2c_config_t  i2c_config_struct;
    gpio_config_t  gpio_config_struct;
	
    rcu_ahb_periph_clock_enable_ctrl(RCU_AHB_PERI_PORTB, ENABLE);
    rcu_apb1_periph_clock_enable_ctrl(RCU_APB1_PERI_I2C1, ENABLE);

    //I2C Pin Config
    gpio_mf_config(GPIOB, GPIO_PIN_NUM6, GPIO_MF_SEL1);  // PB6--I2C_SCL
    gpio_mf_config(GPIOB, GPIO_PIN_NUM7, GPIO_MF_SEL1);  // PB7--I2C_SDA

    gpio_config_struct.gpio_pin = GPIO_PIN_6|GPIO_PIN_7;   
    gpio_config_struct.gpio_mode = GPIO_MODE_MF; gpio_config_struct.gpio_speed = GPIO_SPEED_LOW;
    gpio_config_struct.gpio_out_type = GPIO_OUTPUT_OD;
    gpio_config_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio_config_struct);
	
    //I2C configuration 
    i2c_config_struct.mode = I2C_BUS_MODE_I2C;
    i2c_config_struct.analog_flag = I2C_ANALOG_FILTER_ENABLE;
		i2c_config_struct.slave_addr1 = 0x00;
    i2c_config_struct.digital_value = 0x00; 
    i2c_config_struct.ack_flag = I2C_ACK_ENABLE;
    i2c_config_struct.ack_addr = I2C_ACK_ADDR_7BITS;
    i2c_config_struct.tmr_value = 0x00303E5D;   
  
    i2c_init(I2C1, &i2c_config_struct);
    i2c_enable_ctrl(I2C1, ENABLE);
  
}


/***********************************************************************************
函数功能：EXTI初始化函数
函数参数：无
函数返回：无
***********************************************************************************/
void exti0_1_config(void)
{ 
  gpio_config_t gpio_config_struct;
  exti_config_t exti_config_struct;
  nvic_config_t nvic_config_struct;
    
	rcu_ahb_periph_clock_enable_ctrl(RCU_AHB_PERI_PORTB, ENABLE); //Enable GPIOB clock 
	rcu_apb2_periph_clock_enable_ctrl(RCU_APB2_PERI_SYSCFG, ENABLE);//Enable SYSCFG clock

	//Configure PB0 pins as input
	gpio_config_struct.gpio_pin = GPIO_PIN_0;
	gpio_config_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_config_struct.gpio_pull = GPIO_PULL_NO_PULL;
	gpio_init(GPIOB, &gpio_config_struct);

	syscfg_exti_line_config(SYSCFG_EXTI_PORT_PB, SYSCFG_EXTI_PIN_0); //Connect EXTI0 Line to PB0 pin

	//Configure EXTI0 line 
	exti_config_struct.exti_line = EXTI_LINE_0;  
	exti_config_struct.exti_mode = EXTI_MODE_INTR;
	exti_config_struct.exti_trigger = EXTI_TRIGGER_RISING;
	exti_config_struct.exti_line_cmd = ENABLE;
	exti_init(&exti_config_struct);

	//Configure EXTI0line 
	exti_config_struct.exti_line = EXTI_LINE_0;
	exti_init(&exti_config_struct);

	//Enable and set EXTI0_1 Interrupt
	nvic_config_struct.nvic_IRQ_channel = IRQn_EXTI0_1;
	nvic_config_struct.nvic_channel_priority = 0x00;
	nvic_config_struct.nvic_enable_flag = ENABLE;
	nvic_init(&nvic_config_struct);
}


/***********************************************************************************
函数功能：EXTI中断使能函数
函数参数：enable -> 是否使能EXTI中断
函数返回：无
***********************************************************************************/
void exti0_1_interrupt(enable_state_t enable)
{ 
  nvic_config_t nvic_config_struct;

	//Enable and set EXTI0_1 Interrupt
	nvic_config_struct.nvic_IRQ_channel = IRQn_EXTI0_1;
	nvic_config_struct.nvic_channel_priority = 0x00;
	nvic_config_struct.nvic_enable_flag = enable;
	nvic_init(&nvic_config_struct);
}


/***********************************************************************************
函数功能：ECR8660初始化函数
函数参数：number -> 初始化多少个AD
函数返回：是否初始化成功，返回0为成功，返回1为失败
***********************************************************************************/
unsigned char cs32_ecr8660_init(unsigned char number)
{
	unsigned char ret = 0;
	unsigned char i = 0;
	for(i = 0; i < number; i++){
		//复位AD1
		if(!board_imformation[i].ad_lock_staet){
			spi_write(i,1+i*2,1);
			spi_write(i,1+i*2,0);
			spi_write(i,1+i*2,1);
		}
	}
		
	delay(1000);
	
	for(i = 0; i < number; i++){
		//初始化AD1
		if(!board_imformation[i].ad_lock_staet){
			//初始化AD1
			ECR8660_INIT(i);
		}
	}
			delay(1000);
		//下载代码到AD1
		ECR8660_CodeDownload();
		
	for(i = 0; i < number; i++){
		//软件复位AD1
		if(!board_imformation[i].ad_lock_staet){
			ECR8660_write(i,ECR8660_Inter_register, 0x7fc, 0);
			ECR8660_write(i,ECR8660_Inter_register, 0x7fc, 1);
		}
	}
		
			delay(2000);
	for(i = 0; i < number; i++){
		if(!board_imformation[i].ad_lock_staet){
			//读AD锁定状态寄存器
			ECR8660_Data = 0;
			ECR8660_Read(i,ECR8660_ecternal_register, 0x20000000, &ECR8660_Data);
			printf(" AD%d data 0x20000000: %8x ==data\r\n",i,ECR8660_Data);
			if(ECR8660_Data != 0x123)
			{
				ret = 1;
			}
		}
	}
	
	//AD0配置
	if(REG_Config(0,ad_reg_arr,sizeof(ad_reg_arr)/sizeof(AD_REG_T))){
		ret = 1;
	}

	return ret;
}


/***********************************************************************************
函数功能：配置flash存储的地址
函数参数：number -> 初始化多少个AD
函数返回：无
***********************************************************************************/
void flash_addr_config(unsigned char number)
{
	unsigned char i = 0;
	for(i = 0; i < number; i++){
		board_imformation[i].reg_start_addr = 0x08008000 + (0x1000*i);
		board_imformation[i].reg_number_addr = 0x08008004 + (0x1000*i);
	}
}


/***********************************************************************************
函数功能：flash数据更新函数
函数参数：number -> AD序号
函数返回：是否更新成功，返回0为成功，返回1为失败
***********************************************************************************/
unsigned char flash_config(unsigned char number)
{
	unsigned char ret = 0;
	unsigned char i = 0;
	printf("lock state : %d\r\n",board_imformation[number].ad_lock_staet);
	if(!board_imformation[number].ad_lock_staet){
		usart_interrupt_config(USART1, USART_INT_RXNE, DISABLE); // Disable the USART Receive interrupt
			//AD1 flash读写寄存器值
			flash_unlock();
			flash_flag_clear(FLASH_FLAG_ENDF | FLASH_FLAG_PGERR | FLASH_FLAG_WPERR);
			
			data_value = *(__IO uint32_t *)board_imformation[number].reg_start_addr;
			printf("AD%d start  mark: %8x\r\n",number,data_value);
			//取出存储在flash中之前寄存器的值
			
			if(FIRST_PROGRAMME == data_value){
				board_imformation[number].reg_number = *(__IO uint32_t *)board_imformation[number].reg_number_addr;
				if((board_imformation[number].reg_number > 0) && (board_imformation[number].reg_number <= 30))
				printf("AD%d register number is %d\r\n",number,board_imformation[number].reg_number);
				
				//往AD芯片写入之前存储在flash的寄存器和值
				if((board_imformation[number].reg_number > 0) && (board_imformation[number].reg_number <= 30)){
					memcpy(board_imformation[number].reg_arr,(unsigned int*)(board_imformation[number].reg_number_addr+4),board_imformation[number].reg_number*8);
					
					//写寄存器
					for(i = 0; i < board_imformation[number].reg_number; i++){
						if(Time_Out(number));
						else ret = 1;
						ECR8660_write(number,ECR8660_ecternal_register, board_imformation[number].reg_arr[i].reg, board_imformation[number].reg_arr[i].reg_value);
						delay(10);
						if(board_imformation[number].reg_arr[i].reg == 0x20004008){
							if(Time_Out(number));
							else ret = 1;
							ECR8660_write(number,ECR8660_ecternal_register, 0x00201180, 0x00000008);
							delay(100);
						}
						if(board_imformation[number].reg_arr[i].reg == 0x20004018){
							if(Time_Out(number));
							else ret = 1;
							ECR8660_write(number,ECR8660_ecternal_register, 0x00201180, 0x20000008);
							delay(100);
						}
						printf("AD%d write register : %8x value: %8x\r\n",number,board_imformation[number].reg_arr[i].reg,board_imformation[number].reg_arr[i].reg_value);
					}
					
				}
			}else{
				printf("AD%d this is first time write flash\r\n",number);
				flash_word_program(board_imformation[number].reg_start_addr,FIRST_PROGRAMME);
				board_imformation[number].reg_number = 0;
				printf("AD%d register number is %d\r\n",number,board_imformation[number].reg_number);
			}
			if(board_imformation[number].reg_number > 30){
				board_imformation[number].reg_number = 0;
				printf("AD%d register number is %d\r\n",number,board_imformation[number].reg_number);
			}
			flash_lock();
			
			printf("AD%d flash init complate\r\n",number);
		}
		usart_interrupt_config(USART1, USART_INT_RXNE, ENABLE); // Disable the USART Receive interrupt
		board_imformation[number].ad_lock_staet = 1;
		printf("ret : %d\r\n",ret);
		return ret;
}


/***********************************************************************************
函数功能：板载控制函数
函数参数：number -> AD的序号
函数返回：是否更新成功，返回0为成功，返回1为失败
***********************************************************************************/
unsigned char board_config(unsigned char number)
{
	unsigned ret = 0;
	unsigned char i = 0;
	unsigned char j = 0;
	switch(uart_data.type){
		//清空flash中修改过的AD1寄存器
		case 0x1:
			memset(board_imformation[number].reg_arr,0,sizeof(board_imformation[number].reg_arr));
			usart_interrupt_config(USART2, USART_INT_RXNE, DISABLE); // Disable the USART Receive interrupt
			flash_unlock(); 
			flash_flag_clear(FLASH_FLAG_ENDF | FLASH_FLAG_PGERR | FLASH_FLAG_WPERR);
			flash_page_erase(board_imformation[number].reg_start_addr);
			flash_lock(); 
			usart_interrupt_config(USART2, USART_INT_RXNE, ENABLE); // Enable the USART Receive interrupt
			board_imformation[number].reg_number = 0;
			printf("AD%d clear flash complate\r\n",number);
			break;
		
		//打印flash中存储的修改过的AD1寄存器
		case 0x2:
			usart_interrupt_config(USART2, USART_INT_RXNE, DISABLE); // Disable the USART Receive interrupt
			flash_unlock();                        
			flash_flag_clear(FLASH_FLAG_ENDF | FLASH_FLAG_PGERR | FLASH_FLAG_WPERR);
			if((board_imformation[number].reg_number > 0) && (board_imformation[number].reg_number <= 30)){//flash中有寄存器
				data_value = *(__IO uint32_t *)board_imformation[number].reg_number_addr;
				printf("AD%d register number is %d\r\n",number,data_value);
				for(i= 0; i < board_imformation[number].reg_number; i++){
					Sim_Pack(0x31,uart_data.id,board_imformation[number].reg_arr[i].reg,board_imformation[number].reg_arr[i].reg_value);
					Uart_Send_Buff(tx_buf,sizeof(tx_buf));
					printf("AD%d write register : %8x value: %8x\r\n",number,board_imformation[number].reg_arr[i].reg,board_imformation[number].reg_arr[i].reg_value);
				}
			}else{//flash中没有寄存器
				printf("AD%d flash have no register\r\n",number);
			}
			flash_lock();
			usart_interrupt_config(USART2, USART_INT_RXNE, ENABLE); // Enable the USART Receive interrupt
			break;
		
		//读AD1寄存器	
		case 0x3:
			if(Time_Out(number));
			else{ 
				ret = 1;
				board_imformation[number].ad_lock_staet = 0;
			}
			ECR8660_Read(number,ECR8660_SPI_Read, uart_data.reg, &ECR8660_Data);
			Sim_Pack(0x30,uart_data.id,uart_data.reg,ECR8660_Data);
			Uart_Send_Buff(tx_buf,sizeof(tx_buf));
			printf("AD%d addr:%8x,value:%8x\r\n",number,uart_data.reg,ECR8660_Data);
			delay(100);
		
			break;
		
		case 0x4:
			for(j = 0; j < board_imformation[number].reg_number; j++){
				if(uart_data.reg == board_imformation[number].reg_arr[j].reg){
					//旧寄存器改动新值
					printf("AD%d register is exist\r\n",number);
					board_imformation[number].reg_arr[j].reg_value = uart_data.value;
					exist_state = 1;
					break;
				}else{
					exist_state = 0;
				}
			}
			if(!exist_state){
				board_imformation[number].reg_arr[board_imformation[number].reg_number].reg = uart_data.reg;
				board_imformation[number].reg_arr[board_imformation[number].reg_number].reg_value = uart_data.value;
				printf("AD1 write addr : %8x , write value : %8x\r\n",board_imformation[number].reg_arr[board_imformation[number].reg_number].reg,board_imformation[number].reg_arr[board_imformation[number].reg_number].reg_value);
				if(board_imformation[number].reg_number < 30)
				board_imformation[number].reg_number++;
			}
			
			printf("AD%d reg number : %d\r\n",number,board_imformation[number].reg_number);
			//如果有上行或下行寄存器写入，要先写然后写一个特殊寄存器
			for(i = 0; i < board_imformation[number].reg_number; i++){
				if(Time_Out(number));
				else ret = 1;
				ECR8660_write(number,ECR8660_ecternal_register, board_imformation[number].reg_arr[i].reg, board_imformation[number].reg_arr[i].reg_value);
				if(board_imformation[number].reg_arr[i].reg == 0x20004008){
					if(Time_Out(number));
					else{ 
						ret = 1;
						board_imformation[number].ad_lock_staet = 0;
					}
					ECR8660_write(number,ECR8660_ecternal_register, 0x00201180, 0x00000008);
					delay(1000);
				}
				if(board_imformation[number].reg_arr[i].reg == 0x20004018){
					if(Time_Out(number));
					else{ 
						ret = 1;
						board_imformation[number].ad_lock_staet = 0;
					}
					ECR8660_write(number,ECR8660_ecternal_register, 0x00201180, 0x20000008);
					delay(1000);
				}
				
				printf("AD%d write register : %8x value: %8x\r\n",number,board_imformation[number].reg_arr[i].reg,board_imformation[number].reg_arr[i].reg_value);
			}
			
			printf("AD%d data programmer spi complate\r\n",number);
			
			usart_interrupt_config(USART1, USART_INT_RXNE, DISABLE); // Disable the USART Receive interrupt
			
			flash_unlock();                        
			flash_flag_clear(FLASH_FLAG_ENDF | FLASH_FLAG_PGERR | FLASH_FLAG_WPERR);
			flash_page_erase(board_imformation[number].reg_start_addr);
			
			flash_word_program(board_imformation[number].reg_start_addr,FIRST_PROGRAMME);
			flash_word_program(board_imformation[number].reg_number_addr,board_imformation[number].reg_number);
			
			for(i = 0; i < board_imformation[number].reg_number; i++){                
				flash_word_program(board_imformation[number].reg_start_addr + (8*(i+1)),board_imformation[number].reg_arr[i].reg);
				flash_word_program(board_imformation[number].reg_start_addr + (8*(i+1))+4,board_imformation[number].reg_arr[i].reg_value);
			}
			flash_lock();
			
			usart_interrupt_config(USART1, USART_INT_RXNE, ENABLE); // Enable the USART Receive interrupt
			
			printf("AD%d data programmer flash complate\r\n",number);

			break;
		
		case 0x40:
			scan_start_freq = uart_data.reg;
			scan_end_freq = uart_data.value;
			printf("scan freq config complate\r\n");
			break;
			
		case 0x50:
			printf("fpga read reg : %d,fpga read value : %d",rx_buf[5],spi_read(number,rx_buf[5]));
			break;
		
		case 0x51:
			spi_write(number,rx_buf[5],rx_buf[6]);
			printf("fpga write reg : %d,fpga write value : %d",rx_buf[5],rx_buf[6]);
			break;
		
		case 0xe0:
			cs_start_e2prom_buffer_write((unsigned char *)&uart_data.value,uart_data.reg,4);
			break;
		
	}
	return ret;
}








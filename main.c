#include "main.h"
#include "common.h"
#include "command.h"
#include "ECR8660.h"
#include <string.h>
#include <stdlib.h>
#include "spi_adc.h"
#include "i2c_ee.h"
#include "init.h"

#define FPGA_RESET_HIGH()					gpio_bits_set(GPIOA,GPIO_PIN_12)
#define FPGA_RESET_LOW()					gpio_bits_reset(GPIOA,GPIO_PIN_12)

unsigned char rx_buf [80];							//USART接收数组
unsigned char tx_buf [20];							//发送数组
unsigned char rx_index = 0x00;					//USART接收个数
unsigned char rx_flag = 0x00;						//USART接收是否完成标志位
unsigned char exist_state = 0;					//寄存器是否存在flash标志位
unsigned char fpga_state = 0;						//FPGA准备完成
unsigned char fpga_error = 0;						//FPGA运行出错标志
unsigned char fpga_error_count = 0;			//FPGA出错次数
unsigned char fpga_sync_count = 0;			//失步次数
unsigned char ret = 0;									//函数返回状态值
unsigned char fpga_select = 0; 					//fpga扫频选择
unsigned char scan_state = 0;						//扫频标志位
unsigned int data_value = 0;						//读取flash时的暂存变量
unsigned int time_out;									//超时时间
unsigned int fpga_data = 0;							//读fpga的寄存器值
unsigned int scan_start_freq = 0;				//起始扫描频点
unsigned int scan_end_freq = 0;					//结束扫描频点
unsigned int fpga_version = 0;					//FPGA版本
double freq = 0.0;											//频率

UART_PACK_T uart_data;
unsigned int freq_default;

extern BOARD_CONFIG_T board_imformation[BOARD_NUMBER];
extern PEAK_T peak_4g;

/**
  * @fn int main(void)
  * @brief  Main program.   MCU clock setting is  configured  through SystemInit() 
  *          in startup file (startup_cs32f0xx.s) before to enter to application main.       
  * @param  None
  * @return None
  */
int main(void)
{ 
		
		unsigned char i = 0;										//for循环变量
		memset(rx_buf,0,sizeof(rx_buf));
		memset(&uart_data,0,sizeof(UART_PACK_T));
	
		//定时器初始化
		time_config();
		
		//初始化滴答定时器
    cs_start_systick_config();
	
		//GPIO口初始化
		gpio_config();
	
		//初始化I2C
		adc_i2c_init(); 
	
		//IO口中断初始化
		exti0_1_config();
		
		//uart初始化
    usart_config();
	
		//初始化SPI
		spi_gpio_init();

init:

		spi_write(0,1,1);
		spi_write(0,1,0);
		spi_write(0,1,1);
		
		for(i = 0; i < BOARD_NUMBER; i++){		
			Fpga_Start(i);
		}
		
		if(gpio_input_data_bit_read(GPIOB,GPIO_PIN_3)){
			fpga_select = 0;
			spi_write(0,0x15,0);
			scan_start_freq = 6252;
			scan_end_freq = 6499;
		}
		else{
			fpga_select = 1;
			spi_write(0,0x15,1);
			scan_start_freq = 7777;
			scan_end_freq = 7916;
		}
		
		fpga_version = spi_read(0,0x19);
		printf("software version %s %s v1.2\r\n",__DATE__,__TIME__);
		printf("test mode: %d\r\n",TEST_MODE);
		printf("fpga select : %d\r\n",fpga_select);
		printf("fpga version : %d",fpga_version);
		
		//初始化AD
		if(cs32_ecr8660_init(BOARD_NUMBER))
			goto init;
		//射频是否锁住
		for(i = 0; i < BOARD_NUMBER; i++){
			if(rf_lock(i))
				goto init;
		}
		
		//亮灯提示AD已锁住
		gpio_bits_set(GPIOB,GPIO_PIN_9);
		
		//告诉FPGA AD已锁定
		for(i = 0; i < BOARD_NUMBER; i++){
			spi_write(i,2+i,1);
		}
		//初始化flash存储地址
		flash_addr_config(BOARD_NUMBER);
		
		//flash加载寄存器配置
		for(i = 0; i < BOARD_NUMBER; i++){
			if(flash_config(i))
				goto init;
		}
		
		//取出默认频点
		usart_interrupt_config(USART1, USART_INT_RXNE, DISABLE); // Disable the USART Receive interrupt
		//AD1 flash读写寄存器值
		flash_unlock();
		flash_flag_clear(FLASH_FLAG_ENDF | FLASH_FLAG_PGERR | FLASH_FLAG_WPERR);
		memcpy(&freq_default,(unsigned int*)(FREQ_DEFALUT_ADDR),4);
		flash_lock();
		usart_interrupt_config(USART1, USART_INT_RXNE, ENABLE); // Enable the USART Receive interrupt
		printf("default freq init complate\r\n");
		printf("default freq : %d\r\n",freq_default);
		
    while(1)
    {
			if(rx_flag){
				rx_flag = 0;
				memcpy(&uart_data,rx_buf,sizeof(UART_PACK_T));
				memset(rx_buf,0,sizeof(rx_buf));
				
				switch(uart_data.id)
				{
					case 0: 
						if(board_config(0)) goto init;
					break;
					
					case 1:
						if(board_config(1)) goto init;
						break;
				}	
				
				//清空暂存buff
				memset(&uart_data,0,sizeof(UART_PACK_T));
			}
			
			if(!scan_state){//扫频
				ret = Sweep_Frequency();
				if(ret == 1){
					printf("sync timeout\r\n");
					board_imformation[0].ad_lock_staet = 0;
					goto init;
				}else if(ret == 0){
					freq_default = peak_4g.freq;
					usart_interrupt_config(USART1, USART_INT_RXNE, DISABLE); // Disable the USART Receive interrupt
						
						flash_unlock();                        
						flash_flag_clear(FLASH_FLAG_ENDF | FLASH_FLAG_PGERR | FLASH_FLAG_WPERR);
						flash_page_erase(FREQ_DEFALUT_ADDR);
						
						for(i = 0; i < 5; i++){                
							flash_word_program(FREQ_DEFALUT_ADDR,freq_default);
						}
						flash_lock();
						
					usart_interrupt_config(USART1, USART_INT_RXNE, ENABLE); // Enable the USART Receive interrupt
					scan_state = 1;
				}
				printf("fpga sweep complate\r\n");
			}else{//同步
				if(fpga_error){
					fpga_error = 0;
					if(fpga_sync_count){//有同步
						printf("fpga sync count : %d",fpga_sync_count);
						fpga_sync_count = 0;
						fpga_error_count = 0;
					}else{//没有同步
						fpga_sync_count = 0;
						fpga_error_count++;
						printf("fpga not sync count : %d\r\n",fpga_error_count);
						if(fpga_error_count > 5){
							fpga_error_count = 0;
							printf("fpga sync again\r\n");
							scan_state = 0;
						}
						if(scan_state){
							spi_write(0,6,0);
							spi_write(0,6,1);
						}	
					}
				}
			}
		}	
}


/***********************************************************************************
函数功能：USART1中断函数
函数参数：无
函数返回：无
***********************************************************************************/
void USART1_IRQHandler(void)
{
	/* USART Receiver*/
    if (usart_interrupt_status_get(USART1, USART_INT_RXNE) == SET){
			
        rx_buf[rx_index++] = (uint8_t)usart_data_recv(USART1);
				if(rx_index == 4){
					//找到协议头
					if((rx_buf[0] != 0x43) || (rx_buf[1] != 0x53) || (rx_buf[2] != 0x41) || (rx_buf[3] != 0x44)){
						
						rx_buf[0] = rx_buf[1];
						rx_buf[1] = rx_buf[2];
						rx_buf[2] = rx_buf[3];
						rx_index = 3;
					}
				}
				//确定协议尾
        if((rx_index == 80)  ||((rx_buf[rx_index -4]== 0xfe) && (rx_buf[rx_index -3]== 0xfc) && (rx_buf[rx_index -2]== 0xfe) && (rx_buf[rx_index -1]== 0xfc))){
					
            rx_flag = 1;
            rx_index = 0;
        }

    }
}


/***********************************************************************************
函数功能：定时器3中断函数
函数参数：无
函数返回：无
***********************************************************************************/
void TIM3_IRQHandler(void)
{
		//1S进入中断一次
		static unsigned char fpga_count = 0;
		static unsigned char wdog_state = 0;
    if (tim_interrupt_status_get(TIM3, TIM_INTR_UPDATE) != RESET)
    {
        tim_interrupt_status_clear(TIM3, TIM_INTR_UPDATE); 
				fpga_count++;
				if(wdog_state){
					wdog_state = 0;
					gpio_bits_set(GPIOB,GPIO_PIN_4);
				}else{
					wdog_state = 1;
					gpio_bits_reset(GPIOB,GPIO_PIN_4);
				}
				
				if(fpga_count > 10){
					fpga_count = 0;
					fpga_error = 1;
				}
    }
}


/***********************************************************************************
函数功能：定时器2中断函数
函数参数：无
函数返回：无
***********************************************************************************/
void TIM2_IRQHandler(void)
{
		//1S进入中断一次
		static unsigned char fpga_count = 0;
		static unsigned char wdog_state = 0;
    if (tim_interrupt_status_get(TIM3, TIM_INTR_UPDATE) != RESET)
    {
        tim_interrupt_status_clear(TIM3, TIM_INTR_UPDATE); 
				fpga_count++;
				if(wdog_state){
					wdog_state = 0;
					gpio_bits_set(GPIOB,GPIO_PIN_4);
				}else{
					wdog_state = 1;
					gpio_bits_reset(GPIOB,GPIO_PIN_4);
				}
				
				if(fpga_count > 10){
					fpga_count = 0;
					fpga_error = 1;
				}
    }
}


/***********************************************************************************
函数功能：EXTI中断函数
函数参数：无
函数返回：无
***********************************************************************************/
void EXTI0_1_IRQHandler(void)
{
    if(exti_interrupt_status_get(EXTI_LINE_0) != RESET)
    {
				fpga_sync_count++;
        /* Clear the EXTI line 0  bit */
        exti_interrupt_status_clear(EXTI_LINE_0);
    }
  
}


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @return None
  */
int fputc(int ch, FILE *f)
{
    UNUSED(f);
    usart_data_send(USART1, (uint8_t) ch);
    while(RESET == usart_flag_status_get(USART1, USART_FLAG_TCF));
    return ch;
}


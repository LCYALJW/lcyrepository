#include "command.h"
#include "common.h"
#include "ECR8660.h"
#include "cs32f0xx_usart.h"

extern unsigned char tx_buf [20];

unsigned int rx_state = 0;							//RX锁定标志位
unsigned int tx_state = 0;							//TX锁定标志位
extern unsigned int time_out;
extern unsigned char i;							
extern unsigned char j;		
extern unsigned int ts_number;				
extern unsigned int reg_count;			
extern unsigned char exist_state;						
extern rgroup_t ts_buff[5];			
extern unsigned char rx_buf [80];
extern BOARD_CONFIG_T board_imformation[BOARD_NUMBER];


/***********************************************************************************
函数功能：检测AD是否处于忙状态
函数参数：ad_number -> AD的序号
函数返回：是否发生错误，返回1表示没有发生错误，返回0表示发生了错误
***********************************************************************************/
unsigned char Time_Out(unsigned char ad_number)
{
	unsigned char ret = 0;
	time_out = 0;
	switch(ad_number){
		case 0:
			while(FPGA_READ() && (time_out < AD_TIME_OUT)){
					time_out++;
					__nop();
				}
				if(time_out < AD_TIME_OUT){
					time_out = 0;
					ret = 1;
				}
				else{
					time_out = 0;
					ret = 0;
				}
			break;
				
		case 1:
			while(FPGA_READ_2() && (time_out < AD_TIME_OUT)){
					time_out++;
					__nop();
				}
				if(time_out < AD_TIME_OUT){
					time_out = 0;
					ret = 1;
				}
				else{
					time_out = 0;
					ret = 0;
				}
			break;
	}
	return ret;
	
}


/***********************************************************************************
函数功能：检测AD 射频信号是否锁住
函数参数：ad_number -> AD的序号
函数返回：是否发生错误，返回0表示没有发生错误，返回1表示发生了错误
***********************************************************************************/
unsigned char	rf_lock(unsigned char ad_number)
{
	unsigned char ret = 0;
	
	//判断AD RX频点是否锁住
	rx_state = 0;
	while((rx_state != 1) && (time_out < AD_TIME_OUT)){
		ECR8660_Read(ad_number,ECR8660_ecternal_register, 0x00200160, &rx_state);
		time_out++;
		__nop();
	}
	if(time_out < AD_TIME_OUT){
		time_out = 0;
		printf("AD%d RX is lock\r\n",ad_number);
	}else{
		time_out = 0;
		board_imformation[ad_number].ad_lock_staet = 0;
		printf("AD%d RX is not lock\r\n",ad_number);
		ret = 1;
	}
	
	//判断AD TX频点是否锁住
	tx_state = 0;
	while((rx_state != 1) && (time_out < AD_TIME_OUT)){
		ECR8660_Read(ad_number,ECR8660_ecternal_register, 0x00200060, &tx_state);
		time_out++;
		__nop();
	}
	if(time_out < AD_TIME_OUT){
		time_out = 0;
		printf("AD%d TX is lock\r\n",ad_number);
	}else{
		time_out = 0;
		board_imformation[ad_number].ad_lock_staet = 0;
		printf("AD%d TX is not lock\r\n",ad_number);
		ret = 1;
	}
		
	return ret;
}


/***********************************************************************************
函数功能：打包一个协议包
函数参数：type-> 协议类型 length-> value数据长度	value->数据
函数返回：无
***********************************************************************************/
void Sim_Pack(unsigned type,unsigned id,unsigned int reg,unsigned int reg_value)
{
	memset(tx_buf,0,sizeof(tx_buf));
	tx_buf[0] = 0x43;
	tx_buf[1] = 0x53;
	tx_buf[2] = 0x41;
	tx_buf[3] = 0x44;
	tx_buf[4] = 20;
	tx_buf[5] = 0;
	tx_buf[6] = id;
	tx_buf[7] = type;
	memcpy(&tx_buf[8],&reg,4);
	memcpy(&tx_buf[12],&reg_value,4);
	tx_buf[16] = 0xfe;
	tx_buf[17] = 0xfc;
	tx_buf[18] = 0xfe;
	tx_buf[19] = 0xfc;
}


/***********************************************************************************
函数功能：打包一个协议包
函数参数：buff-> 要发送的数据 length-> 要发送的数据长度
函数返回：无
***********************************************************************************/
void Uart_Send_Buff(unsigned char *buff,unsigned int length)
{
	unsigned i = 0;
	while(i < length)
	{
		usart_data_send(USART1, buff[i++]);
		while(RESET == usart_flag_status_get(USART1, USART_FLAG_TCF));
	}
}

#include "command.h"
#include "common.h"
#include "ECR8660.h"
#include "cs32f0xx_usart.h"

extern unsigned char tx_buf [20];

unsigned int rx_state = 0;							//RX������־λ
unsigned int tx_state = 0;							//TX������־λ
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
�������ܣ����AD�Ƿ���æ״̬
����������ad_number -> AD�����
�������أ��Ƿ������󣬷���1��ʾû�з������󣬷���0��ʾ�����˴���
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
�������ܣ����AD ��Ƶ�ź��Ƿ���ס
����������ad_number -> AD�����
�������أ��Ƿ������󣬷���0��ʾû�з������󣬷���1��ʾ�����˴���
***********************************************************************************/
unsigned char	rf_lock(unsigned char ad_number)
{
	unsigned char ret = 0;
	
	//�ж�AD RXƵ���Ƿ���ס
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
	
	//�ж�AD TXƵ���Ƿ���ס
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
�������ܣ����һ��Э���
����������type-> Э������ length-> value���ݳ���	value->����
�������أ���
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
�������ܣ����һ��Э���
����������buff-> Ҫ���͵����� length-> Ҫ���͵����ݳ���
�������أ���
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

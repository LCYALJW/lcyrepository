#ifndef _SPI_COMMAND_H
#define _SPI_COMMAND_H


unsigned char Time_Out(unsigned char ad_number);
unsigned char	rf_lock(unsigned char ad_number);
void Sim_Pack(unsigned type,unsigned id,unsigned int reg,unsigned int reg_value);
void Uart_Send_Buff(unsigned char *buff,unsigned int length);

#endif




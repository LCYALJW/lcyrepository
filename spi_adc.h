#ifndef _SPI_ADC_H_
#define _SPI_ADC_H_
#include <stdio.h>

enum command{
	FPGA_STATE,
	ADC_RESET,
	ADC_STATE,
	INTERRUPT_SIGN,
	MASTER_SYNC,
	SECONDARY_SYNC,
	FREQ_COMPLATE,
	NID2_0_PEAK_1,
	NID2_0_PEAK_2,
	NID2_0_PEAK_3,
	NID2_0_PEAK_4,
	NID2_1_PEAK_1,
	NID2_1_PEAK_2,
	NID2_1_PEAK_3,
	NID2_1_PEAK_4,
	NID2_2_PEAK_1,
	NID2_2_PEAK_2,
	NID2_2_PEAK_3,
	NID2_2_PEAK_4,
	NID2,
	NID1,
	FPGA_SYNC
};

typedef struct{
	unsigned char nid2;
	double freq;
	unsigned int peak;
}PEAK_T;


double compute_freq_from_gscn(int gscn);
void spi_write(unsigned char number,unsigned char addr,unsigned int send_data);
unsigned int  spi_read(unsigned char number,unsigned char addr);
unsigned char Sweep_Frequency(void);
void Fpga_Start(unsigned char number);

#endif


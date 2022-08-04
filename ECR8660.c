
/********************************************************************************
 * @ Copyright    : Transa Semi.
 * @
 * @ File Name    : ECR8660.c
 * @ File Mark    :
 * @ Others       :
 * @ Author       : GuoHao
 * @ Version      : v1.0
 * @ Date         : 2020-06-24 21:37:20
 * @ Description  :
 * @ History:
 * @       Version      : v1.0
 * @LastEditors: Please set LastEditors
 * @       LastEditTime : 2020-08-04 21:16:10
 * @       Modification :
 ********************************************************************************/


/****************************************************************************
*               Include file  eg:#include "header_file.h"
****************************************************************************/
#include "ECR8660.h"
#include "systick.h"
#include "w25x64.h"
#include <stdio.h>
#include "i2c_ee.h"
#include <string.h>
#include "common.h"

#define SPI_1_SET()							 gpio_bits_reset(GPIOB,GPIO_PIN_10)						//配置ADC1
#define SPI_2_SET()							 gpio_bits_set(GPIOB,GPIO_PIN_10)							//配置ADC2
#define SPI_AD()								 gpio_bits_set(GPIOB,GPIO_PIN_11)							//配置AD
#define SPI_FLASH_CS_LOW()       gpio_bits_reset(GPIOA, GPIO_PIN_4)
#define SPI_FLASH_CS_HIGH()      gpio_bits_set(GPIOA, GPIO_PIN_4)
#define CLK_LOW()       				 gpio_bits_reset(GPIOA, GPIO_PIN_5)          
#define CLK_HIGH()       				 gpio_bits_set(GPIOA, GPIO_PIN_5)
#define MOSI_LOW()       				 gpio_bits_reset(GPIOA, GPIO_PIN_7)
#define MOSI_HIGH()       			 gpio_bits_set(GPIOA, GPIO_PIN_7)
#define MISO_READ() 						 gpio_input_data_bit_read(GPIOA,GPIO_PIN_6)

extern BOARD_CONFIG_T board_imformation[BOARD_NUMBER];

/****************************************************************************
*               Macro Definition  eg:#define Macro Definition
****************************************************************************/

/****************************************************************************
*        Global Function Prototypes  eg:void Function_Definitions(void);
****************************************************************************/


/****************************************************************************
*            Global variable  eg:unsigned int Global_variable;
****************************************************************************/


// 用于拼凑时序的空间
ECR8660_FM_SPI_TIME_SERIES ECR8660_spi_time_series[3];
// 基地址的映射表
ECR8660_FM_BASE_ADDR_DICT  ECR8660_BaseAddr_Dict[] = {
    {0x00000,   0x00},
    {0x00001,   0x01},
    {0x00002,   0x02},
    {0x00003,   0x03},
    {0x00004,   0x04},
    {0x00005,   0x05},
    {0x00006,   0x06},
    {0x00007,   0x07},
    {0x00008,   0x08},
    {0x00009,   0x09},
    {0x0000A,   0x0A},
    {0x0000B,   0x0B},
    {0x0000C,   0x0C},
    {0x0000D,   0x0D},
    {0x0000E,   0x0E},
    {0x0000F,   0x0F},
    {0x20000,   0x10},
    {0x20001,   0x11},
    {0x20002,   0x12},
    {0x20003,   0x13},
    {0x20004,   0x14},
    {0x20005,   0x15},
    {0x20006,   0x16},
    {0x20007,   0x17},
    {0x00200,   0x18},
    {0x00201,   0x19},
    {0xFFFFF,   0xFF},
};
// ECR8660 的上电序列
ECR8660_Reg_List PwrUp_ECR8660_EN[] = {
    { ECR8660_SPI_Writ ,    1, 0xF08     , 0x83       , 0 },
    { ECR8660_SPI_Writ ,    1, 0xFFC     , 0x0        , 0 }, // 全局复位
    { ECR8660_SPI_Writ ,    1, 0xFFC     , 0x1        , 0 }, // 全局复位释放
    { ECR8660_SPI_Writ ,    1, 0xF00     , 0x0        , 0 }, // SPI_驱动
    { ECR8660_SPI_Writ ,    1, 0xF1C     , 0x1        , 0 },      //++++++//
    { ECR8660_SPI_Writ ,    1, 0xF20     , 0xFCD0     , 0 },      //++++++//
    { ECR8660_SPI_Writ ,    1, 0xF04     , 0x4741     , 0 },
    { ECR8660_SPI_Writ ,    1, 0xF40     , 0x03       , 0 },
    { ECR8660_SPI_Writ ,    1, 0xF40     , 0x01       , 0 }, // 释放隔离
    { ECR8660_SPI_Writ ,    1, 0xFFC     , 0x0        , 0 }, // 全局复位
    { ECR8660_SPI_Writ ,    1, 0xFFC     , 0x1        , 0 }, // 全局复位释放
    { ECR8660_SPI_Writ ,    1, 0x7FC     , 0x00       , 0 }, // M0复位
    { ECR8660_SPI_Writ ,    1, 0x7FC     , 0x01       , 0 }, // M0复位释放
    { ECR8660_SPI_Writ ,    0, 0x2002B0  , 0x0        , 0 },
    { ECR8660_SPI_Writ ,    0, 0x00200494, 0x0        , 0 },
    { ECR8660_SPI_Writ ,    0, 0x002004D4, 0x0        , 0 },
    { ECR8660_SPI_Writ ,    1, 0xF08     , 0xC3       , 0 },
    { ECR8660_SPI_Writ , 0xFF, 0xFFFFFFFF, 0xFFFFFFFF , 0 },
};

void AD_Select(unsigned char number)
{
	SPI_AD();
	/*switch(number){
		case 0:
				FPGA_GPIO1_HIGH();
				FPGA_GPIO2_LOW();
			break;
		case 1:
				FPGA_GPIO1_HIGH();
				FPGA_GPIO2_HIGH();
			break;
	}*/
}

/********************************************************************************
 * @ Description : SSC_SPI_Write_read
 * @ Parameter   : Trans_size
 *                 Addr
 *                 Data 
 * @ Return      : None
 * @ Note        : None
 ********************************************************************************/
INT32U SSC_SPI_Write_read(pSSCSpiTrans pdata)
{
    // 基于用户底层实现
	unsigned int rece_data = 0;
	
	uint8_t i = 0;
	
	SPI_FLASH_CS_LOW();
	
	__nop();
	__nop();
	__nop();
	__nop();
	
	if(pdata->Bits.cmdlen == 31){
		for(i = 0; i < 32; i++)
		{
			CLK_HIGH();
			
			if((pdata->Bits.spidata) & (0x80000000 >> i)){
				MOSI_HIGH();
			}else{
				MOSI_LOW();
			}
			if(i >= 16)
			{
				if(MISO_READ())
				{
					rece_data |= (0x1 << (31 - i));
				}
			}
			
			__nop();
			__nop();
			
			CLK_LOW();
			
			__nop();
			__nop();
		}
	}else{
		for(i = 0; i < 32; i++)
		{
			CLK_HIGH();
			
			if((pdata->Bits.spidata) & (0x80000000 >> i)){
				MOSI_HIGH();
			}else{
				MOSI_LOW();
			}
			if(i >= 16)
			{
				if(MISO_READ())
				{
					rece_data |= (0x1 << (47 - i));
				}
			}
			
			__nop();
			__nop();
			
			CLK_LOW();
			
			__nop();
			__nop();
			
		}
		
		for(i = 0; i < 16; i++)
		{
			CLK_HIGH();
			
			if((pdata->Bits.spidata2) & (0x80000000 >> i)){
				MOSI_HIGH();
			}else{
				MOSI_LOW();
			}
			if(MISO_READ())
			{
				rece_data |= (0x1 << (15 - i));
			}
			
			__nop();
			__nop();
			
			CLK_LOW();
			
			__nop();
			__nop();
			
		}
	}
	
	SPI_FLASH_CS_HIGH();
	CLK_LOW();
	MOSI_LOW();
	
	__nop();
	__nop();
	__nop();
	__nop();                                                                                                                             
	
	return rece_data;
	
}


INT8U ECR8660_SPI_WR(INT8U Read_write, INT8U Trans_size, INT16U Addr, INT64U Data)
{
		unsigned int tem_data = 0;
		
		ECR8660_spi_time_series[0].Word = 0;
		ECR8660_spi_time_series[1].Word = 0;
	
    switch (Trans_size)
    {
    case Trans_Internal_Register:
					tem_data = Read_write;
					tem_data <<= 31;
					ECR8660_spi_time_series[0].Word |= (tem_data & 0x80000000);
		
					tem_data = Trans_size;
					tem_data <<= 28;
					ECR8660_spi_time_series[0].Word |= (tem_data & 0x70000000);
		
					tem_data = Addr;
					tem_data <<= 16;
					ECR8660_spi_time_series[0].Word |= (tem_data & 0x0fff0000);
					tem_data =  (unsigned int)(Data & 0xffff);
					ECR8660_spi_time_series[0].Word |= (tem_data & 0x0000ffff);
					return (INT8U)Use_One_DataReg;
    case Trans_FourByte_Data:
					tem_data = Read_write;
					tem_data <<= 31;
					ECR8660_spi_time_series[0].Word |= (tem_data & 0x80000000);
		
					tem_data = Trans_size;
					tem_data <<= 28;
					ECR8660_spi_time_series[0].Word |= (tem_data & 0x70000000);
		
					tem_data = Addr;
					tem_data <<= 16;
					ECR8660_spi_time_series[0].Word |= (tem_data & 0x0fff0000);
					
					tem_data =  (unsigned int)(Data >> 16);
					ECR8660_spi_time_series[0].Word |= (tem_data & 0x0000ffff);
					
					tem_data =  (unsigned int)(Data << 16);
					ECR8660_spi_time_series[1].Word |= (tem_data & 0xffff0000);
        return (INT8U)Use_Two_DataReg;

    default:
        return 0;
    }
}

INT32U ECR8660_read_write(INT8U ReadWrite, INT8U Trans_size, INT32U Addr, INT64U SPI_Data)
{
    SSCSpiTrans            data;
    data.word[0]           =  0;
    data.word[1]           =  0;

    // 配置 接收 的数据的长度   n = len - 1
    data.Bits.datalen      =  (ECR8660_SPI_Read == ReadWrite) ? ((Trans_Internal_Register == Trans_size) ? 15 : 31) : 0;
    // 配置 发送 的数据的长度   n = len - 1
    data.Bits.cmdlen       =   (Trans_Internal_Register == Trans_size) ? 31 : 47;

    // 拼凑时序，将 ECR8660 所需要的 SPI 时序拼凑为 Data 数据
    ECR8660_SPI_WR(ReadWrite, Trans_size, Addr, SPI_Data);
    data.Bits.spidata      = ECR8660_spi_time_series[0].Word;
    data.Bits.spidata2     = ECR8660_spi_time_series[1].Word;

    return SSC_SPI_Write_read(&data);
}

/********************************************************************************
 * @ Description : _ECR8660_Base_Address_Dict_Find
 * @ Parameter   : Addr : 需要查询的地址
 * @ Return      : None
 * @ Note        : 在基地址字典(ECR8660_BaseAddr_Dict)内查询地址Addr的基地址标号
 ********************************************************************************/
INT8U _ECR8660_Base_Address_Dict_Find(INT32U Addr)
{
    INT16U i;
    for(i = 0; (0xFFFFF != ECR8660_BaseAddr_Dict[i].Base_Addr) && (ECR8660_BaseAddr_Dict[i].Base_Addr != Addr) ; i++ );
    return ECR8660_BaseAddr_Dict[i].Base_Log;
}

/********************************************************************************
 * @ Description : ECR8660_Base_Address
 * @ Parameter   : Addr 地址
 * @ Return      : None
 * @ Note        : 对页外寄存器访问时，进行页切换的函数
 ********************************************************************************/
void _ECR8660_Base_Address(INT32U Addr)
{
    ECR8660_read_write(ECR8660_SPI_Writ, Trans_Internal_Register, BASE_ADDR_REG, _ECR8660_Base_Address_Dict_Find(ECR8660_SPI_Base_Addr(Addr)));
}

/********************************************************************************
 * @ Description : ECR8660_write
* @ Parameter   :  Moudle_number:	AD的序号
									 ECR8660_mode : 指示页内寄存器（0x1） or 页外寄存器（0x0）
 *                 ECR8660_Addr : 需要操作的地址
 *                 ECR8660_Data : 需要操作的值
 * @ Return      : None
 * @ Note        : None
 ********************************************************************************/
void ECR8660_write(INT8U Moudle_number,INT8U ECR8660_mode, INT32U ECR8660_Addr, INT32U ECR8660_Data)
{
		AD_Select(Moudle_number);
		if(ECR8660_mode != 1)
				_ECR8660_Base_Address(ECR8660_Addr);
		ECR8660_read_write(ECR8660_SPI_Writ, ECR8660_mode ? Trans_Internal_Register : Trans_FourByte_Data, ECR8660_Addr, ECR8660_Data);
		
		
}

/********************************************************************************
 * @ Description : ECR8660_Read
 * @ Parameter   : Moudle_number:	AD的序号
									 ECR8660_mode : 指示页内寄存器（0x1） or 页外寄存器（0x0）
 *                 ECR8660_Addr : 需要操作的地址
 *                 ECR8660_Data : 返回的值
 * @ Return      : None
 * @ Note        : None
 ********************************************************************************/
void ECR8660_Read(INT8U Moudle_number,INT8U ECR8660_mode, INT32U ECR8660_Addr, INT32U *ECR8660_Data)
{
		AD_Select(Moudle_number);
		if(ECR8660_mode != 1)
				_ECR8660_Base_Address(ECR8660_Addr);
		*ECR8660_Data = (INT32U)ECR8660_read_write(ECR8660_SPI_Read, ECR8660_mode ? Trans_Internal_Register : Trans_FourByte_Data, ECR8660_Addr, 0);
		
}

/********************************************************************************
 * @ Description : ECR8660_Buff_Write
 * @ Parameter   : None
 * @ Return      : None
 * @ Note        : None
 ********************************************************************************/
INT32U ECR8660_Buff_Write(INT32U Addr, INT32U* Buff, INT32U Size)
{
    INT32U  i, Base_Addr = 0xFFFFFFFF;
	
    for (i = 0; i < Size; Addr += 4 , i += 4 ) {
        if(ECR8660_SPI_Base_Addr(Addr) != Base_Addr) {
            _ECR8660_Base_Address(Addr);
        }
        ECR8660_read_write(ECR8660_SPI_Writ, Trans_FourByte_Data, Addr, *((INT32U *)(Buff++)));
				delay(1);
    }
    ECR8660_read_write(ECR8660_SPI_Read, Trans_FourByte_Data, Addr, 0);
    return Addr;
}

/********************************************************************************
 * @ Description : ECR8660_CodeDownload
 * @ Parameter   : None
 * @ Return      : None
 * @ Note        : None
 ********************************************************************************/
void ECR8660_CodeDownload(void)
{
    INT32U  CodeSize = 0;
		INT16U  number = 4;
		INT32U 	i = 0;
		INT32U  j = 0;
		INT32U  k = 0;
		INT32U	Addr = 0;
		INT32U	addr = 0;
		INT8U 	ts_data[128];
		INT32U 	send_data = 0;
		INT32U  Base_Addr = 0xFFFFFFFF;
		INT32U	count = 0;
		
		memset(ts_data,0,sizeof(ts_data));
	
		cs_start_e2prom_buff_read((unsigned char *)&CodeSize,0x10,&number);
		
		printf("begin download ( Heap Memoryxxxx: 0x%08X )\r\n", (INT32U)CodeSize);
    if((CodeSize < 0x400) || (CodeSize > 0x10000)) {     // Bin文件大小限制为 1K 到 64K
        printf("ERR! ECR8660 code download time out\r\n");
        printf("******ERROR!   ERROR!  ERROR!******\r\n");
        return;
    }
		
		count = CodeSize / 128;
		count += 1;
		for(i = 0; i < count; Addr+=128,i++){
			
			number = 128;
			cs_start_e2prom_buff_read(ts_data,Addr,&number);
			
			for(j = 0; j < 128; addr+=4, j+=4){
				
				memcpy(&send_data,&ts_data[j],4);
				
				for(k = 0; k < BOARD_NUMBER; k++){
					if(!board_imformation[k].ad_lock_staet){
						AD_Select(k);
						if(ECR8660_SPI_Base_Addr(addr) != Base_Addr) {
								_ECR8660_Base_Address(addr);
						}
						ECR8660_read_write(ECR8660_SPI_Writ, Trans_FourByte_Data, addr, send_data);
					}
				}
			}
		}
}


INT32U _SetECR8660_to_List(INT8U Moudle_number,ECR8660_Reg_List List)
{
    INT32U data;
		if(ECR8660_SPI_SLEEP == List.readwrite){
				printf("Delay_us(%d)\r\n", List.data);
				delay(List.data);
				return 0;
		}
		if(ECR8660_SPI_Writ == List.readwrite) {
				ECR8660_write(Moudle_number,List.mode, List.addr, List.data);
				printf("write %d 0x%08X 0x%08X\r\n",List.mode, List.addr, List.data);
		}
		else {
				ECR8660_Read(Moudle_number,List.mode, List.addr, &List.data);
				printf("read  %d 0x%08X 0x%08X\r\n",List.mode, List.addr, List.data);
		}
    return data;
}

INT32U SetECR8660_to_List(INT8U Moudle_number,ECR8660_Reg_List *List)
{
    int i = 0;
	
    INT32U data;
		for(i = 0; List[i].mode != 0xFF; i++) {
				data = _SetECR8660_to_List(Moudle_number,List[i]);
		}
    
    return data;
}

void ECR8660_INIT(INT8U Moudle_number)
{
	SetECR8660_to_List(Moudle_number,PwrUp_ECR8660_EN);
   
}

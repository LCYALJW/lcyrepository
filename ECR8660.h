   
/*********************************************************************************
 * @ Copyright    : Transa Semi.
 * @
 * @ File Name    : ECR8660.h
 * @ File Mark    : 
 * @ Others       : 
 * @ Author       : GuoHao
 * @ Version      : v1.0
 * @ Date         : 2019-09-18 16:53:27
 * @ Description  : 
 * @ History:  
 * @       Version      : v1.0
 * @LastEditors: Please set LastEditors
 * @       LastEditTime : 2020-08-04 21:18:54
 * @       Modification :  
 ********************************************************************************/

   

#ifndef __ECR8660_H
#define __ECR8660_H

#ifdef __cplusplus
extern "C" {
#endif
/****************************************************************************
*                 Include file  eg:#include "header_file.h"
****************************************************************************/

/****************************************************************************
*               Macro Definition  eg:#define Macro Definition
****************************************************************************/
//      Read_write
#define ECR8660_SPI_Read            0x0                 // SPI读模式
#define ECR8660_SPI_Writ            0x1                 // SPI写模式
#define ECR8660_SPI_SLEEP           0x2
#define ECR8660_Inter_register   		0x1
#define ECR8660_ecternal_register		0x0

//      Trans_size
#define Trans_OneByte_Data          0x0                 // 传输1个byte数据
#define Trans_TwoByte_Data          0x1                 // 传输2个byte数据
#define Trans_FourByte_Data         0x3                 // 传输4个byte数据
#define Trans_EightByte_Data        0x7                 // 传输8个byte数据
#define Trans_Internal_Register     0x6                 // 作为SPI内部寄存器读写状态指示
//      Addr
#define ECR8660_SPI_Addr(Addr)      ((Addr) &  0xFFF)   // ECR8660的SPI时序中地址长度限制
#define ECR8660_SPI_Base_Addr(Addr) ((Addr) >> 12)      // 获取基地址
//      BASE_ADDR_REG
#define BASE_ADDR_REG               0x00000FF0

#define Use_One_DataReg             0x1                 // SPI总时序小于32Bit
#define Use_Two_DataReg             0x2                 // SPI总时序小于64Bit大于32Bit
#define Use_Thr_DataReg             0x3                 // SPI总时序小于80Bit大于64Bit

#define ECR8660_CODE_ADDR_HEAD      0x00000000

#define INT8U unsigned char 
#define INT16U unsigned short 
#define INT32U unsigned int 
#define INT64U unsigned long int 
#define CHAR char

/****************************************************************************
*           Function Definitions  eg:void Function_Definitions(void);
****************************************************************************/
void ECR8660_write(INT8U Moudle_number,INT8U ECR8660_mode, INT32U ECR8660_Addr, INT32U ECR8660_Data);
void ECR8660_Read(INT8U Moudle_number,INT8U ECR8660_mode, INT32U ECR8660_Addr, INT32U *ECR8660_Data);
INT32U ECR8660_read_write(INT8U ReadWrite, INT8U Trans_size, INT32U Addr, INT64U SPI_Data);
void ECR8660_CodeDownload(void);
void ECR8660_INIT(INT8U Moudle_number);
void AD_Select(unsigned char number);

/****************************************************************************
*            Global variable  eg:extern unsigned int Global_variable;
****************************************************************************/

typedef union _ECR8660_spi_comd_{
    INT16U Word;
    struct {
        INT16U Addr       : 12;
				INT16U Trans_size : 3;
				INT16U Read_Write : 1;
    }Bits;
}ECR8660_FM_SPI_CMD;

typedef union _ECR8660e_spi_data_{ // union低地址对齐
    INT64U Clear;       //元长度8
    struct {
        INT64U Reserve  : 56;
        INT64U Data     : 8;
    }OneByte;               // 元长度1 Byte
    struct {
        INT64U Reserve  : 48;
        INT64U Data     : 16;
    }TwoByte;               // 元长度2
    struct {
        INT64U Reserve  : 32;
        INT64U Data     : 32;
    }FourByte;              // 元长度4
    struct {
        INT64U Data     : 64;
    }EightByte;             // 元长度4
    struct {
        INT64U Low_16    : 16;
        INT64U Middle_32 : 32;
        INT64U High_16   : 16;
    }Bits;
}ECR8660_FM_SPI_DATA;

typedef union _ECR8660_fm_spi_time_series_{
    INT32U  Word;
    struct {
        INT32U Low_Half_Word   : 16;
        INT32U Hight_Half_Word : 16;
    }Bits;
}ECR8660_FM_SPI_TIME_SERIES;

typedef struct {
    INT32U  Base_Addr;
    INT16U  Base_Log;
}ECR8660_FM_BASE_ADDR_DICT;

typedef struct {
    INT8U   readwrite;
    INT8U   mode;
    INT32U  addr;
    INT32U  data;
    CHAR   *log;
}ECR8660_Reg_List;

typedef  union _ssc_spi_transfer_t_1_
{
    INT32U word[2];
    struct
    {
        INT32U datalen    : 5;
        INT32U cmdlen     : 6;
        INT32U reserved   : 21;
        INT32U spidata    : 32;
        INT32U spidata2   : 32;
    }Bits;
}*pSSCSpiTrans, SSCSpiTrans;

#ifdef __cplusplus
}
#endif

#endif /* __ECR8660_H */



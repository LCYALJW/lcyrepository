/**
  * @file    cs32f0xx_conf.h 
  * @brief   Library configuration file.
  * @author  ChipSea MCU Group
  * @version V1.0.0
  * @date   2018.11.01         
  * @copyright CHIPSEA TECHNOLOGIES (SHENZHEN) CORP.
  * @note 
  * <h2><center>&copy; COPYRIGHT 2018 ChipSea</center></h2>
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CS32F0XX_CONF_H__
#define __CS32F0XX_CONF_H__

#include "RTE_Components.h"             /* Component selection */

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver 
  */
#ifdef RTE_DEVICE_HAL_COMMON
#define HAL_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_ADC
#define HAL_ADC_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_CAN
#define HAL_CAN_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_CEC
#define HAL_CEC_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_COMP
#define HAL_COMP_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_CORTEX
#define HAL_CORTEX_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_CRC
#define HAL_CRC_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_DAC
#define HAL_DAC_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_DMA
#define HAL_DMA_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_FLASH
#define HAL_FLASH_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_GPIO
#define HAL_GPIO_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_I2C
#define HAL_I2C_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_I2S
#define HAL_I2S_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_IRDA
#define HAL_IRDA_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_FWDT
#define HAL_FWDT_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_PCD
#define HAL_PCD_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_PMU
#define HAL_PMU_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_RCU
#define HAL_RCU_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_RTC
#define HAL_RTC_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_SMARTCARD
#define HAL_SMARTCARD_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_SMBUS
#define HAL_SMBUS_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_SPI
#define HAL_SPI_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_TIM
#define HAL_TIM_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_TSC
#define HAL_TSC_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_UART
#define HAL_UART_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_USART
#define HAL_USART_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_WWDT
#define HAL_WWDT_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_DEBUG
#define HAL_DEBUG_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_EXTI
#define HAL_EXTI_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_SYSCFG
#define HAL_SYSCFG_MODULE_ENABLED
#endif
#ifdef RTE_DEVICE_HAL_FRAMEWORK
#define HAL_FRAMEWORK_MODULE_ENABLED
#endif

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */     
#define  VDD_VALUE                    3300U  /*!< Value of VDD in mv */           
#define  TICK_INT_PRIORITY            ((uint32_t)(1U<<__NVIC_PRIO_BITS) - 1U) /*!< tick interrupt priority (lowest by default)             */
                                                                              /*  Warning: Must be set to higher priority for HAL_Delay()  */
                                                                              /*  and HAL_GetTick() usage under interrupt context          */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     0U
#define  DATA_CACHE_ENABLE            0U
#define  USE_SPI_CRC                  1U

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/*#define USE_FULL_ASSERT    1*/

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file 
  */

#ifdef HAL_RCU_MODULE_ENABLED
    #include "cs32f0xx_rcu.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
    #include "cs32f0xx_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
    #include "cs32f0xx_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
    #include "cs32f0xx_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
    #include "cs32f0xx_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
    #include "cs32f0xx_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_CEC_MODULE_ENABLED
    #include "cs32f0xx_cec.h"
#endif /* HAL_CEC_MODULE_ENABLED */

#ifdef HAL_COMP_MODULE_ENABLED
    #include "cs32f0xx_comp.h"
#endif /* HAL_COMP_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
    #include "cs32f0xx_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
    #include "cs32f0xx_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
    #include "cs32f0xx_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
    #include "cs32f0xx_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
    #include "cs32f0xx_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
    #include "cs32f0xx_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_FWDT_MODULE_ENABLED
    #include "cs32f0xx_fwdt.h"
#endif /* HAL_FWDT_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
    #include "cs32f0xx_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_PMU_MODULE_ENABLED
    #include "cs32f0xx_pmu.h"
#endif /* HAL_PMU_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
    #include "cs32f0xx_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
    #include "cs32f0xx_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_SMBUS_MODULE_ENABLED
    #include "cs32f0xx_smbus.h"
#endif /* HAL_SMBUS_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
    #include "cs32f0xx_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
    #include "cs32f0xx_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_TSC_MODULE_ENABLED
    #include "cs32f0xx_tsc.h"
#endif /* HAL_TSC_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
    #include "cs32f0xx_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
    #include "cs32f0xx_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_WWDT_MODULE_ENABLED
    #include "cs32f0xx_wwdt.h"
#endif /* HAL_WWDT_MODULE_ENABLED */

#ifdef RTE_DEVICE_HAL_DEBUG
    #include "cs32f0xx_debug.h"
#endif /* RTE_DEVICE_HAL_DEBUG */

#ifdef RTE_DEVICE_HAL_EXTI
    #include "cs32f0xx_exti.h"
#endif /* RTE_DEVICE_HAL_EXTI */

#ifdef RTE_DEVICE_HAL_SYSCFG
    #include "cs32f0xx_syscfg.h"
#endif /* RTE_DEVICE_HAL_SYSCFG */

#ifdef HAL_FRAMEWORK_MODULE_ENABLED
    #include "cs32f0xx_misc.h"
#endif /* RTE_DEVICE_HAL_FRAMEWORK */
/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed. 
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((char *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
    void assert_failed(char* file, uint32_t line);
#else
    #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */    
    
#ifdef __cplusplus
}
#endif

#endif /* __CS32F0XX_CONF_H__ */


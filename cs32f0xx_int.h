#ifndef __CS32F0XX_INT_H__
#define __CS32F0XX_INT_H__

/**
  * @file     w25x64/cs32f0xx_int.h 
  * @brief   This file contains the headers of the interrupt handlers.
  * @author  ChipSea MCU Group
  * @version V1.0.0
  * @date   2018.11.01         
  * @copyright CHIPSEA TECHNOLOGIES (SHENZHEN) CORP.
  * @note 
  * <h2><center>&copy; COPYRIGHT 2018 ChipSea</center></h2>
  *
  */  


#ifdef __cplusplus
 extern "C" {
#endif 

#include "main.h"


/**
  * @fn void NMI_Handler(void)
  * @brief  This function handles NMI exception.
  * @param  None
  * @return None
  */
void NMI_Handler(void);

/**
  * @fn void HardFault_Handler(void)
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @return None
  */
void HardFault_Handler(void) __attribute__((noreturn));

/**
  * @fn void SVC_Handler(void)
  * @brief  This function handles SVCall exception.
  * @param  None
  * @return None
  */
void SVC_Handler(void);

/**
  * @fn void PendSV_Handler(void)
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @return None
  */
void PendSV_Handler(void);

/**
  * @fn void SysTick_Handler(void)
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @return None
  */
void SysTick_Handler(void);

/**
  * @fn void MemManage_Handler(void)
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @return None
  */
void MemManage_Handler(void) __attribute__((noreturn));

/**
  * @fn void MemManage_Handler(void)
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @return None
  */
void BusFault_Handler(void) __attribute__((noreturn));

/**
  * @fn void UsageFault_Handler(void)
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @return None
  */
void UsageFault_Handler(void) __attribute__((noreturn));

/**
  * @fn void DebugMon_Handler(void)
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @return None
  */
void DebugMon_Handler(void);


/**
  * @fn void EXTI4_15_IRQHandler(void)
  * @brief  This function handles External line 4 to 15 interrupt request.
  * @param  None
  * @return None
  */


#ifdef __cplusplus
}
#endif

#endif 


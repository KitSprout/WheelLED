/* #include "wheelled_core_bsp.h" */

#ifndef __WHEELLED_CORE_BSP_H
#define __WHEELLED_CORE_BSP_H

#include "stm32f4xx_hal.h"
/*====================================================================================================*/
/*====================================================================================================*/
//#define SampleRateFreg      ((uint16_t)200)   // 200Hz
//#define SampleRate          ((float)0.005f)   // 5.0ms
//#define SampleRateHelf      ((float)0.0025f)  // 2.5ms

#define SampleRateFreg      ((uint16_t)100)   // 200Hz
#define SampleRate          ((float)0.01f)    // 5.0ms

#define TIMx                TIM3      // 100MHz
#define TIMx_IRQ            TIM3_IRQn // 
#define TIMx_PRES           100       // 100MHz / 100 = 1MHz
#define TIMx_PERIOD         (uint32_t)((SystemCoreClock / TIMx_PRES) * SampleRate)  // SampleRateFreg
#define TIMx_CLK_ENABLE()   __HAL_RCC_TIM3_CLK_ENABLE()

extern TIM_HandleTypeDef WLED_TIM_HandleStruct;
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_UART_Config( void );
void WLED_IMU_Config( void );
void WLED_TIM_Config( void );
void WLED_WheelLED_Config( void );
/*====================================================================================================*/
/*====================================================================================================*/
#endif

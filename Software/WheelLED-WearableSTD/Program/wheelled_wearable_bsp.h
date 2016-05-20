/* #include "wheelled_wearable_bsp.h" */

#ifndef __WHEELLED_WEARABLE_BSP_H
#define __WHEELLED_WEARABLE_BSP_H

#include "stm32f0xx.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define SampleRateFreg      ((uint16_t)100)   // 200Hz
#define SampleRate          ((float)0.01f)    // 5.0ms

#define TIMx                TIM3      // 100MHz
#define TIMx_IRQ            TIM3_IRQn // 
#define TIMx_PRES           100       // 100MHz / 100 = 1MHz
#define TIMx_PERIOD         (uint32_t)((SystemCoreClock / TIMx_PRES) * SampleRate)  // SampleRateFreg
#define TIMx_CLK_ENABLE()   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)

#define KEY_PIN             GPIO_Pin_3
#define KEY_GPIO_PORT       GPIOA
#define KEY_Read()          (__GPIO_READ(KEY_GPIO_PORT, KEY_PIN) != KEY_PIN)

#define MOT_PIN             GPIO_Pin_1
#define MOT_GPIO_PORT       GPIOB
#define MOT_Set()           __GPIO_SET(MOT_GPIO_PORT, MOT_PIN)
#define MOT_Reset()         __GPIO_RST(MOT_GPIO_PORT, MOT_PIN)
#define MOT_Toggle()        __GPIO_TOG(MOT_GPIO_PORT, MOT_PIN)
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_GPIO_Config( void );
void WLED_MOT_Config( void );
void WLED_BT_Config( void );
void WLED_IMU_Config( void );
void WLED_TIM_Config( void );
/*====================================================================================================*/
/*====================================================================================================*/
#endif

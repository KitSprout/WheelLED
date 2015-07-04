/* #include "module_wheelLED.h" */

#ifndef __MODULE_WHEELWheelLED_H
#define __MODULE_WHEELWheelLED_H

#include "stm32f0xx.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define WLED_ADDR1  0x00
#define WLED_ADDR2  0x20
#define WLED_ADDR3  0x40
#define WLED_ADDR4  0x60
#define WLED_ADDR5  0x80
#define WLED_ADDR6  0xA0
#define WLED_ADDR7  0xC0
#define WLED_ADDR8  0xE0

#define WLED_CH1  0x00
#define WLED_CH2  0x04
#define WLED_CH3  0x08
#define WLED_CH4  0x0C
#define WLED_CH5  0x10
#define WLED_CH6  0x14
#define WLED_CH7  0x18
#define WLED_CH8  0x1C

#define WLED_ADDRESS_MASK 0xE0
#define WLED_ADDRESS_STEP 0x20
#define WLED_CHANNEL_MASK 0x1C
#define WLED_CHANNEL_STEP 0x04

#define WLED_CMD_SET          0xE0
#define WLED_CMD_SET_START    0x00
#define WLED_CMD_SET_END      0x04
#define WLED_CMD_SET_ID       0x08
#define WLED_CMD_SET_BOUDRATE 0x0C
#define WLED_CMD_IAP_START    0x10
#define WLED_CMD_REC1         0x14
#define WLED_CMD_REC2         0x18
#define WLED_CMD_REC3         0x1C

#define WLED_MODE_SET   0xF0
#define WLED_MODE_RUN   0x0F
#define WLED_MODE_IAP   0xFF

#define WheelLED_PWM_MIN  0
#define WheelLED_PWM_MED  128
#define WheelLED_PWM_MAX  256

#define WheelLED_PWM_CH1  TIM2->CCR1  // PA0
#define WheelLED_PWM_CH2  TIM2->CCR2  // PA1
#define WheelLED_PWM_CH3  TIM2->CCR3  // PA2
#define WheelLED_PWM_CH4  TIM2->CCR4  // PA3
#define WheelLED_PWM_CH5  TIM14->CCR1 // PA4
#define WheelLED_PWM_CH6  TIM3->CCR1  // PA6
#define WheelLED_PWM_CH7  TIM3->CCR2  // PA7
#define WheelLED_PWM_CH8  TIM3->CCR4  // PB1
/*====================================================================================================*/
/*====================================================================================================*/
void WheelLED_Config( void );
void WheelLED_RecvData( uint8_t *recvByte );

extern __IO uint8_t WheelLED_Mode;
extern __IO uint8_t WheelLED_DeviceID;
/*====================================================================================================*/
/*====================================================================================================*/
#endif

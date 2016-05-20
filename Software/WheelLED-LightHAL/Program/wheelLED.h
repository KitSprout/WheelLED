/* #include "wheelLED.h" */

#ifndef __WHEELLED_H
#define __WHEELLED_H

#include "stm32f0xx_hal.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define LED_PIN                     GPIO_PIN_5
#define LED_GPIO_PORT               GPIOA
#define LED_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED_Set                     _GPIO_SET(LED_GPIO_PORT, LED_PIN)
#define LED_Reset                   _GPIO_RST(LED_GPIO_PORT, LED_PIN)
#define LED_Toggle                  _GPIO_TOG(LED_GPIO_PORT, LED_PIN)

#define WLED_PWM_CH1  TIM2->CCR1  // PA0
#define WLED_PWM_CH2  TIM2->CCR2  // PA1
#define WLED_PWM_CH3  TIM2->CCR3  // PA2
#define WLED_PWM_CH4  TIM2->CCR4  // PA3
#define WLED_PWM_CH5  TIM14->CCR1 // PA4
#define WLED_PWM_CH6  TIM3->CCR1  // PA6
#define WLED_PWM_CH7  TIM3->CCR2  // PA7
#define WLED_PWM_CH8  TIM3->CCR4  // PB1

#define WLED_PWM_MIN  0
#define WLED_PWM_MED  128
#define WLED_PWM_MAX  256
/*====================================================================================================*/
/*====================================================================================================*/
#define WLED_ADDR1            ((uint8_t)0x00)
#define WLED_ADDR2            ((uint8_t)0x20)
#define WLED_ADDR3            ((uint8_t)0x40)
#define WLED_ADDR4            ((uint8_t)0x60)
#define WLED_ADDR5            ((uint8_t)0x80)
#define WLED_ADDR6            ((uint8_t)0xA0)
#define WLED_ADDR7            ((uint8_t)0xC0)
#define WLED_ADDR8            ((uint8_t)0xE0)
#define WLED_ADDRESS_MASK     ((uint8_t)0xE0)
#define WLED_ADDRESS_STEP     ((uint8_t)0x20)

#define WLED_CH1              ((uint8_t)0x00)
#define WLED_CH2              ((uint8_t)0x04)
#define WLED_CH3              ((uint8_t)0x08)
#define WLED_CH4              ((uint8_t)0x0C)
#define WLED_CH5              ((uint8_t)0x10)
#define WLED_CH6              ((uint8_t)0x14)
#define WLED_CH7              ((uint8_t)0x18)
#define WLED_CH8              ((uint8_t)0x1C)
#define WLED_CHANNEL_MASK     ((uint8_t)0x1C)
#define WLED_CHANNEL_STEP     ((uint8_t)0x04)

#define WLED_CMD_SET          ((uint8_t)0xE0)
#define WLED_CMD_SET_START    ((uint8_t)0x00)
#define WLED_CMD_SET_END      ((uint8_t)0x04)
#define WLED_CMD_SET_ID       ((uint8_t)0x08)
#define WLED_CMD_SET_BOUDRATE ((uint8_t)0x0C)
#define WLED_CMD_IAP_START    ((uint8_t)0x10)
#define WLED_CMD_RESTORE      ((uint8_t)0x14)
#define WLED_CMD_REC2         ((uint8_t)0x18)
#define WLED_CMD_REC3         ((uint8_t)0x1C)

#define WLED_MODE_SET         ((uint8_t)0xF0)
#define WLED_MODE_RUN         ((uint8_t)0x0F)
#define WLED_MODE_IAP         ((uint8_t)0xFF)
/*====================================================================================================*/
/*====================================================================================================*/
void WheelLED_Config( void );
void WheelLED_Init( void );

void WheelLED_RecvData( uint8_t *recvByte );
void WheelLED_IRQHandler( void );
/*====================================================================================================*/
/*====================================================================================================*/
#endif

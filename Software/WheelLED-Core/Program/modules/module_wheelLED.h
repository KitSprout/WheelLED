/* #include "module_wheelLED.h" */

#ifndef __MODULE_WHEELLED_H
#define __MODULE_WHEELLED_H

#include "stm32f1xx_hal.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define WLED_TOTAL_LEDS 56
#define WLED_LED_PER_DEGREE (WLED_TOTAL_LEDS / 360.0f)

#define WLED_ADDR1  ((uint8_t)0x00)
#define WLED_ADDR2  ((uint8_t)0x20)
#define WLED_ADDR3  ((uint8_t)0x40)
#define WLED_ADDR4  ((uint8_t)0x60)
#define WLED_ADDR5  ((uint8_t)0x80)
#define WLED_ADDR6  ((uint8_t)0xA0)
#define WLED_ADDR7  ((uint8_t)0xC0)
#define WLED_ADDR8  ((uint8_t)0xE0)

#define WLED_CH1    ((uint8_t)0x00)
#define WLED_CH2    ((uint8_t)0x04)
#define WLED_CH3    ((uint8_t)0x08)
#define WLED_CH4    ((uint8_t)0x0C)
#define WLED_CH5    ((uint8_t)0x10)
#define WLED_CH6    ((uint8_t)0x14)
#define WLED_CH7    ((uint8_t)0x18)
#define WLED_CH8    ((uint8_t)0x1C)

#define WLED_ADDRESS_MASK   ((uint8_t)0xE0)
#define WLED_ADDRESS_STEP   ((uint8_t)0x20)
#define WLED_CHANNEL_MASK   ((uint8_t)0x1C)
#define WLED_CHANNEL_STEP   ((uint8_t)0x04)

#define WLED_CMD_SET          ((uint8_t)0xE0)
#define WLED_CMD_SET_MODE     ((uint8_t)0x00)
#define WLED_CMD_RUN_MODE     ((uint8_t)0x04)
#define WLED_CMD_SET_ID       ((uint8_t)0x08)
#define WLED_CMD_SET_BOUDRATE ((uint8_t)0x0C)
#define WLED_CMD_IAP_MODE     ((uint8_t)0x10)
#define WLED_CMD_RESTORE      ((uint8_t)0x14)
#define WLED_CMD_REC2         ((uint8_t)0x18)
#define WLED_CMD_REC3         ((uint8_t)0x1C)

#define WLED_MODE_SET   ((uint8_t)0xF0)
#define WLED_MODE_RUN   ((uint8_t)0x0F)
#define WLED_MODE_IAP   ((uint8_t)0xFF)

#define WLED_LIGHT_MAX  255
#define WLED_LIGHT_MIN  0
/*====================================================================================================*/
/*====================================================================================================*/
void WheelLED_Config( void );
void WheelLED_testFunc( void );
void WheelLED_testFunc_1( void );

void WheelLED_setLED( uint16_t ang, uint16_t angScatter, uint8_t light );
void WheelLED_setLED_byAddr( uint8_t addr, uint8_t ch, uint8_t sendData );
void WheelLED_setLED_byNum( uint8_t num, uint8_t sendData );
void WheelLED_setLED_all( uint8_t sendData );

void WheelLED_CMD( uint8_t sendCMD, uint8_t sendData );
void WheelLED_CMD_SetID( void );
void WheelLED_CMD_Restore( void );

void WheelLED_setLED_tmp( uint16_t ang, uint16_t angScatter, uint8_t light );
/*====================================================================================================*/
/*====================================================================================================*/
#endif

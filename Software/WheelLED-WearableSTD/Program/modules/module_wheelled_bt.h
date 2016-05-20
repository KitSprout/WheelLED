/* #include "module_wheelLED_bt.h" */

#ifndef __MODULE_WHEELLED_BT_H
#define __MODULE_WHEELLED_BT_H

#include <stdio.h>

#include "stm32f0xx.h"
/*====================================================================================================*/
/*====================================================================================================*/
/* Connand */
#define LINKIT_CMD_KEYOFF 0
#define LINKIT_CMD_KEYON  1
#define LINKIT_CMD_LOCK   2
#define LINKIT_CMD_UNLOCK 3
#define LINKIT_CMD_ANGLE  4
#define LINKIT_CMD_LIGHT  5
#define LINKIT_CMD_WIDTH  6
#define LINKIT_WARN_TILT  7
#define LINKIT_WARN_RSSI  8
/*====================================================================================================*/
/*====================================================================================================*/
void    WheelLED_BT_Config( void );
void    WheelLED_BT_SendData( uint8_t cmd, uint16_t sendData );
uint8_t WheelLED_BT_RecvData( void );
void    WheelLED_BT_evenCallBack( void );
/*====================================================================================================*/
/*====================================================================================================*/
#endif

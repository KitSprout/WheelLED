/* #include "module_linkit.h" */

#ifndef __MODULE_LINKIT_H
#define __MODULE_LINKIT_H

#include "stm32f1xx_hal.h"
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
/*====================================================================================================*/
/*====================================================================================================*/
void Linkit_SerialConfig( void );
void Linkit_SendByte( uint8_t sendByte );
void Linkit_SendData( uint8_t cmd, uint16_t sendData );
uint8_t Linkit_RecvData( void );
void Linkit_Interrupt_CallBack( void );
/*====================================================================================================*/
/*====================================================================================================*/
#endif

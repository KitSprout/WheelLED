/* #include "wheelLED_param.h" */

#ifndef __WHEELLED_PARAM_H
#define __WHEELLED_PARAM_H

#include "stm32f0xx_hal.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define PARAM_TYPE_MASK   0xC0
#define PARAM_TYPE_INT    0x00
#define PARAM_TYPE_FLOAT  0x40
#define PARAM_TYPE_STRING 0x80
#define PARAM_TYPE_ARRAY  0xC0

#define PARAM_SIGN_MASK   0x20
#define PARAM_SIGNED      0x00
#define PARAM_UNSIGNED    0x20

#define PARAM_BYTE_MASK   0x1F
#define PARAM_SIZE_1BYTE  0x01
#define PARAM_SIZE_2BYTE  0x02
#define PARAM_SIZE_4BYTE  0x04
#define PARAM_SIZE_8BYTE  0x08

#define TYPE_S8     ((uint8_t)0x01)  // ((uint8_t)(PARAM_TYPE_INT    | PARAM_SIGNED   | PARAM_SIZE_1BYTE))
#define TYPE_U8     ((uint8_t)0x21)  // ((uint8_t)(PARAM_TYPE_INT    | PARAM_UNSIGNED | PARAM_SIZE_1BYTE))
#define TYPE_S16    ((uint8_t)0x02)  // ((uint8_t)(PARAM_TYPE_INT    | PARAM_SIGNED   | PARAM_SIZE_2BYTE))
#define TYPE_U16    ((uint8_t)0x22)  // ((uint8_t)(PARAM_TYPE_INT    | PARAM_UNSIGNED | PARAM_SIZE_2BYTE))
#define TYPE_S32    ((uint8_t)0x04)  // ((uint8_t)(PARAM_TYPE_INT    | PARAM_SIGNED   | PARAM_SIZE_4BYTE))
#define TYPE_U32    ((uint8_t)0x24)  // ((uint8_t)(PARAM_TYPE_INT    | PARAM_UNSIGNED | PARAM_SIZE_4BYTE))
#define TYPE_S64    ((uint8_t)0x08)  // ((uint8_t)(PARAM_TYPE_INT    | PARAM_SIGNED   | PARAM_SIZE_8BYTE))
#define TYPE_U64    ((uint8_t)0x28)  // ((uint8_t)(PARAM_TYPE_INT    | PARAM_UNSIGNED | PARAM_SIZE_8BYTE))
#define TYPE_FP32   ((uint8_t)0x44)  // ((uint8_t)(PARAM_TYPE_FLOAT  | PARAM_SIGNED   | PARAM_SIZE_4BYTE))
#define TYPE_FP64   ((uint8_t)0x48)  // ((uint8_t)(PARAM_TYPE_FLOAT  | PARAM_SIGNED   | PARAM_SIZE_8BYTE))
#define TYPE_STR    ((uint8_t)0x80)  // ((uint8_t)(PARAM_TYPE_STRING | PARAM_SIGNED   | PARAM_BYTE_MASK))
#define TYPE_ARR    ((uint8_t)0xC0)  // ((uint8_t)(PARAM_TYPE_ARRAY  | PARAM_SIGNED   | PARAM_BYTE_MASK))
/*====================================================================================================*/
/*====================================================================================================*/
typedef __IO struct {
  uint8_t TYPE;
  uint8_t *NAME;
  void *ADDR;
} PARAM_ST;

enum {
  DEVICE_ID = 0,
  DEVICE_NAME,
  DEVICE_ADDR,
  FIRMWARE_VER,
  LAST_UPDATE,
  STARTUP_TIMES,

  BAUDRATE,

  PARAM_SIZE
};
/*====================================================================================================*/
/*====================================================================================================*/
HAL_StatusTypeDef WheelLED_getParams( void );
HAL_StatusTypeDef WheelLED_saveParams( void );
void              WheelLED_setParams( void );

void              WheelLED_testParam( void );
/*====================================================================================================*/
/*====================================================================================================*/
#endif

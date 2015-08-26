/*====================================================================================================*/
/*====================================================================================================*/
#include "Dirvers\stm32f0_system.h"
#include "Dirvers\stm32f0_flash.h"

#include "wheelLED.h"
#include "wheelLED_param.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define PARAM_SAVE_ADDR     FLASH_PAGE_ADDR(15)
#define PARAM_SAVE_LENS     FLASH_PAGE_SIZE

#define PARAM_PACKAGE_LENS  16
/*====================================================================================================*/
/*====================================================================================================*/
extern __IO uint8_t  WheelLED_address;
extern __IO uint32_t WheelLED_baudrate;

static __IO uint32_t WheelLED_id           = 0x19910422;
static __IO uint8_t  WheelLED_name[32]     = "WheelLED";
static __IO uint32_t WheelLED_firmwareVer  = 1000;        // 1.0
static __IO uint32_t WheelLED_lastUpdate   = 0x20150823;  // 2015.08.23
static __IO uint32_t WheelLED_startupTimes = 0;

PARAM_ST PARAM[PARAM_SIZE] =
{
  /* Board Info*/
  [DEVICE_ID]     = { .TYPE = TYPE_U32,  .NAME = (uint8_t*)"DEVICE_ID",     (void*)(&WheelLED_id) },

  /* Device Info */
  [DEVICE_NAME]   = { .TYPE = TYPE_STR,  .NAME = (uint8_t*)"DEVICE_NAME",   (void*)(&WheelLED_name) },
  [DEVICE_ADDR]   = { .TYPE = TYPE_U8,   .NAME = (uint8_t*)"DEVICE_ADDR",   (void*)(&WheelLED_address) },
  [FIRMWARE_VER]  = { .TYPE = TYPE_U32,  .NAME = (uint8_t*)"FIRMWARE_VER",  (void*)(&WheelLED_firmwareVer) },
  [LAST_UPDATE]   = { .TYPE = TYPE_U32,  .NAME = (uint8_t*)"LAST_UPDATE",   (void*)(&WheelLED_lastUpdate) },
  [STARTUP_TIMES] = { .TYPE = TYPE_U32,  .NAME = (uint8_t*)"STARTUP_TIMES", (void*)(&WheelLED_startupTimes) },

  /* Bautrate */
  [BAUDRATE]      = { .TYPE = TYPE_U32,  .NAME = (uint8_t*)"BAUDRATE",      (void*)(&WheelLED_baudrate) },
};

// Parameter Package Formate
// byte[00]    - HeaderInfo
// byte[01]    - WheelLED_id
// byte[02:09] - WheelLED_name
// byte[10]    - WheelLED_address
// byte[11]    - WheelLED_firmwareVer
// byte[12]    - WheelLED_lastUpdate
// byte[13]    - WheelLED_startupTimes
// byte[14]    - WheelLED_baudrate
// byte[15]    - CheckSum

/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_getParams
**功能 : WheelLED get Parameter
**輸入 : None
**輸出 : None
**使用 : WheelLED_getParams();
**====================================================================================================*/
/*====================================================================================================*/
HAL_StatusTypeDef WheelLED_getParams( void )
{
  uint32_t checkSum = 0;
  uint32_t readBuf[PARAM_PACKAGE_LENS] = {0};

  Flash_ReadPageU32(PARAM_SAVE_ADDR, readBuf, PARAM_PACKAGE_LENS);
  for(uint8_t i = 0; i < readBuf[0] - 1; i++)
    checkSum += (readBuf[i] % 1024);

  if(readBuf[15] != checkSum)
    return HAL_ERROR;

  *(uint32_t*)(PARAM[DEVICE_ID].ADDR)     = readBuf[1];
  *(uint8_t*) (PARAM[DEVICE_ADDR].ADDR)   = readBuf[10] & WLED_ADDRESS_MASK;
  *(uint32_t*)(PARAM[FIRMWARE_VER].ADDR)  = readBuf[11];
  *(uint32_t*)(PARAM[LAST_UPDATE].ADDR)   = readBuf[12];
  *(uint32_t*)(PARAM[STARTUP_TIMES].ADDR) = readBuf[13];
  *(uint32_t*)(PARAM[BAUDRATE].ADDR)      = readBuf[14];

  return HAL_OK;
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_saveParams
**功能 : WheelLED Save Parameter
**輸入 : None
**輸出 : None
**使用 : WheelLED_saveParams();
**====================================================================================================*/
/*====================================================================================================*/
HAL_StatusTypeDef WheelLED_saveParams( void )
{
  uint32_t checkSum = 0;
  uint32_t readBuf[PARAM_PACKAGE_LENS] = {0};

  Flash_ReadPageU32(PARAM_SAVE_ADDR, readBuf, PARAM_PACKAGE_LENS);
  for(uint8_t i = 0; i < readBuf[0] - 1; i++)
    checkSum += (readBuf[i] % 1024);

  if(readBuf[15] != checkSum)
    return HAL_ERROR;

  checkSum = 0;
  readBuf[1]  = *(uint32_t*)(PARAM[DEVICE_ID].ADDR);
  readBuf[10] = *(uint8_t*) (PARAM[DEVICE_ADDR].ADDR);
  readBuf[11] = *(uint32_t*)(PARAM[FIRMWARE_VER].ADDR);
  readBuf[12] = *(uint32_t*)(PARAM[LAST_UPDATE].ADDR);
  readBuf[13] = *(uint32_t*)(PARAM[STARTUP_TIMES].ADDR);
  readBuf[14] = *(uint32_t*)(PARAM[BAUDRATE].ADDR);
  for(uint8_t i = 0; i < readBuf[0] - 1; i++)
    checkSum += (readBuf[i] % 1024);
  readBuf[15] = checkSum;
  Flash_ErasePages(PARAM_SAVE_ADDR, 1);
  Flash_WritePageU32(PARAM_SAVE_ADDR, readBuf, PARAM_PACKAGE_LENS);

  return HAL_OK;
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_setParams
**功能 : WheelLED set Parameter
**輸入 : None
**輸出 : None
**使用 : WheelLED_setParams();
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_setParams( void )
{
  uint32_t checkSum = 0;
  uint32_t readBuf[PARAM_PACKAGE_LENS] = {0};

  readBuf[0]  = 16;
  readBuf[1]  = WheelLED_id;
  readBuf[2]  = 'W';
  readBuf[3]  = 'h';
  readBuf[4]  = 'e';
  readBuf[5]  = 'e';
  readBuf[6]  = 'l';
  readBuf[7]  = 'L';
  readBuf[8]  = 'E';
  readBuf[9]  = 'D';
  readBuf[10] = WLED_ADDR1;
  readBuf[11] = WheelLED_firmwareVer;
  readBuf[12] = WheelLED_lastUpdate;
  readBuf[13] = WheelLED_startupTimes;
  readBuf[14] = WheelLED_baudrate;
  for(uint8_t i = 0; i < readBuf[0] - 1; i++)
    checkSum += (readBuf[i] % 1024);
  readBuf[15] = checkSum;

  Flash_ErasePages(PARAM_SAVE_ADDR, 1);
  Flash_WritePageU32(PARAM_SAVE_ADDR, readBuf, PARAM_PACKAGE_LENS);
}
/*====================================================================================================*/
/*====================================================================================================*/
//void WheelLED_testParam( void )
//{
//  uint32_t checkSum = 0;
//  uint32_t readBuf[PARAM_PACKAGE_LENS] = {0};

//  WheelLED_Config();
////  WheelLED_setParams();
//  Flash_ReadPageU32(PARAM_SAVE_ADDR, readBuf, PARAM_PACKAGE_LENS);

//  for(uint8_t i = 0; i < readBuf[0] - 1; i++) {
//    checkSum += (readBuf[i] % 1024);
//  }
//  if(readBuf[15] != checkSum) {
//    while(1) {
//      LED_Reset;
//      Delay_100ms(10);
//      LED_Set;
//      Delay_100ms(10);
//    }
//  }
//  else {
//    while(1) {
//      LED_Reset;
//      Delay_100ms(1);
//      LED_Set;
//      Delay_100ms(19);
//    }
//  }
//}
/*====================================================================================================*/
/*====================================================================================================*/

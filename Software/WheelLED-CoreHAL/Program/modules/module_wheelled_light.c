/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_usart.h"

#include "module_wheelLED_light.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define UARTx                       USART2
#define UARTx_CLK_ENABLE()          __HAL_RCC_USART2_CLK_ENABLE()

#define UARTx_TX_PIN                GPIO_PIN_2
#define UARTx_TX_GPIO_PORT          GPIOA
#define UARTx_TX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define UARTx_TX_AF                 GPIO_AF7_USART2

#define UARTx_RX_PIN                GPIO_PIN_3
#define UARTx_RX_GPIO_PORT          GPIOA
#define UARTx_RX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define UARTx_RX_AF                 GPIO_AF7_USART2

#define UARTx_BAUDRATE              460800
#define UARTx_BYTESIZE              UART_WORDLENGTH_8B
#define UARTx_STOPBITS              UART_STOPBITS_1
#define UARTx_PARITY                UART_PARITY_NONE
#define UARTx_HARDWARECTRL          UART_HWCONTROL_NONE
#define UARTx_MODE                  UART_MODE_TX_RX
#define UARTx_OVERSAMPLE            UART_OVERSAMPLING_16
/*====================================================================================================*/
/*====================================================================================================*/
static UART_HandleTypeDef WheelLED_HandleStruct;
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_Config
**功能 : WheelLED Config
**輸入 : None
**輸出 : None
**使用 : WheelLED_Config();
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* UART Clk ******************************************************************/
  UARTx_TX_GPIO_CLK_ENABLE();
  UARTx_RX_GPIO_CLK_ENABLE();
  UARTx_CLK_ENABLE();

  /* UART Pin ******************************************************************/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin       = UARTx_TX_PIN;
  GPIO_InitStruct.Alternate = UARTx_TX_AF;
  HAL_GPIO_Init(UARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = UARTx_RX_PIN;
  GPIO_InitStruct.Alternate = UARTx_RX_AF;
  HAL_GPIO_Init(UARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  /* UART Init *****************************************************************/
  WheelLED_HandleStruct.Instance          = UARTx;
  WheelLED_HandleStruct.Init.BaudRate     = UARTx_BAUDRATE;
  WheelLED_HandleStruct.Init.WordLength   = UARTx_BYTESIZE;
  WheelLED_HandleStruct.Init.StopBits     = UARTx_STOPBITS;
  WheelLED_HandleStruct.Init.Parity       = UARTx_PARITY;
  WheelLED_HandleStruct.Init.HwFlowCtl    = UARTx_HARDWARECTRL;
  WheelLED_HandleStruct.Init.Mode         = UARTx_MODE;
  HAL_UART_Init(&WheelLED_HandleStruct);

  /* UART Enable ***************************************************************/
  __HAL_UART_ENABLE(&WheelLED_HandleStruct);
  __HAL_UART_CLEAR_FLAG(&WheelLED_HandleStruct, UART_FLAG_TC);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_sendData
**功能 : Send Data
**輸入 : *sendData
**輸出 : None
**使用 : WheelLED_sendData(sendData);
**====================================================================================================*/
/*====================================================================================================*/
static void WheelLED_sendData( uint8_t *sendData )
{
  UART_SendData(UARTx, sendData, 2);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : 
**功能 : 
**輸入 : None
**輸出 : None
**使用 : 
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_setLED_byAddr( uint8_t addr, uint8_t ch, uint8_t sendData )
{
  uint8_t tmpData[2] = {0};

  tmpData[0] = addr | ch | ((sendData&0x01) << 1) | 0x01;
  tmpData[1] = sendData & 0xFE;
  WheelLED_sendData(tmpData);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : 
**功能 : 
**輸入 : None
**輸出 : None
**使用 : 
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_setLED_byNum( uint8_t num, uint8_t sendData )
{
  uint8_t addr = 0;
  uint8_t ch   = 0;

  num  = num % WLED_TOTAL_LEDS;
  addr = (num / 8) << 5;
  ch   = (num % 8) << 2;

  WheelLED_setLED_byAddr(addr, ch, sendData);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : 
**功能 : 
**輸入 : None
**輸出 : None
**使用 : 
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_setLED( uint16_t ang, uint16_t angScatter, uint8_t light )
{
  static uint8_t tPoint = 0;
  static uint8_t tWidth = 0;
  static uint8_t tLight = 0;

  uint8_t point = (uint8_t)(ang * WLED_LED_PER_DEGREE);
  uint8_t width = (uint8_t)(angScatter * WLED_LED_PER_DEGREE);

  if((point != tPoint) || (width != tWidth)) {
    for(uint8_t i = 0; i <= tWidth; i++)
      WheelLED_setLED_byNum(tPoint + i, WLED_LIGHT_MIN);
    for(uint8_t i = 0; i <= width; i++)
      WheelLED_setLED_byNum(point + i, light);
    tPoint = point;
    tWidth = width;
    tLight = light;
  }
  else if(tLight != light) {
    for(uint8_t i = 0; i <= width; i++)
      WheelLED_setLED_byNum(point + i, light);
    tLight = light;
  }
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : 
**功能 : 
**輸入 : None
**輸出 : None
**使用 : 
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_setLED_all( uint8_t sendData )
{
  for(uint8_t i = 0; i < WLED_TOTAL_LEDS; i++)
    WheelLED_setLED_byNum(i, sendData);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : 
**功能 : 
**輸入 : None
**輸出 : None
**使用 : 
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_CMD( uint8_t sendCMD, uint8_t sendData )
{
  uint8_t sendTmp[2] = {0};

  sendTmp[0] = WLED_CMD_SET | sendCMD | 0x01;
  sendTmp[1] = sendData & WLED_ADDRESS_MASK;
  WheelLED_sendData(sendTmp);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : 
**功能 : 
**輸入 : None
**輸出 : None
**使用 : 
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_CMD_SetID( void )
{
  WheelLED_CMD(WLED_CMD_SET_MODE, 0);
  delay_ms(10);
  WheelLED_CMD(WLED_CMD_SET_ID, WLED_ADDR1);
  delay_ms(10);
  WheelLED_CMD(WLED_CMD_RUN_MODE, 0);
  delay_ms(10);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : 
**功能 : 
**輸入 : None
**輸出 : None
**使用 : 
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_CMD_Restore( void )
{
  WheelLED_CMD(WLED_CMD_SET_MODE, 0);
  delay_ms(10);
  WheelLED_CMD(WLED_CMD_RESTORE, 0);
  delay_ms(10);
  WheelLED_CMD(WLED_CMD_RUN_MODE, 0);
  delay_ms(10);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : 
**功能 : 
**輸入 : None
**輸出 : None
**使用 : 
**====================================================================================================*/
/*====================================================================================================*/
#define LIGHT_ON    32
#define LIGHT_OFF   0
#define DEMO_DELAY  5
void WheelLED_testFunc( void )
{
  WheelLED_setLED_all(LIGHT_OFF);
  delay_ms(100);

  for(uint8_t i = 0; i < WLED_TOTAL_LEDS; i++) {
    WheelLED_setLED_byNum(i, LIGHT_ON);
    delay_ms(DEMO_DELAY);
    WheelLED_setLED_byNum(i, LIGHT_OFF);
  }

  for(uint8_t i = WLED_TOTAL_LEDS; i > 0; i--) {
    WheelLED_setLED_byNum(i-1, LIGHT_ON);
    delay_ms(DEMO_DELAY);
    WheelLED_setLED_byNum(i-1, LIGHT_OFF);
  }

  WheelLED_setLED_all(LIGHT_ON);
  delay_ms(DEMO_DELAY << 4);
  WheelLED_setLED_all(LIGHT_OFF);
  delay_ms(DEMO_DELAY << 4);
  WheelLED_setLED_all(LIGHT_ON);
  delay_ms(DEMO_DELAY << 4);
  WheelLED_setLED_all(LIGHT_OFF);
  delay_ms(DEMO_DELAY << 4);
}
void WheelLED_testFunc_1( void )
{
  for(uint16_t i = 0; i < 360; i++) {
    WheelLED_setLED(i, 0, 32);
    delay_ms(40);
  }
}
/*====================================================================================================*/
/*====================================================================================================*/

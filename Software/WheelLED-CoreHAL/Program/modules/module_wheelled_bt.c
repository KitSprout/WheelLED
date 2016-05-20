/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_usart.h"

#include "module_wheelLED_bt.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define UARTx                       USART1
#define UARTx_CLK_ENABLE()          __HAL_RCC_USART1_CLK_ENABLE()
#define UARTx_IRQn                  USART1_IRQn

#define UARTx_TX_PIN                GPIO_PIN_6
#define UARTx_TX_GPIO_PORT          GPIOB
#define UARTx_TX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define UARTx_TX_AF                 GPIO_AF7_USART1

#define UARTx_RX_PIN                GPIO_PIN_3
#define UARTx_RX_GPIO_PORT          GPIOB
#define UARTx_RX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define UARTx_RX_AF                 GPIO_AF7_USART1

#define UARTx_BAUDRATE              115200
#define UARTx_BYTESIZE              UART_WORDLENGTH_8B
#define UARTx_STOPBITS              UART_STOPBITS_1
#define UARTx_PARITY                UART_PARITY_NONE
#define UARTx_HARDWARECTRL          UART_HWCONTROL_NONE
#define UARTx_MODE                  UART_MODE_TX_RX
#define UARTx_OVERSAMPLE            UART_OVERSAMPLING_16
/*====================================================================================================*/
/*====================================================================================================*/
UART_HandleTypeDef UART_HandleStruct;

__IO uint8_t  bluetoothCMD = BLUETOOTH_CMD_KEYOFF;
__IO uint16_t bluetoothANG = 0;
__IO uint16_t bluetoothLIG = 0;
__IO uint16_t bluetoothWID = 0;
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_BT_Config
**功能 : WheelLED Bluetooth Config
**輸入 : None
**輸出 : None
**使用 : WheelLED_BT_Config();
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_BT_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* UART Clk ******************************************************************/
  UARTx_TX_GPIO_CLK_ENABLE();
  UARTx_RX_GPIO_CLK_ENABLE();
  UARTx_CLK_ENABLE();

  /* UART Pin ******************************************************************/
  GPIO_InitStruct.Mode      = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin       = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin       = UARTx_TX_PIN;
  GPIO_InitStruct.Alternate = UARTx_TX_AF;
  HAL_GPIO_Init(UARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = UARTx_RX_PIN;
  GPIO_InitStruct.Alternate = UARTx_RX_AF;
  HAL_GPIO_Init(UARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  /* UART IT *******************************************************************/
  HAL_NVIC_SetPriority(UARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(UARTx_IRQn);

  /* UART Init *****************************************************************/
  UART_HandleStruct.Instance          = UARTx;
  UART_HandleStruct.Init.BaudRate     = UARTx_BAUDRATE;
  UART_HandleStruct.Init.WordLength   = UARTx_BYTESIZE;
  UART_HandleStruct.Init.StopBits     = UARTx_STOPBITS;
  UART_HandleStruct.Init.Parity       = UARTx_PARITY;
  UART_HandleStruct.Init.HwFlowCtl    = UARTx_HARDWARECTRL;
  UART_HandleStruct.Init.Mode         = UARTx_MODE;
  UART_HandleStruct.Init.OverSampling = UARTx_OVERSAMPLE;
  HAL_UART_Init(&UART_HandleStruct);

  /* UART Enable ***************************************************************/
  __HAL_UART_ENABLE_IT(&UART_HandleStruct, UART_IT_RXNE);
  __HAL_UART_ENABLE(&UART_HandleStruct);
  __HAL_UART_CLEAR_FLAG(&UART_HandleStruct, UART_FLAG_TC);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_BT_SendByte
**功能 : Send Byte
**輸入 : sendByte
**輸出 : None
**使用 : WheelLED_BT_SendByte('A');
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_BT_SendByte( uint8_t sendByte )
{
  UART_SendByte(UARTx, &sendByte);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_BT_SendData
**功能 : Send Byte
**輸入 : cmd, sendData
**輸出 : None
**使用 : WheelLED_BT_SendData(cmd, sendData);
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_BT_SendData( uint8_t cmd, uint16_t sendData )
{
  WheelLED_BT_SendByte('S');
  WheelLED_BT_SendByte(cmd);
  WheelLED_BT_SendByte(Byte16U8H(sendData));
  WheelLED_BT_SendByte(Byte16U8L(sendData));
  WheelLED_BT_SendByte('\r');
  WheelLED_BT_SendByte('\n');
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_BT_RecvData
**功能 : Send Byte
**輸入 : sendByte
**輸出 : None
**使用 : WheelLED_BT_RecvData('A');
**====================================================================================================*/
/*====================================================================================================*/
uint8_t WheelLED_BT_RecvData( void )
{
  uint8_t recvByte = 0;
  UART_RecvByte(UARTx, &recvByte);
  return recvByte;
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Linkit_SendByte
**功能 : Send Byte
**輸入 : sendByte
**輸出 : None
**使用 : Linkit_SendByte('A');
**====================================================================================================*/
/*====================================================================================================*/
extern __IO uint8_t cmdState;
extern __IO uint8_t lockState;

void WheelLED_BT_evenCallBack( void )
{
  static __IO uint8_t buf[6] = {0};

  buf[0] = buf[1];
  buf[1] = buf[2];
  buf[2] = buf[3];
  buf[3] = buf[4];
  buf[4] = buf[5];
  buf[5] = WheelLED_BT_RecvData();

  if((buf[0] == 'S') && (buf[4] == '\r') && (buf[5] == '\n')) {
    bluetoothCMD = buf[1];
    switch(bluetoothCMD) {
      case BLUETOOTH_CMD_KEYOFF: break;
      case BLUETOOTH_CMD_KEYON:  break;
      case BLUETOOTH_CMD_LOCK:   break;
      case BLUETOOTH_CMD_UNLOCK: break;
      case BLUETOOTH_CMD_ANGLE:  bluetoothANG = Byte16(uint16_t, buf[2], buf[3]);   break;
      case BLUETOOTH_CMD_LIGHT:  bluetoothLIG = Byte16(uint16_t, buf[2], buf[3]);   break;
      case BLUETOOTH_CMD_WIDTH:  bluetoothWID = Byte16(uint16_t, buf[2], buf[3]);   break;
      default:                   bluetoothCMD = BLUETOOTH_CMD_KEYOFF;
    }
  }
}
/*====================================================================================================*/
/*====================================================================================================*/
int fputc( int ch, FILE *f )
{
  UARTx->DR = ((uint8_t)ch & (uint16_t)0x00FF);
  while(!(UARTx->SR & UART_FLAG_TXE));
  return (ch);
}
int fgetc( FILE *f )
{
  while(!(UARTx->SR & UART_FLAG_RXNE));
  return (uint16_t)(UARTx->DR & (uint16_t)0x01FF);
}
/*====================================================================================================*/
/*====================================================================================================*/

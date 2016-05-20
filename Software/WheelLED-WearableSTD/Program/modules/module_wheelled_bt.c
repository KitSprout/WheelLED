/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f0_system.h"
#include "drivers\stm32f0_usart.h"

#include "module_wheelLED_bt.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define UARTx               USART1
#define UARTx_CLK_ENABLE()  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define UARTx_IRQn          USART1_IRQn

#define UARTx_TX_PIN        GPIO_Pin_9
#define UARTx_TX_GPIO_PORT  GPIOA
#define UARTx_TX_AF         GPIO_AF_1
#define UARTx_TX_SOURCE     GPIO_PinSource9

#define UARTx_RX_PIN        GPIO_Pin_10
#define UARTx_RX_GPIO_PORT  GPIOA
#define UARTx_RX_AF         GPIO_AF_1
#define UARTx_RX_SOURCE     GPIO_PinSource10

#define UARTx_BAUDRATE      115200
#define UARTx_BYTESIZE      USART_WordLength_8b
#define UARTx_STOPBITS      USART_StopBits_1
#define UARTx_PARITY        USART_Parity_No
#define UARTx_HARDWARECTRL  USART_HardwareFlowControl_None
#define UARTx_MODE          USART_Mode_Rx | USART_Mode_Tx
/*====================================================================================================*/
/*====================================================================================================*/
__IO uint8_t  bluetoothCMD  = 0;
__IO uint16_t bluetoothDATA = 0;
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
  USART_InitTypeDef UART_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;

  /* UART Clk ******************************************************************/
  UARTx_CLK_ENABLE();

  /* UART AF *******************************************************************/
  GPIO_PinAFConfig(UARTx_TX_GPIO_PORT, UARTx_TX_SOURCE, UARTx_TX_AF);
  GPIO_PinAFConfig(UARTx_RX_GPIO_PORT, UARTx_RX_SOURCE, UARTx_RX_AF);

  /* UART Pin ******************************************************************/
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;

  GPIO_InitStruct.GPIO_Pin   = UARTx_TX_PIN;
  GPIO_Init(UARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = UARTx_RX_PIN;
  GPIO_Init(UARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  /* UART IT *******************************************************************/
  NVIC_InitStruct.NVIC_IRQChannel         = UARTx_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  /* UART Init *****************************************************************/
  UART_InitStruct.USART_BaudRate            = UARTx_BAUDRATE;
  UART_InitStruct.USART_WordLength          = UARTx_BYTESIZE;
  UART_InitStruct.USART_StopBits            = UARTx_STOPBITS;
  UART_InitStruct.USART_Parity              = UARTx_PARITY;
  UART_InitStruct.USART_HardwareFlowControl = UARTx_HARDWARECTRL;
  UART_InitStruct.USART_Mode                = UARTx_MODE;
  USART_Init(UARTx, &UART_InitStruct);

  /* UART Enable ***************************************************************/
  USART_ITConfig(UARTx, USART_IT_RXNE, ENABLE);
  USART_Cmd(UARTx, ENABLE);
  USART_ClearFlag(UARTx, USART_FLAG_TC);
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
    bluetoothCMD  = buf[1];
    bluetoothDATA = Byte16(uint16_t, buf[2], buf[3]);
  }
}
/*====================================================================================================*/
/*====================================================================================================*/
int fputc( int ch, FILE *f )
{
  UARTx->TDR = ((uint8_t)ch & (uint16_t)0x01FF);
  while(!(UARTx->ISR & USART_FLAG_TC));
  return (ch);
}
int fgetc( FILE *f )
{
//  while(!(UARTx->ISR & USART_FLAG_RXNE));
  return (uint16_t)(UARTx->RDR & (uint16_t)0x01FF);
}
/*====================================================================================================*/
/*====================================================================================================*/

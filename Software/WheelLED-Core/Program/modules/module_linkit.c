/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f1_system.h"
#include "drivers\stm32f1_usart.h"

#include "module_linkit.h"
#include "experiment_stm32f1.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define UARTx                       USART3
#define UARTx_CLK_ENABLE()          __HAL_RCC_USART3_CLK_ENABLE()
#define UARTx_IRQn                  USART3_IRQn

#define UARTx_TX_PIN                GPIO_PIN_10
#define UARTx_TX_GPIO_PORT          GPIOB
#define UARTx_TX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define UARTx_RX_PIN                GPIO_PIN_11
#define UARTx_RX_GPIO_PORT          GPIOB
#define UARTx_RX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define UARTx_BAUDRATE              115200
#define UARTx_BYTESIZE              UART_WORDLENGTH_8B
#define UARTx_STOPBITS              UART_STOPBITS_1
#define UARTx_PARITY                UART_PARITY_NONE
#define UARTx_HARDWARECTRL          UART_HWCONTROL_NONE
#define UARTx_MODE                  UART_MODE_TX_RX
/*====================================================================================================*/
/*====================================================================================================*/
UART_HandleTypeDef UART_HandleStruct_Linkit;

__IO uint8_t  LinkitCMD = 0;
__IO uint16_t LinkitANG = 0;
__IO uint16_t LinkitLIG = 0;
__IO uint16_t LinkitWID = 0;
/*====================================================================================================*/
/*====================================================================================================*
**函數 : RS232_Config
**功能 : RS232 Config
**輸入 : None
**輸出 : None
**使用 : RS232_Config();
**====================================================================================================*/
/*====================================================================================================*/
void Linkit_SerialConfig( void )
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
  HAL_GPIO_Init(UARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Mode      = GPIO_MODE_AF_INPUT;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin       = UARTx_RX_PIN;
  HAL_GPIO_Init(UARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  /* UART IT *******************************************************************/
  HAL_NVIC_SetPriority(UARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(UARTx_IRQn);

  /* UART Init *****************************************************************/
  UART_HandleStruct_Linkit.Instance          = UARTx;
  UART_HandleStruct_Linkit.Init.BaudRate     = UARTx_BAUDRATE;
  UART_HandleStruct_Linkit.Init.WordLength   = UARTx_BYTESIZE;
  UART_HandleStruct_Linkit.Init.StopBits     = UARTx_STOPBITS;
  UART_HandleStruct_Linkit.Init.Parity       = UARTx_PARITY;
  UART_HandleStruct_Linkit.Init.HwFlowCtl    = UARTx_HARDWARECTRL;
  UART_HandleStruct_Linkit.Init.Mode         = UARTx_MODE;
  HAL_UART_Init(&UART_HandleStruct_Linkit);

  /* UART Enable ***************************************************************/
  __HAL_UART_ENABLE_IT(&UART_HandleStruct_Linkit, UART_IT_RXNE);
  __HAL_UART_ENABLE(&UART_HandleStruct_Linkit);
  __HAL_UART_CLEAR_FLAG(&UART_HandleStruct_Linkit, UART_FLAG_TC);
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
void Linkit_SendByte( uint8_t sendByte )
{
  UART_SendByte(&UART_HandleStruct_Linkit, &sendByte);
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
void Linkit_SendData( uint8_t cmd, uint16_t sendData )
{
  Linkit_SendByte('S');
  Linkit_SendByte(cmd);
  Linkit_SendByte(Byte16U8H(sendData));
  Linkit_SendByte(Byte16U8L(sendData));
  Linkit_SendByte('\r');
  Linkit_SendByte('\n');
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
uint8_t Linkit_RecvData( void )
{
  uint8_t recvByte = 0;
  UART_RecvByte(&UART_HandleStruct_Linkit, &recvByte);
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

void Linkit_Interrupt_CallBack( void )
{
  static __IO uint8_t buf[6] = {0};

  buf[0] = buf[1];
  buf[1] = buf[2];
  buf[2] = buf[3];
  buf[3] = buf[4];
  buf[4] = buf[5];
  buf[5] = Linkit_RecvData();

  if((buf[0] == 'S') && (buf[4] == '\r') && (buf[5] == '\n')) {
    LinkitCMD = buf[1];
    switch(LinkitCMD) {
      case LINKIT_CMD_KEYOFF: break;
      case LINKIT_CMD_KEYON:  break;
      case LINKIT_CMD_LOCK:   break;
      case LINKIT_CMD_UNLOCK: break;
      case LINKIT_CMD_ANGLE:  LinkitANG = Byte16(uint16_t, buf[2], buf[3]);   break;
      case LINKIT_CMD_LIGHT:  LinkitLIG = Byte16(uint16_t, buf[2], buf[3]);   break;
      case LINKIT_CMD_WIDTH:  LinkitWID = Byte16(uint16_t, buf[2], buf[3]);   break;
      default:                LinkitCMD = LINKIT_CMD_KEYOFF;
    }
  }
}
/*====================================================================================================*/
/*====================================================================================================*/

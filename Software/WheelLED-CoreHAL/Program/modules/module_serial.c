/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f4_system.h"
#include "drivers\stm32f4_usart.h"
#include "algorithms\algorithm_string.h"

#include "module_serial.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define UARTx                       USART1
#define UARTx_CLK_ENABLE()          __HAL_RCC_USART1_CLK_ENABLE()

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
static UART_HandleTypeDef Serial_HandleStruct;
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_Config
**功能 : Serial Config
**輸入 : None
**輸出 : None
**使用 : Serial_Config();
**====================================================================================================*/
/*====================================================================================================*/
void Serial_Config( void )
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

  /* UART Init *****************************************************************/
  Serial_HandleStruct.Instance          = UARTx;
  Serial_HandleStruct.Init.BaudRate     = UARTx_BAUDRATE;
  Serial_HandleStruct.Init.WordLength   = UARTx_BYTESIZE;
  Serial_HandleStruct.Init.StopBits     = UARTx_STOPBITS;
  Serial_HandleStruct.Init.Parity       = UARTx_PARITY;
  Serial_HandleStruct.Init.HwFlowCtl    = UARTx_HARDWARECTRL;
  Serial_HandleStruct.Init.Mode         = UARTx_MODE;
  Serial_HandleStruct.Init.OverSampling = UARTx_OVERSAMPLE;
  HAL_UART_Init(&Serial_HandleStruct);

  /* UART Enable ***************************************************************/
  __HAL_UART_ENABLE(&Serial_HandleStruct);
  __HAL_UART_CLEAR_FLAG(&Serial_HandleStruct, UART_FLAG_TC);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_SendByte
**功能 : Send Byte
**輸入 : SendByte
**輸出 : None
**使用 : Serial_SendByte('A');
**====================================================================================================*/
/*====================================================================================================*/
void Serial_SendByte( uint8_t sendByte )
{
  UART_SendByte(UARTx, &sendByte);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_SendData
**功能 : Send Data
**輸入 : *sendData, lens
**輸出 : None
**使用 : Serial_SendData(sendData, lens);
**====================================================================================================*/
/*====================================================================================================*/
void Serial_SendData( uint8_t *sendData, uint16_t lens )
{
  UART_SendData(UARTx, sendData, lens);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_SendStr
**功能 : Send String
**輸入 : *pWord
**輸出 : None
**使用 : Serial_SendStr("Hellow World!");
**====================================================================================================*/
/*====================================================================================================*/
void Serial_SendStr( char *pWord )
{
  while(*pWord != '\0') {
    UART_SendByte(UARTx, (uint8_t*)pWord++);
  }
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_SendNum
**功能 : Send Number
**輸入 : type, lens, sendNum
**輸出 : None
**使用 : Serial_SendNum(Type_D, 6, 1024);
**====================================================================================================*/
/*====================================================================================================*/
void Serial_SendNum( StringType type, uint8_t lens, int32_t sendNum )
{
  char tmpStr[32] = {0};
  char *pWord = tmpStr;

  num2Str(type, lens, tmpStr, sendNum);

  while(*pWord != '\0') {
    UART_SendByte(UARTx, (uint8_t*)pWord++);
  }
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_RecvByte
**功能 : Recv Byte
**輸入 : *recvByte
**輸出 : None
**使用 : recvByte = Serial_RecvByte();
**====================================================================================================*/
/*====================================================================================================*/
uint8_t Serial_RecvByte( void )
{
  uint8_t recvByte = 0;
  UART_RecvByte(UARTx, &recvByte);
  return recvByte;
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_RecvData
**功能 : Recv Data
**輸入 : *recvData, lens
**輸出 : None
**使用 : Serial_RecvData(recvData, lens);
**====================================================================================================*/
/*====================================================================================================*/
void Serial_RecvData( uint8_t *recvData, uint16_t lens )
{
  UART_RecvData(UARTx, recvData, lens);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_RecvDataWTO
**功能 : Recv Data Wait Timeout
**輸入 : *recvData, lens, timeoutMs
**輸出 : state
**使用 : state = Serial_RecvDataWTO(recvData, lens, timeoutMs);
**====================================================================================================*/
/*====================================================================================================*/
int8_t Serial_RecvDataWTO( uint8_t *recvData, uint16_t lens, int32_t timeoutMs )
{
  return UART_RecvDataWTO(UARTx, recvData, lens, timeoutMs);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_RecvStr
**功能 : Recv String
**輸入 : *pWord
**輸出 : None
**使用 : Serial_RecvStr(recvStr);
**====================================================================================================*/
/*====================================================================================================*/
void Serial_RecvStr( char *pWord )
{
  do {
    UART_RecvByte(UARTx, (uint8_t*)pWord++);
  } while(*(pWord-1) != '\0');
  *pWord = '\0';
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_RecvStrWTO
**功能 : Recv String Wait Timeout
**輸入 : *pWord, timeoutMs
**輸出 : State
**使用 : Serial_RecvStrWTO(RecvStr, 200);
**====================================================================================================*/
/*====================================================================================================*/
int8_t Serial_RecvStrWTO( char *pWord, int32_t timeoutMs )
{
  int8_t state = ERROR;

  do {
    state = UART_RecvByteWTO(UARTx, (uint8_t*)pWord++, timeoutMs);
    if(state == ERROR)
      return ERROR;
  } while(*(pWord-1) != '\0');
  *pWord = '\0';

  return SUCCESS;
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : Serial_SendDataMATLAB
**功能 : Send Data to MATLAB 
**輸入 : *sendData, lens
**輸出 : None
**使用 : Serial_SendDataMATLAB(sendData, 10); // int16 * 10 data
**====================================================================================================*/
/*====================================================================================================*/
void Serial_SendDataMATLAB( int16_t *sendData, uint8_t lens )
{
  uint8_t tmpData[32] = {0};  // tmpData lens >= 2 * lens + 4
  uint8_t *ptrData = tmpData;
  uint8_t dataBytes = lens << 1;
  uint8_t dataLens = dataBytes + 4;
  uint8_t count = 0;
  uint16_t tmpSum = 0;

  tmpData[0] = 'S';
  while(count < dataBytes) {
    tmpData[count + 1] = Byte8H(sendData[count >> 1]);
    tmpData[count + 2] = Byte8L(sendData[count >> 1]);
    count = count + 2;
  }
  for(uint8_t i = 0; i < dataBytes; i++)
    tmpSum += tmpData[i+1];
  tmpData[dataLens - 3] = (uint8_t)(tmpSum & 0x00FF);
  tmpData[dataLens - 2] = '\r';
  tmpData[dataLens - 1] = '\n';

  do {
    Serial_SendByte(*ptrData++);
  } while(--dataLens);
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

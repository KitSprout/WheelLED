/*====================================================================================================*/
/*====================================================================================================*/
#include "Dirvers\stm32f0_system.h"

#include "wheelLED.h"
#include "wheelLED_param.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define UARTx                       USART1
#define UARTx_CLK_ENABLE()          __HAL_RCC_USART1_CLK_ENABLE()
#define UARTx_IRQn                  USART1_IRQn

#define UARTx_TX_PIN                GPIO_PIN_9
#define UARTx_TX_GPIO_PORT          GPIOA
#define UARTx_TX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define UARTx_TX_AF                 GPIO_AF1_USART1

#define UARTx_RX_PIN                GPIO_PIN_10
#define UARTx_RX_GPIO_PORT          GPIOA
#define UARTx_RX_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define UARTx_RX_AF                 GPIO_AF1_USART1

#define UARTx_BAUDRATE              460800
#define UARTx_BYTESIZE              UART_WORDLENGTH_8B
#define UARTx_STOPBITS              UART_STOPBITS_1
#define UARTx_PARITY                UART_PARITY_NONE
#define UARTx_HARDWARECTRL          UART_HWCONTROL_NONE
#define UARTx_MODE                  UART_MODE_TX_RX
#define UARTx_OVERSAMPLE            UART_OVERSAMPLING_16
/*====================================================================================================*/
/*====================================================================================================*/
static UART_HandleTypeDef WLED_UART_HandleStruct;

__IO uint8_t  WheelLED_Mode     = WLED_MODE_RUN;
__IO uint8_t  WheelLED_address  = WLED_ADDR1;
__IO uint32_t WheelLED_baudrate = UARTx_BAUDRATE;

__IO uint8_t WLED_Flag_addressUpdate = RESET;
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_LED_Config
**功能 : WheelLED LED Config
**輸入 : None
**輸出 : None
**使用 : WheelLED_LED_Config();
**====================================================================================================*/
/*====================================================================================================*/
static void WheelLED_LED_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Clk ******************************************************************/
  LED_GPIO_CLK_ENABLE();

  /* GPIO Pin ******************************************************************/
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin   = LED_PIN;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

  /* GPIO Pin ******************************************************************/
  LED_Set;
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_PWM_Config
**功能 : WheelLED PWM Config
**輸入 : None
**輸出 : None
**使用 : WheelLED_PWM_Config();
**====================================================================================================*/
/*====================================================================================================*/
static void WheelLED_PWM_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_HandleTypeDef TIM_HandleStruct;
  TIM_OC_InitTypeDef TIM_OC_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM14_CLK_ENABLE();

  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
  GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  GPIO_InitStruct.Pin       = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Alternate = GPIO_AF4_TIM14;
  GPIO_InitStruct.Pin       = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM2 use APB1 = 48 MHz */
  TIM_HandleStruct.Instance               = TIM2;
  TIM_HandleStruct.Init.Prescaler         = 0;
  TIM_HandleStruct.Init.Period            = WLED_PWM_MAX - 1;
  TIM_HandleStruct.Init.ClockDivision     = 0;
  TIM_HandleStruct.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TIM_HandleStruct.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&TIM_HandleStruct);

  TIM_OC_InitStruct.OCMode       = TIM_OCMODE_PWM2;
  TIM_OC_InitStruct.OCFastMode   = TIM_OCFAST_DISABLE;
  TIM_OC_InitStruct.OCPolarity   = TIM_OCPOLARITY_LOW;
  TIM_OC_InitStruct.OCNPolarity  = TIM_OCPOLARITY_LOW;
  TIM_OC_InitStruct.OCIdleState  = TIM_OCIDLESTATE_SET;
  TIM_OC_InitStruct.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  TIM_OC_InitStruct.Pulse = WLED_PWM_MIN;
  HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_4);

  /* TIM3 */
  TIM_HandleStruct.Instance = TIM3;
  HAL_TIM_PWM_Init(&TIM_HandleStruct);

  HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_4);

  /* TIM14 */
  TIM_HandleStruct.Instance = TIM14;
  HAL_TIM_PWM_Init(&TIM_HandleStruct);

  HAL_TIM_PWM_ConfigChannel(&TIM_HandleStruct, &TIM_OC_InitStruct, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&TIM_HandleStruct, TIM_CHANNEL_1);
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_UART_Config
**功能 : WheelLED UART Config
**輸入 : None
**輸出 : None
**使用 : WheelLED_UART_Config();
**====================================================================================================*/
/*====================================================================================================*/
static void WheelLED_UART_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* UART Clk ******************************************************************/
  UARTx_TX_GPIO_CLK_ENABLE();
  UARTx_RX_GPIO_CLK_ENABLE();
  UARTx_CLK_ENABLE();

  /* UART IT *******************************************************************/
  HAL_NVIC_SetPriority(UARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(UARTx_IRQn);

  /* UART Pin ******************************************************************/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin       = UARTx_TX_PIN;
  GPIO_InitStruct.Alternate = UARTx_TX_AF;
  HAL_GPIO_Init(UARTx_TX_GPIO_PORT, &GPIO_InitStruct);

//  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
//  GPIO_InitStruct.Pull      = GPIO_PULLUP;

  GPIO_InitStruct.Pin       = UARTx_RX_PIN;
  GPIO_InitStruct.Alternate = UARTx_RX_AF;
  HAL_GPIO_Init(UARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  /* UART IT *******************************************************************/
  HAL_NVIC_SetPriority(UARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(UARTx_IRQn);

  /* UART Init *****************************************************************/
  WLED_UART_HandleStruct.Instance          = UARTx;
  WLED_UART_HandleStruct.Init.BaudRate     = WheelLED_baudrate;
  WLED_UART_HandleStruct.Init.WordLength   = UARTx_BYTESIZE;
  WLED_UART_HandleStruct.Init.StopBits     = UARTx_STOPBITS;
  WLED_UART_HandleStruct.Init.Parity       = UARTx_PARITY;
  WLED_UART_HandleStruct.Init.HwFlowCtl    = UARTx_HARDWARECTRL;
  WLED_UART_HandleStruct.Init.Mode         = UARTx_MODE;
  WLED_UART_HandleStruct.Init.OverSampling = UARTx_OVERSAMPLE;
  HAL_UART_Init(&WLED_UART_HandleStruct);

  /* UART Enable ***************************************************************/
  __HAL_UART_ENABLE_IT(&WLED_UART_HandleStruct, UART_IT_RXNE);
  __HAL_UART_ENABLE(&WLED_UART_HandleStruct);
  __HAL_UART_CLEAR_FLAG(&WLED_UART_HandleStruct, UART_FLAG_TC);
}
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
  WheelLED_LED_Config();
  WheelLED_PWM_Config();

  WLED_PWM_CH1 = WLED_PWM_MIN;
  WLED_PWM_CH2 = WLED_PWM_MIN;
  WLED_PWM_CH3 = WLED_PWM_MIN;
  WLED_PWM_CH4 = WLED_PWM_MIN;
  WLED_PWM_CH5 = WLED_PWM_MIN;
  WLED_PWM_CH6 = WLED_PWM_MIN;
  WLED_PWM_CH7 = WLED_PWM_MIN;
  WLED_PWM_CH8 = WLED_PWM_MIN;

  WheelLED_UART_Config();
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_Config
**功能 : WheelLED Config
**輸入 : None
**輸出 : None
**使用 : WheelLED_Config();
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_Init( void )
{
//  WheelLED_setParams();
  WheelLED_getParams();

  WheelLED_Config();
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_RecvData
**功能 : WheelLED Recv Data
**輸入 : recvByte
**輸出 : None
**使用 : WheelLED_RecvData();
**====================================================================================================*/
/*====================================================================================================*/
#define FIFO_SIZE 2
void WheelLED_RecvData( uint8_t *recvByte )
{
  static __IO uint8_t recvFIFO[FIFO_SIZE] = {0};

  __IO uint8_t setData = 0;

  recvFIFO[1] = recvFIFO[0];
  recvFIFO[0] = *recvByte;

  if(((recvFIFO[1] & 0x01) == 0x01) && ((recvFIFO[0] & 0x01) == 0x00)) {  // check package
    if((recvFIFO[1] & WLED_ADDRESS_MASK) == WheelLED_address) {           // check address
      setData = recvFIFO[0] | ((recvFIFO[1] & 0x02) >> 1);
      switch(recvFIFO[1] & WLED_CHANNEL_MASK) {
        case WLED_CH1:  { WLED_PWM_CH1 = setData;  break; }
        case WLED_CH2:  { WLED_PWM_CH2 = setData;  break; }
        case WLED_CH3:  { WLED_PWM_CH3 = setData;  break; }
        case WLED_CH4:  { WLED_PWM_CH4 = setData;  break; }
        case WLED_CH5:  { WLED_PWM_CH5 = setData;  break; }
        case WLED_CH6:  { WLED_PWM_CH6 = setData;  break; }
        case WLED_CH7:  { WLED_PWM_CH7 = setData;  break; }
        case WLED_CH8:  { WLED_PWM_CH8 = setData;  break; }
      }
    }
    else if((recvFIFO[1] & WLED_ADDRESS_MASK) == WLED_CMD_SET) {          // check Mode
      switch(recvFIFO[1] & WLED_CHANNEL_MASK) {
        case WLED_CMD_IAP_START: {
          WheelLED_Mode = WLED_MODE_IAP;
          break;
        }
        case WLED_CMD_SET_START: {
          WheelLED_Mode = WLED_MODE_SET;
          break;
        }
        case WLED_CMD_SET_END: {
          if(WLED_Flag_addressUpdate == SET) {
            WheelLED_saveParams();
            WLED_Flag_addressUpdate = RESET;
          }
          WheelLED_Mode = WLED_MODE_RUN;
          break;
        }
        case WLED_CMD_SET_ID: {
          WheelLED_address = recvFIFO[0] & WLED_ADDRESS_MASK;
          *recvByte = (WheelLED_address == WLED_ADDR7) ? WLED_ADDR7 : recvFIFO[0] + WLED_ADDRESS_STEP;
          WLED_Flag_addressUpdate = SET;
          break;
        }
        case WLED_CMD_SET_BOUDRATE: {
          break;
        }
        case WLED_CMD_RESTORE:  {
          WheelLED_setParams();
          break;
        }
        case WLED_CMD_REC2:  { break; }
        case WLED_CMD_REC3:  { break; }
      }
    }
  }
}
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelLED_IRQHandler
**功能 : WheelLED IRQ
**輸入 : None
**輸出 : None
**使用 : WheelLED_IRQHandler();
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_IRQHandler( void )
{
  uint8_t recvByte = 0;

  if((UARTx->ISR & UART_FLAG_TXE) != RESET) {
    switch(WheelLED_Mode) {
      case WLED_MODE_RUN: {
        recvByte = (uint8_t)(UARTx->RDR & (uint16_t)0x00FF);
        UARTx->TDR = (recvByte & (uint16_t)0x00FF);
        while((UARTx->ISR & UART_FLAG_TXE) == RESET);
        WheelLED_RecvData(&recvByte);
        break;
      }
      case WLED_MODE_SET: {
        recvByte = (uint8_t)(UARTx->RDR & (uint16_t)0x00FF);
        WheelLED_RecvData(&recvByte);
        UARTx->TDR = (recvByte & (uint16_t)0x00FF);
        while((UARTx->ISR & UART_FLAG_TXE) == RESET);
        break;
      }
      case WLED_MODE_IAP: {
        recvByte = (uint8_t)(UARTx->RDR & (uint16_t)0x00FF);
        UARTx->TDR = (recvByte & (uint16_t)0x00FF);
        while((UARTx->ISR & UART_FLAG_TXE) == RESET);
        break;
      }
      default: {
        
      }
    }
  }
  UARTx->ICR = UART_IT_RXNE;
}
/*====================================================================================================*/
/*====================================================================================================*/

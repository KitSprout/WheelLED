/*====================================================================================================*/
/*====================================================================================================*/
#include "stm32f0_system.h"
#include "wheelLED.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define USARTx                USART1
#define USARTx_CLK            RCC_APB2Periph_USART1
#define USARTx_IRQ            USART1_IRQn

#define USARTx_TX_PIN         GPIO_Pin_9
#define USARTx_TX_GPIO_PORT   GPIOA
#define USARTx_TX_GPIO_CLK    RCC_AHBPeriph_GPIOA
#define USARTx_TX_SOURCE      GPIO_PinSource9
#define USARTx_TX_AF          GPIO_AF_1

#define USARTx_RX_PIN         GPIO_Pin_10
#define USARTx_RX_GPIO_PORT   GPIOA
#define USARTx_RX_GPIO_CLK    RCC_AHBPeriph_GPIOA
#define USARTx_RX_SOURCE      GPIO_PinSource10
#define USARTx_RX_AF          GPIO_AF_1

#define USARTx_BAUDRATE       921600
#define USARTx_BYTESIZE       USART_WordLength_8b
#define USARTx_STOPBITS       USART_StopBits_1
#define USARTx_PARITY         USART_Parity_No
#define USARTx_HARDWARECTRL   USART_HardwareFlowControl_None
/*====================================================================================================*/
/*====================================================================================================*/
static void WheelLED_PWM_Config( void );
static void WheelLED_UART_Config( void );

__IO uint8_t WheelLED_Mode      = WLED_MODE_RUN;
__IO uint8_t WheelLED_DeviceID  = WLED_ADDR7;
/*====================================================================================================*/
/*====================================================================================================*
**函數 : WheelWheelLED_Config
**功能 : WheelLED Config
**輸入 : None
**輸出 : None
**使用 : WheelWheelLED_Config();
**====================================================================================================*/
/*====================================================================================================*/
void WheelLED_Config( void )
{
  WheelLED_PWM_Config();

  WheelLED_PWM_CH1 = WheelLED_PWM_MIN;
  WheelLED_PWM_CH2 = WheelLED_PWM_MIN;
  WheelLED_PWM_CH3 = WheelLED_PWM_MIN;
  WheelLED_PWM_CH4 = WheelLED_PWM_MIN;
  WheelLED_PWM_CH5 = WheelLED_PWM_MIN;
  WheelLED_PWM_CH6 = WheelLED_PWM_MIN;
  WheelLED_PWM_CH7 = WheelLED_PWM_MIN;
  WheelLED_PWM_CH8 = WheelLED_PWM_MIN;

  WheelLED_UART_Config();
}
/*=====================================================================================================*/
/*=====================================================================================================*/
static void WheelLED_UART_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  USART_InitTypeDef USART_InitStruct;

  /* UART Clk Init *************************************************************/
  RCC_APB2PeriphClockCmd(USARTx_CLK, ENABLE);
  RCC_AHBPeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

  /* UART NVIC Config **********************************************************/
  NVIC_InitStruct.NVIC_IRQChannel = USARTx_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  /* USARTx Tx PA9 */
  GPIO_InitStruct.GPIO_Pin = USARTx_TX_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
  /* USARTx Rx PA10 */
  GPIO_InitStruct.GPIO_Pin = USARTx_RX_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  /* UART Init *****************************************************************/
  USART_InitStruct.USART_BaudRate = USARTx_BAUDRATE;
  USART_InitStruct.USART_WordLength = USARTx_BYTESIZE;
  USART_InitStruct.USART_StopBits = USARTx_STOPBITS;
  USART_InitStruct.USART_Parity = USARTx_PARITY;
  USART_InitStruct.USART_HardwareFlowControl = USARTx_HARDWARECTRL;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStruct);

  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
  USART_Cmd(USARTx, ENABLE);
  USART_ClearFlag(USARTx, USART_FLAG_TC);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
static void WheelLED_PWM_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
  TIM_OCInitTypeDef TIM_OCInitStruct;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM14, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_2);  // TIM2  CH1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2);  // TIM2  CH2
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_2);  // TIM2  CH3
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_2);  // TIM2  CH4
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_4);  // TIM14 CH1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);  // TIM3  CH1
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);  // TIM3  CH2
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);  // TIM3  CH4

  /* TIM GPIO Init */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3  |
                             GPIO_Pin_4  | GPIO_Pin_6  | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  /************************** PWM Output **************************************/
  /* TIM Time Base */
  TIM_TimeBaseStruct.TIM_Period = (uint16_t)(WheelLED_PWM_MAX-1);
  TIM_TimeBaseStruct.TIM_Prescaler = (uint16_t)(0);
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;    // Count Up
  TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2,  &TIM_TimeBaseStruct);
  TIM_TimeBaseInit(TIM3,  &TIM_TimeBaseStruct);
  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStruct);

  /* PWM Mode */
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStruct.TIM_Pulse = WheelLED_PWM_MIN;
  TIM_OC1Init(TIM2,  &TIM_OCInitStruct);
  TIM_OC2Init(TIM2,  &TIM_OCInitStruct);
  TIM_OC3Init(TIM2,  &TIM_OCInitStruct);
  TIM_OC4Init(TIM2,  &TIM_OCInitStruct);
  TIM_OC1Init(TIM3,  &TIM_OCInitStruct);
  TIM_OC2Init(TIM3,  &TIM_OCInitStruct);
  TIM_OC4Init(TIM3,  &TIM_OCInitStruct);
  TIM_OC1Init(TIM14, &TIM_OCInitStruct);
  TIM_OC1PreloadConfig(TIM2,  TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM2,  TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM2,  TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM2,  TIM_OCPreload_Enable);
  TIM_OC1PreloadConfig(TIM3,  TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM3,  TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM3,  TIM_OCPreload_Enable);
  TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM2,  ENABLE);
  TIM_ARRPreloadConfig(TIM3,  ENABLE);
  TIM_ARRPreloadConfig(TIM14, ENABLE);

  /* Enable */
  TIM_Cmd(TIM2,  ENABLE);
  TIM_Cmd(TIM3,  ENABLE);
  TIM_Cmd(TIM14, ENABLE);
  TIM_CtrlPWMOutputs(TIM2,  ENABLE);
  TIM_CtrlPWMOutputs(TIM3,  ENABLE);
  TIM_CtrlPWMOutputs(TIM14, ENABLE);
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
#define FIFO_SIZE 2
void WheelLED_RecvData( uint8_t *recvByte )
{
  static __IO uint8_t recvFIFO[FIFO_SIZE] = {0};

  __IO uint8_t setData = 0;

  recvFIFO[1] = recvFIFO[0];
  recvFIFO[0] = *recvByte;

  if(((recvFIFO[1] & 0x01) == 0x01) && ((recvFIFO[0] & 0x01) == 0x00)) {  // check package
    if((recvFIFO[1] & WLED_ADDRESS_MASK) == WheelLED_DeviceID) {          // check DeviceID
      setData = recvFIFO[0] | ((recvFIFO[1] & 0x02) >> 1);
      switch(recvFIFO[1] & WLED_CHANNEL_MASK) {
        case WLED_CH1:  { WheelLED_PWM_CH1 = setData;  break; }
        case WLED_CH2:  { WheelLED_PWM_CH2 = setData;  break; }
        case WLED_CH3:  { WheelLED_PWM_CH3 = setData;  break; }
        case WLED_CH4:  { WheelLED_PWM_CH4 = setData;  break; }
        case WLED_CH5:  { WheelLED_PWM_CH5 = setData;  break; }
        case WLED_CH6:  { WheelLED_PWM_CH6 = setData;  break; }
        case WLED_CH7:  { WheelLED_PWM_CH7 = setData;  break; }
        case WLED_CH8:  { WheelLED_PWM_CH8 = setData;  break; }
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
          WheelLED_Mode = WLED_MODE_RUN;
          break;
        }
        case WLED_CMD_SET_ID: {
          WheelLED_DeviceID = recvFIFO[0] & WLED_ADDRESS_MASK;
          *recvByte = (WheelLED_DeviceID == WLED_ADDR7) ? WLED_ADDR7 : recvFIFO[0] + WLED_ADDRESS_STEP;
          break;
        }
        case WLED_CMD_SET_BOUDRATE: {
          break;
        }
        case WLED_CMD_REC1:  { break; }
        case WLED_CMD_REC2:  { break; }
        case WLED_CMD_REC3:  { break; }
      }
    }
  }
}

/*====================================================================================================*/
/*====================================================================================================*/

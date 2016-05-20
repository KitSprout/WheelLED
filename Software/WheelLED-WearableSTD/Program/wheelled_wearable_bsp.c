/*=====================================================================================================*/
/*=====================================================================================================*/
#include "drivers\stm32f0_system.h"
#include "modules\module_mpu6500.h"
#include "modules\module_wheelLED_bt.h"

#include "wheelled_wearable_bsp.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
void WLED_GPIO_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Clk Init *************************************************************/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3  | GPIO_Pin_4  |
                               GPIO_Pin_5  | GPIO_Pin_6  | GPIO_Pin_7  | GPIO_Pin_9  | GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_InitStruct.GPIO_Pin   = KEY_PIN;
  GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStruct);
}
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_MOT_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;

  GPIO_InitStruct.GPIO_Pin   = MOT_PIN;
  GPIO_Init(MOT_GPIO_PORT, &GPIO_InitStruct);

  MOT_Set();
}
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_BT_Config( void )
{
  WheelLED_BT_Config();
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void WLED_IMU_Config( void )
{
  MPU_ConfigTypeDef MPU_ConfigStruct;
  uint8_t state = ERROR;

  MPU6500_Config();
  delay_ms(10);

  printf("Init ... ");
  MPU_ConfigStruct.MPU_Gyr_FullScale     = MPU_GyrFS_2000dps;
  MPU_ConfigStruct.MPU_Gyr_LowPassFilter = MPU_GyrLPS_41Hz;
  MPU_ConfigStruct.MPU_Acc_FullScale     = MPU_AccFS_4g;
  MPU_ConfigStruct.MPU_Acc_LowPassFilter = MPU_AccLPS_41Hz;
  state = MPU6500_Init(&MPU_ConfigStruct);
  if(state != SUCCESS) {
    printf("ERROR\r\n");
    while(1);
  }
  printf("SUCCESS\r\n");
  delay_ms(100);
}
/*=====================================================================================================*/
/*=====================================================================================================*/
void WLED_TIM_Config( void )
{
  NVIC_InitTypeDef NVIC_InitStruct;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

  TIMx_CLK_ENABLE();

  /* TIM IT *******************************************************************/
  NVIC_InitStruct.NVIC_IRQChannel         = TIMx_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelCmd      = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  TIM_TimeBaseStruct.TIM_Period        = TIMx_PERIOD;
  TIM_TimeBaseStruct.TIM_Prescaler     = TIMx_PRES;
  TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStruct);

  TIM_ClearFlag(TIMx, TIM_FLAG_Update);
  TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIMx, ENABLE);
}
/*=====================================================================================================*/
/*=====================================================================================================*/

/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f4_system.h"
#include "modules\module_mpu6500.h"
#include "modules\module_wheelLED_light.h"
#include "modules\module_wheelLED_bt.h"

#include "wheelled_core_bsp.h"
/*====================================================================================================*/
/*====================================================================================================*/
TIM_HandleTypeDef WLED_TIM_HandleStruct;
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_IMU_Config( void )
{
  uint8_t state = ERROR;

  MPU_InitTypeDef MPU_InitStruct;

  MPU_InitStruct.MPU_Gyr_FullScale     = MPU_GyrFS_2000dps;
  MPU_InitStruct.MPU_Gyr_LowPassFilter = MPU_GyrLPS_41Hz;
  MPU_InitStruct.MPU_Acc_FullScale     = MPU_AccFS_4g;
  MPU_InitStruct.MPU_Acc_LowPassFilter = MPU_AccLPS_41Hz;
  state = MPU6500_Init(&MPU_InitStruct);
  while(state != SUCCESS) {
    WheelLED_setLED_all(32);
    delay_ms(50);
    WheelLED_setLED_all(0);
    delay_ms(50);
  }

  delay_ms(100);
}
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_TIM_Config( void )
{
  /* TIM Clk *******************************************************************/
  TIMx_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIMx_IRQ, 3, 0);
  HAL_NVIC_EnableIRQ(TIMx_IRQ);

  /* TIM TimeBase **************************************************************/
  WLED_TIM_HandleStruct.Instance               = TIMx;
  WLED_TIM_HandleStruct.Init.Prescaler         = TIMx_PRES - 1;
  WLED_TIM_HandleStruct.Init.Period            = TIMx_PERIOD - 1;
  WLED_TIM_HandleStruct.Init.ClockDivision     = 0;
  WLED_TIM_HandleStruct.Init.CounterMode       = TIM_COUNTERMODE_UP;
  WLED_TIM_HandleStruct.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&WLED_TIM_HandleStruct);
  HAL_TIM_Base_Start_IT(&WLED_TIM_HandleStruct);
}
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_WheelLED_Config( void )
{
  WheelLED_Config();

  delay_ms(100);
  WheelLED_testFunc();

  WheelLED_BT_Config();

//  delay_ms(100);
//  WheelLED_setLED_all(0);
}
/*====================================================================================================*/
/*====================================================================================================*/

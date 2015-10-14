/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f1_system.h"
#include "modules\module_rs232.h"
#include "modules\module_linkit.h"
#include "modules\module_mpu9250.h"
#include "modules\module_wheelLED.h"
#include "algorithms\algorithm_moveAve.h"
#include "algorithms\algorithm_mathUnit.h"

#include "experiment_stm32f1.h"
/*====================================================================================================*/
/*====================================================================================================*/
__IO int16_t AccelX;
__IO int16_t AccelY;
__IO int16_t AccelZ;
__IO int32_t dAccelZ;

__IO int16_t GyroZ;
__IO int32_t dGyroZ;

__IO int16_t gyroX_offset;
__IO int16_t gyroY_offset;
__IO int16_t gyroZ_offset;

__IO int16_t  theta = 0;
__IO uint16_t angle = 330;
__IO uint16_t width = 165;
__IO uint16_t light = 32;

extern __IO uint8_t  LinkitCMD;
extern __IO uint16_t LinkitANG;
extern __IO uint16_t LinkitLIG;
extern __IO uint16_t LinkitWID;

#define Freq 200

TIM_HandleTypeDef IMU_HandleStruct;
/*====================================================================================================*/
/*====================================================================================================*/
void Serial_sendDataMATLAB( int16_t *sendData, uint8_t dataNum )
{
  uint8_t tmpData[16] = {0};
  uint8_t *ptrData = tmpData;
  uint8_t dataBytes = dataNum << 1;
  uint8_t dataLens = dataBytes + 4;
  uint8_t count = 0;
  uint16_t tmpSum = 0;

  tmpData[0] = 'S';
  while(count < dataBytes) {
    tmpData[count+1] = Byte8H(sendData[count >> 1]);
    tmpData[count+2] = Byte8L(sendData[count >> 1]);
    count = count + 2;
  }
  for(uint8_t i = 0; i < dataBytes; i++)
    tmpSum += tmpData[i+1];
  tmpData[dataLens - 3] = (uint8_t)(tmpSum & 0x00FF);
  tmpData[dataLens - 2] = '\r';
  tmpData[dataLens - 1] = '\n';

  do {
    RS232_SendByte(*ptrData++);
  } while(--dataLens);
}
/*====================================================================================================*/
/*====================================================================================================*/
void GPIO_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Clk Init *************************************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* LED_B PC13 */  /* LED_G PC14 */  /* LED_R PC15 */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* KEY_WU PA0 */  /* KEY_BO PB2 */
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  
  GPIO_InitStruct.Pin   = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Init
  LED_R_Set;
  LED_G_Set;
  LED_B_Set;
}
/*====================================================================================================*/
/*====================================================================================================*/
void IMU_InterruptConfig( void )
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  /* TIM3 72MHz */
  IMU_HandleStruct.Instance               = TIM3;
  IMU_HandleStruct.Init.Prescaler         = 72 - 1;
  IMU_HandleStruct.Init.Period            = 1000000 / Freq - 1;
  IMU_HandleStruct.Init.ClockDivision     = 0;
  IMU_HandleStruct.Init.CounterMode       = TIM_COUNTERMODE_UP;
  IMU_HandleStruct.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&IMU_HandleStruct);
  HAL_TIM_Base_Start_IT(&IMU_HandleStruct);
}

void System_Init( void )
{
  uint8_t status = ERROR;
  int16_t tmpRead[12] = {0};
  int32_t tmpSumData[3] = {0};

  MPU_InitTypeDef MPU_InitStruct;

  HAL_Init();
  GPIO_Config();
  Linkit_SerialConfig();
  WheelLED_Config();
  MPU9250_Config();
  Delay_100ms(5);

  MPU_InitStruct.MPU_Gyr_FullScale     = MPU_GyrFS_2000dps;
  MPU_InitStruct.MPU_Gyr_LowPassFilter = MPU_GyrLPS_41Hz;
  MPU_InitStruct.MPU_Acc_FullScale     = MPU_AccFS_4g;
  MPU_InitStruct.MPU_Acc_LowPassFilter = MPU_AccLPS_42Hz;
  MPU_InitStruct.MPU_Mag_FullScale     = MPU_MagFS_16b;
  status = MPU9250_Init(&MPU_InitStruct);
  while(status != SUCCESS) {
    LED_G_Toggle;
    Delay_100ms(1);
  }
  Delay_100ms(1);

  WheelLED_testFunc();

  for(uint16_t i = 0; i < 256; i++) {
    MPU9250_getData(tmpRead);
    tmpSumData[0] += tmpRead[4];   // Gyr.X
    tmpSumData[1] += tmpRead[5];   // Gyr.Y
    tmpSumData[2] += tmpRead[6];   // Gyr.Z    
  }
  gyroX_offset = tmpSumData[0] >> 8;
  gyroY_offset = tmpSumData[1] >> 8;
  gyroZ_offset = tmpSumData[2] >> 8;

  IMU_InterruptConfig();

  LED_R_Set;
  LED_G_Set;
  LED_B_Set;
}

int main( void )
{
  static uint8_t modeSel = 1;

  System_Init();

  while(1) {
//    LED_B_Toggle;
//    Delay_100ms(1);

    if(KEY_WU_Read)
      WheelLED_testFunc();
    if(KEY_BO_Read) {
/*      WheelLED_CMD_SetID();*/
      switch(modeSel) {
        case 0:
          LED_R_Set;
          LED_G_Reset;
          angle = 340;
          width = 165;
          light = 32;
          modeSel = 1;
          break;
        case 1:
          LED_R_Reset;
          LED_G_Set;
          angle = 225;
          width = 165;
          light = 64;
          modeSel = 0;
          break;
      }
      Delay_100ms(5);
    }

    if(modeSel)
      WheelLED_FrontFSM();
    else
      WheelLED_RearFSM();
  }
}
/*====================================================================================================*/
/*====================================================================================================*/
#define FSM_RUN   0
#define FSM_STOP  1
#define FSM_BRAKE 2
#define FSM_CMD   3
#define FSM_LOCK  4

#define AccelZ_threshold  360
#define dAccelZ_threshold 3800

#define GyroZ_threshold   1000
#define dGyroZ_threshold -4000

void WheelLED_Light( uint16_t _angle, uint16_t _width, uint16_t _light )
{
  theta = (int16_t)(toDeg(atan2f(-AccelX, -AccelY)) + 180.0f);
  theta = (theta + _angle) % 360;
  WheelLED_setLED(theta, _width, _light);
}

void WheelLED_FrontFSM( void )
{
  static uint8_t state = FSM_STOP;

  switch(state) {
    case FSM_RUN:
      WheelLED_Light(angle, width, light);
      if(LinkitCMD != LINKIT_CMD_KEYOFF)
        state = FSM_CMD;
      else if(GyroZ < GyroZ_threshold)
        state = FSM_STOP;
      break;

    case FSM_STOP:
      WheelLED_Light(330, 60, 1);
      if(LinkitCMD != LINKIT_CMD_KEYOFF)
        state = FSM_CMD;
      else if(GyroZ > GyroZ_threshold)
        state = FSM_RUN;
      break;

    case FSM_CMD:
      switch(LinkitCMD) {

        case LINKIT_CMD_KEYOFF:
          state = FSM_STOP;
          break;

        case LINKIT_CMD_KEYON:
          WheelLED_Light(angle, width, light);
          break;

        case LINKIT_CMD_ANGLE:
          while(LinkitCMD == LINKIT_CMD_ANGLE) {
            angle = LinkitANG;
            WheelLED_Light(angle, width, light);
          }
          state = FSM_STOP;
          break;

        case LINKIT_CMD_LIGHT: 
          while(LinkitCMD == LINKIT_CMD_LIGHT) {
            light = LinkitLIG;
            if(light > 255)
              light = WLED_LIGHT_MAX;
            WheelLED_Light(angle, width, light);
          }
          state = FSM_STOP;
          break;

        case LINKIT_CMD_WIDTH:
          while(LinkitCMD == LINKIT_CMD_WIDTH) {
            width = LinkitWID;
            WheelLED_Light(angle, width, light);
          }
          state = FSM_STOP;
          break;

        case LINKIT_CMD_LOCK:
          state = FSM_LOCK;
          break;

      }
      break;

    case FSM_LOCK:
      WheelLED_setLED_all(0);
      Delay_100ms(1);
      WheelLED_setLED_all(32);
      Delay_100ms(1);
      WheelLED_setLED_all(0);
      Delay_100ms(1);
      WheelLED_setLED_all(32);
      Delay_100ms(1);
      WheelLED_setLED_all(0);
      Delay_100ms(10);
      WheelLED_setLED(0, 0, 0);
      while(LinkitCMD != LINKIT_CMD_UNLOCK) {
        if((AccelZ > AccelZ_threshold)   || (AccelZ < -AccelZ_threshold) ||
           (dAccelZ > dAccelZ_threshold) || (dAccelZ < -dAccelZ_threshold)) {
          WheelLED_setLED_all(32);
          Delay_100ms(1);
          WheelLED_setLED_all(0);
          Delay_100ms(1);
        }
      }
      WheelLED_testFunc();
      state = FSM_STOP;
      break;
  }
}

void WheelLED_RearFSM( void )
{
  static uint8_t state = FSM_STOP;
  static uint32_t BCount = 0;
  static uint32_t RCount = 0;

  switch(state) {
    case FSM_RUN:
      WheelLED_Light(angle, width, 4);
      if(dGyroZ < (dGyroZ_threshold - 3000)) {
        BCount++;
        if(BCount > 8) {
          RCount = 0;
          BCount = 0;
          state = FSM_BRAKE;
        }
      }
      else if(GyroZ < GyroZ_threshold) {
        state = FSM_STOP;
      }
      else {
        RCount++;
        if(RCount > 128) {
          RCount = 0;
          BCount = 0;
        }
      }
      break;

    case FSM_BRAKE:
      WheelLED_Light(angle, width, light);
      if((dGyroZ > -1000) && (GyroZ > GyroZ_threshold)) {
        state = FSM_RUN;
        for(uint16_t i = 0; i < 8192; i++)
          WheelLED_Light(angle, width, light);
      }
      else if(GyroZ < GyroZ_threshold) {
        state = FSM_STOP;
        for(uint16_t i = 0; i < 8192; i++)
          WheelLED_Light(angle, width, light);
      }
      break;

    case FSM_STOP:
      WheelLED_Light(330, 60, 1);
      if(GyroZ > GyroZ_threshold)
        state = FSM_RUN;
      break;
  }
}

#define Sxx 0.122283792196740f
#define Syy 0.121850074073742f
#define Szz 0.120978603690181f
#define Bx  -177.552500000000f
#define By  -159.498000000001f
#define Bz  -291.033000000000f

#define dt 0.005f  // 200Hz 5ms

static int16_t FIFO_ACC[3][16] = {0};
static int16_t FIFO_GYR[3][16] = {0};

void IMU_Interrupt_CallBack( TIM_HandleTypeDef *htim )
{
  static int16_t AccelZ_old = 0;
  static int16_t GyroZ_old = 0;

  float tmpGyrZ = 0.0f;

  int16_t readData[12] = {0};
  int16_t IMU_Buf[12] = {0};

  MPU9250_getData(readData);

  IMU_Buf[0] = (int16_t)((readData[1] - Bx) * Sxx);   // Acc.X
  IMU_Buf[1] = (int16_t)((readData[2] - By) * Syy);   // Acc.Y
  IMU_Buf[2] = (int16_t)((readData[3] - Bz) * Szz);   // Acc.Z
  IMU_Buf[3] = readData[4] - gyroX_offset;    // Gyr.X
  IMU_Buf[4] = readData[5] - gyroY_offset;    // Gyr.Y
  IMU_Buf[5] = readData[6] - gyroZ_offset;    // Gyr.Z

  IMU_Buf[0] = MoveAve_WMA(IMU_Buf[0], FIFO_ACC[0], 4);
  IMU_Buf[1] = MoveAve_WMA(IMU_Buf[1], FIFO_ACC[1], 4);
  IMU_Buf[2] = MoveAve_WMA(IMU_Buf[2], FIFO_ACC[2], 4);
  IMU_Buf[3] = MoveAve_WMA(IMU_Buf[3], FIFO_GYR[0], 8);
  IMU_Buf[4] = MoveAve_WMA(IMU_Buf[4], FIFO_GYR[1], 8);
  IMU_Buf[5] = MoveAve_WMA(IMU_Buf[5], FIFO_GYR[2], 8);

  tmpGyrZ = toRad(IMU_Buf[5]);
  AccelX  = IMU_Buf[0];
  AccelY  = IMU_Buf[1] - (int16_t)(0.007f * tmpGyrZ * (50.0f + tmpGyrZ));
  AccelZ  = IMU_Buf[2];
  GyroZ   = IMU_Buf[5];

  dAccelZ = (int32_t)((AccelZ - AccelZ_old) / dt);
  dGyroZ  = (int32_t)((GyroZ - GyroZ_old) / dt);

  AccelZ_old = AccelZ;
  GyroZ_old  = GyroZ;

//  Serial_sendDataMATLAB(IMU_Buf, 6);
}
/*====================================================================================================*/
/*====================================================================================================*/

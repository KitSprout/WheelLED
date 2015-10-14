/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f1_system.h"
#include "modules\module_rs232.h"
#include "modules\module_mpu6500.h"
#include "algorithms\algorithm_moveAve.h"
#include "algorithms\algorithm_mathUnit.h"

#include "experiment_stm32f1.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define abs_num(num) (((num) < 0) ? (-1 * num) : (num))

__IO int16_t AccX = 0;
__IO int16_t AccY = 0;
__IO int16_t AccZ = 0;
__IO int16_t GyrX = 0;
__IO int16_t GyrY = 0;
__IO int16_t GyrZ = 0;

__IO uint16_t AccX_abs = 0;
__IO uint16_t AccY_abs = 0;
__IO uint16_t AccZ_abs = 0;
  
__IO int16_t gyroX_offset;
__IO int16_t gyroY_offset;
__IO int16_t gyroZ_offset;

#define LINKIT_CMD_KEYOFF 0
#define LINKIT_CMD_KEYON  1
#define LINKIT_CMD_LOCK   2
#define LINKIT_CMD_UNLOCK 3
#define LINKIT_CMD_ANGLE  4
#define LINKIT_CMD_LIGHT  5
#define LINKIT_CMD_WIDTH  6
/*====================================================================================================*/
/*====================================================================================================*/
#define LED_PIN                   GPIO_PIN_10
#define LED_GPIO_PORT             GPIOA
#define LED_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()

#define KEY_PIN                   GPIO_PIN_9
#define KEY_GPIO_PORT             GPIOA
#define KEY_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define KEY_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()

#define MOT_Set     __GPIO_SET(LED_GPIO_PORT, LED_PIN)
#define MOT_Reset   __GPIO_RST(LED_GPIO_PORT, LED_PIN)
#define LED_Toggle  __GPIO_TOG(LED_GPIO_PORT, LED_PIN)

#define KEY_Read   (__GPIO_READ(KEY_GPIO_PORT, KEY_PIN) != KEY_PIN)

void WLED_wearable( void );
/*====================================================================================================*/
/*====================================================================================================*/
void GPIO_EX_Config( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin   = LED_PIN;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin   = KEY_PIN;
  HAL_GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStruct);

  MOT_Set;
}
/*====================================================================================================*/
/*====================================================================================================*/
void Linkit_SendData( uint8_t cmd, uint16_t sendData )
{
  RS232_SendByte('S');
  RS232_SendByte(cmd);
  RS232_SendByte(Byte16U8H(sendData));
  RS232_SendByte(Byte16U8L(sendData));
  RS232_SendByte('\r');
  RS232_SendByte('\n');
}
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

#define Freq 200

TIM_HandleTypeDef IMU_HandleStruct;
TIM_HandleTypeDef TIMR_HandleStruct;

void IMU_InterruptConfig( void )
{
  __HAL_RCC_TIM2_CLK_ENABLE();
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

  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  /* TIM2 72MHz */
  TIMR_HandleStruct.Instance               = TIM2;
  TIMR_HandleStruct.Init.Prescaler         = 36000 - 1;  // 2kHz
  TIMR_HandleStruct.Init.Period            = 2000 - 1;
  TIMR_HandleStruct.Init.ClockDivision     = 0;
  TIMR_HandleStruct.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TIMR_HandleStruct.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&TIMR_HandleStruct);
  HAL_TIM_Base_Start_IT(&TIMR_HandleStruct);
}
/*====================================================================================================*/
/*====================================================================================================*/
void System_Init( void )
{
  int16_t tmpRead[12] = {0};
  int32_t tmpSumData[3] = {0};

  MPU_InitTypeDef MPU_InitStruct;

  HAL_Init();
  GPIO_Config();
  GPIO_EX_Config();
  RS232_Config();
  MPU6500_Config();

  Delay_100ms(1);

  MPU_InitStruct.MPU_Gyr_FullScale     = MPU_GyrFS_2000dps;
  MPU_InitStruct.MPU_Gyr_LowPassFilter = MPU_GyrLPS_41Hz;
  MPU_InitStruct.MPU_Acc_FullScale     = MPU_AccFS_4g;
  MPU_InitStruct.MPU_Acc_LowPassFilter = MPU_AccLPS_42Hz;
  while(MPU6500_Init(&MPU_InitStruct) != SUCCESS) {
    LED_R_Toggle;
    Delay_100ms(1);
  }
  Delay_100ms(1);

  for(uint16_t i = 0; i < 256; i++) {
    MPU6500_getData(tmpRead);
    tmpSumData[0] += tmpRead[4];   // Gyr.X
    tmpSumData[1] += tmpRead[5];   // Gyr.Y
    tmpSumData[2] += tmpRead[6];   // Gyr.Z    
  }
  gyroX_offset = tmpSumData[0] >> 8;
  gyroY_offset = tmpSumData[1] >> 8;
  gyroZ_offset = tmpSumData[2] >> 8;

  IMU_InterruptConfig();
}

void MOTOR_S( void )
{
  MOT_Reset;
  Delay_10ms(16);
  MOT_Set;
  Delay_10ms(10);
}
void MOTOR_SS( void )
{
  MOT_Reset;
  Delay_10ms(16);
  MOT_Set;
  Delay_10ms(10);
  MOT_Reset;
  Delay_10ms(16);
  MOT_Set;
  Delay_10ms(10);
}
void MOTOR_LSS( void )
{
  MOT_Reset;
  Delay_10ms(16);
  MOT_Set;
  Delay_10ms(40);
  MOT_Reset;
  Delay_10ms(16);
  MOT_Set;
  Delay_10ms(10);
  MOT_Reset;
  Delay_10ms(16);
  MOT_Set;
  Delay_10ms(40);
}
void MOTOR_L( void )
{
  MOT_Reset;
  Delay_10ms(100);
  MOT_Set;
  Delay_10ms(100);
}
void MOTOR_LL( void )
{
  MOT_Reset;
  Delay_10ms(100);
  MOT_Set;
  Delay_10ms(100);
  MOT_Reset;
  Delay_10ms(100);
  MOT_Set;
  Delay_10ms(100);
}

int main( void )
{
  System_Init();

  while(1) {
    LED_G_Toggle;
    Delay_100ms(1);

    if(KEY_WU_Read) {
      Linkit_SendData(LINKIT_CMD_LOCK, 0);
      Delay_100ms(10);
      Linkit_SendData(LINKIT_CMD_KEYOFF, 0);
    }
    if(KEY_BO_Read) {
      Linkit_SendData(LINKIT_CMD_UNLOCK, 0);
      Delay_100ms(10);
      Linkit_SendData(LINKIT_CMD_KEYOFF, 0);
    }

    if(KEY_Read)
      WLED_wearable();

    TimeRemind();
    // dis-connect warning
    // too tilt warning
    // too far (rssi) warning
  }
}
/*====================================================================================================*/
/*====================================================================================================*/
__IO uint8_t sec = 0;
__IO uint8_t min = 0;
__IO uint8_t hur = 0;

void TimeRemind_Interrupt_CallBack( TIM_HandleTypeDef *htim )
{
  sec++;
  if(sec == 60) {
    LED_B_Toggle;
    sec = 0;
    min++;
    if(min == 60) {
      min = 0;
      hur++;
    }
  }
}

void TimeRemind( void )
{
  if(sec == 50) {
    while(!KEY_Read)
      MOTOR_L();
    MOTOR_SS();
    Delay_100ms(10);
  }
}
/*====================================================================================================*/
/*====================================================================================================*/
#define Sxx 0.122283792196740f
#define Syy 0.121250074073742f
#define Szz 0.120978603690181f
#define Bx  -177.552500000000f
#define By  -369.498000000001f
#define Bz  -291.033000000000f

static int16_t FIFO_ACC[3][32] = {0};
static int16_t FIFO_GYR[3][32] = {0};

void IMU_Interrupt_CallBack( TIM_HandleTypeDef *htim )
{
  int16_t readData[10] = {0};

  MPU6500_getData(readData);

  readData[0] = (int16_t)((readData[1] - Bx) * Sxx);  // Acc.X
  readData[1] = (int16_t)((readData[2] - By) * Syy);  // Acc.Y
  readData[2] = (int16_t)((readData[3] - Bz) * Szz);  // Acc.Z
  readData[3] = readData[4] - gyroX_offset;   // Gyr.X
  readData[4] = readData[5] - gyroY_offset;   // Gyr.Y
  readData[5] = readData[6] - gyroZ_offset;   // Gyr.Z

  AccX = MoveAve_WMA(readData[0], FIFO_ACC[0], 32);
  AccY = MoveAve_WMA(readData[1], FIFO_ACC[1], 32);
  AccZ = MoveAve_WMA(readData[2], FIFO_ACC[2], 32);
  GyrX = MoveAve_WMA(readData[3], FIFO_GYR[0], 32);
  GyrY = MoveAve_WMA(readData[4], FIFO_GYR[1], 32);
  GyrZ = MoveAve_WMA(readData[5], FIFO_GYR[2], 32);

  AccX_abs = abs_num(AccX);
  AccY_abs = abs_num(AccY);
  AccZ_abs = abs_num(AccZ);

//  Serial_sendDataMATLAB(IMU_Buf, 6);
}

#define ACC_X_ZERO    300
#define ACC_Y_ZERO    300
#define ACC_Z_ZERO    300

#define ACC_X_45      500
#define ACC_Y_45      500
#define ACC_Z_45      500

#define ACC_X_GRAVITY 700
#define ACC_Y_GRAVITY 700
#define ACC_Z_GRAVITY 700

void WLED_wearable( void )
{
  int16_t theta = 0;
  uint16_t tmpData = 0;

  // LOCK
  if((AccX_abs < ACC_X_ZERO) && (AccY > ACC_Y_GRAVITY) && (AccZ_abs < ACC_Z_ZERO)) {
    MOTOR_S();
    while(KEY_Read) {
//      printf("LOCK ...\r\n");
      Delay_100ms(1);
      if((AccX_abs < ACC_X_ZERO) && (AccY_abs < ACC_Y_ZERO) && (AccZ > ACC_Z_GRAVITY)) {
        Linkit_SendData(LINKIT_CMD_LOCK, 0);
//        printf("Send Command LOCK\r\n");
        MOTOR_LSS();
        Linkit_SendData(LINKIT_CMD_KEYOFF, 0);
//        printf("Send Command KEY_OFF\r\n\r\n");
        Delay_100ms(20);
        break;
      }
    }
  }

  // UNLOCK
  else if((AccX_abs < ACC_X_ZERO) && (AccY < -ACC_Y_GRAVITY) && (AccZ_abs < ACC_Z_ZERO)) {
    MOTOR_S();
    while(KEY_Read) {
//      printf("UNLOCK ...\r\n");
      Delay_100ms(1);
      if((AccX_abs < ACC_X_ZERO) && (AccY_abs < ACC_Y_ZERO) && (AccZ > ACC_Z_GRAVITY)) {
        Linkit_SendData(LINKIT_CMD_UNLOCK, 0);
//        printf("Send Command UNLOCK\r\n");
        MOTOR_LSS();
        Linkit_SendData(LINKIT_CMD_KEYOFF, 0);
//        printf("Send Command KEY_OFF\r\n\r\n");
        Delay_100ms(20);
        break;
      }
    }
  }

  // ANGLE
  else if((AccX < -ACC_X_GRAVITY) && (AccY_abs < ACC_Y_ZERO) && (AccZ_abs > ACC_Z_ZERO)) {
    MOTOR_S();
    Linkit_SendData(LINKIT_CMD_KEYON, 0);
//    printf("Send Command KEY_ON\r\n");
    while(KEY_Read) {
      if(AccY_abs < ACC_Y_ZERO) {
        theta = (int16_t)((toDeg(atan2f(AccX, AccZ)) + 120.0f) * 3.0f);
        if(theta < 0)
          tmpData = 0;
        else if(theta > 360)
          tmpData = 360;
        else
          tmpData = theta;
        Linkit_SendData(LINKIT_CMD_ANGLE, tmpData);
      }
//      printf("Send Command ANGLE : %d\r\n", tmpData);
      Delay_10ms(5);
    }
    MOTOR_LSS();
    Linkit_SendData(LINKIT_CMD_KEYOFF, 0);
//    printf("Send Command KEY_OFF\r\n\r\n");
  }

  // LIGHT
  else if((AccX_abs < ACC_X_ZERO) && (AccY_abs < ACC_Y_ZERO) && (AccZ > ACC_Z_GRAVITY)) {
    MOTOR_S();
    Linkit_SendData(LINKIT_CMD_KEYON, 0);
//    printf("Send Command KEY_ON\r\n");
    while(KEY_Read) {
      if(AccX_abs < ACC_X_ZERO) {
        theta = (int16_t)((toDeg(atan2f(AccZ, AccY))) * 1.4f);
        if(theta < 0)
          tmpData = 0;
        else
          tmpData = theta;
        Linkit_SendData(LINKIT_CMD_LIGHT, tmpData);
      }
//      printf("Send Command LIGHT : %d\r\n", tmpData);
      Delay_10ms(5);
    }
    MOTOR_LSS();
    Linkit_SendData(LINKIT_CMD_KEYOFF, 0);
//    printf("Send Command KEY_OFF\r\n\r\n");
  }

  // WIDTH
  else if((AccX_abs < ACC_X_45) && (AccY_abs < ACC_Y_ZERO) && (AccZ > ACC_X_45)) {
    MOTOR_S();
    Linkit_SendData(LINKIT_CMD_KEYON, 0);
//    printf("Send Command KEY_ON\r\n");
    while(KEY_Read) {
      if(AccY_abs < ACC_Y_ZERO) {
        theta = (int16_t)((toDeg(atan2f(AccX, AccZ)) + 120.0f) * 3.0f);
        if(theta < 0)
          tmpData = 0;
        else if(theta > 360)
          tmpData = 360;
        else
          tmpData = theta;
        Linkit_SendData(LINKIT_CMD_WIDTH, tmpData);
      }
//      printf("Send Command WIDTH : %d\r\n", tmpData);
      Delay_10ms(5);
    }
    MOTOR_LSS();
    Linkit_SendData(LINKIT_CMD_KEYOFF, 0);
//    printf("Send Command KEY_OFF\r\n\r\n");
  }
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

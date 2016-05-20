/*=====================================================================================================*/
/*=====================================================================================================*/
#include "drivers\stm32f0_system.h"
#include "modules\module_mpu6500.h"
#include "modules\module_wheelLED_bt.h"
#include "algorithms\algorithm_mathUnit.h"
#include "algorithms\algorithm_moveAve.h"

#include "wheelled_wearable.h"
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

extern __IO uint8_t  bluetoothCMD;
extern __IO uint16_t bluetoothDATA;

void WLED_wearable( void );
/*====================================================================================================*/
/*====================================================================================================*/
void MOTOR_S( void )
{
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(100);
}
void MOTOR_SS( void )
{
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(100);
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(100);
}
void MOTOR_SSS( void )
{
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(100);
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(100);
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(100);
}
void MOTOR_LSS( void )
{
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(400);
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(100);
  MOT_Reset();  delay_ms(160);
  MOT_Set();    delay_ms(400);
}
void MOTOR_L( void )
{
  MOT_Reset();  delay_ms(1000);
  MOT_Set();    delay_ms(1000);
}
void MOTOR_LL( void )
{
  MOT_Reset();  delay_ms(1000);
  MOT_Set();    delay_ms(1000);
  MOT_Reset();  delay_ms(1000);
  MOT_Set();    delay_ms(1000);
}
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_Init( void )
{
  SystemInit();
  HAL_InitTick();

  WLED_GPIO_Config();
  WLED_MOT_Config();
  WLED_BT_Config();
  WLED_IMU_Config();
  WLED_TIM_Config();

  delay_ms(1000);
  MOTOR_SSS();
  delay_ms(1000);
}
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_Loop( void )
{
//  int16_t imu[8] = {0};

  while(1) {
    if(KEY_Read())
      WLED_wearable();
  }
//  while(1) {
//    printf("AX = %6d\tAY = %6d\tAZ = %6d\tGX = %6d\tGY = %6d\tGZ = %6d\r\n", AccX, AccY, AccZ, GyrX, GyrY, GyrZ);
//    delay_ms(100);
//    if(KEY_Read())
//      printf("\f");
//  }
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

void IMU_Interrupt_CallBack( void )
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
}
/*====================================================================================================*/
/*====================================================================================================*/
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

  /* LOCK */
  if((AccX_abs < ACC_X_ZERO) && (AccY > ACC_Y_GRAVITY) && (AccZ_abs < ACC_Z_ZERO)) {
    MOTOR_S();
    while(KEY_Read()) {
//printf("LOCK ...\r\n");
      delay_ms(100);
      if((AccX_abs < ACC_X_ZERO) && (AccY_abs < ACC_Y_ZERO) && (AccZ > ACC_Z_GRAVITY)) {
        WheelLED_BT_SendData(LINKIT_CMD_LOCK, 0);
//printf("Send Command LOCK\r\n");
        MOTOR_LSS();
        WheelLED_BT_SendData(LINKIT_CMD_KEYOFF, 0);
//printf("Send Command KEY_OFF\r\n\r\n");
        delay_ms(2000);
        break;
      }
    }
  }

  /* UNLOCK */
  else if((AccX_abs < ACC_X_ZERO) && (AccY < -ACC_Y_GRAVITY) && (AccZ_abs < ACC_Z_ZERO)) {
    MOTOR_S();
    while(KEY_Read()) {
//printf("UNLOCK ...\r\n");
      delay_ms(100);
      if((AccX_abs < ACC_X_ZERO) && (AccY_abs < ACC_Y_ZERO) && (AccZ > ACC_Z_GRAVITY)) {
        WheelLED_BT_SendData(LINKIT_CMD_UNLOCK, 0);
//printf("Send Command UNLOCK\r\n");
        MOTOR_LSS();
        WheelLED_BT_SendData(LINKIT_CMD_KEYOFF, 0);
//printf("Send Command KEY_OFF\r\n\r\n");
        delay_ms(2000);
        break;
      }
    }
  }

  /* ANGLE */
  else if((AccX < -ACC_X_GRAVITY) && (AccY_abs < ACC_Y_ZERO) && (AccZ_abs > ACC_Z_ZERO)) {
    MOTOR_S();
    WheelLED_BT_SendData(LINKIT_CMD_KEYON, 0);
//printf("Send Command KEY_ON\r\n");
    while(KEY_Read()) {
      if(AccY_abs < ACC_Y_ZERO) {
        theta = (int16_t)((toDeg(atan2f(AccX, AccZ)) + 120.0f) * 3.0f);
        if(theta < 0)
          tmpData = 0;
        else if(theta > 360)
          tmpData = 360;
        else
          tmpData = theta;
        WheelLED_BT_SendData(LINKIT_CMD_ANGLE, tmpData);
      }
//printf("Send Command ANGLE : %d\r\n", tmpData);
      delay_ms(50);
    }
    MOTOR_LSS();
    WheelLED_BT_SendData(LINKIT_CMD_KEYOFF, 0);
//printf("Send Command KEY_OFF\r\n\r\n");
  }

  /* LIGHT */
  else if((AccX_abs < ACC_X_ZERO) && (AccY_abs < ACC_Y_ZERO) && (AccZ > ACC_Z_GRAVITY)) {
    MOTOR_S();
    WheelLED_BT_SendData(LINKIT_CMD_KEYON, 0);
//printf("Send Command KEY_ON\r\n");
    while(KEY_Read()) {
      if(AccX_abs < ACC_X_ZERO) {
        theta = (int16_t)((toDeg(atan2f(AccZ, AccY))) * 1.4f);
        if(theta < 0)
          tmpData = 0;
        else
          tmpData = theta;
        WheelLED_BT_SendData(LINKIT_CMD_LIGHT, tmpData);
      }
//printf("Send Command LIGHT : %d\r\n", tmpData);
      delay_ms(50);
    }
    MOTOR_LSS();
    WheelLED_BT_SendData(LINKIT_CMD_KEYOFF, 0);
//printf("Send Command KEY_OFF\r\n\r\n");
  }

  /* WIDTH */
  else if((AccX_abs < ACC_X_45) && (AccY_abs < ACC_Y_ZERO) && (AccZ > ACC_X_45)) {
    MOTOR_S();
    WheelLED_BT_SendData(LINKIT_CMD_KEYON, 0);
//printf("Send Command KEY_ON\r\n");
    while(KEY_Read()) {
      if(AccY_abs < ACC_Y_ZERO) {
        theta = (int16_t)((toDeg(atan2f(AccX, AccZ)) + 120.0f) * 3.0f);
        if(theta < 0)
          tmpData = 0;
        else if(theta > 360)
          tmpData = 360;
        else
          tmpData = theta;
        WheelLED_BT_SendData(LINKIT_CMD_WIDTH, tmpData);
      }
//printf("Send Command WIDTH : %d\r\n", tmpData);
      delay_ms(50);
    }
    MOTOR_LSS();
    WheelLED_BT_SendData(LINKIT_CMD_KEYOFF, 0);
//printf("Send Command KEY_OFF\r\n\r\n");
  }
}
/*=====================================================================================================*/
/*=====================================================================================================*/

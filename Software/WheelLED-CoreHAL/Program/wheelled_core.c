/*====================================================================================================*/
/*====================================================================================================*/
#include "drivers\stm32f4_system.h"
#include "modules\module_mpu6500.h"
#include "modules\module_wheelled_light.h"
#include "algorithms\algorithm_mathUnit.h"
#include "algorithms\algorithm_moveAve.h"

#include "wheelled_core.h"
/*====================================================================================================*/
/*====================================================================================================*/
__IO int16_t AccelX;
__IO int16_t AccelY;
__IO int16_t AccelZ;
__IO int32_t dAccelZ;

__IO int16_t GyroZ;
__IO int32_t dGyroZ;

__IO int16_t theta = 0;

#define WHEELLED_FRONT
//#define WHEELLED_REAR

#ifdef WHEELLED_FRONT
  __IO uint16_t angle = 340;
  __IO uint16_t width = 165;
  __IO uint16_t light = 32;
  #define PARAM_K1  0.0064f
  #define PARAM_K2  50.0f
#else
  __IO uint16_t angle = 225;
  __IO uint16_t width = 165;
  __IO uint16_t light = 64;
  #define PARAM_K1  0.0064f
  #define PARAM_K2  50.0f
#endif

#define BLUETOOTH_CMD_KEYOFF 0
#define BLUETOOTH_CMD_KEYON  1
#define BLUETOOTH_CMD_LOCK   2
#define BLUETOOTH_CMD_UNLOCK 3
#define BLUETOOTH_CMD_ANGLE  4
#define BLUETOOTH_CMD_LIGHT  5
#define BLUETOOTH_CMD_WIDTH  6
#define BLUETOOTH_WARN_TILT  7
#define BLUETOOTH_WARN_RSSI  8

extern __IO uint8_t  bluetoothCMD;
extern __IO uint16_t bluetoothANG;
extern __IO uint16_t bluetoothLIG;
extern __IO uint16_t bluetoothWID;

void WheelLED_FrontFSM( void );
void WheelLED_RearFSM( void );
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_Init( void )
{
  HAL_Init();

  WLED_WheelLED_Config();
  WLED_IMU_Config();
  WLED_TIM_Config();
}
/*====================================================================================================*/
/*====================================================================================================*/
void WLED_Loop( void )
{
//  WheelLED_CMD_SetID();
  while(1) {
#ifdef WHEELLED_FRONT
    WheelLED_FrontFSM();
#else
    WheelLED_RearFSM();
#endif
  }
}
/*====================================================================================================*/
/*====================================================================================================*/
#define FSM_RUN   0
#define FSM_STOP  1
#define FSM_BRAKE 2
#define FSM_CMD   3
#define FSM_LOCK  4

#define AccelZ_threshold   360
#define dAccelZ_threshold  3800

#define GyroZ_threshold    1000
#define dGyroZ_threshold  -4000

void WheelLED_Light( uint16_t _angle, uint16_t _width, uint16_t _light )
{
  theta = (int16_t)(toDeg(atan2f(-AccelX, -AccelY)) + 180.0f);
  theta = (theta + _angle + 180) % 360;
  WheelLED_setLED(theta, _width, _light);
}

void WheelLED_FrontFSM( void )
{
  static uint8_t state = FSM_STOP;

  switch(state) {
    case FSM_RUN:
      WheelLED_Light(angle, width, light);
      if(bluetoothCMD != BLUETOOTH_CMD_KEYOFF)
        state = FSM_CMD;
      else if(GyroZ < GyroZ_threshold)
        state = FSM_STOP;
      break;

    case FSM_STOP:
      WheelLED_Light(330, 60, 1);
      if(bluetoothCMD != BLUETOOTH_CMD_KEYOFF)
        state = FSM_CMD;
      else if(GyroZ > GyroZ_threshold)
        state = FSM_RUN;
      break;

    case FSM_CMD:
      switch(bluetoothCMD) {

        case BLUETOOTH_CMD_KEYOFF:
          state = FSM_STOP;
          break;

        case BLUETOOTH_CMD_KEYON:
          WheelLED_Light(angle, width, light);
          break;

        case BLUETOOTH_CMD_ANGLE:
          while(bluetoothCMD == BLUETOOTH_CMD_ANGLE) {
            angle = bluetoothANG;
            WheelLED_Light(angle, width, light);
          }
          state = FSM_STOP;
          break;

        case BLUETOOTH_CMD_LIGHT: 
          while(bluetoothCMD == BLUETOOTH_CMD_LIGHT) {
            light = bluetoothLIG;
            if(light > 255)
              light = WLED_LIGHT_MAX;
            WheelLED_Light(angle, width, light);
          }
          state = FSM_STOP;
          break;

        case BLUETOOTH_CMD_WIDTH:
          while(bluetoothCMD == BLUETOOTH_CMD_WIDTH) {
            width = bluetoothWID;
            WheelLED_Light(angle, width, light);
          }
          state = FSM_STOP;
          break;

        case BLUETOOTH_CMD_LOCK:
          state = FSM_LOCK;
          break;

      }
      break;

    case FSM_LOCK:
      WheelLED_setLED_all(0);
      delay_ms(100);
      WheelLED_setLED_all(32);
      delay_ms(100);
      WheelLED_setLED_all(0);
      delay_ms(100);
      WheelLED_setLED_all(32);
      delay_ms(100);
      WheelLED_setLED_all(0);
      delay_ms(1000);
      WheelLED_setLED(0, 0, 0);
      while(bluetoothCMD != BLUETOOTH_CMD_UNLOCK) {
        if((AccelZ  > AccelZ_threshold)  || (AccelZ  < -AccelZ_threshold) ||
           (dAccelZ > dAccelZ_threshold) || (dAccelZ < -dAccelZ_threshold)) {
//          BUZZER_ON();
//          Linkit_SendData(LINKIT_WARN_TILT, LINKIT_WARN_TILT);
          WheelLED_setLED_all(32);
          delay_ms(100);
          WheelLED_setLED_all(0);
          delay_ms(100);
        }
        else {
//          BUZZER_OFF();
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
/*====================================================================================================*/
/*====================================================================================================*/
#ifdef WHEELLED_FRONT
#define Sxx   (0.122241f)
#define Syy   (0.122084f)
#define Szz   (0.121840f)
#define Bx    (44.313705f)
#define By    (188.735168f)
#define Bz    (189.599335f)

#define gyroX_offset  (-33)
#define gyroY_offset  (1)
#define gyroZ_offset  (10)

//G1 = 0.122241
//G2 = 0.000073
//G3 = 0.000262
//G4 = 0.122084
//G5 = 0.000053
//G6 = 0.121840
//O1 = 44.313705
//O2 = 188.735168
//O3 = 189.599335

#else
#define Sxx   (0.121966f)
#define Syy   (0.121677f)
#define Szz   (0.120579f)
#define Bx    (172.091141f)
#define By    (-102.026054f)
#define Bz    (166.469498f)

#define gyroX_offset  (7)
#define gyroY_offset  (19)
#define gyroZ_offset  (-2)

//G1  = 0.121966
//G2 = 0.000345
//G3 = -0.000094
//G4  = 0.121677
//G5 = 0.000046
//G6  = 0.120579
//O1  = 172.091141
//O2  = -102.026054
//O3  = 166.469498

#endif

static int16_t FIFO_ACC[3][16] = {0};
static int16_t FIFO_GYR[3][16] = {0};

void WLED_UpdateEven( TIM_HandleTypeDef *htim )
{
  static int16_t AccelZ_old = 0;
  static int16_t GyroZ_old = 0;

  float tmpGyrZ = 0.0f;

  int16_t readData[12] = {0};
  int16_t imu[6] = {0};

  MPU6500_getData(readData);

  imu[0] = (int16_t)((readData[1] - Bx) * Sxx);   // Acc.X
  imu[1] = (int16_t)((readData[2] - By) * Syy);   // Acc.Y
  imu[2] = (int16_t)((readData[3] - Bz) * Szz);   // Acc.Z
  imu[3] = readData[4] - gyroX_offset;    // Gyr.X
  imu[4] = readData[5] - gyroY_offset;    // Gyr.Y
  imu[5] = readData[6] - gyroZ_offset;    // Gyr.Z

  imu[0] = MoveAve_WMA(imu[0], FIFO_ACC[0], 4);
  imu[1] = MoveAve_WMA(imu[1], FIFO_ACC[1], 4);
  imu[2] = MoveAve_WMA(imu[2], FIFO_ACC[2], 4);
  imu[3] = MoveAve_WMA(imu[3], FIFO_GYR[0], 8);
  imu[4] = MoveAve_WMA(imu[4], FIFO_GYR[1], 8);
  imu[5] = MoveAve_WMA(imu[5], FIFO_GYR[2], 8);

  tmpGyrZ = toRad(imu[5]);
  AccelX  = imu[0] + (int16_t)(PARAM_K1 * tmpGyrZ * (PARAM_K2 + tmpGyrZ));
  AccelY  = imu[1];
  AccelZ  = imu[2];
  GyroZ   = imu[5];

  dAccelZ = (int32_t)((AccelZ - AccelZ_old) / SampleRate);
  dGyroZ  = (int32_t)((GyroZ - GyroZ_old) / SampleRate);

  AccelZ_old = AccelZ;
  GyroZ_old  = GyroZ;

  imu[3] = AccelX;
//  Serial_SendDataMATLAB(imu, 6);
}
/*====================================================================================================*/
/*====================================================================================================*/

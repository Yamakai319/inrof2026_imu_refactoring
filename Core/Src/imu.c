#include "imu.h"
#include "macro.h"
#include "stm32g474xx.h"
#include <stdint.h>
#include <main.h>
#include <stdio.h>
#include <math.h>

extern SPI_HandleTypeDef hspi2;
GPIO_TypeDef* const IMU_CS_PORTS[3] = {IMU1_CS_GPIO_Port, IMU2_CS_GPIO_Port, IMU3_CS_GPIO_Port};
const uint16_t IMU_CS_PINS[3]       = {IMU1_CS_Pin, IMU2_CS_Pin, IMU3_CS_Pin};
volatile IMUData imu[3];

void LSM6_Write(uint8_t reg, uint8_t data, int i)
{
    GPIO_TypeDef* PORT = IMU_CS_PORTS[i-1];
    uint16_t PIN = IMU_CS_PINS[i-1];
    uint8_t tx[2] = {reg & 0x7F, data};

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET);
}

uint8_t LSM6_Read(uint8_t reg, int i) { //1つのIMUから単独で読み出す場合
    GPIO_TypeDef* PORT = IMU_CS_PORTS[i-1];
    uint16_t PIN = IMU_CS_PINS[i-1];
    uint8_t tx = reg | 0x80;  // Read（MSB=1）
    uint8_t rx = 0;

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY); // アドレス送信
    HAL_SPI_Receive(&hspi2, &rx, 1, HAL_MAX_DELAY); // データ受信
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET); // CS HIGH

    return rx;
}

void LSM6_ReadMulti(uint8_t reg, uint8_t* pData, uint16_t size, int i) {
    GPIO_TypeDef* PORT = IMU_CS_PORTS[i-1];
    uint16_t PIN = IMU_CS_PINS[i-1];
    uint8_t addr = reg | 0x80;

    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_RESET);
    
    // まずレジスタアドレスを送る
    HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
    // その後、必要なサイズ分だけ受信する
    HAL_SPI_Receive(&hspi2, pData, size, HAL_MAX_DELAY);
    
    HAL_GPIO_WritePin(PORT, PIN, GPIO_PIN_SET);
}

void INIT_IMU(int i){
  /*センサの初期化*/
  LSM6_Write(0x12, 0x44, i); // CTRL3: reboot,BDU有効化,アドレス自動インクリメント有効化
  /*ジャイロの初期化*/
  LSM6_Write(0x15, 0x04, i); // CTRL6: FS=±2000dps
  LSM6_Write(0x11, 0x06, i); // CTRL2: ODR=120Hz
  /*加速度の初期化*/
  LSM6_Write(0x17, 0x03, i); // CTRL8: FS=±16g
  LSM6_Write(0x10, 0x06, i); // CTRL1: ODR=120Hz
}

void IMU_ReadAll(IMUData imu[3]){
    uint8_t buffer[12];
    for (int i=0; i<3; i++){
        LSM6_ReadMulti(0x22, buffer, 12, i+1); // IMU:一括読み出し
        imu[i].gx = ((int16_t)(buffer[1] << 8 | buffer[0]));
        imu[i].gy = ((int16_t)(buffer[3] << 8 | buffer[2]));
        imu[i].gz = ((int16_t)(buffer[5] << 8 | buffer[4]));
        imu[i].ax = ((int16_t)(buffer[7] << 8 | buffer[6]));
        imu[i].ay = ((int16_t)(buffer[9] << 8 | buffer[8]));
        imu[i].az = ((int16_t)(buffer[11] << 8 | buffer[10]));
    }  
}
void IMU_CalculateCombined(IMUData imu[3], 
    float gyro_x, float gyro_y, float gyro_z, 
    float accel_x, float accel_y, float accel_z, 
    float gyro_x_bias, float gyro_y_bias, float gyro_z_bias)
{
    gyro_x = (float)(-imu[1].gy + imu[0].gy*SIN_30_DEG - imu[0].gx*COS_30_DEG + imu[2].gx*COS_30_DEG + imu[2].gy*SIN_30_DEG)*IMU_GYRO_SENSITIVITY/3.0f - gyro_x_bias;
    gyro_y = (float)( imu[1].gx - imu[0].gy*COS_30_DEG - imu[0].gx*SIN_30_DEG - imu[2].gx*SIN_30_DEG + imu[2].gy*COS_30_DEG)*IMU_GYRO_SENSITIVITY/3.0f - gyro_y_bias;
    gyro_z = (float)( imu[0].gz + imu[1].gz + imu[2].gz )*IMU_GYRO_SENSITIVITY/3.0f - gyro_z_bias;
    gyro_z = gyro_z * IMU_GYRO_Z_SCALE;

    accel_x = (float)(-imu[1].ay + imu[0].ay*SIN_30_DEG - imu[0].ax*COS_30_DEG + imu[2].ax*COS_30_DEG + imu[2].ay*SIN_30_DEG)*IMU_ACCEL_SENSITIVITY/3.0f;
    accel_y = (float)( imu[1].ax - imu[0].ay*COS_30_DEG - imu[0].ax*SIN_30_DEG - imu[2].ax*SIN_30_DEG + imu[2].ay*COS_30_DEG)*IMU_ACCEL_SENSITIVITY/3.0f;
    accel_z = (float)( imu[0].az + imu[1].az + imu[2].az )*IMU_ACCEL_SENSITIVITY/3.0f;

    if (fabsf(gyro_x) < 0.5f) gyro_x = 0.0f;
    if (fabsf(gyro_y) < 0.5f) gyro_y = 0.0f;
    if (fabsf(gyro_z) < 0.5f) gyro_z = 0.0f;
}
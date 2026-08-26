#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/*構造体の定義*/
typedef struct {
    int16_t gx, gy, gz; //gyro (x,y,z)
    int16_t ax, ay, az; //accel(x,y,z)
} IMUData;

/*変数の宣言*/
extern volatile IMUData imu[3];

/*関数のプロトタイプ宣言*/
void LSM6_Write(uint8_t reg, uint8_t data, int port);
void LSM6_ReadMulti(uint8_t reg, uint8_t* pData, uint16_t size, int port);
uint8_t LSM6_Read(uint8_t reg, int port);
void INIT_IMU(int i);
void IMU_ReadAll(IMUData imu[3]);
void IMU_CalculateCombined(IMUData imu[3], 
    float *gyro_x, float *gyro_y, float *gyro_z, float *accel_x, float *accel_y, float *accel_z, 
    float gyro_x_bias, float gyro_y_bias, float gyro_z_bias);

#endif
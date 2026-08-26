#ifndef MACRO_H
#define MACRO_H

/*CANID*/
#define CAN_ID_YAW_RESET 0x500
#define CAN_ID_YAW_FEEDBACK 0x350

/*IMU*/
#define IMU_GYRO_SENSITIVITY 0.0175f
#define IMU_ACCEL_SENSITIVITY 0.122f
#define IMU_GYRO_Z_SCALE 0.9965f

#define MADGWICK_BETA 0.1f
#define DEG_TO_RAD 0.017453292519943295f //度からラジアンへの変換時にかける
#define RAD_TO_DEG 57.29577951308232f //ラジアンから度への変換時にかける

/*math*/
#define COS_30_DEG 0.8660254037844386f
#define SIN_30_DEG 0.5f

#endif
/**
 * @file imu_mpu6050.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief MPU6050 register list, functions, data structures, etc.
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */
 
#ifndef __IMU_MPU6050_H_
#define __IMU_MPU6050_H_

#include "imu.h"
#include <stdbool.h>
#include "Fusion.h"

/**
*   @defgroup IMU 
*   @{
*   @defgroup MPU6050
*   @{
*/

/**
*@defgroup MPU6050 Constants
* @{
*/
#define MPU6050_GYRO_FSR_250DPS  0  /**< MPU6050 gyroscope range ±250°/s */
#define MPU6050_GYRO_FSR_500DPS  1  /**< MPU6050 gyroscope range ±500°/s */
#define MPU6050_GYRO_FSR_1000DPS 2  /**< MPU6050 gyroscope range ±1000°/s */
#define MPU6050_GYRO_FSR_2000DPS 3  /**< MPU6050 gyroscope range ±2000°/s */

#define MPU6050_ACCEL_FSR_2G       0  /**< MPU6050 accelerometer range ±2g */
#define MPU6050_ACCEL_FSR_4G       1  /**< MPU6050 accelerometer range ±4g */
#define MPU6050_ACCEL_FSR_8G       2  /**< MPU6050 accelerometer range ±8g */
#define MPU6050_ACCEL_FSR_16G      3  /**< MPU6050 accelerometer range ±16g */
/** @} */

/**
*@defgroup MPU6050 Device Addresses
* @{
*/
#define MPU6050_DEV_ADDR_1 0x68 /**< MPU6050 device address */
#define MPU6050_DEV_ADDR_2 0x69 /**< MPU6050 device address */
/** @} */

/**
* @defgroup MPU6050 Register Addresses
* @{
*/
#define MPU6050_SMPLRT_DIV   0x19  /**< Gyroscope sample rate, typical: 0x07 (125Hz) */
#define MPU6050_CONFIG       0x1A  /**< Low-pass filter frequency, typical: 0x06 (5Hz) */
#define MPU6050_GYRO_CONFIG  0x1B  /**< Gyroscope self-test and range, typical: 0x18 (no self-test, 2000deg/s) */
#define MPU6050_ACCEL_CONFIG 0x1C  /**< Accelerometer self-test, range, and high-pass filter frequency, typical: 0x01 (no self-test, 2G, 5Hz) */

#define MPU6050_ACCEL_XOUT_H 0x3B  /**< High byte of X-axis acceleration */
#define MPU6050_ACCEL_XOUT_L 0x3C  /**< Low byte of X-axis acceleration */
#define MPU6050_ACCEL_YOUT_H 0x3D  /**< High byte of Y-axis acceleration */
#define MPU6050_ACCEL_YOUT_L 0x3E  /**< Low byte of Y-axis acceleration */
#define MPU6050_ACCEL_ZOUT_H 0x3F  /**< High byte of Z-axis acceleration */
#define MPU6050_ACCEL_ZOUT_L 0x40  /**< Low byte of Z-axis acceleration */

#define MPU6050_TEMP_OUT_H 0x41  /**< High byte of sensor temperature */
#define MPU6050_TEMP_OUT_L 0x42  /**< Low byte of sensor temperature */

#define MPU6050_GYRO_XOUT_H 0x43  /**< High byte of X-axis gyroscope */
#define MPU6050_GYRO_XOUT_L 0x44  /**< Low byte of X-axis gyroscope */
#define MPU6050_GYRO_YOUT_H 0x45  /**< High byte of Y-axis gyroscope */
#define MPU6050_GYRO_YOUT_L 0x46  /**< Low byte of Y-axis gyroscope */
#define MPU6050_GYRO_ZOUT_H 0x47  /**< High byte of Z-axis gyroscope */
#define MPU6050_GYRO_ZOUT_L 0x48  /**< Low byte of Z-axis gyroscope */

#define MPU6050_FIFO_EN_REG 0x23 /**< FIFO enable register */
#define MPU6050_USER_CTRL   0x6A /**< FIFO control register */
#define MPU6050_PWR_MGMT_1 0x6B  /**< Power management, typical: 0x00 (normal operation) */
#define MPU6050_WHO_AM_I 0x75    /**< IIC address register (default: 0x68, read-only) */
#define MPU6050_INT_EN_REG  0x38 /**< Interrupt enable register */

#define MPU6050_INT_PIN_CFG 0x37 /**< Interrupt pin configuration register */
/** @} */

typedef struct MPU6050Object  MPU6050ObjectTypeDef;
struct MPU6050Object {
    IMU_ObjectTypeDef base;

    FusionAhrs ahrs;
    FusionEuler euler;
    FusionQuaternion quat;
    FusionVector linearAcceleration;
    FusionOffset offset; /**< Gyroscope bias corrector, automatically corrects gyroscope bias when stationary for an extended time */

    FusionMatrix accel_misalignment; /**< Accelerometer axis misalignment */
    FusionMatrix accel_sensitivity; /**< Accelerometer scale factor */
    FusionMatrix accel_offset; /**< Accelerometer bias */

    FusionMatrix gyro_misalignment; /**< Gyroscope axis misalignment */
    FusionMatrix gyro_sensitivity; /**< Gyroscope scale factor */
    FusionMatrix gyro_offset; /**< Gyroscope bias */

    uint8_t dev_addr; /**< Device address corresponding to this instance */
    bool data_ready; /**< Data ready? From external interrupt */
    float gyro_sf; /**< Gyroscope scaling factor (value per 1°) */
    float accel_sf;  /**< Accelerometer scaling factor (value per 1g) */
    float accel[3]; /**< Last updated 3-axis acceleration */
    float gyro[3]; /**< Last updated 3-axis angular velocity */
    float temperature; /**< Last updated device temperature */

    /**
    * @brief Delay for x milliseconds
    * @param ms Number of milliseconds to delay
    */
    void (*sleep_ms)(uint32_t ms);

    /**
     * @brief Write data to the specified I2C device register
    * @param self Pointer to MPU6050 object instance
    * @param reg_addr Starting address of the register to write
    * @param len Number of registers to write
    * @param data Pointer to the data to write
    * @retval 0 Success
    * @retval !=0 Failure
    */
    int (*i2c_write_byte_to_mem)(MPU6050ObjectTypeDef *self, uint8_t reg_addr, uint8_t data);

    /**
     * @brief Read data from the specified I2C device register
     * @param self Pointer to MPU6050 object instance
     * @param reg_addr Starting address of the register to read
     * @param len Number of registers to read
     * @param data Pointer to store the resulting data
     * @retval 0 Success
     * @retval !=0 Failure
    */
    int (*i2c_read_from_mem)(MPU6050ObjectTypeDef *self, uint8_t reg_addr, uint32_t len, uint8_t *data);
};



int mpu6050_set_gyro_fsr(MPU6050ObjectTypeDef *self, uint32_t fsr);
int mpu6050_set_accel_fsr(MPU6050ObjectTypeDef *self, uint32_t fsr);
int mpu6050_set_lpf(MPU6050ObjectTypeDef *self, uint32_t lpf);
int mpu6050_set_rate(MPU6050ObjectTypeDef *self, uint32_t rate);
int mpu6050_get_temperature(MPU6050ObjectTypeDef *self, float *temp);
int mpu6050_get_accel(MPU6050ObjectTypeDef *self, float *x, float *y, float *z);
int mpu6050_get_gyro(MPU6050ObjectTypeDef *self, float *gx, float *gy, float *gz);
int mpu6050_get_all(MPU6050ObjectTypeDef *self, float *accel, float *temp, float *gyro);
void mpu6050_object_init(MPU6050ObjectTypeDef *obj, uint8_t dev_addr);

/** @} */
/** @} */

#endif

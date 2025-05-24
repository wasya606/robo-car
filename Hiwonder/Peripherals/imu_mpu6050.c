/**
 * @file imu_mpu6050.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief Implementation of the mpu6050 driver
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "imu_mpu6050.h"
#include <string.h>
#include "Fusion.h"

/**
*   @ingroup IMU 
*   @{
*   @addtogroup MPU6050
*   @{
*/

/**
    * @brief Set the full-scale range of the mpu6050 gyroscope sensor
    * @param self Pointer to the mpu6050 object instance
    * @param fsr @li 0 ±250dps
    *            @li 1 ±500dps
    *            @li 2 ±1000dps
    *            @li 3 ±2000dps
    * @retval  0 Success
    * @retval !=0 Failure
    */
int mpu6050_set_gyro_fsr(MPU6050ObjectTypeDef *self, uint32_t fsr)
{
    /* Regardless of the range, the output value is a 16-bit signed number.
       The range determines how many values correspond to 1 degree (refer to the chip manual). */
    switch(fsr) {
    case MPU6050_GYRO_FSR_250DPS:
        self->gyro_sf = 131.0f;
        break;
    case MPU6050_GYRO_FSR_500DPS:
        self->gyro_sf = 65.5f;
        break;
    case MPU6050_GYRO_FSR_1000DPS:
        self->gyro_sf = 32.8f;
        break;
    case MPU6050_GYRO_FSR_2000DPS:
        self->gyro_sf = 16.4f;
        break;
    default:
        return -2;
    }
    return self->i2c_write_byte_to_mem(self, MPU6050_GYRO_CONFIG, (uint8_t)(fsr << 3));
}

/**
* @brief Set the full-scale range of the mpu6050 accelerometer sensor
* @param self Pointer to the mpu6050 object instance
* @param fsr @li 0 ±2g
*            @li 1 ±4g
*            @li 2 ±8g
*            @li 3 ±16g
* @retval  0 Success
* @retval !=0 Failure
*/
int mpu6050_set_accel_fsr(MPU6050ObjectTypeDef *self, uint32_t fsr)
{
    switch(fsr) {
    case MPU6050_ACCEL_FSR_2G:
        self->accel_sf = 16384.0f;
        break;
    case MPU6050_ACCEL_FSR_4G:
        self->accel_sf = 8192.0f;
        break;
    case MPU6050_ACCEL_FSR_8G:
        self->accel_sf = 4096.0f;
        break;
    case MPU6050_ACCEL_FSR_16G:
        self->accel_sf = 2048.0f;
        break;
    default:
        return -2;
    }
    return self->i2c_write_byte_to_mem(self, MPU6050_ACCEL_CONFIG, (uint8_t)(fsr << 3));
}

/**
* @brief Set the mpu6050 digital low-pass filter
* @param self Pointer to the mpu6050 object instance
* @param lpf Digital low-pass filter frequency (Hz)
* @retval 0 Success
* @retval !=0 Failure
*/
int mpu6050_set_lpf(MPU6050ObjectTypeDef *self, uint32_t lpf)
{
    uint8_t data = 0;
    if(lpf > 188) {
        data = 1;
    } else if (lpf > 98) {
        data = 2;
    } else if (lpf > 42) {
        data = 3;
    } else if (lpf > 20) {
        data = 4;
    } else if(lpf > 10) {
        data = 5;
    } else {
        data = 6;
    }
    return self->i2c_write_byte_to_mem(self, MPU6050_CONFIG, data);
}

/**
* @brief Set the MPU6050 sampling rate
* @param self Pointer to the MPU6050 object instance
* @param rate New sampling rate between 4~1000 (Hz)
* @retval 0 Success
* @retval != Failure
*/
int mpu6050_set_rate(MPU6050ObjectTypeDef *self, uint32_t rate)
{
    uint32_t data = 0;
    if(rate > 1000) {
        rate = 1000;
    }
    if(rate < 4) {
        rate = 4;
    }
    data = 1000 / rate - 1;

    if(self->i2c_write_byte_to_mem(self, MPU6050_SMPLRT_DIV, data) != 0) { // Set the sampling rate
        return -2;
    }

    if(mpu6050_set_lpf(self, rate / 2) != 0) { // Set the digital low-pass filter to half the sampling rate
        return -1;
    }
    return 0;
}

/**
* @brief Read the MPU6050 temperature sensor value
* @param self Pointer to the MPU6050 object instance
* @param temp Pointer to store the temperature result
* @retval 0 Success
* @retval !=0 Failure
*/
int mpu6050_get_temperature(MPU6050ObjectTypeDef *self, float *temp)
{
    uint8_t buf[2] = {0};
    uint16_t raw = 0;
    if(self->i2c_read_from_mem(self, MPU6050_TEMP_OUT_H, 2, buf) != 0) {
        return -1;
    }
    raw = (((uint16_t)buf[0]) << 8) | ((uint16_t)buf[1]);
    printf("0x%.4X\r\n", raw);
    *temp = ((float)((int16_t)raw)) / 340.0f + 36.53f;
    return 0;
}

/**
* @brief Read MPU6050 accelerometer values
* @param self Pointer to the MPU6050 object instance
* @param x Pointer to store the X-axis result
* @param y Pointer to store the Y-axis result
* @param z Pointer to store the Z-axis result
* @retval 0 Success
* @retval != Failure
*/
int mpu6050_get_accel(MPU6050ObjectTypeDef *self, float *x, float *y, float *z)
{
    uint8_t buf[6] = {0};
    if(self->i2c_read_from_mem(self, MPU6050_ACCEL_XOUT_H, 6, buf) != 0) {
        return -1;
    }
    int16_t acc_x_raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    int16_t acc_y_raw = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    int16_t acc_z_raw = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];
    *x = (float)((int)acc_x_raw) / self->accel_sf;
    *y = (float)((int)acc_y_raw) / self->accel_sf;
    *z = (float)((int)acc_z_raw) / self->accel_sf;
    return 0;
}

/**
* @brief Read MPU6050 gyroscope values
* @param self Pointer to the MPU6050 object instance
* @param gx Pointer to store the X-axis result
* @param gy Pointer to store the Y-axis result
* @param gz Pointer to store the Z-axis result
* @retval 0 Success
* @retval != Failure
*/
int mpu6050_get_gyro(MPU6050ObjectTypeDef *self, float *gx, float *gy, float *gz)
{
    uint8_t buf[6] = {0};
    if(self->i2c_read_from_mem(self, MPU6050_GYRO_XOUT_H, 6, buf) != 0) {
        return -1;
    }

    int16_t gyro_x_raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    int16_t gyro_y_raw = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    int16_t gyro_z_raw = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];
    *gx = (float)((int)gyro_x_raw) / self->gyro_sf;
    *gy = (float)((int)gyro_y_raw) / self->gyro_sf;
    *gz = (float)((int)gyro_z_raw) / self->gyro_sf;
    return 0;
}

/**
* @brief Read MPU6050 accelerometer, temperature, and gyroscope values
* @param self Pointer to the MPU6050 object instance
* @param accel Pointer to store XYZ-axis accelerometer results
* @param temp Pointer to store the temperature result
* @param gyro Pointer to store XYZ-axis gyroscope results
* @retval 0 Success
* @retval !=0 Failure
*/
int mpu6050_get_all(MPU6050ObjectTypeDef *self, float *accel, float *temp, float *gyro)
{
    static uint8_t buf[14] = {0};
    if(self->i2c_read_from_mem(self, MPU6050_ACCEL_XOUT_H, 14, buf) != 0) {
        return -1;
    }

    int16_t acc_x_raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    int16_t acc_y_raw = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];
    int16_t acc_z_raw = ((uint16_t)buf[4] << 8) | (uint16_t)buf[5];
    int16_t temp_raw = ((uint16_t)buf[6] << 8) | (uint16_t)buf[7];
    int16_t gyro_x_raw = ((uint16_t)buf[8] << 8) | (uint16_t)buf[9];
    int16_t gyro_y_raw = ((uint16_t)buf[10] << 8) | (uint16_t)buf[12];
    int16_t gyro_z_raw = ((uint16_t)buf[12] << 8) | (uint16_t)buf[13];

    *accel++ = (float)((int)acc_x_raw) / self->accel_sf;
    *accel++ = (float)((int)acc_y_raw) / self->accel_sf;
    *accel++ = (float)((int)acc_z_raw) / self->accel_sf;
    *temp = ((float)((int)temp_raw)) / 340.0f + 36.53f;
    *gyro++ = (float)((int)gyro_x_raw) / self->gyro_sf;
    *gyro++ = (float)((int)gyro_y_raw) / self->gyro_sf;
    *gyro++ = (float)((int)gyro_z_raw) / self->gyro_sf;

    return 0;
}

/**
* @brief Reset the MPU6050 device
* @param self Pointer to the MPU6050 object instance
* @retval None
*/
static void export_mpu6050_reset(IMU_ObjectTypeDef *self_base)
{
    MPU6050ObjectTypeDef *self = (MPU6050ObjectTypeDef*)self_base;
    self->i2c_write_byte_to_mem(self, MPU6050_PWR_MGMT_1, 0x80); // Reset MPU6050
    self->i2c_write_byte_to_mem(self, MPU6050_PWR_MGMT_1, 0x80); // Reset MPU6050
    self->sleep_ms(50);
    self->i2c_write_byte_to_mem(self, MPU6050_PWR_MGMT_1, 0x00); // Wake up MPU6050
    self->i2c_write_byte_to_mem(self, MPU6050_PWR_MGMT_1, 0x00); // Wake up MPU6050

    mpu6050_set_accel_fsr(self, MPU6050_ACCEL_FSR_4G); // Set accelerometer range to ±4G
    mpu6050_set_gyro_fsr(self, MPU6050_GYRO_FSR_2000DPS); // Set gyroscope range to ±2000°/s

    self->i2c_write_byte_to_mem(self, MPU6050_INT_EN_REG, 0x00); // Disable interrupt
    self->i2c_write_byte_to_mem(self, MPU6050_USER_CTRL, 0x00);  // Disable I2C master mode
    self->i2c_write_byte_to_mem(self, MPU6050_FIFO_EN_REG, 0x00); // Disable FIFO
    self->i2c_write_byte_to_mem(self, MPU6050_INT_PIN_CFG, 0x00); // Interrupt triggered by high level
    mpu6050_set_rate(self, 100); // Set sampling rate to 100SPS
    self->i2c_write_byte_to_mem(self, MPU6050_INT_EN_REG, 0x01); // Enable data ready interrupt
}

/**
* @brief Configure MPU6050 FIFO interrupt
* @param self Pointer to the MPU6050 object instance
* @param enable @li true Enable FIFO interrupt
*               @li false Disable FIFO interrupt
* @retval 0 Success
* @retval !=0 Failure
*/
int mpu6050_enable_int(MPU6050ObjectTypeDef *self, bool enable)
{
    if(enable) {
        if(self->i2c_write_byte_to_mem(self, MPU6050_INT_EN_REG, 0x01) != 0) {
            return -1;
        }
    } else {
        if(self->i2c_write_byte_to_mem(self, MPU6050_INT_EN_REG, 0x00) != 0) {
            return -1;
        }
    }
    return 0;
}


/**
* @brief Read all sensor values from MPU6050 and update instance members
* @param self_ Pointer to the MPU6050 object instance
* @retval 0 Update successful
* @retval !=0 Update failed
*/
static int export_mpu6050_update(IMU_ObjectTypeDef *self_base)
{
    MPU6050ObjectTypeDef *self = (MPU6050ObjectTypeDef*)self_base;
    //LL_GPIO_SetOutputPin(LED_SYS_GPIO_Port, LED_SYS_Pin); // Measure the rising edge of the data collection and calculation time
    if(mpu6050_get_all(self, self->accel, &self->temperature, self->gyro) != 0) {
        return -1;
    }
    FusionVector gyroscope = {.axis.x= self->gyro[0], .axis.y=self->gyro[1], .axis.z = self->gyro[2]};
    FusionVector accelerometer = {.axis.x=self->accel[0], .axis.y = self->accel[1], .axis.z = self->accel[2]};
    gyroscope = FusionOffsetUpdate(&self->offset, gyroscope);
    if(self->offset.cal_count > 0) {
        return 0;
    }

    FusionAhrsUpdateNoMagnetometer(&self->ahrs, gyroscope, accelerometer, 0.01);
    const FusionQuaternion quat = FusionAhrsGetQuaternion(&self->ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(quat);
    //self->linearAcceleration = FusionAhrsGetLinearAcceleration(&self->ahrs);
    memcpy(&self->quat, &quat, sizeof(FusionQuaternion));
    memcpy(&self->euler, &euler, sizeof(FusionEuler));
    //LL_GPIO_ResetOutputPin(LED_SYS_GPIO_Port, LED_SYS_Pin); // Measure the falling edge of the data collection and calculation time
//    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    return 0;
}

/**
* @brief Get Euler angles from MPU6050
* @param self Pointer to the IMU object instance
* @param rpy Pointer to store the Roll, Pitch, and Yaw angles
* @retval 0 Success
*/
int export_mpu6050_get_euler(IMU_ObjectTypeDef *self, float *rpy)
{
    memcpy(rpy, &((MPU6050ObjectTypeDef*)self)->euler, sizeof(FusionEuler));
    return 0;
}

/**
* @brief Get quaternion from MPU6050
* @param self Pointer to the IMU object instance
* @param quat Pointer to store the quaternion values
* @retval 0 Success
*/
int export_mpu6050_get_quat(IMU_ObjectTypeDef *self, float *quat)
{
    memcpy(quat, &((MPU6050ObjectTypeDef*)self)->quat, sizeof(FusionQuaternion));
    return 0;
}

void mpu6050_data_ready_read(IMU_ObjectTypeDef *self)
{
    const MPU6050ObjectTypeDef* imuDev = (MPU6050ObjectTypeDef*)self;
    printf("---- Accel\tX: %f,\tY: %f,\tZ: %f\n", imuDev->accel[0], imuDev->accel[1], imuDev->accel[2]);
    printf("~~~~ Gyro\tX: %f,\tY: %f,\tZ: %f\n\n", imuDev->gyro[0], imuDev->gyro[1], imuDev->gyro[2]);
}

/**
  * @brief Initialize MPU6050 object memory
  * @param obj Pointer to the object to be initialized
  * @param dev_addr Device address
  * @retval None
  */
void mpu6050_object_init(MPU6050ObjectTypeDef *obj, uint8_t dev_addr)
{
    memset(obj, 0, sizeof(MPU6050ObjectTypeDef));
    FusionAhrsInitialise(&obj->ahrs);
    FusionOffsetInitialise(&obj->offset, 100);
    obj->dev_addr = dev_addr;
    obj->base.reset = export_mpu6050_reset;
    obj->base.update = export_mpu6050_update;
    obj->base.get_euler = export_mpu6050_get_euler;
    obj->base.get_quat = export_mpu6050_get_quat;
    obj->base.on_data_ready_read = mpu6050_data_ready_read;
}

/** @} */
/** @} */

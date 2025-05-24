/**
 * @file imu.h
 * @author 
 * @brief IMU-related data definitions and function declarations
 * @version 0.1
 * @date 2023-05-06
 *
 * @copyright 
 *
 */
#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>
#include <stdio.h>

/**
 * @defgroup IMU
 * @{
*/

/**
* @brief Enumeration for the axes that might appear in the IMU
*
*/
typedef enum {
    IMU_AXIS_X, /**< @brief IMU X-axis */
    IMU_AXIS_Y, /**< @brief IMU Y-axis */
    IMU_AXIS_Z, /**< @brief IMU Z-axis */
    IMU_AXIS_W, /**< @brief IMU W-axis (quaternion) */
} IMU_AxisEnum;

typedef struct IMU_Object IMU_ObjectTypeDef; /**< @brief Type definition declaration for IMU object */

/**
* @brief Structure for the IMU object
*
*/
struct IMU_Object {
    /**
      * @brief Reset the IMU
    */
    void (*reset)(IMU_ObjectTypeDef *self);

    /**
      * @brief Calibrate the IMU with the current posture as the baseline
      */
    void (*calibrat)(IMU_ObjectTypeDef *self);

    /**
      * @brief Update data from the sensor
    */
    int (*update)(IMU_ObjectTypeDef *self);

    /**
     * @brief Get the current posture's Euler angles
     * @param [out] rpy Pointer to a float array of length 3 to store the result
     * @retval 0 Success, rpy values are valid
     * @retval !=0 Failure, rpy values are invalid
    */
    int (*get_euler)(IMU_ObjectTypeDef *self, float *rpy);

    /**
      * @brief Get the current posture's quaternion
      * @param [out] quat Pointer to a float array of length 4 to store the result
      * @retval 0 Success, quat values are valid
      * @retval !=0 Failure, quat values are invalid
    */
    int (*get_quat)(IMU_ObjectTypeDef *self, float *quat);

    /**
      * @brief Get the current acceleration (G values) for all axes
      * @param [out] xyz Pointer to a float array of length 3 to store the result
      * @retval 0 Success, xyz values are valid
      * @retval !=0 Failure, xyz values are invalid
    */
    int (*get_accel)(IMU_ObjectTypeDef *self, float *xyz);

    /**
      * @brief Get the current angular velocity values for all axes
      * @param [out] xyz Pointer to a float array of length 3 to store the result
      * @retval 0 Success, xyz values are valid
      * @retval !=0 Failure, xyz values are invalid
    */
    int (*get_gyro)(IMU_ObjectTypeDef *self, float *xyz);

    /**
      * @brief Get the current acceleration (G value) for a specific axis
      * @param axis The axis to retrieve
      * @param [out] accel Pointer to store the result acceleration
      * @retval 0 Success, accel value is valid
      * @retval !=0 Failure, accel value is invalid
    */
    int (*get_accel_axis)(IMU_ObjectTypeDef *self, IMU_AxisEnum axis, float *accel);

    /**
      * @brief Get the current angular velocity for a specific axis
      * @param axis The axis to retrieve
      * @param [out] gyro Pointer to store the result angular velocity
      * @retval 0 Success, gyro value is valid
      * @retval !=0 Failure, gyro value is invalid
    */
    int (*get_gyro_axis)(IMU_ObjectTypeDef *self, IMU_AxisEnum axis, float *gyro);
    
    void (*on_data_ready_read)(IMU_ObjectTypeDef *self);
};

/** @} */
#endif

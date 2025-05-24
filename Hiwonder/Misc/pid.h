/**
 * @file pid.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief Declaration of PID-related data structures and methods
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _PID_H
#define _PID_H

#include <stdint.h>

/**
  * @brief PID controller structure
  *
  */
typedef struct {
    float set_point; /**< @brief Target value */
    float kp;        /**< @brief Proportional gain */
    float ki;        /**< @brief Integral gain */
    float kd;        /**< @brief Derivative gain */
    
    float previous_0_err; /**< @brief Previous error */
    float previous_1_err; /**< @brief Second-to-last error */
    
    float output; /**< @brief PID output */
}PID_ControllerTypeDef;



/**
 * @brief PID control update
 * @param self Pointer to the PID controller object
 * @param actual Current actual value
 * @param time_delta Time interval since the last update
 * @retval None.
 */
void pid_controller_update(PID_ControllerTypeDef *self, float actual, float time_delta);



/**
 * @brief Initialize the PID controller
 * @param self Pointer to the PID controller to initialize
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @retval None.
 */
void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd);

#endif

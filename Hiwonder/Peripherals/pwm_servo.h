/**
 * @file pwm_servo.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief st7735显示屏驱动
 * @version 0.1
 * @date 2023-05-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __PWM_SERVO_H_
#define __PWM_SERVO_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct PWMServoObject  PWMServoObjectTypeDef;

/**
 * @brief PWM Servo Object Structure
 */
struct PWMServoObject {
    int id;     /**< @brief Servo ID */
    int offset; /**< @brief Servo offset */
    int target_duty;  /**< @brief Target pulse width */
    int current_duty; /**< @brief Current pulse width */
    int duty_raw;     /**< @brief Raw pulse width, the final pulse width written to the timer, includes offset, == current_duty + offset */

    /* Variables needed for speed control */
    uint32_t duration;    /**< @brief Time for the servo to move from the current angle to the specified angle, which controls the speed (unit: ms) */
    float duty_inc; /**< @brief Increment of pulse width per position update */
    int  inc_times; /**< @brief Number of increments required */
    bool is_running;  /**< @brief Whether the servo is in motion */
    bool duty_changed; /**< @brief Whether the pulse width has been changed */

    /* Provided externally, hardware abstraction interface */
    void (*write_pin)(uint32_t new_state); /* IO pin level setting */
};

/**
 * @brief Servo object initialization
 * @param object Pointer to the servo object to be initialized
 * @retval None.
*/
void pwm_servo_object_init(PWMServoObjectTypeDef *object);

/**
 * @brief Servo pulse width control
 * @details Calculates the pulse width change and the pulse width required for the current speed, and implements control. Needs to be called every 50 ms.
 * @param self Pointer to the servo object to be controlled
 * @retval None.
*/
void pwm_servo_duty_compare(PWMServoObjectTypeDef *self);

/**
 * @brief Set servo angle
 * @details Used to set the servo angle. Actually, it only sets a few internal variables of the servo object.
 * It does not immediately control the servo. The actual control is calculated and implemented by pwm_servo_duty_compare.
 * @param self Pointer to the servo object to be controlled
 * @param duty New pulse width of the servo (an integer value between 500~2500)
 * @param duration Time for the movement (in ms)
 * @retval None.
*/
void pwm_servo_set_position (PWMServoObjectTypeDef *self, uint32_t duty, uint32_t duration);

/**
 * @brief Set servo offset
 * @param self Pointer to the servo object to be controlled
 * @param offset New servo offset
 * @retval None.
*/
void pwm_servo_set_offset(PWMServoObjectTypeDef *self, int offset);

#endif


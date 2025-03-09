#ifndef __ACKERMANN_CHASSIS_H
#define __ACKERMANN_CHASSIS_H
#include "chassis.h"
#include <stdint.h>

enum MotorPosition {
    MOTOR_RIGHT,
    MOTOR_LEFT
};

enum MotorSpeedValueType {
    RPS,
    TPS
};

typedef struct {
	ChassisTypeDef base;
	float wheelbase;
	float shaft_length;
	float wheel_diameter;
	float correction_factor;
    float target_speed;
	void (*set_motors)(void *self, float rps_l, float rps_r, int position);
    void (*set_speed)(const float speed_left, const float speed_right);
    int (*get_current_helm_position)();
    float (*get_current_speed)();
    float (*get_motor_speed)(enum MotorPosition motor_position, enum MotorSpeedValueType speed_value_type);
}AckermannChassisTypeDef;
void ackermann_chassis_object_init(AckermannChassisTypeDef *self);

#endif


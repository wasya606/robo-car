/**
 * @file app.c
 * @author Wu TongXing (Lucas@hiwonder.com)
 * @brief Main application logic
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "cmsis_os2.h"
#include "led.h"
#include "lwmem_porting.h"
#include "global.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "adc.h"
#include "u8g2_porting.h"
#include "pwm_servo.h"
#include "serial_servo.h"
#include "FlashSave_porting.h"

/* PTZ Servo Limit Parameters */
#define HOLDER_MIN 500  // Corresponding to 0° of the servo
#define HOLDER_MAX 2500 // Corresponding to 180° of the servo

#define MINACKER_PWM_DEV 23 // Low limit 800 , 1500-800=700  2200-1500=700  700/30= 23 ··· 10

/* Buzzer frequency */
#define BUZZER_FREQ 1300

// Chassis type (default is four-wheel differential chassis)
uint32_t Chassis_run_type = CHASSIS_TYPE_TI4WD;
uint8_t save_flash = 0;

/* Hardware initialization statement */
void buzzers_init(void);    // buzer
void buttons_init(void);    // button
void leds_init(void);       // LED
void motors_init(void);     // Motor
void pwm_servos_init(void); // Servo
void chassis_init(void);    // Initialization of car parameters

// Button callback function
void button_event_callback(ButtonObjectTypeDef *button, ButtonEventIDEnum event)
{
    if (button == buttons[0])
    { /* Button 1 event  */
        if (event == BUTTON_EVENT_CLICK)
        { // If it is a single click
            // Chassis type++
            Chassis_run_type++;
            if (CHASSIS_TYPE_NONE <= Chassis_run_type)
            {
                Chassis_run_type = CHASSIS_TYPE_TI4WD;
            }
            // Reinitialize chassis
            set_chassis_type(Chassis_run_type);

            // Enter the critical section of the interrupt level code
            uint32_t ret = taskENTER_CRITICAL_FROM_ISR();
            // Write to Flash
            hw_flash_write(SAVE_ADD, &Chassis_run_type, 1);
            // Exit the critical section of the interrupt level code
            taskEXIT_CRITICAL_FROM_ISR(ret);

            // Buzzer
            buzzer_didi(buzzers[0], 1800, 100, 200, Chassis_run_type); // Buzzer sounds, indicating chassis type
        }
    }
    if (button == buttons[1])
    { /* Button 2 event  */
        if (event == BUTTON_EVENT_LONGPRESS)
        {                                                              // If it is a long press
            buzzer_didi(buzzers[0], 1800, 100, 200, Chassis_run_type); // Buzzer sounds, indicating chassis type
        }
        if (event == BUTTON_EVENT_CLICK)
        { // If it is a long press
        }
    }
}

// IMU read timer callback function
void IMU_read_timer_callback(void *argument)
{
    //	//Note: MCU should not print floating point numbers as much as possible
    //	static float rpy[3] = {0,0,0};
    //	static char msg[16];
    //	imus[0]->get_euler(imus[0], rpy);	//Reading IMU data
    //	sprintf(msg, "Roll :%3.2f", rpy[0]);	//Convert data into a string
    //	printf("%s\n",msg);		//Printing a string
    //	sprintf(msg, "Pitch:%3.2f", rpy[1]);	//Convert data into a string
    //	printf("%s\n",msg);		//Printing a string
    //	sprintf(msg, "Yaw  :%3.2f", rpy[2]);	//Convert data into a string
    //	printf("%s\n",msg);		//Printing a string
}

void send_type(ChassisTypeEnum chassis_type);

// Four-wheel differential vehicle control function
void ti4wd_control(char msg);
// Small tracked vehicle control function
void tankblack_control(char msg);
// Large tracked vehicle control function
void jettank_control(char msg);
// Mecanum wheel car control function
void jetauto_control(char msg);
// Large Ackerman Car Control Function
void jetacker_control(char msg);
// Small Ackerman car control function
void minacker_control(char msg);

// PTZ control function
void holder_control(char msg, uint8_t servo_1, uint8_t servo_2);

/* User entry function*/
void app_task_entry(void *argument)
{
    /* Declaring an external handle */
    // Buzzer handle
    extern osTimerId_t buzzer_timerHandle;
    // Power monitoring handle
    extern osTimerId_t battery_check_timerHandle;
    // LED control handle
    extern osTimerId_t led_timerHandle;
    // Button control handle
    extern osTimerId_t button_timerHandle;
    // IMU timer handle
    extern osTimerId_t IMU_read_timerHandle;

    // Queue handle for motion control
    // The handle signal is pushed into the USBH_HID_EventCallback() callback function in gampad_handle.c
    // Take out the control of the car movement in the user entry function
    extern osMessageQueueId_t moving_ctrl_queueHandle;

    /* Flash read type */
    hw_flash_Read(SAVE_ADD, &Chassis_run_type, 1);
    if (CHASSIS_TYPE_START >= Chassis_run_type || CHASSIS_TYPE_NONE <= Chassis_run_type)
    {
        Chassis_run_type = CHASSIS_TYPE_TI4WD; // Set to differential
        hw_flash_write(SAVE_ADD, &Chassis_run_type, 1);
    }

    /* Hardware Initialization */
    motors_init();     // Motor initialization
    pwm_servos_init(); // Gimbal Servo Initialization
    leds_init();       // LED Initialization
    buzzers_init();    // Buzzer initialization
    buttons_init();    // Button initialization

    // Register a button callback function to process the button value
    button_register_callback(buttons[0], button_event_callback);
    button_register_callback(buttons[1], button_event_callback);

    // Turn on LED timer
    osTimerStart(led_timerHandle, LED_TASK_PERIOD);
    // Turn on the buzzer timer and let it run in the interrupt, then call the interface function
    // Parameter 1: timer handle, Parameter 2: timer working interval ms
    osTimerStart(buzzer_timerHandle, BUZZER_TASK_PERIOD);
    // Start the key timer
    osTimerStart(button_timerHandle, BUTTON_TASK_PERIOD);
    // Turn on the power monitoring timer to monitor the power in real time
    osTimerStart(battery_check_timerHandle, BATTERY_TASK_PERIOD);
    // Start the IMU reading timer, reading once every 500ms
    osTimerStart(IMU_read_timerHandle, IMU_TASK_PERIOD);
    // Start ADC channel conversion
    HAL_ADC_Start(&hadc1);

    char msg = '\0';
    uint8_t msg_prio;
    // Initialize the motion control queue (parameter: queue handle)
    osMessageQueueReset(moving_ctrl_queueHandle);

    // Initialize chassis motor motion parameters
    chassis_init();

    // Select chassis type
    set_chassis_type(Chassis_run_type);

    osDelay(2000);

    buzzer_didi(buzzers[0], 1800, 100, 200, Chassis_run_type); // The buzzer sounds, indicating the chassis type

    // Loop: The loop in the RTOS task must have osDelay or other system blocking functions, otherwise it will cause system abnormalities
    for (;;)
    {

        // Receive information from the motion control queue. If the acquisition takes more than 100ms, it is considered unsuccessful and the motor is stopped, skipping this cycle.
        // osMessageQueueGet() Get the message in the queue
        //  Parameter 1: message queue handle
        //  Parameter 2: the address of the object to be placed (here is a char type address)
        //  Parameter 3: message priority
        //  Parameter 4: Timeout setting (you can set the waiting time for acquisition)
        if (osMessageQueueGet(moving_ctrl_queueHandle, &msg, &msg_prio, 300) != osOK)
        {
            printf("stop\n");
            chassis->stop(chassis);
            continue;
        }

        printf("msg: %c\r\n", msg); // debug print

        // Chassis control function
        switch (Chassis_run_type)
        {
        case CHASSIS_TYPE_TI4WD: // Differential
        case CHASSIS_TYPE_HUGE_TI4WD:
            //				printf("CHASSIS_TYPE_TI4WD\n");
            ti4wd_control(msg);
            // PTZ
            holder_control(msg, 0, 1);
            break;
        case CHASSIS_TYPE_TANKBLACK: // Small crawler
            //				printf("CHASSIS_TYPE_TANKBLACK\n");
            tankblack_control(msg);
            holder_control(msg, 0, 1);
            break;
        case CHASSIS_TYPE_JETTANK: // Large Tracks
            //				printf("CHASSIS_TYPE_JETTANK\n");
            jettank_control(msg);
            holder_control(msg, 0, 1);
            break;
        case CHASSIS_TYPE_JETAUTO: // McLennan
            //				printf("CHASSIS_TYPE_JETAUTO\n");
            jetauto_control(msg);
            holder_control(msg, 0, 1);
            break;
        case CHASSIS_TYPE_JETACKER: // Big Ackerman
            //				printf("CHASSIS_TYPE_JETACKER\n");
            jetacker_control(msg);
            holder_control(msg, 0, 1);
            break;
        case CHASSIS_TYPE_MINACKER: // Ackerman Jr.
            minacker_control(msg);
            break;

        default: // Differential
            //				printf("Chassis_run_type:default\n");
            Chassis_run_type = CHASSIS_TYPE_TI4WD;
            ti4wd_control(msg);
            holder_control(msg, 0, 1);
            break;
        }
    }
}

void send_type(ChassisTypeEnum chassis_type)
{
    extern osMessageQueueId_t bluetooth_tx_queueHandle; /* Bluetooth data sending queue */
    char msg[8] = "";
    uint32_t type = 0;
    switch (chassis_type)
    {
    case CHASSIS_TYPE_MINACKER:
        type = 1;
        break;

    case CHASSIS_TYPE_JETAUTO:
        type = 2;
        break;
    case CHASSIS_TYPE_JETTANK:
        type = 3;
        break;
    case CHASSIS_TYPE_TI4WD:
        type = 4;
        break;
    case CHASSIS_TYPE_TANKBLACK:
        type = 5;
        break;
    case CHASSIS_TYPE_JETACKER:
        type = 6;
        break;

    default:
        break;
    }
    sprintf(&msg[1], "T%dT", type);
    msg[0] = 3;
    printf("%s\n", msg);
    osMessageQueuePut(bluetooth_tx_queueHandle, msg, 0, 0); /* Push into the send queue */
}

/*
 *  Four-wheel differential chassis control function
 *  Parameters: Control command
 *  This function receives a char type parameter to determine which action to run.
 */
void ti4wd_control(char msg)
{
    // Define the motor speed
    // The recommended range is [50 , 450]
    static float speed = 300.0f;

    switch (msg)
    {

    case 'S':
    { // START
        // The connection is successful and the buzzer sounds once.
        // buzzer_didi() Buzzer interface
        // Parameter 1: itself
        // Parameter 2: Buzzer frequency (adjustable buzzer tone)
        // Parameter 3; ringing time in ms
        // Parameter 4: silent time (ms)
        // Parameter 5: Number of beeps
        buzzer_didi(buzzers[0], BUZZER_FREQ, 150, 200, 1);
        send_type(CHASSIS_TYPE_TI4WD);
        break;
    }

    /* Chassis motor */
    case 'I':
    { // Left Stick Middle
        // Motor stop
        chassis->stop(chassis);
        break;
    }
    case 'A':
    { // Left Stick Up
        // Moving at the linear speed of x-axis speed (i.e. moving forward)
        // set_velocity()
        // Parameter 1: itself, parameter 2: x-axis speed, parameter 3: y-axis speed, parameter 4: angular velocity of the car
        chassis->set_velocity(chassis, speed, 0, 0);
        break;
    }
    case 'B':
    { // Left stick upper left
        // The x-axis speed is used to make a circular motion around a circle with a radius of 250 mm on the left side.
        // set_velocity_radius() Rotation function
        // Parameter 1: itself
        // Parameter 2: x speed
        // Parameter 3: Radius (mm)
        // Parameter 4: Whether to rotate (the differential car is not suitable for spinning due to its own structure)
        chassis->set_velocity_radius(chassis, speed, 300, false);
        break;
    }
    case 'C':
    { // Left Stick Left
        // With the linear speed of x-axis speed, it moves in a circle around its left side with a radius of 100mm
        chassis->set_velocity_radius(chassis, speed, 100, false);
        break;
    }
    case 'D':
    { // Left stick lower left
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 250mm on its left side.
        chassis->set_velocity_radius(chassis, -speed, 250, false);
        break;
    }
    case 'E':
    { // Left Stick Down
        // Moving at a linear velocity of x-speed (i.e. backwards）
        chassis->set_velocity(chassis, -speed, 0, 0);
        break;
    }
    case 'F':
    { // Left stick right down
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 250mm to the right of itself.
        chassis->set_velocity_radius(chassis, -speed, -250, false);
        break;
    }
    case 'G':
    { // Left Stick Right
        // With the linear speed of x-axis speed, it moves in a circle with a radius of 100mm on its right side.
        chassis->set_velocity_radius(chassis, speed, -100, false);
        break;
    }
    case 'H':
    { // Left stick Up right
        // With the linear speed of x-axis speed, it moves in a circle with a radius of 500mm on its right side.
        chassis->set_velocity_radius(chassis, speed, -250, false);
        break;
    }

    /* Right button */
    case 'j':
    { // triangle
        // Acceleration
        speed += 50;
        speed = speed > 450 ? 450 : speed;
        break;
    }
    case 'l':
    { // square
        chassis->set_velocity(chassis, 0, speed, 0);
        break;
    }
    case 'n':
    { // fork
        // Deceleration
        speed -= 50;
        speed = speed < 50 ? 50 : speed;
        break;
    }
    case 'p':
    { // circle
        chassis->set_velocity(chassis, 0, -speed, 0);
        break;
    }

    default:
        break;
    }
}

/*
 *  Small track chassis control function
 *  Parameters: Control command
 *  This function receives a char type parameter to determine which action to run.
 */
void tankblack_control(char msg)
{
    // Define the motor speed
    // The recommended range is [50 , 450]
    static float speed = 300.0f;

    switch (msg)
    {

    case 'S':
    { // START
        // The connection is successful and the buzzer sounds once.
        // buzzer_didi() Buzzer interface
        // Parameter 1: itself
        // Parameter 2: Buzzer frequency (adjustable buzzer tone)
        // Parameter 3; ringing time in ms
        // Parameter 4: silent time (ms)
        // Parameter 5: Number of beeps
        buzzer_didi(buzzers[0], BUZZER_FREQ, 150, 200, 1);
        send_type(CHASSIS_TYPE_TANKBLACK);
        break;
    }

    /* Chassis motor */
    case 'I':
    { // Left Stick Middle
        // Motor stop
        chassis->stop(chassis);
        break;
    }
    case 'A':
    { // Left Stick Up
        // Moving at the linear speed of x-axis speed (i.e. moving forward)
        // set_velocity()
        // Parameter 1: itself, parameter 2: x-axis speed, parameter 3: y-axis speed, parameter 4: angular velocity of the car
        chassis->set_velocity(chassis, speed, 0, 0);
        break;
    }
    case 'B':
    { // Left stick upper left
        // With the linear speed of x-axis speed, it moves in a circle around its left side with a radius of 300mm
        // set_velocity_radius() Rotation function
        // Parameter 1: itself
        // Parameter 2: x speed
        // Parameter 3: Radius mm [positive on the left, negative on the right]
        // Parameter 4: Whether to rotate
        chassis->set_velocity_radius(chassis, speed, 300, false);
        break;
    }
    case 'C':
    { // Left Stick Left
        // Rotate left at speed
        chassis->set_velocity_radius(chassis, speed, 150, true);
        break;
    }
    case 'D':
    { // Left stick lower left
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 300mm on its left side.
        chassis->set_velocity_radius(chassis, -speed, 300, false);
        break;
    }
    case 'E':
    { // Left Stick Down
        // Moving at a linear speed of -speed on the x-axis (i.e. backwards)
        chassis->set_velocity(chassis, -speed, 0, 0);
        break;
    }
    case 'F':
    { // Left stick right down
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 300mm to the right of itself.
        chassis->set_velocity_radius(chassis, -speed, -300, false);
        break;
    }
    case 'G':
    { // Left Stick Right
        // Rotate right at speed
        chassis->set_velocity_radius(chassis, speed, -150, true);
        break;
    }
    case 'H':
    { // Left stick Up right
        // With the linear speed of x-axis speed, it moves in a circle with a radius of 300mm on its right side.
        chassis->set_velocity_radius(chassis, speed, -300, false);
        break;
    }

    /* Right button */
    case 'j':
    { // triangle
        // Acceleration
        speed += 50;
        speed = speed > 450 ? 450 : speed;
        break;
    }
    case 'l':
    { // square
        break;
    }
    case 'n':
    { // fork
        // Deceleration
        speed -= 50;
        speed = speed < 50 ? 50 : speed;
        break;
    }
    case 'p':
    { // circle
        break;
    }

    default:
        break;
    }
}

/*
 *  Large track chassis control function
 *  Parameters: Control command
 *  This function receives a char type parameter to determine which action to run.
 */
void jettank_control(char msg)
{
    // Define the motor speed
    // The recommended range is [50 , 200]
    static float speed = 150.0f;

    switch (msg)
    {

    case 'S':
    { // START
        // The connection is successful and the buzzer sounds once.
        // buzzer_didi() Buzzer interface
        // Parameter 1: itself
        // Parameter 2: Buzzer frequency (adjustable buzzer tone)
        // Parameter 3: ringing time in ms
        // Parameter 4: silent time (ms)
        // Parameter 5: Number of beeps
        buzzer_didi(buzzers[0], BUZZER_FREQ, 150, 200, 1);
        send_type(CHASSIS_TYPE_JETTANK);
        break;
    }

    /* Chassis motor */
    case 'I':
    { // Left Stick Middle
        // Motor stop
        chassis->stop(chassis);
        break;
    }
    case 'A':
    { // Left Stick Up
        // Moving at the linear speed of x-axis speed (i.e. moving forward)
        // set_velocity()
        // Parameter 1: itself, parameter 2: x-axis speed, parameter 3: y-axis speed, parameter 4: angular velocity of the car
        chassis->set_velocity(chassis, speed, 0, 0);
        break;
    }
    case 'B':
    { // Left stick upper left
        // With the linear speed of x-axis speed, it moves in a circle around its left side with a radius of 200mm
        // set_velocity_radius() Rotation function
        // Parameter 1: itself
        // Parameter 2: x speed
        // Parameter 3: Radius mm [positive on the left, negative on the right]
        // Parameter 4: Whether to rotate
        chassis->set_velocity_radius(chassis, speed, 200, false);
        break;
    }
    case 'C':
    { // Left Stick Left
        // Rotate left at speed
        chassis->set_velocity_radius(chassis, speed, 200, true);
        break;
    }
    case 'D':
    { // Left stick lower left
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 200mm on its left side.
        chassis->set_velocity_radius(chassis, -speed, 200, false);
        break;
    }
    case 'E':
    { // Left Stick Down
        // Moving at a linear speed of -speed on the x-axis (i.e. backwards)
        chassis->set_velocity(chassis, -speed, 0, 0);
        break;
    }
    case 'F':
    { // Left stick right down
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 200mm to the right of itself.
        chassis->set_velocity_radius(chassis, -speed, -200, false);
        break;
    }
    case 'G':
    { // Left Stick Right
        // Rotate right at speed
        chassis->set_velocity_radius(chassis, speed, -200, true);
        break;
    }
    case 'H':
    { // Left stick Up right
        // With the linear speed of x-axis speed, it moves in a circle with a radius of 200mm on its right side.
        chassis->set_velocity_radius(chassis, speed, -200, false);
        break;
    }

    /* Right button */
    case 'j':
    { // triangle
        // Acceleration
        speed += 25;
        speed = speed > 200 ? 200 : speed;
        break;
    }
    case 'l':
    { // square
        break;
    }
    case 'n':
    { // fork
        // Deceleration
        speed -= 25;
        speed = speed < 50 ? 50 : speed;
        break;
    }
    case 'p':
    { // circle
        break;
    }

    default:
        break;
    }
}

/*
 *  McLennan chassis control function
 *  Parameters: Control command
 *  This function receives a char type parameter to determine which action to run.
 */
void jetauto_control(char msg)
{
    // Define the motor speed
    // The recommended range is [50 , 450]
    static float speed = 300.0f;

    switch (msg)
    {

    case 'S':
    { // START
        // The connection is successful and the buzzer sounds once.
        // buzzer_didi() Buzzer interface
        // Parameter 1: itself
        // Parameter 2: Buzzer frequency (adjustable buzzer tone)
        // Parameter 3; ringing time in ms
        // Parameter 4: silent time (ms)
        // Parameter 5: Number of beeps
        buzzer_didi(buzzers[0], BUZZER_FREQ, 150, 200, 1);
        send_type(CHASSIS_TYPE_JETAUTO);
        break;
    }

    /* Chassis motor */
    case 'I':
    { // Left Stick Middle
        // Motor stop
        chassis->stop(chassis);
        break;
    }
    case 'A':
    { // Left Stick Up
        // Moving at the linear speed of x-axis speed (i.e. moving forward)
        // set_velocity()
        // Parameter 1: itself, parameter 2: x-axis speed, parameter 3: y-axis speed, parameter 4: angular velocity of the car
        chassis->set_velocity(chassis, speed, 0, 0);
        break;
    }
    case 'B':
    { // Left stick upper left
        // With the linear speed of x-axis speed, it moves in a circle around its left side with a radius of 500mm
        // set_velocity_radius() Rotation function
        // Parameter 1: itself
        // Parameter 2: x speed
        // Parameter 3: Radius (mm)
        // Parameter 4: Whether to rotate
        chassis->set_velocity_radius(chassis, speed, 500, false);
        break;
    }
    case 'C':
    { // Left Stick Left
        // Rotate left at speed
        chassis->set_velocity_radius(chassis, speed, 400, true);
        break;
    }
    case 'D':
    { // Left stick lower left
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 500mm on its left side.
        chassis->set_velocity_radius(chassis, -speed, 500, false);
        break;
    }
    case 'E':
    { // Left Stick Down
        // Moving at a linear speed of -speed on the x-axis (i.e. backwards)
        chassis->set_velocity(chassis, -speed, 0, 0);
        break;
    }
    case 'F':
    { // Left stick right down
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 500mm to the right of itself.
        chassis->set_velocity_radius(chassis, -speed, -500, false);
        break;
    }
    case 'G':
    { // Left Stick Right
        // Rotate right at speed
        chassis->set_velocity_radius(chassis, speed, -400, true);
        break;
    }
    case 'H':
    { // Left stick Up right
        // With the linear speed of x-axis speed, it moves in a circle with a radius of 500mm on its right side.
        chassis->set_velocity_radius(chassis, speed, -500, false);
        break;
    }

    /* Right button */
    case 'j':
    { // triangle
        // Acceleration
        speed += 50;
        speed = speed > 450 ? 450 : speed;
        break;
    }
    case 'l':
    { // square
        chassis->set_velocity(chassis, 0, speed, 0);
        break;
    }
    case 'n':
    { // fork
        // Deceleration
        speed -= 50;
        speed = speed < 50 ? 50 : speed;
        break;
    }
    case 'p':
    { // circle
        chassis->set_velocity(chassis, 0, -speed, 0);
        break;
    }

    default:
        break;
    }
}

/*
 *  Great Ackermann Chassis Control Function
 *  Parameters: Control command
 *  This function receives a char type parameter to determine which action to run.
 */
void jetacker_control(char msg)
{
    // Define the motor speed
    // The recommended range is [50 , 450]
    static float speed = 300.0f;

    switch (msg)
    {

    case 'S':
    { // START
        // The connection is successful and the buzzer sounds once.
        // buzzer_didi() Buzzer interface
        // Parameter 1: itself
        // Parameter 2: Buzzer frequency (adjustable buzzer tone)
        // Parameter 3: ringing time in ms
        // Parameter 4: silent time (ms)
        // Parameter 5: Number of beeps
        buzzer_didi(buzzers[0], BUZZER_FREQ, 150, 200, 1);
        send_type(CHASSIS_TYPE_JETACKER);
        break;
    }

    /* Chassis motor */
    case 'I':
    { // Left Stick Middle
        // Motor stop
        chassis->stop(chassis);
        break;
    }
    case 'A':
    { // Left Stick Up
        // Moving at the linear speed of x-axis speed (i.e. moving forward)
        // set_velocity()
        // Parameter 1: itself, parameter 2: x-axis speed, parameter 3: y-axis speed, parameter 4: angular velocity of the car
        chassis->set_velocity(chassis, speed, 0, 0);
        break;
    }
    case 'B':
    { // Left stick upper left
        // With the linear speed of x-axis speed, it moves in a circle around its left side with a radius of 625mm
        // set_velocity_radius() Rotation function
        // Parameter 1: itself
        // Parameter 2: x speed
        // Parameter 3: Radius mm [positive on the left, negative on the right]
        // Parameter 4: Whether to rotate (the Ackerman car is not suitable for spinning due to its own structure, so this option has no control effect)
        chassis->set_velocity_radius(chassis, speed, 625, true);
        break;
    }
    case 'C':
    { // Left Stick Left
        // With the linear speed of x-axis speed, it moves in a circle around its left side with a radius of 300mm
        chassis->set_velocity_radius(chassis, speed, 300, true);
        break;
    }
    case 'D':
    { // Left stick lower left
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 300mm on its left side.
        chassis->set_velocity_radius(chassis, -speed, 300, true);
        break;
    }
    case 'E':
    { // Left Stick Down
        // Moving at a linear speed of -speed on the x-axis (i.e. backwards)
        chassis->set_velocity(chassis, -speed, 0, 0);
        break;
    }
    case 'F':
    { // Left stick right down
        // With the linear speed of x-axis speed, it moves in the opposite direction around a circle with a radius of 300mm to the right of itself.
        chassis->set_velocity_radius(chassis, -speed, -300, false);
        break;
    }
    case 'G':
    { // Left Stick Right
        // With the linear speed of x-axis speed, it moves in a circle with a radius of 300mm on its right side.
        chassis->set_velocity_radius(chassis, speed, -300, false);
        break;
    }
    case 'H':
    { // Left stick Up right
        // With the linear speed of x-axis speed, it moves in a circle around its right side with a radius of 625mm.
        chassis->set_velocity_radius(chassis, speed, -625, false);
        break;
    }

    /* Right button */
    case 'j':
    { // triangle
        // Acceleration
        speed += 50;
        speed = speed > 450 ? 450 : speed;
        break;
    }
    case 'l':
    { // square
        break;
    }
    case 'n':
    { // fork
        // Deceleration
        speed -= 50;
        speed = speed < 50 ? 50 : speed;
        break;
    }
    case 'p':
    { // circle
        break;
    }

    default:
        break;
    }
}

void get_pwm_servo_position(char msg)
{
    static char msgs[5] = "";
    static uint32_t index = 0;
    static uint32_t step = 0;
    static uint32_t posi = 1500;
    int rec_num = 0;

    switch (step)
    {
    case 0:
        if ((char)msg == '$')
        {
            index = 0;
            step = 1;
        }
        break;

    case 1:
        if ((char)msg != '$')
        {
            msgs[index] = (char)msg;
            index++;
            if (index > 3) // Beyond normal value
            {
                step = 0;
            }
        }
        else
        {
            // Parsing Numerical Values
            if (msgs[0] == '-') // Negative Values
            {
                rec_num = index == 2 ? -(msgs[1] - 48) : -((msgs[1] - 48) * 10 + (msgs[2] - 48));
            }
            else
            {
                rec_num = index == 1 ? (msgs[0] - 48) : (msgs[0] - 48) * 10 + (msgs[1] - 48);
            }
            // Calculate pwm servo position (1500 midpoint)
            posi = MINACKER_PWM_DEV * rec_num + 1500;
            pwm_servo_set_position(pwm_servos[0], posi, 120);
            step = 0;
        }
        break;

    default:
        step = 0;
        break;
    }
}

/*
 *  Little Ackermann chassis control function
 *  Parameters: Control command
 *  This function receives a char type parameter to determine which action to run.
 */
void minacker_control(char msg)
{
    // Define the motor speed
    // The recommended range is [50 , 300]
    static float speed = 200.0f;
    static uint32_t posi = 1500;

    get_pwm_servo_position(msg);

    switch (msg)
    {
    case 'S':
    { // START
        // The connection is successful and the buzzer sounds once.
        // buzzer_didi() Buzzer interface
        // Parameter 1: itself
        // Parameter 2: Buzzer frequency (adjustable buzzer tone)
        // Parameter 3; ringing time in ms
        // Parameter 4: silent time (ms)
        // Parameter 5: Number of beeps
        buzzer_didi(buzzers[0], BUZZER_FREQ, 150, 200, 1);
        send_type(CHASSIS_TYPE_MINACKER);
        break;
    }

    /* Chassis motor */
    case 'I':
    { // Left Stick Middle
        // Motor stop
        chassis->stop(chassis);
        break;
    }
    case 'A':
    { // Left Stick Up
        // Moving at the linear speed of x-axis speed (i.e. moving forward)
        // set_velocity()
        // Parameter 1: itself, parameter 2: x-axis speed, parameter 3: y-axis speed, parameter 4: angular velocity of the car
        chassis->set_velocity(chassis, speed, 0, 0);
        break;
    }
    case 'B':
    { // Left stick upper left
        chassis->set_velocity(chassis, speed, 0, 0);
        break;
    }
    case 'C':
    { // Left Stick Left

        break;
    }
    case 'D':
    { // Left stick lower left
        chassis->set_velocity(chassis, -speed, 0, 0);
        break;
    }
    case 'E':
    { // Left Stick Down
        // Moving at a linear speed of -speed on the x-axis (i.e. backwards)
        chassis->set_velocity(chassis, -speed, 0, 0);
        break;
    }
    case 'F':
    { // Left stick right down
        chassis->set_velocity(chassis, -speed, 0, 0);
        break;
    }
    case 'G':
    { // Left Stick Right

        break;
    }
    case 'H':
    { // Left stick Up right
        chassis->set_velocity(chassis, speed, 0, 0);
        break;
    }

    /* Right button */
    case 'j':
    { // triangle
        // Acceleration
        speed += 50;
        speed = speed > 300 ? 300 : speed;
        break;
    }
    case 'l':
    { // square
        break;
    }
    case 'n':
    { // fork
        // Deceleration
        speed -= 50;
        speed = speed < 50 ? 50 : speed;
        break;
    }
    case 'p':
    { // circle
        break;
    }

    /* Gimbal Servo */
    case 'R':
    { // Right Stick Middle
        posi = 1500;
        pwm_servo_set_position(pwm_servos[0], posi, 120);
        break;
    }
    case 'J':
    { // Right Stick Up

        break;
    }
    case 'K':
    { // Right stick upper left
        break;
    }
    case 'L':
    { // Right Stick Left
        // Set the gimbal lower servo to the original angle -50
        posi -= 100;
        posi = (posi <= 800) ? 800 : posi;
        pwm_servo_set_position(pwm_servos[0], posi, 120);
        break;
    }
    case 'M':
    { // Right stick Down left
        break;
    }
    case 'N':
    { // Right Stick Down

        break;
    }
    case 'O':
    { // Right stick right down
        break;
    }
    case 'P':
    { // Right Stick Right
        // Adjust the servo angle of the gimbal to +50
        posi += 100;
        posi = (posi >= 2200) ? 2200 : posi;
        pwm_servo_set_position(pwm_servos[0], posi, 120);
        break;
    }
    case 'Q':
    { // Right stick upper right
        break;
    }
    case 'f':
    { // R3 (right stick pressed)
        // Calculate the difference between the upper servo and the reset angle
        uint16_t run_time = posi > 1500 ? (posi - 1500) : (1500 - posi);
        // Calculate the rotation time
        run_time >>= 1; // The time value of the movement is set to 1/2 of the angle difference
        posi = 1500;
        // Set the servo rotation parameters
        pwm_servo_set_position(pwm_servos[0], posi, run_time);
    }

    default:
        break;
    }
}

// PTZ control function
void holder_control(char msg, uint8_t servo_1, uint8_t servo_2)
{
    // Define the movement angle of the gimbal servo
    // The initial angle value of the servo is 1500 (i.e. 90°), and the range is [500, 2500] (i.e. [0°, 180°]). The macro definition is [HOLDER_MIN, HOLDER_MAX]
    static int16_t angle_1 = 1500, angle_2 = 1500;

    switch (msg)
    {
    /* Gimbal Servo */
    case 'R':
    { // Right Stick Middle
        break;
    }
    case 'J':
    { // Right Stick Up
        // Set the gimbal servo to the original angle +50
        angle_1 += 100;
        // Determine whether overflow
        angle_1 = (angle_1 >= HOLDER_MAX) ? HOLDER_MAX : angle_1;
        if (servo_1 < 3)
        {
            // Set the servo angle and rotation time
            // pwm_servo_set_position()  Set the servo angle and rotation time
            // Parameter 1: Servo object
            // Parameter 2: Specified angle (angle value range is [500, 2500])
            // Parameter 3: Rotation time
            pwm_servo_set_position(pwm_servos[servo_1], angle_1, 120);
        }
        break;
    }
    case 'K':
    { // Right stick upper left
        break;
    }
    case 'L':
    { // Right Stick Left
        // Adjust the servo angle of the gimbal to +50
        angle_2 += 100;
        angle_2 = (angle_2 >= HOLDER_MAX) ? HOLDER_MAX : angle_2;
        if (servo_2 < 3)
        {
            pwm_servo_set_position(pwm_servos[servo_2], angle_2, 120);
        }
        break;
    }
    case 'M':
    { // Right stick Down left
        break;
    }
    case 'N':
    { // Right Stick Down
        // Set the gimbal servo to the original angle -50
        angle_1 -= 100;
        angle_1 = (angle_1 <= HOLDER_MIN) ? HOLDER_MIN : angle_1;
        if (servo_1 < 3)
        {
            pwm_servo_set_position(pwm_servos[servo_1], angle_1, 120);
        }
        break;
    }
    case 'O':
    { // Right stick right down
        break;
    }
    case 'P':
    { // Right Stick Right
        // Set the gimbal lower servo to the original angle -50
        angle_2 -= 100;
        angle_2 = (angle_2 <= HOLDER_MIN) ? HOLDER_MIN : angle_2;
        if (servo_2 < 3)
        {
            pwm_servo_set_position(pwm_servos[servo_2], angle_2, 120);
        }
        break;
    }
    case 'Q':
    { // Right stick upper right
        break;
    }
    case 'f':
    { // R3 (right stick pressed)
        // Calculate the difference between the upper servo and the reset angle
        uint16_t run_time = angle_1 > 1500 ? (angle_1 - 1500) : (1500 - angle_1);
        // Calculate the rotation time
        run_time >>= 1; // The time value of the movement is set to 1/2 of the angle difference
        angle_1 = 1500;
        // Set the servo rotation parameters
        if (servo_1 < 4)
        {
            pwm_servo_set_position(pwm_servos[servo_1], angle_1, run_time);
        }
        // Lower the servo and repeat the above process
        run_time = angle_2 > 1500 ? (angle_2 - 1500) : (1500 - angle_2);
        run_time >>= 1;
        angle_2 = 1500;
        if (servo_2 < 4)
        {
            pwm_servo_set_position(pwm_servos[servo_2], angle_2, run_time);
        }
    }

    default:
        break;
    }
}

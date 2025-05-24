#ifndef __GLOBAL_CONF_H
#define __GLOBAL_CONF_H

#define ENABLE_DEBUG_UART    1			  /* Whether to use the serial port to print logs, 1: serial port output, 0: JLink output */
#define DEBUG_UART_REFERENCE &huart4

#define ENABLE_IMU  0                     /* IMU task enabled */
#define ENABLE_LVGL 0                     /* LVGL task enabled */
#define ENABLE_SBUS 1                     /* SBUS task enabled */
#define ENABLE_BLUETOOTH                0 /* Bluetooth enabled */
#define ENABLE_BLUETOOTH_BATTERY_REPORT 1 /* Bluetooth voltage reporting enabled */
#define ENABLE_BATTERY_LOW_ALARM        1 /* Low voltage alarm enabled */


#define BATTERY_LOW_ALARM_THRESHOLD 6300  /* Low voltage alarm threshold, in millivolts */
//#define BATTERY_LOW_ALARM_THRESHOLD 9500


#define KEY1_PUSHED_LEVEL 0
#define KEY2_PUSHED_LEVEL 0
#define LED_SYS_LEVEL_ON  0

#define LED_TASK_PERIOD     30u /* LED status refresh interval */
#define BUZZER_TASK_PERIOD  30u /* Buzzer status refresh interval */
#define BUTTON_TASK_PERIOD  30u /* Onboard key scan interval */
#define BATTERY_TASK_PERIOD 50u /* Battery level detection interval */
#define IMU_TASK_PERIOD 500u /* IMU reading interval */

#endif


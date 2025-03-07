#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbh_hid_gamepad.h"
#include "cmsis_os2.h"

#define ATC_THRESHOLD 10

static char A_T_C(int8_t analog_x, int8_t analog_y);

static void A_T_C_2(int8_t analog_x, int8_t analog_y, char* msgs);


void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    extern osMessageQueueId_t moving_ctrl_queueHandle_2;
    static HID_GAMEPAD_Info_TypeDef last_info;
    static char last_direction_msg = 'I';
    static char last_button = 'R';
	static GamePadMessage gp_message;

    switch(USBH_HID_GetDeviceType(phost)) {
        case 0xFF: {/* Controller data */
            HID_GAMEPAD_Info_TypeDef *info = USBH_HID_GetGamepadInfo(phost);
            //printf("Gamepad info => LX: %d, LY: %d, RX: %d, RY: %d \n", info->lx, info->ly, info->rx, info->ry);
            if(info == NULL) {
                break;
            }
       
            gp_message.lx = info->lx;
            gp_message.ly = info->ly;
            gp_message.rx = info->rx;
            gp_message.ry = info->ry;
            gp_message.msg = ' ';
            if(info->hat & 0x08) {
                gp_message.msg = (((info->hat & 0x07) + 1) & 0x07) + 0x41;
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            } else {
                if(last_info.hat != info->hat) { /* Send a release signal only once when releasing */
                    gp_message.msg = 'I';
                    last_direction_msg = 'I';
                    osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
                } else {
                    gp_message.msg = A_T_C(info->lx, info->ly);
                    if(gp_message.msg != 'I') {
                        //gp_message.msg = gp_message.msg - 'A' + 'J';
                        osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
                    } else {
                        if(last_direction_msg != 'I') { /* Send a release signal only once when releasing */
                            osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
                        }
                    }
                    last_direction_msg = gp_message.msg;
                }
            }
            char button = A_T_C(info->rx, info->ry);
            if((last_button != 'I' && button == 'I') || button != 'I') {
              gp_message.msg = button - 'A' + 'J';
              osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            last_button = button;
            if(GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_CIRCLE)) {
							  gp_message.msg = 'p';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_SQUARE)) {
							  gp_message.msg = 'l';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            /** Triggered when pressed */
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_TRIANGLE) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_TRIANGLE)) {
							  gp_message.msg = 'j';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }

            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_CROSS) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_CROSS)) {
							  gp_message.msg = 'n';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_SELECT) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_SELECT)) {
							  gp_message.msg = 'T';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_START) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_START)) {
							  gp_message.msg = 'S';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_L1) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_L1)) {
							  gp_message.msg = 'a';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_L2) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_L2)) {
							  gp_message.msg = 'b';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_L3) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_L3)) {
							  gp_message.msg = 'c';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_R1) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_R1)) {
							  gp_message.msg = 'd';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_R2) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_R2)) {
							  gp_message.msg = 'e';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }
            if(!GAMEPAD_GET_BUTTON(&last_info, GAMEPAD_BUTTON_MASK_R3) && GAMEPAD_GET_BUTTON(info, GAMEPAD_BUTTON_MASK_R3)) {
							  gp_message.msg = 'f';
                osMessageQueuePut(moving_ctrl_queueHandle_2, &gp_message, 0, 0);
            }

            memcpy(&last_info, info, sizeof(HID_GAMEPAD_Info_TypeDef));
            break;
        }
        default:
            break;
    }
}


static char A_T_C(int8_t analog_x, int8_t analog_y)
{
    char result = ' ';
    if(analog_x < -ATC_THRESHOLD) {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'D';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'B';
        } else {
            result = 'C';
        }
    } else if(analog_x > ATC_THRESHOLD) {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'F';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'H';
        } else {
            result = 'G';
        }
    } else {
        if(analog_y < -ATC_THRESHOLD) {
            result = 'E';
        } else if(analog_y > ATC_THRESHOLD) {
            result = 'A';
        } else {
            result = 'I';
        }
    }
    return result;
}

static void A_T_C_2(int8_t analog_x, int8_t analog_y, char* msgs)
{
    uint8_t index = 0;
    if(analog_x < -ATC_THRESHOLD) {
        msgs[index++] = 'L';
    } else if(analog_x > ATC_THRESHOLD) {
        msgs[index++] = 'P';
    }
    
    if(analog_y < -ATC_THRESHOLD) {
        msgs[index++] = 'E';
    } else if(analog_y > ATC_THRESHOLD) {
        msgs[index++] = 'A';
    }
    
    if (index == 0) {
        msgs[index++] = 'I';
    }
    
}


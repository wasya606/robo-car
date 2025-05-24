#include "u8g2_porting.h"
#include "i2c.h"
#include "cmsis_os2.h"
#include "lwmem_porting.h"

/* Global variable */
u8g2_t *u8g2;

/**
 * @brief OLED screen driver interface
 * @details Implements writing display content to the OLED screen
 * @param u8x8 Screen instance object
 * @param msg Operation to be performed
 * @param arg_int Operation parameter
 * @param arg_ptr Operation parameter pointer
 */
static uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
    static uint8_t buffer[64];
    static uint8_t buf_idx;
    uint8_t *data;

    switch (msg) {
        case U8X8_MSG_BYTE_INIT: {
            /* Add your custom code to initialize the I2C subsystem */
            // MX_I2C2_Init(); // I2C initialization. I2C has already been initialized, no need to initialize again
            break;
        }
        case U8X8_MSG_BYTE_START_TRANSFER: {
            buf_idx = 0;
            break;
        }
        case U8X8_MSG_BYTE_SEND: {
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0) {
                buffer[buf_idx++] = *data;
                data++;
                arg_int--;
            }
            break;
        }
        case U8X8_MSG_BYTE_END_TRANSFER: {
            if (HAL_I2C_Master_Transmit(&hi2c2, OLED_ADDRESS, buffer, buf_idx, 0xFFFF) != HAL_OK) {
                return 0;
            }
            break;
        }
        case U8X8_MSG_BYTE_SET_DC:
            break;
        default:
            return 0;
    }
    return 1;
}

/**
 * @brief Provides delay and GPIO operation interfaces for the u8g2 library
 * @param u8x8 Pointer to the screen instance object
 * @param msg Operation to be performed
 * @param arg_int Operation parameter
 * @param arg_ptr Operation parameter pointer
 */
static uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg) {
        case U8X8_MSG_DELAY_100NANO: // Delay arg_int * 100 nanoseconds
            __NOP();
            break;
        case U8X8_MSG_DELAY_10MICRO: // Delay arg_int * 10 microseconds
            osDelay(10);
            break;
        case U8X8_MSG_DELAY_MILLI: // Delay arg_int * 1 millisecond
            osDelay(1);
            break;
        case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g., 4 = 400 KHz
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            break;                    // arg_int=1: Delay by 5us, arg_int=4: Delay by 1.25us
        case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
            break;                    // arg_int=1: Input direction with pull-up high for I2C clock pin
        case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
            break;                    // arg_int=1: Input direction with pull-up high for I2C data pin
        case U8X8_MSG_GPIO_MENU_SELECT:
            u8x8_SetGPIOResult(u8x8, /* Get menu select pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_NEXT:
            u8x8_SetGPIOResult(u8x8, /* Get menu next pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_PREV:
            u8x8_SetGPIOResult(u8x8, /* Get menu prev pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_HOME:
            u8x8_SetGPIOResult(u8x8, /* Get menu home pin state */ 0);
            break;
        default:
            u8x8_SetGPIOResult(u8x8, 1); // Default return value
            break;
    }
    return 1;
}

/**
 * @brief u8g2 initialization
 * @details Completes the initialization of the u8g2 object, registers the related driver interfaces, and initializes the screen
 * @retval None.
 */
void u8g2_init()
{
    u8g2 = LWMEM_CCM_MALLOC(sizeof(u8g2_t));

    // U8G2_R0: Default to U8G2_R0 (used to configure whether the screen needs to be rotated)
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay); // Initialize the u8g2 structure using hardware I2C
    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
    u8g2_ClearBuffer(u8g2);
}

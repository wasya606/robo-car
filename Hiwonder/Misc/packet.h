/**
* @file protocol.h
* @brief Communication protocol handling with the host computer
* @author LuYongPing
* @date 20 MAY 2023
*/

#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_

#include <stdint.h>
#include <cmsis_os2.h>
#include "lwrb.h"
#include "lwmem.h"

#define PROTO_CONST_STARTBYTE1 0xAAu
#define PROTO_CONST_STARTBYTE2 0x55u
#define PACKET_PARSE_BUFFER_SIZE 64

#pragma pack(1)
struct PacketRawFrame {
    uint8_t start_byte1;
    uint8_t start_byte2;
    uint8_t function;
    uint8_t data_length;
    uint8_t data_and_checksum[257];
};
#pragma pack()

/**
 * @brief Parser state machine state enumeration
 *
 */
enum PacketControllerState {
    PACKET_CONTROLLER_STATE_STARTBYTE1, /**< Searching for frame header marker 1 */
    PACKET_CONTROLLER_STATE_STARTBYTE2, /**< Searching for frame header marker 2 */
    PACKET_CONTROLLER_STATE_FUNCTION, /**< Processing frame function number */
    PACKET_CONTROLLER_STATE_LENGTH, /**< Processing frame length */
    PACKET_CONTROLLER_STATE_DATA, /**< Processing frame data */
    PACKET_CONTROLLER_STATE_CHECKSUM, /**< Processing data checksum */
};

/**
 * @brief Frame function number enumeration
 *
 */
enum PACKET_FUNCTION {
    PACKET_FUNC_SYS = 0,
    PACKET_FUNC_LED,
    PACKET_FUNC_BUZZER,
    PACKET_FUNC_MOTOR,
    PACKET_FUNC_PWM_SERVO,
    PACKET_FUNC_BUS_SERVO,
    PACKET_FUNC_KEY,
    PACKET_FUNC_IMU,
    PACKET_FUNC_GAMEPAD,
    PACKET_FUNC_SBUS,
    PACKET_FUNC_NONE,
};

typedef void(*packet_handle)(struct PacketRawFrame *);

/**
 * @brief Protocol parser
 * @details Protocol parser, storing parser working state, state machine state, etc.
 */
struct PacketController {
    enum PacketControllerState state;        /**< Current state of the parser state machine */
    struct PacketRawFrame frame;             /**< Frame currently being processed by the parser */
    packet_handle handles[PACKET_FUNC_NONE]; /**< List of parsing operations */
    int data_index;

    uint8_t *rx_dma_buffers[2]; /**< DMA buffer list */
    size_t rx_dma_buffer_size; /**< Size of a single DMA buffer */
    volatile int rx_dma_buffer_index;    /**< Index number of the DMA buffer currently being received */

    uint8_t *rx_fifo_buffer;    /**< Receive FIFO buffer */
    lwrb_t *rx_fifo;            /**< Receive buffer FIFO object */

    int (*send_packet)(struct PacketController *self, struct PacketRawFrame *frame);
    struct PacketRawFrame* tx_dma_buffer; /**< DMA buffer being sent */
};

/**
 * @brief Serial command callback registration
 * @param self Protocol instance
 * @param func Function ID
 * @param handle Callback function
 * @retval None
*/
void packet_register_callback(struct PacketController *self, enum PACKET_FUNCTION func, packet_handle p);

/**
 * @brief Serial protocol receive processing
 * @details Pass in data, switch the state of the state machine based on the state and data of the state machine, 
 *          and complete the reception and parsing of the protocol.
 *          The parsed data frame will be pushed into the receive frame queue.
 * @param self Protocol instance
 * @retval None
 */
void packet_recv(struct PacketController *self);

/**
 * @brief Serial protocol send processing
 *
 * @param self Protocol instance
 * @param func Function number
 * @param data Data segment
 * @param data_len Length of the data segment
 * @retval  ==0 Success
 * @retval  !=0 Failure
 */
int packet_transmit(struct PacketController *self, uint8_t func, void* data, size_t data_len);

#endif

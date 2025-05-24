#include "packet.h"
#include "string.h"
#include "lwmem.h"
#include "checksum.h"
#include "cmsis_os2.h"
#include "global.h"
#include "lwmem_porting.h"

/**
    * @brief Serial command callback registration
    * @param self Protocol instance
    * @param func Function ID
    * @param handle Callback function
    * @retval None
*/
void packet_register_callback(struct PacketController *self, enum PACKET_FUNCTION func, packet_handle p)
{
    if(func < PACKET_FUNC_NONE) {
        self->handles[func] = p;
    }
}

/**
 * @brief Serial protocol parsing
 * @details Pass in data, switch the state of the state machine based on the state and data of the state machine, 
 *          and complete the reception and parsing of the protocol.
 *          The parsed data frame will be pushed into the receive frame queue.
 * @param self Protocol instance
 * @param data Data to parse
 * @param length Length of the data
 * @retval None
 */

void packet_recv(struct PacketController *self)
{
    uint8_t data[PACKET_PARSE_BUFFER_SIZE];
    size_t avaliable = lwrb_get_full(self->rx_fifo);
    size_t readed_len = 0;
    uint8_t crc = 0;
    while(avaliable > 0) {
        avaliable = avaliable > PACKET_PARSE_BUFFER_SIZE ? PACKET_PARSE_BUFFER_SIZE : avaliable;
        readed_len = lwrb_read(self->rx_fifo, data, avaliable);

        for(int i = 0; i < readed_len; ++i) {
            //printf("%0.2X ", data[i]);
            switch(self->state) {
                case PACKET_CONTROLLER_STATE_STARTBYTE1: /* Process frame header marker 1 */
                    self->state = PROTO_CONST_STARTBYTE1 == data[i] ? PACKET_CONTROLLER_STATE_STARTBYTE2 : PACKET_CONTROLLER_STATE_STARTBYTE1;
                    break;
                case PACKET_CONTROLLER_STATE_STARTBYTE2: /* Process frame header marker 2 */
                    self->state = PROTO_CONST_STARTBYTE2 == data[i] ? PACKET_CONTROLLER_STATE_FUNCTION : PACKET_CONTROLLER_STATE_STARTBYTE1;
                    break;
                case PACKET_CONTROLLER_STATE_FUNCTION: /* Process frame function number */
                    self->state = PACKET_FUNC_NONE > data[i] ? PACKET_CONTROLLER_STATE_LENGTH : PACKET_CONTROLLER_STATE_STARTBYTE1;
                    if(PACKET_CONTROLLER_STATE_LENGTH == self->state) {
                        self->frame.function = data[i];
                    }
                    break;
                case PACKET_CONTROLLER_STATE_LENGTH: /* Process frame data length */
                    self->frame.data_length = data[i];
                    self->state = (0 == self->frame.data_length) ? PACKET_CONTROLLER_STATE_CHECKSUM : PACKET_CONTROLLER_STATE_DATA;
                    self->data_index = 0;
                    break;
                case PACKET_CONTROLLER_STATE_DATA: /* Process frame data */
                    self->frame.data_and_checksum[self->data_index] = data[i];
                    ++self->data_index;
                    if(self->data_index >= self->frame.data_length) {
                        self->state = PACKET_CONTROLLER_STATE_CHECKSUM;
                    }
                    break;
                case PACKET_CONTROLLER_STATE_CHECKSUM: /* Process checksum */
                    self->frame.data_and_checksum[self->frame.data_length] = data[i];
                    crc = checksum_crc8((uint8_t*)&self->frame.function, self->frame.data_length + 2);
                    if(crc == self->frame.data_and_checksum[self->frame.data_length]) { /* If verification fails, skip execution */
                        if(NULL != self->handles[self->frame.function]) {
                            self->handles[self->frame.function](&self->frame);
                        }
                    }
                    memset(&self->frame, 0, sizeof(struct PacketRawFrame));
                    self->state = PACKET_CONTROLLER_STATE_STARTBYTE1;
                    break;
                default:
                    while(1);
            }
        }
        avaliable = lwrb_get_full(self->rx_fifo);
    }
}

struct PacketRawFrame* packet_serialize(uint8_t func, void* data_raw, size_t data_len)
{
    struct PacketRawFrame *p =  LWMEM_RAM_MALLOC(data_len + 6);
    if(NULL != p) {
        p->start_byte1 = PROTO_CONST_STARTBYTE1;
        p->start_byte2 = PROTO_CONST_STARTBYTE2;
        p->data_length = data_len;
        p->function = func;
        memcpy(p->data_and_checksum, data_raw, data_len);
        uint8_t checksum = checksum_crc8(&p->function, data_len + 2);
        p->data_and_checksum[data_len] = checksum;
    }
    return p;
}


int packet_transmit(struct PacketController *self, uint8_t func, void* data, size_t data_len)
{
    struct PacketRawFrame *p = packet_serialize(func, data, data_len);
    if(NULL != p) {
        return self->send_packet(self, p);
    }
    return -2;
}

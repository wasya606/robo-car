/**
 * @file packet_portting.c
 * @brief UART protocol interface implementation
 * @version 0.1
 * @date 2023-05-23
 * 
 */

 #include "global.h"
 #include "lwrb.h"
 #include "usart.h"
 #include "packet.h"
 #include "cmsis_os2.h"
 #include <stdio.h>
 #include "lwmem_porting.h"
 
 #define PACKET_RX_FIFO_BUFFER_SIZE 2048 /* FIFO buffer length */
 #define PACKET_RX_DMA_BUFFER_SIZE 256 /* Single DMA buffer length */
 
 /* Exposed variables */
 struct PacketController packet_controller; /* Protocol controller instance */
 
 /* Internal functions */
 static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length);
 static void packet_dma_transmit_finished(UART_HandleTypeDef *huart);
 static int send_packet(struct PacketController *self, struct PacketRawFrame *frame);
 
 /* External variables required for UART control */
 extern osSemaphoreId_t packet_tx_idleHandle;
 extern osSemaphoreId_t packet_rx_not_emptyHandle;
 extern osMessageQueueId_t packet_tx_queueHandle;
 
 /**
  * @brief Initializes the packet controller object
  * @retval void
  */
 void packet_init(void)
 {
     memset(&packet_controller, 0, sizeof(packet_controller));
     packet_controller.state = PACKET_CONTROLLER_STATE_STARTBYTE1;
     packet_controller.data_index = 0;
 
     /* DMA receive buffer initialization */
         static uint8_t rx_dma_buffer1[PACKET_RX_DMA_BUFFER_SIZE];
         static uint8_t rx_dma_buffer2[PACKET_RX_DMA_BUFFER_SIZE];
 
     packet_controller.rx_dma_buffers[0] = rx_dma_buffer1;
     packet_controller.rx_dma_buffers[1] = rx_dma_buffer2;
     packet_controller.rx_dma_buffer_size = PACKET_RX_DMA_BUFFER_SIZE;
     packet_controller.rx_dma_buffer_index = 0;
 
     /* Receive FIFO initialization */
     packet_controller.rx_fifo_buffer  = LWMEM_CCM_MALLOC(PACKET_RX_FIFO_BUFFER_SIZE);
     packet_controller.rx_fifo = LWMEM_CCM_MALLOC(sizeof(lwrb_t)); 
     lwrb_init(packet_controller.rx_fifo, packet_controller.rx_fifo_buffer, PACKET_RX_FIFO_BUFFER_SIZE);
 
     /* Send interface */
     packet_controller.send_packet = send_packet;
 }
 
 /**
  * @brief Send interface
  * @details Does not send directly; instead, pushes the pointer of the frame to be sent into the send queue, 
  *          waiting for the sending task to operate hardware for completion.
  *
  * @param self  Protocol controller instance
  * @param frame  Frame to send
  * @return int
  */
 static int send_packet(struct PacketController *self, struct PacketRawFrame *frame)
 {
     return osMessageQueuePut(packet_tx_queueHandle, &frame, 0, 10);
 }
 
 /**
  * @brief Starts UART protocol reception
  * @retval void
  */
 void packet_start_recv(void)
 {
     HAL_UART_RegisterRxEventCallback(&huart3, packet_dma_receive_event_callback); /* Register receive event callback */
     /* Use ReceiveToIdle_DMA for reception. This function interrupts when the DMA buffer is full or when reception is idle, 
        triggering the receive event callback. */
     HAL_UARTEx_ReceiveToIdle_DMA(&huart3, packet_controller.rx_dma_buffers[packet_controller.rx_dma_buffer_index], PACKET_RX_DMA_BUFFER_SIZE); /* Start reception */
 }
 
 
 /**
  * @brief UART protocol packet reception event callback
  * Triggered after the DMA receive buffer is full or idle, this function pushes the received data into the receive FIFO buffer, 
  * which is then parsed and processed by the reception task.
  * @param huart UART instance
  * @param Pos Number of bytes received
  * @retval void
  */
 static void packet_dma_receive_event_callback(UART_HandleTypeDef *huart, uint16_t length)
 {
     printf("recv_len:%d\r\n", length);
     int cur_index = packet_controller.rx_dma_buffer_index; /* Get the current DMA buffer index number */
     packet_controller.rx_dma_buffer_index ^= 1;
     if(length < PACKET_RX_DMA_BUFFER_SIZE) {
         HAL_UART_AbortReceive(&huart3);
     }
     HAL_UARTEx_ReceiveToIdle_DMA(&huart3, packet_controller.rx_dma_buffers[packet_controller.rx_dma_buffer_index], PACKET_RX_DMA_BUFFER_SIZE);
     lwrb_write(packet_controller.rx_fifo, packet_controller.rx_dma_buffers[cur_index], length); /* Write received data into FIFO ring */
     osSemaphoreRelease(packet_rx_not_emptyHandle); /* Set receive buffer non-empty signal */
 }
 
 /**
  * @brief Entry point for the UART protocol packet reception task
  * Waits for the UART buffer non-empty signal, then extracts data from the UART receive FIFO buffer for parsing and processing. 
  * The UART buffer non-empty signal is set by the UART reception event.
  * @param argument Reserved
  * @retval void
  */
 void packet_rx_task_entry1(void *argument)
 {
     osSemaphoreAcquire(packet_rx_not_emptyHandle, osWaitForever); /* Clear initial signal if not zero */
     for(;;) {
         osSemaphoreAcquire(packet_rx_not_emptyHandle, osWaitForever); /* Wait for the receive buffer to be non-empty */
         packet_recv(&packet_controller);
         printf("recv\r\n");
     }
 }
 
 
 /**
  * @brief Entry point for the UART protocol packet transmission task
  * Upon UART transmission completion, checks if the queue is empty. If not, retrieves data from the queue for another transmission. 
  * Ends the interrupt and sets the transmission idle flag when the queue is empty.
  * @param argument Reserved
  * @retval void
  */
 void packet_tx_task_entry1(void *argument)
 {
     for(;;) {
         osSemaphoreAcquire(packet_tx_idleHandle, osWaitForever); /* Wait for the transmission idle signal */
         osStatus_t status = osMessageQueueGet(packet_tx_queueHandle, &packet_controller.tx_dma_buffer, NULL, osWaitForever); /* Retrieve data from the send queue */
         if(osOK == status) {
             HAL_UART_RegisterCallback(&huart3, HAL_UART_TX_COMPLETE_CB_ID, packet_dma_transmit_finished); /* Register DMA transmission complete callback */
             HAL_UART_Transmit_DMA(&huart3, (uint8_t*)packet_controller.tx_dma_buffer, packet_controller.tx_dma_buffer->data_length + 5);  /* Trigger DMA transmission */
         }
     }
 }
 
 /**
  * @brief DMA transmission completion callback for UART protocol packets
  * @param huart UART instance
  * @retval void
  */
 static void packet_dma_transmit_finished(UART_HandleTypeDef *huart)
 {
     lwmem_free(packet_controller.tx_dma_buffer);
     osStatus_t status = osMessageQueueGet(packet_tx_queueHandle, &packet_controller.tx_dma_buffer, NULL, 0); /* Retrieve data from the send queue */
     if(osOK == status) {
         HAL_UART_RegisterCallback(&huart3, HAL_UART_TX_COMPLETE_CB_ID, packet_dma_transmit_finished); /* Register DMA reception complete callback */
         HAL_UART_Transmit_DMA(&huart3, (uint8_t*)packet_controller.tx_dma_buffer, packet_controller.tx_dma_buffer->data_length + 5); /* Trigger DMA transmission */
     } else {
         osSemaphoreRelease(packet_tx_idleHandle); /* Set transmission idle signal */
     }
 }
 
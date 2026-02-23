/**
  * @file   can.h
  * @brief  Core FDCAN library
  * 
  * This core library component is used to interface with the FDCAN hardware.
  *
  * ## Initialization
  * An FDCAN module is initialized by calling the core_CAN_init() function. The
  * CAN bitrate is set to `CORE_CAN_BITRATE`, which is given in bits/second and
  * defined in `core_config.h`. If `CORE_FDCANx_AUTO_RETRANSMISSION` is set to
  * 1 in `core_config.h`, the FDCANx module will be cofigured to automatically 
  * retransmit packets that were not acknowledged. If `CORE_FDCANx_USE_FD` is 
  * set to 1 in `core_config.h`, the FDCANx module will be able to send and 
  * receive FD CAN frames as well as standard CAN frames.
  *
  * ## Transmitting
  * Transmission on the CAN bus is managed by a FreeRTOS queue and occurs in
  * two parts. First, the user code adds a CAN frame to the queue with
  * core_CAN_add_message_to_tx_queue() (classic and FD CAN) or with 
  * core_CAN_add_extended_message_to_tx_queue() (FD CAN only). 
  *
  * The user code must also run core_CAN_send_from_tx_queue_task() in a
  * dedicated FreeRTOS task. If core_CAN_send_from_tx_queue_task(), an error
  * has occurred while transmitting.
  *
  * Internally, a FreeRTOS semaphore is used to ensure only one message sits
  * in the hardware CAN queue at a time. The semaphore is given whenever a
  * transmission completes (or fails) and is taken whenever a message is
  * added to the hardware queue.
  *
  * Alternatively, the user can disable the FreeRTOS queue by defining
  * `CORE_CAN_DISABLE_TX_QUEUE` in `core_config.h`. This is useful if the user
  * needs finer control over the transmission, wants to synchronize CAN
  * transmissions between FDCAN modules, or would like to save SRAM space.
  * The RX queue is not affected and the user can access the semaphore using
  * core_CAN_convert(), but will be responsible for taking the semaphore.
  *
  * ## Receiving
  * In order to enable receiving CAN frames, the user code must first set up
  * one or more filters using the core_CAN_add_filter() function. When a frame
  * matching any of the applied filters is received, the FDCAN hardware
  * triggers an interrupt that processes the received data.
  *
  * There are two options for processing received frames: queues or message
  * buffers. If CORE_CAN_USE_MSGBUF is not set, then one RX queue is created
  * for each initialized CAN module. When the receive interrupt is triggered,
  * the received frame is inserted into the corresponding queue. Queues are
  * preferred when data from different busses is processed separately and when
  * different busses are assigned different priorities. However, a queue can
  * only store a fixed number of elements, and a fixed amount of space will be
  * allocated for each message, regardless of the message's size.
  *
  * If CORE_CAN_USE_MSGBUF is set to 1, then up to three RX message buffers
  * can be created. Each FDCAN module can then be connected to one of the
  * three message buffers. This allows data from multiple busses to be 
  * combined into one message buffer, which is helpful if data from all busses
  * is processed identically. Furthermore, message will only take up as much
  * space in the message buffer as needed, allowing the space to be used more
  * efficiently.
  *
  * If not using message buffers, the user code must define a task that 
  * repeatedly calls core_CAN_receive_from_queue() (classic CAN) or 
  * core_CAN_receive_extended_from_queue() (FD CAN only). If using message
  * buffers, the user code must define a task that repeatedly calls
  * core_CAN_receive_from_msgbuf(). These functions will wait up to
  * CORE_CAN_RX_TIMEOUT for a message to appear in the buffer before returning.
  */

#pragma once

#include "clock.h"
#include "timestamp.h"
#include "core_config.h"
#include <stdbool.h>
#include <stdint.h>

#if (CORE_CAN_DISABLE_TX_QUEUE != 1) || (CORE_CAN_DISABLE_SEMAPHORE != 1) || (CORE_CAN_USE_MSGBUF == 1) || (CORE_CAN_DISABLE_RX_QUEUE != 1)
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "message_buffer.h"
#endif


typedef struct
{
    int id;
    int dlc;
    uint64_t data;
} CanMessage_s;

typedef struct
{
    int id;
    int dlc;
    bool use_fd;
    uint8_t data[64];
} CanExtendedMessage_s;

typedef struct core_CAN_module_s {
    FDCAN_HandleTypeDef hfdcan;         /**< @brief HAL FDCAN handle **/
#if CORE_CAN_DISABLE_RX_QUEUE != 1
    QueueHandle_t can_queue_rx;         /**< @brief Handle for FreeRTOS RX queue **/
#endif
#if CORE_CAN_DISABLE_TX_QUEUE != 1
    QueueHandle_t can_queue_tx;         /**< @brief Handle for FreeRTOS TX queue **/
#endif
#if CORE_CAN_USE_MSGBUF != 0
    MessageBufferHandle_t msgbuf;       /**< @brief Handle for FreeRTOS RX message buffer **/
#endif
#if CORE_CAN_DISABLE_SEMAPHORE != 1
    SemaphoreHandle_t can_tx_semaphore; /**< @brief TX semaphore, taken when a message is added
                                                    to the hardware FIFO and given when
                                                    transmission completes. **/
#endif
    uint32_t timestamp_msb;             /**< @brief Most significant bits of the timestamp for
                                                    the most recently received packet. **/
    uint8_t fdcan_num_standard_filters;
    uint8_t fdcan_num_extended_filters;
    uint8_t autort;
    uint8_t use_fd;
} core_CAN_module_t;

/**
  * @struct core_CAN_head_s
  * @brief  Header for a CAN packet
  */
typedef struct core_CAN_head_s {
    uint8_t type;           /**< @brief Packet type **/
    uint8_t length : 7;     /**< @brief Length of the packet not including the header **/
    uint8_t fdf : 1;        /**< @brief 1 for FD frames, 0 otherwise **/
    uint16_t timestamp;     /**< @brief Timestamp for received packets **/
    uint32_t id : 29;       /**< @brief CAN ID **/
    uint8_t rtr : 1;        /**< @brief Indicates if the frame was a remote transmission request **/
    uint8_t xtd : 1;        /**< @brief Indicates if the ID is an extended ID **/
    uint8_t esi : 1;        /**< @brief Indicates if the transmitting node is error passive **/
} core_CAN_head_t;

typedef struct core_CAN_errors_s {
    uint16_t arbitration_error;
    uint16_t data_error;
    uint16_t bus_off;
    uint16_t tx_lost;
} core_CAN_errors_t;

extern const uint8_t core_CAN_dlc_lookup[16];
extern core_CAN_errors_t core_CAN_errors;

/**
  * @brief  Initialize an FDCAN module, the RX and TX pins, the TX queue (if
  *         enabled), and the RX queue or message buffer (if enabled).
  * @retval 0 if the given FDCAN is not valid or the initialization failed
  * @retval 1 otherwise
  */
bool core_CAN_init(FDCAN_GlobalTypeDef *fdcan, uint32_t baudrate);

core_CAN_module_t *core_CAN_convert(FDCAN_GlobalTypeDef *fdcan);

#define core_CAN_enable_timestamps core_timestamp_init

/**
  * @brief  Add a CAN message to the hardware FIFO
  * @param  can FDCAN module
  * @param  id ID of the message to be transmitted. If this value is greater
  *         than 2047, then an extended ID is automatically selected. Only the
  *         lowest 29 bits are kept, so setting the MSB will force an extended
  *         ID even if the ID is less than or equal to 2047
  * @param  dlc Length of the packet (0-8)
  * @param  data Data encoded as a uint64_t
  * @retval 0 if an error occurred while adding the message to the queue
  * @retval 1 otherwise
  */
bool core_CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);

/**
  * @brief  Add an FDCAN message to the hardware FIFO
  * @param  can FDCAN module
  * @param  id ID of the message to be transmitted. If this value is greater
  *         than 2047, then an extended ID is automatically selected. Only the
  *         lowest 29 bits are kept, so setting the MSB will force an extended
  *         ID even if the ID is less than or equal to 2047
  * @param  dlc Length of the packet. If the length does not correspond to a
  *         valid FDCAN packet length, 
  * @param  data Pointer to the data to be transmitted
  * @retval 0 if an error occurred while adding the message to the queue
  * @retval 1 otherwise
  */
bool core_CAN_send_fd_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);

#if !defined(CORE_CAN_DISABLE_TX_QUEUE) || (CORE_CAN_DISABLE_TX_QUEUE == 0) || defined(DOXYGEN)
/**
  * @brief  Add a CAN frame to the TX queue. This function is only available 
  *         if CORE_CAN_DISABLE_TX_QUEUE is set to 0.
  * @param  can FDCAN module for which the frame is being enqueued
  * @param  id ID of the CAN frame
  * @param  dlc Number of data bytes in the CAN frame
  * @param  data Data bytes, encoded LSB-first as a uint64
  * @retval 0 if the queue is full
  * @retval 1 otherwise
  */
bool core_CAN_add_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);

/**
  * @brief  Add a CAN frame to the TX queue. This function is only available
  *         if CORE_CAN_DISABLE_TX_QUEUE is set to 0
  * @param  can FDCAN module for which the frame is being enqueued
  * @param  id ID of the CAN frame
  * @param  dlc Number of data bytes in the CAN frame
  * @param  data Pointer to an array of data bytes
  * @retval 0 if the queue is full, if the FDCAN is not configured for
  *         FD operation, or if `dlc` > 64
  * @retval 1 otherwise
  */
bool core_CAN_add_extended_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);

/**
  * @brief  Loop for sending data in the TX queue over CAN. This function
  *         must be run in its own task. This function is only available if
  *         CORE_CAN_DISABLE_TX_QUEUE is set to 0.
  * @param  can FDCAN module
  */
bool core_CAN_send_from_tx_queue_task(FDCAN_GlobalTypeDef *can);
#endif

#if ((CORE_CAN_USE_MSGBUF != 1) && (CORE_CAN_DISABLE_RX_QUEUE != 1)) || defined(DOXYGEN)
/**
  * @brief  If a frame is waiting in the RX queue, copy it to the given location
  *
  * If no frame is waiting in the RX queue, this function will wait up to
  * CORE_CAN_RX_TIMEOUT milliseconds for one to enter the queue. This function
  * is only defined if CORE_CAN_USE_MSGBUF and CORE_CAN_DISABLE_RX_QUEUE are
  * set to 0.
  *
  * @param  can FDCAN module from which the frame is read
  * @param  received_message Pointer to the location where the received frame
  *         would be stored
  * @retval 1 if a frame was copied from the queue into the given location
  * @retval 0 otherwise
  */
bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message);

/**
  * @brief  If a frame is waiting in the RX queue, copy it to the given location
  *
  * If no frame is waiting in the RX queue, this function will wait up to
  * CORE_CAN_RX_TIMEOUT milliseconds for one to enter the queue. This function
  * is only defined if CORE_CAN_USE_MSGBUF and CORE_CAN_DISABLE_RX_QUEUE are
  * set to 0.
  *
  * @param  can FDCAN module from which the frame is read
  * @param  received_message Pointer to the location where the received frame
  *         would be stored
  * @retval 1 if a frame was copied from the queue into the given location
  * @retval 0 otherwise
  */
bool core_CAN_receive_extended_from_queue(FDCAN_GlobalTypeDef *can, CanExtendedMessage_s *received_message);
#endif

#if (defined(CORE_CAN_USE_MSGBUF)) && (CORE_CAN_USE_MSGBUF != 0)
/**
  * @brief  If a frame is waiting in an RX message buffer, copy it to the given
  *         location and return its length.
  *
  * If no frame is waiting in the RX message buffer, this function will wait up
  * to the given number of milliseconds for one to enter the message buffer.
  * This function is only defined if CORE_CAN_USE_MSGBUF is set to 1
  *
  * @param  can FDCAN module from which the frame is read
  * @param  buf Buffer to which the received data should be stored
  * @retval 1 if a frame was copied from the queue into the given location
  * @retval 0 otherwise
  */
uint8_t core_CAN_receive_from_msgbuf(FDCAN_GlobalTypeDef *can, uint8_t *buf, TickType_t timeout);

/**
  * @brief  Insert a timestamped message into a message buffer
  */
BaseType_t core_CAN_msgbuf_insert_ts(core_CAN_module_t *p_can, uint8_t *buf, uint8_t buflen, uint32_t lsb);
#endif

/**
  * @brief  Add an RX filter for the given FDCAN module
  *
  * All frames with IDs greater than or equal to `id1` and less than or equal
  * to `id2` will be placed in the RX queue
  * @param  can FDCAN module for which the filter should be created
  * @param  isExtended Specifies whether the IDs are extended CAN IDs
  * @param  id1 Lower bound (inclusive)
  * @param  id2 Upper bound (inclusive)
  * @retval 1 if the filter was added successfully
  * @retval 0 otherwise
  */
bool core_CAN_add_filter(FDCAN_GlobalTypeDef *can, bool isExtended, uint32_t id1, uint32_t id2);


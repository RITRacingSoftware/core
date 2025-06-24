#pragma once

#include "clock.h"
#include "timestamp.h"
#include "core_config.h"
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "message_buffer.h"


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
    QueueHandle_t can_queue_rx;         /**< @brief Handle for FreeRTOS RX queue **/
    QueueHandle_t can_queue_tx;         /**< @brief Handle for FreeRTOS TX queue **/
    MessageBufferHandle_t msgbuf;       /**< @brief Handle for FreeRTOS RX message buffer **/
    SemaphoreHandle_t can_tx_semaphore; /**< @brief TX semaphore, taken when a message is added
                                                    to the hardware FIFO and given when
                                                    transmission completes. **/
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
} core_CAN_errors_t;

extern const uint8_t core_CAN_dlc_lookup[16];
extern core_CAN_errors_t core_CAN_errors;

bool core_CAN_init(FDCAN_GlobalTypeDef *fdcan, uint32_t baudrate);
core_CAN_module_t *core_CAN_convert(FDCAN_GlobalTypeDef *fdcan);

#define core_CAN_enable_timestamps core_timestamp_init

bool core_CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
bool core_CAN_send_fd_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);
bool core_CAN_add_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
bool core_CAN_add_extended_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);
bool core_CAN_send_from_tx_queue_task(FDCAN_GlobalTypeDef *can);

bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message);
bool core_CAN_receive_extended_from_queue(FDCAN_GlobalTypeDef *can, CanExtendedMessage_s *received_message);
uint8_t core_CAN_receive_from_msgbuf(FDCAN_GlobalTypeDef *can, uint8_t *buf);

bool core_CAN_add_filter(FDCAN_GlobalTypeDef *can, bool isExtended, uint32_t id1, uint32_t id2);

/* STATIC FUNCTIONS (DECLARED/DEFINED IN CAN.C)
static bool CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
static void rx_handler(FDCAN_GlobalTypeDef *can);
static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);
*/

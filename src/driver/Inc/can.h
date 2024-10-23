#pragma once

#include "clock.h"
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

typedef struct
{
    int id;
    int dlc;
    uint64_t data;
} CanMessage_s;

typedef struct core_CAN_module_s {
    FDCAN_HandleTypeDef hfdcan;
    QueueHandle_t can_queue_rx;
    QueueHandle_t can_queue_tx;
    SemaphoreHandle_t can_tx_semaphore;
    uint8_t fdcan_num_standard_filters;
    uint8_t fdcan_num_extended_filters;
} core_CAN_module_t;

bool core_CAN_init(FDCAN_GlobalTypeDef *fdcan);

bool core_CAN_add_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
bool core_CAN_send_from_tx_queue_task(FDCAN_GlobalTypeDef *can);

bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message);

bool core_CAN_add_filter(FDCAN_GlobalTypeDef *can, bool isExtended, uint32_t id1, uint32_t id2);

/* STATIC FUNCTIONS (DECLARED/DEFINED IN CAN.C)
static bool CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
static void rx_handler(FDCAN_GlobalTypeDef *can);
static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);
*/

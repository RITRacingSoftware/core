#pragma once

#include "clock.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int id;
    int dlc;
    uint64_t data;
} CanMessage_s;


bool core_CAN_init(FDCAN_GlobalTypeDef *fdcan);

bool core_CAN_add_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
bool core_CAN_send_from_tx_queue_task(FDCAN_GlobalTypeDef *can);
static bool CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);

static void rx_handler(FDCAN_GlobalTypeDef *can);
static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);
bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message);
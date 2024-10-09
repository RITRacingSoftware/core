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
bool core_CAN_send(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message);
//static void rx_handler(FDCAN_HandleTypeDef *can, uint32_t RxFifo0ITs);
static void rx_handler(FDCAN_HandleTypeDef *hfdcan);
static void add_CAN_message_to_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);

bool test_CAN_init();
bool test_CAN_send(uint32_t id, uint8_t dlc, uint64_t data);

void stupidShit();
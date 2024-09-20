#pragma once

#include "clock.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAN1,
    CAN2,
    CAN3
}CAN_num;

bool fake_CAN_init();
bool fake_CAN_send(uint32_t id, uint8_t dlc, uint64_t data);

bool core_CAN_init(CAN_num canNum);
bool core_CAN_send(CAN_num canNum, uint32_t id, uint8_t dlc, uint64_t data);

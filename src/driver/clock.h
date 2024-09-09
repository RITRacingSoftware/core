#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    CAN1,
    CAN2,
    CAN3
}CAN_num;

bool fake_clock_init();

bool core_clock_init();

bool core_clock_ADC12_init();
bool core_clock_FDCAN_init(CAN_num can);
bool clock_port_init(uint32_t port);
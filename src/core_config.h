#ifndef CORE_CORE_CONFIG_H
#define CORE_CORE_CONFIG_H

/*** CLOCK CONFIG PARAMETERS ***/

#define CORE_CLOCK_USE_HSE
#define CORE_CLOCK_HSE_FREQ 24000
#define CORE_CLOCK_SYSCLK_FREQ 102400


/*** CAN CONFIG PARAMETERS ***/

// Number of CAN messages that can be stored in the CAN FreeRTOS queue
#define CAN_QUEUE_LENGTH 15


// Ports and pins for CAN communication
#define CAN1_PORT GPIOA
#define CAN2_PORT GPIOB
#define CAN3_PORT GPIOB
#define CAN1_PINS (GPIO_PIN_11 | GPIO_PIN_12)
#define CAN2_PINS (GPIO_PIN_12 | GPIO_PIN_13)
#define CAN3_PINS (GPIO_PIN_3 | GPIO_PIN_4)

// Pin alternate function declaration for CAN
#define CORE_FDCAN1_AF GPIO_AF9_FDCAN1
#define CORE_FDCAN2_AF GPIO_AF9_FDCAN2
#define CORE_FDCAN3_AF GPIO_AF11_FDCAN3





#endif //CORE_CORE_CONFIG_H

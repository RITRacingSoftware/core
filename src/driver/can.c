#include "can.h"
#include "core_config.h"

#include <stdio.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "gpio.h"
#include "clock.h"
#include "error_handler.h"

static FDCAN_HandleTypeDef can1;
static FDCAN_HandleTypeDef can2;
static FDCAN_HandleTypeDef can3;

static QueueHandle_t can1_queue;
static QueueHandle_t can2_queue;
static QueueHandle_t can3_queue;

static void rx_handler(FDCAN_HandleTypeDef *hfdcan)
{

    // If the interrupt flag is set for FIFO0
    if (hfdcan->Instance->IR & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        // Reset interrupt flag for FIFO0
        hfdcan->Instance->IR = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];

        // Retrieve Rx messages from RX FIFO0
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            error_handler();
        }

        add_CAN_message_to_queue(FDCAN2, header.Identifier, header.DataLength, data);
        /*if (data[1] == 0xfa && data[0] == 0x55)
        {
            GPIO_set_heartbeat(GPIO_PIN_SET);
        }*/
//        GPIO_set_heartbeat(data[1] == 0xfa && data[0] == 0x55);

    }
    //GPIO_set_heartbeat(HAL_FDCAN_GetRxFifoFillLevel(&can2, FDCAN_RX_FIFO0) > 0);
}

void FDCAN1_IT0_IRQHandler(void) {
    rx_handler(&can1);
}
void FDCAN2_IT0_IRQHandler(void) {
    rx_handler(&can2);
}
void FDCAN3_IT0_IRQHandler(void) {
    rx_handler(&can3);
}


bool core_CAN_init(FDCAN_GlobalTypeDef *can)
{
    // Init port clock based on which CAN bus
    core_clock_FDCAN_init(can);
    FDCAN_HandleTypeDef *p_can;
    QueueHandle_t *p_canQueue;

    // Initialize pins
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;


    // GPIO inits specific to different CAN buses, and HAL GPIO inits
    if (can == FDCAN1)
    {
        p_canQueue = &can1_queue;
        p_can = &can1;
        p_can->Instance = FDCAN1;
        gpio.Pin = CAN1_PINS;
        gpio.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(CAN1_PORT, &gpio);
    }
    else if (can == FDCAN2)
    {
        p_canQueue = &can2_queue;
        p_can = &can2;
        p_can->Instance = FDCAN2;
        gpio.Pin = CAN2_PINS;
        gpio.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(CAN2_PORT, &gpio);
    }
    else if (can == FDCAN3)
    {
        p_canQueue = &can3_queue;
        p_can = &can3;
        p_can->Instance = FDCAN3;
        gpio.Pin = CAN3_PINS;
        gpio.Alternate = GPIO_AF11_FDCAN3;
        HAL_GPIO_Init(CAN3_PORT, &gpio);
    }
    else return false;

	// Initialize CAN interface
	p_can->Init.ClockDivider = FDCAN_CLOCK_DIV1;
	p_can->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	p_can->Init.Mode = FDCAN_MODE_NORMAL;
	p_can->Init.AutoRetransmission = DISABLE;
	p_can->Init.TransmitPause = DISABLE;
	p_can->Init.ProtocolException = ENABLE;
	p_can->Init.NominalPrescaler = 8;
	p_can->Init.NominalSyncJumpWidth = 1;
	p_can->Init.NominalTimeSeg1 = 12;
	p_can->Init.NominalTimeSeg2 = 2;
	p_can->Init.DataPrescaler = 1; // Data timing fields unused for classic CAN
	p_can->Init.DataSyncJumpWidth = 1;
	p_can->Init.DataTimeSeg1 = 1;
	p_can->Init.DataTimeSeg2 = 1;
	p_can->Init.StdFiltersNbr = 28;
	p_can->Init.ExtFiltersNbr = 8;
	p_can->Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;

    // Init CAN interface
    if (HAL_FDCAN_Init(p_can) != HAL_OK)
    {
        return false;
    }

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID; // vs. Extended ID
    sFilterConfig.FilterIndex = 0; // the number of the filter we are using
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // what should be done for a match with the filter
    sFilterConfig.FilterID1 = 0; // filter
    sFilterConfig.FilterID2 = 0; // mask: 0=>accept all messages
    sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;

    // Init filters
    if (HAL_FDCAN_ConfigGlobalFilter(&can2, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0) != HAL_OK)
    {
        return false;
    }


    // Set up RX interrupts
    if (can == FDCAN1) {
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0); // Main bus has slightly higher priority than sensor bus
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    } else if (can == FDCAN2) {
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0); // Main bus has slightly higher priority than sensor bus
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    } else {
        HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5, 0); // Main bus has slightly higher priority than sensor bus
        HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    }
    HAL_FDCAN_ConfigInterruptLines(p_can, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
    HAL_FDCAN_ActivateNotification(p_can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    *p_canQueue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CanMessage_s));
    if (*p_canQueue == 0) error_handler();

	// Start can interface
	if (HAL_FDCAN_Start(p_can) != HAL_OK) {
		return false;
	}

	return true;
}



bool core_CAN_send(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data)
{
    FDCAN_HandleTypeDef *p_can;

    // Which CAN bus to send through
    if (can == FDCAN1) p_can = &can1;
    else if (can == FDCAN2) p_can = &can2;
    else if (can == FDCAN3) p_can = &can3;
    else return false;


    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    header.MessageMarker = 0;

    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(p_can, &header, (uint8_t*) &data);
    return (err == HAL_OK);
}

/*static void rx_handler(FDCAN_HandleTypeDef *can, uint32_t RxFifo0ITs)
{
    error_handler();
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];

        // Retrieve Rx messages from RX FIFO0
        if (HAL_FDCAN_GetRxMessage(can, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            error_handler();
        }

//        add_CAN_message_to_queue(FDCAN2, header.Identifier, header.DataLength, data);
        if (data[3] > 0)
        {
            GPIO_set_heartbeat(GPIO_PIN_SET);
        }

    }
}*/

static void add_CAN_message_to_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data)
{
    QueueHandle_t *p_can_queue;
    if (can == FDCAN1) p_can_queue = &can1_queue;
    else if (can == FDCAN2) p_can_queue = &can2_queue;
    else p_can_queue = &can3_queue;

    uint64_t msg_data = 0;
    memcpy(&msg_data, data, dlc);

    CanMessage_s rx_msg;
    rx_msg.data = msg_data;
    rx_msg.id = id;
    rx_msg.dlc = dlc;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(*p_can_queue, &rx_msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message)
{
    QueueHandle_t *p_can_queue;

    if (can == FDCAN1) p_can_queue = &can1_queue;
    else if (can == FDCAN2) p_can_queue = &can2_queue;
    else p_can_queue = &can3_queue;


    // Receive CAN message from queue, copy it to buffer "received_message"
    if (xQueueReceive(*p_can_queue, received_message, 10))
    {
        return true;
    }

    return false;
}

bool test_CAN_init()
{
    // Initialize pins
    GPIO_InitTypeDef gpio1 = {CAN3_PINS, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF9_FDCAN2};
    HAL_GPIO_Init(GPIOB, &gpio1);

    // Initialize CAN interfaces
    can2.Instance = FDCAN3;
    can2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    can2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    can2.Init.Mode = FDCAN_MODE_NORMAL;
    can2.Init.AutoRetransmission = ENABLE;
    can2.Init.TransmitPause = DISABLE;
    can2.Init.ProtocolException = ENABLE;
    can2.Init.NominalPrescaler = 8;
    can2.Init.NominalSyncJumpWidth = 1;
    can2.Init.NominalTimeSeg1 = 12;
    can2.Init.NominalTimeSeg2 = 2;
    can2.Init.DataPrescaler = 1; // Data timing fields unused for classic CAN
    can2.Init.DataSyncJumpWidth = 1;
    can2.Init.DataTimeSeg1 = 1;
    can2.Init.DataTimeSeg2 = 1;
    can2.Init.StdFiltersNbr = 28;
    can2.Init.ExtFiltersNbr = 8;
    can2.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;

    if (HAL_FDCAN_Init(&can3) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_Start(&can3) != HAL_OK)
    {
        return false;
    }

    return true;
}
bool test_CAN_send(uint32_t id, uint8_t dlc, uint64_t data)
{
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    header.MessageMarker = 0;

    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(&can3, &header, (uint8_t*) &data);

    return (err == HAL_OK);
}

void stupidShit()
{
//    GPIO_set_heartbeat(HAL_FDCAN_GetRxFifoFillLevel(&can2, FDCAN_RX_FIFO0) > 0);
}
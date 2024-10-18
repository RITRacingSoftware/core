#include "can.h"
#include "core_config.h"

#include <stdio.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "gpio.h"
#include "clock.h"
#include "error_handler.h"

// Handlers for different CAN buses
static FDCAN_HandleTypeDef hfdcan1;
static FDCAN_HandleTypeDef hfdcan2;
static FDCAN_HandleTypeDef hfdcan3;

// Handlers for RX queues
static QueueHandle_t can1_queue_rx;
static QueueHandle_t can2_queue_rx;
static QueueHandle_t can3_queue_rx;

// Handle for TX queues
static QueueHandle_t can1_queue_tx;
static QueueHandle_t can2_queue_tx;
static QueueHandle_t can3_queue_tx;

// Number of filters for each bus
static uint8_t fdcan1_num_standard_filters = 0;
static uint8_t fdcan1_num_extended_filters = 0;
static uint8_t fdcan2_num_standard_filters = 0;
static uint8_t fdcan2_num_extended_filters = 0;
static uint8_t fdcan3_num_standard_filters = 0;
static uint8_t fdcan3_num_extended_filters = 0;

static bool CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
static void rx_handler(FDCAN_GlobalTypeDef *can);
static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);
static bool CAN_clock_set_params(FDCAN_HandleTypeDef *hfdcan);


bool core_CAN_init(FDCAN_GlobalTypeDef *can)
{
    // Init port clock based on which CAN bus
    core_clock_FDCAN_init(can);
    FDCAN_HandleTypeDef *p_hfdcan;
    QueueHandle_t *p_canQueueRx;
    QueueHandle_t *p_canQueueTx;

    // Initialize pins
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;


    // GPIO inits specific to different CAN buses, and HAL GPIO inits
    if (can == FDCAN1)
    {

        // Set RX and TX queue pointers to can1
        p_canQueueRx = &can1_queue_rx;
        p_canQueueTx = &can1_queue_tx;

        p_hfdcan = &hfdcan1;
        p_hfdcan->Instance = FDCAN1;
        gpio.Pin = CAN1_PINS;
        gpio.Alternate = CORE_FDCAN1_AF;
        HAL_GPIO_Init(CAN1_PORT, &gpio);

        // Set max filter numbers
        p_hfdcan->Init.StdFiltersNbr = CORE_FDCAN1_MAX_STANDARD_FILTER_NUM;
        p_hfdcan->Init.ExtFiltersNbr = CORE_FDCAN1_MAX_EXTENDED_FILTER_NUM;
    }
    else if (can == FDCAN2)
    {
        // Set RX and TX queue pointers
        p_canQueueRx = &can2_queue_rx;
        p_canQueueTx = &can2_queue_tx;

        p_hfdcan = &hfdcan2;
        p_hfdcan->Instance = FDCAN2;
        gpio.Pin = CAN2_PINS;
        gpio.Alternate = CORE_FDCAN2_AF;
        HAL_GPIO_Init(CAN2_PORT, &gpio);

        // Set max filter numbers
        p_hfdcan->Init.StdFiltersNbr = CORE_FDCAN2_MAX_STANDARD_FILTER_NUM;
        p_hfdcan->Init.ExtFiltersNbr = CORE_FDCAN2_MAX_EXTENDED_FILTER_NUM;
    }
    else if (can == FDCAN3)
    {
        // Set RX and TX queue pointers
        p_canQueueRx = &can3_queue_rx;
        p_canQueueTx = &can3_queue_tx;

        p_hfdcan = &hfdcan3;
        p_hfdcan->Instance = FDCAN3;
        gpio.Pin = CAN3_PINS;
        gpio.Alternate = CORE_FDCAN3_AF;
        HAL_GPIO_Init(CAN3_PORT, &gpio);

        // Set max filter numbers
        p_hfdcan->Init.StdFiltersNbr = CORE_FDCAN3_MAX_STANDARD_FILTER_NUM;
        p_hfdcan->Init.ExtFiltersNbr = CORE_FDCAN3_MAX_EXTENDED_FILTER_NUM;
    }
    else return false;

	// Initialize CAN interface
	p_hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
	p_hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	p_hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
	p_hfdcan->Init.AutoRetransmission = DISABLE;
	p_hfdcan->Init.TransmitPause = DISABLE;
	p_hfdcan->Init.ProtocolException = ENABLE;
	p_hfdcan->Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
    CAN_clock_set_params(p_hfdcan);

    // Init CAN interface
    if (HAL_FDCAN_Init(p_hfdcan) != HAL_OK)
    {
        return false;
    }

    // Reject all frames not configured in filter
    if (HAL_FDCAN_ConfigGlobalFilter(p_hfdcan, FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return false;
    }
//    if (HAL_FDCAN_ConfigGlobalFilter(p_hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,
//                                     FDCAN_ACCEPT_IN_RX_FIFO0,
//                                     FDCAN_REJECT_REMOTE,
//                                     FDCAN_REJECT_REMOTE) != HAL_OK)
//    {
//        return false;
//    }

    // Set up RX interrupts
    if (can == FDCAN1)
    {
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0); // Main bus has slightly higher priority than sensor bus
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
    else if (can == FDCAN2)
    {
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0); // Main bus has slightly higher priority than sensor bus
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    }
    else
    {
        HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5, 0); // Main bus has slightly higher priority than sensor bus
        HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    }

    // Configure interrupts and set notifications for CAN bus
    HAL_FDCAN_ConfigInterruptLines(p_hfdcan, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
    HAL_FDCAN_ActivateNotification(p_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);


    // Create queue to put received messages in
    *p_canQueueRx = xQueueCreate(CORE_CAN_QUEUE_LENGTH, sizeof(CanMessage_s));
    if (*p_canQueueRx == 0) error_handler();

    // Create queue to put outgoing messages in
    *p_canQueueTx = xQueueCreate(CORE_CAN_QUEUE_LENGTH, sizeof(CanMessage_s));
    if (*p_canQueueTx == 0) error_handler();

	// Start can interface
	if (HAL_FDCAN_Start(p_hfdcan) != HAL_OK)
    {
		return false;
	}

	return true;
}

bool core_CAN_add_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data)
{
    QueueHandle_t *p_can_queue;

    if (can == FDCAN1) p_can_queue = &can1_queue_tx;
    else if (can == FDCAN2) p_can_queue = &can2_queue_tx;
    else p_can_queue = &can3_queue_tx;

    CanMessage_s message = {(int)id, dlc, data};

    return xQueueSendToBack(*p_can_queue, &message, 0);
}

bool core_CAN_send_from_tx_queue_task(FDCAN_GlobalTypeDef *can)
{
    CanMessage_s dequeuedMessage;
    QueueHandle_t *p_can_queue;

    if (can == FDCAN1) p_can_queue = &can1_queue_tx;
    else if (can == FDCAN2) p_can_queue = &can2_queue_tx;
    else p_can_queue = &can3_queue_tx;

    while ((xQueueReceive(*p_can_queue, &dequeuedMessage, portMAX_DELAY) == pdTRUE))
    {
        CAN_send_message(can, dequeuedMessage.id, dequeuedMessage.dlc, dequeuedMessage.data);
    }

    return false;
}

static bool CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data)
{
    FDCAN_HandleTypeDef *p_hfdcan;

    // Which CAN bus to send through
    if (can == FDCAN1) p_hfdcan = &hfdcan1;
    else if (can == FDCAN2) p_hfdcan = &hfdcan2;
    else p_hfdcan = &hfdcan3;


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

    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(p_hfdcan, &header, (uint8_t*) &data);
    return err == HAL_OK;
}


static void rx_handler(FDCAN_GlobalTypeDef *can)
{
    FDCAN_HandleTypeDef *hfdcan;

    if (can == FDCAN1) hfdcan = &hfdcan1;
    else if (can == FDCAN2) hfdcan = &hfdcan2;
    else hfdcan = &hfdcan3;

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

        // Add the message to the RX queue
        add_CAN_message_to_rx_queue(can, header.Identifier, header.DataLength, data);
    }
}
static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data)
{
    QueueHandle_t *p_can_queue;
    if (can == FDCAN1) p_can_queue = &can1_queue_rx;
    else if (can == FDCAN2) p_can_queue = &can2_queue_rx;
    else p_can_queue = &can3_queue_rx;

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

    if (can == FDCAN1) p_can_queue = &can1_queue_rx;
    else if (can == FDCAN2) p_can_queue = &can2_queue_rx;
    else p_can_queue = &can3_queue_rx;

    // Receive CAN message from queue, copy it to buffer "received_message"
    // Return true if it read a value from the queue, false if not
    return (xQueueReceive(*p_can_queue, received_message, 0) == pdTRUE);
}

// Call RX interrupt handlers
void FDCAN1_IT0_IRQHandler(void) {
    GPIO_set_heartbeat(GPIO_PIN_SET);
    rx_handler(FDCAN1);
}
void FDCAN2_IT0_IRQHandler(void) {rx_handler(FDCAN2);}
void FDCAN3_IT0_IRQHandler(void) {rx_handler(FDCAN3);}

bool core_CAN_add_filter(FDCAN_GlobalTypeDef *can, bool isExtended, uint32_t id1, uint32_t id2)
{
    FDCAN_HandleTypeDef *p_hfdcan;
    uint8_t *p_num_filters;
    uint8_t max_filter_num;

    // Setup for each CAN bus, whether it's standard or extended
    if (can == FDCAN1)
    {
        p_hfdcan = &hfdcan1;
        if (isExtended)
        {
            p_num_filters = &fdcan1_num_extended_filters;
            max_filter_num = CORE_FDCAN1_MAX_EXTENDED_FILTER_NUM;
        }
        else
        {
            p_num_filters = &fdcan1_num_standard_filters;
            max_filter_num = CORE_FDCAN1_MAX_STANDARD_FILTER_NUM;
        }
    }
    else if (can == FDCAN2)
    {
        p_hfdcan = &hfdcan2;
        if (isExtended)
        {
            p_num_filters = &fdcan2_num_extended_filters;
            max_filter_num = CORE_FDCAN2_MAX_EXTENDED_FILTER_NUM;
        }
        else
        {
            p_num_filters = &fdcan2_num_standard_filters;
            max_filter_num = CORE_FDCAN2_MAX_STANDARD_FILTER_NUM;
        }
    }
    else
    {
        p_hfdcan = &hfdcan3;
        if (isExtended)
        {
            p_num_filters = &fdcan3_num_extended_filters;
            max_filter_num = CORE_FDCAN3_MAX_EXTENDED_FILTER_NUM;
        }
        else
        {
            p_num_filters = &fdcan3_num_standard_filters;
            max_filter_num = CORE_FDCAN3_MAX_STANDARD_FILTER_NUM;
        }
    }

    if (*p_num_filters + 1 >  max_filter_num) return false;

    FDCAN_FilterTypeDef filter;
    filter.IdType = isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterIndex = *p_num_filters++;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = id1;
    filter.FilterID2 = id2;

    return HAL_FDCAN_ConfigFilter(p_hfdcan, &filter) == HAL_OK;

}

static bool CAN_clock_set_params(FDCAN_HandleTypeDef *hfdcan)
{
    int nominal_bit_time = 1/(CORE_CAN_BITRATE);

    hfdcan->Init.NominalPrescaler = 8;
    hfdcan->Init.NominalSyncJumpWidth = 1;
    hfdcan->Init.DataPrescaler = 1; // Data timing fields unused for classic CAN
    hfdcan->Init.DataSyncJumpWidth = 1;
    hfdcan->Init.DataTimeSeg1 = 1;
    hfdcan->Init.DataTimeSeg2 = 1;


    // CAN BitRate = SysClk/
    //              (APB1ClockDivider * NominalPrescaler * (1 + NominalTimeSeg1 + NominalTimeSeg2))
    double ns_sum = (((double)CORE_CLOCK_SYSCLK_FREQ * 1000)/(double)(CORE_CAN_BITRATE * 1 * 8)) - 1;

    // Sum of both time segments is not an integer
    if (floor(ns_sum) != ns_sum) return false;

    // Make NominalTimeSeg1 ~75% of the sum of it and NominalTimeSeg2.
    uint8_t seg1 = round(ns_sum * 0.75);
    uint8_t seg2 = ns_sum - seg1;

    hfdcan->Init.NominalTimeSeg1 = seg1;
    hfdcan->Init.NominalTimeSeg2 = seg2;
    return true;
}
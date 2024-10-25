#include "can.h"
#include "core_config.h"

#include <stdio.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "gpio.h"
#include "clock.h"
#include "timeout.h"
#include "error_handler.h"

core_CAN_module_t can1;
core_CAN_module_t can2;
core_CAN_module_t can3;

static bool CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data);
static void rx_handler(FDCAN_GlobalTypeDef *can);
static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);
static bool CAN_clock_set_params(FDCAN_HandleTypeDef *hfdcan);

core_CAN_module_t *core_CAN_convert(FDCAN_GlobalTypeDef *can) {
    if (can == FDCAN1) return &can1;
    if (can == FDCAN2) return &can2;
    if (can == FDCAN3) return &can3;
    return NULL;
}

bool core_CAN_init(FDCAN_GlobalTypeDef *can)
{
    // Init port clock based on which CAN bus
    core_clock_FDCAN_init(can);
    core_CAN_module_t *p_can = core_CAN_convert(can);
    if (p_can == NULL) return false;

    // Initialize pins
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;


    // GPIO inits specific to different CAN buses, and HAL GPIO inits
    if (can == FDCAN1)
    {
        p_can->hfdcan.Instance = FDCAN1;
        gpio.Pin = CAN1_PINS;
        gpio.Alternate = CORE_FDCAN1_AF;
        HAL_GPIO_Init(CAN1_PORT, &gpio);

        // Auto retransmission settings
        p_can->hfdcan.Init.AutoRetransmission = CORE_FDCAN1_AUTO_RETRANSMISSION ? ENABLE : DISABLE;

        // Set max filter numbers
        p_can->hfdcan.Init.StdFiltersNbr = CORE_FDCAN1_MAX_STANDARD_FILTER_NUM;
        p_can->hfdcan.Init.ExtFiltersNbr = CORE_FDCAN1_MAX_EXTENDED_FILTER_NUM;
    }
    else if (can == FDCAN2)
    {
        p_can->hfdcan.Instance = FDCAN2;
        gpio.Pin = CAN2_PINS;
        gpio.Alternate = CORE_FDCAN2_AF;
        HAL_GPIO_Init(CAN2_PORT, &gpio);

        // Auto retransmission settings
        p_can->hfdcan.Init.AutoRetransmission = CORE_FDCAN2_AUTO_RETRANSMISSION ? ENABLE : DISABLE;

        // Set max filter numbers
        p_can->hfdcan.Init.StdFiltersNbr = CORE_FDCAN2_MAX_STANDARD_FILTER_NUM;
        p_can->hfdcan.Init.ExtFiltersNbr = CORE_FDCAN2_MAX_EXTENDED_FILTER_NUM;
    }
    else if (can == FDCAN3)
    {
        p_can->hfdcan.Instance = FDCAN3;
        gpio.Pin = CAN3_PINS;
        gpio.Alternate = CORE_FDCAN3_AF;
        HAL_GPIO_Init(CAN3_PORT, &gpio);

        // Auto retransmission settings
        p_can->hfdcan.Init.AutoRetransmission = CORE_FDCAN3_AUTO_RETRANSMISSION ? ENABLE : DISABLE;

        // Set max filter numbers
        p_can->hfdcan.Init.StdFiltersNbr = CORE_FDCAN3_MAX_STANDARD_FILTER_NUM;
        p_can->hfdcan.Init.ExtFiltersNbr = CORE_FDCAN3_MAX_EXTENDED_FILTER_NUM;
    }
    else return false;

	// Initialize CAN interface
	p_can->hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	p_can->hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	p_can->hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
	p_can->hfdcan.Init.TransmitPause = DISABLE;
	p_can->hfdcan.Init.ProtocolException = ENABLE;
	p_can->hfdcan.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
    CAN_clock_set_params(&(p_can->hfdcan));

    // Init CAN interface
    if (HAL_FDCAN_Init(&(p_can->hfdcan)) != HAL_OK)
    {
        return false;
    }

    // Reject all frames not configured in filter
    if (HAL_FDCAN_ConfigGlobalFilter(&(p_can->hfdcan), FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return false;
    }

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
    if (HAL_FDCAN_ConfigInterruptLines(&(p_can->hfdcan), FDCAN_IT_GROUP_RX_FIFO0 | FDCAN_IT_GROUP_SMSG, FDCAN_INTERRUPT_LINE0)) return false;
    if (HAL_FDCAN_ActivateNotification(&(p_can->hfdcan), FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) return false;
    if (HAL_FDCAN_ActivateNotification(&(p_can->hfdcan), FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK) return false;

    // Create queue to put received messages in
    p_can->can_queue_rx = xQueueCreate(CORE_CAN_QUEUE_LENGTH, sizeof(CanMessage_s));
    if (p_can->can_queue_rx == 0) error_handler();

    // Create queue to put outgoing messages in
    p_can->can_queue_tx = xQueueCreate(CORE_CAN_QUEUE_LENGTH, sizeof(CanMessage_s));
    if (p_can->can_queue_tx == 0) error_handler();

    p_can->can_tx_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(p_can->can_tx_semaphore);

	// Start can interface
	if (HAL_FDCAN_Start(&(p_can->hfdcan)) != HAL_OK)
    {
		return false;
	}

	return true;
}

bool core_CAN_add_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);
    CanMessage_s message = {(int)id, dlc, data};

    return xQueueSendToBack(p_can->can_queue_tx, &message, 0);
}

bool core_CAN_send_from_tx_queue_task(FDCAN_GlobalTypeDef *can)
{
    CanMessage_s dequeuedMessage;
    core_CAN_module_t *p_can = core_CAN_convert(can);
    bool autort;
    if (can == FDCAN1) autort = CORE_FDCAN1_AUTO_RETRANSMISSION;
    else if (can == FDCAN2) autort = CORE_FDCAN2_AUTO_RETRANSMISSION;
    else if (can == FDCAN3) autort = CORE_FDCAN3_AUTO_RETRANSMISSION;

    while ((xQueueReceive(p_can->can_queue_tx, &dequeuedMessage, portMAX_DELAY) == pdTRUE))
    {
        if (autort) xSemaphoreTake(p_can->can_tx_semaphore, portMAX_DELAY);
        if (!CAN_send_message(can, dequeuedMessage.id, dequeuedMessage.dlc, dequeuedMessage.data)) break;
    }

    return false;
}

static bool CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);

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

    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(&(p_can->hfdcan), &header, (uint8_t*) &data);
    return err == HAL_OK;
}


static void rx_handler(FDCAN_GlobalTypeDef *can)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);

    // If the interrupt flag is set for FIFO0
    if (p_can->hfdcan.Instance->IR & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        // Reset interrupt flag for FIFO0
        p_can->hfdcan.Instance->IR = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];

        // Retrieve Rx messages from RX FIFO0
        if (HAL_FDCAN_GetRxMessage(&(p_can->hfdcan), FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            error_handler();
        }
        // Reset the timeout
        core_timeout_reset_by_module_ref(can, header.Identifier);
        // Add the message to the RX queue
        add_CAN_message_to_rx_queue(can, header.Identifier, header.DataLength, data);
    }
    else if (p_can->hfdcan.Instance->IR & FDCAN_IR_TC) {
        // Clear interrupt flag
        p_can->hfdcan.Instance->IR = FDCAN_IR_TC;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(p_can->can_tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);

    uint64_t msg_data = 0;
    memcpy(&msg_data, data, dlc);

    CanMessage_s rx_msg;
    rx_msg.data = msg_data;
    rx_msg.id = id;
    rx_msg.dlc = dlc;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(p_can->can_queue_rx, &rx_msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);
    if (p_can == NULL) return false;

    // Receive CAN message from queue, copy it to buffer "received_message"
    // Return true if it read a value from the queue, false if not
    return (xQueueReceive(p_can->can_queue_rx, received_message, 0) == pdTRUE);
}

// Call RX interrupt handlers
void FDCAN1_IT0_IRQHandler(void) {rx_handler(FDCAN1);}
void FDCAN2_IT0_IRQHandler(void) {rx_handler(FDCAN2);}
void FDCAN3_IT0_IRQHandler(void) {rx_handler(FDCAN3);}

bool core_CAN_add_filter(FDCAN_GlobalTypeDef *can, bool isExtended, uint32_t id1, uint32_t id2)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);
    if (p_can == NULL) return false;
    uint8_t *p_num_filters;
    uint8_t max_filter_num;

    // Setup for each CAN bus, whether it's standard or extended
    if (can == FDCAN1)
    {
        if (isExtended) max_filter_num = CORE_FDCAN1_MAX_EXTENDED_FILTER_NUM;
        else max_filter_num = CORE_FDCAN1_MAX_STANDARD_FILTER_NUM;
    }
    else if (can == FDCAN2)
    {
        if (isExtended) max_filter_num = CORE_FDCAN2_MAX_EXTENDED_FILTER_NUM;
        else max_filter_num = CORE_FDCAN2_MAX_STANDARD_FILTER_NUM;
    }
    else
    {
        if (isExtended) max_filter_num = CORE_FDCAN3_MAX_EXTENDED_FILTER_NUM;
        else max_filter_num = CORE_FDCAN3_MAX_STANDARD_FILTER_NUM;
    }

    if (isExtended) {
        p_num_filters = &(p_can->fdcan_num_extended_filters);
    } else {
        p_num_filters = &(p_can->fdcan_num_standard_filters);
    }
    if (*p_num_filters + 1 >  max_filter_num) return false;
    FDCAN_FilterTypeDef filter;
    filter.IdType = isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterIndex = *p_num_filters++;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = id1;
    filter.FilterID2 = id2;

    return HAL_FDCAN_ConfigFilter(&(p_can->hfdcan), &filter) == HAL_OK;

}

static bool CAN_clock_set_params(FDCAN_HandleTypeDef *hfdcan)
{
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

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

// Handlers for different CAN buses
static FDCAN_HandleTypeDef hfdcan1;
static FDCAN_HandleTypeDef hfdcan2;
static FDCAN_HandleTypeDef hfdcan3;

// Handlers for RX queues
static QueueHandle_t can1_queue_rx;
static QueueHandle_t can2_queue_rx;
static QueueHandle_t can3_queue_rx;

// Handlers for TX queues
static QueueHandle_t can1_queue_tx;
static QueueHandle_t can2_queue_tx;
static QueueHandle_t can3_queue_tx;


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
        p_canQueueRx = &can1_queue_rx;
        p_canQueueTx = &can1_queue_tx;
        p_hfdcan = &hfdcan1;
        p_hfdcan->Instance = FDCAN1;
        gpio.Pin = CAN1_PINS;
        gpio.Alternate = CORE_FDCAN1_AF;
        HAL_GPIO_Init(CAN1_PORT, &gpio);
    }
    else if (can == FDCAN2)
    {
        p_canQueueRx = &can2_queue_rx;
        p_canQueueTx = &can2_queue_tx;
        p_hfdcan = &hfdcan2;
        p_hfdcan->Instance = FDCAN2;
        gpio.Pin = CAN2_PINS;
        gpio.Alternate = CORE_FDCAN3_AF;
        HAL_GPIO_Init(CAN2_PORT, &gpio);
    }
    else if (can == FDCAN3)
    {
        p_canQueueRx = &can3_queue_rx;
        p_canQueueTx = &can3_queue_tx;
        p_hfdcan = &hfdcan3;
        p_hfdcan->Instance = FDCAN3;
        gpio.Pin = CAN3_PINS;
        gpio.Alternate = CORE_FDCAN3_AF;
        HAL_GPIO_Init(CAN3_PORT, &gpio);
    }
    else return false;

	// Initialize CAN interface
	p_hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
	p_hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	p_hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
	p_hfdcan->Init.AutoRetransmission = DISABLE;
	p_hfdcan->Init.TransmitPause = DISABLE;
	p_hfdcan->Init.ProtocolException = ENABLE;
	p_hfdcan->Init.NominalPrescaler = 8;
	p_hfdcan->Init.NominalSyncJumpWidth = 1;
	p_hfdcan->Init.NominalTimeSeg1 = 12;
	p_hfdcan->Init.NominalTimeSeg2 = 2;
	p_hfdcan->Init.DataPrescaler = 1; // Data timing fields unused for classic CAN
	p_hfdcan->Init.DataSyncJumpWidth = 1;
	p_hfdcan->Init.DataTimeSeg1 = 1;
	p_hfdcan->Init.DataTimeSeg2 = 1;
	p_hfdcan->Init.StdFiltersNbr = 28;
	p_hfdcan->Init.ExtFiltersNbr = 8;
	p_hfdcan->Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;

    // Init CAN interface
    if (HAL_FDCAN_Init(p_hfdcan) != HAL_OK)
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
    if (HAL_FDCAN_ConfigGlobalFilter(p_hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0) != HAL_OK)
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
    HAL_FDCAN_ConfigInterruptLines(p_hfdcan, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
    HAL_FDCAN_ActivateNotification(p_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);


    // Create queue to put received messages in
    *p_canQueueRx = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CanMessage_s));
    if (*p_canQueueRx == 0) error_handler();

    // Create queue to put outgoing messages in
    *p_canQueueTx = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CanMessage_s));
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
void FDCAN1_IT0_IRQHandler(void) {rx_handler(FDCAN1);}
void FDCAN2_IT0_IRQHandler(void) {
    GPIO_set_heartbeat(GPIO_PIN_SET);
    rx_handler(FDCAN2);
}
void FDCAN3_IT0_IRQHandler(void) {rx_handler(FDCAN3);}
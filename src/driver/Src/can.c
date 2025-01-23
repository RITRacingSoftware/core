/**
  * @file   can.c
  * @brief  Core FDCAN library
  * 
  * This core library component is used to interface with the FDCAN hardware.
  *
  * ## Initialization
  * An FDCAN module is initialized by calling the core_CAN_init() function. The
  * CAN bitrate is set to `CORE_CAN_BITRATE`, which is given in bits/second and
  * defined in `core_config.h`. If `CORE_FDCANx_AUTO_RETRANSMISSION` is set to
  * 1 in `core_config.h`, the FDCANx module will be cofigured to automatically 
  * retransmit packets that were not acknowledged. If `CORE_FDCANx_USE_FD` is 
  * set to 1 in `core_config.h`, the FDCANx module will be able to send and 
  * receive FD CAN frames as well as standard CAN frames.
  *
  * ## Transmitting
  * Transmission on the CAN bus is managed by a FreeRTOS queue and occurs in
  * two parts. First, the user code adds a CAN frame to the queue with
  * core_CAN_add_message_to_tx_queue() (classic and FD CAN) or with 
  * core_CAN_add_extended_message_to_tx_queue() (FD CAN only). 
  *
  * The user code must also run core_CAN_send_from_tx_queue_task() in a
  * dedicated FreeRTOS task. If core_CAN_send_from_tx_queue_task(), an error
  * has occurred while transmitting.
  *
  * Internally, a FreeRTOS semaphore is used to ensure only one message sits
  * in the hardware CAN queue at a time. The semaphore is given whenever a
  * transmission completes (or fails) and is taken whenever a message is
  * added to the hardware queue.
  *
  * Alternatively, the user can disable the FreeRTOS queue by defining
  * `CORE_CAN_DISABLE_TX_QUEUE` in `core_config.h`. This is useful if the user
  * needs finer control over the transmission, wants to synchronize CAN
  * transmissions between FDCAN modules, or would like to save SRAM space.
  * The RX queue is not affected and the user can access the semaphore using
  * core_CAN_convert(), but will be responsible for taking the semaphore.
  *
  * ## Receiving
  * In order to enable receiving CAN frames, the user code must first set up
  * one or more filters using the core_CAN_add_filter() function. When a frame
  * matching any of the applied filters is received, the FDCAN hardware
  * triggers an interrupt that inserts the contents of this frame into a
  * receiving FreeRTOS queue.
  *
  * The user code must the define a task that repeatedly calls 
  * core_CAN_receive_from_queue() (classic CAN) or 
  * core_CAN_receive_extended_from_queue() (FD CAN only). These functions will
  * return 1 if a frame has been loaded from the queue and 0 otherwise. It is 
  * necessary to poll these functions, since they are non-blocking.
  */

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
#include "boot.h"

static core_CAN_module_t can1;
static core_CAN_module_t can2;
static core_CAN_module_t can3;

const uint8_t core_CAN_dlc_lookup[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

static void rx_handler(FDCAN_GlobalTypeDef *can);
static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data);
static void add_CAN_extended_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data, bool use_fd);
static bool CAN_clock_set_params(FDCAN_HandleTypeDef *hfdcan);

core_CAN_module_t *core_CAN_convert(FDCAN_GlobalTypeDef *can) {
    if (can == FDCAN1) return &can1;
    if (can == FDCAN2) return &can2;
    if (can == FDCAN3) return &can3;
    return NULL;
}

/**
  * @brief  Initialize an FDCAN module, the RX and TX queues, 
  *         and the RX and TX pins.
  * @retval 0 if the given FDCAN is not valid or the initialization failed
  * @retval 1 otherwise
  */
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
        gpio.Pin = CORE_FDCAN1_TX_PIN;
        gpio.Alternate = CORE_FDCAN1_TX_AF;
        HAL_GPIO_Init(CORE_FDCAN1_TX_PORT, &gpio);
        gpio.Pin = CORE_FDCAN1_RX_PIN;
        gpio.Alternate = CORE_FDCAN1_RX_AF;
        HAL_GPIO_Init(CORE_FDCAN1_RX_PORT, &gpio);

        // Auto retransmission settings
        p_can->hfdcan.Init.AutoRetransmission = CORE_FDCAN1_AUTO_RETRANSMISSION ? ENABLE : DISABLE;
        p_can->autort = CORE_FDCAN1_AUTO_RETRANSMISSION;

        // Set max filter numbers
        p_can->hfdcan.Init.StdFiltersNbr = CORE_FDCAN1_MAX_STANDARD_FILTER_NUM;
        p_can->hfdcan.Init.ExtFiltersNbr = CORE_FDCAN1_MAX_EXTENDED_FILTER_NUM;

        // Extended frame settings
        p_can->use_fd = CORE_FDCAN1_USE_FD;
    }
    else if (can == FDCAN2)
    {
        p_can->hfdcan.Instance = FDCAN2;
        gpio.Pin = CORE_FDCAN2_TX_PIN;
        gpio.Alternate = CORE_FDCAN2_TX_AF;
        HAL_GPIO_Init(CORE_FDCAN2_TX_PORT, &gpio);
        gpio.Pin = CORE_FDCAN2_RX_PIN;
        gpio.Alternate = CORE_FDCAN2_RX_AF;
        HAL_GPIO_Init(CORE_FDCAN2_RX_PORT, &gpio);

        // Auto retransmission settings
        p_can->hfdcan.Init.AutoRetransmission = CORE_FDCAN2_AUTO_RETRANSMISSION ? ENABLE : DISABLE;
        p_can->autort = CORE_FDCAN2_AUTO_RETRANSMISSION;

        // Set max filter numbers
        p_can->hfdcan.Init.StdFiltersNbr = CORE_FDCAN2_MAX_STANDARD_FILTER_NUM;
        p_can->hfdcan.Init.ExtFiltersNbr = CORE_FDCAN2_MAX_EXTENDED_FILTER_NUM;

        // Extended frame settings
        p_can->use_fd = CORE_FDCAN2_USE_FD;
    }
    else if (can == FDCAN3)
    {
        p_can->hfdcan.Instance = FDCAN3;
        gpio.Pin = CORE_FDCAN3_TX_PIN;
        gpio.Alternate = CORE_FDCAN3_TX_AF;
        HAL_GPIO_Init(CORE_FDCAN3_TX_PORT, &gpio);
        gpio.Pin = CORE_FDCAN3_RX_PIN;
        gpio.Alternate = CORE_FDCAN3_RX_AF;
        HAL_GPIO_Init(CORE_FDCAN3_RX_PORT, &gpio);

        // Auto retransmission settings
        p_can->hfdcan.Init.AutoRetransmission = CORE_FDCAN3_AUTO_RETRANSMISSION ? ENABLE : DISABLE;
        p_can->autort = CORE_FDCAN3_AUTO_RETRANSMISSION;

        // Set max filter numbers
        p_can->hfdcan.Init.StdFiltersNbr = CORE_FDCAN3_MAX_STANDARD_FILTER_NUM;
        p_can->hfdcan.Init.ExtFiltersNbr = CORE_FDCAN3_MAX_EXTENDED_FILTER_NUM;

        // Extended frame settings
        p_can->use_fd = CORE_FDCAN3_USE_FD;
    }
    else return false;

    // Initialize CAN interface
    p_can->hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    p_can->hfdcan.Init.FrameFormat = (p_can->use_fd ? FDCAN_FRAME_FD_NO_BRS : FDCAN_FRAME_CLASSIC);
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
    if (HAL_FDCAN_ConfigInterruptLines(&(p_can->hfdcan), FDCAN_IT_GROUP_RX_FIFO0 | FDCAN_IT_GROUP_SMSG | FDCAN_IT_GROUP_PROTOCOL_ERROR, FDCAN_INTERRUPT_LINE0)) return false;
    if (HAL_FDCAN_ActivateNotification(&(p_can->hfdcan), FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) return false;
    if (HAL_FDCAN_ActivateNotification(&(p_can->hfdcan), FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK) return false;
    if (HAL_FDCAN_ActivateNotification(&(p_can->hfdcan), FDCAN_IT_ARB_PROTOCOL_ERROR, 0) != HAL_OK) return false;
    //if (HAL_FDCAN_ActivateNotification(&(p_can->hfdcan), FDCAN_IT_TX_ABORT_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK) return false;

    // Create queue to put received messages in
    size_t msgsize = (p_can->use_fd ? sizeof(CanExtendedMessage_s) : sizeof(CanMessage_s));
    p_can->can_queue_rx = xQueueCreate(CORE_CAN_QUEUE_LENGTH, msgsize);
    if (p_can->can_queue_rx == 0) error_handler();

#ifndef CORE_CAN_DISABLE_TX_QUEUE
    // Create queue to put outgoing messages in
    p_can->can_queue_tx = xQueueCreate(CORE_CAN_QUEUE_LENGTH, msgsize);
    if (p_can->can_queue_tx == 0) error_handler();
#endif

    p_can->can_tx_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(p_can->can_tx_semaphore);

    // Start can interface
    if (HAL_FDCAN_Start(&(p_can->hfdcan)) != HAL_OK)
    {
        return false;
    }

    return true;
}

#ifndef CORE_CAN_DISABLE_TX_QUEUE
/**
  * @brief  Add a CAN frame to the TX queue
  * @param  can FDCAN module for which the frame is being enqueued
  * @param  id ID of the CAN frame
  * @param  dlc Number of data bytes in the CAN frame
  * @param  data Data bytes, encoded LSB-first as a uint64
  * @retval 0 if the queue is full
  * @retval 1 otherwise
  */
bool core_CAN_add_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);
    if (p_can->use_fd) {
        CanExtendedMessage_s message;
        message.id = id;
        message.dlc = dlc;
        message.use_fd = false;
        memcpy(&(message.data), &data, 8);
        return xQueueSendToBack(p_can->can_queue_tx, &message, 0);
    } else {
        CanMessage_s message = {(int)id, dlc, data};
        return xQueueSendToBack(p_can->can_queue_tx, &message, 0);
    }
}

/**
  * @brief  Add a CAN frame to the TX queue
  * @param  can FDCAN module for which the frame is being enqueued
  * @param  id ID of the CAN frame
  * @param  dlc Number of data bytes in the CAN frame
  * @param  data Pointer to an array of data bytes
  * @retval 0 if the queue is full, if the FDCAN is not configured for
  *         FD operation, or if `dlc` > 64
  * @retval 1 otherwise
  */
bool core_CAN_add_extended_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);
    if (!(p_can->use_fd)) return false;
    if (dlc > 64) return false;
    CanExtendedMessage_s message;
    message.id = id;
    message.dlc = dlc;
    message.use_fd = true;
    //for (uint8_t i=0; i < dlc; i++) message.data[i] = data[i];
    memcpy(&(message.data), data, dlc);
    return xQueueSendToBack(p_can->can_queue_tx, &message, 0);
}

/**
  * @brief  Loop for sending data in the TX queue over CAN. This function
  *         must be run in its own task.
  * @param  can FDCAN module
  */
bool core_CAN_send_from_tx_queue_task(FDCAN_GlobalTypeDef *can)
{
    CanExtendedMessage_s dequeuedExtendedMessage;
    CanMessage_s dequeuedMessage;
    core_CAN_module_t *p_can = core_CAN_convert(can);
    bool autort = p_can->autort;
    if (p_can->use_fd) {
        uint64_t data;
        while ((xQueueReceive(p_can->can_queue_tx, &dequeuedExtendedMessage, portMAX_DELAY) == pdTRUE)) {
            /*if (autort)*/ xSemaphoreTake(p_can->can_tx_semaphore, portMAX_DELAY);
            if (dequeuedExtendedMessage.use_fd) {
                if (!core_CAN_send_fd_message(can, dequeuedExtendedMessage.id, dequeuedExtendedMessage.dlc, dequeuedExtendedMessage.data)) break;
            } else {
                memcpy(&data, dequeuedExtendedMessage.data, 8);
                //uint8_t dlc = dequeuedExtendedMessage.dlc;
                if (!core_CAN_send_message(can, dequeuedExtendedMessage.id, dequeuedExtendedMessage.dlc, data)) break;
            }
        }
    } else {
        while ((xQueueReceive(p_can->can_queue_tx, &dequeuedMessage, portMAX_DELAY) == pdTRUE)) {
            /*if (autort)*/ xSemaphoreTake(p_can->can_tx_semaphore, portMAX_DELAY);
            if (!core_CAN_send_message(can, dequeuedMessage.id, dequeuedMessage.dlc, dequeuedMessage.data)) break;
        }
    }
    return false;
}
#endif

/**
  * @brief  Add a CAN message to the hardware FIFO
  * @param  can FDCAN module
  * @param  id ID of the message to be transmitted. If this value is greater
  *         than 2047, then an extended ID is automatically selected. Only the
  *         lowest 29 bits are kept, so setting the MSB will force an extended
  *         ID even if the ID is less than or equal to 2047
  * @param  dlc Length of the packet (0-8)
  * @param  data Data encoded as a uint64_t
  * @retval 0 if an error occurred while adding the message to the queue
  * @retval 1 otherwise
  */
bool core_CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);

    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = (id & 0x1fffffff);
    header.IdType = (id >= 2048 ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID);
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = dlc;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    header.MessageMarker = 0;

    while ((can->PSR & (0x3 << 3)) != (0x01 << 3));
    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(&(p_can->hfdcan), &header, (uint8_t*) &data);
    return err == HAL_OK;
}

/**
  * @brief  Add an FDCAN message to the hardware FIFO
  * @param  can FDCAN module
  * @param  id ID of the message to be transmitted. If this value is greater
  *         than 2047, then an extended ID is automatically selected. Only the
  *         lowest 29 bits are kept, so setting the MSB will force an extended
  *         ID even if the ID is less than or equal to 2047
  * @param  dlc Length of the packet. If the length does not correspond to a
  *         valid FDCAN packet length, 
  * @param  data Pointer to the data to be transmitted
  * @retval 0 if an error occurred while adding the message to the queue
  * @retval 1 otherwise
  */
bool core_CAN_send_fd_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);

    uint8_t i=0;
    while (core_CAN_dlc_lookup[i] < dlc) i++;
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = (id & 0x1fffffff);
    header.IdType = (id >= 2048 ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID);
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = i;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_FD_CAN;
    header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    header.MessageMarker = 0;

    while ((can->PSR & (0x3 << 3)) != (0x01 << 3));
    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(&(p_can->hfdcan), &header, data);
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
        uint8_t data[64];

        // Retrieve Rx messages from RX FIFO0
        if (HAL_FDCAN_GetRxMessage(&(p_can->hfdcan), FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            error_handler();
        }
        core_GPIO_toggle_heartbeat();
        if ((header.IdType == FDCAN_EXTENDED_ID) && (header.Identifier == (CORE_BOOT_FDCAN_ID << 18))) {
            //core_GPIO_toggle_heartbeat();
            core_boot_reset_and_enter();
        }
        // Reset the timeout
        core_timeout_reset_by_module_ref(can, header.Identifier);
        // Add the message to the RX queue
        if (p_can->use_fd) add_CAN_extended_message_to_rx_queue(can, header.Identifier, core_CAN_dlc_lookup[header.DataLength], data, header.FDFormat == FDCAN_FD_CAN);
        else add_CAN_message_to_rx_queue(can, header.Identifier, core_CAN_dlc_lookup[header.DataLength], data);
    }
    else if (p_can->hfdcan.Instance->IR & FDCAN_IR_TC) {
        // Clear interrupt flag
        p_can->hfdcan.Instance->IR = FDCAN_IR_TC;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(p_can->can_tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else if (p_can->hfdcan.Instance->IR & FDCAN_IR_PEA) {
        p_can->hfdcan.Instance->IR = FDCAN_IR_PEA;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(p_can->can_tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void add_CAN_extended_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data, bool use_fd) {
    core_CAN_module_t *p_can = core_CAN_convert(can);

    CanExtendedMessage_s rx_msg;
    rx_msg.id = id;
    rx_msg.dlc = dlc;
    rx_msg.use_fd = use_fd;
    memcpy(rx_msg.data, data, dlc);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(p_can->can_queue_rx, &rx_msg, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data) {
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

/**
  * @brief  If a frame is waiting in the RX queue, copy it to the given location
  * @param  can FDCAN module from which the frame is read
  * @param  received_message Pointer to the location where the received frame
  *         would be stored
  * @retval 1 if a frame was copied from the queue into the given location
  * @retval 0 otherwise
  */
bool core_CAN_receive_extended_from_queue(FDCAN_GlobalTypeDef *can, CanExtendedMessage_s *received_message)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);
    if (p_can == NULL) return false;

    // Receive CAN message from queue, copy it to buffer "received_message"
    // Return true if it read a value from the queue, false if not
    return (xQueueReceive(p_can->can_queue_rx, received_message, CORE_CAN_RX_TIMEOUT) == pdTRUE);
}

/**
  * @brief  If a frame is waiting in the RX queue, copy it to the given location
  * @param  can FDCAN module from which the frame is read
  * @param  received_message Pointer to the location where the received frame
  *         would be stored
  * @retval 1 if a frame was copied from the queue into the given location
  * @retval 0 otherwise
  */
bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);
    if (p_can == NULL) return false;

    // Receive CAN message from queue, copy it to buffer "received_message"
    // Return true if it read a value from the queue, false if not
    return (xQueueReceive(p_can->can_queue_rx, received_message, CORE_CAN_RX_TIMEOUT) == pdTRUE);
}

// Call RX interrupt handlers
void FDCAN1_IT0_IRQHandler(void) {rx_handler(FDCAN1);}
void FDCAN2_IT0_IRQHandler(void) {rx_handler(FDCAN2);}
void FDCAN3_IT0_IRQHandler(void) {rx_handler(FDCAN3);}

/**
  * @brief  Add an RX filter for the given FDCAN module
  *
  * All frames with IDs greater than or equal to `id1` and less than or equal
  * to `id2` will be placed in the RX queue
  * @param  can FDCAN module for which the filter should be created
  * @param  isExtended Specifies whether the IDs are extended CAN IDs
  * @param  id1 Lower bound (inclusive)
  * @param  id2 Upper bound (inclusive)
  * @retval 1 if the filter was added successfully
  * @retval 0 otherwise
  */
bool core_CAN_add_filter(FDCAN_GlobalTypeDef *can, bool isExtended, uint32_t id1, uint32_t id2)
{
    core_CAN_module_t *p_can = core_CAN_convert(can);
    if (p_can == NULL) return false;
    uint8_t *p_num_filters;
    uint8_t max_filter_num;

    // Setup for each CAN bus, whether it's standard or extended
    if (isExtended) max_filter_num = p_can->hfdcan.Init.ExtFiltersNbr;
    else max_filter_num = p_can->hfdcan.Init.StdFiltersNbr;

    if (isExtended) {
        p_num_filters = &(p_can->fdcan_num_extended_filters);
    } else {
        p_num_filters = &(p_can->fdcan_num_standard_filters);
    }
    if (*p_num_filters + 1 >  max_filter_num) return false;
    FDCAN_FilterTypeDef filter;
    filter.IdType = isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterIndex = *p_num_filters++;
    filter.FilterType = FDCAN_FILTER_RANGE;
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

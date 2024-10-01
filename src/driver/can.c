#include "can.h"

#include "clock.h"
#include <stdbool.h>
#include <stdint.h>

#include <stm32g4xx_hal.h>

#define CAN1_PORT GPIOA
#define CAN2_PORT GPIOB
#define CAN3_PORT GPIOB
#define CAN1_PINS (GPIO_PIN_11 | GPIO_PIN_12)
#define CAN2_PINS (GPIO_PIN_12 | GPIO_PIN_13)
#define CAN3_PINS (GPIO_PIN_3 | GPIO_PIN_4)

static FDCAN_HandleTypeDef can1;
static FDCAN_HandleTypeDef can2;
static FDCAN_HandleTypeDef can3;

bool core_CAN_init(CAN_num canNum)
{

    // Init port clock based on which CAN bus
    core_clock_FDCAN_init();
    FDCAN_HandleTypeDef *p_can;

    // Initialize pins
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;


    // GPIO inits specific to different CAN buses, and HAL GPIO inits
    switch(canNum)
    {
        case CAN1:
            p_can = &can1;
            p_can->Instance = FDCAN1;
            gpio.Pin = CAN1_PINS;
            gpio.Alternate = GPIO_AF9_FDCAN1;
            HAL_GPIO_Init(CAN1_PORT, &gpio);
            break;

        case CAN2:
            p_can = &can2;
            p_can->Instance = FDCAN2;
            gpio.Pin = CAN2_PINS;
            gpio.Alternate = GPIO_AF9_FDCAN2;
            HAL_GPIO_Init(CAN2_PORT, &gpio);
            break;

        case CAN3:
            p_can = &can3;
            p_can->Instance = FDCAN3;
            gpio.Pin = CAN3_PINS;
            gpio.Alternate = GPIO_AF11_FDCAN3;
            HAL_GPIO_Init(CAN3_PORT, &gpio);
            break;
    }


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

	// Start can interface
	if (HAL_FDCAN_Start(p_can) != HAL_OK)
	{
		return false;
	}

	return true;
}

bool core_CAN_send(CAN_num canNum, uint32_t id, uint8_t dlc, uint64_t data)
{
    FDCAN_HandleTypeDef *p_can;

    // Which CAN bus to send through
    switch (canNum)
    {
        case CAN1: p_can = &can1; break;
        case CAN2: p_can = &can2; break;
        case CAN3: p_can = &can3; break;
    }


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

bool test_CAN_init()
{
    // Initialize pins
    GPIO_InitTypeDef gpio1 = {CAN2_PINS, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF9_FDCAN2};
    HAL_GPIO_Init(GPIOB, &gpio1);

    // Initialize CAN interfaces
    can2.Instance = FDCAN2;
    can2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    can2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    can2.Init.Mode = FDCAN_MODE_NORMAL;
    can2.Init.AutoRetransmission = DISABLE;
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

    if (HAL_FDCAN_Init(&can2) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_Start(&can2) != HAL_OK)
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

    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(&can2, &header, (uint8_t*) &data);

    return (err == HAL_OK);
}
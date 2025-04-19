#include can.h

__weak bool core_CAN_init(FDCAN_GlobalTypeDef *fdcan) {}
__weak core_CAN_module_t *core_CAN_convert(FDCAN_GlobalTypeDef *fdcan) {}
__weak bool core_CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data) {}
__weak bool core_CAN_send_fd_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data) {}
__weak bool core_CAN_add_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data) {}
__weak bool core_CAN_add_extended_message_to_tx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data) {}
__weak bool core_CAN_send_from_tx_queue_task(FDCAN_GlobalTypeDef *can) {}
__weak bool core_CAN_receive_from_queue(FDCAN_GlobalTypeDef *can, CanMessage_s *received_message) {}
__weak bool core_CAN_receive_extended_from_queue(FDCAN_GlobalTypeDef *can, CanExtendedMessage_s *received_message) {}
__weak bool core_CAN_add_filter(FDCAN_GlobalTypeDef *can, bool isExtended, uint32_t id1, uint32_t id2) {}
__weak static bool CAN_send_message(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint64_t data) {}
__weak static void rx_handler(FDCAN_GlobalTypeDef *can) {}
__weak static void add_CAN_message_to_rx_queue(FDCAN_GlobalTypeDef *can, uint32_t id, uint8_t dlc, uint8_t *data) {}

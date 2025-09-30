/**
  * @file   usart.h
  * @brief  Core UART library
  *
  * This core library component is used to initialize USARTs, transmit data 
  * over USART, and asynchronously receive data over USART.
  *
  * ## Initialization
  * To initialize a USART for transmitting, user code must call the function
  * core_USART_init() and specify the desired baud rate. 
  *
  * ## Receiving
  * To receive data, the user must call core_USART_register_callback() with
  * a pointer to a function that will be called when data has been received.
  * If no data has been received for CORE_USART_TIMEOUT bit periods, then 
  * the callback function is called with a pointer to the internal buffer as
  * well as the number of received bytes.
  *
  */

#ifndef CORE_USART_H
#define CORE_USART_H

#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include "core_config.h"

#define CORE_USART1_UPDATE 0x01
#define CORE_USART2_UPDATE 0x02
#define CORE_USART3_UPDATE 0x04

/**
  * @brief  Initialize a USART module in asynchronous mode with the given baud
  *         rate.
  * @param  usart The USART module to initialize
  * @param  baud Baud rate
  * @retval 0 if the given USART is not valid of if the initialization failed
  * @retval 1 otherwise
  */
bool core_USART_init(USART_TypeDef *usart, uint32_t baud);

/**
  * @brief  Transmit data from a USART
  * @note   This function is blocking and will not return until all data has
  *         been transmitted.
  * @param  usart The USART module
  * @param  txbuf Location where the data to be transmitted is read from
  * @param  txbuflen Number of bytes to transmit
  * @retval 1 if transmission was successful
  * @retval 0 otherwise
  */
bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen);

/**
  * @brief  Set the RX callback and start the receiver for the given USART module
  * @param  usart The USART module
  * @param  callback Function to be called after data is received (the RX
  *         timeout elapses). The function must take a pointer to a byte array
  *         as the first argument and a uint32_t length as the second argument
  * @retval 0 if the given USART is not valid
  * @retval 1 otherwise
  */
bool core_USART_register_callback(USART_TypeDef *usart, void (*callback)(uint8_t *, uint32_t));

/**
  * @brief  Synchronously receive data from a USART
  * @note   This function is blocking and will not return until all data has
  *         been read or until the timeout has elapsed
  * @param  usart The USART module
  * @param  rxbuf Location where the data to be received is stored
  * @param  rxbuflen Size of the RX buffer
  * @param  timeout RX timeout in microseconds
  * @return Returns the number of bytes received
  */
uint32_t core_USART_receive(USART_TypeDef *usart, uint8_t *rxbuf, uint32_t rxbuflen, uint32_t timeout);

#if (CORE_USART_UPRINTF != 0) || defined(DOXYGEN)
/**
  * @brief  Print a formatted string to a USART
  * @note   This function is blocking and will not return until all data has
  *         been transmitted. This function is only defined if
  *         CORE_USART_UPRINTF is not zero
  * @param  usart The USART module
  * @param  format Format string
  * @param  txbuflen Number of bytes to transmit
  * @return Returns -1 if the transmission failed. Otherwise, returns the
  *         number of transmitted bytes.
  */
int uprintf(USART_TypeDef *usart, const char *format, ...);
#endif

#endif

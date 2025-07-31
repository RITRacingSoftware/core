#pragma once
#include <stdint.h>


// GENERAL
uint32_t HAL_GetTick();

// USART
typedef struct {
} USART_InitTypeDef;

typedef struct {
} USART_TypeDef;

typedef struct __USART_HandleTypeDef {
} USART_HandleTypeDef;

#define USART1              ((USART_TypeDef *) 1)
#define USART2              ((USART_TypeDef *) 1)
#define USART3              ((USART_TypeDef *) 1)

// GPIO
typedef struct {
} GPIO_TypeDef;

typedef enum
{
  GPIO_PIN_RESET,
  GPIO_PIN_SET
} GPIO_PinState;

#define GPIO_MODE_INPUT     1
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_PULLDOWN       1
#define GPIO_PULLUP         1
#define GPIO_NOPULL         1

#define GPIOA               ((GPIO_TypeDef *) 0)
#define GPIOB               ((GPIO_TypeDef *) 1)
#define GPIOC               ((GPIO_TypeDef *) 2)
#define GPIOD               ((GPIO_TypeDef *) 3)
#define GPIOE               ((GPIO_TypeDef *) 4)
#define GPIOF               ((GPIO_TypeDef *) 5)
#define GPIOG               ((GPIO_TypeDef *) 6)


#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */


// ADC
typedef struct {
} ADC_TypeDef;

#define ADC1 ((ADC_TypeDef *) 2)
#define ADC2 ((ADC_TypeDef *) 2)

// OPAMP
typedef struct {
} OPAMP_TypeDef;


// CAN
typedef struct {
} FDCAN_GlobalTypeDef;

typedef struct {
} FDCAN_HandleTypeDef;

#define FDCAN1 ((FDCAN_GlobalTypeDef *) 1)
#define FDCAN2 ((FDCAN_GlobalTypeDef *) 1)
#define FDCAN3 ((FDCAN_GlobalTypeDef *) 1)


// I2C
typedef struct {
} I2C_TypeDef;


// SPI
typedef struct {
} SPI_TypeDef;

// CLOCK
typedef struct {
} TIM_TypeDef;

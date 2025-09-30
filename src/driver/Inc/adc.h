/**
  * @file   adc.h
  * @brief  Core ADC library
  *
  *
  *
  * This core library component is used to initialize ADCs and read from analog
  * inputs. 
  *
  * ## Initialization
  * Before initializing any pins, the user code must initialize one or more ADC
  * modules. 
  * 
  * To initialize a pin, the user code calls core_ADC_setup_pin(). This
  * function takes the port and pin number as well as a third argument
  * specifying whether the analog signal should be routed through an internal
  * op amp follower circuit. This will improve the accuracy of the measurement
  * when the analog input is fed from a high-impedance source
  *
  * If core_ADC_setup_pin() returns 0, then the desired pin cannot be connected
  * to any of the ADCs that are currently initialized. It may be necessary to
  * initialize an additional ADC module or change the pin the analog input is
  * connected to.
  *
  * ## Reading
  * To read from an analog input, the user code calls core_ADC_read_channel().
  * The result is stored in a pointer passed as an argument, and the return
  * value specifies whether the conversion was successful or not.
  *
  * ## Table of connections
  * The table below indicates which pins connect to which ADC and whether an
  * op amp can be connected in between the input and the ADC. If a table cell
  * contains a number, then the pin corresponding to the row connects to the
  * ADC corrsponding to the column directly. If the cell also indicates an 
  * op amp (OPAMP1-OPAMP6), then the pin can be connected to the ADC via
  * an op amp. Note that not all chips will have all 5 ADCs. 
  * | Pin  |    ADC1    |    ADC2    |    ADC3    |    ADC4    |    ADC5    |
  * | ---- | ----------:| ----------:| ----------:| ----------:| ----------:|
  * | PA0  |          1 |          1 |            |            |            |
  * | PA1  |   OPAMP1/2 |   OPAMP3/2 |     OPAMP3 |            |            |
  * | PA2  |          3 |            |            |            |            |
  * | PA3  |   OPAMP1/4 |            |            |            |            |
  * | PA4  |            |         17 |            |            |            |
  * | PA5  |            |         13 |            |            |            |
  * | PA6  |            |          3 |            |            |            |
  * | PA7  |     OPAMP1 |   OPAMP2/4 |            |            |            |
  * | PA8  |            |            |            |            |          1 |
  * | PA9  |            |            |            |            |          2 |
  * | PB0  |         15 | OPAMP2/OPAMP3 | OPAMP3/12 |          |            |
  * | PB1  |         12 |            |          1 |            |            |
  * | PB2  |            |         12 |            |            |            |
  * | PB11 |         14 |         14 |            |            |     OPAMP4 |
  * | PB12 |         11 |            |            |   OPAMP6/3 |            |
  * | PB13 |            |     OPAMP3 |   OPAMP3/5 |     OPAMP6 |     OPAMP4 |
  * | PB14 |          5 |     OPAMP2 |            |          4 |     OPAMP5 |
  * | PB15 |            |         15 |            |          5 |            |
  * | PC0  |          6 |          6 |            |            |            |
  * | PC1  |          7 |          7 |            |            |            |
  * | PC2  |          8 |          8 |            |            |            |
  * | PC3  |          9 |          9 |            |            |     OPAMP5 |
  * | PC4  |            |          5 |            |            |            |
  * | PC5  |            |         11 |            |            |            |
  * | PD10 |            |            |          7 |          7 |          7 |
  * | PD11 |            |            |          8 |          8 |   OPAMP4/8 |
  * | PD12 |            |            |          9 |          9 |   OPAMP5/9 |
  * | PD13 |            |            |         10 |         10 |         10 |
  * | PD14 |            |    OPAMP2  |         11 |         11 |         11 |
  * | PD8  |            |            |            |         12 |         12 |
  * | PD9  |            |            |            |  OPAMP6/13 |         13 |
  * | PE7  |            |            |          4 |            |            |
  * | PE8  |            |            |          6 |          6 |          6 |
  * | PE9  |            |            |          2 |            |            |
  * | PE10 |            |            |         14 |         14 |         14 |
  * | PE11 |            |            |         15 |         15 |         15 |
  * | PE12 |            |            |         16 |         16 |         16 |
  * | PE13 |            |            |          3 |            |            |
  * | PE14 |            |            |            |          1 |            |
  * | PE15 |            |            |            |          2 |            |
  * | PF0  |         10 |            |            |            |            |
  * | PF1  |            |         10 |            |            |            |
  * 
  */

#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>

#ifndef CORE_ADC_H
#define CORE_ADC_H

#define CORE_ADC_ALLOWED_ADC1       0x0001
#define CORE_ADC_ALLOWED_ADC2       0x0002
#define CORE_ADC_ALLOWED_ADC3       0x0004
#define CORE_ADC_ALLOWED_ADC4       0x0008
#define CORE_ADC_ALLOWED_ADC5       0x0010
#define CORE_ADC_ALLOWED_OPAMP1     0x0100
#define CORE_ADC_ALLOWED_OPAMP2     0x0200
#define CORE_ADC_ALLOWED_OPAMP3     0x0400
#define CORE_ADC_ALLOWED_OPAMP4     0x0800
#define CORE_ADC_ALLOWED_OPAMP5     0x1000
#define CORE_ADC_ALLOWED_OPAMP6     0x2000
#define CORE_ADC_ALLOWED_ADC_MASK   0x001f
#define CORE_ADC_ALLOWED_OPAMP_MASK 0x3f00

typedef struct core_ADC_def_s {
    // Constant fields
    GPIO_TypeDef *port;
    uint32_t pin;
    uint16_t allowed_connections;
    uint8_t chan[5];
    uint16_t opamp_chan;
    // Fields edited by the program
    ADC_TypeDef *adc;
    OPAMP_TypeDef *opamp;
    uint8_t adc_chan_sel;
    uint8_t opamp_chan_sel;
} core_ADC_def_t;


/**
  * @brief  Initialize an ADC module, including its clock. Also performs
  *         calibration. GPIO ports are not initialized.
  * @param  adc The ADC module to initialize
  * @retval 0 if adc is not a valid ADC module or if the ADC fails to
  *         initialize
  * @retval 1 otherwise.
  */
bool core_ADC_init(ADC_TypeDef *adc);

/**
  * @brief  Set up a pin as an analog input
  * @param  port GPIO port (GPIOx)
  * @param  pin GPIO pin (GPIO_PIN_x)
  * @param  opamp 1 if the input should be routed through an opamp, 
  *         0 otherwise
  * @retval 1 if a configuration was found for the given pin
  * @retval 0 otherwise
  */
bool core_ADC_setup_pin(GPIO_TypeDef *port, uint32_t pin, uint8_t opamp);

/**
  * @brief  Read the value of an analog input as a value between 0 and 4095
  * @param  port GPIO port of the pin to be read (GPIOx)
  * @param  pin Pin number of the pin to be read (GPIO_PIN_x)
  * @param  result Location to which the result should be stored
  * @retval 0 if the given pin is not an analog input or if the corresponding
  *         ADC module is not initialized or if an error occurs while reading,
  * @retval 1 otherwise
  */
bool core_ADC_read_channel(GPIO_TypeDef *port, uint32_t pin, uint16_t *result);

/**
  * @brief  Read the value of the internal voltage reference with respect to
  *         the VREF pin
  * @return ADC measurement
  */
uint16_t core_ADC_read_vrefint();

#endif

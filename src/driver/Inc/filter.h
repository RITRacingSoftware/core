/**
 * @file  filter.h
 * @brief Core filter library
 *
 * This core library components is used to handle general-purpose digital filters.
 * The filters work via coefficient arrays, which determine the weighting that
 * previous inputs and outputs have on the current output.
 * 
 * These arrays and the corresponding information for them will be set up 
 * within the specialized init functions for each filter type.
 */


#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct core_filter_s {
    float *coeffX;          // Pointer to coefficient array for inputs. Stored most recent last
    float *coeffY;          // Pointer to coefficient array for outputs. Stored most recent last
    float *prevX;           // Pointer to circular buffer holding previous inputs
    float *prevY;           // Pointer to circular buffer holding previous outputs
    uint8_t posX;           // Current position in input circular buffer
    uint8_t posY;           // Current position in output circular buffer
    uint8_t orderX;         // How many previous X values to store
    uint8_t orderY;         // How many previous Y values to store
} core_filter_t;

/**
 * @brief  Get an output value from the filter. This is the primary function being called
 * @param  x Input to filter
 * @param  filt Filter object to apply input to
 * @return The filter output
 */
float core_filter_update(float x, core_filter_t *filt);

/**
 * @brief  Initialize a rolling average. This will apply an even coefficient 
 *         across the last (order) inputs to the filter
 * @param  order Number of inputs to consider in the rolling average
 *         (an order of 1 is no filter, the output will be the input)
 * @param  filt Filter object to apply input to
 * @return True if initialized, false if error occured
 */
bool core_filter_rolling_avg_init(uint8_t order, core_filter_t *filt);

/**
 * @brief  Initialize an exponential lowpass filter.
 *        
 * @param  a Alpha value for the filter. This represents the weighting of the current input.
 *         The weighting of the previous output is indirectly propertional to (a)
 *        
 * @param  filt Filter object to apply input to 
 * @return True if initialized, false if error occured
 */
bool core_filter_exp_lowpass_init(float a, core_filter_t *filt);

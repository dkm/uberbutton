#pragma once

#include <stdint.h>
#include "periph/gpio.h"

/**
 * Maximum number of seven segments per chip
 */
#define MAX_SEGMENTS_PER_MM545X	4

/**
 * Number of segment per display
 */
#define SEGMENT_COUNT	8

typedef struct {
  gpio_t clock_pin;
  gpio_t data_pin;
  uint8_t sevSegPins[MAX_SEGMENTS_PER_MM545X][SEGMENT_COUNT];
  uint8_t sevSegValue[MAX_SEGMENTS_PER_MM545X];
} mm545x_t;

int mm545x_init(mm545x_t *mm, gpio_t clock_pin, gpio_t data_pin);

/**
 * Set the mask for all leds, useful when working with other leds than 7 segments
 * @param bits A bitmask of all output (only the first 35 bits are significant)
 * 		The lower bit correspond to output 0.
 */
void mm545x_setLeds(mm545x_t *mm, uint64_t bits);
	
/**
 *  When using the chip as a seven segment controller, setup segments
 * @param sevSeg Seven segment display index
 * @param pins, pin mapping for the seven segment
 * 		Pins are given in the following order: [a, b, c, d ,e , f, g, dp]
 */
void mm545x_setupSegment(mm545x_t *mm, int sevSeg, uint8_t pins[SEGMENT_COUNT]);
	
/**
 * Set a segment value
 * @param sevSeg Seven sigment display index
 * @param value Character value to display 
 */
void mm545x_setSegment(mm545x_t *mm, int sevSeg, char value);
	
/**
 * Set a raw value for a segment
 * @param sevSeg Seven sigment display index
 * @param segMask Mask of segment to set (a, b, c, d ,e , f, g, dp)
 * 		Note that the lower bit correspond to segment a.
 */
void mm545x_setSegmentRaw(mm545x_t *mm, int sevSeg, uint8_t segMask);
	
/**
 *  Update the display and apply the previously set segments value. 
 */
void mm545x_refreshSegments(mm545x_t *mm);

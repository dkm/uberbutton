#include <string.h>
#include "mm545x.h"
#include "xtimer.h"
/**
 * Delay between clock transitions
 */
#define  CLOCK_USEC_DELAY	2

/**
 *  Count of output bits
 */
#define  OUTPUT_BIT_COUNT		35

static uint8_t digitPattern[] = {
        0x3F,     /* 0 */
        0x06,     /* 1 */
        0x5B,     /* 2 */
        0x4F,     /* 3 */
        0x66,     /* 4 */
        0x6D,     /* 5 */
        0x7D,     /* 6 */
        0x07,     /* 7 */
        0x7F,     /* 8 */
        0x6F,     /* 9 */
};

static uint8_t alphaPattern[] = {
        0b01110111,     /* a */
        0b01111100,     /* b */
        0b00111001,     /* c */
        0b01011110,     /* d */
        0b01111001,     /* e */
        0b01110001,     /* f */
        0b01101111,     /* g */
        0b01110110,     /* h */
        0b00110000,     /* i */
        0b00011110,     /* j */
        0b01110110,     /* k */
        0b00111000,     /* l */
        0b00010101,     /* m */
        0b01010100,     /* n */
        0b00111111,     /* o */
        0b01110011,     /* p */
        0b01100111,     /* q */
        0b01010000,     /* r */
        0b01101101,     /* s */
        0b01111000,     /* t */
        0b00111110,     /* u */
        0b00011100,     /* v */
        0b00101010,     /* w */
        0b01110110,     /* x */
        0b01101110,     /* y */
        0b01011011,     /* z */
};

#define COMMA 		0b10000000
#define DASH 		0b01000000
#define UNDERSCORE 	0b00001000
#define QUOTE 		0b00100000
#define EQUAL 		0b01001000


int mm545x_init(mm545x_t *mm, gpio_t clock_pin, gpio_t data_pin){
        int i;
	mm->clock_pin = clock_pin;
	mm->data_pin = data_pin;
	
	gpio_init(clock_pin, GPIO_DIR_OUT, GPIO_NOPULL);
        gpio_init(data_pin, GPIO_DIR_OUT, GPIO_NOPULL);
	
        gpio_clear(clock_pin);
        gpio_clear(data_pin);
        
        for (i = 0; i < MAX_SEGMENTS_PER_MM545X; i++) {
                memset(mm->sevSegPins[i], 0, SEGMENT_COUNT);
                mm->sevSegValue[i] = 0;
        }
        
        /* reset the leds */
        mm545x_setLeds(mm, (uint64_t) 0);
	return 0;
}

void mm545x_setLeds(mm545x_t *mm, uint64_t leds) {
        int i;

	/* Send the preamble */
        gpio_set(mm->data_pin);
        gpio_clear(mm->clock_pin);
        xtimer_usleep(CLOCK_USEC_DELAY);
        gpio_set(mm->clock_pin);

        /* Then output the bits */
        for (i = 0; i < OUTPUT_BIT_COUNT; i++) {
                xtimer_usleep(CLOCK_USEC_DELAY);
                if ((leds >> i) & 0x1) {
		  gpio_set(mm->data_pin);
		} else {
		  gpio_clear(mm->data_pin);
		}
                gpio_clear(mm->clock_pin);
                xtimer_usleep(CLOCK_USEC_DELAY);
                gpio_set(mm->clock_pin);
        }

        xtimer_usleep(CLOCK_USEC_DELAY);
        gpio_clear(mm->clock_pin);
}

void mm545x_setupSegment(mm545x_t *mm, int sevSeg, uint8_t pins[SEGMENT_COUNT])
{
        memcpy(mm->sevSegPins[sevSeg], pins, SEGMENT_COUNT);
}

void mm545x_setSegment(mm545x_t *mm, int sevSeg, char value)
{
        if (value >= '0' && value <= '9') {
	  mm545x_setSegmentRaw(mm, sevSeg, digitPattern[value - '0']);
        } else if (value >= 'a' && value <= 'z') {
	  mm545x_setSegmentRaw(mm, sevSeg, alphaPattern[value - 'a']);
        } else if (value >= 'A' && value <= 'Z') {
	  mm545x_setSegmentRaw(mm, sevSeg, alphaPattern[value - 'A']);
	} else if (value == ' ') {
	  mm545x_setSegmentRaw(mm, sevSeg, 0x0);
	} else if (value == ',' || value == ';' || value == '.') {
	  mm545x_setSegmentRaw(mm, sevSeg, COMMA);
	} else if (value == '_') {
	  mm545x_setSegmentRaw(mm, sevSeg, UNDERSCORE);
	} else if (value == '-') {
	  mm545x_setSegmentRaw(mm, sevSeg, DASH);
	} else if (value == '\'') {
	  mm545x_setSegmentRaw(mm, sevSeg, QUOTE);
	} else if (value == '=') {
	  mm545x_setSegmentRaw(mm, sevSeg, QUOTE);
	}
}

void mm545x_setSegmentRaw(mm545x_t *mm, int sevSeg, uint8_t segMask)
{
        mm->sevSegValue[sevSeg] = segMask;
}

void mm545x_refreshSegments(mm545x_t *mm)
{
        uint64_t value = 0;
        int sevSeg, seg;
        for (sevSeg = 0; sevSeg < MAX_SEGMENTS_PER_MM545X; sevSeg++) {
                for (seg = 0; seg < SEGMENT_COUNT; seg++) {
                        /* If the bit is set in the value, then set the pin as high */
                        if ((mm->sevSegValue[sevSeg] >> seg) & 0x1) {
                                value |= ((uint64_t) 0x1 << (uint64_t) mm->sevSegPins[sevSeg][seg]);
                        }
                }
        }

        mm545x_setLeds(mm, value);
}

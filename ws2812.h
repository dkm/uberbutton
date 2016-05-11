#pragma once

#include "periph/spi.h"

typedef struct {
  spi_t spi;
} ws2812_t;

typedef struct __attribute__((packed)) {
  char g,r,b;
} ws2812_rgb_t;

#define BLUE {0x00, 0x00, 0xFF}
#define RED {0x00, 0xFF, 0x00}
#define GREEN {0xFF, 0x00, 0x00}
#define OFF {0,0,0}
int ws2812_init(ws2812_t *dev, spi_t spi);
int ws2812_write(ws2812_t *dev, char *b, unsigned len);
int ws2812_write_rgb(ws2812_t *dev, ws2812_rgb_t *leds, unsigned len, char* buffer);
void ws2812_fill_rgb(ws2812_rgb_t *leds, unsigned len, char* buffer);

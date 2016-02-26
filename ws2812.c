#include "ws2812.h"
#include "periph/spi.h"
#include <stdio.h>

#define log printf("%d\n", __LINE__)

int ws2812_init(ws2812_t *dev, spi_t spi){
  
  dev->spi = spi;
  log;
  /* Init SPI */
  spi_poweron(dev->spi);
  log;
  spi_acquire(dev->spi);
  log;
  int status = spi_init_master(dev->spi, SPI_CONF_FIRST_RISING, SPI_SPEED_4MHZ);
  log;
  spi_release(dev->spi);
  log;
  return status;
}

int ws2812_write(ws2812_t *dev, char *b, unsigned len) {
  static char reset_pulse[20] = {0};

  spi_acquire(dev->spi);
  /* int i = 0; */
  /* for (i=0; i<len; i++) */
  /*   printf("%x.", b[i]); */
  /* printf("\n"); */
  spi_transfer_bytes(dev->spi, b, NULL, len);

  spi_transfer_bytes(dev->spi, reset_pulse, NULL, sizeof(reset_pulse));
  spi_release(dev->spi);
  return 0;
}

#define ZERO_BIT_PULSE 0x8 // 1000 (0000)  ~250ns
#define ONE_BIT_PULSE 0xE //  1110 (0000)  ~750ns

#define W00 0x88
#define W01 0x8E
#define W10 0xE8
#define W11 0xEE
  
int ws2812_write_rgb(ws2812_t *dev, ws2812_rgb_t *leds, unsigned len, char* buffer){
  int out_count = 0;
  char *bytes = (char*)leds;
  int led_idx;

  for (led_idx = (len*sizeof(ws2812_rgb_t))-1; led_idx >= 0; led_idx--){
    int bit_idx;
    unsigned char cur_byte = bytes[led_idx];

    for (bit_idx = 0; bit_idx < 8; bit_idx+=2){
      switch((cur_byte & 0xC0)>>6) {
      case 0x0:
	buffer[out_count] = W00;
	break;
      case 0x1:
	buffer[out_count] = W01;
	break;
      case 0x2:
	buffer[out_count] = W10;
	break;
      case 0x3:
	buffer[out_count] = W11;
	break;
      }
      cur_byte<<2;
      /* buffer[out_count] = cur_byte & 0x80 ? ONE_BIT_PULSE : ZERO_BIT_PULSE; */
      /* cur_byte <<=1; */

      /* buffer[out_count] |= (cur_byte & 0x80 ? ONE_BIT_PULSE : ZERO_BIT_PULSE)<<4; */
      /* cur_byte >>=1; */

      out_count++;
    }
  }
  return ws2812_write(dev, buffer, len*sizeof(ws2812_rgb_t)*8/2);
}

#pragma once

#define SSI_UDMA_FINISHED 2

typedef struct {
  unsigned int listener;
  unsigned int state;
  uint8_t *data;
  uint16_t data_len;
  uint8_t *done;
} ssi_udma_t;

void ssi_udma_restart_tx(ssi_udma_t *dev);
void ssi_udma_register(ssi_udma_t *dev, unsigned int pid);

void ssi_udma_init(ssi_udma_t *dev, uint8_t *pui8SPIData, uint16_t ui16DataSize,
		   uint8_t *pui8DoneVar, unsigned int pid);

APPLICATION = uber_button
RIOTBASE = ../RIOT/

FEATURES_REQUIRED = periph_spi

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += xtimer
USEMODULE += nrf24l01p

# set default device parameters in case they are undefined
SPI_PORT ?= SPI_0
CE_PIN   ?= GPIO_PIN\(PORT_A,6\)
CS_PIN   ?= GPIO_PIN\(PORT_E,3\)
IRQ_PIN  ?= GPIO_PIN\(PORT_A,7\)

# export parameters
CFLAGS += -DSPI_PORT=$(SPI_PORT)
CFLAGS += -DCE_PIN=$(CE_PIN)
CFLAGS += -DCS_PIN=$(CS_PIN)
CFLAGS += -DIRQ_PIN=$(IRQ_PIN)
##CFLAGS += -I$(HOME)/git/ubberFrame

include $(RIOTBASE)/Makefile.include

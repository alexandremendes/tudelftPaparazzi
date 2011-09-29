#ifndef OVERO_STM_SPI_DUPLEX_H
#define OVERO_STM_SPI_DUPLEX_H

#include <inttypes.h>

typedef struct __attribute__ ((packed)) {
	int x, y, area;
} overo_msg_rx_t;

typedef struct __attribute__ ((packed)) {
	int a, b, c;
} overo_msg_tx_t;

extern overo_msg_rx_t overo_msg_rx;
extern uint8_t overo_msg_available;
extern overo_msg_tx_t overo_msg_tx;

void init_overo_stm_spi_duplex(void);
void event_overo_stm_spi_duplex(void);

// Consumer example. Should be disabled when using overo_msg somewhere else
void periodic_70Hz_overo_stm_spi_duplex(void);

#endif

#ifndef OVERO_STM_SPI_SIMPLEX_H
#define OVERO_STM_SPI_SIMPLEX_H

#include <inttypes.h>

typedef struct __attribute__ ((packed)) {
	int x, y, area;
} overo_msg_t;

extern overo_msg_t overo_msg;
extern uint8_t overo_msg_available;

void init_overo_stm_spi_simplex(void);
void event_overo_stm_spi_simplex(void);

// Consumer example. Should be disabled when using overo_msg somewhere else
void periodic_70Hz_overo_stm_spi_simplex(void);

#endif

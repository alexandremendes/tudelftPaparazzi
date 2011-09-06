#include "overo_stm_spi_simplex.h"
#include "led.h" // DEBUG

#include <stm32/rcc.h>
#include <stm32/exti.h>
#include <stm32/flash.h>
#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/spi.h>
#include <stm32/dma.h>
#include <string.h>

// Circular buffer 
#define CB_SIZE		128 // power of 2 < 255
uint8_t cb[CB_SIZE]; 
uint8_t cb_optr; // cb_iptr is provided by DMA

// Protocol specifics 
#define SYNC_1				0xB5
#define SYNC_2				0x62
#define PAYLOAD_SIZE	(sizeof(overo_msg_t))

// Parser status 
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_PAYLOAD   3
#define GOT_CHECKSUM1 4

// Parser engine
struct {
  uint8_t msg_buf[PAYLOAD_SIZE];

  uint8_t status;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
	int msg_cnt;
  uint8_t error_cnt;
} spi_parser;
void spi_parse(uint8_t c);

// Received struct
overo_msg_t overo_msg;
uint8_t overo_msg_available = FALSE;

const DMA_InitTypeDef DMA_InitStructure_rx = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI1_BASE+0x0C),
    .DMA_MemoryBaseAddr     = (uint32_t)cb,
    .DMA_DIR                = DMA_DIR_PeripheralSRC,
    .DMA_BufferSize         = CB_SIZE,
    .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc          = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
    .DMA_Mode               = DMA_Mode_Circular,
    .DMA_Priority           = DMA_Priority_VeryHigh,
    .DMA_M2M                = DMA_M2M_Disable
  };

// Log results received from images processor counter
uint16_t dc_photo_nr = 0;
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

void init_overo_stm_spi_simplex(void) {
	/* DEBUG */	
	LED_INIT(2);
	LED_INIT(3);
	LED_INIT(4);

	/* Circular buffer */	
	cb_optr = 0;

	/* Parser engine */
	spi_parser.status = UNINIT;
	spi_parser.msg_idx = 0;
	spi_parser.error_cnt = 0;
	spi_parser.msg_cnt = 0;

	/* System clocks configuration ----------------------------------------------*/
  /* Enable DMA1 clock --------------------------------------------------------*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* Enable SPI1 Periph clock -------------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* GPIO configuration -------------------------------------------------------*/
  /* Configure GPIOs: NSS, SCK, MISO and MOSI  --------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* SPI_SLAVE_Rx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel2);
  DMA_Init(DMA1_Channel2, (DMA_InitTypeDef*)&DMA_InitStructure_rx);
  /* Enable DMA1 Channel2 */
  DMA_Cmd(DMA1_Channel2, ENABLE);

  /* SPI_SLAVE configuration --------------------------------------------------*/
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction         = SPI_Direction_1Line_Rx;
  SPI_InitStructure.SPI_Mode              = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS               = SPI_NSS_Hard;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial     = 0x31;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* enable SPI */
  SPI_Cmd(SPI1, ENABLE);
  /* Enable SPI1 Rx request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

	//LED_ON(2);
}

void event_overo_stm_spi_simplex(void) {
	while(cb_optr != (CB_SIZE - DMA_GetCurrDataCounter(DMA1_Channel2)))	{
		spi_parse(cb[cb_optr++]);
		cb_optr &= (CB_SIZE - 1);
	}
}

void spi_parse(uint8_t c) {
	if (spi_parser.status < GOT_PAYLOAD) {
		  spi_parser.ck_a += c;
		  spi_parser.ck_b += spi_parser.ck_a;
	}
	switch (spi_parser.status) {
		case UNINIT:
			if (c == SYNC_1)
			  spi_parser.status++;
			break;
		case GOT_SYNC1:
			if (c != SYNC_2)
			  goto error;
			spi_parser.ck_a = 0;
			spi_parser.ck_b = 0;
			spi_parser.msg_idx = 0;
			spi_parser.status++;
			break;
		case GOT_SYNC2:
			if (overo_msg_available) {
			  /* Previous message has not yet been parsed: discard this one */
				//spi_parser.error_cnt++;
			  goto error;
			}
			spi_parser.msg_buf[spi_parser.msg_idx] = c;
			spi_parser.msg_idx++;
			if (spi_parser.msg_idx >= PAYLOAD_SIZE) {
			  spi_parser.status++;
			}
			break;
		case GOT_PAYLOAD:
			if (c != spi_parser.ck_a) {
			  goto error;
			}
			spi_parser.status++;
			break;
		case GOT_CHECKSUM1:
			if (c != spi_parser.ck_b) {
			  goto error;
			}
			memcpy(&overo_msg, spi_parser.msg_buf, PAYLOAD_SIZE); // copy to user space
			overo_msg_available = TRUE;
			spi_parser.msg_cnt++;
			goto restart;
			break;
		default:
			goto error;
	}
	return;
 error:

	spi_parser.error_cnt++;
 restart:
	spi_parser.status = UNINIT;
	return;
}

void periodic_70Hz_overo_stm_spi_simplex(void) {
													// seems to be running at more than 70Hz =(
	static int timeout = 0; // reseting dc_photo_nr if no processing result
													// is received for ~10s

	// DEBUG	
	if(spi_parser.msg_cnt == 10)
		LED_ON(4);	
	if(spi_parser.error_cnt > 0) 
		{LED_ON(3);}
	else 
		LED_OFF(3);

	if (overo_msg_available == TRUE) {
		overo_msg_available = FALSE;
		DOWNLINK_SEND_CAMERA_SNAPSHOT(DefaultChannel, &dc_photo_nr);
		dc_photo_nr++;
		timeout = 0;
	}

	if (timeout >= 70*10)
		dc_photo_nr = 0;
	timeout++;
}

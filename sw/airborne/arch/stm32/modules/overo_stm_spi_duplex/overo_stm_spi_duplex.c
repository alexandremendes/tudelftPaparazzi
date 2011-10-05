#include "overo_stm_spi_duplex.h"
#include "led.h" // DEBUG

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/misc.h>
#include <stm32/spi.h>
#include <stm32/dma.h>

#include <string.h>

/*-- User interface --*/
	// Received struct
	overo_msg_rx_t 	overo_msg_rx;
	uint8_t 	overo_msg_available = FALSE;
	// Transmitted struct
	overo_msg_tx_t 	overo_msg_tx;
/*-- --*/

/*-- Communication buffers --*/
#define BUFSIZE sizeof(overo_msg_rx_t)
uint8_t buf_in[BUFSIZE];
uint8_t buf_out[BUFSIZE];
/*-- --*/

/*-- Platform configuration --*/
#define SPI_SLAVE                   SPI1
#define SPI_SLAVE_CLK               RCC_APB2Periph_SPI1
#define SPI_SLAVE_GPIO              GPIOA
#define SPI_SLAVE_GPIO_CLK          RCC_APB2Periph_GPIOA 
#define SPI_SLAVE_PIN_NSS	    GPIO_Pin_4 
#define SPI_SLAVE_PIN_SCK           GPIO_Pin_5
#define SPI_SLAVE_PIN_MISO          GPIO_Pin_6
#define SPI_SLAVE_PIN_MOSI          GPIO_Pin_7
#define SPI_SLAVE_DMA               DMA1
#define SPI_SLAVE_DMA_CLK           RCC_AHBPeriph_DMA1  
#define SPI_SLAVE_Rx_DMA_Channel    DMA1_Channel2
#define SPI_SLAVE_Rx_DMA_FLAG       DMA1_FLAG_TC2
#define SPI_SLAVE_Tx_DMA_Channel    DMA1_Channel3
#define SPI_SLAVE_Tx_DMA_FLAG       DMA1_FLAG_TC3  
#define SPI_SLAVE_DR_Base           0x4001300C 

SPI_InitTypeDef    SPI_InitStructure;
DMA_InitTypeDef    DMA_InitStructure_rx, DMA_InitStructure_tx;
void RCC_Configuration(void);
void GPIO_Configuration(void);
/*-- --*/

void RCC_Configuration(void)
{
  /* Enable peripheral clocks ------------------------------------------------*/
  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(SPI_SLAVE_DMA_CLK, ENABLE);
  /* Enable GPIO clock for SPI_SLAVE */
  RCC_APB2PeriphClockCmd(SPI_SLAVE_GPIO_CLK, ENABLE);
  /* Enable SPI_SLAVE Periph clock */
  RCC_APB2PeriphClockCmd(SPI_SLAVE_CLK, ENABLE);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; // SPI bus speed definition
  /* Configure SPI_SLAVE pins: SCK and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI_SLAVE_PIN_NSS | SPI_SLAVE_PIN_SCK | SPI_SLAVE_PIN_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SPI_SLAVE_GPIO, &GPIO_InitStructure);
  /* Configure SPI_SLAVE pins: MISO  */
  GPIO_InitStructure.GPIO_Pin = SPI_SLAVE_PIN_MISO ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(SPI_SLAVE_GPIO, &GPIO_InitStructure);
}

void init_overo_stm_spi_duplex(void) {
	/* DEBUG */	
	LED_INIT(2);

	RCC_Configuration();
	GPIO_Configuration();	
	
	/* SPI_SLAVE configuration ------------------------------------------------*/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	// SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // for Master
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_SLAVE, &SPI_InitStructure);

	/* SPI_SLAVE_Rx_DMA_Channel configuration ----------------------------------*/
	DMA_DeInit(SPI_SLAVE_Rx_DMA_Channel);  
	DMA_InitStructure_rx.DMA_PeripheralBaseAddr = (uint32_t)SPI_SLAVE_DR_Base;
	DMA_InitStructure_rx.DMA_MemoryBaseAddr = (uint32_t)buf_in;
	DMA_InitStructure_rx.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure_rx.DMA_BufferSize = BUFSIZE;
	DMA_InitStructure_rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure_rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_rx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure_rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure_rx.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure_rx.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure_rx.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(SPI_SLAVE_Rx_DMA_Channel, &DMA_InitStructure_rx);

	/* SPI_SLAVE_Tx_DMA_Channel configuration ----------------------------------*/
	memcpy(&DMA_InitStructure_tx,&DMA_InitStructure_rx,sizeof(DMA_InitTypeDef));
	DMA_DeInit(SPI_SLAVE_Tx_DMA_Channel);  
	DMA_InitStructure_tx.DMA_PeripheralBaseAddr = (uint32_t)SPI_SLAVE_DR_Base;
	DMA_InitStructure_tx.DMA_MemoryBaseAddr = (uint32_t)buf_out;
	DMA_InitStructure_tx.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure_tx.DMA_Priority = DMA_Priority_Medium;
	DMA_Init(SPI_SLAVE_Tx_DMA_Channel, &DMA_InitStructure_tx);

	/* Enable SPI_SLAVE DMA Tx request */
	SPI_I2S_DMACmd(SPI_SLAVE, SPI_I2S_DMAReq_Tx, ENABLE);
	/* Enable SPI_SLAVE DMA Rx request */
	SPI_I2S_DMACmd(SPI_SLAVE, SPI_I2S_DMAReq_Rx, ENABLE);

	/* Enable SPI_SLAVE CRC calculation */
	// SPI_CalculateCRC(SPI_SLAVE, ENABLE);

	/* Enable DMA channels */
	DMA_Cmd(SPI_SLAVE_Rx_DMA_Channel, ENABLE);
	DMA_Cmd(SPI_SLAVE_Tx_DMA_Channel, ENABLE);
	/* Enable SPI_SLAVE */
	SPI_Cmd(SPI_SLAVE, ENABLE);
}

void event_overo_stm_spi_duplex(void) {
	// incoming buffer filled and old message read?
	if(DMA_GetFlagStatus(SPI_SLAVE_Rx_DMA_FLAG) && (overo_msg_available == FALSE) ) {
		memcpy((uint8_t*)&overo_msg_rx, buf_in, BUFSIZE); // copy incoming message to user space
		memcpy(buf_out, (uint8_t*)&overo_msg_tx, BUFSIZE); // copy from user space; prepare to be clocked out
		overo_msg_available = TRUE;			  // signal new message
	
		/* Reset devices */		
		DMA_Cmd(SPI_SLAVE_Rx_DMA_Channel, DISABLE);
		DMA_ClearFlag(SPI_SLAVE_Rx_DMA_FLAG);	
		SPI_SLAVE_Rx_DMA_Channel->CNDTR = DMA_InitStructure_rx.DMA_BufferSize;

		DMA_Cmd(SPI_SLAVE_Tx_DMA_Channel, DISABLE);
		DMA_ClearFlag(SPI_SLAVE_Tx_DMA_FLAG);	
		SPI_SLAVE_Tx_DMA_Channel->CNDTR = DMA_InitStructure_tx.DMA_BufferSize;

		DMA_Cmd(SPI_SLAVE_Rx_DMA_Channel, ENABLE);
		DMA_Cmd(SPI_SLAVE_Tx_DMA_Channel, ENABLE);
	}
}

// ATT info
#include "subsystems/ahrs.h"
// Paparazzi Logging 
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

void periodic_70Hz_overo_stm_spi_duplex(void) {
	static uint16_t dc_photo_nr = 0; // last processed image id	
	static int timeout = 0; // reseting dc_photo_nr if no processing result is received for ~5s

	if (overo_msg_available == TRUE) {
		LED_TOGGLE(2);
		
		// Log image processing result		
		// DOWNLINK_SEND_CAMERA_SNAPSHOT(DefaultChannel, &dc_photo_nr);
		// DOWNLINK_SEND_CAMERA_BLOB_X(DefaultChannel, &overo_msg_tx.phi);
		// DOWNLINK_SEND_CAMERA_BLOB_Y(DefaultChannel, &overo_msg_tx.theta);
		// DOWNLINK_SEND_CAMERA_BLOB_AREA(DefaultChannel, &overo_msg_rx.area);
		
		// Populate next to be sent message
		overo_msg_tx.phi = ahrs.ltp_to_imu_euler.phi; // angles in rad with 12 FP bits 
		overo_msg_tx.theta = ahrs.ltp_to_imu_euler.theta;

		// Signal message as read
		overo_msg_available = FALSE;
		
		dc_photo_nr++;
		timeout = 0;
	}

	if (timeout >= 70*5) 
		dc_photo_nr = 0;
	
	timeout++;
}

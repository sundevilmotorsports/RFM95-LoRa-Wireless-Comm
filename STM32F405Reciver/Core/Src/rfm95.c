/*
 * rfm95.c
 *
 *  Created on: Apr 30, 2025
 *      Author: Elio
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "rfm95.h"
#include "console.h"
/* Includes END --------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
#define RFM95_REG_FIFO_ACCESS			0x00
#define RFM95_REG_OP_MODE				0x01
#define RFM95_REG_FR_MSB				0x06
#define RFM95_REG_FR_MID				0x07
#define RFM95_REG_FR_LSB				0x08
#define RFM95_REG_PA_CONFIG				0x09
#define RFM95_REG_LNA					0x0C
#define RFM95_REG_FIFO_ADDR_PTR			0x0D
#define RFM95_REG_FIFO_TX_BASE_ADDR		0x0E
#define RFM95_REG_FIFO_RX_BASE_ADDR		0x0F
#define RFM95_REG_IRQ_FLAGS0			0x12
#define RFM95_REG_FIFO_RX_BYTES_NB		0x13
#define RFM95_REG_PACKET_SNR			0x19
#define RFM95_REG_MODEM_CONFIG_1		0x1D
#define RFM95_REG_MODEM_CONFIG_2		0x1E
#define RFM95_REG_SYMB_TIMEOUT_LSB		0x1F
#define RFM95_REG_PREAMBLE_MSB			0x20
#define RFM95_REG_PREAMBLE_LSB			0x21
#define RFM95_REG_PAYLOAD_LENGTH		0x22
#define RFM95_REG_MAX_PAYLOAD_LENGTH	0x23
#define RFM95_REG_MODEM_CONFIG_3		0x26
#define RFM95_REG_INVERT_IQ_1			0x33
#define RFM95_REG_SYNC_WORD				0x39
#define RFM95_REG_INVERT_IQ_2			0x3B
#define RFM95_REG_DIO_MAPPING_1			0x40
#define RFM95_REG_VERSION				0x42
#define RFM95_REG_PA_DAC				0x4D

#define RFM95_WRITE_BUFF_LEN			2
#define RFM95_SPI_TIMEOUT 				1000
/* Defines END ---------------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/

/* Typedefs END --------------------------------------------------------------*/


/* Local Variables -----------------------------------------------------------*/
static rfm95_handle_t rfm95_handle;
/* Local Variables END -------------------------------------------------------*/


/* Local Functions -----------------------------------------------------------*/
bool read_register( rfm95_handle_t *handle, uint8_t reg, uint8_t *buffer, size_t length )
{
	/* Function to read a register from RFM95 via SPI (SINGLE access)
	   Datasheet section 4.3 */

	uint8_t transmit_buffer;

	// Set CS to low
	HAL_GPIO_WritePin( handle->nss_port, handle->nss_pin, GPIO_PIN_RESET );

	// MSB is 0 for read access
	transmit_buffer = reg & 0x7Fu;

	// Transmit SPI message
	if( HAL_SPI_Transmit( handle->spi_handle, &transmit_buffer, 1, RFM95_SPI_TIMEOUT ) != HAL_OK )
	{
		return false;
	}

	// Receive SPI response
	if( HAL_SPI_Receive( handle->spi_handle, buffer, length, RFM95_SPI_TIMEOUT ) != HAL_OK )
	{
		return false;
	}

	// CS back to high
	HAL_GPIO_WritePin( handle->nss_port, handle->nss_pin, GPIO_PIN_SET );

	return true;
}


bool write_register( rfm95_handle_t *handle, uint8_t reg, uint8_t value )
{
	/* Function to write to a register via SPI (SINGLE access)
	   Datasheet section 4.3 */

	uint8_t transmit_buffer[ RFM95_WRITE_BUFF_LEN ];

	HAL_GPIO_WritePin( handle->nss_port, handle->nss_pin, GPIO_PIN_RESET );

	// First byte is the register address, second byte is the value to write
	// MSB is 1 for write access
	transmit_buffer[ 0 ] =  reg | 0x80u;
	transmit_buffer[ 1 ] = value;

	if( HAL_SPI_Transmit( handle->spi_handle, transmit_buffer, 2, RFM95_SPI_TIMEOUT ) != HAL_OK )
	{
		return false;
	}

	HAL_GPIO_WritePin( handle->nss_port, handle->nss_pin, GPIO_PIN_SET );

	return true;
}
/* Local Functions END -------------------------------------------------------*/


/* Global Functions ----------------------------------------------------------*/
void RFM95_Init( SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *nss_port, uint16_t nss_pin )
{
	rfm95_handle.spi_handle = spi_handle;
	rfm95_handle.nss_port = nss_port;
	rfm95_handle.nss_pin = nss_pin;
}
/* Global Functions END ------------------------------------------------------*/

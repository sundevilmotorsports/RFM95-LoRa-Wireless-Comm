/*
 * rfm95.c
 *
 *  Created on: Apr 30, 2025
 *      Author: Elio
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "rfm95.h"
#include "console.h"
/* Includes END --------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
// Register list
#define RFM95_REG_FIFO_ACCESS			0x00
#define RFM95_REG_OP_MODE				0x01
#define RFM95_REG_FR_MSB				0x06
#define RFM95_REG_FR_MID				0x07
#define RFM95_REG_FR_LSB				0x08
#define RFM95_REG_PA_CONFIG				0x09
#define RFM95_REG_LNA					0x0C
#define RFM95_REG_FIFO_ADDR_PTR			0x0D
#define RFM95_REG_FIFO_TX_BASE			0x0E
#define RFM95_REG_FIFO_RX_BASE			0x0F
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

// Register values
#define RFM95_REG_OP_MODE_SLEEP       			0x00
#define RFM95_REG_OP_MODE_LORA_SLEEP        	0x80
#define RFM95_REG_OP_MODE_LORA_STANDBY      	0x81
#define RFM95_REG_OP_MODE_LORA_TX           	0x83

#define RFM95_REG_DIO_MAPPING_1_DIO0_TXDONE		0x40

#define RFM95_REG_PA_DAC_LOW_POWER              0x84
#define RFM95_REG_PA_DAC_HIGH_POWER            	0x87

#define RFM95_REG_FIFO_RX_ADDR					0x00
#define RFM95_REG_FIFO_TX_ADDR					0x80


// Other defines
#define RFM95_WRITE_BUFF_LEN			2
#define RFM95_SPI_TIMEOUT 				1000

#define FALSE	0x00
#define TRUE	0x01
/* Defines END ---------------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/

/* Typedefs END --------------------------------------------------------------*/


/* Local Declarations --------------------------------------------------------*/
// Variables
static rfm95_handle_t rfm95_handle1;

// Functions
static uint8_t read_register( rfm95_handle_t *handle, uint8_t register, uint8_t *buffer, size_t length );
static uint8_t write_register( rfm95_handle_t* handle, uint8_t reg, uint8_t value );
static uint8_t set_power( rfm95_handle_t *handle, uint8_t power );
/* Local Declarations END ----------------------------------------------------*/


/* Local Functions -----------------------------------------------------------*/
static uint8_t read_register( rfm95_handle_t *handle, uint8_t reg, uint8_t *buffer, size_t length )
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
		return FALSE;
	}

	// Receive SPI response
	if( HAL_SPI_Receive( handle->spi_handle, buffer, length, RFM95_SPI_TIMEOUT ) != HAL_OK )
	{
		return FALSE;
	}

	// CS back to high
	HAL_GPIO_WritePin( handle->nss_port, handle->nss_pin, GPIO_PIN_SET );

	return TRUE;
}


static uint8_t write_register( rfm95_handle_t *handle, uint8_t reg, uint8_t value )
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
		return FALSE;
	}

	HAL_GPIO_WritePin( handle->nss_port, handle->nss_pin, GPIO_PIN_SET );

	return TRUE;
}


static uint8_t set_power( rfm95_handle_t *handle, uint8_t power )
{
	uint8_t pa_config;
	uint8_t pa_dac;
	uint8_t status = 0;

	if ( power >= 2 && power <= 17 )
	{
		pa_config = 0xF0 | ( power - 2 );
		pa_dac = RFM95_REG_PA_DAC_LOW_POWER;
	}
	else if ( 20 == power )
	{
		pa_config = 0xF0 | 15;
		pa_dac = RFM95_REG_PA_DAC_HIGH_POWER;
	}
	else
	{
		CONSOLE_Printf( "RFM95_set_power(): power should be between 2 and 17dBm or 20dBm\r\n" );
		return FALSE;
	}

	status |= write_register( handle, RFM95_REG_PA_CONFIG, pa_config );
	status |= write_register( handle, RFM95_REG_PA_DAC, pa_dac );
	return status;
}
/* Local Functions END -------------------------------------------------------*/


/* Global Functions ----------------------------------------------------------*/
uint8_t RFM95_Init( SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *nss_port, uint16_t nss_pin )
{
	// All the steps of this function can be included in a for loop that iterates over an array of rfm95_handle_t for all the chips
	// Module power, should be between 2 and 17 or equal to 20
	uint8_t power = 17;

	// Add the SPI details for the handle
	rfm95_handle1.spi_handle = spi_handle;
	rfm95_handle1.nss_port = nss_port;
	rfm95_handle1.nss_pin = nss_pin;

	// Can start by reading the version, not necessary now

	// Module must be placed in sleep mode before switching to LoRA.
	if ( !write_register( &rfm95_handle1, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_SLEEP ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d, %d\r\n", RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_SLEEP );
		return FALSE;
	}

	if ( !write_register( &rfm95_handle1, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d, %d\r\n", RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP );
		return FALSE;
	}

	// Set DIO0 to TXDone
	if ( !write_register( &rfm95_handle1, RFM95_REG_DIO_MAPPING_1, RFM95_REG_DIO_MAPPING_1_DIO0_TXDONE ))
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d\r\n", RFM95_REG_DIO_MAPPING_1 );
		return FALSE;
	}

	// Set the power of the module in dBm. Can change it by changing the variable power
	if ( !set_power( &rfm95_handle1, power ) )
	{
		CONSOLE_Printf( "RFM_Init(): failed to set power\r\n" );
		return FALSE;
	}

	// Set up TX and RX FIFO base addresses
	if ( !write_register( &rfm95_handle1, RFM95_REG_FIFO_TX_BASE, RFM95_REG_FIFO_TX_ADDR ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d\r\n", RFM95_REG_FIFO_TX_BASE );
		return FALSE;
	}

	if ( !write_register( &rfm95_handle1, RFM95_REG_FIFO_RX_BASE, RFM95_REG_FIFO_RX_ADDR ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d\r\n", RFM95_REG_FIFO_RX_BASE );
		return FALSE;
	}

	// Set max payload length to 64
	if ( !write_register( &rfm95_handle1, RFM95_REG_MAX_PAYLOAD_LENGTH, 64 ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d\r\n", RFM95_REG_MAX_PAYLOAD_LENGTH );
		return FALSE;
	}

	// Let module sleep after initialization
	if ( !write_register( &rfm95_handle1, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed 2 %d, %d\r\n", RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP );
		return FALSE;
	}

	return TRUE;
}

/* Global Functions END ------------------------------------------------------*/

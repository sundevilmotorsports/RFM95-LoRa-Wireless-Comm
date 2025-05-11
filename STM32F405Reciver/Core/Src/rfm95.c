/*
 * rfm95.c
 *
 *  Created on: Apr 30, 2025
 *      Author: Elio
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

#include "rfm95.h"
#include "console.h"
/* Includes END --------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
// Register list
#define RFM95_REG_FIFO_ACCESS			0x00
#define RFM95_REG_OP_MODE				0x01
#define RFM95_REG_FREQ_MSB				0x06
#define RFM95_REG_FREQ_MID				0x07
#define RFM95_REG_FREQ_LSB				0x08
#define RFM95_REG_PA_CONFIG				0x09
#define RFM95_REG_LNA					0x0C
#define RFM95_REG_FIFO_ADDR_PTR			0x0D
#define RFM95_REG_FIFO_TX_BASE			0x0E
#define RFM95_REG_FIFO_RX_BASE			0x0F
#define RFM95_REG_IRQ_FLAGS 			0x12
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
#define RFM95_REG_FIFO_TX_ADDR					0x00

#define RFM95_CLEAR_IRQ_FLAGS					0xFF

// RFM95 Configuration
#define	RFM95_POWER					17
#define RFM95_FREQUENCY				915000000
#define RFM95_MAX_PAYLOAD_LEN		128
#define RFM95_MODEM_CONFIG_1		0x94
#define RFM95_MODEM_CONFIG_2		0x74
#define RFM95_MODEM_CONFIG_3		0x04


// Other defines
#define RFM95_WRITE_BUFF_LEN		2
#define RFM95_SPI_TIMEOUT 			1000
#define	TX_STATUS_IDLE				0
#define	TX_STATUS_BUSY				1
#define	TX_TIMEOUT					250
#define TIMER_LIMIT					0xFFFF

#define FALSE		0x00
#define TRUE		0x01
/* Defines END ---------------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/

/* Typedefs END --------------------------------------------------------------*/


/* Local Declarations --------------------------------------------------------*/
// Variables
static rfm95_handle_t rfm95_handle1;
static uint8_t status = 0;
static uint16_t timer_val;
static uint16_t timer_diff;

// Functions
static uint8_t ReadRegister( rfm95_handle_t *handle, uint8_t reg, uint8_t *buffer, size_t length );
static uint8_t WriteRegister( rfm95_handle_t* handle, uint8_t reg, uint8_t value );
static uint8_t SetPower( rfm95_handle_t *handle, uint8_t power );
static uint8_t SetFrequency( rfm95_handle_t *handle, uint32_t frequency );
static uint8_t Transmit( rfm95_handle_t *handle, uint8_t *payload, uint8_t payload_length );
/* Local Declarations END ----------------------------------------------------*/


/* Local Functions -----------------------------------------------------------*/
static uint8_t ReadRegister( rfm95_handle_t *handle, uint8_t reg, uint8_t *buffer, size_t length )
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
		CONSOLE_Printf( "READ FAILED 1\r\n" );
		return FALSE;
	}

	// Receive SPI response
	if( HAL_SPI_Receive( handle->spi_handle, buffer, length, RFM95_SPI_TIMEOUT ) != HAL_OK )
	{
		CONSOLE_Printf( "READ FAILED 2\r\n" );
		return FALSE;
	}

	// CS back to high
	HAL_GPIO_WritePin( handle->nss_port, handle->nss_pin, GPIO_PIN_SET );

	return TRUE;
}


static uint8_t WriteRegister( rfm95_handle_t *handle, uint8_t reg, uint8_t value )
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


static uint8_t SetPower( rfm95_handle_t *handle, uint8_t power )
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

	status |= WriteRegister( handle, RFM95_REG_PA_CONFIG, pa_config );
	status |= WriteRegister( handle, RFM95_REG_PA_DAC, pa_dac );
	return status;
}


static uint8_t SetFrequency( rfm95_handle_t *handle, uint32_t frequency )
{
	// FQ = (FRF * 32 Mhz) / (2 ^ 19)
	uint64_t frf = ( ( uint64_t )frequency << 19 ) / 32000000;

	if ( !WriteRegister( handle, RFM95_REG_FREQ_MSB, ( uint8_t )( frf >> 16 ) ) )
	{
		return FALSE;
	}

	if ( !WriteRegister( handle, RFM95_REG_FREQ_MID, ( uint8_t )( frf >> 8 ) ) )
	{
		return FALSE;
	}

	if ( !WriteRegister( handle, RFM95_REG_FREQ_LSB, ( uint8_t )( frf >> 0 ) ) )
	{
		return FALSE;
	}

	return TRUE;
}


static uint8_t Transmit( rfm95_handle_t *handle, uint8_t *payload, uint8_t payload_length )
{
	// Configure modem (500kHz, 4/6 error coding rate, SF7, single packet, CRC enable, AGC auto on)
	// MIGHT WANT TO DO IT ONCE IN RFM95_Init()
	WriteRegister( handle, RFM95_REG_MODEM_CONFIG_1, RFM95_MODEM_CONFIG_1 );
	WriteRegister( handle, RFM95_REG_MODEM_CONFIG_2, RFM95_MODEM_CONFIG_2 );
	WriteRegister( handle, RFM95_REG_MODEM_CONFIG_3, RFM95_MODEM_CONFIG_3 );

	// Set payload length
	WriteRegister( handle, RFM95_REG_PAYLOAD_LENGTH, payload_length );

	// Clear interrupt flags
	WriteRegister( handle, RFM95_REG_IRQ_FLAGS, RFM95_CLEAR_IRQ_FLAGS );

	// Set module to Lora Standby
	WriteRegister( handle, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_STANDBY );

	// Might have to add a small delay here for the module to be ready because we don't have DIO5 connected

	// Set the FIFO pointer to the start of the TX section
	WriteRegister( handle, RFM95_REG_FIFO_ADDR_PTR, RFM95_REG_FIFO_TX_ADDR );

	// Write payload to FIFO.
	for ( uint8_t i = 0; i < payload_length; i++ )
	{
		WriteRegister( handle, RFM95_REG_FIFO_ACCESS, payload[ i ] );
	}

	handle->tx_status = TX_STATUS_BUSY;
	timer_val = __HAL_TIM_GET_COUNTER( &htim3 );
	timer_diff = 0;

	// Set module to TX mode
	WriteRegister( handle, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_TX );

	// Wait for the transmission to complete
	while( ( handle->tx_status == TX_STATUS_BUSY ) && ( timer_diff <= TX_TIMEOUT ) )
	{
		timer_diff = ( ( __HAL_TIM_GET_COUNTER( &htim3 ) - timer_val ) + TIMER3_LIMIT ) % TIMER3_LIMIT;
	}

	// Set the module back to sleep
	WriteRegister( handle, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP );

	return TRUE;
}

/* Local Functions END -------------------------------------------------------*/


/* Global Functions ----------------------------------------------------------*/
uint8_t RFM95_Init( SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *nss_port, uint16_t nss_pin )
{
	// All the steps of this function can be included in a for loop that iterates over an array of rfm95_handle_t for all the chips

	// Add the SPI details for the handle
	rfm95_handle1.spi_handle = spi_handle;
	rfm95_handle1.nss_port = nss_port;
	rfm95_handle1.nss_pin = nss_pin;
	rfm95_handle1.tx_status = TX_STATUS_IDLE;

	// Can start by reading the version, not necessary now

	// Module must be placed in sleep mode before switching to LoRA.
	if ( !WriteRegister( &rfm95_handle1, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_SLEEP ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d, %d\r\n", RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_SLEEP );
		return FALSE;
	}

	if ( !WriteRegister( &rfm95_handle1, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d, %d\r\n", RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP );
		return FALSE;
	}

	// Set DIO0 to TXDone
	if ( !WriteRegister( &rfm95_handle1, RFM95_REG_DIO_MAPPING_1, RFM95_REG_DIO_MAPPING_1_DIO0_TXDONE ))
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d\r\n", RFM95_REG_DIO_MAPPING_1 );
		return FALSE;
	}

	// Set the power of the module in dBm. Can change it by modifying the define RFM95_POWER
	if ( !SetPower( &rfm95_handle1, RFM95_POWER ) )
	{
		CONSOLE_Printf( "RFM95_Init(): failed to set power\r\n" );
		return FALSE;
	}

	// Set the RF frequency in Hz. Can be changed by modifying the define RFM95_FREQUENCY
	if( !SetFrequency( &rfm95_handle1, RFM95_FREQUENCY ) )
	{
		CONSOLE_Printf( "RFM95_Init(): failed to set frequency\r\b\n" );
		return FALSE;
	}

	// Set up TX FIFO base address
	if ( !WriteRegister( &rfm95_handle1, RFM95_REG_FIFO_TX_BASE, RFM95_REG_FIFO_TX_ADDR ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d\r\n", RFM95_REG_FIFO_TX_BASE );
		return FALSE;
	}

	// Set up RX FIFO base address
	if ( !WriteRegister( &rfm95_handle1, RFM95_REG_FIFO_RX_BASE, RFM95_REG_FIFO_RX_ADDR ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d\r\n", RFM95_REG_FIFO_RX_BASE );
		return FALSE;
	}

	// Set max payload length. I think it's for RX only to filter payloads
	if ( !WriteRegister( &rfm95_handle1, RFM95_REG_MAX_PAYLOAD_LENGTH, RFM95_MAX_PAYLOAD_LEN ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed %d\r\n", RFM95_REG_MAX_PAYLOAD_LENGTH );
		return FALSE;
	}

	// Let module sleep after initialization
	if ( !WriteRegister( &rfm95_handle1, RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP ) )
	{
		CONSOLE_Printf( "RFM95_Init(): Intialization failed 2 %d, %d\r\n", RFM95_REG_OP_MODE, RFM95_REG_OP_MODE_LORA_SLEEP );
		return FALSE;
	}

	CONSOLE_Printf( "RFM95_Init(): Initialization completed...\r\n" );
	return TRUE;
}


void RFM95_InterruptCallbackG0( void )
{
	rfm95_handle1.tx_status = TX_STATUS_IDLE;
}


void RFM95_ReadVersion( void )
{
	/*
	uint8_t version = 0;

	CONSOLE_Printf( "Version before reading: %02X\r\n", version );

	read_register( &rfm95_handle1, RFM95_REG_VERSION, &version, 1 );
	CONSOLE_Printf( "Version after reading: %02X\r\n", version );
	*/

	uint8_t payload;

	if ( 0 == status )
	{
		WriteRegister( &rfm95_handle1, RFM95_REG_MAX_PAYLOAD_LENGTH, 64 );
		status = 1;
	}
	else
	{
		WriteRegister( &rfm95_handle1, RFM95_REG_MAX_PAYLOAD_LENGTH, 32 );
		status = 0;
	}

	ReadRegister( &rfm95_handle1, RFM95_REG_MAX_PAYLOAD_LENGTH, &payload, 1 );
	CONSOLE_Printf( "Payload Length: %02X\r\n", payload );

}

/* Global Functions END ------------------------------------------------------*/

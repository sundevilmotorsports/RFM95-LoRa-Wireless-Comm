/*
 * rfm95.h
 *
 *  Created on: Apr 30, 2025
 *      Author: Elio
 */

#ifndef INC_RFM95_H_
#define INC_RFM95_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
/* Includes END --------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
//#define RFM95_INTERRUPT_COUNT 	3
#define RFM95_DIO0_PIN		GPIO_PIN_10
/* Defines END ---------------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/
typedef struct {

	SPI_HandleTypeDef *spi_handle; 				// The handle to the SPI bus for the device
	GPIO_TypeDef *nss_port;						// The port of the NSS pin
	uint16_t nss_pin;							// The NSS pin
	uint8_t tx_status;
	//uint8_t device_address[ 4 ];					// The device address for the LoraWAN
	//uint8_t network_session_key[ 16 ];			// The network session key for ABP activation with the LoraWAN
	//uint8_t application_session_key[ 16 ];		// The application session key for ABP activation with the LoraWAN
	//uint32_t precision_tick_frequency;			// The frequency of the precision tick in Hz
	//uint32_t precision_tick_drift_ns_per_s;		// The +/- timing drift per second in nanoseconds
	//volatile uint32_t interrupt_times[ RFM95_INTERRUPT_COUNT ];		 // Tick values when each interrupt was called.

} rfm95_handle_t;
/* Typedefs END --------------------------------------------------------------*/


/* Global Variables ----------------------------------------------------------*/

/* Global Variables END ------------------------------------------------------*/


/* Global Functions ----------------------------------------------------------*/
uint8_t RFM95_Init( SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *nss_port, uint16_t nss_pin );
void RFM95_InterruptCallbackG0( void );
void RFM95_ReadVersion( void );
/* Global Functions END ------------------------------------------------------*/


#endif /* INC_RFM95_H_ */

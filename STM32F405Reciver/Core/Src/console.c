/*
 * console.c
 *
 *  Created on: Apr 26, 2025
 *      Author: Elio
 */

/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <stdint.h>

#include "console.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
/* Includes END --------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
#define USB_BUFF_LEN	128
/* Defines END ---------------------------------------------------------------*/


/* Typedefs ------------------------------------------------------------------*/

/* Typedefs END --------------------------------------------------------------*/


/* Local Variables -----------------------------------------------------------*/
static char usb_tx_buff[ USB_BUFF_LEN ];
/* Local Variables END -------------------------------------------------------*/


/* Local Functions -----------------------------------------------------------*/

/* Local Functions END -------------------------------------------------------*/


/* Global Functions ----------------------------------------------------------*/
uint8_t CONSOLE_Printf( char* fmt_str, ... )
{
	uint8_t status;
	int n;

	// Copy the formatted string to the buffer
	va_list args;
	va_start( args, fmt_str );
	n = vsnprintf( usb_tx_buff, USB_BUFF_LEN, fmt_str, args );
	va_end( args );

	// Send the string over USB
	if( n < USB_BUFF_LEN )
	{
		status = CDC_Transmit_FS( ( uint8_t * )usb_tx_buff, n );
	}
	else
	{
		usb_tx_buff[ USB_BUFF_LEN - 1 ] = '\n';
		status = CDC_Transmit_FS( ( uint8_t * )usb_tx_buff, USB_BUFF_LEN );

		sprintf( usb_tx_buff, "CONSOLE_Printf: string bigger than buffer\r\n" );
		status += CDC_Transmit_FS( ( uint8_t * )usb_tx_buff, USB_BUFF_LEN );
	}

	return status;
}
/* Global Functions END ------------------------------------------------------*/

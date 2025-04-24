#pragma once

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#ifndef RFM95_SPI_TIMEOUT
#define RFM95_SPI_TIMEOUT 10
#endif

#ifndef RFM95_WAKEUP_TIMEOUT
#define RFM95_WAKEUP_TIMEOUT 10
#endif

#ifndef RFM95_SEND_TIMEOUT
#define RFM95_SEND_TIMEOUT 100
#endif

#ifndef RFM95_RECEIVE_TIMEOUT
#define RFM95_RECEIVE_TIMEOUT 1000
#endif

#define RFM95_EEPROM_CONFIG_MAGIC 0xab67

#define RFM95_INTERRUPT_COUNT 4

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RFM95_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
#define RFM95_MAX_PAYLOAD_LEN RFM95_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RFM95_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver.
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RFM95_MAX_MESSAGE_LEN
 #define RFM95_MAX_MESSAGE_LEN (RFM95_MAX_PAYLOAD_LEN - RFM95_HEADER_LEN)
#endif

// The crystal oscillator frequency of the module
#define RFM95_FXOSC 32000000.0

// The Frequency Synthesizer step = RFM95_FXOSC / 2^^19
#define RFM95_FSTEP  (RFM95_FXOSC / 524288)


// Register names (LoRa Mode, from table 85)
#define RFM95_REG_00_FIFO                                0x00
#define RFM95_REG_01_OP_MODE                             0x01
#define RFM95_REG_02_RESERVED                            0x02
#define RFM95_REG_03_RESERVED                            0x03
#define RFM95_REG_04_RESERVED                            0x04
#define RFM95_REG_05_RESERVED                            0x05
#define RFM95_REG_06_FRF_MSB                             0x06
#define RFM95_REG_07_FRF_MID                             0x07
#define RFM95_REG_08_FRF_LSB                             0x08
#define RFM95_REG_09_PA_CONFIG                           0x09
#define RFM95_REG_0A_PA_RAMP                             0x0a
#define RFM95_REG_0B_OCP                                 0x0b
#define RFM95_REG_0C_LNA                                 0x0c
#define RFM95_REG_0D_FIFO_ADDR_PTR                       0x0d
#define RFM95_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RFM95_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define RFM95_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define RFM95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RFM95_REG_12_IRQ_FLAGS                           0x12
#define RFM95_REG_13_RX_NB_BYTES                         0x13
#define RFM95_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define RFM95_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define RFM95_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define RFM95_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define RFM95_REG_18_MODEM_STAT                          0x18
#define RFM95_REG_19_PKT_SNR_VALUE                       0x19
#define RFM95_REG_1A_PKT_RSSI_VALUE                      0x1a
#define RFM95_REG_1B_RSSI_VALUE                          0x1b
#define RFM95_REG_1C_HOP_CHANNEL                         0x1c
#define RFM95_REG_1D_MODEM_CONFIG1                       0x1d
#define RFM95_REG_1E_MODEM_CONFIG2                       0x1e
#define RFM95_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define RFM95_REG_20_PREAMBLE_MSB                        0x20
#define RFM95_REG_21_PREAMBLE_LSB                        0x21
#define RFM95_REG_22_PAYLOAD_LENGTH                      0x22
#define RFM95_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define RFM95_REG_24_HOP_PERIOD                          0x24
#define RFM95_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define RFM95_REG_26_MODEM_CONFIG3                       0x26

#define RFM95_REG_27_PPM_CORRECTION                      0x27
#define RFM95_REG_28_FEI_MSB                             0x28
#define RFM95_REG_29_FEI_MID                             0x29
#define RFM95_REG_2A_FEI_LSB                             0x2a
#define RFM95_REG_2C_RSSI_WIDEBAND                       0x2c
#define RFM95_REG_31_DETECT_OPTIMIZE                     0x31
#define RFM95_REG_33_INVERT_IQ                           0x33
#define RFM95_REG_37_DETECTION_THRESHOLD                 0x37
#define RFM95_REG_39_SYNC_WORD                           0x39

#define RFM95_REG_40_DIO_MAPPING1                        0x40
#define RFM95_REG_41_DIO_MAPPING2                        0x41
#define RFM95_REG_42_VERSION                             0x42

#define RFM95_REG_4B_TCXO                                0x4b
#define RFM95_REG_4D_PA_DAC                              0x4d
#define RFM95_REG_5B_FORMER_TEMP                         0x5b
#define RFM95_REG_61_AGC_REF                             0x61
#define RFM95_REG_62_AGC_THRESH1                         0x62
#define RFM95_REG_63_AGC_THRESH2                         0x63
#define RFM95_REG_64_AGC_THRESH3                         0x64

// RFM95_REG_01_OP_MODE                             0x01
#define RFM95_LONG_RANGE_MODE                       0x80
#define RFM95_ACCESS_SHARED_REG                     0x40
#define RFM95_LOW_FREQUENCY_MODE                    0x08
#define RFM95_MODE                                  0x07
#define RFM95_MODE_SLEEP                            0x00
#define RFM95_MODE_STDBY                            0x01
#define RFM95_MODE_FSTX                             0x02
#define RFM95_MODE_TX                               0x03
#define RFM95_MODE_FSRX                             0x04
#define RFM95_MODE_RXCONTINUOUS                     0x05
#define RFM95_MODE_RXSINGLE                         0x06
#define RFM95_MODE_CAD                              0x07

// RFM95_REG_09_PA_CONFIG                           0x09
#define RFM95_PA_SELECT                             0x80
#define RFM95_MAX_POWER                             0x70
#define RFM95_OUTPUT_POWER                          0x0f
#define RFM95_REGISTER_PA_DAC_LOW_POWER             0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER            0x87

// RFM95_REG_0A_PA_RAMP                             0x0a
#define RFM95_LOW_PN_TX_PLL_OFF                     0x10
#define RFM95_PA_RAMP                               0x0f
#define RFM95_PA_RAMP_3_4MS                         0x00
#define RFM95_PA_RAMP_2MS                           0x01
#define RFM95_PA_RAMP_1MS                           0x02
#define RFM95_PA_RAMP_500US                         0x03
#define RFM95_PA_RAMP_250US                         0x04
#define RFM95_PA_RAMP_125US                         0x05
#define RFM95_PA_RAMP_100US                         0x06
#define RFM95_PA_RAMP_62US                          0x07
#define RFM95_PA_RAMP_50US                          0x08
#define RFM95_PA_RAMP_40US                          0x09
#define RFM95_PA_RAMP_31US                          0x0a
#define RFM95_PA_RAMP_25US                          0x0b
#define RFM95_PA_RAMP_20US                          0x0c
#define RFM95_PA_RAMP_15US                          0x0d
#define RFM95_PA_RAMP_12US                          0x0e
#define RFM95_PA_RAMP_10US                          0x0f

// RFM95_REG_0B_OCP                                 0x0b
#define RFM95_OCP_ON                                0x20
#define RFM95_OCP_TRIM                              0x1f

// RFM95_REG_0C_LNA                                 0x0c
#define RFM95_LNA_GAIN                              0xe0
#define RFM95_LNA_GAIN_G1                           0x20
#define RFM95_LNA_GAIN_G2                           0x40
#define RFM95_LNA_GAIN_G3                           0x60
#define RFM95_LNA_GAIN_G4                           0x80
#define RFM95_LNA_GAIN_G5                           0xa0
#define RFM95_LNA_GAIN_G6                           0xc0
#define RFM95_LNA_BOOST_LF                          0x18
#define RFM95_LNA_BOOST_LF_DEFAULT                  0x00
#define RFM95_LNA_BOOST_HF                          0x03
#define RFM95_LNA_BOOST_HF_DEFAULT                  0x00
#define RFM95_LNA_BOOST_HF_150PC                    0x03

// RFM95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RFM95_RX_TIMEOUT_MASK                       0x80
#define RFM95_RX_DONE_MASK                          0x40
#define RFM95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RFM95_VALID_HEADER_MASK                     0x10
#define RFM95_TX_DONE_MASK                          0x08
#define RFM95_CAD_DONE_MASK                         0x04
#define RFM95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RFM95_CAD_DETECTED_MASK                     0x01

// RFM95_REG_12_IRQ_FLAGS                           0x12
#define RFM95_RX_TIMEOUT                            0x80
#define RFM95_RX_DONE                               0x40
#define RFM95_PAYLOAD_CRC_ERROR                     0x20
#define RFM95_VALID_HEADER                          0x10
#define RFM95_TX_DONE                               0x08
#define RFM95_CAD_DONE                              0x04
#define RFM95_FHSS_CHANGE_CHANNEL                   0x02
#define RFM95_CAD_DETECTED                          0x01

// RFM95_REG_18_MODEM_STAT                          0x18
#define RFM95_RX_CODING_RATE                        0xe0
#define RFM95_MODEM_STATUS_CLEAR                    0x10
#define RFM95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RFM95_MODEM_STATUS_RX_ONGOING               0x04
#define RFM95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RFM95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RFM95_REG_1C_HOP_CHANNEL                         0x1c
#define RFM95_PLL_TIMEOUT                           0x80
#define RFM95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RFM95_FHSS_PRESENT_CHANNEL                  0x3f

// RFM95_REG_1D_MODEM_CONFIG1                       0x1d
#define RFM95_BW                                    0xf0

#define RFM95_BW_7_8KHZ                             0x00
#define RFM95_BW_10_4KHZ                            0x10
#define RFM95_BW_15_6KHZ                            0x20
#define RFM95_BW_20_8KHZ                            0x30
#define RFM95_BW_31_25KHZ                           0x40
#define RFM95_BW_41_7KHZ                            0x50
#define RFM95_BW_62_5KHZ                            0x60
#define RFM95_BW_125KHZ                             0x70
#define RFM95_BW_250KHZ                             0x80
#define RFM95_BW_500KHZ                             0x90
#define RFM95_CODING_RATE                           0x0e
#define RFM95_CODING_RATE_4_5                       0x02
#define RFM95_CODING_RATE_4_6                       0x04
#define RFM95_CODING_RATE_4_7                       0x06
#define RFM95_CODING_RATE_4_8                       0x08
#define RFM95_IMPLICIT_HEADER_MODE_ON               0x01

// RFM95_REG_1E_MODEM_CONFIG2                       0x1e
#define RFM95_SPREADING_FACTOR                      0xf0
#define RFM95_SPREADING_FACTOR_64CPS                0x60
#define RFM95_SPREADING_FACTOR_128CPS               0x70
#define RFM95_SPREADING_FACTOR_256CPS               0x80
#define RFM95_SPREADING_FACTOR_512CPS               0x90
#define RFM95_SPREADING_FACTOR_1024CPS              0xa0
#define RFM95_SPREADING_FACTOR_2048CPS              0xb0
#define RFM95_SPREADING_FACTOR_4096CPS              0xc0
#define RFM95_TX_CONTINUOUS_MODE                    0x08

#define RFM95_PAYLOAD_CRC_ON                        0x04
#define RFM95_SYM_TIMEOUT_MSB                       0x03

// RFM95_REG_26_MODEM_CONFIG3
#define RFM95_MOBILE_NODE                           0x08 // HopeRF term
#define RFM95_LOW_DATA_RATE_OPTIMIZE                0x08 // Semtechs term
#define RFM95_AGC_AUTO_ON                           0x04

// RFM95_REG_4B_TCXO                                0x4b
#define RFM95_TCXO_TCXO_INPUT_ON                    0x10

// RFM95_REG_4D_PA_DAC                              0x4d
#define RFM95_PA_DAC_DISABLE                        0x04
#define RFM95_PA_DAC_ENABLE                         0x07

#define RFM95_REGISTER_OP_MODE_SLEEP                            0x00
#define RFM95_REGISTER_OP_MODE_LORA_SLEEP                       0x80
#define RFM95_REGISTER_OP_MODE_LORA_STANDBY                     0x81
#define RFM95_REGISTER_OP_MODE_LORA_TX                          0x83
#define RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE                   0x86

#define RFM95_REGISTER_PA_DAC_LOW_POWER                         0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER                        0x87

#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE             0x40
#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE             0x00

#define RFM95_REGISTER_INVERT_IQ_1_TX                    		0x27
#define RFM95_REGISTER_INVERT_IQ_2_TX							0x1d

#define RFM95_REGISTER_INVERT_IQ_1_RX                    		0x67
#define RFM95_REGISTER_INVERT_IQ_2_RX							0x19

#define RFM95_REGISTER_FIFO_ACCESS 0x00
#define	RFM95_REGISTER_OP_MODE 0x01
#define	RFM95_REGISTER_FR_MSB 0x06
#define	RFM95_REGISTER_FR_MID 0x07
#define	RFM95_REGISTER_FR_LSB 0x08
#define	RFM95_REGISTER_PA_CONFIG 0x09
#define	RFM95_REGISTER_LNA 0x0C
#define	RFM95_REGISTER_FIFO_ADDR_PTR 0x0D
#define	RFM95_REGISTER_FIFO_TX_BASE_ADDR 0x0E
#define	RFM95_REGISTER_FIFO_RX_BASE_ADDR 0x0F
#define	RFM95_REGISTER_IRQ_FLAGS 0x12
#define	RFM95_REGISTER_FIFO_RX_BYTES_NB 0x13
#define	RFM95_REGISTER_PACKET_SNR 0x19
#define	RFM95_REGISTER_MODEM_CONFIG_1 0x1D
#define	RFM95_REGISTER_MODEM_CONFIG_2 0x1E
#define	RFM95_REGISTER_SYMB_TIMEOUT_LSB 0x1F
#define	RFM95_REGISTER_PREAMBLE_MSB 0x20
#define	RFM95_REGISTER_PREAMBLE_LSB 0x21
#define	RFM95_REGISTER_PAYLOAD_LENGTH 0x22
#define	RFM95_REGISTER_MAX_PAYLOAD_LENGTH 0x23
#define	RFM95_REGISTER_MODEM_CONFIG_3 0x26
#define	RFM95_REGISTER_INVERT_IQ_1 0x33
#define	RFM95_REGISTER_SYNC_WORD 0x39
#define	RFM95_REGISTER_INVERT_IQ_2 0x3B
#define	RFM95_REGISTER_DIO_MAPPING_1 0x40
#define	RFM95_REGISTER_VERSION 0x42
#define	RFM95_REGISTER_PA_DAC 0x4D

typedef struct {

	uint32_t frequency;

} rfm95_channel_config_t;

typedef struct {

	/**
	 * MAGIC
	 */
	uint16_t magic;

	/**
	 * The current RX frame counter value.
	 */
	uint16_t rx_frame_count;

	/**
	 * The current TX frame counter value.
	 */
	uint16_t tx_frame_count;

	/**
	 * The delay to the RX1 window.
	 */
	uint8_t rx1_delay;

	/**
	 * The configuration of channels;
	 */
	rfm95_channel_config_t channels[16];

	/**
	 * Mask defining which channels are configured.
	 */
	uint16_t channel_mask;

} rfm95_eeprom_config_t;

typedef void (*rfm95_on_after_interrupts_configured)();

typedef bool (*rfm95_load_eeprom_config)(rfm95_eeprom_config_t *config);
typedef void (*rfm95_save_eeprom_config)(const rfm95_eeprom_config_t *config);

typedef uint32_t (*rfm95_get_precision_tick)();
typedef void (*rfm95_precision_sleep_until)(uint32_t ticks_target);

typedef uint8_t (*rfm95_random_int)(uint8_t max);
typedef uint8_t (*rfm95_get_battery_level)();

typedef enum
{
	RFM95_INTERRUPT_DIO0,
	RFM95_INTERRUPT_DIO1,
	RFM95_INTERRUPT_DIO2,
	RFM95_INTERRUPT_DIO3,

} rfm95_interrupt_t;

typedef enum
{
	RFM95_RECEIVE_MODE_NONE,
	RFM95_RECEIVE_MODE_RX1_ONLY,
	RFM95_RECEIVE_MODE_RX12,
} rfm95_receive_mode_t;

#define RFM9x_VER 0x12

typedef struct
{
	union {
		struct {
			uint8_t output_power : 4;
			uint8_t max_power : 3;
			uint8_t pa_select : 1;
		};
		uint8_t buffer;
	};
} rfm95_register_pa_config_t;

	// Defines register values for a set of modem configuration registers
    /// that can be passed to setModemRegisters() if none of the choices in
    /// ModemConfigChoice suit your need setModemRegisters() writes the
    /// register values from this structure to the appropriate registers
    /// to set the desired spreading factor, coding rate and bandwidth
typedef struct
    {
	uint8_t    reg_1d;   ///< Value for register RFM95_REG_1D_MODEM_CONFIG1
	uint8_t    reg_1e;   ///< Value for register RFM95_REG_1E_MODEM_CONFIG2
	uint8_t    reg_26;   ///< Value for register RFM95_REG_26_MODEM_CONFIG3
    } ModemConfig;

/// Choices for setModemConfig() for a selected subset of common
/// data rates. If you need another configuration,
/// determine the necessary settings and call setModemRegisters() with your
/// desired settings. It might be helpful to use the LoRa calculator mentioned in
/// http://www.semtech.com/images/datasheet/LoraDesignGuide_STD.pdf
/// These are indexes into MODEM_CONFIG_TABLE. We strongly recommend you use these symbolic
/// definitions and not their integer equivalents: its possible that new values will be
/// introduced in later versions (though we will try to avoid it).
/// Caution: if you are using slow packet rates and long packets with RHReliableDatagram or subclasses
/// you may need to change the RHReliableDatagram timeout for reliable operations.
/// Caution: for some slow rates nad with ReliableDatagrams you may need to increase the reply timeout
/// with manager.setTimeout() to
/// deal with the long transmission times.
/// Caution: SX1276 family errata suggests alternate settings for some LoRa registers when 500kHz bandwidth
/// is in use. See the Semtech SX1276/77/78 Errata Note. These are not implemented by RFM95.
typedef enum
{
Bw125Cr45Sf128 = 0,	   ///< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
Bw500Cr45Sf128,	           ///< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
Bw31_25Cr48Sf512,	   ///< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
Bw125Cr48Sf4096,           ///< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
Bw125Cr45Sf2048,           ///< Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range
} ModemConfigChoice;

/**
 * Structure defining a handle describing an RFM95(W) transceiver.
 */
typedef struct {

	/**
	 * The handle to the SPI bus for the device.
	 */
	SPI_HandleTypeDef *spi_handle;

	/**
	 * The port of the NSS pin.
	 */
	GPIO_TypeDef *nss_port;

	/**
	 * The NSS pin.
	 */
	uint16_t nss_pin;

	/**
	 * The port of the RST pin.
	 */
	GPIO_TypeDef *nrst_port;

	/**
	 * The RST pin.
	 */
	uint16_t nrst_pin;

	// enable for CRC
	uint8_t crc_enable;

	/**
	 * The device address for the LoraWAN
	 */
	uint8_t device_address[4];

	/**
	 * The network session key for ABP activation with the LoraWAN
	 */
	uint8_t network_session_key[16];

	/**
	 * The application session key for ABP activation with the LoraWAN
	 */
	uint8_t application_session_key[16];

	/**
	 * The frequency of the precision tick in Hz.
	 */
	uint32_t precision_tick_frequency;

	/**
	 * The +/- timing drift per second in nanoseconds.
	 */
	uint32_t precision_tick_drift_ns_per_s;

	/**
	 * The receive mode to operate at.
	 */
	rfm95_receive_mode_t receive_mode;

	/**
	 * Function provided that returns a precise tick for timing critical operations.
	 */
	rfm95_get_precision_tick get_precision_tick;

	/**
	 * Function that provides a precise sleep until a given tick count is reached.
	 */
	rfm95_precision_sleep_until precision_sleep_until;

	/**
	 * Function that provides a random integer.
	 */
	rfm95_random_int random_int;

	/**
	 * Function that returns the device's battery level.
	 */
	rfm95_get_battery_level get_battery_level;

	/**
	 * The load config function pointer can be used to load config values from non-volatile memory.
	 * Can be set to NULL to skip.
	 */
	rfm95_load_eeprom_config reload_config;

	/**
	 * The save config function pointer can be used to store config values in non-volatile memory.
	 * Can be set to NULL to skip.
	 */
	rfm95_save_eeprom_config save_config;

	/**
	 * Callback called after the interrupt functions have been properly configred;
	 */
	rfm95_on_after_interrupts_configured on_after_interrupts_configured;

	/**
	 * The config saved into the eeprom.
	 */
	rfm95_eeprom_config_t config;

	/**
	 * Tick values when each interrupt was called.
	 */
	volatile uint32_t interrupt_times[RFM95_INTERRUPT_COUNT];

} rfm95_handle_t;

bool rfm95_init(rfm95_handle_t *handle);

bool rfm95_set_power(rfm95_handle_t *handle, int8_t power);

bool rfm95_send_receive_cycle(rfm95_handle_t *handle, const uint8_t *send_data, size_t send_data_length);

void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt);

/**
 * @file sx1278.h
 * @author
 * @date
 * @brief
 */

#ifndef _SX1278_H_
#define _SX1278_H_

//=============================================================================
// [Inclusions] ===============================================================
#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"

//=============================================================================

//=============================================================================
// [External Defines] =========================================================
#define SX1278_GPIO_MOSI        GPIO_NUM_32
#define SX1278_GPIO_MISO        GPIO_NUM_35
#define SX1278_GPIO_SCK         GPIO_NUM_33
#define SX1278_GPIO_NSS         GPIO_NUM_25
#define SX1278_GPIO_RESET      	GPIO_NUM_26
#define SX1278_GPIO_DIO0     	GPIO_NUM_34
#define SX1278_GPIO_DIO1     	GPIO_NUM_39

// Important register addresses
#define SX1278_REG_LR_FIFO                      0x00
// Common settings
#define SX1278_REG_LR_OPMODE                    0x01
#define SX1278_REG_LR_BANDSETTING               0x04
#define SX1278_REG_LR_FRFMSB                    0x06
#define SX1278_REG_LR_FRFMID                    0x07
#define SX1278_REG_LR_FRFLSB                    0x08
// Tx settings
#define SX1278_REG_LR_PACONFIG                  0x09
#define SX1278_REG_LR_PARAMP                    0x0A
#define SX1278_REG_LR_OCP                       0x0B
// Rx settings
#define SX1278_REG_LR_LNA                       0x0C
// LoRa registers
#define SX1278_REG_LR_FIFOADDRPTR               0x0D
#define SX1278_REG_LR_FIFOTXBASEADDR            0x0E
#define SX1278_REG_LR_FIFORXBASEADDR            0x0F
#define SX1278_REG_LR_FIFORXCURRENTADDR         0x10
#define SX1278_REG_LR_IRQFLAGSMASK              0x11
#define SX1278_REG_LR_IRQFLAGS                  0x12
#define SX1278_REG_LR_NBRXBYTES                 0x13
#define SX1278_REG_LR_RXHEADERCNTVALUEMSB       0x14
#define SX1278_REG_LR_RXHEADERCNTVALUELSB       0x15
#define SX1278_REG_LR_RXPACKETCNTVALUEMSB       0x16
#define SX1278_REG_LR_RXPACKETCNTVALUELSB       0x17
#define SX1278_REG_LR_MODEMSTAT                 0x18
#define SX1278_REG_LR_PKTSNRVALUE               0x19
#define SX1278_REG_LR_PKTRSSIVALUE              0x1A
#define SX1278_REG_LR_RSSIVALUE                 0x1B
#define SX1278_REG_LR_HOPCHANNEL                0x1C
#define SX1278_REG_LR_MODEMCONFIG1              0x1D
#define SX1278_REG_LR_MODEMCONFIG2              0x1E
#define SX1278_REG_LR_SYMBTIMEOUTLSB            0x1F
#define SX1278_REG_LR_PREAMBLEMSB               0x20
#define SX1278_REG_LR_PREAMBLELSB               0x21
#define SX1278_REG_LR_PAYLOADLENGTH             0x22
#define SX1278_REG_LR_PAYLOADMAXLENGTH          0x23
#define SX1278_REG_LR_HOPPERIOD                 0x24
#define SX1278_REG_LR_FIFORXBYTEADDR            0x25
#define SX1278_REG_LR_MODEMCONFIG3              0x26
#define SX1278_REG_LR_FEIMSB                    0x28
#define SX1278_REG_LR_FEIMIB                    0x29
#define SX1278_REG_LR_FEILSB                    0x2A
#define SX1278_REG_LR_LORADETECTOPTIMIZE        0x31
#define SX1278_REG_LR_INVERTIQ                  0x33
#define SX1278_REG_LR_DETECTIONTHRESHOLD        0x37
// I/O settings
#define SX1278_REG_LR_DIOMAPPING1               0x40
#define SX1278_REG_LR_DIOMAPPING2               0x41
// Version
#define SX1278_REG_LR_VERSION                   0x42
// Additional settings
#define SX1278_REG_LR_PLLHOP                    0x44
#define SX1278_REG_LR_TCXO                      0x4B
#define SX1278_REG_LR_PADAC                     0x4D
#define SX1278_REG_LR_FORMERTEMP                0x5B
#define SX1278_REG_LR_BITRATEFRAC               0x5D
#define SX1278_REG_LR_AGCREF                    0x61
#define SX1278_REG_LR_AGCTHRESH1                0x62
#define SX1278_REG_LR_AGCTHRESH2                0x63
#define SX1278_REG_LR_AGCTHRESH3                0x64

// Radio mode values
#define SX1278_LORA_MODE    0x01
#define SX1278_FSKOOK_MODE  0x00

// Operative mode values
#define SX1278_SLEEP        0x00    // Lowest power. It allows change OOK/FSK to LoRa
#define SX1278_STDBY        0x01    // Basic state
#define SX1278_FSTX         0x02    // Prepare to transmit
#define SX1278_TX           0x03    // Transmit what is in FIFO queue
#define SX1278_FSRX         0x04    // Prepare to set in receive mode
#define SX1278_RXCONT       0x05    // Receive mode continuous in LoRa configuration
#define SX1278_RX           0x05    // Receive mode in OOK/FSK configuration
#define SX1278_RXSING       0x06    // Receive mode single packet in LoRa configuration
#define SX1278_CAD          0x07    // Channel activity detection

// Flags
#define SX1278_RX_TIMEOUT           0x07
#define SX1278_RX_DONE              0x06
#define SX1278_PAYLOAD_CRC_ERROR    0x05
#define SX1278_VALID_HEADER         0x04
#define SX1278_TX_DONE              0x03
#define SX1278_CAD_DONE             0x02
#define SX1278_FHSS_CHANGED_CH      0x01
#define SX1278_CAD_DETECTED         0x00

#define SX1278_BUFF_SIZE            48      //Debe ser multiplo de 16 y mayor a lora_packet_t

//=============================================================================

//=============================================================================
// [External Typedef] =========================================================

//=============================================================================

//=============================================================================
// [External Data Definitions] ================================================

// Const ---------------------------------------------
//----------------------------------------------------

// Vars ----------------------------------------------
//----------------------------------------------------

// Task Handlers -------------------------------------
//----------------------------------------------------

// Queue Handlers ------------------------------------
//----------------------------------------------------

// Event Group Handlers ------------------------------
//----------------------------------------------------

// Semaphore Handlers --------------------------------
//----------------------------------------------------

//=============================================================================

//=============================================================================
// [External Function Declarations] ===========================================

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_init(void);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_write(uint8_t* data);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
bool sx1278_read(uint8_t* data);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_set_boost_on(bool boost_on);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_set_power(uint8_t power);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_set_frequency(float frequency);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_set_spreading_factor(uint8_t spreading_factor);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_set_bandwidth(uint8_t bandwidth);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_set_coding_rate(uint8_t coding_rate);

/**
 * @details
 * @param[in]
 * @param[in]
 * @return
 */
void sx1278_set_rx_enable(void);

//=============================================================================

#endif /* _SX1278_H_ */

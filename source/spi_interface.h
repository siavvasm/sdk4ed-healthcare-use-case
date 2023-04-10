/**************************************************************************//**
 * @file
 * @brief Declarations related to the main MCU's SPI interface with its co-proc
 * @version 0.0.1
 ******************************************************************************/

#ifndef _SPI_INTERFACE_H_
#define _SPI_INTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_msc.h"
#include "spi_common.h"
#include "fir.h"
#include "rtcc.h"

#define SPI_BUFFER_SIZE PAYLOAD_SIZE+HEADER_SIZE

extern uint8_t TxBuffer[SPI_BUFFER_SIZE];
extern uint8_t RxBuffer[SPI_BUFFER_SIZE];

extern uint32_t RxBufferIndex;
extern uint32_t TxBufferIndex;

extern uint16_t samples_array[NR_SAMPLES];
extern uint16_t supply_voltage_mV;
extern bool treatment_on;
extern uint32_t seizure_time[NR_SAMPLES/2];
extern uint32_t seizure_date[NR_SAMPLES/2];

void spi_transfer_master(uint8_t * tx_buffer, uint8_t * rx_buffer, uint32_t * tx_buffer_index, uint32_t * rx_buffer_index, USART_TypeDef *usart);
void command_decoding_tree(uint8_t * tx_buffer, uint8_t * rx_buffer);
void receive_firmware(uint32_t * tx_buffer_index, uint32_t * rx_buffer_index, USART_TypeDef *usart);

#endif //_SPI_INTERFACE_H_

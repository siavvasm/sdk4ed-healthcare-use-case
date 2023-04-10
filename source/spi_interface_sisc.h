/**************************************************************************//**
 * @file
 * @brief Declarations related to the security-core's SPI interface with the TRX
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

// Addresses for ZL70103 reg_txrxbuff SPI read/write accesses
// The MSB bit indicates read (1) or write (0) access
#define TRX_READ_ADDRESS 0xFA
#define TRX_WRITE_ADDRESS 0x7A
#define TRX_BLOCKSIZE_ADDRESS 0x7B
#define TRX_PACKETSIZE_ADDRESS 0x7C
#define TRX_RXIRQ_ADDRESS 0x7D

#define BLOCK_SIZE 14 // number of bytes in a block
#define PACKET_SIZE 24 // number of blocks in a packet
#define MAC_SIZE 8
#define TRX_BUFFER_SIZE BLOCK_SIZE*PACKET_SIZE

#define FW_PACKET_SIZE 9376
#define TRX_FW_BUFFER_SIZE BLOCK_SIZE*FW_PACKET_SIZE

extern uint8_t TxBuffer[TRX_BUFFER_SIZE];
extern uint8_t TxDummyBuffer;
extern uint8_t RxBuffer[TRX_BUFFER_SIZE];
extern uint8_t RxDummyBuffer;

void write_tx_buffer(uint8_t * tx_buffer, USART_TypeDef *usart);
void read_rx_buffer(uint8_t * rx_buffer, USART_TypeDef *usart);
void get_firmware(USART_TypeDef *usart, uint8_t * received_mac);
void send_firmware(USART_TypeDef *usart);
void send_signatures(USART_TypeDef *usart);

#endif //_SPI_INTERFACE_H_

/**************************************************************************//**
 * @file
 * @brief Definitions related to the SPI interfaces common in both MCUs 
 * @version 0.0.1
 ******************************************************************************/

#include "spi_common.h"

/**************************************************************************//**
 * @brief Combine 4 consecutive 8-bit array values into a 32-bit int
 *****************************************************************************/
void pack_4x8bits_into_32(uint8_t * trx_buffer, uint16_t start_index, uint32_t * combined)
{
  *combined = (trx_buffer[start_index+3] << 24) | (trx_buffer[start_index+2] << 16) | (trx_buffer[start_index+1] << 8) | (trx_buffer[start_index+0] & 0xff);
}

/**************************************************************************//**
 * @brief Split a 32-bit int to fit into 4 consecutive 8-bit array addresses
 *****************************************************************************/
void split_32bits_into_4x8(uint8_t * trx_buffer, uint16_t start_index, uint32_t initial)
{
  trx_buffer[start_index+0] = (initial & 0xFF);
  trx_buffer[start_index+1] = (initial & 0xFF00) >> 8;
  trx_buffer[start_index+2] = (initial & 0xFF0000) >> 16;
  trx_buffer[start_index+3] = (initial & 0xFF000000) >> 24;
}

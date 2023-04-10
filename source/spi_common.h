/**************************************************************************//**
 * @file
 * @brief Common SPI-related macro definitions
 * @version 0.0.1
 ******************************************************************************/

#ifndef _SPI_COMMON_H_
#define _SPI_COMMON_H_

#include "fir.h"

#define PAYLOAD_SIZE NR_SAMPLES*4 // 4 = 2 types of coefficients x 2 8-bit parts
#define HEADER_SIZE 16
#define FW_PAYLOAD_SIZE 16*1024*8 // 16 kB
#define FW_START_ADDRESS 0x0001B000

// Code-words of commands received from the programmer (via the IMD TRX)
#define SEND_BATTERY_STATUS 0x01
#define SEND_DATE 0x02
#define SET_DATE 0x21
#define SEND_STATUS 0x22
#define SEND_SENSOR_VALUES 0x23
#define SEND_COEFFICIENTS 0x24
#define SEND_THRESHOLDS 0x25
#define UPDATE_COEFFICIENTS 0x41
#define UPDATE_THRESHOLDS 0x42
#define TREATMENT_OFF 0x43
#define TREATMENT_ON 0x44
#define UPDATE_SISC_FIRMWARE 0x81
#define UPDATE_SIMS_FIRMWARE 0x82
#define SEND_SIGNATURE 0x83

// Privilege boundaries of different roles (not used, only for reference)
#define PATIENT 0x20
#define NURSE 0x40
#define DOCTOR 0x80
#define TECHNICIAN 0xFF

void pack_4x8bits_into_32(uint8_t * trx_buffer, uint16_t start_index, uint32_t * combined);
void split_32bits_into_4x8(uint8_t * trx_buffer, uint16_t start_index, uint32_t initial);

#endif //_SPI_COMMON_H_

/**************************************************************************//**
 * @file
 * @brief Declarations related to the reader-IMD security protocol
 * @version 0.0.1
 ******************************************************************************/

#ifndef _SECURITY_H_
#define _SECURITY_H_

#include <stdint.h>
#include <stdbool.h>
#include "em_crypto.h"
#include "spi_interface_sisc.h"

#define STATE_IDLE 0
#define STATE_TRX0 1
#define STATE_TRX1 2
#define STATE_TRX2 3
#define STATE_FW_DL 4

// Seeds for the PRN-generator (provided by the factory)
#define PRNG_SEED 0x0F0F0F0F0F0F0F0F

// Block cipher block and key sizes in bytes
#define CIPHER_BLOCK_SIZE 16
#define KEY_SIZE 16

#define ID_SIZE 4
#define NONCE_SIZE 4
#define SIG_SIZE 48 // 384-bit ECDSA signature
#define CRYPTOGRAM_SIZE CIPHER_BLOCK_SIZE+MAC_SIZE+(3*CIPHER_BLOCK_SIZE)+MAC_SIZE // combined size of m_RI and m_I
#define BUFFER_PLAINTEXT_SIZE PAYLOAD_SIZE+HEADER_SIZE+SIG_SIZE

#define SIG_INDEX_ADDRESS 0x0001A000 // Flash address of the index of the next signature location
#define FIRST_SIG_ADDRESS 0x0001A800 // Flash address of the first signature location
#define NO_OF_SIG_ENTRIES 8
#define SIG_ENTRY_SIZE 64 // in bytes

#define BCC_RX_PACKET_SIZE 8*((2*NONCE_SIZE)+ID_SIZE) // in bits
#define BCC_TX_PACKET_SIZE 8*(KEY_SIZE+NONCE_SIZE+ID_SIZE) // in bits

#endif /* _SECURITY_H_ */

/**************************************************************************//**
 * @file
 * @brief This application is executed by the security co-processor
 * @version 0.0.1
 ******************************************************************************
 * This application implements the reader-IMD security protocol and the
 * required security primitives. The protocol is described in
 * https://arxiv.org/abs/2002.09546
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_cryotimer.h"
#include "bsp.h"
#include "security.h"
#include "housekeeping.h"

uint8_t protocol_state = STATE_IDLE;
uint8_t nonce_I[NONCE_SIZE];
uint8_t s_nonce_I[NONCE_SIZE]; // received from the server
uint8_t nonce_R[NONCE_SIZE];
uint8_t r_nonce_R[NONCE_SIZE]; // received from R the second time
uint8_t r_nonce_I[NONCE_SIZE]; // received from R the second time
uint8_t nonce_C[NONCE_SIZE];
uint8_t id_R[ID_SIZE];
uint8_t s_id_R[ID_SIZE]; // received from the server
uint8_t id_C[ID_SIZE];
uint8_t id_I[ID_SIZE] = {0xBA, 0xAD, 0xDA, 0xAB};
uint8_t privilege_C; // User privilege information
uint8_t sig[SIG_SIZE];
uint64_t PrngCounter = PRNG_SEED;
uint8_t CmdCodeword = 0;

// Pre-shared key between the server and IMD
uint8_t key_SI[] = {0x00,0x01,0x02,0x03,0x08,0x09,0x0a,0x0b,0x10,0x11,0x12,0x13,0x18,0x19,0x1a,0x1b};

// Short-term session key between the reader and the IMD
uint8_t key_RI[KEY_SIZE];

uint8_t valid;

uint8_t Plaintext8bit[BUFFER_PLAINTEXT_SIZE] = {0};
uint8_t RxPlaintext8bit[BUFFER_PLAINTEXT_SIZE] = {0};

uint8_t receivedMAC[MAC_SIZE];
uint8_t calculatedMAC[MAC_SIZE];
uint8_t receivedMAC1[MAC_SIZE];
uint8_t calculatedMAC1[MAC_SIZE];
uint8_t MACinput_block_sized[CIPHER_BLOCK_SIZE] = {0};

// BCC interface related
// Note: change this to one of the defined periods in em_cryotimer.h
// Wakeup events occur every 8 prescaled clock cycles
#define CRYOTIMER_PERIOD    cryotimerPeriod_8
// Note: change this to one of the defined prescalers in em_cryotimer.h
// The clock is divided by one
#define CRYOTIMER_PRESCALE  cryotimerPresc_1
uint16_t bcc_receiving = 0;
uint16_t bcc_transmitting = 0;
uint16_t bcc_rx_index = 0;
uint16_t bcc_tx_index = 0;
uint8_t bcc_sample;
uint32_t bcc_rx_packet[4];
uint32_t bcc_tx_packet[6];
uint8_t offline_key_1[NONCE_SIZE];
uint8_t offline_key_2[NONCE_SIZE];
uint8_t offline_key_3[NONCE_SIZE];
uint8_t offline_key_4[NONCE_SIZE];

/**************************************************************************//**
 * @brief CMAC algorithm which uses the employed cipher
 *****************************************************************************/
void cmac(CRYPTO_TypeDef *crypto, uint8_t *input, uint16_t start_index, uint16_t length, uint8_t *output, uint8_t *key) {
  uint8_t C[CIPHER_BLOCK_SIZE] = {0};
  uint8_t plaintext[CIPHER_BLOCK_SIZE];
  int size_of_msg = sizeof(plaintext);

  for (uint16_t i=start_index; i < (start_index+length); i=i+CIPHER_BLOCK_SIZE) {
    for (uint16_t j=0; j < CIPHER_BLOCK_SIZE; j=j+1) {
      plaintext[j] = (C[j] ^ input[i+j]);
    }
    CRYPTO_AES_ECB128(crypto, C, plaintext, size_of_msg, key, true);
  }

  for (uint16_t i=0; i < MAC_SIZE; i=i+1) { 
    output[i] = C[i];
  }
}

/**************************************************************************//**
 * @brief CMAC calculation of the firmware
 *****************************************************************************/
void cmac_firmware(CRYPTO_TypeDef *crypto, uint8_t *output, uint8_t *key) {

  uint32_t *flash_addr = (uint32_t *)FW_START_ADDRESS;
  uint8_t flash_word[CIPHER_BLOCK_SIZE];
  uint8_t C[CIPHER_BLOCK_SIZE] = {0};
  uint8_t plaintext[CIPHER_BLOCK_SIZE];
  int size_of_msg = sizeof(plaintext);

  for (uint32_t i = 0; i < FW_PAYLOAD_SIZE; i=i+CIPHER_BLOCK_SIZE) {
    for (uint16_t k=0; k < CIPHER_BLOCK_SIZE; k=k+4) {
      flash_addr = (uint32_t *)(FW_START_ADDRESS+i+k);
      split_32bits_into_4x8(flash_word, k, *flash_addr);
    }
    for (uint16_t j=0; j < CIPHER_BLOCK_SIZE; j=j+1) {
      plaintext[j] = (C[j] ^ flash_word[j]);
    }
    CRYPTO_AES_ECB128(crypto, C, plaintext, size_of_msg, key, true);
  }

  for (uint16_t i=0; i < MAC_SIZE; i=i+1) { 
    output[i] = C[i];
  }
}

/**************************************************************************//**
 * @brief Cipher-based random number generator - Counter mode
 *****************************************************************************/
void crand(CRYPTO_TypeDef *crypto, uint64_t *counter, uint8_t *output, uint8_t *key) {
  uint8_t counter_split[CIPHER_BLOCK_SIZE];
  uint8_t random_number[CIPHER_BLOCK_SIZE];
  int size_of_msg = sizeof(counter_split);

  // Splitting count value (only loading the lower half with the counter)
  counter_split[0] = (*counter & 0xFF);
  counter_split[1] = (*counter & 0xFF00) >> 8;
  counter_split[2] = (*counter & 0xFF0000) >> 16;
  counter_split[3] = (*counter & 0xFF000000) >> 24;
  counter_split[4] = (*counter & 0xFF00000000) >> 32;
  counter_split[5] = (*counter & 0xFF0000000000) >> 40;
  counter_split[6] = (*counter & 0xFF000000000000) >> 48;
  counter_split[7] = (*counter & 0xFF00000000000000) >> 56;

  CRYPTO_AES_ECB128(crypto, random_number, counter_split, size_of_msg, key, true);

  // Increment counter
  (*counter)++;

  memcpy(output, random_number, NONCE_SIZE);
}

/**************************************************************************//**
 * @brief Comparing two cipher results
 *****************************************************************************/
uint8_t cipher_compare(const void *source, void *target, size_t n) {
  int fail_count = 0;
  for(size_t i=0; i < n; i++) {
    uint8_t * src_bytes = (uint8_t *)source;
    uint8_t * trg_bytes = (uint8_t *)target;
    if (src_bytes[i] != trg_bytes[i]) {
      fail_count++;
    }
  }

  if (fail_count>0){
    return 0;
  }
  else {
    return 1;
  }
}

/**************************************************************************//**
 * @brief This function packs all the MAC input together
 *****************************************************************************/
void pack_block_sized_mac_input(uint8_t * mac_input, uint8_t * nonce_1, uint8_t * nonce_2, uint8_t * id_1, uint8_t * id_2) {

  memcpy(&mac_input[0], nonce_1, NONCE_SIZE);
  memcpy(&mac_input[NONCE_SIZE], nonce_2, NONCE_SIZE);
  memcpy(&mac_input[2*NONCE_SIZE], id_1, ID_SIZE);
  memcpy(&mac_input[2*NONCE_SIZE+ID_SIZE], id_2, ID_SIZE);
}

/**************************************************************************//**
 * @brief Initialize Cryotimer
 *
 * @note
 *    No need to enable the ULFRCO since it is always on and cannot be shut off
 *    under software control. The ULFRCO is used in this example because it is
 *    the only oscillator capable of running in EM3.
 *****************************************************************************/
void initCryotimer(void)
{
  // Enable cryotimer clock
  CMU_ClockEnable(cmuClock_CRYOTIMER, true);

  // Initialize cryotimer
  CRYOTIMER_Init_TypeDef init = CRYOTIMER_INIT_DEFAULT;
  init.osc = cryotimerOscULFRCO;   // Use the ULFRCO
  init.presc = CRYOTIMER_PRESCALE; // Set the prescaler
  init.period = CRYOTIMER_PERIOD;  // Set when wakeup events occur
  init.enable = true;              // Start the cryotimer after initialization is done
  CRYOTIMER_Init(&init);

  // Enable cryotimer interrupts
  CRYOTIMER_IntEnable(CRYOTIMER_IEN_PERIOD);
  NVIC_EnableIRQ(CRYOTIMER_IRQn);
}

/**************************************************************************//**
 * @brief Disable Cryotimer
 *****************************************************************************/
void disableCryotimer(void)
{
  CRYOTIMER_Enable(false);
  // Disable cryotimer interrupts
  CRYOTIMER_IntDisable(CRYOTIMER_IEN_PERIOD);
  NVIC_DisableIRQ(CRYOTIMER_IRQn);
  // Disable cryotimer clock
  CMU_ClockEnable(cmuClock_CRYOTIMER, false);  
}

/**************************************************************************//**
 * @brief GPIO Odd IRQ for odd-numbered pins
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  // Disable IRQ to avoid retriggering
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  CMU_ClockEnable(cmuClock_CRYPTO0, true);
  
  // Check whether BTN1 is pressed. This emulates GPIO interrupt from the radio TRX
  if((GPIO->IF & 0x00000200) == 0x0200) {

    if (protocol_state == STATE_IDLE) {

      // Switching to the harvested-energy supply
      GPIO_PinOutSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      
      // receive first message from the reader
      read_rx_buffer(RxBuffer, USART0);

      // extract ID_R and N_R
      memcpy(id_R, &RxBuffer[0], ID_SIZE); 
      memcpy(nonce_R, &RxBuffer[ID_SIZE], NONCE_SIZE);

      // generate nonce
      crand(CRYPTO0, &PrngCounter, nonce_I, key_SI);
      
      // construct response
      memcpy(&TxBuffer[0], id_I, ID_SIZE); 
      memcpy(&TxBuffer[ID_SIZE], nonce_I, NONCE_SIZE);
      
      // send response to the TRX, which will forward it to the reader
      write_tx_buffer(TxBuffer, USART0);

      protocol_state = STATE_TRX0;
      
    } else if (protocol_state == STATE_TRX0) {

      // Receive second message from the reader
      read_rx_buffer(RxBuffer, USART0);

      // Retrieving the received MACs
      for (uint32_t i=CIPHER_BLOCK_SIZE; i < (CIPHER_BLOCK_SIZE+MAC_SIZE); i=i+1){
	receivedMAC[i-CIPHER_BLOCK_SIZE] = RxBuffer[i];
      }
      for (uint32_t i=CIPHER_BLOCK_SIZE+MAC_SIZE+(3*CIPHER_BLOCK_SIZE); i < (CIPHER_BLOCK_SIZE+MAC_SIZE+(3*CIPHER_BLOCK_SIZE)+MAC_SIZE); i=i+1){
	receivedMAC1[i-(CIPHER_BLOCK_SIZE+MAC_SIZE+(3*CIPHER_BLOCK_SIZE))] = RxBuffer[i];
      }

      // Calculating MAC of m_I
      cmac(CRYPTO0, RxBuffer, CIPHER_BLOCK_SIZE+MAC_SIZE, 3*CIPHER_BLOCK_SIZE, calculatedMAC1, key_SI);
	
      // Decrypting m_I
      for (uint32_t i=CIPHER_BLOCK_SIZE+MAC_SIZE; i < (CIPHER_BLOCK_SIZE+MAC_SIZE+(3*CIPHER_BLOCK_SIZE)); i=i+CIPHER_BLOCK_SIZE){
	CRYPTO_AES_ECB128(CRYPTO0, &Plaintext8bit[i-(CIPHER_BLOCK_SIZE+MAC_SIZE)], &RxBuffer[i], CIPHER_BLOCK_SIZE, key_SI, false);
      }
	
      //Extracting the session key, privilege info, IDs and nonces
      memcpy(key_RI,     &Plaintext8bit[0], KEY_SIZE);
      memcpy(s_id_R,     &Plaintext8bit[KEY_SIZE], ID_SIZE);
      memcpy(s_nonce_I,  &Plaintext8bit[KEY_SIZE+ID_SIZE], NONCE_SIZE);
      memcpy(id_C,       &Plaintext8bit[KEY_SIZE+ID_SIZE+NONCE_SIZE], ID_SIZE);
      memcpy(nonce_C,    &Plaintext8bit[KEY_SIZE+ID_SIZE+NONCE_SIZE+ID_SIZE], NONCE_SIZE);
      memcpy(&privilege_C,&Plaintext8bit[KEY_SIZE+ID_SIZE+NONCE_SIZE+ID_SIZE+NONCE_SIZE], sizeof(privilege_C));

      // Calculating MAC of m_RI
      cmac(CRYPTO0, RxBuffer, 0, CIPHER_BLOCK_SIZE, calculatedMAC, key_RI);
	
      // Decrypting m_RI
      for (uint32_t i=0; i < CIPHER_BLOCK_SIZE; i=i+CIPHER_BLOCK_SIZE){
	CRYPTO_AES_ECB128(CRYPTO0, &Plaintext8bit[i], &RxBuffer[i], CIPHER_BLOCK_SIZE, key_RI, false);
      }

      //Extracting nonces
      memcpy(r_nonce_R, &Plaintext8bit[0], NONCE_SIZE);
      memcpy(r_nonce_I, &Plaintext8bit[NONCE_SIZE], NONCE_SIZE);

      valid = cipher_compare(&calculatedMAC, &receivedMAC, MAC_SIZE)
	& cipher_compare(&calculatedMAC1, &receivedMAC1, MAC_SIZE)
	& cipher_compare(&r_nonce_R, &nonce_R, NONCE_SIZE)
	& cipher_compare(&r_nonce_I, &nonce_I, NONCE_SIZE)
	& cipher_compare(&s_id_R, &id_R, ID_SIZE)
	& cipher_compare(&s_nonce_I, &nonce_I, NONCE_SIZE);

      if (valid){
	
	// Switching to the main supply
	GPIO_PinOutClear(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);

	// Creating response for the reader
	pack_block_sized_mac_input(MACinput_block_sized, nonce_I, nonce_R, id_I, id_R);
	cmac(CRYPTO0, MACinput_block_sized, 0, CIPHER_BLOCK_SIZE, calculatedMAC, key_RI);
	memcpy(&TxBuffer[0], calculatedMAC, MAC_SIZE);
	
	// Send response
	write_tx_buffer(TxBuffer, USART0);      
	
	protocol_state = STATE_TRX1;
      
      } else {
	
	// Doing nothing, i.e., aborting protocol (due to cryptogram (m_I, m_RI)-MAC-check failures)

	// Switching to the main supply
	GPIO_PinOutClear(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
	
        protocol_state = STATE_IDLE;
    
      }
      
    } else if (protocol_state == STATE_TRX1) {
      
      // receive third message
      read_rx_buffer(RxBuffer, USART0);

      // Retrieving received MAC
      for (uint32_t i=BUFFER_PLAINTEXT_SIZE; i < (BUFFER_PLAINTEXT_SIZE+MAC_SIZE); i=i+1){
	receivedMAC[i-(BUFFER_PLAINTEXT_SIZE)] = RxBuffer[i];
      }
      
      // Locally calculating MAC
      cmac(CRYPTO0, RxBuffer, 0, BUFFER_PLAINTEXT_SIZE, calculatedMAC, key_RI);

      valid = cipher_compare(&calculatedMAC, &receivedMAC, MAC_SIZE);

      // Verifying received MAC
      if (valid){
	
	// Decrypting received blocks
        for (uint32_t i=0; i < (BUFFER_PLAINTEXT_SIZE); i=i+CIPHER_BLOCK_SIZE){
	  CRYPTO_AES_ECB128(CRYPTO0, &Plaintext8bit[i], &RxBuffer[i], CIPHER_BLOCK_SIZE, key_RI, false);
	}

        // Extracting command codeword from the received packet
	CmdCodeword = Plaintext8bit[PAYLOAD_SIZE];

	// Access control
	if (CmdCodeword < privilege_C) {

	  if ((CmdCodeword == UPDATE_SISC_FIRMWARE) || (CmdCodeword == UPDATE_SIMS_FIRMWARE)) {

	    // construct ack to send to the external reader
	    pack_block_sized_mac_input(MACinput_block_sized, nonce_I, nonce_R, id_I, id_R);
	    cmac(CRYPTO0, MACinput_block_sized, 0, CIPHER_BLOCK_SIZE, calculatedMAC, key_RI);
	    memcpy(&TxBuffer[0], calculatedMAC, MAC_SIZE);
      
	    // send response to the TRX
	    write_tx_buffer(TxBuffer, USART0);

	    protocol_state = STATE_FW_DL;

	  }  else if (CmdCodeword == SEND_SIGNATURE) {

	    // Fetch the signature and send to TRX
	    send_signatures(USART0);

	    protocol_state = STATE_IDLE;
	  
	  } else {

	    // Extracting signature from the command message
	    memcpy(sig, &Plaintext8bit[PAYLOAD_SIZE+HEADER_SIZE], SIG_SIZE);

	    // Storing: ID_C, N_C, N_R, CMD_codeword, sig
	    // 32 + 32 + 32 + 32(actually 8) + 384 = 64 bytes

	    uint32_t *sig_index_addr = (uint32_t *)SIG_INDEX_ADDRESS;
	    uint32_t sig_index = *sig_index_addr;
	    uint32_t CmdCodeword_array[] = {CmdCodeword};
	    uint32_t sig_index_array[] = {0};

	    // Retrieving index of the next signature	    
	    if ((sig_index >= (FIRST_SIG_ADDRESS+NO_OF_SIG_ENTRIES*FLASH_PAGE_SIZE)) || (sig_index < FIRST_SIG_ADDRESS)) {
	      // This is the first ever signature, hence initializing it
	      sig_index = FIRST_SIG_ADDRESS;
	    }
	    
	    // Erasing the pages first (not actually doing it since min. 10K erase cycles endurance)
	    MSC_Init();
	    MSC_ErasePage(sig_index_addr);
	    MSC_ErasePage((uint32_t *)sig_index);

	    // Storing the signature along with other parameters
	    MSC_WriteWord((uint32_t *)sig_index, id_C, sizeof(id_C));
	    MSC_WriteWord((uint32_t *)(sig_index+4), nonce_C, sizeof(nonce_C));
	    MSC_WriteWord((uint32_t *)(sig_index+8), nonce_R, sizeof(nonce_R));
	    MSC_WriteWord((uint32_t *)(sig_index+12), CmdCodeword_array, sizeof(CmdCodeword_array));
	    MSC_WriteWord((uint32_t *)(sig_index+16), sig, sizeof(sig));
	    MSC_Deinit();
	    
	    // Incrementing the signature address for future round
	    sig_index = sig_index + FLASH_PAGE_SIZE;

	    sig_index_array[0] = sig_index;
	    MSC_WriteWord(sig_index_addr, sig_index_array, sizeof(sig_index_array));
	    
	    // Send GPIO interrupt to the main processor
	    GPIO_PinOutSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
	    GPIO_PinOutClear(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);

	    // Send data to the main processor using USART1    
	    for (uint32_t i=0; i < (PAYLOAD_SIZE+HEADER_SIZE); i=i+1){
	      RxPlaintext8bit[i] = USART_SpiTransfer(USART1, Plaintext8bit[i]);
	    }

	    protocol_state = STATE_TRX2;
	  }
	  
	} else {
	
	  // Doing nothing, i.e., aborting protocol (due to access-control-check failure)
	  protocol_state = STATE_IDLE;
	}
		
      } else {
	
	// Doing nothing, i.e., aborting protocol (due to MAC-check failure in the command message)	
        protocol_state = STATE_IDLE;
    
      }
      
    } else if (protocol_state == STATE_FW_DL) {

      // receive firmware from the reader via TRX
      get_firmware(USART0, receivedMAC);
      
      // Performing auth. and integrity checks on the firmware
      cmac_firmware(CRYPTO0, calculatedMAC, key_RI);
      valid = cipher_compare(&calculatedMAC, &receivedMAC, MAC_SIZE);

      if (valid) {

        if (CmdCodeword == UPDATE_SISC_FIRMWARE) {
	  // Reboot and install new firmware using Application Bootloader
	  bootloader_rebootAndInstall();
	  
	  protocol_state = STATE_IDLE;
	  
	} else if (CmdCodeword == UPDATE_SIMS_FIRMWARE) {

	  // Send GPIO interrupt to the main processor
	  GPIO_PinOutSet(gpioPortC, 3);
	  GPIO_PinOutClear(gpioPortC, 3);

	  // Send firmware to the main processor using USART1
	  send_firmware(USART1);
	  
	  protocol_state = STATE_IDLE;
	}
	
      } else {

        // Doing nothing, i.e., aborting protocol (Due to MAC-check failure in the firmware-update-command message)
        protocol_state = STATE_IDLE;	
      }
      
    }
  }

  // Check whether BTN0 is pressed. This emulates GPIO interrupt from the main co-processor
  if(((GPIO->IF & 0x00000020) == 0x0020) && (protocol_state == STATE_TRX2)) {

    // Receive data from the co-processor
    for (uint32_t i=0; i < (PAYLOAD_SIZE+HEADER_SIZE); i=i+1){
      RxPlaintext8bit[i] = USART_SpiTransfer(USART1, Plaintext8bit[i]);
    }
	
    // Encrypting blocks before sending to the TRX
    for (uint32_t i=0; i < (PAYLOAD_SIZE+HEADER_SIZE); i=i+CIPHER_BLOCK_SIZE){
      CRYPTO_AES_ECB128(CRYPTO0, &TxBuffer[i], &RxPlaintext8bit[i], CIPHER_BLOCK_SIZE, key_RI, true);
    }

    // Calculating MAC
    cmac(CRYPTO0, TxBuffer, 0, PAYLOAD_SIZE+HEADER_SIZE, calculatedMAC, key_RI);

    // Appending MAC with the encrypted data
    memcpy(&TxBuffer[PAYLOAD_SIZE+HEADER_SIZE], calculatedMAC, MAC_SIZE);

    // Sending the encrypted message to the TRX
    write_tx_buffer(TxBuffer, USART0);
      
    protocol_state = STATE_IDLE;
    
  } else if(((GPIO->IF & 0x00000020) == 0x0020) && (protocol_state == STATE_IDLE)) {
    // Check whether we got a signal from the BCC interface.
    // Initialize and start the cryotimer
    initCryotimer();
    bcc_receiving = 1;

    if (bcc_rx_index > (BCC_RX_PACKET_SIZE-1)) {
      disableCryotimer();
      // Extract data
      memcpy(id_R, &bcc_rx_packet[0], ID_SIZE); 
      memcpy(id_C, &bcc_rx_packet[1], ID_SIZE); 
      memcpy(nonce_C, &bcc_rx_packet[2], NONCE_SIZE); 

      // Constructing response
      crand(CRYPTO0, &PrngCounter, offline_key_1, key_SI);
      crand(CRYPTO0, &PrngCounter, offline_key_2, key_SI);
      crand(CRYPTO0, &PrngCounter, offline_key_3, key_SI);
      crand(CRYPTO0, &PrngCounter, offline_key_4, key_SI);
      crand(CRYPTO0, &PrngCounter, nonce_I, key_SI);
     
      memcpy(&bcc_tx_packet[0], offline_key_1, NONCE_SIZE); 
      memcpy(&bcc_tx_packet[1], offline_key_2, NONCE_SIZE); 
      memcpy(&bcc_tx_packet[2], offline_key_3, NONCE_SIZE); 
      memcpy(&bcc_tx_packet[3], offline_key_4, NONCE_SIZE); 
      memcpy(&bcc_tx_packet[4], id_I, ID_SIZE); 
      memcpy(&bcc_tx_packet[5], nonce_I, NONCE_SIZE); 
      
      initCryotimer();
      bcc_transmitting = 1;
    }
  }

  CMU_ClockEnable(cmuClock_CRYPTO0, false);
  
  // Acknowledge interrupt
  GPIO_IntClear(0xAAAA);
  // Enabling IRQ again
  NVIC_EnableIRQ(GPIO_ODD_IRQn);  
}

/**************************************************************************//**
 * @brief Cryotimer interrupt service routine
 *****************************************************************************/
void CRYOTIMER_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = CRYOTIMER_IntGet();
  uint32_t cnt = CRYOTIMER->CNT;
  CRYOTIMER_IntClear(flags);

  if (bcc_receiving && (!bcc_transmitting)) {
    // Sample the BCC data from the respective GPIO pin
    bcc_sample = GPIO_PinOutGet(gpioPortC, 5);
    if (bcc_rx_index < 32) {
      bcc_rx_packet[0] = bcc_rx_packet[0] << 1 | (bcc_sample & 0x1);
    } else if ((bcc_rx_index < 64) && (bcc_rx_index >= 32)) {
      bcc_rx_packet[1] = bcc_rx_packet[1] << 1 | (bcc_sample & 0x1);
    } else if ((bcc_rx_index < 96) && (bcc_rx_index >= 64)) {
      bcc_rx_packet[2] = bcc_rx_packet[2] << 1 | (bcc_sample & 0x1);
    } else {
      bcc_rx_packet[3] = bcc_rx_packet[3] << 1 | (bcc_sample & 0x1);
    }
    
    bcc_rx_index++;
    
  } else if (bcc_transmitting) {

    GPIO_PinOutSet(gpioPortC, 4);
    
    if (bcc_tx_index < 32) {
      bcc_sample = bcc_tx_packet[0] & 0x1;
      bcc_tx_packet[0] = bcc_tx_packet[0] >> 1;
    } else if ((bcc_tx_index < 64) && (bcc_tx_index >= 32)) {
      bcc_sample = bcc_tx_packet[1] & 0x1;
      bcc_tx_packet[1] = bcc_tx_packet[1] >> 1;
    } else if ((bcc_tx_index < 96) && (bcc_tx_index >= 64)) {
      bcc_sample = bcc_tx_packet[2] & 0x1;
      bcc_tx_packet[2] = bcc_tx_packet[2] >> 1;
    } else if ((bcc_tx_index < 128) && (bcc_tx_index >= 96)) {
      bcc_sample = bcc_tx_packet[3] & 0x1;
      bcc_tx_packet[3] = bcc_tx_packet[3] >> 1;
    } else if ((bcc_tx_index < 160) && (bcc_tx_index >= 128)) {
      bcc_sample = bcc_tx_packet[4] & 0x1;
      bcc_tx_packet[4] = bcc_tx_packet[4] >> 1;
    } else {
      bcc_sample = bcc_tx_packet[5] & 0x1;
      bcc_tx_packet[5] = bcc_tx_packet[5] >> 1;
    }

    if (bcc_sample == 1) {
      // Clearing GPIO after a longer delay (sending 1)
      while (CRYOTIMER->CNT < (cnt+5));
      GPIO_PinOutClear(gpioPortC, 4);
    } else {
      // Clearing GPIO after a shorter delay (sending 0)
      while (CRYOTIMER->CNT < (cnt+2));
      GPIO_PinOutClear(gpioPortC, 4);
    }
    
    bcc_tx_index++;

    if (bcc_tx_index > (BCC_TX_PACKET_SIZE-1)){
      bcc_transmitting = 0;
      bcc_rx_index = 0;
      bcc_tx_index = 0;
      disableCryotimer();
      protocol_state = STATE_TRX1;
      // Assigning paramedic level privileges (equivalent to doctor) in emergency mode
      privilege_C = DOCTOR;
    }
    
  } else {
    // No new bit received, hence the BCC session has finished/expired.
    bcc_rx_index = 0;
    bcc_tx_index = 0;
    disableCryotimer();
  }
  bcc_receiving = 0;
}

/**************************************************************************//**
 * @brief Initialize LED and Push-button pins
 *****************************************************************************/
void initGpio(void)
{
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PB1 (push-button #1) (PC9) as input with glitch filter enabled. This emulates GPIO interrupt from the radio TRX.
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPullFilter, 1);

  // Configure PB0 (push-button #0) (PD5) as input with glitch filter enabled. This emulates GPIO interrupt from the main co-processor.
  // This is also connected to the input signal received from the BCC interface (which is also connected to PC5 below).
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPullFilter, 1);

  // Configure LED0 pin (PD2) as output. This LED denotes sending GPIO interrupt to the main co-processor.
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);
  // Configure LED1 pin (PC2) as output. This LED denotes the input to the switch that selects between the harvested-energy source and the main battery.
  GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull, 0);

  // Configure PC3 as output. This is for sending GPIO interrupt to the main co-processor to indicate firmware update.
  GPIO_PinModeSet(gpioPortC, 3, gpioModePushPull, 0);

  // Configure PC4 as output. This is used for sending data via the BCC interface.
  GPIO_PinModeSet(gpioPortC, 4, gpioModePushPull, 0);

  // Configure PC5 as input. This is used for receiving data via the BCC interface.
  GPIO_PinModeSet(gpioPortC, 5, gpioModeInput, 0);

  // Enable IRQ for odd numbered GPIO pins
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  // Enable falling/rising-edge interrupts for PB pins
  GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, BSP_GPIO_PB1_PIN, 0, 1, true);
  GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, BSP_GPIO_PB0_PIN, 1, 0, true);
  
}

/**************************************************************************//**
 * @brief Initialize USART0 as SPI master; used to communicate with the IMD TRX
 *****************************************************************************/
void initUSART0 (void)
{
  CMU_ClockEnable(cmuClock_USART0, true);

  // Configure GPIO mode
  GPIO_PinModeSet(gpioPortA, 12, gpioModePushPull, 0); // US0_CLK is push pull
  GPIO_PinModeSet(gpioPortC, 8, gpioModePushPull, 1);  // US0_CS is push pull
  GPIO_PinModeSet(gpioPortC, 11, gpioModePushPull, 1); // US0_TX (MOSI) is push pull
  GPIO_PinModeSet(gpioPortC, 10, gpioModeInput, 1);    // US0_RX (MISO) is input

  // Start with default config, then modify as necessary
  USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
  config.master       = true;            // master mode
  config.baudrate     = 1000000;         // CLK freq is 1 MHz
  config.autoCsEnable = true;            // CS pin controlled by hardware, not firmware
  config.clockMode    = usartClockMode0; // clock idle low, sample on rising/first edge
  config.msbf         = true;            // send MSB first
  config.enable       = usartDisable;    // Make sure to keep USART disabled until it's all set up
  USART_InitSync(USART0, &config);

  // Set USART pin locations
  USART0->ROUTELOC0 = (USART_ROUTELOC0_CLKLOC_LOC5) | // US0_CLK       on location 5 = PA12 per datasheet section 6.4 = EXP Header pin 8
                      (USART_ROUTELOC0_CSLOC_LOC2)  | // US0_CS        on location 2 = PC8 per datasheet section 6.4 = EXP Header pin 10
                      (USART_ROUTELOC0_TXLOC_LOC2)  | // US0_TX (MOSI) on location 2 = PC11 per datasheet section 6.4 = EXP Header pin 4
                      (USART_ROUTELOC0_RXLOC_LOC2);   // US0_RX (MISO) on location 2 = PC10 per datasheet section 6.4 = EXP Header pin 6

  // Enable USART pins
  USART0->ROUTEPEN = USART_ROUTEPEN_CLKPEN | USART_ROUTEPEN_CSPEN | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;

  // Enable USART0
  USART_Enable(USART0, usartEnable);
  
  // Enabling ZL70103 TRX IRQ (RX buffer not empty)
  RxDummyBuffer = USART_SpiTransfer(USART0, TRX_RXIRQ_ADDRESS);
  RxDummyBuffer = USART_SpiTransfer(USART0, 0x01);

  // Configuring the TRX block-size
  RxDummyBuffer = USART_SpiTransfer(USART0, TRX_BLOCKSIZE_ADDRESS);
  RxDummyBuffer = USART_SpiTransfer(USART0, BLOCK_SIZE);

  // Configuring the TRX packet-size
  RxDummyBuffer = USART_SpiTransfer(USART0, TRX_PACKETSIZE_ADDRESS);
  RxDummyBuffer = USART_SpiTransfer(USART0, PACKET_SIZE);
  
}

/**************************************************************************//**
 * @brief Initialize USART1 as SPI master to communicate with the main MCU
 *****************************************************************************/
void initUSART1 (void)
{
  CMU_ClockEnable(cmuClock_USART1, true);

  // Configure GPIO mode
  GPIO_PinModeSet(gpioPortB, 7, gpioModePushPull, 0); // US1_CLK is push pull
  GPIO_PinModeSet(gpioPortB, 8, gpioModePushPull, 1);  // US1_CS is push pull
  GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1); // US1_TX (MOSI) is push pull
  GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 1);    // US1_RX (MISO) is input

  // Start with default config, then modify as necessary
  USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
  config.master       = true;            // master mode
  config.baudrate     = 1000000;         // CLK freq is 1 MHz
  config.autoCsEnable = true;            // CS pin controlled by hardware, not firmware
  config.clockMode    = usartClockMode0; // clock idle low, sample on rising/first edge
  config.msbf         = true;            // send MSB first
  config.enable       = usartDisable;    // Make sure to keep USART disabled until it's all set up
  USART_InitSync(USART1, &config);

  // Set USART pin locations
  USART1->ROUTELOC0 = (USART_ROUTELOC0_CLKLOC_LOC0) | // US1_CLK       on location 0 = PB7 per datasheet section 6.4
                      (USART_ROUTELOC0_CSLOC_LOC0)  | // US1_CS        on location 0 = PB8 per datasheet section 6.4
                      (USART_ROUTELOC0_TXLOC_LOC0)  | // US1_TX (MOSI) on location 0 = PC0 per datasheet section 6.4
                      (USART_ROUTELOC0_RXLOC_LOC0);   // US1_RX (MISO) on location 0 = PC1 per datasheet section 6.4

  // Enable USART pins
  USART1->ROUTEPEN = USART_ROUTEPEN_CLKPEN | USART_ROUTEPEN_CSPEN | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;

  // Enable USART1
  USART_Enable(USART1, usartEnable);  
}

/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;

  /* Chip errata */  
  CHIP_Init();

  EMU_DCDCInit(&dcdcInit);

  // Initializations
  initGpio(); 
  initUSART0();
  initUSART1();

  while (1) {

    // Wait for GPIO interrupt
    EMU_EnterEM3(false);
  }
}

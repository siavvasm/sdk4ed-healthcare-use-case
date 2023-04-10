/**************************************************************************//**
 * @file
 * @brief Definitions related to the security-core's SPI interface with the TRX
 * @version 0.0.1
 ******************************************************************************/

#include "spi_interface_sisc.h"
#include "security.h"

uint8_t TxBuffer[TRX_BUFFER_SIZE] = {0};
uint8_t TxDummyBuffer = 0;
uint8_t RxBuffer[TRX_BUFFER_SIZE] = {0};
uint8_t RxDummyBuffer = 0;

/**************************************************************************//**
 * @brief Send TxBuffer data to the TRX via SPI
 *****************************************************************************/
void write_tx_buffer(uint8_t * tx_buffer, USART_TypeDef *usart)
{    
  for (uint16_t i = 0; i < TRX_BUFFER_SIZE; i=i+1) {

    if (i == 0) {
      // Sending write-access address while receiving dummy data
      RxDummyBuffer = USART_SpiTransfer(usart, TRX_WRITE_ADDRESS);
    }

    // Sending data to TRX while receiving dummy data
    RxDummyBuffer = USART_SpiTransfer(usart, tx_buffer[i]);
  }  
}

/**************************************************************************//**
 * @brief Receive RxBuffer data from the TRX via SPI
 *****************************************************************************/
void read_rx_buffer(uint8_t * rx_buffer, USART_TypeDef *usart)
{
  // Receiving 20 blocks of data from the TRX while sending dummy data
  // Each block comprises of 14 bytes
  // Frame format:
  // <--PAYLOAD_SIZE-->
  //                      <-----------HEADER_SIZE---------->
  //                                                           <----SIG_SIZE--->
  //                                                                      
  // 256*8 bits payload + 8 bit codeword + 120 bits reserved + 384 bit signature + 64 bit MAC + 64 bits extra
  // = 20 blocks * 14 bytes per block
  
  for (uint16_t i = 0; i < TRX_BUFFER_SIZE; i=i+1) {

    if (i == 0) {
      // Sending read-access address while receiving dummy data
      RxDummyBuffer = USART_SpiTransfer(usart, TRX_READ_ADDRESS);
    }
      
    // Receiving actual data from TRX while sending dummy data
    rx_buffer[i] = USART_SpiTransfer(usart, TxDummyBuffer);
  }  
}

/**************************************************************************//**
 * @brief Receive firmware update from the TRX via SPI and store in internal flash
 *****************************************************************************/
void get_firmware(USART_TypeDef *usart, uint8_t * received_mac)
{
  // Receiving 20 blocks of data from the TRX while sending dummy data
  // Each block comprises of 14 bytes
  // Frame format:
  // <-16 kB PAYLOAD_SIZE->
  //                                                                      
  // 16*1024*8 bits payload + 64 bit MAC + 128 bits extra
  // = 9376 blocks * 14 bytes per block

  uint8_t rx_buffer[4]; // 32-bit flash location
  uint32_t flash_word[] = {0};
  uint32_t *addr = (uint32_t *)FW_START_ADDRESS;

  MSC_Init();

  for (uint32_t i = 0; i < FW_PAYLOAD_SIZE/FLASH_PAGE_SIZE; i=i+1) {
    addr = (uint32_t *)(FW_START_ADDRESS+i*FLASH_PAGE_SIZE);
    // Erasing the page first (not actually doing it since min. 10K erase cycles endurance)
    MSC_ErasePage(addr);
  }

  for (uint32_t i = 0; i < TRX_FW_BUFFER_SIZE; i=i+4) {

    if (i == 0) {
      // Sending read-access address while receiving dummy data
      RxDummyBuffer = USART_SpiTransfer(usart, TRX_READ_ADDRESS);
    }
    
    // Receiving actual data from TRX while sending dummy data
    rx_buffer[0] = USART_SpiTransfer(usart, TxDummyBuffer);
    rx_buffer[1] = USART_SpiTransfer(usart, TxDummyBuffer);
    rx_buffer[2] = USART_SpiTransfer(usart, TxDummyBuffer);
    rx_buffer[3] = USART_SpiTransfer(usart, TxDummyBuffer);

    if (i < FW_PAYLOAD_SIZE) {
      pack_4x8bits_into_32(rx_buffer, 0, flash_word);

      addr = (uint32_t *)(FW_START_ADDRESS+i);
      // write this value to flash (won't actually write it)
      MSC_WriteWord(addr, flash_word, sizeof(flash_word));
    }

    if ((i >= FW_PAYLOAD_SIZE) && (i < (FW_PAYLOAD_SIZE+MAC_SIZE))) {
      // Receiving the MAC of the firmware for auth. and integrity check
      received_mac[i-FW_PAYLOAD_SIZE] = rx_buffer[0];
      received_mac[i-FW_PAYLOAD_SIZE+1] = rx_buffer[1];
      received_mac[i-FW_PAYLOAD_SIZE+2] = rx_buffer[2];
      received_mac[i-FW_PAYLOAD_SIZE+3] = rx_buffer[3];
    }  
  }

  MSC_Deinit();
  
}

/**************************************************************************//**
 * @brief Send firmware to the main processor
 *****************************************************************************/
void send_firmware(USART_TypeDef *usart)
{

  uint32_t *flash_addr = (uint32_t *)FW_START_ADDRESS;
  uint8_t flash_word[4];
  
  for (uint32_t i=0; i < FW_PAYLOAD_SIZE; i=i+4){
    
    flash_addr = (uint32_t *)(FW_START_ADDRESS+i);
    split_32bits_into_4x8(flash_word, 0, *flash_addr);
    
    RxDummyBuffer = USART_SpiTransfer(usart, flash_word[0]);
    RxDummyBuffer = USART_SpiTransfer(usart, flash_word[1]);
    RxDummyBuffer = USART_SpiTransfer(usart, flash_word[2]);
    RxDummyBuffer = USART_SpiTransfer(usart, flash_word[3]);
  }
  
}

/**************************************************************************//**
 * @brief Send signatures to the TRX
 *****************************************************************************/
void send_signatures(USART_TypeDef *usart)
{

  uint32_t *flash_addr = (uint32_t *)FIRST_SIG_ADDRESS;
  uint8_t flash_word[4];
  
  for (uint16_t i=0; i < NO_OF_SIG_ENTRIES; i=i+1){

    if (i == 0) {
      // Sending write-access address while receiving dummy data
      RxDummyBuffer = USART_SpiTransfer(usart, TRX_WRITE_ADDRESS);
    }

    for (uint16_t j=0; j < SIG_ENTRY_SIZE; j=j+4){
    
      flash_addr = (uint32_t *)(FIRST_SIG_ADDRESS+i*FLASH_PAGE_SIZE+j);
      split_32bits_into_4x8(flash_word, 0, *flash_addr);
    
      RxDummyBuffer = USART_SpiTransfer(usart, flash_word[0]);
      RxDummyBuffer = USART_SpiTransfer(usart, flash_word[1]);
      RxDummyBuffer = USART_SpiTransfer(usart, flash_word[2]);
      RxDummyBuffer = USART_SpiTransfer(usart, flash_word[3]);
    }
  }
  
}

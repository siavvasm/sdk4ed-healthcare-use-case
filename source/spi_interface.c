/**************************************************************************//**
 * @file
 * @brief Definitions related to the main MCU's SPI interface with its co-proc 
 * @version 0.0.1
 ******************************************************************************/

#include "spi_interface.h"
#include "adc.h"
#include "housekeeping.h"

uint8_t TxBuffer[SPI_BUFFER_SIZE] = {0};
uint8_t RxBuffer[SPI_BUFFER_SIZE] = {0};

uint32_t RxBufferIndex = 0;
uint32_t TxBufferIndex = 0;

uint16_t samples_array[NR_SAMPLES];
uint16_t supply_voltage_mV = 0;
bool treatment_on = 1;
uint32_t seizure_time[NR_SAMPLES/2] = {0};
uint32_t seizure_date[NR_SAMPLES/2] = {0};

/**************************************************************************//**
 * @brief Send/receive data to/from the SPI master
 *****************************************************************************/
void spi_transfer_master(uint8_t * tx_buffer, uint8_t * rx_buffer, uint32_t * tx_buffer_index, uint32_t * rx_buffer_index, USART_TypeDef *usart)
{
  while((*tx_buffer_index < SPI_BUFFER_SIZE) || (*rx_buffer_index < SPI_BUFFER_SIZE))
    {
      if ((usart->STATUS & USART_STATUS_TXBL) && (*tx_buffer_index < SPI_BUFFER_SIZE))
	{
	  usart->TXDATA = tx_buffer[*tx_buffer_index];
	  *tx_buffer_index += 1;
	}

      if ((usart->STATUS & USART_STATUS_RXDATAV) && (*rx_buffer_index < SPI_BUFFER_SIZE))
	{
	  rx_buffer[*rx_buffer_index] = usart->RXDATA;
	  *rx_buffer_index += 1;
	}
    }  
  *tx_buffer_index = 0;
  *rx_buffer_index = 0;
}

/**************************************************************************//**
 * @brief Decoding tree for the received CMD, which also impacts the ANS
 *****************************************************************************/
void command_decoding_tree(uint8_t * tx_buffer, uint8_t * rx_buffer)
{

  uint8_t CmdCodeword = 0;

  // Extracting command codeword from the received packet
  CmdCodeword = rx_buffer[PAYLOAD_SIZE];

  if (CmdCodeword == SEND_COEFFICIENTS) {
      
    for (uint16_t i = 0; i < PAYLOAD_SIZE; i=i+2) {
      // Splitting 16-bit coefficients into 8-bit parts in order to send to TRX via SPI
      if (i < PAYLOAD_SIZE/2) {
	tx_buffer[i] = (coeffIM[i/2] & 0xFF);
	tx_buffer[i+1] = (coeffIM[i/2] & 0xFF00) >> 8;
      } else {
	tx_buffer[i] = (coeffRE[(i-(PAYLOAD_SIZE/2))/2] & 0xFF);
	tx_buffer[i+1] = (coeffRE[(i-(PAYLOAD_SIZE/2))/2] & 0xFF00) >> 8;
      }	
    }
    
  }
  else if (CmdCodeword == SEND_THRESHOLDS) {
    
    // Sending threshold values for the detection algorithm	
    split_32bits_into_4x8(tx_buffer, 0, threshold_l);
    split_32bits_into_4x8(tx_buffer, 4, threshold_u);
  }
  else if (CmdCodeword == UPDATE_COEFFICIENTS) {

    for (uint16_t i = 0; i < PAYLOAD_SIZE; i=i+2) {
      // Updating filter coefficients
      // Combining consecutive 8-bit values received via SPI to form a 16-bit coefficient
      if (i < PAYLOAD_SIZE/2) {
	coeffIM[i/2] = (rx_buffer[i+1] << 8) | (rx_buffer[i] & 0xff);
      }
      else {
	coeffRE[(i-(PAYLOAD_SIZE/2))/2] = (rx_buffer[i+1] << 8) | (rx_buffer[i] & 0xff);
      }
    }    
    
  }
  else if (CmdCodeword == UPDATE_THRESHOLDS) {
    // Updating threshold values for the detection algorithm
    // Combining consecutive 8-bit values received via SPI to form 32-bit thresholds
    threshold_l = (rx_buffer[3] << 24) | (rx_buffer[2] << 16) | (rx_buffer[1] << 8) | (rx_buffer[0] & 0xff);
    threshold_u = (rx_buffer[7] << 24) | (rx_buffer[6] << 16) | (rx_buffer[5] << 8) | (rx_buffer[4] & 0xff);
    
  }
  else if (CmdCodeword == SEND_STATUS) {

    // Sending status log, in this case the record of seizure time and date
    for (uint16_t i = 0; i < PAYLOAD_SIZE; i=i+8) {	  
      split_32bits_into_4x8(tx_buffer, i, seizure_time[i/8]);
      split_32bits_into_4x8(tx_buffer, i+4, seizure_date[i/8]);
    }
  }
  else if (CmdCodeword == SEND_SENSOR_VALUES) {
      
    for (uint16_t i = 0; i < (PAYLOAD_SIZE/2); i=i+2) {

      // Splitting 16-bit sensor values into 8-bit parts in order to send to TRX via SPI
      tx_buffer[i] = (samples_array[i/2] & 0xFF);
      tx_buffer[i+1] = (samples_array[i/2] & 0xFF00) >> 8;
    }
  }
  else if (CmdCodeword == SEND_BATTERY_STATUS) {

    // Measuring battery level
    InitADCforSupplyMeasurement();
    supply_voltage_mV = readSupplyVoltage();

    // Sending battery status
    tx_buffer[0] = (supply_voltage_mV & 0xFF);
    tx_buffer[1] = (supply_voltage_mV & 0xFF00) >> 8;

    // Re-init the ADC for regular operation
    initADC();
  }
  else if (CmdCodeword == TREATMENT_OFF) {

    treatment_on = 0;
  }
  else if (CmdCodeword == TREATMENT_ON) {

    treatment_on = 1;
  }
  else if (CmdCodeword == SET_DATE) {

    pack_4x8bits_into_32(rx_buffer, 0, &current_time);
    pack_4x8bits_into_32(rx_buffer, 4, &current_date);
	
    rtccSetup(current_time, current_date);
  }
  else if (CmdCodeword == SEND_DATE) {

    split_32bits_into_4x8(tx_buffer, 0, RTCC->TIME);
    split_32bits_into_4x8(tx_buffer, 4, RTCC->DATE);
  }

}

/**************************************************************************//**
 * @brief Receive firmware from the SPI master (SISC)
 *****************************************************************************/
void receive_firmware(uint32_t * tx_buffer_index, uint32_t * rx_buffer_index, USART_TypeDef *usart)
{
  uint8_t TxDummyBuffer = 0;
  uint8_t rx_buffer[4]; // 32-bit flash location
  uint32_t flash_word[] = {0};
  uint32_t *addr = (uint32_t *)FW_START_ADDRESS;

  MSC_Init();

  for (uint32_t i = 0; i < FW_PAYLOAD_SIZE/FLASH_PAGE_SIZE; i=i+1) {
    addr = (uint32_t *)(FW_START_ADDRESS+i*FLASH_PAGE_SIZE);
    // Erasing the page first (not actually doing it since min. 10K erase cycles endurance)
    MSC_ErasePage(addr);
  }
  
  while((*tx_buffer_index < FW_PAYLOAD_SIZE) || (*rx_buffer_index < FW_PAYLOAD_SIZE))
    {
      if ((usart->STATUS & USART_STATUS_TXBL) && (*tx_buffer_index < FW_PAYLOAD_SIZE))
	{
	  usart->TXDATA = TxDummyBuffer;
	  *tx_buffer_index += 1;
	}

      if ((usart->STATUS & USART_STATUS_RXDATAV) && (*rx_buffer_index < FW_PAYLOAD_SIZE))
	{
	  rx_buffer[(*rx_buffer_index)%4] = usart->RXDATA;
	  *rx_buffer_index += 1;

	  if ((*rx_buffer_index%4) == 0) {
	    pack_4x8bits_into_32(rx_buffer, 0, flash_word);

	    addr = (uint32_t *)(FW_START_ADDRESS+*rx_buffer_index-4);
	    // write this value to flash (won't actually write it)
	    MSC_WriteWord(addr, flash_word, sizeof(flash_word));	
	  }
	}
      
    }  
  *tx_buffer_index = 0;
  *rx_buffer_index = 0;

  MSC_Deinit();

  // Reboot and install new firmware using Application Bootloader
  bootloader_rebootAndInstall();
  
}

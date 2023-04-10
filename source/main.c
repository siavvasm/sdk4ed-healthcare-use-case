/**************************************************************************//**
 * @file
 * @brief Main IMD application - Seizure detection and suppression
 * @version 0.0.1
 *****************************************************************************
 * This code performs the following tasks:
 *  - Get the sensor (ECoG/EEG) values via ADC periodically (using cryotimer
 *    interrupts)
 *  - Perform an FIR filter operation on the input samples. This filter
 *    accurately approximates a continuous complex Morlet wavelet
 *  - Based on the filter output decide whether the seizure is detected or not
 *  - Apply optogenetic or electrical stimulus via GPIO in order to suppress 
 *    the seizure
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_cryotimer.h"
#include "em_gpio.h"
#include "bsp.h"
#include "spi_interface.h"
#include "fir.h"
#include "wdog.h"
#include "adc.h"
#include "rtcc.h"

volatile uint32_t sample;
volatile uint32_t millivolts[NUM_INPUTS];
uint32_t id;
uint32_t battery_check_timer;

// Note: change this to one of the defined periods in em_cryotimer.h
// Wakeup events occur every 8 prescaled clock cycles (resulting in 125 Hz ADC sampling rate)
#define CRYOTIMER_PERIOD    cryotimerPeriod_8

// Note: change this to one of the defined prescalers in em_cryotimer.h
// The clock is divided by one
#define CRYOTIMER_PRESCALE  cryotimerPresc_1

/**************************************************************************//**
 * @brief Cryotimer interrupt service routine
 *****************************************************************************/
void CRYOTIMER_IRQHandler(void)
{
  // Acknowledge the interrupt
  uint32_t flags = CRYOTIMER_IntGet();
  CRYOTIMER_IntClear(flags);

  // Start next ADC conversion
  ADC_Start(ADC0, adcStartScan);

  // Wait for conversion to be complete
  while(!(ADC0->STATUS & _ADC_STATUS_SCANDV_MASK));

  // Get ADC results
  for(uint8_t i = 0; i < NUM_INPUTS; i++)
    {
      // Get data from ADC scan
      sample = ADC_DataIdScanGet(ADC0, &id);

      // Convert data to mV and store to array
      millivolts[i] = sample * 2500 / 4096;
    }

  if (battery_check_timer < ALERT_TONE_DURATION) {
  
    if (battery_check_timer == 0) {
      // Measuring the battery level once every day
      InitADCforSupplyMeasurement();
      supply_voltage_mV = readSupplyVoltage();

      if (supply_voltage_mV < MIN_BATTERY_VOLTAGE) {
	// Starting patient alert tone
	GPIO_PinOutSet(gpioPortC, 4);
      }

      // Re-init the ADC for regular operation
      initADC();
    } else if (battery_check_timer == (ALERT_TONE_DURATION-1)) {
      // Disabling patient alert tone
      GPIO_PinOutClear(gpioPortC, 4);
    }

  }

  battery_check_timer = (battery_check_timer + 1) % BATTERY_CHECK_TIMEOUT;
}

/**************************************************************************//**
 * @brief GPIO Odd IRQ for odd-numbered pins
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  // Disable IRQ to avoid retriggering
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  
  // Check whether BTN1 is pressed. This emulates GPIO interrupt from the security co-processor for normal config commands
  if((GPIO->IF & 0x00000200) == 0x0200) {
    // receive/send data from/to the security co-processor
    spi_transfer_master(TxBuffer, RxBuffer, &TxBufferIndex, &RxBufferIndex, USART0);

    // Executing the decoding tree to understand the command and formulate the response
    command_decoding_tree(TxBuffer, RxBuffer);
    
    // Send GPIO interrupt to the security co-processor
    GPIO_PinOutSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
    GPIO_PinOutClear(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
    
    // receive/send data from/to the security co-processor
    spi_transfer_master(TxBuffer, RxBuffer, &TxBufferIndex, &RxBufferIndex, USART0);
    
  }
  
  // Check whether BTN0 is pressed. This emulates GPIO interrupt from the security co-processor for firmware update
  if((GPIO->IF & 0x00000020) == 0x0020) {

    // Receive firmware from the security co-processor
    receive_firmware(&TxBufferIndex, &RxBufferIndex, USART0);
  }

  // Acknowledge interrupt
  GPIO_IntClear(0xAAAA);
  // Enabling IRQ again
  NVIC_EnableIRQ(GPIO_ODD_IRQn);  
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
 * @brief Initialize LED and Push-button pins
 *****************************************************************************/
void initGpio(void)
{
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PB1 (push-button #1) (PC9) as input with glitch filter enabled. This emulates GPIO interrupt from the security co-processor for regular config commands.
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInputPullFilter, 1);

  // Configure PB0 (push-button #0) (PD5) as input with glitch filter enabled. This emulates GPIO interrupt from the security co-processor for firmware update.
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPullFilter, 1);

  // Configure LED0 pin (PD2) as output. This LED denotes the GPIO interrupt to the security co-processor.
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);
  // Configure LED1 pin (PC2) as output. This LED denotes the optogenetic/electrical stimulation for seizure suppression.
  GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull, 0);

  // Configure PC4 as output. This is used for sending the patient alert tone.
  GPIO_PinModeSet(gpioPortC, 4, gpioModePushPull, 0);

  // Enable IRQ for odd numbered GPIO pins
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  // Enable falling-edge interrupts for PB pins
  GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, BSP_GPIO_PB1_PIN, 0, 1, true);
  GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, BSP_GPIO_PB0_PIN, 0, 1, true);
  
}

/**************************************************************************//**
 * @brief Initialize USART0 as SPI slave to communicate with the co-processor
 *****************************************************************************/
void initUSART0 (void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART0, true);

  // Configure GPIO mode
  GPIO_PinModeSet(gpioPortA, 12, gpioModeInput, 1);    // US0_CLK is input
  GPIO_PinModeSet(gpioPortC, 8, gpioModeInput, 1);     // US0_CS is input
  GPIO_PinModeSet(gpioPortC, 11, gpioModeInput, 1);    // US0_TX (MOSI) is input
  GPIO_PinModeSet(gpioPortC, 10, gpioModePushPull, 1); // US0_RX (MISO) is push pull

  // Start with default config, then modify as necessary
  USART_InitSync_TypeDef config = USART_INITSYNC_DEFAULT;
  config.master    = false;
  config.clockMode = usartClockMode0; // clock idle low, sample on rising/first edge
  config.msbf      = true;            // send MSB first
  config.enable    = usartDisable;    // making sure to keep USART disabled until we've set everything up
  USART_InitSync(USART0, &config);
  USART0->CTRL |= USART_CTRL_SSSEARLY;

  // Set USART pin locations
  USART0->ROUTELOC0 = (USART_ROUTELOC0_CLKLOC_LOC5) | // US0_CLK       on location 5 = PA12 per datasheet section 6.4 = EXP Header pin 8
                      (USART_ROUTELOC0_CSLOC_LOC2)  | // US0_CS        on location 2 = PC8 per datasheet section 6.4 = EXP Header pin 10
                      (USART_ROUTELOC0_TXLOC_LOC2)  | // US0_TX (MOSI) on location 2 = PC11 per datasheet section 6.4 = EXP Header pin 4
                      (USART_ROUTELOC0_RXLOC_LOC2);   // US0_RX (MISO) on location 2 = PC10 per datasheet section 6.4 = EXP Header pin 6

  // Enable USART pins
  USART0->ROUTEPEN = USART_ROUTEPEN_CLKPEN | USART_ROUTEPEN_CSPEN | USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;

  // Enable USART0
  USART_Enable(USART0, usartEnable);
}

/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{
  uint32_t sample_position = 0;
  uint16_t seizure_count = 0;
  uint8_t channel = 0;
  uint8_t effective_seizure = 0;
  seizure_detection det[NUM_INPUTS];
  
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;

  /* Chip errata */  
  CHIP_Init();

  EMU_DCDCInit(&dcdcInit);

  // Initializations
  initGpio();
  initWDOG();
  initADC();
  initUSART0();
  initCryotimer();
  rtccSetup(0, 0);

  // Initialize channels
  for (channel = 0; channel < NUM_INPUTS; channel++) {
    det[channel].seizure =  SEIZ_OFF;
    det[channel].sum = 0;
  }
  
  /* Starting position before FIR operation */
  sample_position = NR_SAMPLES-1;

  seizure_count = 0;

  while (1) {

    // Wait for sample
    EMU_EnterEM3(false);

    // Feed the watchdog
    WDOG_Feed();
 
    // Getting captured sample from the ADC 
    for (channel = 0; channel < NUM_INPUTS; channel++) {
      det[channel].ain[sample_position] = ((int16_t)millivolts[channel] - 2048) >> 4;
    }
    samples_array[sample_position] = millivolts[0];

    /* Filtering */
    for (channel = 0; channel < NUM_INPUTS; channel++) {
      fir_simple_wav(&(det[channel].sum), det[channel].ain, coeffRE, coeffIM, sample_position);
    }

    effective_seizure = SEIZ_ON;
    /* Determining seizure occurrence based on filter output */
    for (channel = 0; channel < NUM_INPUTS; channel++) {
      det[channel].seizure = detection(det[channel].sum, threshold_u, threshold_l, det[channel].seizure);
      effective_seizure = effective_seizure & det[channel].seizure;
    }

    // Performing optogenetic/electrical stimulation
    if (effective_seizure && treatment_on) {
      // Set LED1
      GPIO_PinOutSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
      seizure_time[seizure_count]=RTCC->TIME;
      seizure_date[seizure_count]=RTCC->DATE;
      seizure_count = (seizure_count + 1) % (NR_SAMPLES/2);
    } else {
      // Clear LED1
      GPIO_PinOutClear(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
    }

    sample_position = (sample_position + 1) % NR_SAMPLES;
  }
}

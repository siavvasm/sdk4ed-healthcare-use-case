/**************************************************************************//**
 * @file
 * @brief Definitions related to the ADC operation
 * @version 0.0.1
 ******************************************************************************/

#include "adc.h"

/**************************************************************************//**
 * @brief ADC initialization
 *****************************************************************************/
void initADC (void)
{
  // Enable clocks required
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitScan_TypeDef initScan = ADC_INITSCAN_DEFAULT;

  // Modify init structs
  init.prescale   = ADC_PrescaleCalc(adcFreq, 0);
  init.timebase = ADC_TimebaseCalc(0);

  initScan.diff          = false;       // single ended
  initScan.reference     = adcRef2V5;   // internal 2.5V reference
  initScan.resolution    = adcRes12Bit; // 12-bit resolution
  initScan.acqTime       = adcAcqTime4; // set acquisition time to meet minimum requirements
  initScan.fifoOverwrite = true;        // FIFO overflow overwrites old data

  // Select ADC input. See README for corresponding EXP header pin.
  // Add VDD to scan for demonstration purposes
  ADC_ScanSingleEndedInputAdd(&initScan, adcScanInputGroup0, adcPosSelAPORT0XCH2); // PD2 (Same as LED0)
  ADC_ScanSingleEndedInputAdd(&initScan, adcScanInputGroup1, adcPosSelAPORT2XCH21); // PB5

  // Set scan data valid level (DVL) to 2
  ADC0->SCANCTRLX = (ADC0->SCANCTRLX & ~_ADC_SCANCTRLX_DVL_MASK) | (((NUM_INPUTS - 1) << _ADC_SCANCTRLX_DVL_SHIFT) & _ADC_SCANCTRLX_DVL_MASK);

  // Clear ADC scan FIFO
  ADC0->SCANFIFOCLEAR = ADC_SCANFIFOCLEAR_SCANFIFOCLEAR;

  // Initialize ADC and Scans
  ADC_Init(ADC0, &init);
  ADC_InitScan(ADC0, &initScan);
}

/**************************************************************************//**
 * @brief ADC initialization for battery-level measurement
 *****************************************************************************/
void InitADCforSupplyMeasurement()
{

  ADC_Init_TypeDef ADC_Defaults = ADC_INIT_DEFAULT;

  ADC_InitSingle_TypeDef init = ADC_INITSINGLE_DEFAULT ;
  init.negSel = adcNegSelVSS;
  init.posSel = adcPosSelIOVDD;
  init.reference = adcRefVDD ;	/* VREF = VDD */

  /*start with defaults */
  ADC_Init(ADC0,&ADC_Defaults);

  ADC_InitSingle(ADC0, &init);

  ADC0->SINGLECTRLX = VINATT(6) | VREFATT(0) ;	/* cf p22 an0021 VFS = ( VREF*(VREFATT+6) )/VINATT */
  
}

/**************************************************************************//**
 * @brief Check to see if conversion is complete
 *****************************************************************************/
bool ADC_SingleDataValid()
{
  return ADC0->STATUS & ADC_STATUS_SINGLEDV;
}

/**************************************************************************//**
 * @brief Read supply voltage
 *****************************************************************************/
float readSupplyVoltage()
{
  uint32_t raw = 0;
  uint16_t supplyVoltagemV = 0;
  
  // Start next ADC conversion
  ADC_Start(ADC0, adcStartSingle);
  
  // Wait for conversion to be complete
  while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

  if(ADC_SingleDataValid())
    {
      raw = ((5*ADC_DataSingleGet(ADC0))/4096)*1000;
      supplyVoltagemV = raw*microvoltsPerStep/1000UL;
      return(supplyVoltagemV);
    }
  return(-1);
}

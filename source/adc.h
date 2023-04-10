/**************************************************************************//**
 * @file
 * @brief Declarations related to the ADC operation
 * @version 0.0.1
 ******************************************************************************/

#ifndef _ADC_H_
#define _ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_adc.h"

// Init to max ADC clock for Series 1
#define adcFreq         16000000
#define NUM_INPUTS  2

#define VINATT(ATT_FACTOR) ATT_FACTOR << _ADC_SINGLECTRLX_VINATT_SHIFT
#define VREFATT(ATT_FACTOR)ATT_FACTOR << _ADC_SINGLECTRLX_VREFATT_SHIFT

#define microvoltsPerStep 806// 1221

#define MIN_BATTERY_VOLTAGE 2100 // mV
#define BATTERY_CHECK_TIMEOUT 10800000 // in cryotimer cycles: (86400 seconds) x (125 Hz)
#define ALERT_TONE_DURATION 1250 // in cryotimer cycles: (10 seconds) x (125 Hz)

void initADC();
void InitADCforSupplyMeasurement();
bool ADC_SingleDataValid();
float readSupplyVoltage();

#endif //_ADC_H_

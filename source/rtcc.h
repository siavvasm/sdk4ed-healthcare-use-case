/**************************************************************************//**
 * @file
 * @brief Declarations related to the RTCC operation
 * @version 0.0.1
 ******************************************************************************/

#ifndef _RTCC_H_
#define _RTCC_H_

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_rtcc.h"
#include "spi_interface.h"

extern uint32_t current_time;
extern uint32_t current_date;

void rtccSetup(int start_time, int start_date);

#endif //_RTCC_H_

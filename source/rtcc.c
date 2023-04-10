/**************************************************************************//**
 * @file
 * @brief Definitions related to the RTCC operation
 * @version 0.0.1
 ******************************************************************************/

#include "rtcc.h"

uint32_t current_time = 0;
uint32_t current_date = 0;

uint32_t rtccFlag;

/**************************************************************************//**
 * @brief RTCC initialization
 *****************************************************************************/
void rtccSetup(int start_time, int start_date)
{
  // Configure the RTCC settings
  RTCC_Init_TypeDef rtcc = RTCC_INIT_DEFAULT;
  rtcc.enable   = false;
  rtcc.presc = rtccCntPresc_32768;
  rtcc.cntMode = rtccCntModeCalendar;

  // Turn on the clock for the RTCC
  // LFRCO will not work while in EM3
  CMU_ClockEnable(cmuClock_HFLE, true);
  CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_RTCC, true);

  // Initialise RTCC with pre-defined settings
  RTCC_Init(&rtcc);

  // Set current date and time
  RTCC_DateSet(start_date);
  RTCC_TimeSet(start_time);

  // Start counter after all initialisations are complete
  RTCC_Enable(true);
}


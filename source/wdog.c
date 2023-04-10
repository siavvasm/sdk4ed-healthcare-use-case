/**************************************************************************//**
 * @file
 * @brief Definitions related to the Watch-dog timer 
 * @version 0.0.1
 ******************************************************************************/

#include "wdog.h"

/**************************************************************************//**
 * @brief Watchdog initialization
 *****************************************************************************/
void initWDOG(void)
{
  // Enabling clock to the interface of the low energy modules (including the Watchdog)
  CMU_ClockEnable(cmuClock_HFLE, true);

  // Watchdog Initialize settings 
  WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
  wdogInit.debugRun = true;
  wdogInit.em3Run = true;
  wdogInit.clkSel = wdogClkSelULFRCO;
  wdogInit.perSel = wdogPeriod_2k; // 2049 clock cycles of a 1kHz clock  ~2 seconds period

  // Initializing watchdog with chosen settings 
  WDOG_Init(&wdogInit);
}

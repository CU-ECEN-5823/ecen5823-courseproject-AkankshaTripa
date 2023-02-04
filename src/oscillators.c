#include "src/oscillators.h"
#include "src/timers.h"

void Oscillator_Init(void)
{

  //Initialising oscillator includes below 3 steps:

  //      1) CMU_OscillatorEnable()
  //       2) CMU_ClockSelectSet()
  //        3) CMU_ClockDivSet()
  //        4) CMU_ClockEnable()                        */

if((LOWEST_ENERGY_MODE==EM0)||(LOWEST_ENERGY_MODE==EM1)||(LOWEST_ENERGY_MODE==EM2))
  {

    //In higher power modes (EM0/EM1/EM2) the LFXO (32.768 KHz) should be used

    //Enable setting wait to true, this routine will not return to the program until the clock source has been stabilized
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);                  //freq=32.768 KHz

    //Setting clock parameter to clock branch LFA and clock source to LFX0
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

    //Setting prescaler value to 4 for clock branch
    CMU_ClockDivSet(cmuClock_LETIMER0, PRESCALER_VALUE);            //prescaler value = 4

    //Enabling the peripheral LETIMER0 to be clocked
    CMU_ClockEnable(cmuClock_LETIMER0, true);
  }
else
  {
    //In Lower power mode (EM3) the ULFRCO (1 KHz) should be used
    CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);              //freq=1 KHz

    //Setting clock parameter to clock branch LFA and clock source to ULFRCO
     CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

     //Setting prescaler value to 4 for clock branch
     CMU_ClockDivSet(cmuClock_LETIMER0, PRESCALER_VALUE);          //prescaler value= 4

     //Enabling the peripheral LETIMER0 to be clocked
     CMU_ClockEnable(cmuClock_LETIMER0, true);
  }


}

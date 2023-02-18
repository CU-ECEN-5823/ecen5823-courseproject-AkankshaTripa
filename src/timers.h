
#ifndef __myTimers
#define __myTimers

#include "app.h"
#include "em_letimer.h"

// define anything a caller needs
#define LETIMER_ON_TIME_MS 175
//#define LETIMER_PERIOD_MS 2250
#define LETIMER_PERIOD_MS 3000                  //to give interrupt of 3 sec
#define PRESCALER_VALUE 4

#if((LOWEST_ENERGY_MODE==0)||(LOWEST_ENERGY_MODE==1)||(LOWEST_ENERGY_MODE==2))
   #define ACTUAL_CLK_FREQ 8192                                       //defining frequency for high power mode
#else
   #define ACTUAL_CLK_FREQ 250                                       //defining frequency for low power mode
#endif

#define LOADCOUNTER (LETIMER_PERIOD_MS*ACTUAL_CLK_FREQ)/1000          //calculating  counter load value

#define INTERRUPTCOUNTER (LETIMER_ON_TIME_MS*ACTUAL_CLK_FREQ)/1000    //calculating interrupt counter value

// function prototypes
void initLETIMER0 ();

void timerWaitUs(uint32_t waitUs);                //timerwait function for polling

void timerWaitUs_irq(uint32_t us_wait);           //timerwait function for interrupt

#endif

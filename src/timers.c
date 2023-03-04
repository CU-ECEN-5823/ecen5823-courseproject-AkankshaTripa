/*
 * References: All the initialization has been taken form lecture slides
 *
 * timerWaitUs(): Guidance by Varun Mehta
 * letimerMilliseconds(): Guidance as in lecture slide and slack channel
 * */

#include <em_letimer.h>
#include "timers.h"
#include "oscillators.h"

//#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define ACTUAL_FREQUENCY (CMU_ClockFreqGet(cmuClock_LETIMER0))

uint32_t timestamp=0;

void initLETIMER0 ()
{

 uint32_t temp;



 // this data structure is passed to LETIMER_Init (), used to set LETIMER0_CTRL reg bits and other value

 const LETIMER_Init_TypeDef letimerInitData = {
 false, // enable; don't enable when init completes, we'll enable last
 true, // debugRun; useful to have the timer running when single-stepping in the debugger
 true, // comp0Top; load COMP0 into CNT on underflow
 false, // bufTop; don't load COMP1 into COMP0 when REP0==0
 0, // out0Pol; 0 default output pin value
 0, // out1Pol; 0 default output pin value
 letimerUFOANone, // ufoa0; no underflow output action
 letimerUFOANone, // ufoa1; no underflow output action
 letimerRepeatFree, // repMode; free running mode i.e. load & go forever
 LOADCOUNTER // COMP0(top) Value
 };


 // initialize the timer
 LETIMER_Init (LETIMER0, &letimerInitData);

 // calculate and load COMP0 (top)
 // calculate and load COMP1

 // Clear all IRQ flags in the LETIMER0 IF status register
 LETIMER_IntClear (LETIMER0, 0xFFFFFFFF); // punch them all down

 // Set UF and COMP1 in LETIMER0_IEN, so that the timer will generate IRQs to the NVIC.
 temp = LETIMER_IEN_UF ;
 LETIMER_IntEnable (LETIMER0, temp); //  the ISR routine LETIMER0_IRQHandler() should be defined

 //Initializing COMP1 INTERRUPT
 LETIMER_CompareSet(LETIMER0, 1, INTERRUPTCOUNTER);

 // Enable the timer to starting counting down, set LETIMER0_CMD[START] bit, see LETIMER0_STATUS[RUNNING] bit
 LETIMER_Enable (LETIMER0, true);


} // initLETIMER0 ()


void timerWaitUs(uint32_t us_wait)
{

  uint32_t delay,ticks;
  if((us_wait/1000)>LETIMER_PERIOD_MS)
    {
      us_wait=(LETIMER_PERIOD_MS*1000);
     // LOG_ERROR("Wait time exceeded the range, now wait time in %lu usecs : ", us_wait);
    }

  delay=(((us_wait/1000) * ACTUAL_FREQUENCY)/1000);
  ticks= LETIMER_CounterGet(LETIMER0);

  while(delay)
    {
      if(ticks !=LETIMER_CounterGet(LETIMER0))
        {
          delay--;
          ticks=LETIMER_CounterGet(LETIMER0);
        }
    }

}//timerWaitUs()


// function to return a count of milliseconds since power up
uint32_t letimerMilliseconds()
{

   timestamp += LETIMER_PERIOD_MS;
    return timestamp;

}//letimerMilliseconds()


/*timerwait function with interrupt */
void timerWaitUs_irq(uint32_t us_wait)
{
    uint32_t ticks, temp, wait_time;

    uint32_t ms_wait =us_wait/1000;

  if(ms_wait > LETIMER_PERIOD_MS)
    {
    //  LOG_ERROR("Wait time exceeded the range\n\r");
      ms_wait = LETIMER_PERIOD_MS;
    }

       wait_time=(ms_wait*ACTUAL_CLK_FREQ)/1000;

       ticks=LETIMER_CounterGet (LETIMER0);

      if(wait_time<ticks)
      {
          temp = (ticks - wait_time);
      }
      else
      {
          temp = (LOADCOUNTER) - (wait_time - ticks);
      }
       LETIMER_CompareSet(LETIMER0, 1, temp);

       LETIMER_IntClear (LETIMER0, LETIMER_IFC_COMP1);
       LETIMER_IntEnable (LETIMER0, LETIMER_IEN_COMP1); // Make sure you have defined the ISR routine LETIMER0_IRQHandler()

       LETIMER0->IEN |= LETIMER_IF_COMP1;


}//timerWaitUs_irq()

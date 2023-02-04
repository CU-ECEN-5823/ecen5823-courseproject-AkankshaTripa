/*
 * References: All the initialization has been taken form lecture slides
 * */

#include <em_letimer.h>
#include "timers.h"

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
 temp = LETIMER_IEN_UF | LETIMER_IEN_COMP1;
 LETIMER_IntEnable (LETIMER0, temp); //  the ISR routine LETIMER0_IRQHandler() should be defined

 //Initializing COMP1 INTERRUPT
 LETIMER_CompareSet(LETIMER0, 1, INTERRUPTCOUNTER);

 // Enable the timer to starting counting down, set LETIMER0_CMD[START] bit, see LETIMER0_STATUS[RUNNING] bit
 LETIMER_Enable (LETIMER0, true);

 // Test code:
 // read it a few times to make sure it's running within the range of values we expect
 //temp = LETIMER_CounterGet (LETIMER0);
 //temp = LETIMER_CounterGet (LETIMER0);
 //temp = LETIMER_CounterGet (LETIMER0);

} // initLETIMER0 ()





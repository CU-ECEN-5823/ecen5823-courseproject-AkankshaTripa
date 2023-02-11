#include "em_core.h"
#include "scheduler.h"

// interrupt service routine for a peripheral
// CPU+NVIC clear the IRQ pending bit in the NVIC
// when this routine is fetched from memory.


 uint32_t event;
// scheduler routine to set a scheduler event
void schedulerSetEventTemperatureMeasurement()
{
 //STEPS:
 // enter critical section
 // set the event in your data structure, this has to be a read-modify-write
 // exit critical section


  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL(); // NVIC IRQs are disabled

  event |= temperature_measure_event;              //RMW for event

  CORE_EXIT_CRITICAL(); // NVIC IRQs are re-enabled



} // schedulerSetEventXXX()


// scheduler routine to return 1 event to main()code and clear that event
uint32_t getNextEvent()
{
  uint32_t theEvent;
  // select 1 event to return to main() code, apply priorities etc.

  theEvent = wait_event; // default event, does nothing

 // STEPS:
  // enter critical section
  // clear the event in your data structure, this has to be a read-modify-write
  // exit critical section


  CORE_DECLARE_IRQ_STATE;

  if(event & temperature_measure_event)
    {
      theEvent=temperature_measure_event;                             //event return to main function
      CORE_ENTER_CRITICAL();                                           // NVIC IRQs are disabled
      event=event & (temperature_measure_event^0xFFFFFFFF);           //clear event
      CORE_EXIT_CRITICAL();                                           // NVIC IRQs are re-enabled
    }


  return (theEvent);
 } // getNextEvent()

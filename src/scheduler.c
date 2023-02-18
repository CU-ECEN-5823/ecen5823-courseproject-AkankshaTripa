/*
 * Code Credits: Lecture Slides
 * */
//#include <em_i2c.h>
#include "em_core.h"
#include "scheduler.h"
#include "src/gpio.h"
#include "src/i2c.h"
#include <sl_power_manager.h>
#include "em_device.h"
#include "timers.h"
//#define INCLUDE_LOG_DEBUG 1
#include "log.h"

// interrupt service routine for a peripheral
// CPU+NVIC clear the IRQ pending bit in the NVIC
// when this routine is fetched from memory.


uint32_t event;

// scheduler routine to set a scheduler event
void schedulerSetEventUF()
{
 //STEPS:
 // enter critical section
 // set the event in your data structure, this has to be a read-modify-write
 // exit critical section

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();                          // NVIC IRQs are disabled

  event |= eventuf;                               //RMW for event

  CORE_EXIT_CRITICAL();                           // NVIC IRQs are re-enabled
} // schedulerSetEventUF()


void schedulerSetEventCOMP1()
{
 //STEPS:
 // enter critical section
 // set the event in your data structure, this has to be a read-modify-write
 // exit critical section

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();                          // NVIC IRQs are disabled

  event |= eventcomp1;                               //RMW for event

  CORE_EXIT_CRITICAL();                           // NVIC IRQs are re-enabled
} // schedulerSetEventCOMP1()


void schedulerSetEventI2CDone()
{
 //STEPS:
 // enter critical section
 // set the event in your data structure, this has to be a read-modify-write
 // exit critical section

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();                          // NVIC IRQs are disabled

  event |= i2ccomplete;                               //RMW for event

  CORE_EXIT_CRITICAL();                           // NVIC IRQs are re-enabled
} // schedulerSetEventI2CDone()



// scheduler routine to return 1 event to main()code and clear that event
uint32_t getNextEvent()
{
  uint32_t theEvent;
  // select 1 event to return to main() code, apply priorities etc.

  //theEvent = waitevent;                                    // default event, does nothing

 // STEPS:
  // enter critical section
  // clear the event in your data structure, this has to be a read-modify-write
  // exit critical section

  CORE_DECLARE_IRQ_STATE;

  if((event & eventuf) == eventuf)
    {
      theEvent=eventuf;                             //event return to main function
      CORE_ENTER_CRITICAL();                                           // NVIC IRQs are disabled
      event=event & (eventuf^0xFFFFFFFF);           //clear event
      CORE_EXIT_CRITICAL();                                           // NVIC IRQs are re-enabled
    }

  if((event & eventcomp1) == eventcomp1)
      {
        theEvent=eventcomp1;                             //event return to main function
        CORE_ENTER_CRITICAL();                                           // NVIC IRQs are disabled
        event=event & (eventcomp1^0xFFFFFFFF);           //clear event
        CORE_EXIT_CRITICAL();                                           // NVIC IRQs are re-enabled
      }

  if((event & i2ccomplete) == i2ccomplete)
      {
        theEvent=i2ccomplete;                             //event return to main function
        CORE_ENTER_CRITICAL();                                           // NVIC IRQs are disabled
        event=event & (i2ccomplete^0xFFFFFFFF);           //clear event
        CORE_EXIT_CRITICAL();                                           // NVIC IRQs are re-enabled
      }

  return (theEvent);
 } // getNextEvent()


void state_machine(uint32_t event)
{

   State_t currentState;
   static State_t nextState = stateIdle;
   currentState = nextState;

  event_si7021 event_new=event;

  switch(currentState)
  {
    case stateIdle:
      nextState = stateIdle;
      if(event_new==eventuf)
        {
          //LOG_INFO("stateIdle\n\r");
          nextState=statetimerwait80;
          gpioSi7021enable();                           //enable temp sensor
          timerWaitUs_irq(80000);                       //wait for 80ms to powerup si7021
        }
      break;

    case statetimerwait80:
      nextState=statetimerwait80;
      if(event_new==eventcomp1)
        {
         // LOG_INFO("statetimerwait80\n\r");
          i2c_init();                                                     //intilaise i2c transfer
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);      //add power requirements for EM1 mode
          i2c_write();                                                    //perform i2c write
          nextState=statei2cwrite;

        }
      break;

    case statei2cwrite:
      nextState=statei2cwrite;
      if(event_new==i2ccomplete)
        {
         // LOG_INFO("statei2cwrite\n\r");
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);       //remove power for EM1
          timerWaitUs_irq(10800);                                             //wait for 10.8 ms for calculations
          nextState=statetimerwait108;
        }
      break;

    case statetimerwait108:
        nextState=statetimerwait108;
        if(event_new==eventcomp1)
        {
          // LOG_INFO("statetimerwait108\n\r");
           sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);          //add power req for read operation
           i2c_read();                                                         //perform read operation
           nextState=statei2cread;
         }
        break;

    case statei2cread:
          nextState=statei2cread;
            if(event_new==i2ccomplete)
             {
              //  LOG_INFO("statei2cread\n\r");
                temperaturereading();
                sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);       //remove power req
                gpioSi7021disable();                                                //disabling Si7021
                i2cStop();                                                          //stop i2c transfer
                nextState=stateIdle;
              }

      break;
    default:
          {
          //  LOG_INFO("Si7021, temperature sensor not working\n\r");
          }
       }

  }





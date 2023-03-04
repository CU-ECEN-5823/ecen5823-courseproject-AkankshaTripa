#ifndef __SCHEDULER_H
#define __SCHEDULER_H


#include "sl_bt_api.h"

// define anything a caller needs
// function prototypes
void schedulerSetEventI2CDone();
void schedulerSetEventCOMP1();
void schedulerSetEventUF();

typedef enum uint32_t
{
  eventuf=1,            //event for temperature measurement
  eventcomp1=2,
  i2ccomplete=4,

}event_si7021;

typedef enum
{
  stateIdle,
  statetimerwait80,
  statei2cwrite,
  statetimerwait108,
  statei2cread

}State_t;

uint32_t getNextEvent();                            //function to getNextEvent

//void schedulerSetEventTemperatureMeasurement();     //function to set event temperature in A3

void state_machine(sl_bt_msg_t *evt);

#endif  //__SCHEDULER_H

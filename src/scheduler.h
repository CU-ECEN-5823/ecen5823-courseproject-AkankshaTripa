#ifndef __SCHEDULER_H
#define __SCHEDULER_H


#include "sl_bt_api.h"

// define anything a caller needs
// function prototypes
void schedulerSetEventI2CDone();
void schedulerSetEventCOMP1();
void schedulerSetEventUF();

void schedulerSetEventOpenedConnection();
void schedulerSetEventGattCompleted();
void schedulerSetEventClosedEvent();

void schedulerSetEventCheckButtonStatusPB0();
void schedulerSetEventCheckButtonStatusPB1();

void schedulerSetEventCheckEventEcho();
//
///*  GLOBAL VARIABLES  */
///*
// * t1             = LETIMER0 value when rising edge echo pulse GPIO Interrupt is received
// * t2             = LETIMER0 value when rising edge echo pulse GPIO Interrupt is received
// * difference     = time difference between two GPIO interrupts
// * duration       = convert LETIMER difference to microseconds
// * distance       = distance in cm
// */

typedef enum uint32_t
{
  eventuf=1,            //event for temperature measurement
  eventcomp1=2,
  i2ccomplete=4,
  checkbuttonPB0=8,
  checkbuttonPB1=16,
  eventecho=32

}event_si7021;

typedef enum
{
  stateIdle,
  statetimerwait80,
  statei2cwrite,
  statetimerwait108,
  statei2cread

}State_t;

typedef enum
{
 ideal,
 open_first,
 open_second,
 discovery_first,
 discovery_second,
 notify_first,
 notify_second,
 confirmation,
 close
}Client_t;

/*typedef enum
{
    Noeevent=0,
    OpenConnectionevent=1,
    GATTCompleteevent=2,
    ConnectionClosedevent=4
}Events;*/

uint32_t getNextEvent();                            //function to getNextEvent

//void schedulerSetEventTemperatureMeasurement();     //function to set event temperature in A3

void state_machine(sl_bt_msg_t *evt);

void discovery_state_machine(sl_bt_msg_t *evt);

#endif  //__SCHEDULER_H

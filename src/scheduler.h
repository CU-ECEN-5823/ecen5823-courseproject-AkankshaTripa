#ifndef __SCHEDULER_H
#define __SCHEDULER_H
// define anything a caller needs
// function prototypes


typedef enum
{
  temperature_measure_event = 1,                   //event for temperature measurement
  wait_event = 2                                   //no event, does nothing

}event_si7021;

uint32_t getNextEvent();                            //function to getNextEvent

void schedulerSetEventTemperatureMeasurement();     //function to set event temperature


#endif  //__SCHEDULER_H

#ifndef __SCHEDULER_H
#define __SCHEDULER_H
// define anything a caller needs
// function prototypes


typedef enum
{
  temperature_measure_event = 1,
  wait_event = 2

}event_si7021;

uint32_t getNextEvent();

void schedulerSetEventTemperatureMeasurement();


#endif  //__SCHEDULER_H

/**************************************************************

*                     Project Name : Home Automation System
                      File Name    : scheduler.h
                      Description  : A  a home automation system that uses HC-SR04 and TEMT6000 sensors
                                     to greatly improve the functionality and convenience of a home.
                      Author       : Akanksha Tripathi & Vaibhavi Thakur
                      Date:        : 05/02/2023
                      Version      : 5.6
                      Course       : IoT Embedded Firmware
                      Target Device: Blue GECKO EFR32
                      IDE          :  Simplicity Studio
 *                    Code Credits : All the initialization has been taken form lecture slides
 *                                    Function handle_ble_event : reference from SOC thermomemter project and SOC client project
 *                                    Sensor Interfacing Guidance by Varun Mehta
 *                                    ADC Configuration Guidance by Professor
 *                                    All the other references are from the previous project
 *
 *
 *
 *
*************************************************************************************************************************************/


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
 UvConnection,
 LightConnection,
 UvService,
 LightService,
 UvCharacteristic,
 LightCharacteristic,
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

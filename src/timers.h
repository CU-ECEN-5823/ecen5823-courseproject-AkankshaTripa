/************************************************************

*                     Project Name: Home Automation System
                      File Name : timers.h
                      Description: A  a home automation system that uses HC-SR04 and TEMT6000 sensors
                                   to greatly improve the functionality and convenience of a home.
                      Author: Akanksha Tripathi & Vaibhavi Thakur
                      Date:   05/02/2023
                      Version: 5.6
                      Course: IoT Embedded Firmware
                      Target Device: Blue GECKO EFR32
                      IDE: Simplicity Studio
 *                    References: All the initialization has been taken form lecture slides
 *                                Function handle_ble_event : reference from SOC thermomemter project and SOC client project
 *                                Sensor Interfacing Guidance by Varun Mehta
 *                                ADC Configuration Guidance by Professor
 *                                All the other references are from the previous project
 *
 *
 *
 *
*************************************************************************************************************************************/

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

uint32_t letimerMilliseconds();

#endif

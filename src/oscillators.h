/************************************************************

*                     Project Name: Home Automation System
                      File Name : oscillators.h
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

#ifndef __myOscillators
#define __myOscillators
// define anything a caller needs
// function prototypes
#include "app.h"
#include "em_cmu.h"

//Function call
void Oscillator_Init(void);


#endif





/**************************************************************

*                     Project Name : Home Automation System
                      File Name    : i2c.h
                      Description  : A home automation system that uses HC-SR04 and TEMT6000 sensors
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
*************************************************************************************************************************************/#ifndef EM_I2C_H
#define EM_I2C_H
// define anything a caller needs
// function prototypes


//initialise i2c
void  i2c_init();

//i2c read operation
void i2c_read();

//i2c write opeartion
void i2c_write();

//deinitialise i2c
void i2cStop();

//void i2cGetTemperature();           //used in A3

//temperature measurement
int temperaturereading();          //used in A4



#endif  //EM_I2C_H

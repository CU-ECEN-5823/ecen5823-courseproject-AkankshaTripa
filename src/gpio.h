/*
   gpio.h
  
    Created on: Dec 12, 2018
        Author: Dan Walkes

    Updated by Dave Sluiter Sept 7, 2020. moved #defines from .c to .h file.
    Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

    Editor: Feb 26, 2022, Dave Sluiter
    Change: Added comment about use of .h files.

 */


// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.


#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_

#define LED0_port  5 // change to correct ports and pins
#define LED0_pin   4
#define LCD_PORT gpioPortD
#define LED1_port  5
#define LED1_pin   5
#define LCD_EXTCOMIN_PIN 13

#define SCL_PIN 10
#define SDA_PIN 11

#define I2C_TEMP_PIN 15

#define PB0_port gpioPortF
#define PB0_pin  6

#define PB1_port gpioPortF
#define PB1_pin  7

#define TRIG_PORT gpioPortD    //pin 11 on board
#define TRIG_PIN 12

#define ECHO_PORT gpioPortF    //pin 13 on board
#define ECHO_PIN 3

#define SIG_PORT gpioPortD
#define SIG_PIN  10

// Function prototypes
void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();

void gpiosdaclear();
void gpiosclclear();
void gpioSi7021enable();
void gpioSi7021disable();

void gpioSetDisplayExtcomin(bool pin);


void button_enable();
void disable_button_irq();


void set_trig();
void clear_trig();

#endif /* SRC_GPIO_H_ */

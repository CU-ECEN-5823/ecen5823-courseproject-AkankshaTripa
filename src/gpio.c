/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

Modified By : Akanksha Tripathi & Vaibhavi Thakur
Credits     : Varun Mehta for GPIO Initialisation
 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>

#include "gpio.h"
#include "ble_device_type.h"

//// Student Edit: Define these, 0's are placeholder values.
//// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
//// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
//// to determine the correct values for these.


//// Set GPIO drive strengths and modes of operation
void gpioInit()
{

  // Student Edit:

 //  GPIO_DriveStrengthSet(gpioPortC, gpioDriveStrengthStrongAlternateStrong);


  GPIO_DriveStrengthSet(gpioPortD, gpioDriveStrengthStrongAlternateStrong);
    //GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(gpioPortD, 13, gpioModePushPull, false);
	GPIO_PinModeSet( gpioPortD, I2C_TEMP_PIN, gpioModePushPull, true );


	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, true);
  GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);

	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, true);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);

	GPIO_PinModeSet(PB0_port, PB0_pin, gpioModeInput, true);
	GPIO_PinModeSet(PB1_port, PB1_pin, gpioModeInput, true);

#if DEVICE_IS_BLE_SERVER
  // NO ACTION
#else
    GPIO_PinModeSet(PB1_port, PB1_pin, gpioModeInputPullFilter, true);
    GPIO_ExtIntConfig (PB1_port, PB1_pin, PB1_pin, true, true, true);
#endif

    GPIO_DriveStrengthSet(TRIG_PORT, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(TRIG_PORT, TRIG_PIN, gpioModePushPull, false);

    GPIO_PinModeSet(ECHO_PORT, ECHO_PIN, gpioModeInput, false);  //previously configured as o/p
     GPIO_ExtIntConfig(ECHO_PORT, ECHO_PIN, ECHO_PIN, true, true, true);

    GPIO_DriveStrengthSet(SIG_PORT, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(SIG_PORT, SIG_PIN, gpioModeInput, false);  //previously configured as o/p

  //Enabling Interrupt for GPIO
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	button_enable();

} // gpioInit()


void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

void gpiosdaclear()
{
  GPIO_PinOutClear(gpioPortC, SDA_PIN);
}
void gpiosclclear()
{
  GPIO_PinOutClear(gpioPortC, SCL_PIN );
}

void gpioSi7021enable()
{
  GPIO_PinOutSet(gpioPortD,I2C_TEMP_PIN);
}

void gpioSi7021disable()
{
  GPIO_PinOutClear(gpioPortD,I2C_TEMP_PIN);
}


void gpioSetDisplayExtcomin(bool pin)
{
    if(pin == true )
    {
        GPIO_PinOutSet(LCD_PORT, LCD_EXTCOMIN_PIN);
    }
    else
    {
        GPIO_PinOutClear(LCD_PORT, LCD_EXTCOMIN_PIN);
    }
}


void button_enable()
{
  GPIO_ExtIntConfig(PB0_port, PB0_pin, PB0_pin, true, true, true);              //added as part of A8
  GPIO_ExtIntConfig(PB1_port, PB1_pin, PB1_pin, true, true, true);             //added for A9
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

void disable_button_irq()
{
  NVIC_DisableIRQ(GPIO_EVEN_IRQn);
  NVIC_DisableIRQ(GPIO_ODD_IRQn);
}

void set_trig()
{
  GPIO_PinOutSet(TRIG_PORT, TRIG_PIN);
}
void clear_trig()
{
  GPIO_PinOutClear(TRIG_PORT, TRIG_PIN);
}

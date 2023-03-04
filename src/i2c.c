/*
 *Code Credits: Lecture Slides
 * */


//Initialize the I2C hardware
#include <em_i2c.h>
#define INCLUDE_LOG_DEBUG 1
#include "em_device.h"
#include "i2c.h"
#include "gpio.h"
#include "src/log.h"
#include "oscillators.h"
#include "sl_i2cspm.h"
#include "src/timers.h"
#include "src/ble.h"

#define SI7021_DEVICE_ADDR 0x40         //i2c slave address
#define NO_HOLD_MASTER_MODE 0XF3         //Sequence to perform temp measurement and read back result


I2CSPM_Init_TypeDef I2C_Config = {
 .port = I2C0,
 .sclPort = gpioPortC,
 .sclPin = 10,                          //scl pin = PC10
 .sdaPort = gpioPortC,
 .sdaPin = 11,                         //sda pin = pc11
 .portLocationScl = 14,
 .portLocationSda = 16,
 .i2cRefFreq = 0,
 .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,
 .i2cClhr = i2cClockHLRStandard
 };

void i2c_init()                         //initialise i2C
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  I2CSPM_Init(&I2C_Config);

  NVIC_EnableIRQ( I2C0_IRQn );
  CORE_EXIT_CRITICAL();

 }


// Send Measure Temperature command
I2C_TransferReturn_TypeDef transferStatus;
I2C_TransferSeq_TypeDef transferSequence;
uint8_t cmd_data;                            //variable to data to write
uint8_t read_data[2];                         //variable for read operation


void i2c_write()
{
  cmd_data = 0xF3;

  transferSequence.addr = SI7021_DEVICE_ADDR << 1; // shift device address left
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &cmd_data;        // pointer to data to write
  transferSequence.buf[0].len = sizeof(cmd_data);

  transferStatus = I2C_TransferInit (I2C0, &transferSequence);
  if (transferStatus < 0)
         {
            //LOG_ERROR("%d", transferStatus);
         }
}


void i2c_read()
{

  transferSequence.addr = SI7021_DEVICE_ADDR << 1;          // shift device address left
  transferSequence.flags = I2C_FLAG_READ;
  transferSequence.buf[0].data = read_data;               // pointer to data to read
  transferSequence.buf[0].len = sizeof(read_data);

  transferStatus = I2C_TransferInit (I2C0, &transferSequence);
  if (transferStatus < 0)
  {
    //LOG_ERROR("%d", transferStatus);
   }
}

void i2cStop()                                        //stops i2c
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

    I2C_Reset(I2C0);                                  //Reseting I2C to the same state that it was in after a hardware reset
    I2C_Enable(I2C0,false);                           //Disabling I2C
    gpiosdaclear();                                   //Clearing SDA pin
    gpiosclclear();                                   //Clearing SCL pin
    CMU_ClockEnable(cmuClock_I2C0, false);            //Disabling I2C clock

    NVIC_DisableIRQ( I2C0_IRQn );
    CORE_EXIT_CRITICAL();
    return;
}



int temperaturereading()
{
  uint16_t temperature;
  uint32_t temp_code;

              temperature= ((read_data[1]) | (read_data[0]<<8));          //performing temperature calculation
              temp_code = (175.72 * (temperature/65536.0)) - 46.85;
              LOG_INFO("Value of Si7021, temperature sensor is %d C\n\r",(int)temp_code);
              server_indication(temp_code);
              return temp_code;

}


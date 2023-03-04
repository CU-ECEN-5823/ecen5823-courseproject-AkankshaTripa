#ifndef EM_I2C_H
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

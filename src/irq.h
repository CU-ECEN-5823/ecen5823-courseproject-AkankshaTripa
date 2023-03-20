#ifndef __myInterrupt
#define __myInterrupt
// define anything a caller needs
// function prototypes

//LETIMER handler function
void LETIMER0_IRQHandler ();

//IRQ handler function
void I2C0_IRQHandler(void);

void GPIO_EVEN_IRQHandler();

#endif

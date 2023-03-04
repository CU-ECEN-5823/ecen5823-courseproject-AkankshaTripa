#include "irq.h"
//#define INCLUDE_LOG_DEBUG 1
#include <em_letimer.h>
#include <em_i2c.h>
#include "em_device.h"
#include "src/log.h"
#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>
#include "gpio.h"
#include "scheduler.h"
#include "i2c.h"


void LETIMER0_IRQHandler (void)
{
 // 1st: determine source of IRQ
 // 2nd: clear source of IRQ set in step 3
 // 3rd: perform whatever processing is required

  //uint8_t GO_TO_SLEEP=1;

    uint32_t flags;
    flags = LETIMER_IntGetEnabled(LETIMER0);               //determines source of irq

    LETIMER_IntClear(LETIMER0, flags);                     //clear source of irq
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();                                 // NVIC IRQs are disabled


    if(flags & LETIMER_IF_UF)                               //checking underflow flag
    {
        schedulerSetEventUF();                              //event scheduler to set temperature
    }

    if( flags & LETIMER_IF_COMP1 )                          //checking COMP1 flag
     {
        schedulerSetEventCOMP1();

        LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);//event set for Comp1
     }

    CORE_EXIT_CRITICAL();                                  // NVIC IRQs are re-enabled

 }// LETIMER0_IRQHandler()


void I2C0_IRQHandler(void)
{
  I2C_TransferReturn_TypeDef transferStatus;

  CORE_ATOMIC_IRQ_DISABLE();

  transferStatus = I2C_Transfer(I2C0);

  if (transferStatus == i2cTransferDone)
   {
      schedulerSetEventI2CDone();                              //setting event I2c done

   }
  if (transferStatus < 0)
   {
    // LOG_ERROR("%d", transferStatus);                        //printing error in other case
   }
  CORE_ATOMIC_IRQ_ENABLE();
 }



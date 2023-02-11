#include "irq.h"
#define INCLUDE_LOG_DEBUG 1
#include <em_letimer.h>
#include "src/log.h"
#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>
#include "gpio.h"
#include "scheduler.h"

void LETIMER0_IRQHandler (void)
{
 // 1st: determine source of IRQ
 // 2nd: clear source of IRQ set in step 3
 // 3rd: perform whatever processing is required

  //uint8_t GO_TO_SLEEP=1;

    uint32_t flags;
    flags = LETIMER_IntGetEnabled(LETIMER0);        //determines source of irq

    LETIMER_IntClear(LETIMER0, flags);         //clear source of irq
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL(); // NVIC IRQs are disabled


    if(flags & LETIMER_IF_UF)               //checking underflow flag
    {

        schedulerSetEventTemperatureMeasurement();         //perform action

    }



    CORE_EXIT_CRITICAL(); // NVIC IRQs are re-enabled

 }// LETIMER0_IRQHandler()

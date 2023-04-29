/*
 * Code Credits: Lecture Slides
 * ADCInit: Reference by Professor
 * Distance Sensor Configuration : Guidance by Varun Mehta
 * */
//#include <em_i2c.h>
#include "em_core.h"
#include "scheduler.h"
#include "src/gpio.h"
#include "src/i2c.h"
#include <sl_power_manager.h>
#include "em_device.h"
#include "timers.h"
#define INCLUDE_LOG_DEBUG 1
#include "log.h"
#include "ble.h"
#include "sl_bt_api.h"
#include "sl_bgapi.h"
#include "lcd.h"
#include "math.h"

#include "em_device.h"
#include "em_adc.h"
#include "em_chip.h"
#include "em_cmu.h"


int time_start = 0, time_end = 0, delta_time=0;
int  time = 0, distance = 0;
bool detection = 0;
uint16_t  sensor_data;
uint32_t event;
#define TABLE_INDEX_INVALID           ((uint8_t)0xFFu)

// scheduler routine to set a scheduler event
void schedulerSetEventUF()
{
 //STEPS:
 // enter critical section
 // set the event in your data structure, this has to be a read-modify-write
 // exit critical section

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();                          // NVIC IRQs are disabled

 // event |= eventuf;                               //RMW for event commented for A5

  sl_bt_external_signal(eventuf);

  CORE_EXIT_CRITICAL();                           // NVIC IRQs are re-enabled
} // schedulerSetEventUF()


void schedulerSetEventCOMP1()
{
 //STEPS:
 // enter critical section
 // set the event in your data structure, this has to be a read-modify-write
 // exit critical section

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();                          // NVIC IRQs are disabled

 // event |= eventcomp1;                               //RMW for event

  sl_bt_external_signal(eventcomp1);

  CORE_EXIT_CRITICAL();                           // NVIC IRQs are re-enabled
} // schedulerSetEventCOMP1()


void schedulerSetEventI2CDone()
{
 //STEPS:
 // enter critical section
 // set the event in your data structure, this has to be a read-modify-write
 // exit critical section

  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();                          // NVIC IRQs are disabled
  sl_bt_external_signal(i2ccomplete);

  //event |= i2ccomplete;                               //RMW for event

  CORE_EXIT_CRITICAL();                           // NVIC IRQs are re-enabled
} // schedulerSetEventI2CDone()

void schedulerSetEventCheckButtonStatusPB0()
{
  //STEPS:
  // enter critical section
  // set the event in your data structure, this has to be a read-modify-write
  // exit critical section

   CORE_DECLARE_IRQ_STATE;
   CORE_ENTER_CRITICAL();
  sl_bt_external_signal(checkbuttonPB0);
  CORE_EXIT_CRITICAL();
}//schedulerSetEventCheckButtonStatus()

void schedulerSetEventCheckButtonStatusPB1()
{
  //STEPS:
    // enter critical section
    // set the event in your data structure, this has to be a read-modify-write
    // exit critical section

     CORE_DECLARE_IRQ_STATE;
     CORE_ENTER_CRITICAL();
    sl_bt_external_signal(checkbuttonPB1);
    CORE_EXIT_CRITICAL();
}


void schedulerSetEventCheckEventEcho()
{
  //STEPS:
    // enter critical section
    // set the event in your data structure, this has to be a read-modify-write
    // exit critical section

     CORE_DECLARE_IRQ_STATE;
     CORE_ENTER_CRITICAL();
    sl_bt_external_signal(eventecho);
    CORE_EXIT_CRITICAL();
}

//comemnted below function as part of A5
// scheduler routine to return 1 event to main()code and clear that event
/*uint32_t getNextEvent()
{
  uint32_t theEvent;
  // select 1 event to return to main() code, apply priorities etc.
  //theEvent = waitevent;                                    // default event, does nothing
 // STEPS:
  // enter critical section
  // clear the event in your data structure, this has to be a read-modify-write
  // exit critical section
  CORE_DECLARE_IRQ_STATE;
  if((event & eventuf) == eventuf)
    {
      theEvent=eventuf;                             //event return to main function
      CORE_ENTER_CRITICAL();                                           // NVIC IRQs are disabled
      event=event & (eventuf^0xFFFFFFFF);           //clear event
      CORE_EXIT_CRITICAL();                                           // NVIC IRQs are re-enabled
    }
  if((event & eventcomp1) == eventcomp1)
      {
        theEvent=eventcomp1;                             //event return to main function
        CORE_ENTER_CRITICAL();                                           // NVIC IRQs are disabled
        event=event & (eventcomp1^0xFFFFFFFF);           //clear event
        CORE_EXIT_CRITICAL();                                           // NVIC IRQs are re-enabled
      }
  if((event & i2ccomplete) == i2ccomplete)
      {
        theEvent=i2ccomplete;                             //event return to main function
        CORE_ENTER_CRITICAL();                                           // NVIC IRQs are disabled
        event=event & (i2ccomplete^0xFFFFFFFF);           //clear event
        CORE_EXIT_CRITICAL();                                           // NVIC IRQs are re-enabled
      }
  return (theEvent);
 } // getNextEvent()
*/

//getting value from ADC
uint16_t get_adc_data()
{
  LOG_INFO("IN ADC DATA");
//  NVIC_EnableIRQ(ADC0_IRQn);
 // ADC_IntEnable(ADC0,ADC_IF_SINGLE);
  ADC_Start(ADC0, adcStartSingle);

  while(!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK));

      uint32_t sample = ADC_DataSingleGet(ADC0);
      LOG_INFO("sample= %d\n\r", sample);
      sensor_data = ((sample * 2500)/4096);
      LOG_INFO("Sensor data= %d\n\r", sensor_data);

  return sensor_data;
}

//Initialising ADC
void ADCInit(void)
{

  CMU_Select_TypeDef clockSelectType;
  clockSelectType=CMU_ClockSelectGet(cmuClock_HF);

  uint32_t frequency;
  frequency=CMU_ClockFreqGet(cmuClock_HFPER);

  LOG_INFO("IN SENSOR INIT");
  /* Enabling the ADC0 clock*/
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* Base the ADC configuration on the default setup. */
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;

  uint8_t timebase, prescale;
  timebase=ADC_TimebaseCalc(frequency);
  prescale=ADC_PrescaleCalc(4000000, 0);

  init.ovsRateSel = adcOvsRateSel2;
  init.warmUpMode = adcWarmupNormal;
  init.timebase = timebase;
  init.prescale = prescale;
  init.tailgate = false;
  init.em2ClockConfig = adcEm2Disabled;


  ADC_Init(ADC0, &init);


  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

//  init.prescale = ADC_PrescaleCalc(4000000, 0); // Init to max ADC clock

  initSingle.prsSel     = adcPRSSELCh0;
  initSingle.diff       = false;        // single ended
  initSingle.reference  = adcRefVDD;    // internal 2.5V reference
  initSingle.resolution = adcRes12Bit;  // 12-bit resolution
  initSingle.acqTime    = adcAcqTime1;  // set acquisition time to meet minimum requirement

  initSingle.posSel = adcPosSelAPORT3XCH2;

  initSingle.negSel = adcNegSelVSS;

  initSingle.prsEnable  = false;
  initSingle.leftAdjust = false;
  initSingle.rep        = false;
  initSingle.singleDmaEm2Wu= false;
  initSingle.fifoOverwrite = false;


  ADC_InitSingle(ADC0, &initSingle);
}



void state_machine(sl_bt_msg_t *evt)
{

  sl_status_t sc;
  //State_t currentState;
  static State_t nextState = stateIdle;
  //currentState = nextState;


  ble_data_struct_t *bleDataPtr = getBleDataPtr();
  if(SL_BT_MSG_ID(evt->header)!=sl_bt_evt_system_external_signal_id)
    {
      return;
    }

//  if( (bleDataPtr->connection_open==false) && bleDataPtr->ok_to_send_htm_indications==false)
//    {
//      LOG_INFO("stateIdle disabling\n\r");
//   //   gpioSi7021disable();                                                //disabling Si7021
//      i2cStop();                                                          //stop i2c transfer
//      nextState=stateIdle;
//    }


 // event_si7021 event_new=evt;
  if(bleDataPtr->connection_open==true && bleDataPtr->ok_to_send_htm_indications==true)
  {
      switch(nextState)
  {



   case stateIdle:
      nextState = stateIdle;
      LOG_INFO("stateIdle before if %d\n\r", evt->data.evt_system_external_signal.extsignals);
      if(evt->data.evt_system_external_signal.extsignals ==eventuf)
        {
          LOG_INFO("stateIdle\n\r");

          gpioSi7021enable();                           //enable temp sensor
          timerWaitUs_irq(80000);                       //wait for 80ms to powerup si7021

          set_trig();

           ADCInit();

                    uint16_t light_intensity_data = get_adc_data();

                   LOG_INFO("Light Intensity= %d\n\r", light_intensity_data);

                    displayPrintf(DISPLAY_ROW_8, "Light = %d", light_intensity_data);

                     sc = sl_bt_gatt_server_send_indication(bleDataPtr->connectionhandle,      // characteristic
                                                            gattdb_light_measure,            // Attribute from gatt_db.h
                                                            2,                                  // value_len
                                                            &light_intensity_data);

                              timerWaitUs(10);
                              nextState=statetimerwait80;

        }
      break;

    case statetimerwait80:
      nextState=statetimerwait80;
      LOG_INFO("statetimer wait before if %d\n\r", evt->data.evt_system_external_signal);
      if(evt->data.evt_system_external_signal.extsignals==eventcomp1)
        {
          LOG_INFO("statetimerwait80\n\r");
          clear_trig();
          i2c_init();                                                     //intilaise i2c transfer
         sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);      //add power requirements for EM1 mode
          i2c_write();                                                    //perform i2c write
          nextState=statei2cwrite;

        }
      break;

    case statei2cwrite:
      nextState=statei2cwrite;
      if(evt->data.evt_system_external_signal.extsignals==(i2ccomplete||eventecho))
        {
            LOG_INFO("ECHO CHECK statei2cwrite\n\r");
            time_start = LETIMER_CounterGet(LETIMER0);
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);       //remove power for EM1
          timerWaitUs_irq(10800);                                             //wait for 10.8 ms for calculations
          nextState=statetimerwait108;
        }
      break;

    case statetimerwait108:
        nextState=statetimerwait108;
        if(evt->data.evt_system_external_signal.extsignals==eventcomp1)
        {
           LOG_INFO("statetimerwait108\n\r");
           time_end = LETIMER_CounterGet(LETIMER0);
             LOG_INFO("time_end = %d\n\r", time_end);
           sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);          //add power req for read operation
           i2c_read();                                                         //perform read operation
           nextState=statei2cread;
         }
        break;

    case statei2cread:
          nextState=statei2cread;
            if(evt->data.evt_system_external_signal.extsignals==i2ccomplete)
             {
               LOG_INFO("statei2cread\n\r");

               // server_indication();
               temperaturereading();

                sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);       //remove power req

                              delta_time = time_start-time_end;
                             LOG_INFO("delta_time = %d\n\r", delta_time);
                             time = (delta_time*1000000)>>13;
                             LOG_INFO("time = %d\n\r", time);
                             distance = (float) time/58;

                             LOG_INFO("distance1 = %d\n\r", distance);
                             displayPrintf(DISPLAY_ROW_TEMPVALUE, "Distance = %d", distance);
                             sc = sl_bt_gatt_server_send_indication(bleDataPtr->connectionhandle,
                                                                    gattdb_distance_measure,
                                                                    sizeof(distance),
                                                                    &distance);

               // gpioSi7021disable();                                                //disabling Si7021
                i2cStop();                                                          //stop i2c transfer
                nextState=stateIdle;
              }

      break;
    default:
          {
            LOG_INFO("Si7021, temperature sensor not working\n\r");
          }
       }

  }
}


void discovery_state_machine(sl_bt_msg_t *evt)
{

 uint32_t event = SL_BT_MSG_ID(evt->header);
  //uint8_t * client_temperature;
  ble_data_struct_t *bleDataPtr = getBleDataPtr();

  Client_t currentState;
  static Client_t nextState = open_first;
  currentState = nextState;
  //uint16_t addr_value;
 sl_status_t sc;

  switch(currentState)
  {
    case ideal:
      {
        LOG_INFO("ideal\n\r");
        if(event == sl_bt_evt_scanner_scan_report_id){
          nextState=open_first;
        }

      }
      break;


    case open_first :
      {
        nextState=open_first;
        LOG_INFO("open started\n\r");
        if(event == sl_bt_evt_connection_opened_id)
           {
            LOG_INFO("open mid started\n\r");
            bleDataPtr->connectionhandle =evt->data.evt_connection_opened.connection;

            sc=sl_bt_gatt_discover_primary_services_by_uuid(bleDataPtr->connectionhandle ,
                                                     sizeof(thermo_service),
                                                     thermo_service
                                                     );
            if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
              }

             LOG_INFO("open\n\r");
             nextState=  open_second;
           }
        else
          {
            if(event == sl_bt_evt_connection_closed_id)
              {
                LOG_INFO("open closed else\n\r");
                nextState=ideal;
              }
           }
      }
      LOG_INFO("open closed\n\r");
      break;

    case open_second:
            nextState=open_second;
            LOG_INFO("discovery started\n\r");
            if(event == sl_bt_evt_gatt_procedure_completed_id)
              {
                sc = sl_bt_gatt_discover_primary_services_by_uuid(bleDataPtr->connectionhandle,
                                                                  sizeof(Pushbutton_service),
                                                                  Pushbutton_service);
                 if (sc != SL_STATUS_OK)
                 {
                    LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                 }

                 nextState=  discovery_first;
              }
            else
                {
                   if(event == sl_bt_evt_connection_closed_id)
                     {
                            LOG_INFO("open closed else\n\r");
                            nextState=ideal;
                       }
                  }
      break;

    case discovery_first :
      {
        nextState=discovery_first;
        LOG_INFO("discovery first started\n\r");
        if(event == sl_bt_evt_gatt_procedure_completed_id)
        {
          //  bleDataPtr.coconnectionhandle =evt->data.evt_connection_opened.connection;
            LOG_INFO("discovery start inside if condition\n\r");
           sc= sl_bt_gatt_discover_characteristics_by_uuid(bleDataPtr->connectionhandle,
                                                           bleDataPtr->serviceHandle[0],
                                                           sizeof(thermo_char),
                                                           (const uint8_t*)thermo_char
                                                            ) ;
                                                                 // uint8_t connection,
                                                                 //   uint32_t service,
                                                                 //   size_t uuid_len,
                                                                 //   const uint8_t* uuid
           if (sc != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
             }

                  nextState=discovery_second;
        }
        else{
                 if(event == sl_bt_evt_connection_closed_id)
                 {
                    nextState=ideal;
                 }
              }
      }
      LOG_INFO("discovery ended\n\r");

      break;

    case discovery_second :
         {
           nextState=discovery_second;
           LOG_INFO("discovery second started\n\r");
           if(event == sl_bt_evt_gatt_procedure_completed_id)
           {
             //  bleDataPtr.coconnectionhandle =evt->data.evt_connection_opened.connection;
               LOG_INFO("discovery start inside if condition\n\r");
              sc= sl_bt_gatt_discover_characteristics_by_uuid(bleDataPtr->connectionhandle,
                                                           bleDataPtr->serviceHandle[1],
                                                           sizeof(Pushbutton_characteristics),
                                                           (const uint8_t*)Pushbutton_characteristics
                                                           ) ;
                                                                    // uint8_t connection,
                                                                    //   uint32_t service,
                                                                    //   size_t uuid_len,
                                                                    //   const uint8_t* uuid
              if (sc != SL_STATUS_OK)
               {
                 LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                }

                     nextState=notify_first;
           }
           else{
                    if(event == sl_bt_evt_connection_closed_id)
                    {
                       nextState=ideal;
                    }
                 }
         }
         LOG_INFO("discovery second ended\n\r");

         break;

    case notify_first :
      {
        nextState=notify_first;
        LOG_INFO("notify first started\n\r");
        if(event == sl_bt_evt_gatt_procedure_completed_id)
          {
            LOG_INFO("notify state in\n\r");

            sc= sl_bt_gatt_set_characteristic_notification(bleDataPtr->connectionhandle,
                                                           bleDataPtr->characteristicHandle[0],
                                                           sl_bt_gatt_indication);

                                                        //uint8_t connection,
                                                        // uint16_t characteristic,
                                                        // uint8_t flags

            displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications");

            if (sc != SL_STATUS_OK)
           {
               LOG_ERROR("sl_bt_gatt_set_characteristic_notification() returned != 0 status=0x%04x", (unsigned int) sc);
            }

             nextState=notify_second;
           }
     else{

            if(event == sl_bt_evt_connection_closed_id)
           {
              nextState=ideal;
            }
         }
      }
      LOG_INFO("notify first ended\n\r");
      break;

    case notify_second :
          {
            nextState=notify_second;
            LOG_INFO("notify first started\n\r");
            if(event == sl_bt_evt_gatt_procedure_completed_id)
              {
                LOG_INFO("notify state in\n\r");

                sc= sl_bt_gatt_set_characteristic_notification(bleDataPtr->connectionhandle,
                                                               bleDataPtr->characteristicHandle[1],
                                                               sl_bt_gatt_indication);
                                                                //uint8_t connection,
                                                            // uint16_t characteristic,
                                                            // uint8_t flags
                if (sc != SL_STATUS_OK)
               {
                   LOG_ERROR("sl_bt_gatt_set_characteristic_notification() returned != 0 status=0x%04x", (unsigned int) sc);
                }

                 nextState=confirmation;
               }
         else{

                if(event == sl_bt_evt_connection_closed_id)
               {
                  nextState=ideal;
                }
             }
          }
          LOG_INFO("notify first ended\n\r");
          break;


    case confirmation:
      {
        LOG_INFO("entering confirmation\n\r");
       if(event == sl_bt_evt_gatt_characteristic_value_id)
          {
           //confirmation state changes done in ble
                      nextState=close;
           }
          else{
                 if(event == sl_bt_evt_connection_closed_id)
                  {
                      nextState=ideal;
                   }
                  }

         }
         LOG_INFO("confirmation ended\n\r");
     break;
    case close :
      {
        LOG_INFO("close started\n\r");
        nextState=close;
        if(event == sl_bt_evt_connection_closed_id)
          {
            nextState=ideal;
           }
      }
      LOG_INFO("close ended\n\r");
      break;
  }
}

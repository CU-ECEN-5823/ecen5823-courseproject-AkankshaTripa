/*
 * Code Credits: Lecture Slides
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

// interrupt service routine for a peripheral
// CPU+NVIC clear the IRQ pending bit in the NVIC
// when this routine is fetched from memory.

// -----------------------------------------------
// Private function, original from Dan Walkes. I fixed a sign extension bug.
// We'll need this for Client A7 assignment to convert health thermometer
// indications back to an integer. Convert IEEE-11073 32-bit float to signed integer.
// -----------------------------------------------
int32_t FLOAT_TO_INT32(const uint8_t *value_start_little_endian)
{
 uint8_t signByte = 0;
 int32_t mantissa;
 // input data format is:
 // [0] = flags byte
 // [3][2][1] = mantissa (2's complement)
 // [4] = exponent (2's complement)
 // BT value_start_little_endian[0] has the flags byte
 int8_t exponent = (int8_t)value_start_little_endian[4];
 // sign extend the mantissa value if the mantissa is negative
 if (value_start_little_endian[3] & 0x80) { // msb of [3] is the sign of the mantissa
 signByte = 0xFF;
 }
 mantissa = (int32_t) (value_start_little_endian[1] << 0) |
 (value_start_little_endian[2] << 8) |
 (value_start_little_endian[3] << 16) |
 (signByte << 24) ;
 // value = 10^exponent * mantissa, pow() returns a double type
 return (int32_t) (pow(10, exponent) * mantissa);
}
// FLOAT_TO_INT32



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

void schedulerSetEventCheckButtonStatus()
{
  //STEPS:
  // enter critical section
  // set the event in your data structure, this has to be a read-modify-write
  // exit critical section

   CORE_DECLARE_IRQ_STATE;
   CORE_ENTER_CRITICAL();
  sl_bt_external_signal(checkbutton);
  CORE_EXIT_CRITICAL();
}//schedulerSetEventCheckButtonStatus()

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

void state_machine(sl_bt_msg_t *evt)
{


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
  if(bleDataPtr->connection_open==true && bleDataPtr->ok_to_send_htm_indications==true )
  {
      switch(nextState)
  {



   case stateIdle:
      nextState = stateIdle;
      LOG_INFO("stateIdle before if %d\n\r", evt->data.evt_system_external_signal.extsignals);
      if(evt->data.evt_system_external_signal.extsignals ==eventuf)
        {
          LOG_INFO("stateIdle\n\r");
          nextState=statetimerwait80;
          gpioSi7021enable();                           //enable temp sensor
          timerWaitUs_irq(80000);                       //wait for 80ms to powerup si7021
        }
      break;

    case statetimerwait80:
      nextState=statetimerwait80;
      LOG_INFO("statetimer wait before if %d\n\r", evt->data.evt_system_external_signal);
      if(evt->data.evt_system_external_signal.extsignals==eventcomp1)
        {
          LOG_INFO("statetimerwait80\n\r");
          i2c_init();                                                     //intilaise i2c transfer
         sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);      //add power requirements for EM1 mode
          i2c_write();                                                    //perform i2c write
          nextState=statei2cwrite;

        }
      break;

    case statei2cwrite:
      nextState=statei2cwrite;
      if(evt->data.evt_system_external_signal.extsignals==i2ccomplete)
        {
          LOG_INFO("statei2cwrite\n\r");
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
  uint8_t * client_temperature;
  ble_data_struct_t *bleDataPtr = getBleDataPtr();

  Client_t currentState;
  static Client_t nextState = open;
  currentState = nextState;
  //uint16_t addr_value;
 sl_status_t sc;

  switch(currentState)
  {
    case ideal:
      {
        LOG_INFO("ideal\n\r");
        if(event == sl_bt_evt_scanner_scan_report_id){
          nextState=open;
        }

      }
      break;


    case open :
      {
        nextState=open;
        LOG_INFO("open started\n\r");
        if(event == sl_bt_evt_connection_opened_id)
           {
            LOG_INFO("open mid started\n\r");
            bleDataPtr->connectionopenhandle =evt->data.evt_connection_opened.connection;

            sc=sl_bt_gatt_discover_primary_services_by_uuid(bleDataPtr->connectionopenhandle ,
                                                     sizeof(thermo_service),
                                                     thermo_service
                                                     );
            if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
              }

             LOG_INFO("open\n\r");
             nextState=  discovery;
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


    case discovery :
      {
        nextState=discovery;
        LOG_INFO("discovery started\n\r");
        if(event == sl_bt_evt_gatt_procedure_completed_id)
        {
          //  bleDataPtr.connectionopenhandle =evt->data.evt_connection_opened.connection;
            LOG_INFO("discovery start inside if condition\n\r");
           sc= sl_bt_gatt_discover_characteristics_by_uuid(bleDataPtr->connectionopenhandle,
                                                        bleDataPtr->serviceHandle,
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

                  nextState=notify;
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
    case notify :
      {
        nextState=notify;
        LOG_INFO("notify started\n\r");
        if(event == sl_bt_evt_gatt_procedure_completed_id)
          {
            LOG_INFO("notify state in\n\r");

            sc= sl_bt_gatt_set_characteristic_notification(bleDataPtr->connectionopenhandle,
                                                   bleDataPtr->characteristicHandle,
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
      LOG_INFO("notify ended\n\r");
      break;
    case confirmation:
      {
        if(event == sl_bt_evt_gatt_characteristic_value_id)
          {
                LOG_INFO("confirmation started\n\r");

            sc = sl_bt_gatt_send_characteristic_confirmation(bleDataPtr->connectionopenhandle);

           if (sc != SL_STATUS_OK)
           {
             LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() returned != 0 status=0x%04x", (unsigned int) sc);
           }


            client_temperature = (evt->data.evt_gatt_characteristic_value.value.data);
           LOG_INFO("Client Temperature = %d\r\n", client_temperature[4]);
           int32_t temp = FLOAT_TO_INT32 (client_temperature);
           LOG_INFO("Received Temperature = %d\r\n", (temp));

           displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %d", (temp));

                nextState=confirmation;
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

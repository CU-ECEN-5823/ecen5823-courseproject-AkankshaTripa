/**************************************************************

*                     Project Name : Home Automation System
                      File Name    : ble.c
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
*************************************************************************************************************************************/
#include "ble.h"
#include <stdbool.h>
#include "em_common.h"
#include "sl_status.h"
#include "scheduler.h"
#include "app_log.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "sl_bt_api.h"
#include "sl_bt_api_compatibility.h"
#include "i2c.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "src/lcd.h"
#include "ble_device_type.h"
#include "src/gpio.h"
#include "em_gpio.h"
#include "math.h"
#include "timers.h"

#define MIN_INTERVAL 60
#define MAX_INTERVAL 60
#define LATENCY 4
#define TIMEOUT 660
#define MIN_CE_LENGTH 0
#define MAX_CE_LENGTH 0
buffer_t bufferobj;
buff queue;

#define BONDING 2
#define PAIRING 1
#define DELETE 0
#define CHECK_BUTTON_EVENT_PB0 8
#define CHECK_BUTTON_EVENT_PB1 16

// BLE private data
ble_data_struct_t ble_data;
#define SCAN_PASSIVE 0
#define SECURITY_FLAG 0x0F

#define SERVICE 0
#define CHARACTERISTIC 1
#define ULTRASONIC 2
#define LIGHT 3

#define MIN_ADVERTISING_INTERVAL 400
#define MAX_ADVERTISING_INTERVAL 400
#define DURATION 0
#define MAX_EVENTS 0
#define TIME 4096
#define HANDLE 1
#define SINGLE_SHOT 0

bool DetectionSuccess=0;

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

// Parse advertisements looking for advertised Health Thermometer service
uint8_t find_service_in_advertisement(uint8_t *data, uint8_t len)
{
  uint8_t ad_field_length;
  uint8_t ad_field_type;
  uint8_t i = 0;
  // Parse advertisement packet
  while (i < len) {
    ad_field_length = data[i];
    ad_field_type = data[i + 1];
    // Partial ($02) or complete ($03) list of 16-bit UUIDs
    if (ad_field_type == 0x02 || ad_field_type == 0x03) {
      // compare UUID to Health Thermometer service UUID
      if (memcmp(&data[i + 2], thermo_service, 2) == 0) {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + ad_field_length + 1;
  }
  return 0;
}

// this is the declaration
// function that returns a pointer to the
// BLE private data
ble_data_struct_t* getBleDataPtr() {
 return (&ble_data);
} // getBleDataPtr()


/****************************************************************************/

#if (DEVICE_IS_BLE_SERVER==1)

void initialise_cbfifo(buff *cb)
{
  cb->wptr = 0;
  cb->rptr = 0;
  cb->isFull= 0;
}

uint32_t nextPtr(uint32_t ptr)
{

 //referred this function according to the pdf provided in class on Circular Buffer
  if(ptr+1==CAPACITY)
  {
       return 0;  //wrapping back to 0
  }
  else
  {
    return ptr+1;
  }
} // nextPtr()

int write_queue(buff* Cbfifo, buffer_t *buf)
{
    if(buf == NULL || Cbfifo == NULL)
        return -1;

    if((Cbfifo->wptr == Cbfifo->rptr) && (Cbfifo->isFull == 1))
      return -1;

    Cbfifo->Data[Cbfifo->wptr] = *buf;
    Cbfifo->wptr=nextPtr(Cbfifo->wptr);

    if(Cbfifo->wptr == Cbfifo->rptr)            //checking for full condition
       Cbfifo->isFull = 1;

    return 0;
}

int read_queue(buff* Cbfifo, buffer_t *buf)
{
    if(buf == NULL || Cbfifo == NULL)
        return -1;

    if((Cbfifo->wptr == Cbfifo->rptr) && (Cbfifo->isFull == 0))
      return -1;

    *buf = Cbfifo->Data[Cbfifo->rptr];
     Cbfifo->rptr=nextPtr(Cbfifo->rptr);

    if(Cbfifo->isFull == 1)
      Cbfifo->isFull = 0;

    return 0;
}

size_t cbfifo_length(buff* Cbfifo)
{
  if(Cbfifo->isFull == 0)
      return (Cbfifo->wptr - Cbfifo->rptr);

  else return 1;
}

#else

static uint8_t UUID_Compare(sl_bt_msg_t *evt, uint8_t event_type)
{

  uint8_t check_uuid = 0;                                      //counter for checking uuid

  for(int i = 0; i < 16; i++)                                 // Check each address byte
    {
      if(event_type == SERVICE)
        {
          if(evt->data.evt_gatt_service.uuid.data[i] == Pushbutton_service[i])
          {
            check_uuid++;
          }
        }

      else if(event_type == CHARACTERISTIC)
        {
          if(evt->data.evt_gatt_characteristic.uuid.data[i] == Pushbutton_characteristics[i])
          {
            check_uuid++;
          }
        }

    }

  if(check_uuid == 16)          //address matching done
    {
      return 1;
    }

  return 0;                   //match failure
}
#endif


/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void handle_ble_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;
    bd_addr address;
   uint8_t address_type;
   uint8_t system_id[8];


#if (DEVICE_IS_BLE_SERVER ==0)
   bd_addr add=SERVER_BT_ADDRESS;
#endif

  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header))
  {

  #if DEVICE_IS_BLE_SERVER

    // The advertising set handle allocated from Bluetooth stack.
    static uint8_t advertising_set_handle = 0xff;
         // -------------------------------
         // This event indicates the device has started and the radio is ready.
         // Do not call any stack command before receiving this boot event!
   case sl_bt_evt_system_boot_id:
           // Print boot message.
     //      app_log_info("Bluetooth stack booted: v%d.%d.%d-b%d\n",
     //                   evt->data.evt_system_boot.major,
     //                   evt->data.evt_system_boot.minor,
     //                   evt->data.evt_system_boot.patch,
     //                   evt->data.evt_system_boot.build);

            // Extract unique ID from BT Address.
           sc = sl_bt_system_get_identity_address(&address, &address_type);

           if (sc != SL_STATUS_OK) {
                  LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
                  }

            displayInit();
            displayPrintf(DISPLAY_ROW_NAME, "%s", "Server");
            displayPrintf(DISPLAY_ROW_ASSIGNMENT, "%s", "A9");
            displayPrintf( DISPLAY_ROW_BTADDR, "%X:%X:%X:%X:%X:%X",
                          address.addr[ 0 ],  address.addr[ 1 ],  address.addr[ 2 ],
                          address.addr[ 3 ],  address.addr[ 4 ], address.addr[ 5 ] );
            displayPrintf(DISPLAY_ROW_CONNECTION,"%s", "Advertising");


           if (sc != SL_STATUS_OK) {
            LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
            }

           sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                        0,
                                                       sizeof(system_id),
                                                       system_id);

           if (sc != SL_STATUS_OK) {
                  LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
                  }


          LOG_INFO("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                        address_type ? "static random" : "public device",
                        address.addr[5],
                        address.addr[4],
                        address.addr[3],
                        address.addr[2],
                        address.addr[1],
                        address.addr[0]);

           // Create an advertising set.
           sc = sl_bt_advertiser_create_set(&advertising_set_handle);

           if (sc != SL_STATUS_OK) {
                  LOG_ERROR("sl_bt_advertiser_create_set() returned != 0 status=0x%04x", (unsigned int) sc);
                  }


           // Set advertising interval to 100ms.
           sc = sl_bt_advertiser_set_timing(
                           advertising_set_handle, // advertising set handle
                           MIN_ADVERTISING_INTERVAL, // min. adv. interval (milliseconds * 1.6)=250*1.6
                           MAX_ADVERTISING_INTERVAL, // max. adv. interval (milliseconds * 1.6)
                           DURATION,   // adv. duration
                           MAX_EVENTS);  // max. num. adv. events

           if (sc != SL_STATUS_OK) {
                           LOG_ERROR("sl_bt_advertiser_set_timing() returned != 0 status=0x%04x", (unsigned int) sc);
                           }

           // Start general advertising and enable connections.
           sc = sl_bt_advertiser_start(
             advertising_set_handle,
             sl_bt_advertiser_general_discoverable,
             sl_bt_advertiser_connectable_scannable);
           if (sc != SL_STATUS_OK) {
                           LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x", (unsigned int) sc);
                           }


           LOG_INFO("Started advertising first\n\r");

           //Added as part of A8

            sc = sl_bt_sm_configure(SECURITY_FLAG, sl_bt_sm_io_capability_displayyesno);                          //Configure security requirements and I/O capabilities of the system
            if (sc != SL_STATUS_OK) {
                            LOG_ERROR("sl_bt_sm_configure() returned != 0 status=0x%04x", (unsigned int) sc);
                            }


           sc = sl_bt_sm_delete_bondings();                                                                       //Delete all bonding information
           if (sc != SL_STATUS_OK) {
                           LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x", (unsigned int) sc);
                           }


          sc = sl_bt_system_set_soft_timer(TIME, HANDLE, SINGLE_SHOT);                                                          //uint32_t time,uint8_t handle, uint8_t single_shot


          if (sc != SL_STATUS_OK) {
                          LOG_ERROR("sl_bt_system_set_soft_timer() returned != 0 status=0x%04x", (unsigned int) sc);
                          }


         gpioLed0SetOff();
         gpioLed1SetOff();


           break;

         // -------------------------------
         // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:

           LOG_INFO("Connection opened\n\r");

           sl_bt_advertiser_stop(advertising_set_handle);

           ble_data.connectionhandle =evt->data.evt_connection_opened.connection;
           ble_data.connection_open=true;

           if(ble_data.connection_open==true)
             {
               displayPrintf( DISPLAY_ROW_TEMPVALUE, "%s"," " );
               displayPrintf( DISPLAY_ROW_CONNECTION, "%s","Connected" );
             }


           sl_bt_connection_set_parameters(ble_data.connectionhandle,
                                           MIN_INTERVAL,                     //60*1.25=75ms    //Time = Value x 1.25 ms
                                           MAX_INTERVAL,
                                           LATENCY,                          // 4 * 75ms intervals = 300ms for slave latency
                                           TIMEOUT,
                                           MIN_CE_LENGTH,
                                           MAX_CE_LENGTH);                  // timeout  =  ((1 + slave latency) * (connection_interval * 2))

                                              /*uint8_t connection,
                                                 uint16_t min_interval,
                                                 uint16_t max_interval,
                                                 uint16_t latency,
                                                 uint16_t timeout,
                                                 uint16_t min_ce_length,
                                                 uint16_t max_ce_length*/


     #ifdef SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT
           // Set remote connection power reporting - needed for Power Control
           sc = sl_bt_connection_set_remote_power_reporting(
             evt->data.evt_connection_opened.connection,
             sl_bt_connection_power_reporting_enable);
          // app_assert_status(sc);
     #endif // SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT

           break;

         // -------------------------------------------------------------------------------------------
         // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:

           LOG_INFO("Connection closed\n\r");

           // Restart advertising after client has disconnected.

           ble_data.connection_open=false;
           ble_data.ok_to_send_htm_indications = false;
           ble_data.indication_in_flight = false;

           sc = sl_bt_advertiser_start(
             advertising_set_handle,
             sl_bt_advertiser_general_discoverable,
             sl_bt_advertiser_connectable_scannable);

           if (sc != SL_STATUS_OK)
           {
                 LOG_ERROR(" sl_bt_advertiser_start() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
            }

           if(ble_data.connection_open==false)
             {
                      displayPrintf( DISPLAY_ROW_TEMPVALUE, "%s"," " );
                      displayPrintf(DISPLAY_ROW_CONNECTION, "%s","Advertising");

             }

           LOG_INFO("Started advertising\n\r");

                    //Added below lines as part of A8
                     sc = sl_bt_sm_delete_bondings();

                      if (sc != SL_STATUS_OK)
                     {
                        LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                      }

                              ble_data.state = DELETE;        //DELETE BONDING
                              gpioLed0SetOff();
                              gpioLed1SetOff();

           break;


         case sl_bt_evt_connection_parameters_id:
           //when connection is established
           LOG_INFO("CONNECTION PARAMETER is %d\r", evt->data.evt_connection_parameters.connection);
           LOG_INFO("CONNECTION INTERVAL is %d\r", evt->data.evt_connection_parameters.interval);
           LOG_INFO("LATENCY is %d\r", evt->data.evt_connection_parameters.latency);
           LOG_INFO("TIMEOUT is %d\r", evt->data.evt_connection_parameters.timeout);


           break;



   case sl_bt_evt_system_external_signal_id:

              LOG_INFO("external event called\r\n");
              if(evt->data.evt_system_external_signal.extsignals == CHECK_BUTTON_EVENT_PB0)
               {
                  LOG_INFO("external event called inside\r\n");

                            if(ble_data.state == PAIRING)
                              {

                                displayPrintf(DISPLAY_ROW_PASSKEY, " ");
                                displayPrintf(DISPLAY_ROW_ACTION, " ");

                                //calling passkey confirmation after PB0 is pressed for confirmation
                                sc = sl_bt_sm_passkey_confirm(ble_data.connectionhandle, 1);

                                if (sc != SL_STATUS_OK)
                                {
                                   LOG_ERROR("sl_bt_sm_passkey_confirm() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                                  }


                                   ble_data.state = BONDING;   //bonding

                              }

                     uint8_t  PB0_status = GPIO_PinInGet(PB0_port, PB0_pin);       // Status checking for PB0


                     if(PB0_status == 1)
                    {
                       displayPrintf(DISPLAY_ROW_9, "Button Released");          //PB0 release when 1
                    }

                     else if(PB0_status == 0)
                    {
                        displayPrintf(DISPLAY_ROW_9, "Button Pressed");           //PB0 connected to ground
                     }

                      // Negating the status of PB0
                     PB0_status = !PB0_status;


                     sc = sl_bt_gatt_server_write_attribute_value(
                                                       gattdb_button_state,
                                                       0,
                                                       1,
                                                       &PB0_status
                                                   );
                                                                        // uint16_t attribute,
                                                                       //  uint16_t offset,
                                                                        // size_t value_len,
                                                                       //  const uint8_t* value

                    if (sc != SL_STATUS_OK)
                   {
                       LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                    }



                               // Send indications after bonding if connection is open, indications are enabled, connection handle is not equal to 0 and
                                                                                                    //indication in flight is false

                                 if( ( ble_data.button_enable == 1) &&  (ble_data.state == BONDING) && (ble_data.connectionhandle != 0)
                                                                                                   && (ble_data.indication_in_flight == false) )

                                   {
                                       LOG_INFO("ERROR FROM EXTERNAL SEND INDICATION\r\n");

                                          sc = sl_bt_gatt_server_send_indication(ble_data.connectionhandle,
                                                                                  gattdb_button_state,
                                                                                  1,
                                                                                  &PB0_status);

                                           ble_data.indication_in_flight = true;

                                           if (sc != SL_STATUS_OK)
                                             {
                                               LOG_INFO("CONNECTION OPEN AND FLIGHT=FASLE\r\n");
                                               LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                                               ble_data.indication_in_flight = false;
                                             }


                                   }



                                 else if ( ( ble_data.button_enable == 1) &&  (ble_data.state == BONDING) && ( ble_data.connectionhandle != 0)
                                                                                                && (ble_data.indication_in_flight == true) )

                                  {
                                     LOG_INFO("CONNECTION OPEN AND FLIGHT=TRUE\r\n");
                                    bufferobj.charHandle   = gattdb_button_state;
                                    bufferobj.bufferLength = 1;
                                    memcpy(bufferobj.buffer, &PB0_status, 1);
                                     if((write_queue(&queue, &bufferobj)) == -1)
                                       {
                                          LOG_ERROR("Queue is full now, new events will be discarded\r\n");}

                                  }
               }


               break;

         // Events only for Slaves/Servers
     case sl_bt_evt_gatt_server_characteristic_status_id:

               LOG_INFO("status_flags  = %d\r", evt->data.evt_gatt_server_characteristic_status.status_flags);
               LOG_INFO("client_config_flags  = %d\r", evt->data.evt_gatt_server_characteristic_status.client_config_flags);
               LOG_INFO("client_config  = %d\r", evt->data.evt_gatt_server_characteristic_status.client_config);

               //TEMPERATURE CHARACTERISTICS
               if (evt->data.evt_gatt_server_characteristic_status.characteristic
                   == gattdb_temperature_measurement)
               {

                   LOG_INFO("IN THE TEMPERATURE CHARACTERISTIC\n\r");
                            if (evt->data.evt_gatt_server_characteristic_status.status_flags
                                                                != sl_bt_gatt_server_client_config)
                           {
                              break;
                            }


                           if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config &&
                                           evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)        //when indications are ON
                            {
                               LOG_INFO("**************LED ON ****************\n\r");
                                           ble_data.ok_to_send_htm_indications = true;
                                           ble_data.ult_indications = 1;
                                           gpioLed0SetOn();
                             }



                           if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation &&
                                    evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)         //when there is confirmation from client
                           {
                                          ble_data.indication_in_flight = false;

                            }
                           else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)                 //when indication are OFF
                             {
                                            displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
                                              ble_data.ok_to_send_htm_indications = false;
                                              gpioLed0SetOff();
                                              ble_data.ult_indications = 0;

                             }

                     }


             //BUTTON CHARACTERISTIC
               else if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_button_state)
                {
                      LOG_INFO("IN THE button CHARACTERISTIC\n\r");


                           if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config &&
                                            evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)            //When indications are On
                           {
                                            ble_data.button_enable = 1;
                                            gpioLed1SetOn();

                            }


                      if( evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_confirmation &&                       //when there is confirmation from client
                                   evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_indication)
                          {
                                    ble_data.indication_in_flight = false;
                          }
                      else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_disable)                    // //when indication are OFF
                           {
                          LOG_INFO("IN THE button CHARACTERISTIC  GATT DISABLE\n\r");
                                           ble_data.button_enable =0;
                                           gpioLed1SetOff();
                             }

                }



               //ULTRASONIC CHARACTERISTIC

               else if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_distance_measure &&
                         evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
                       {
                         LOG_INFO("in ultrasonic CHARATCERISTICS\n");

                         if(evt->data.evt_gatt_server_characteristic_status.client_config_flags ==  sl_bt_gatt_server_indication)
                           {
                             ble_data.connectionhandle =  evt->data.evt_gatt_server_characteristic_status.connection;
                             ble_data.ult_indications = 1;
                             gpioLed0SetOn();
                           }

                         else
                           {
                             displayPrintf(DISPLAY_ROW_TEMPVALUE, "%s", "");
                             ble_data.ult_indications = 0;
                             gpioLed0SetOff();
                           }
                       }
                     break;

                     //LIGHT CHARACTERISTICS
                     if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_light_measure &&
                               evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
                             {
                               LOG_INFO("in LIGHT CHARATCERISTICS\n");
                               if(evt->data.evt_gatt_server_characteristic_status.client_config_flags ==  sl_bt_gatt_server_indication)
                                 {
                                   ble_data.connectionhandle =  evt->data.evt_gatt_server_characteristic_status.connection;
                                   ble_data.ult_indications  = 1;
                                   gpioLed0SetOn();
                                 }

                               else
                                 {
                                   ble_data.ult_indications = 0;
                                   gpioLed0SetOff();
                                 }
                             }
                           break;

     case sl_bt_evt_gatt_server_indication_timeout_id:
               //Possible event from calling sl_bt_gatt_server_send_indication() - i.e. we never received
               //a confirmation for a previously transmitted indication.


               // @brief
             /*  This event indicates confirmation from the remote GATT client has not
                * been received within 30 seconds after an indication was sent
                *
                * Furthermore, the stack does not allow GATT transactions over this connection.*/

               //      LOG_INFO("Gatt server indication timeout event");

               break;

   case sl_bt_evt_system_soft_timer_id:


                if(evt->data.evt_system_soft_timer.handle == 1)
                {

                  displayUpdate();

                  if((cbfifo_length(&queue) > 0) && ble_data.indication_in_flight == false)
                    {
                      if((read_queue(&queue, &bufferobj)) == 0)
                        {
                              LOG_INFO("Soft timer ID called\n\r");
                               sc = sl_bt_gatt_server_write_attribute_value(
                                                                 bufferobj.charHandle,
                                                                 0,
                                                                 bufferobj.bufferLength,
                                                                 bufferobj.buffer
                                                             );
                                                                                                   // uint16_t attribute,
                                                                                                   //  uint16_t offset,
                                                                                                    // size_t value_len,
                                                                                                   //  const uint8_t* value

                                  if (sc != SL_STATUS_OK)
                                  {
                                    LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                                   }

                                    sc = sl_bt_gatt_server_send_indication(ble_data.connectionhandle,
                                                                     bufferobj.charHandle,
                                                                     bufferobj.bufferLength,
                                                                     bufferobj.buffer);

                                ble_data.indication_in_flight = true;

                                if (sc != SL_STATUS_OK)
                                  {
                                    LOG_INFO("SOFTTIMER SEND INDICATION HAS SENT ERROR\r\n");
                                    LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                                    ble_data.indication_in_flight = 0;
                                   }

                         }

                    }

                }


               break;

    //Identifier of the confirm_bonding event
    case sl_bt_evt_sm_confirm_bonding_id:

                     LOG_INFO("In confirm bonding\n\r");
                     sc = sl_bt_sm_bonding_confirm(ble_data.connectionhandle, 1);                                         //Bonding request confirmation
                     if (sc != SL_STATUS_OK)
                       {
                         LOG_ERROR("sl_bt_sm_bonding_confirm() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                       }
                     break;



    case sl_bt_evt_sm_bonding_failed_id:


                     LOG_ERROR("BONDING REQUEST IS FAILED : CONNECTION HANDLE - %d\r\n REASON - %d\r\n",
                                evt->data.evt_sm_bonding_failed.connection, evt->data.evt_sm_bonding_failed.reason);        //Bonding failed

                     break;



    case sl_bt_evt_sm_confirm_passkey_id:

                     if(ble_data.connectionhandle == evt->data.evt_sm_confirm_passkey.connection)
                             {
                               displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey %d",
                                              evt->data.evt_sm_confirm_passkey.passkey);
                               displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");


                               ble_data.state = PAIRING;        //PAIRING
                             }
                           break;


    case sl_bt_evt_sm_bonded_id:
                       LOG_INFO("Bonding is done\n\r");
                       displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");                                                         //Bonding is done
                        break;



#else

    case sl_bt_evt_system_boot_id :
      {
        gpioLed0SetOff();
        gpioLed1SetOff();

        LOG_INFO("boot ID\n\r");
        displayInit();

        displayPrintf(DISPLAY_ROW_NAME, "%s", "Client");

        displayPrintf(DISPLAY_ROW_ASSIGNMENT, "%s", "A9");

         // Extract unique ID from BT Address.

        sc = sl_bt_system_get_identity_address(&address, &address_type);

        if (sc != SL_STATUS_OK) {
             LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
            }

                  displayPrintf( DISPLAY_ROW_BTADDR, "%X:%X:%X:%X:%X:%X",
                         address.addr[ 0 ],  address.addr[ 1 ],  address.addr[ 2 ],
                  address.addr[ 3 ],  address.addr[ 4 ], address.addr[ 5 ] );

        displayPrintf(DISPLAY_ROW_CONNECTION,"%s", "Discovering");

      sl_bt_scanner_set_mode(sl_bt_gap_1m_phy,SCAN_PASSIVE);
      sl_bt_scanner_set_timing(sl_bt_gap_1m_phy,80,40);     //50/0.625          //25/.625
      sl_bt_connection_set_default_parameters(60,60,4,660,0,4);                                       //75/1.25,
      sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);

      //Added as part of A9

      sc = sl_bt_sm_configure(SECURITY_FLAG, sl_bt_sm_io_capability_displayyesno);                          //Configure security requirements and I/O capabilities of the system
      if (sc != SL_STATUS_OK) {
                      LOG_ERROR("sl_bt_sm_configure() returned != 0 status=0x%04x", (unsigned int) sc);
                      }


     sc = sl_bt_sm_delete_bondings();                                                                       //Delete all bonding information
     if (sc != SL_STATUS_OK) {
                     LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x", (unsigned int) sc);
                     }
      }



     break;
    case sl_bt_evt_scanner_scan_report_id :
      {
        LOG_INFO("Scanner Report\n\r");

            //if(strcmp)((char *)add.addr,(char*)evt->data.evt_scanner_scan_report.address.addr,6)==0)
        // Parse advertisement packets
         if (evt->data.evt_scanner_scan_report.packet_type == 0)
           {
              // If a thermometer advertisement is found...
                if (find_service_in_advertisement(&(evt->data.evt_scanner_scan_report.data.data[0]),
                                                    evt->data.evt_scanner_scan_report.data.len) != 0)
                   {
                      // then stop scanning for a while
                       sc = sl_bt_scanner_stop();

                       // and connect to that device

                       sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                                  evt->data.evt_scanner_scan_report.address_type,
                                                  sl_bt_gap_1m_phy,
                                                  NULL);
                       if (sc != SL_STATUS_OK) {
                                        LOG_ERROR("sl_bt_connection_open() returned != 0 status=0x%04x", (unsigned int) sc);
                                        }

                       }
               }
      }

      break;

    case sl_bt_evt_connection_opened_id :
      {

        LOG_INFO("Opened ID\n\r");

               displayPrintf( DISPLAY_ROW_TEMPVALUE, "%s"," " );
               displayPrintf( DISPLAY_ROW_CONNECTION, "%s","Connected" );

               displayPrintf( DISPLAY_ROW_BTADDR2,  "%X:%X:%X:%X:%X:%X",
                              add.addr[ 0 ],  add.addr[ 1 ],  add.addr[ 2 ],
                              add.addr[ 3 ],  add.addr[ 4 ], add.addr[ 5 ] );

      }

      break;

    case sl_bt_evt_gatt_procedure_completed_id:


               if(evt->data.evt_gatt_procedure_completed.result == 0)
                 {
                 //do nothing for gatt procedure completed
                 }
               else
                 {

                   LOG_ERROR("sl_bt_evt_gatt_procedure_completed_id() returned != 0 status=0x%04x\r\n",
                              (unsigned int) evt->data.evt_gatt_procedure_completed.result);                                 //increase security in this case


                     if(evt->data.evt_gatt_procedure_completed.result == ((sl_status_t) SL_STATUS_BT_ATT_INSUFFICIENT_ENCRYPTION))
                     {
                       sc = sl_bt_sm_increase_security(ble_data.connectionhandle);
                       if (sc != SL_STATUS_OK)
                         {
                           LOG_ERROR("sl_bt_sm_increase_security() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                         }
                      }
                 }
               break;

    case sl_bt_evt_gatt_service_id :

      LOG_INFO("gatt service\n\r");
      if(evt->data.evt_gatt_service.uuid.data[0] == thermo_service[0] &&
         evt->data.evt_gatt_service.uuid.data[1] == thermo_service[1])                                    //memcmp(evt->data.evt_gatt_service.uuid.data,thermo_service,2) == 0)
      {
               ble_data.serviceHandle[0] = evt->data.evt_gatt_service.service;
       }
      else if(UUID_Compare(evt, SERVICE))
        {
                      ble_data.serviceHandle[1] = evt->data.evt_gatt_service.service;
        }
      //check for ultrasonic service
      else if(memcmp(evt->data.evt_gatt_service.uuid.data, &Ultrasonic_service, 16) == 0)
        {
                 ble_data.UltrasonicServiceHandle = evt->data.evt_gatt_service.service;
         }
      //check for light service
      else if(memcmp(evt->data.evt_gatt_service.uuid.data, &Light_service, 16) == 0)
        {
                 ble_data.lightServiceHandle = evt->data.evt_gatt_service.service;
         }
      else
        {
          //do nothing
        }

      break;

    case sl_bt_evt_gatt_characteristic_id :


      if(evt->data.evt_gatt_characteristic.uuid.data[0] == thermo_char[0] && evt->data.evt_gatt_characteristic.uuid.data[1] == thermo_char[1])                // memcmp(evt->data.evt_gatt_characteristic.uuid.data,thermo_char,2) == 0)
         {
                  ble_data.characteristicHandle[0] = evt->data.evt_gatt_characteristic.characteristic;
                  LOG_INFO("Thermo handle detected\r\n");
         }
       else if( UUID_Compare(evt, CHARACTERISTIC) )
         {
                      ble_data.characteristicHandle[1] = evt->data.evt_gatt_characteristic.characteristic;
                      LOG_INFO("Button handle detected\r\n");
         }
       else if(memcmp(evt->data.evt_gatt_characteristic.uuid.data, &Ultrasonic_char, 16) == 0)
         {
                ble_data.UltrasonicCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;

           }
       else if(memcmp(evt->data.evt_gatt_characteristic.uuid.data, &Light_char, 16) == 0)
         {
                ble_data.lightCharacteristicHandle = evt->data.evt_gatt_characteristic.characteristic;
            }
       else
         {
           //do nothing
         }

            break;


    case sl_bt_evt_connection_closed_id :
      {
        LOG_INFO("Connection Closed\n\r");
        sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);                               //(uint8_t scanning_phy, uint8_t discover_mode);

        displayPrintf(DISPLAY_ROW_TEMPVALUE, " ");
        displayPrintf(DISPLAY_ROW_BTADDR2, " ");
        displayPrintf(DISPLAY_ROW_9, " ");
        displayPrintf(DISPLAY_ROW_CONNECTION,"%s", "Discovering");

        sc = sl_bt_sm_delete_bondings();                                                                       //Delete all bonding information
        if (sc != SL_STATUS_OK) {
                        LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x", (unsigned int) sc);
                        }

        ble_data.state = DELETE;                                                                             //DELETE BONDING

      }
      break;

    case sl_bt_evt_gatt_characteristic_value_id:
      {
        if(evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication &&
                    evt->data.evt_gatt_characteristic_value.characteristic ==ble_data.characteristicHandle[0])
         {
                                 sc = sl_bt_gatt_send_characteristic_confirmation(ble_data.connectionhandle);
                                 if (sc != SL_STATUS_OK)
                                   {
                                     LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() returned != 0 status=0x%04x", (unsigned int) sc);
                                   }
                                 uint8_t * client_temperature = (evt->data.evt_gatt_characteristic_value.value.data);
                                LOG_INFO("Client Temperature = %d\r\n", client_temperature[4]);
                                int32_t temp = FLOAT_TO_INT32 (client_temperature);
                                LOG_INFO("Received Temperature = %d\r\n", (temp));
                             //   displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %d", (temp));
            }

         if((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication || evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_read_response)
                            &&  evt->data.evt_gatt_characteristic_value.characteristic == ble_data.characteristicHandle[1])                  // //  If it is an indication for button state

            {

                if(evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)                    // If it is an indication then only send confirmation
                  {
                      LOG_INFO("entering confirmation button\n\r");
                      sc = sl_bt_gatt_send_characteristic_confirmation(ble_data.connectionhandle);
                      if (sc != SL_STATUS_OK)
                      {
                         LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() returned != 0 status=0x%04x", (unsigned int) sc);
                      }
                  }

                      uint8_t client_btn_state = evt->data.evt_gatt_characteristic_value.value.data[0];

                                 if(client_btn_state == 1)
                                   {
                                     displayPrintf(DISPLAY_ROW_9, "Button Pressed");
                                   }
                                 else if(client_btn_state == 0)
                                   {
                                      displayPrintf(DISPLAY_ROW_9, "Button Released");
                                   }
              }

         //ADDING CASES FOR DISTANCE AND LIGHT
         if(((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)|| (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_read_response))
                   &&  (evt->data.evt_gatt_characteristic_value.characteristic == ble_data.UltrasonicCharacteristicHandle))
           {
               LOG_INFO("ULTRAVIOLET\n\r");
                    if(evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
                    {
                        LOG_INFO("entering confirmation ULTRAVIOLET\n\r");
                        sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
                        if (sc != SL_STATUS_OK)
                         {
                           LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() returned != 0 status=0x%04x", (unsigned int) sc);
                         }
                    }


                    LOG_INFO(" UV value received\r\n");

                    uint8_t uv = evt->data.evt_gatt_characteristic_value.value.data[0];

                    LOG_INFO("Value of distance is = %d\r\n", uv);

                              if (uv > 100){
                                     DetectionSuccess = 0;
                                     displayPrintf(DISPLAY_ROW_8, "  ");
                                     displayPrintf(DISPLAY_ROW_9, "  ");
                                     displayPrintf(DISPLAY_ROW_TEMPVALUE, "No Presence Detected");
                                     gpioLed0SetOff(); }
                                else{
                                    displayPrintf(DISPLAY_ROW_TEMPVALUE, "Detected Presence");
                                    DetectionSuccess = 1;}

                   // displayPrintf(DISPLAY_ROW_TEMPVALUE, "UV = %d", uv);

           }

        if(((evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)|| (evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_read_response))
                   &&  (evt->data.evt_gatt_characteristic_value.characteristic == ble_data.lightCharacteristicHandle))
          {
            LOG_INFO("LIGHT\n\r");

                    if(evt->data.evt_gatt_characteristic_value.att_opcode == sl_bt_gatt_handle_value_indication)
                      {
                        LOG_INFO(" Confirmation of light characteristic\r\n");
                        sc = sl_bt_gatt_send_characteristic_confirmation(evt->data.evt_gatt_characteristic_value.connection);
                      }


                    uint8_t   light = evt->data.evt_gatt_characteristic_value.value.data[0];

                    LOG_INFO("Value of Light received is = %d\r\n", light);

                                if(DetectionSuccess){
                                    if (light < 100)
                                    {
                                        gpioLed0SetOn();
                                        displayPrintf(DISPLAY_ROW_9, "LED ON");
                                        displayPrintf(DISPLAY_ROW_8, "Night");
                                    }
                                    else if (light > 100)
                                    {
                                        gpioLed0SetOff();
                                        displayPrintf(DISPLAY_ROW_9, "LED OFF");
                                        displayPrintf(DISPLAY_ROW_8, "Morning");
                                    } }
                                else
                                {
                                    displayPrintf(DISPLAY_ROW_8, "  ");
                                    displayPrintf(DISPLAY_ROW_9, "  ");
                                }
          }


   }
      break;

    case sl_bt_evt_system_soft_timer_id:
      if (evt->data.evt_system_soft_timer.handle == 0)
         {
            displayUpdate();
         }
      break;

      //below event cases as part of A9



      case sl_bt_evt_system_external_signal_id:


              if(evt->data.evt_system_external_signal.extsignals == CHECK_BUTTON_EVENT_PB1)
                {
                  LOG_INFO("check button PB1\n\r");


                  if(GPIO_PinInGet(gpioPortF, PB0_pin) == 1)             //PB0 is released
                    {
                      LOG_INFO("PB0 pressed\n\r");
                      sc = sl_bt_gatt_read_characteristic_value(ble_data.connectionhandle, ble_data.characteristicHandle[1]);
                      if (sc != SL_STATUS_OK)
                        {
                          LOG_ERROR("sl_bt_gatt_read_characteristic_value() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                        }
                    }

                  else if(GPIO_PinInGet(gpioPortF, PB0_pin) == 0)            //PB0 is pressed
                    {

                      ble_data.button_enable ^= 2;                          //Flip indications

                      sc = sl_bt_gatt_set_characteristic_notification(ble_data.connectionhandle,
                                                                      ble_data.characteristicHandle[1],
                                                                      ble_data.button_enable);
                      if (sc != SL_STATUS_OK)
                        {
                          LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid() returned != 0 status=0x%04x", (unsigned int) sc);
                        }
                    }
                }
                 else if(evt->data.evt_system_external_signal.extsignals == CHECK_BUTTON_EVENT_PB0)
                    {
                           LOG_INFO("check button PB0\n\r");

                    if(ble_data.state == PAIRING)                                   //pairing
                    {
                                LOG_INFO("PAIRING IN PPROGRESS\n\r");
                                displayPrintf(DISPLAY_ROW_PASSKEY, " ");
                               displayPrintf(DISPLAY_ROW_ACTION, " ");

                               sc = sl_bt_sm_passkey_confirm(ble_data.connectionhandle, 1);

                        if (sc != SL_STATUS_OK)
                        {
                          LOG_ERROR("sl_bt_sm_passkey_confirm() returned != 0 status=0x%04x\r\n", (unsigned int) sc);
                        }

                       ble_data.state = BONDING;
                    }
                }
              break;

      case sl_bt_evt_sm_bonding_failed_id:
        {
              ble_data.state = DELETE;

              LOG_ERROR("BONDING REQUEST IS FAILED : CONNECTION HANDLE - %d\r\n REASON - %d\r\n",
                                           evt->data.evt_sm_bonding_failed.connection, evt->data.evt_sm_bonding_failed.reason);        //Bonding failed
        }
              break;

      case sl_bt_evt_sm_confirm_passkey_id:

              if(ble_data.connectionhandle == evt->data.evt_sm_confirm_passkey.connection)
                {
                  displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey %d", evt->data.evt_sm_confirm_passkey.passkey);
                  displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");

                  ble_data.state = PAIRING;
                }
              break;

      case sl_bt_evt_sm_bonded_id:

              displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
              break;


    default:
      {
          break;
      }


#endif
  //}
}

}
void server_indication(uint32_t temperature)
{
  sl_status_t sc;
  uint8_t buffer[4]={temperature};

  uint8_t htm_temperature_buffer[5];
  uint8_t flags = 0x00;
  uint8_t *p = htm_temperature_buffer;
  uint32_t htm_temperature_flt;

  UINT8_TO_BITSTREAM(p,flags);


  htm_temperature_flt = UINT32_TO_FLOAT(temperature*1000, -3);
  // Convert temperature to bitstream and place it in the htm_temperature_buffer
  UINT32_TO_BITSTREAM(p, htm_temperature_flt);



  sc = sl_bt_gatt_server_write_attribute_value(gattdb_temperature_measurement,
                                                         0,
                                                         4,
                                                         &buffer[0]); // in IEEE-11073 format);

  if(sc!=0)
    {
      LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
    }

  ble_data_struct_t *bleDataPtr = getBleDataPtr();

  if (bleDataPtr->connection_open==true && ble_data.ok_to_send_htm_indications==true)
  {

            sc = sl_bt_gatt_server_send_indication(
                                         bleDataPtr->connectionhandle,
                                         gattdb_temperature_measurement, // handle from gatt_db.h
                                         5,
                                         &htm_temperature_buffer[0] // in IEEE-11073 format
                                         );



            displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %d", temperature);

      if (sc != SL_STATUS_OK)
      {
          LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x", (unsigned int) sc);
      }
      else
      {
          LOG_INFO("Status okay\n\r");

    //   ble_data.indication_in_flight=true;
       }
   }


}

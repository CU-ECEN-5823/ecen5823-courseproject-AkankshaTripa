/*
 * References: All the initialization has been taken form lecture slides
 *              Function handle_ble_event : reference from SOC thermomemter project
 *
 * */
#include "ble.h"
#include <stdbool.h>
#include "em_common.h"
#include "sl_status.h"
#include "app_log.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "sl_bt_api.h"
#include "sl_bt_api_compatibility.h"
#include "i2c.h"
//#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#include "src/lcd.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// BLE private data
ble_data_struct_t ble_data;


// this is the declaration
// function that returns a pointer to the
// BLE private data
ble_data_struct_t* getBleDataPtr() {
 return (&ble_data);
} // getBleDataPtr()


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


  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header))
  {
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

      displayInit();

      displayPrintf(DISPLAY_ROW_NAME, "%s", "Server");

      displayPrintf(DISPLAY_ROW_ASSIGNMENT, "%s", "A6");

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);

      if (sc != SL_STATUS_OK) {
             LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
             }

      displayPrintf( DISPLAY_ROW_BTADDR, "%X:%X:%X:%X:%X:%X",
             address.addr[ 0 ],  address.addr[ 1 ],  address.addr[ 2 ],
      address.addr[ 3 ],  address.addr[ 4 ], address.addr[ 5 ] );


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


      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
                      advertising_set_handle, // advertising set handle
                      400, // min. adv. interval (milliseconds * 1.6)=250*1.6
                      400, // max. adv. interval (milliseconds * 1.6)
                      0,   // adv. duration
                      0);  // max. num. adv. events

      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);

      LOG_INFO("Started advertising first\n\r");

      displayPrintf(DISPLAY_ROW_CONNECTION,"%s", "Advertising");

      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:

      LOG_INFO("Connection opened\n\r");

      sl_bt_advertiser_stop(advertising_set_handle);

      ble_data.connectionopenhandle =evt->data.evt_connection_opened.connection;
      ble_data.connection_open=true;

      if(ble_data.connection_open==true)
        {
          displayPrintf( DISPLAY_ROW_TEMPVALUE, "%s"," " );
          displayPrintf( DISPLAY_ROW_CONNECTION, "%s","Connected" );
        }

      sl_bt_connection_set_parameters(ble_data.connectionopenhandle,
                                      60,                     //60*1.25=75ms    //Time = Value x 1.25 ms
                                      60,
                                      4,         // 4 * 75ms intervals = 300ms for slave latency
                                      660,
                                      0,
                                      0);      // timeout  =  ((1 + slave latency) * (connection_interval * 2))

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


      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);

      if(ble_data.connection_open==false)
        {
                 displayPrintf( DISPLAY_ROW_TEMPVALUE, "%s"," " );
                 displayPrintf(DISPLAY_ROW_CONNECTION, "%s","Advertising");

        }

      LOG_INFO("Started advertising\n\r");

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


          break;

    // Events only for Slaves/Servers
        case sl_bt_evt_gatt_server_characteristic_status_id:

          LOG_INFO("status_flags  = %d\r", evt->data.evt_gatt_server_characteristic_status.status_flags);
          LOG_INFO("client_config_flags  = %d\r", evt->data.evt_gatt_server_characteristic_status.client_config_flags);
          LOG_INFO("client_config  = %d\r", evt->data.evt_gatt_server_characteristic_status.client_config);

          if (evt->data.evt_gatt_server_characteristic_status.characteristic
              == gattdb_temperature_measurement)
          {


           if (evt->data.evt_gatt_server_characteristic_status.status_flags
                != sl_bt_gatt_server_client_config)
                     {
                         break;
                    }

                if(evt->data.evt_gatt_server_characteristic_status.client_config_flags
                             == sl_bt_gatt_indication)                             //gatt_server_ind
                 {
                 // Indications have been turned ON
                   LOG_INFO("temperature indication on\r");
                   ble_data.ok_to_send_htm_indications = true;
                 }
                 else
                 {
                    // Indications have been turned OFF
                    LOG_INFO("temperature indication off\r");
                    ble_data.ok_to_send_htm_indications = false;
                    displayPrintf( DISPLAY_ROW_TEMPVALUE, "%s"," " );


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

          displayUpdate();

          break;




    // -------------------------------
    // Default event handler.
    default:
      break;
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
                                         bleDataPtr->connectionopenhandle,
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

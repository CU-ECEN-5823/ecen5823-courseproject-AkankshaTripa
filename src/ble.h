/***********************************************************************************************************************************

*                     Project Name : Home Automation System
                      File Name    : ble.h
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

#ifndef BLE_H
#define BLE_H

#include "stdbool.h"
#include "stdint.h"
#include "sl_bgapi.h"
#include "sl_bt_api.h"
#define CAPACITY 16

#define UINT8_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); }

#define UINT32_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
 *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }

#define UINT32_TO_FLOAT(m, e) (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))


// Health Thermometer service UUID defined by Bluetooth SIG
static const uint8_t thermo_service[2] = { 0x09, 0x18 };
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
static const uint8_t thermo_char[2] = { 0x1c, 0x2a };

static const uint8_t Pushbutton_service[16] = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
                                          0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00, 0x00};

static const uint8_t Pushbutton_characteristics[16] = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
                                           0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00, 0x00};

static const uint8_t Ultrasonic_service[16] = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
                                           0x3e, 0x43, 0xc8, 0x38, 0x03, 0x00, 0x00, 0x00};

static const uint8_t Ultrasonic_char[16] = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
                                          0x3e, 0x43, 0xc8, 0x38, 0x04, 0x00, 0x00, 0x00};

static const uint8_t Light_service[16] = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
                                          0x3e, 0x43, 0xc8, 0x38, 0x05, 0x00, 0x00, 0x00};

static const uint8_t Light_char[16] = {0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87,
                                          0x3e, 0x43, 0xc8, 0x38, 0x06, 0x00, 0x00, 0x00};


// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
 // values that are common to servers and clients
 bd_addr myAddress;

 // values unique for server
 // The advertising set handle allocated from Bluetooth stack.
 //uint8_t advertisingSetHandle;

  bool connection_open;                  // true when in an open connection
  bool ok_to_send_htm_indications;       // true when client enabled indications
  bool indication_in_flight;              // true when an indication is in-flightt

 // bool indication;
  uint8_t connectionhandle;
  uint8_t button_enable;

  uint32_t serviceHandle[2];
  uint16_t characteristicHandle[2];

   int state;                         //1-pairing, 2-bonding 0-deleteBonding

   bool volatile ult_indications;    //variable for both light and uv indications

   uint32_t UltrasonicServiceHandle;
   uint16_t UltrasonicCharacteristicHandle;

   uint32_t lightServiceHandle;
   uint16_t lightCharacteristicHandle;

 // values unique for client
} ble_data_struct_t;

typedef struct{
  uint16_t charHandle;      // Characteristic handle
  size_t   bufferLength;    // Length of buffer in bytes to send
  uint8_t  buffer[5];       // Actual buffer size
}buffer_t;

typedef struct
{
buffer_t Data[CAPACITY];          // Buffer
uint8_t wptr;                    // Write pointer
uint8_t rptr;                    // Read pointer
uint8_t isFull;                   // Flag for checking full condition
}buff;

void server_indication(uint32_t temp);


void handle_ble_event(sl_bt_msg_t *evt);

// function prototypes
ble_data_struct_t* getBleDataPtr(void);



/*******************CBFIFO*******************************************/

void initialise_cbfifo(buff *cb);
uint32_t nextPtr(uint32_t ptr);
int write_queue(buff* Cbfifo, buffer_t *buf);
int read_queue(buff* Cbfifo, buffer_t *buf);
size_t cbfifo_length(buff* Cbfifo);



#endif //BLE_H

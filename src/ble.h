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

// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
 // values that are common to servers and clients
 bd_addr myAddress;

 // values unique for server
 // The advertising set handle allocated from Bluetooth stack.
 uint8_t advertisingSetHandle;

  bool connection_open; // true when in an open connection
  bool ok_to_send_htm_indications; // true when client enabled indications
  bool indication_in_flight; // true when an indication is in-flightt

 // bool indication;
  uint8_t connectionopenhandle;

  uint32_t serviceHandle;
  uint16_t characteristicHandle;
  uint8_t discoveryEvt;

  int bonding_state;

 // values unique for client
} ble_data_struct_t;

typedef struct{
  uint16_t charHandle;      /* Characteristic handle from GATTdb */
  size_t   bufferLength;    /* Length of buffer in bytes to send */
  uint8_t  buffer[5];       /* Actual buffer size. 5 bytes for htm and 2 for btn */
}buffer_t;

typedef struct
{
buffer_t Data[CAPACITY];          /* Buffer */
uint8_t wptr;                    /* Write Location (where to write next) */
uint8_t rptr;                    /* Read Location (where to read from next) */
uint8_t isFull;                   /* Flag to indicate buffer full */
}buff;


void initialise_cbfifo(buff *cb);

void server_indication(uint32_t temp);


void handle_ble_event(sl_bt_msg_t *evt);

// function prototypes
ble_data_struct_t* getBleDataPtr(void);



/*******************CBFIFO*******************************************/

int write_queue(buff* Cbfifo, buffer_t *buf);
int read_queue(buff* Cbfifo, buffer_t *buf);
size_t cbfifo_length(buff* Cbfifo);



#endif //BLE_H

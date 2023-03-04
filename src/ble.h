#ifndef BLE_H
#define BLE_H

#include "stdbool.h"
#include "stdint.h"
#include "sl_bgapi.h"
#include "sl_bt_api.h"

#define UINT8_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); }

#define UINT32_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
 *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }

#define UINT32_TO_FLOAT(m, e) (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

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

 // values unique for client
} ble_data_struct_t;

void server_indication(uint32_t temp);


void handle_ble_event(sl_bt_msg_t *evt);

// function prototypes
ble_data_struct_t* getBleDataPtr(void);

#endif //BLE_H

/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Date:        08-07-2021
 * Author:      Dave Sluiter
 * Description: This code was created by the Silicon Labs application wizard
 *              and started as "Bluetooth - SoC Empty".
 *              It is to be used only for ECEN 5823 "IoT Embedded Firmware".
 *              The MSLA referenced above is in effect.
 *
 ******************************************************************************/


// *************************************************
// Students: It is OK to modify this file.
//           Make edits appropriate for each
//           assignment.
// *************************************************


#ifndef APP_H
#define APP_H


#include "em_common.h"
#include "app_assert.h"
#include "src/ble_device_type.h"
#include "src/gpio.h"
#include "src/lcd.h"
#include "src/timers.h"
#include "src/i2c.h"
#include "src/oscillators.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "sl_status.h"


#define EM0 0   //For Run Mode - Energy Mode 0
#define EM1 1   // For Sleep Mode - Energy Mode 1
#define EM2 2   // For Deep Sleep Mode - Energy Mode 2
#define EM3 3   // For Stop Mode - Energy Mode 3

#define LOWEST_ENERGY_MODE EM2

#define TIME_TO_WAIT 80000 // 100 msec wait time adjusted with tolerance of 20msec

#define TIME_TO_TRANSFER 11000  // 11 msec transfer time converted to microsec.

#define LETIMER_PERIOD_MS 3000  // 3 SEC

#if (LOWEST_ENERGY_MODE == EM3)
 #define ACTUAL_CLK_FREQ  1000    // Frequency of OSC = 1kHz & prescaler = 1
#else
  #define ACTUAL_CLK_FREQ  8192  // Frequency of OSC = 32.768kHz & prescaler = 4
#endif
/*From reference of the Lecture video and presentations*/
#define VALUE_TO_LOAD (LETIMER_PERIOD_MS*ACTUAL_CLK_FREQ)/1000




/**************************************************************************
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

#endif // APP_H */

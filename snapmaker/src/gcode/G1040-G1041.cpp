/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../common/config.h"
#include "../service/system.h"
#include "../module/toolhead_cnc.h"
#include "../module/toolhead_laser.h"

// marlin headers
#include  "src/gcode/gcode.h"

#include "../common/protocol_sstp.h"
#include "../service/bed_level.h"
#include "../common/debug.h"
#include "../module/toolhead_3dp.h"
#include "../../../Marlin/src/module/configuration_store.h"

#if (MOTHERBOARD == BOARD_SNAPMAKER_2_0)

void GcodeSuite::G1040() {
  planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

  SSTP_Event_t event;
  bool seen_p, seen_c, seen_a;  // performance, cross, alignment

  seen_p = parser.seen("P");
  if (seen_p) {
    event.op_code = 0x11;
    event.data = NULL;
    event.length = 0;
    event.id = 9;

    levelservice.DoXCalibration(event);
  }

  seen_c = parser.seen("C");
  seen_a = parser.seen("A");
  if (seen_c && seen_a) {
    uint8_t buff[2];
    buff[0] = (uint8_t)parser.byteval('C', (uint8_t)0);  // cross 0 scale lines
    buff[1] = (uint8_t)parser.intval('A', (uint8_t)0);   // sub alignment line number

    event.op_code = 0x12;
    event.data = buff;
    event.length = 2;
    event.id = 9;
    levelservice.ApplyXCalibration(event);
  }
}

void GcodeSuite::G1041() {
  planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

  SSTP_Event_t event;
  bool seen_p, seen_c, seen_a;  // performance, cross, alignment

  seen_p = parser.seen("P");
  if (seen_p) {
    event.op_code = 0x12;
    event.data = NULL;
    event.length = 0;
    event.id = 9;

    levelservice.DoYCalibration(event);
  }

  seen_c = parser.seen("C");
  seen_a = parser.seen("A");
  if (seen_c && seen_a) {
    uint8_t buff[2];
    buff[0] = (uint8_t)parser.byteval('C', (uint8_t)0);  // cross 0 scale lines
    buff[1] = (uint8_t)parser.intval('A', (uint8_t)0);   // sub alignment line number

    event.op_code = 0x13;
    event.data = buff;
    event.length = 2;
    event.id = 9;
    levelservice.ApplyYCalibration(event);
  }
}

#endif // MOTHERBOARD == BOARD_SNAPMAKER_2_0
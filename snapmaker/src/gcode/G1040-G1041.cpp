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
  bool seen_l, seen_r, seen_e, seen_c, seen_a;  // performance, cross, alignment

  seen_l = parser.seen("L");
  seen_r = parser.seen("R");
  seen_e = parser.seen("E");
  if (seen_l && seen_r && seen_e) {
    int16_t extruder0_temp = (int16_t)parser.ushortval('L', (int16_t)205);
    int16_t extruder1_temp = (int16_t)parser.ushortval('R', (int16_t)205);
    int16_t bed_temp = 50;
    float e_factor = (float)parser.floatval('E', (float)E_MOVES_FACTOR);
    uint8_t buf[8];
    uint8_t index;

    buf[index++] = extruder0_temp >> 8;
    buf[index++] = extruder0_temp & 0xff;
    buf[index++] = extruder1_temp >> 8;
    buf[index++] = extruder1_temp & 0xff;
    buf[index++] = bed_temp >> 8;
    buf[index++] = bed_temp & 0xff;
    buf[index++] = ((uint8_t *)&e_factor)[0];
    buf[index++] = ((uint8_t *)&e_factor)[1];
    buf[index++] = ((uint8_t *)&e_factor)[2];
    buf[index++] = ((uint8_t *)&e_factor)[3];

    event.op_code = 0x11;
    event.data = buf;
    event.length = index;
    event.id = 9;

    levelservice.DoXCalibration(event);
  }

  seen_c = parser.seen("C");
  seen_a = parser.seen("A");
  if (seen_c && seen_a) {
    uint8_t buf[2];
    buf[0] = (uint8_t)parser.byteval('C', (uint8_t)0);  // cross 0 scale lines
    buf[1] = (uint8_t)parser.intval('A', (uint8_t)0);   // sub alignment line number

    event.op_code = 0x12;
    event.data = buf;
    event.length = 2;
    event.id = 9;
    levelservice.ApplyXCalibration(event);
  }
}

void GcodeSuite::G1041() {
  planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

  SSTP_Event_t event;
  bool seen_l, seen_r, seen_e, seen_c, seen_a;  // performance, cross, alignment

  seen_l = parser.seen("L");
  seen_r = parser.seen("R");
  seen_e = parser.seen("E");
  if (seen_l && seen_r && seen_e) {
    int16_t extruder0_temp = (int16_t)parser.ushortval('L', (int16_t)205);
    int16_t extruder1_temp = (int16_t)parser.ushortval('R', (int16_t)205);
    int16_t bed_temp = 50;
    float e_factor = (float)parser.floatval('E', (float)E_MOVES_FACTOR);
    uint8_t buf[8];
    uint8_t index;

    buf[index++] = extruder0_temp >> 8;
    buf[index++] = extruder0_temp & 0xff;
    buf[index++] = extruder1_temp >> 8;
    buf[index++] = extruder1_temp & 0xff;
    buf[index++] = bed_temp >> 8;
    buf[index++] = bed_temp & 0xff;
    buf[index++] = ((uint8_t *)&e_factor)[0];
    buf[index++] = ((uint8_t *)&e_factor)[1];
    buf[index++] = ((uint8_t *)&e_factor)[2];
    buf[index++] = ((uint8_t *)&e_factor)[3];

    event.op_code = 0x12;
    event.data = buf;
    event.length = index;
    event.id = 9;

    levelservice.DoYCalibration(event);
  }

  seen_c = parser.seen("C");
  seen_a = parser.seen("A");
  if (seen_c && seen_a) {
    uint8_t buf[2];
    buf[0] = (uint8_t)parser.byteval('C', (uint8_t)0);  // cross 0 scale lines
    buf[1] = (uint8_t)parser.intval('A', (uint8_t)0);   // sub alignment line number

    event.op_code = 0x13;
    event.data = buf;
    event.length = 2;
    event.id = 9;
    levelservice.ApplyYCalibration(event);
  }
}

#endif // MOTHERBOARD == BOARD_SNAPMAKER_2_0
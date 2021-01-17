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
  bool seen_a, seen_c;

  seen_a = parser.seen("A");
  if (seen_a) {
    int16_t cross_0_scale_lines = (int16_t)parser.intval('A', (int16_t)0);
    event.op_code = 0x11;
    event.data = (uint8_t *)&cross_0_scale_lines;
    event.length = sizeof cross_0_scale_lines;
    event.id = 9;

    levelservice.DoXCalibration(event);
  }

  seen_c = parser.seen("C");
  if (seen_c) {
    // save x direction hotend_offset
    uint8_t sub_alignment_line_number = (uint8_t)parser.intval('C', (uint8_t)0);
    hotend_offset[X_AXIS][TOOLHEAD_3DP_EXTRUDER1] += sub_alignment_line_number * SCALE_MEASUREMENT_ACCURACY;
    settings.save();
  }
}

void GcodeSuite::G1041() {
  planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

  SSTP_Event_t event;
  bool seen_a, seen_c;

  seen_a = parser.seen("A");
  if (seen_a) {
    int16_t cross_0_scale_lines = (int16_t)parser.intval('A', (int16_t)0);
    event.op_code = 0x13;
    event.data = (uint8_t *)&cross_0_scale_lines;
    event.length = sizeof cross_0_scale_lines;
    event.id = 9;

    levelservice.DoYCalibration(event);
  }

  seen_c = parser.seen("C");
  if (seen_c) {
    // save x direction hotend_offset
    uint8_t sub_alignment_line_number = (uint8_t)parser.intval('C', (uint8_t)0);
    hotend_offset[Y_AXIS][TOOLHEAD_3DP_EXTRUDER1] += sub_alignment_line_number * SCALE_MEASUREMENT_ACCURACY;
    settings.save();
  }
}

#endif // MOTHERBOARD == BOARD_SNAPMAKER_2_0
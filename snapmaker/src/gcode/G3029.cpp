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

#if (MOTHERBOARD == BOARD_SNAPMAKER_2_0)


/**
 * M3: Spindle Clockwise
 * M4: Spindle Counter-clockwise
 *
 *  S0 turns off spindle.
 *
 *  If no speed PWM output is defined then M3/M4 just turns it on.
 *
 *  At least 12.8KHz (50Hz * 256) is needed for spindle PWM.
 *  Hardware PWM is required. ISRs are too slow.
 *
 * NOTE: WGM for timers 3, 4, and 5 must be either Mode 1 or Mode 5.
 *       No other settings give a PWM signal that goes from 0 to 5 volts.
 *
 *       The system automatically sets WGM to Mode 1, so no special
 *       initialization is needed.
 *
 *       WGM bits for timer 2 are automatically set by the system to
 *       Mode 1. This produces an acceptable 0 to 5 volt signal.
 *       No special initialization is needed.
 *
 * NOTE: A minimum PWM frequency of 50 Hz is needed. All prescaler
 *       factors for timers 2, 3, 4, and 5 are acceptable.
 *
 *  SPINDLE_LASER_ENABLE_PIN needs an external pullup or it may power on
 *  the spindle/laser during power-up or when connecting to the host
 *  (usually goes through a reset which sets all I/O pins to tri-state)
 *
 *  PWM duty cycle goes from 0 (off) to 255 (always on).
 */

void GcodeSuite::G3029() {
  planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

  uint8_t level_points = 3;
  uint8_t level_point_index = 1;
  SSTP_Event_t event;
  bool seen_p, seen_l, seen_t, seen_r, seen_f;

  seen_p = parser.seen("P");
  if(seen_p) {
    level_points = (uint8_t)parser.byteval('P', (uint16_t)3);

    event.op_code = 4;
    event.data = &level_points;
    event.length = 1;
    event.id = 9;
    levelservice.DoManualLeveling(event);

    goto EXIT;
  }

  seen_l = parser.seen("L");
  if (seen_l) {
    level_point_index = (uint8_t)parser.byteval('L', (uint8_t)1);

    event.op_code = 5;
    event.data = &level_point_index;
    event.length = 1;
    event.id = 9;
    levelservice.SetManualLevelingPoint(event);

    goto EXIT;
  }

  seen_t = parser.seen("T");
  if (seen_t) {
    event.op_code = 0x11;
    event.data = NULL;
    event.length = 0;
    event.id = 9; 
    levelservice.GetCurrentPointZValue(event);

    process_cmd_imd("T1");
  }

  seen_r = parser.seen("R");
  if (seen_r) {
    level_point_index = (uint8_t)parser.byteval('R', (uint8_t)1);

    event.op_code = 5;
    event.data = &level_point_index;
    event.length = 1;
    event.id = 9;
    levelservice.SetManualLevelingPoint(event);

    goto EXIT;
  }

  seen_f = parser.seen("F");
  if (seen_f) {
    event.op_code = 7;
    event.data = NULL;
    event.length = 0;
    event.id = 9;
    levelservice.SaveAndExitLeveling(event);

    goto EXIT;
  }

EXIT:
  return;
}

#endif // MOTHERBOARD == BOARD_SNAPMAKER_2_0
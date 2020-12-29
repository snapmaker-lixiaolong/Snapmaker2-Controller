/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 *
 */

#include <src/gcode/gcode.h>
#include <src/feature/bedlevel/abl/abl.h>
#include <src/module/planner.h>
#include <src/module/temperature.h>
#include <src/feature/bedlevel/bedlevel.h>
#include <src/module/endstops.h>
#include <src/module/configuration_store.h>
#include <src/module/motion.h>

#include "../../../../../snapmaker/src/snapmaker.h"

/**
 * G29.cpp - Auto Bed Leveling
 */


extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;
extern uint32_t ABL_GRID_POINTS_VIRT_X;
extern uint32_t ABL_GRID_POINTS_VIRT_Y;
extern uint32_t ABL_TEMP_POINTS_X;
extern uint32_t ABL_TEMP_POINTS_Y;

extern float nozzle_height_probed;
/**
 * G1029
 *
 * GCode implementation of SM2 auto bed leveling protocol.
 *
 *  P[Size]
 *              set the size of GRID
 *
 *  A
 *              start auto probing
 *
 *  S
 *              tuning and saving the offset
 *              Will move to center point first
 *
 *  W
 *      I[X_index]
 *      J[Y_index]
 *
 *              Move to the MESH(i), MESH(j)
 *              1. After G28, enable bed leveling feature,
 *                  We can use this utility to quickly, verify specific location
 *              2. TODO, implement set z value to allow manual probing.
 *  D[delta]
 *
 *          Z axis, move z-offset
 *          delta > 0 => we raise the reference point
 */
void GcodeSuite::G1029() {
  SSTP_Event_t event;
  uint8_t level_points = 3;
  uint8_t level_point_index = 1;

  const bool seen_p = parser.seenval('P');
  if (seen_p) {
    int size = parser.value_int();
    if (size < 0 || size > GRID_MAX_NUM) {
      SERIAL_ECHOLNPAIR("Invalid grid size , maximum: ", GRID_MAX_NUM);
      return;
    }
    set_bed_leveling_enabled(false);
    GRID_MAX_POINTS_X = size;
    GRID_MAX_POINTS_Y = size;

    bilinear_grid_manual();
    // for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++) {
    //   for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
    //     z_values[x][y] = DEFAUT_LEVELING_HEIGHT;
    //     #if ENABLED(EXTENSIBLE_UI)
    //       ExtUI::onMeshUpdate(x, y, 0);
    //     #endif
    //   }
    // }

    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++) {
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
        z_values[x][y] = DEFAUT_LEVELING_HEIGHT;
      }
    }

    ABL_GRID_POINTS_VIRT_X = (GRID_MAX_POINTS_X - 1) * (BILINEAR_SUBDIVISIONS) + 1;
    ABL_GRID_POINTS_VIRT_Y = (GRID_MAX_POINTS_Y - 1) * (BILINEAR_SUBDIVISIONS) + 1;
    ABL_TEMP_POINTS_X = (GRID_MAX_POINTS_X + 2);
    ABL_TEMP_POINTS_Y = (GRID_MAX_POINTS_Y + 2);

    bed_level_virt_interpolate();

    set_bed_leveling_enabled(true);
    SERIAL_ECHOLNPAIR("Set grid size : ", size);
    return;
  }

  const bool seen_c = parser.seen("C");
  if (seen_c) {
    int32_t z_offset = (int32_t)parser.intval('C', (int32_t)0);
    uint8_t buff[5];
    buff[0] = 4;
    buff[1] = ((uint8_t *)&z_offset)[3];
    buff[2] = ((uint8_t *)&z_offset)[2];
    buff[3] = ((uint8_t *)&z_offset)[1];
    buff[4] = ((uint8_t *)&z_offset)[0];

    event.op_code = 0x0f;
    event.data = buff;
    event.length = 5;
    event.id = 9;
    systemservice.ChangeRuntimeEnv(event);
  }

  const bool seen_a = parser.seen("A");
  if (seen_a) {

    thermalManager.disable_all_heaters();
    process_cmd_imd("G28");
    set_bed_leveling_enabled(false);

    // Set the Z max feedrate to 50mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = 40;

    endstops.enable_z_probe(true);
    auto_probing(false, false);
    endstops.enable_z_probe(false);

    // Recover the Z max feedrate to 20mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = 30;
    return;
  }

  const bool seen_s = parser.seen("S");
  if (seen_s) {
    uint8_t opt_s = (uint8_t)parser.byteval('S', (uint8_t)0);
    if (opt_s == 0) {
      compensate_offset();
    }
    else if (opt_s == 1) {
      // don't need to compensate, because this has already been done
      hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = hotend_offset_z_temp;
    }
    else {
      if (nozzle_height_probed <= 0 || nozzle_height_probed > MAX_NOZZLE_HEIGHT_PROBED) {
        LOG_E("invalid nozzle height after level: %.2f", nozzle_height_probed);
        return;
      }
      else
        compensate_offset(nozzle_height_probed);
    }

    bed_level_virt_interpolate();

    // only save data in flash after adjusting z offset
    settings.save();
    return;
  }

  const bool seen_w = parser.seen('W');
  if (seen_w) {
    uint8_t  i = parser.byteval('I', GRID_MAX_POINTS_X / 2);
    uint8_t  j = parser.byteval('J', GRID_MAX_POINTS_Y / 2);

    do_blocking_move_to_logical_xy(_GET_MESH_X(i), _GET_MESH_Y(j), 50);
    do_blocking_move_to_logical_z(13, 50);
    return;
  }

  const bool seen_d = parser.seenval('D');
  if (seen_d) {
    float delta = -parser.value_float();
    if (delta > 1) {
      SERIAL_ECHOLNPAIR("Error, it should be less than 1mm", delta);
    } else {
      set_bed_leveling_enabled(false);
      sync_plan_position();

      compensate_offset(delta);
      bed_level_virt_interpolate();

      set_bed_leveling_enabled(true);
    }
    return;
  }

  const bool seen_b = parser.seen('B');
  if (seen_b) {
    planner.synchronize();   // wait until previous movement commands (G0/G0/G2/G3) have completed before playing with the spindle

    level_points = (uint8_t)parser.byteval('B', (uint8_t)3);

    event.op_code = 2;
    event.data = &level_points;
    event.length = 1;
    event.id = 9;

    levelservice.DoAutoLeveling(event);
    return ;
  }

  const bool seen_m = parser.seen('M');
  if (seen_m) {
    level_points = (uint8_t)parser.byteval('M', (uint16_t)3);

    event.op_code = 4;
    event.data = &level_points;
    event.length = 1;
    event.id = 9;
    levelservice.DoManualLeveling(event);
  }

  const bool seen_n = parser.seen('N');
  if (seen_n) {
    level_point_index = (uint8_t)parser.byteval('N', (uint8_t)1);

    event.op_code = 5;
    event.data = &level_point_index;
    event.length = 1;
    event.id = 9;
    levelservice.SetManualLevelingPoint(event);
  }

  const bool seen_t = parser.seen('T');
  if (seen_t) {
    event.op_code = 0x11;
    event.data = NULL;
    event.length = 0;
    event.id = 9;
    levelservice.SwitchToExtruder1ForBedLevel(event);
  }

  const bool seen_f = parser.seen('F');
  if (seen_f) {
    event.op_code = 7;
    event.data = NULL;
    event.length = 0;
    event.id = 9;
    levelservice.SaveAndExitLeveling(event);
  }

  const bool seen_l = parser.seen('L');
  if (seen_l) {
    float switch_stroke_extruder0_new = (float)parser.floatval('L', (float)0);
    compensate_offset(switch_stroke_extruder0_new - switch_stroke_extruder0);
    switch_stroke_extruder0 = switch_stroke_extruder0_new;

    event.op_code = 7;
    event.data = NULL;
    event.length = 0;
    event.id = 9;
    levelservice.SaveAndExitLeveling(event);
  }

  const bool seen_r = parser.seen('R');
  if (seen_r) {
    float switch_stroke_extruder1_new = (float)parser.floatval('R', (float)0);
    hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] + switch_stroke_extruder1 - switch_stroke_extruder1_new;
    switch_stroke_extruder1 = switch_stroke_extruder1_new;
  }
}

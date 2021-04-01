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

#include "bed_level.h"
#include "../gcode/M1028.h"

#include "src/Marlin.h"
#include "src/module/planner.h"
#include "src/module/endstops.h"
#include "src/module/temperature.h"
#include "src/module/configuration_store.h"

#include "src/gcode/gcode.h"
#include "src/gcode/parser.h"

#include "src/feature/bedlevel/abl/abl.h"
#include "src/feature/bedlevel/bedlevel.h"

#include "src/module/tool_change.h"
#include "../module/linear.h"

BedLevelService levelservice;

extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;

ErrCode BedLevelService::DoXCalibration(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  if ((MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) && (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL)) {
    if (event.length != 8) {
      return E_PARAM;
    }

    extruder0_temp = (event.data[0] << 8) | event.data[1];
    extruder1_temp = (event.data[2] << 8) | event.data[3];
    bed_temp = (event.data[4] << 8) | event.data[5];
    ((uint8_t *)&e_factor)[0] = event.data[6];
    ((uint8_t *)&e_factor)[1] = event.data[7];
    ((uint8_t *)&e_factor)[2] = event.data[8];
    ((uint8_t *)&e_factor)[3] = event.data[9];

    thermalManager.setTargetHotend(extruder0_temp, TOOLHEAD_3DP_EXTRUDER0);
    thermalManager.setTargetHotend(extruder1_temp, TOOLHEAD_3DP_EXTRUDER1);
    thermalManager.setTargetBed(bed_temp);

    if (!all_axes_homed()) {
      process_cmd_imd("G28");
    }

    relative_mode = false; //absolute positioning

    tool_change(TOOLHEAD_3DP_EXTRUDER0);

    thermalManager.wait_for_hotend(TOOLHEAD_3DP_EXTRUDER0, true);
    thermalManager.wait_for_hotend(TOOLHEAD_3DP_EXTRUDER1, true);
    thermalManager.wait_for_bed(true);

    feedrate_mm_s = MMM_TO_MMS(1080.0f);

    float destination_position_logic[X_TO_E];
    float start_point[XYZ] = X_CALIBRATION_A350_START_POINT_XYZ;

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS];
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = 0;
    if (current_position[Z_AXIS] < 30) {
      do_blocking_move_to_logical_z(30, 30);
    }
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
    do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 10);

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS];
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS] + X_CALIBRATION_UP_DOWN_LINE_LENGTH;
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = X_CALIBRATION_UP_DOWN_LINE_LENGTH * e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // retractiong
    destination_position_logic[E_AXIS] -= E_RETACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    uint32_t i;
    float main_line_y_start_postion = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH/2 + MAIN_SUB_SAFE_DISTANCE;
    for (i = 0; i < MAIN_SCALE_LINES; i++) {
      destination_position_logic[X_AXIS] = start_point[X_AXIS] + FIRST_SCALE_LINE_TO_BORDER + i * MAIN_SCALE_LINE_INTERVAL;
      destination_position_logic[Y_AXIS] = main_line_y_start_postion;
      destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
      destination_position_logic[E_AXIS] += E_RETACTION_LENGTH;
      do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      float print_line_length = 0.0;
      if (i == SCALE_0_LINE_NUMBER) {
        print_line_length = SCALE_0_LINE_LENGTH;
      }
      else if (i%SCALE_LONGER_LINE_SEQUENCE == 0) {
        print_line_length = SCALE_LINE_LENGTH_LONGER;
      }
      else {
        print_line_length = SCALE_LINE_LENGHT_NORMAL;
      }

      current_position[E_AXIS] = 0;
      destination_position_logic[Y_AXIS] = main_line_y_start_postion + print_line_length;
      destination_position_logic[E_AXIS] = print_line_length * e_factor;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      // retraction
      destination_position_logic[E_AXIS] -= E_RETACTION_LENGTH;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();
    }

    tool_change(TOOLHEAD_3DP_EXTRUDER1);

    destination_position_logic[X_AXIS] = start_point[X_AXIS];
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = 0;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS] + X_CALIBRATION_UP_DOWN_LINE_LENGTH;
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = X_CALIBRATION_UP_DOWN_LINE_LENGTH * e_factor ;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS] + X_CALIBRATION_UP_DOWN_LINE_LENGTH;
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // retraction
    destination_position_logic[E_AXIS] -= E_RETACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    float sub_line_y_star_position = start_point[Y_AXIS] + X_CALIBRATION_LEFT_RIGHT_LINE_LENGTH/2 - MAIN_SUB_SAFE_DISTANCE;
    for (i = 0; i < SUB_SCALE_LINES; i++) {
      destination_position_logic[X_AXIS] = start_point[X_AXIS] + FIRST_SCALE_LINE_TO_BORDER + SCALE_0_LINE_NUMBER * MAIN_SCALE_LINE_INTERVAL + i * SUB_SCALE_LINE_INTERVAL;
      destination_position_logic[Y_AXIS] = sub_line_y_star_position;
      destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
      destination_position_logic[E_AXIS] += E_RETACTION_LENGTH;
      do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      float print_line_length = 0.0;
      if (i%SCALE_LONGER_LINE_SEQUENCE == 0) {
        print_line_length = SCALE_LINE_LENGTH_LONGER;
      }
      else {
        print_line_length = SCALE_LINE_LENGHT_NORMAL;
      }

      current_position[E_AXIS] = 0;
      destination_position_logic[Y_AXIS] = sub_line_y_star_position - print_line_length;
      destination_position_logic[E_AXIS] = print_line_length * e_factor;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      // retraction
      destination_position_logic[E_AXIS] -= E_RETACTION_LENGTH;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();
    }

    do_blocking_move_to_logical_z(100, 40);
  }

  return err;
}

ErrCode BedLevelService::ApplyXCalibration(SSTP_Event_t &event) {
  if (event.length != 2) {
    return E_PARAM;
  }

  // save x direction hotend_offset
  signed char cross_0_scale_lines = (signed char)event.data[0];
  uint8_t sub_alignment_line_number = event.data[1];
  float main_measurement_unit_offset = 0.0;
  float main_sub_0_scale_line_distance = 0.0;

  if (cross_0_scale_lines >= 0) {
    main_measurement_unit_offset = sub_alignment_line_number * SCALE_MEASUREMENT_ACCURACY;
    main_sub_0_scale_line_distance = (cross_0_scale_lines - 1) * MAIN_SCALE_LINE_INTERVAL + main_measurement_unit_offset;
  }
  else {
    main_measurement_unit_offset = sub_alignment_line_number * SCALE_MEASUREMENT_ACCURACY;
    main_sub_0_scale_line_distance = cross_0_scale_lines * MAIN_SCALE_LINE_INTERVAL + main_measurement_unit_offset;
  }

  hotend_offset[X_AXIS][TOOLHEAD_3DP_EXTRUDER1] += main_sub_0_scale_line_distance;
  settings.save();

  return E_SUCCESS;
}


ErrCode BedLevelService::DoYCalibration(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  if ((MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) && (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL)) {
    if (event.length != 8) {
      return E_PARAM;
    }

    extruder0_temp = (event.data[0] << 8) | event.data[1];
    extruder1_temp = (event.data[2] << 8) | event.data[3];
    bed_temp = (event.data[4] << 8) | event.data[5];
    ((uint8_t *)&e_factor)[0] = event.data[6];
    ((uint8_t *)&e_factor)[1] = event.data[7];
    ((uint8_t *)&e_factor)[2] = event.data[8];
    ((uint8_t *)&e_factor)[3] = event.data[9];

    thermalManager.setTargetHotend(extruder0_temp, TOOLHEAD_3DP_EXTRUDER0);
    thermalManager.setTargetHotend(extruder1_temp, TOOLHEAD_3DP_EXTRUDER1);
    thermalManager.setTargetBed(bed_temp);

    if (!all_axes_homed()) {
      process_cmd_imd("G28");
    }

    relative_mode = false; //absolute positioning

    tool_change(TOOLHEAD_3DP_EXTRUDER0);

    thermalManager.wait_for_hotend(TOOLHEAD_3DP_EXTRUDER0, true);
    thermalManager.wait_for_hotend(TOOLHEAD_3DP_EXTRUDER1, true);
    thermalManager.wait_for_bed(true);

    feedrate_mm_s = MMM_TO_MMS(1080.0f);

    float destination_position_logic[X_TO_E];
    float start_point[XYZ] = Y_CALIBRATION_A350_START_POINT_XYZ;

    destination_position_logic[X_AXIS] = start_point[X_AXIS];
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = 0;
    if (current_position[Z_AXIS] < 30) {
      do_blocking_move_to_logical_z(30, 30);
    }
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
    do_blocking_move_to_logical_z(destination_position_logic[Z_AXIS], 10);

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS];
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH;
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = Y_CALIBRATION_UP_DOWN_LINE_LENGTH * e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // retraction
    destination_position_logic[E_AXIS] -= E_RETACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    uint32_t i;
    float main_line_x_start_position = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH/2 - MAIN_SUB_SAFE_DISTANCE;
    for (i = 0; i < MAIN_SCALE_LINES; i++) {
      destination_position_logic[X_AXIS] = main_line_x_start_position;
      destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + FIRST_SCALE_LINE_TO_BORDER + i * MAIN_SCALE_LINE_INTERVAL;
      destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
      destination_position_logic[E_AXIS] += E_RETACTION_LENGTH;
      do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      float print_line_length = 0.0;
      if (i == SCALE_0_LINE_NUMBER) {
        print_line_length = SCALE_0_LINE_LENGTH;
      }
      else if (i%SCALE_LONGER_LINE_SEQUENCE == 0) {
        print_line_length = SCALE_LINE_LENGTH_LONGER;
      }
      else {
        print_line_length = SCALE_LINE_LENGHT_NORMAL;
      }

      current_position[E_AXIS] = 0;
      destination_position_logic[X_AXIS] = main_line_x_start_position - print_line_length;
      destination_position_logic[E_AXIS] = print_line_length * e_factor;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      // retraction
      destination_position_logic[E_AXIS] -= E_RETACTION_LENGTH;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();
    }

    tool_change(TOOLHEAD_3DP_EXTRUDER1);

    destination_position_logic[X_AXIS] = start_point[X_AXIS];
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = 0;
    do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH;
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS];
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = Y_CALIBRATION_UP_DOWN_LINE_LENGTH * e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    current_position[E_AXIS] = 0;
    destination_position_logic[X_AXIS] = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH;
    destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH;
    destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
    destination_position_logic[E_AXIS] = Y_CALIBRATION_LEFT_RIGHT_LINE_LENGTH * e_factor;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // retration
    destination_position_logic[E_AXIS] -= E_RETACTION_LENGTH;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    float sub_line_x_start_position = start_point[X_AXIS] + Y_CALIBRATION_UP_DOWN_LINE_LENGTH/2 + MAIN_SUB_SAFE_DISTANCE;
    for (i = 0; i < SUB_SCALE_LINES; i++) {
      destination_position_logic[X_AXIS] = sub_line_x_start_position;
      destination_position_logic[Y_AXIS] = start_point[Y_AXIS] + FIRST_SCALE_LINE_TO_BORDER + SCALE_0_LINE_NUMBER * MAIN_SCALE_LINE_INTERVAL + i * SUB_SCALE_LINE_INTERVAL;
      destination_position_logic[Z_AXIS] = start_point[Z_AXIS];
      destination_position_logic[E_AXIS] += E_RETACTION_LENGTH;
      do_blocking_move_to_logical_xy(destination_position_logic[X_AXIS], destination_position_logic[Y_AXIS], 80);
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      float print_line_length = 0.0;
      if (i%SCALE_LONGER_LINE_SEQUENCE == 0) {
        print_line_length = SCALE_LINE_LENGTH_LONGER;
      }
      else {
        print_line_length = SCALE_LINE_LENGHT_NORMAL;
      }

      current_position[E_AXIS] = 0;
      destination_position_logic[X_AXIS] = sub_line_x_start_position + print_line_length;
      destination_position_logic[E_AXIS] = print_line_length * e_factor;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      // retration
      destination_position_logic[E_AXIS] -= E_RETACTION_LENGTH;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();
    }

    do_blocking_move_to_logical_z(100, 40);
  }

  return err;
}

ErrCode BedLevelService::ApplyYCalibration(SSTP_Event_t &event) {
  if (event.length != 2) {
    return E_PARAM;
  }
  // save x direction hotend_offset
  signed char cross_0_scale_lines = (signed char)event.data[0];
  uint8_t sub_alignment_line_number = (uint8_t)event.data[1];
  float main_measurement_unit_offset = 0.0;
  float main_sub_0_scale_line_distance = 0.0;

  if (cross_0_scale_lines >= 0) {
    main_measurement_unit_offset = sub_alignment_line_number * SCALE_MEASUREMENT_ACCURACY;
    main_sub_0_scale_line_distance = (cross_0_scale_lines - 1) * MAIN_SCALE_LINE_INTERVAL + main_measurement_unit_offset;
  }
  else {
    main_measurement_unit_offset = sub_alignment_line_number * SCALE_MEASUREMENT_ACCURACY;
    main_sub_0_scale_line_distance = cross_0_scale_lines * MAIN_SCALE_LINE_INTERVAL + main_measurement_unit_offset;
  }

  hotend_offset[Y_AXIS][TOOLHEAD_3DP_EXTRUDER1] += main_sub_0_scale_line_distance;
  settings.save();

  return E_SUCCESS;
}

ErrCode BedLevelService::DoAutoLeveling(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t grid = 3;
  char cmd[16];

  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  LOG_I("SC req auto level\n");

  if (event.length > 0) {
    if (event.data[0] > 7 || event.data[0] < 2) {
      LOG_E("grid [%u] from SC is out of range [2:7], set to default: 3\n", event.data[0]);
      goto OUT;
    }
    else {
      grid = event.data[0];
    }
  }

  if (((printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) && (grid % 2 != 1)) || grid < 2) {
    return E_FAILURE;
  }

  LOG_I("e temp: %.2f / %d\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
  LOG_I("b temp: %.2f / %d\n", thermalManager.degBed(), thermalManager.degTargetBed());

  if (MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) {

    // MUST clear live z offset before G28
    // otherwise will apply live z after homing
    live_z_offset_ = 0;

    process_cmd_imd("G28");

    if (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) {
      // switch to the left nozzle
      if (linear_p->machine_size() == MACHINE_SIZE_A150) {
          current_position[X_AXIS] = 100;
          planner.buffer_line(current_position, feedrate_mm_s, active_extruder);
          planner.synchronize();
      }
      tool_change(TOOLHEAD_3DP_EXTRUDER0);
    }

    snprintf(cmd, 16, "G1029 P%u\n", grid);
    process_cmd_imd(cmd);

    set_bed_leveling_enabled(false);

    current_position[Z_AXIS] = Z_MAX_POS;
    sync_plan_position();

    // change the Z max feedrate
    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    endstops.enable_z_probe(true);

    LOG_I("enable z probe\n");

    // move quicky firstly to decrease the time
    // move to the first calibration mesh point allow the sensor to detect the bed if the bed
    // is on an unexpected height
    do_blocking_move_to_logical_xy(_GET_MESH_X(0),_GET_MESH_Y(0),speed_in_calibration[X_AXIS]);
    do_blocking_move_to_z(z_position_before_calibration, speed_in_calibration[Z_AXIS]);
    planner.synchronize();

    if (event.op_code == SETTINGS_OPC_DO_AUTO_LEVELING)
      err = auto_probing(true, false);
    else
      err = auto_probing(true, true);

    endstops.enable_z_probe(false);

    // Recover the Z max feedrate to 20mm/s
    planner.settings.max_feedrate_mm_s[Z_AXIS] = orig_max_z_speed;
  }

  if (err != E_SUCCESS)
    goto OUT;

  if (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) {
    level_mode_ = LEVEL_MODE_AUTO_NO_ADJUST;
    return SaveAndExitLeveling(event);
  }
  else {
    level_mode_ = LEVEL_MODE_AUTO;
  }

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode BedLevelService::DoManualLeveling(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;
  uint32_t i, j;
  float orig_max_z_speed = planner.settings.max_feedrate_mm_s[Z_AXIS];

  uint8_t grid = 3;
  char cmd[16];

  if (event.length > 0) {
    if (event.data[0] > 7 || event.data[0] < 2) {
      LOG_E("grid [%u] from SC is out of range [2:7], set to default: 3\n", event.data[0]);
      goto OUT;
    }
    else {
      grid = event.data[0];
    }
  }

  // when user do manual leveling, clear this var to disable fast-calibration
  nozzle_height_probed = 0;

  LOG_I("SC req manual level\n");

  if (MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) {

    // MUST clear live z offset before G28
    // otherwise will apply live z after homing
    live_z_offset_ = 0;

    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    process_cmd_imd("G28");

    snprintf(cmd, 16, "G1029 P%u\n", grid);
    process_cmd_imd(cmd);

    set_bed_leveling_enabled(false);

    current_position[Z_AXIS] = Z_MAX_POS;
    sync_plan_position();

    // change the Z max feedrate
    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    // Move Z to 20mm height
    do_blocking_move_to_z(z_position_before_calibration, speed_in_calibration[Z_AXIS]);

    // increase 3mm for first leveling point
    // to avoid nozzle gouging the surface when user place glass on the steel sheet
    do_blocking_move_to_z(12, 10);

    for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
      for (i = 0; i < GRID_MAX_POINTS_X; i++) {
        MeshPointZ[j * GRID_MAX_POINTS_X + i] = z_values[i][j];
      }
    }

    planner.settings.max_feedrate_mm_s[Z_AXIS] = orig_max_z_speed;

    // save the leveling mode
    level_mode_ = LEVEL_MODE_MANUAL;
    // Preset the index to 99 for initial status
    manual_level_index_ = MESH_POINT_SIZE;

    err = E_SUCCESS;
  }

OUT:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}


ErrCode BedLevelService::SetManualLevelingPoint(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  uint8_t index;

  if (!event.length) {
    LOG_E("Need to specify point index!\n");
    goto out;
  }
  else {
    index = event.data[0];
    LOG_I("SC req move to pont: %d\n", index);
  }

  LOG_I("manual leveling pre, index:%d, manual_level_index_:%d\n", index, manual_level_index_);
  if ((index <= GRID_MAX_POINTS_INDEX) && (index > 0)) {
    // check point index
    if (manual_level_index_ < GRID_MAX_POINTS_INDEX - 1) {
      // save point index
      MeshPointZ[manual_level_index_] = current_position[Z_AXIS];
      LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", manual_level_index_, current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

      if (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) {
        if (active_extruder == TOOLHEAD_3DP_EXTRUDER0) {
          ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][manual_level_index_] = MeshPointZ[manual_level_index_];
        }
        else if (active_extruder == TOOLHEAD_3DP_EXTRUDER1) {
          ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER1][manual_level_index_] = MeshPointZ[manual_level_index_];
        }
      }

      // if got new point, raise Z firstly
      if ((manual_level_index_ != index -1) && current_position[Z_AXIS] < z_position_before_calibration)
        do_blocking_move_to_z(current_position[Z_AXIS] + 3, speed_in_calibration[Z_AXIS]);
    }

    // move to new point
    manual_level_index_ = index -1;
    do_blocking_move_to_logical_xy(_GET_MESH_X(manual_level_index_ % GRID_MAX_POINTS_X),
                    _GET_MESH_Y(manual_level_index_ / GRID_MAX_POINTS_Y), speed_in_calibration[X_AXIS]);
  }
  LOG_I("manual leveling post, index:%d, manual_level_index_:%d\n", index, manual_level_index_);

  err = E_SUCCESS;

out:
  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

ErrCode BedLevelService::SwitchToExtruder1ForBedLevel(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  MeshPointZ[manual_level_index_] = current_position[Z_AXIS];
  ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][manual_level_index_] = MeshPointZ[manual_level_index_];

  tool_change(TOOLHEAD_3DP_EXTRUDER1);

  err = E_SUCCESS;

  event.data = &err;
  event.length = 1;

  return hmi.Send(event);
}

ErrCode BedLevelService::AdjustZOffsetInLeveling(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;
  float   offset = 0;

  if (event.length < 4) {
    LOG_E("Need to specify z offset!\n");
    event.length = 1;
    event.data = &err;
    return hmi.Send(event);
  }

  PDU_TO_LOCAL_WORD(offset, event.data);

  offset /= 1000;

  LOG_I("SC req Z offset: %.2f\n", offset);

  // sometimes the bed plane will be under the low limit point
  // to make z can move down always by user, we don't use limited API
  do_blocking_move_to_z(current_position[Z_AXIS] + offset, speed_in_calibration[Z_AXIS]);

  planner.synchronize();

  event.length = 1;
  event.data[0] = 0;

  return hmi.Send(event);
}

ErrCode BedLevelService::SaveAndExitLeveling(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  uint32_t i, j;

  event.data = &err;
  event.length = 1;

  LOG_I("SC req save data of leveling\n");

  planner.synchronize();

  switch (level_mode_) {
  case LEVEL_MODE_MANUAL:
    if (manual_level_index_ > GRID_MAX_POINTS_INDEX) {
      err = E_FAILURE;
      goto out;
    }

    MeshPointZ[manual_level_index_] = current_position[Z_AXIS];
    LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n", manual_level_index_,
        current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);

    if (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) {
      if (active_extruder == TOOLHEAD_3DP_EXTRUDER0) {
        ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][manual_level_index_] = MeshPointZ[manual_level_index_];
      }
      else if (active_extruder == TOOLHEAD_3DP_EXTRUDER1) {
        ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER1][manual_level_index_] = MeshPointZ[manual_level_index_];
      }

      for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
        for (i = 0; i < GRID_MAX_POINTS_X; i++) {
          z_values[i][j] = ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][j * GRID_MAX_POINTS_X + i];
        }
      }
    }
    else if (printer1->device_id() == MODULE_DEVICE_ID_3DP_SINGLE) {
      for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
        for (i = 0; i < GRID_MAX_POINTS_X; i++) {
          z_values[i][j] = MeshPointZ[j * GRID_MAX_POINTS_X + i];
        }
      }
    }

    if (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) {
      #if ENABLED(PROBE_ALL_LEVELING_POINTS)
        for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
          for (i = 0; i < GRID_MAX_POINTS_X; i++) {
            extruders_z_values[TOOLHEAD_3DP_EXTRUDER0][i][j] = ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][j * GRID_MAX_POINTS_X + i];
            extruders_z_values[TOOLHEAD_3DP_EXTRUDER1][i][j] = ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER1][j * GRID_MAX_POINTS_X + i];
          }
        }
      #elif ENABLED(PROBE_LAST_LEVELING_POINT)
        float temp_z = ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][manual_level_index_] - ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER1][manual_level_index_];
        hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = temp_z;
        LOG_I("right nozzle higher than left nozzle by: %f\n", temp_z);
      #endif
    }

    bed_level_virt_interpolate();
    settings.save();
    break;

  case LEVEL_MODE_AUTO_NO_ADJUST:
    nozzle_height_probed = 1;
    process_cmd_imd("G1029 S1");
    break;

  case LEVEL_MODE_AUTO:
    process_cmd_imd("G1029 S0");
    break;

  default:
    LOG_E("didn't start leveling!\n");
    err = E_FAILURE;
    break;
  }

  LOG_I("new leveling data:\n");
  for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
    for (i = 0; i < GRID_MAX_POINTS_X; i++) {
      LOG_I("%.2f ", z_values[i][j]);
    }
    LOG_I("\n");
  }

  set_bed_leveling_enabled(true);

  // move to stop
  move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);

  if (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) {
    hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = 0;
    tool_change(TOOLHEAD_3DP_EXTRUDER0);
    hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = hotend_offset_z_temp;
  }

  planner.synchronize();

  // make sure we are in absolute mode
  relative_mode = false;

  // clear flag
  level_mode_ = LEVEL_MODE_INVALD;

out:
  return hmi.Send(event);
}


ErrCode BedLevelService::ExitLeveling(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;

  LOG_I("SC req exit level\n");

  event.data = &err;
  event.length = 1;

  if (level_mode_ == LEVEL_MODE_INVALD) {
    err = E_FAILURE;
    return hmi.Send(event);
  }

  planner.synchronize();

  //Load
  settings.load();

  set_bed_leveling_enabled(true);

  // move to stop
  move_to_limited_z(z_limit_in_cali, XY_PROBE_FEEDRATE_MM_S/2);
  planner.synchronize();

  level_mode_ = LEVEL_MODE_INVALD;
  manual_level_index_ = MANUAL_LEVEL_INDEX_INVALID;

  // make sure we are in absolute mode
  relative_mode = false;

  return hmi.Send(event);
}


ErrCode BedLevelService::SyncPointIndex(uint8_t index) {
  SSTP_Event_t event = {EID_SETTING_ACK, SETTINGS_OPC_SYNC_LEVEL_POINT};

  uint8_t buffer[2];

  event.data = buffer;
  event.length = 2;

  buffer[0] = 0;
  buffer[1] = index;

  return hmi.Send(event);
}


ErrCode BedLevelService::UpdateLiveZOffset(float offset) {
  LOG_I("old live_z_offset: %.3f\n", live_z_offset_);

  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return E_FAILURE;
  }

  if (offset < LIVE_Z_OFFSET_MIN || offset > LIVE_Z_OFFSET_MAX) {
    LOG_E("offset is out of range: %.2f\n", offset);
    return E_PARAM;
  }

  if (offset == live_z_offset_) {
    LOG_W("offset is same with old\n");
    return E_PARAM;
  }

  planner.synchronize();

  float cur_z = current_position[Z_AXIS];

  do_blocking_move_to_z(current_position[Z_AXIS] + (offset - live_z_offset_), 5);
  planner.synchronize();

  current_position[Z_AXIS] = cur_z;
  sync_plan_position();

  LOG_I("new live Z: %.3f, delta: %.3f\n", offset, (offset - live_z_offset_));

  live_z_offset_ = offset;
  live_z_offset_updated_ = true;
  return E_SUCCESS;
}


void BedLevelService::ApplyLiveZOffset() {
  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return;
  }

  planner.synchronize();
  float cur_z = current_position[Z_AXIS];

  do_blocking_move_to_z(current_position[Z_AXIS] + live_z_offset_, 5);
  planner.synchronize();

  current_position[Z_AXIS] = cur_z;
  sync_plan_position();

  LOG_I("Apply Z offset: %.2f\n", live_z_offset_);
}


void BedLevelService::UnapplyLiveZOffset() {
  if (MODULE_TOOLHEAD_3DP != ModuleBase::toolhead()) {
    LOG_E("only enable z offset for 3DP!\n");
    return;
  }
  planner.synchronize();
  float cur_z = current_position[Z_AXIS];

  do_blocking_move_to_z(current_position[Z_AXIS] - live_z_offset_, 5);
  planner.synchronize();

  current_position[Z_AXIS] = cur_z;
  sync_plan_position();

  LOG_I("Unapply Z offset: %.2f\n", live_z_offset_);
}


void BedLevelService::SaveLiveZOffset() {
  if (live_z_offset_updated_) {
    live_z_offset_updated_ = false;
    settings.save();
  }
}

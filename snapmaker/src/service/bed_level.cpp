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

BedLevelService levelservice;

extern uint32_t GRID_MAX_POINTS_X;
extern uint32_t GRID_MAX_POINTS_Y;

ErrCode BedLevelService::DoXCalibration(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  int16_t cross_0_scale_lines = -1;

  cross_0_scale_lines = event.data[0] | (event.data[1] << 8);

  if (cross_0_scale_lines < -5) {
    cross_0_scale_lines = -5;
  }
  else if (cross_0_scale_lines > 10) {
    cross_0_scale_lines = 10;
  }

  LOG_I("cross_0_scale_lines:%d\n", cross_0_scale_lines);

  hotend_offset[X_AXIS][TOOLHEAD_3DP_EXTRUDER1] += cross_0_scale_lines * MAIN_SCALE_LINE_INTERVAL;

  if ((MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) && (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL)) {
    process_cmd_imd("M104 T0 S200");
    process_cmd_imd("M104 T1 S200");
    process_cmd_imd("M140 S70");

    if (!all_axes_homed()) {
      process_cmd_imd("G28");
    }

    process_cmd_imd("G90"); //absolute positioning

    tool_change(TOOLHEAD_3DP_EXTRUDER0);

    process_cmd_imd("M109 T0 S200");
    process_cmd_imd("M109 T1 S200");
    process_cmd_imd("M190 S70");

    process_cmd_imd("G92 E0");
    process_cmd_imd("G1 E20 F200");
    process_cmd_imd("G92 E0");

    float destination_position_logic[XYZE];
    // 走到打印的开始位置
    process_cmd_imd("G92 E0");
    destination_position_logic[X_AXIS] = 140;
    destination_position_logic[Y_AXIS] = 180;
    destination_position_logic[Z_AXIS] = 0.2;
    destination_position_logic[E_AXIS] = 0;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // 打印左边界
    process_cmd_imd("G92 E0");
    destination_position_logic[X_AXIS] = 140;
    destination_position_logic[Y_AXIS] = 240;
    destination_position_logic[Z_AXIS] = 0.2;
    destination_position_logic[E_AXIS] = 1.9956;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // 打印上边界
    process_cmd_imd("G92 E0");
    destination_position_logic[X_AXIS] = 200;
    destination_position_logic[Y_AXIS] = 240;
    destination_position_logic[Z_AXIS] = 0.2;
    destination_position_logic[E_AXIS] = 1.9956;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    uint32_t i;
    // 每隔1mm打印主尺刻度线，第6根线长30mm，其它线长20mm
    for (i = 0; i < MAIN_SCALE_LINES; i++) {
      // 走到要打印刻度线的起始位置
      process_cmd_imd("G92 E0");
      destination_position_logic[X_AXIS] = 140 + 3 + i * MAIN_SCALE_LINE_INTERVAL;
      destination_position_logic[Y_AXIS] = 210.5;
      destination_position_logic[Z_AXIS] = 0.2;
      destination_position_logic[E_AXIS] = 0;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      // 打印这根线
      i != 6 ? destination_position_logic[Y_AXIS] = 230.5,  destination_position_logic[E_AXIS] = 0.6652 : destination_position_logic[Y_AXIS] = 235.5, destination_position_logic[E_AXIS] = 0.8315;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();
    }

    // 走到打印的开始位置
    process_cmd_imd("G92 E0");
    destination_position_logic[X_AXIS] = 140;
    destination_position_logic[Y_AXIS] = 180;
    destination_position_logic[Z_AXIS] = 0.2;
    destination_position_logic[E_AXIS] = 0;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // 打印下边界
    process_cmd_imd("G92 E0");
    destination_position_logic[X_AXIS] = 200;
    destination_position_logic[Y_AXIS] = 180;
    destination_position_logic[Z_AXIS] = 0.2;
    destination_position_logic[E_AXIS] = 1.9956;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // 打印右边界
    process_cmd_imd("G92 E0");
    destination_position_logic[X_AXIS] = 200;
    destination_position_logic[Y_AXIS] = 240;
    destination_position_logic[Z_AXIS] = 0.2;
    destination_position_logic[E_AXIS] = 1.9956;
    get_destination_from_logic(destination_position_logic);
    prepare_move_to_destination();

    // 每隔0.9mm打印副尺刻度线，线长20mm
    for (i = 0; i < 10; i++) {
      // 走到要打印刻度线的起始位置
      process_cmd_imd("G92 E0");
      destination_position_logic[X_AXIS] = 140 + 3 + i * 1;
      destination_position_logic[Y_AXIS] = 209.5;
      destination_position_logic[Z_AXIS] = 0.2;
      destination_position_logic[E_AXIS] = 0;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();

      // 打印这根线
      destination_position_logic[Y_AXIS] = 189.5;
      destination_position_logic[E_AXIS] = 0.6652;
      get_destination_from_logic(destination_position_logic);
      prepare_move_to_destination();
    }

    // 打印完毕，温度降下来，z轴抬升，等待用户选择对齐的刻度线还是重做
    process_cmd_imd("M109 T0 S170");
    process_cmd_imd("M109 T1 S170");
    process_cmd_imd("M190 S50");
  }

  return err;
}

ErrCode BedLevelService::DoYCalibration(SSTP_Event_t &event) {
  ErrCode err = E_SUCCESS;
  int16_t cross_0_scale_lines = -1;

  cross_0_scale_lines = event.data[0] | (event.data[1] << 8);

  if (cross_0_scale_lines < -5) {
    cross_0_scale_lines = -5;
  }
  else if (cross_0_scale_lines > 10) {
    cross_0_scale_lines = 10;
  }

  LOG_I("cross_0_scale_lines:%d\n", cross_0_scale_lines);





  return err;
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

  LOG_I("e temp: %.2f / %d\n", thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
  LOG_I("b temp: %.2f / %d\n", thermalManager.degBed(), thermalManager.degTargetBed());

  if (MODULE_TOOLHEAD_3DP == ModuleBase::toolhead()) {

    // MUST clear live z offset before G28
    // otherwise will apply live z after homing
    live_z_offset_ = 0;

    process_cmd_imd("G28");

    if (printer1->device_id() == MODULE_DEVICE_ID_3DP_DUAL) {
      // switch to the left nozzle
      tool_change(0);
    }

    snprintf(cmd, 16, "G1029 P%u\n", grid);
    process_cmd_imd(cmd);

    set_bed_leveling_enabled(false);

    current_position[Z_AXIS] = Z_MAX_POS;
    sync_plan_position();

    // change the Z max feedrate
    planner.settings.max_feedrate_mm_s[Z_AXIS] = max_speed_in_calibration[Z_AXIS];

    endstops.enable_z_probe(true);

    // move quicky firstly to decrease the time
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

ErrCode BedLevelService::GetCurrentPointZValue(SSTP_Event_t &event) {
  ErrCode err = E_FAILURE;

  MeshPointZ[manual_level_index_] = current_position[Z_AXIS];
  ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][manual_level_index_] = MeshPointZ[manual_level_index_];

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
    }

    for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
      for (i = 0; i < GRID_MAX_POINTS_X; i++) {
        z_values[i][j] = MeshPointZ[j * GRID_MAX_POINTS_X + i];
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
        // for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
        //   for (i = 0; i < GRID_MAX_POINTS_X; i++) {
        //     extruders_z_values[TOOLHEAD_3DP_EXTRUDER0][i][j] = ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][j * GRID_MAX_POINTS_X + i];
        //     extruders_z_values[TOOLHEAD_3DP_EXTRUDER1][i][j] = extruders_z_values[TOOLHEAD_3DP_EXTRUDER0][i][j] + temp_z;
        //   }
        // }
        for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
          for (i = 0; i < GRID_MAX_POINTS_X; i++) {
            extruders_z_values[TOOLHEAD_3DP_EXTRUDER0][i][j] = ExtrudersMeshPointZ[TOOLHEAD_3DP_EXTRUDER0][j * GRID_MAX_POINTS_X + i];
            extruders_z_values[TOOLHEAD_3DP_EXTRUDER1][i][j] = extruders_z_values[TOOLHEAD_3DP_EXTRUDER0][i][j];
          }
        }
        hotend_offset[Z_AXIS][TOOLHEAD_3DP_EXTRUDER1] = temp_z;
        LOG_I("right nozzle higher than left nozzle by: %f\n", temp_z);
      #endif
    }

    bed_level_virt_interpolate();
    bed_level_virt_interpolate(TOOLHEAD_3DP_EXTRUDER0);
    bed_level_virt_interpolate(TOOLHEAD_3DP_EXTRUDER1);
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
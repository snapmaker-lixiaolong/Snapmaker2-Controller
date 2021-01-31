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
void GcodeSuite::G4029() {
  if (parser.seen('S')) {
    uint8_t cmd = parser.byteval('S', 0);

    switch (cmd) {
      case 10:
        printer1->SetBLTouch(10);
        break;
      case 60:
        printer1->SetBLTouch(60);
        break;
      case 90:
        printer1->SetBLTouch(90);
        break;
      case 120:
        printer1->SetBLTouch(120);
        break;
      case 130:
        printer1->SetBLTouch(130);
        break;
      case 140:
        printer1->SetBLTouch(140);
        break;
      case 150:
        printer1->SetBLTouch(150);
        break;
      case 160:
        printer1->SetBLTouch(160);
        break;
      default:
        break;
    }
  }

  if (parser.seen('X')) {
    uint8_t sensor = parser.byteval('X', 0);
    switch (sensor) {
      case 0:
        printer1->SetProbeSensor(0);
        break;
      case 1:
        printer1->SetProbeSensor(1);
        break;
    }
  }
}

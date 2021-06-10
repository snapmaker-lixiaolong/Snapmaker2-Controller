#include "snapmaker.h"
#include "service/system.h"
#include "src/gcode/gcode.h"
#include "../module/toolhead_laser.h"

void GcodeSuite::M2022() {
  uint16_t display_interval = 1000;

  display_interval = (uint16_t)parser.ushortval('D', (uint16_t)1000);
  laser->SetLaserTempDisplayInterval(display_interval);
}



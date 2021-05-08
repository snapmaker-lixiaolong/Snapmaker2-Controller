#include "snapmaker.h"
#include "service/system.h"
#include "src/gcode/gcode.h"
#include "../module/toolhead_laser.h"

void GcodeSuite::M2021() {
  uint16_t display_interval = 1000;

  uint16_t s = (uint16_t)parser.ushortval('S', (uint16_t)0);
  display_interval = (uint16_t)parser.ushortval('D', (uint16_t)1000);
  laser->SetDisplayInterval(display_interval);
  laser->UpdateGestureInfo(s);
}



#include "snapmaker.h"
#include "service/system.h"
#include "src/gcode/gcode.h"
#include "../module/toolhead_laser.h"

void GcodeSuite::G1001() {
  const bool seen_a = parser.seen("A");
  const bool seen_s = parser.seen("S");
  const bool seen_p = parser.seen("P");
  const bool seen_v = parser.seen("V");
  float position = 0;
  float velocity = 0;

  if (seen_p) {
    position = (uint8_t)parser.byteval('P', (uint8_t)0);
  }

  if (seen_v) {
    velocity = (uint8_t)parser.byteval('V', (uint8_t)0);
  }

  if (seen_a) {
    laser->LaserMoveToDestinationAsync(position, velocity);
    LOG_I("LaserMoveToDestinationAsync\n");
    return;
  }

  if (seen_s) {
    laser->LaserMoveToDestinationSync(position, velocity);
    LOG_I("LaserMoveToDestinationSync\n");
    return;
  }

}



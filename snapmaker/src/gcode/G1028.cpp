#include "snapmaker.h"
#include "service/system.h"
#include "src/gcode/gcode.h"
#include "../module/toolhead_laser.h"

void GcodeSuite::G1028() {
  const bool seen_a = parser.seen("A");
  const bool seen_s = parser.seen("S");

  if (seen_a) {
    laser->LaserGoHomeAsync();
    LOG_I("LaserGoHomeAsync\n");
    return;
  }

  if (seen_s) {
    laser->LaserGoHomeSync();
    LOG_I("LaserGoHomeSync\n");
    return;
  }
}



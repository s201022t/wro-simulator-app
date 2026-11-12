#include "tank_on_brake.h"
#include <limits.h>

using namespace tank_on_brake;
static_assert(!finished(12, 3), "Forward deceleration is still moving");
static_assert(!finished(-12, -3), "Reverse deceleration is still moving");
static_assert(finished(12, 0) && finished(-12, 0), "Stop at zero motion");
static_assert(finished(12, -2) && finished(-12, 2), "Cut power on reversal");
static_assert(finished(12, 1) && finished(-12, -1), "One pulse is stop tolerance");
static_assert(delta(LONG_MIN, LONG_MAX) == 1, "Forward encoder rollover");
static_assert(delta(LONG_MAX, LONG_MIN) == -1, "Reverse encoder rollover");

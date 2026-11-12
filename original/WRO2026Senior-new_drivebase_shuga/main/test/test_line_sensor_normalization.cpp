#include "line_sensor_normalization.h"
using namespace line_sensor_normalization;
static_assert(normalize(200, {200, 800}) == 0, "White endpoint");
static_assert(normalize(800, {200, 800}) == 1023, "Black endpoint");
static_assert(normalize(500, {200, 800}) == 512, "Midpoint");
static_assert(normalize(100, {200, 800}) == 0, "White clipping");
static_assert(normalize(900, {200, 800}) == 1023, "Black clipping");
static_assert(normalize(800, {800, 200}) == 0, "Reversed white");
static_assert(normalize(200, {800, 200}) == 1023, "Reversed black");
static_assert(normalize(900, {800, 200}) == 0, "Reversed white clipping");
static_assert(normalize(100, {800, 200}) == 1023, "Reversed black clipping");
static_assert(normalize(840, {0, 1023}) == 840, "Default preserves threshold");
static_assert(!valid({200, 200}) && !valid({-1, 800}) && !valid({0, 1024}), "Reject invalid calibration");
static_assert(normalize(400, {200, 200}) == 400, "No divide by zero");

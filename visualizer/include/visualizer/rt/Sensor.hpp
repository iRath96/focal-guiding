#pragma once

#include <visualizer/core/math.hpp>

struct Sensor {
    Vec2f position;
    Float estimate = 0;
    Float variance = 0;
};

#pragma once

#include <visualizer/core/math.hpp>

struct Line {
    Line() : emission(0) {}
    Line(Vec2f start, Vec2f end, float emission)
    : start(start), span(end - start), emission(emission) {}

    Vec2f start;
    Vec2f span;
    float emission;
    bool reflect{false};
};

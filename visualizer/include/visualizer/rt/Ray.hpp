#pragma once

#include <visualizer/core/math.hpp>
#include <numeric>

struct Ray {
    Vec2f o;
    Vec2f d;

    Vec2f operator()(Float t) const {
        return o + t * d;
    }
};

struct Intersection {
    float t = std::numeric_limits<Float>::infinity();
    int line;

    [[nodiscard]] bool valid() const {
        return !std::isinf(t);
    }
};

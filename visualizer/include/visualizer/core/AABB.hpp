#pragma once

#include <visualizer/core/math.hpp>
#include <numeric>
#include <algorithm>

template<typename Vector = Vec2f>
struct AABB {
    Vector min;
    Vector max;

    [[nodiscard]] Float area() const {
        Vector d = diagonal();
        return d.x() * d.y();
    }

    [[nodiscard]] Float volume() const {
        return area();
    }

    [[nodiscard]] Vector midpoint() const {
        return (min + max) / 2;
    }

    [[nodiscard]] Vector relative(const Vector &absolute) const {
        return (absolute - min) / (max - min);
    }

    [[nodiscard]] Vector absolute(const Vector &relative) const {
        return relative * (max - min) + min;
    }

    static AABB empty() {
        const Float inf = std::numeric_limits<Float>::infinity();

        AABB result;
        result.min = Vector(+inf);
        result.max = Vector(-inf);
        return result;
    }

    void extend(const Vector &p) {
        for (int dim = 0; dim < Vector::Dimensionality; dim++) {
            min[dim] = std::min(min[dim], p[dim]);
            max[dim] = std::max(max[dim], p[dim]);
        }
    }

    [[nodiscard]] Vector diagonal() const {
        return max - min;
    }
};

#pragma once

#include <focal_guiding.h>

#include <vector>
#include <array>
#include <algorithm>

namespace focal_guiding {

template<typename Env>
class UniformDirection : public Distribution<Env> {
    static constexpr int Dimensionality = Env::Dimensionality;

    using Float = typename Env::Float;
    using Vector = typename Env::Vector;
    using Point = typename Env::Point;
    using AABB = typename Env::AABB;
    using PRNG = typename Env::PRNG;
    using Patch = typename Distribution<Env>::Patch;

public:
    Vector sample(const Point &origin, PRNG &prng) const override {
        const Float theta = Float(2.0 * M_PI) * prng();
        return {std::cos(theta), std::sin(theta)};
    }

    [[nodiscard]] Float pdf(const Point &origin, const Vector &direction) const override {
        return Float(2.0 * M_PI);
    }

    void splat(const Point &origin, const Vector &direction, Float distance, Float contribution, Float pdf, PRNG &prng) override {
        // nothing
    }

    void clear() override {
        // nothing
    }

    void build() override {
        // nothing
    }

    void describe(std::ostream &stream) const override {
        stream << "UniformDirection" << std::endl;
    }

    [[nodiscard]] std::vector<Patch> visualize() const override {
        return {};
    }
};

}

#pragma once

#include <visualizer/core/PRNG.hpp>
#include <visualizer/core/AABB.hpp>
#include <visualizer/rt/Ray.hpp>

#include <focal_guiding.h>

#include <vector>

class Env2D {
public:
    static constexpr int Dimensionality = 2;

    using Float = ::Float;
    using Vector = ::Vector<Float, Dimensionality>;
    using Point = ::Vector<Float, Dimensionality>;
    using AABB = ::AABB<Vector>;
    using PRNG = ::PRNG;

    static Vector divide(const Vector &a, const Vector &b) { return a / b; }

    static Float min(const Vector &v) { return v.min(); }

    static Float max(const Vector &v) { return v.max(); }

    static int argmin(const Vector &v) { return v.argmin(); }

    static int argmax(const Vector &v) { return v.argmax(); }

    static void extend(AABB &aabb, const Point &point) { aabb.extend(point); }

    static void atomicAdd(Float &dest, Float delta) {
        dest += delta;
    }

    static Vector normalize(const Vector &vec) {
        return vec.normalized();
    }

    static Float volume(const AABB &aabb) {
        return aabb.volume();
    }

    static Float segment(Float tNear, Float tFar) {
        return (tFar * tFar - tNear * tNear) / 2;
    }

    static Point absolute(const AABB &aabb, const Point &relative) {
        return aabb.absolute(relative);
    }

    static Point relative(const AABB &aabb, const Point &absolute) {
        return aabb.relative(absolute);
    }
};

void gaussian2(float &a, float &b) {
    // box muller
    const float angle = float(2.0 * M_PI) * b;
    const float radius = std::sqrt(-2 * std::log(b));
    a = radius * std::cos(angle);
    b = radius * std::sin(angle);
}

struct Sampling;

struct Gaussian {
    float mu, s;
    Gaussian() : mu(0), s(1) {}
    Gaussian(float mu, float s) : mu(mu), s(s) {}

    float pdf(float x) const {
        return std::exp(-std::pow(x - mu, 2) / (2 * s)) / std::sqrt(float(2 * M_PI) * s);
    }

    float sample(PRNG &prng) const {
        float a = prng();
        float b = prng();
        gaussian2(a, b);
        return a * std::sqrt(s) + mu;
    }
};

struct Reibold {
    Reibold();
    void draw();

    struct Path {
        float x0, x1;
        float contribution;
    };

    struct ExamplePath {
        Path guidePath;
        Path samplePath;
        Gaussian x0;
        Gaussian x1;
        Gaussian conditionalX1;
        float contribution;

        struct Neighbor {
            Path path;
            float distance;

            Neighbor() {}
            Neighbor(const Path &path, float distance) : path(path), distance(distance) {}
            bool operator<(const Neighbor &other) const { return distance < other.distance; }
        };
        std::vector<Neighbor> guideNN;
    };

private:
    ExamplePath examplePath;
    void sampleExamplePath();

    Sampling *sampling;

    void drawScene();
    [[nodiscard]] bool isPathBlocked(float x0, float x1) const;

    bool showImguiDemo = false;

    struct SceneConfiguration {
        float holeWidth = 0.01f;
        float holeDistance = 0.2f;
        int numHoles = 5;
    } scene;

    struct Setup {
        int numRays = 4096;
        int numIterations = 6;
        bool showDensity = false;
        bool showExamplePath = false;
        bool showPaths = true;
        float exposure = -2;
        float pathExposure = -2;

        float showMinX = -1;
        float showMaxX = +1;
    } setup;

    std::vector<int> iterationLog;

    struct Pixel {
        float mu = 0;
        float s = 0;

        void resetStatistics() {
            mu = 0;
            s = 0;
        }

        void splat(float v) {
            mu += v;
            s += v * v;
        }

        void finishStatistics(int N) {
            const float w = float(1) / float(N);
            mu *= w;
            s = w * s - mu * mu;
        }
    };
    std::vector<Pixel> image;

    void clearImage() {
        for (auto &pixel : image) {
            pixel.resetStatistics();
        }
    }

    void splatImage(float x0, float I) {
        x0 = (x0 + 1) / 2;
        const int x = std::clamp(int(std::floor(x0 * float(image.size()))), 0, int(image.size()) - 1);
        auto &pixel = image[x];
        pixel.splat(I);
    }

    void finishImage(int N) {
        for (auto &pixel : image) {
            pixel.finishStatistics(N);
        }
    }

    void iterationReibold();
    void saveSVG();

    PRNG prng;
    std::vector<Path> paths;

    void sampleRay();
};

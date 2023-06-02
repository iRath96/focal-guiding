#pragma once

#include <visualizer/core/math.hpp>
#include <visualizer/core/AABB.hpp>
#include <visualizer/core/PRNG.hpp>

#include <focal_guiding.h>

#include <vector>
#include <array>

struct Env3D {
    static constexpr int Dimensionality = 3;

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
        return Float(2.0 * M_PI * M_PI) * (tFar * tFar - tNear * tNear);
    }

    static Point absolute(const AABB &aabb, const Point &relative) {
        return aabb.absolute(relative);
    }

    static Point relative(const AABB &aabb, const Point &absolute) {
        return aabb.relative(absolute);
    }

    static void extend(AABB &aabb, const Point &p) {
        aabb.extend(p);
    }
};

struct Mesh {
    typedef std::array<float, 3> Vertex;
    typedef std::array<int, 3> Triangle;

    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;

    Mesh() {}
    Mesh(std::istream &file);

    struct Slice {
        typedef std::array<float, 2> Point;

        std::vector<int> polygons;
        std::vector<Point> points;
    };

    Slice slice(int dim, float value) const;

private:
    void loadPly(std::istream &file);
};

struct Visualizer {
    Visualizer();
    void draw();

private:
    focal_guiding::Orthtree<Env3D> octree;
    
    float exposure = -12;

    int sliceAxis = 1;
    float sliceValue = 0;

    float gridOutlineOpacity = 0;

    float volumeNorm = 1;
    float viewScale = 1;
    std::array<float, 2> viewShift { 0, 0 };

    float dynamicRange = 30;

    void updateGrid();
    void drawGrid();
    void saveSVG(const char *filename);

    void unloadScene();
    void loadScene(size_t id);

    struct Patch {
        Env3D::AABB domain;
        Float density;
    };
    std::vector<Patch> patches;

    Mesh mesh;
    Mesh::Slice slice;

    size_t scene;
    size_t req_scene;
};

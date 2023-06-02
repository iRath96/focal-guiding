#pragma once

#include <visualizer/rt/Line.hpp>
#include <visualizer/rt/Sensor.hpp>
#include <visualizer/core/AABB.hpp>
#include <visualizer/rt/Ray.hpp>
#include <visualizer/core/PRNG.hpp>
#include <visualizer/ThreadPool.hpp>
#include <visualizer/Halton.hpp>
#include <visualizer/Uniform.hpp>

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
        return Float(2.0 * M_PI * M_PI) * (tFar * tFar - tNear * tNear);
    }

    static Point absolute(const AABB &aabb, const Point &relative) {
        return aabb.absolute(relative);
    }

    static Point relative(const AABB &aabb, const Point &absolute) {
        return aabb.relative(absolute);
    }
};

class Scene {
public:
    focal_guiding::UniformDirection<Env2D> uniform;
    focal_guiding::Orthtree<Env2D> quadTree;
    bool guidingReady;
    bool iterativeNarrowing{false};
    bool splatCamera{true};

    Scene() {
        guidingReady = false;
    }

    void addLine(const Line &line) {
        m_lines.push_back(line);
        m_aabb.extend(line.start);
        m_aabb.extend(line.start + line.span);
    }

    void addSensor(const Sensor &sensor) {
        m_sensors.push_back(sensor);
        m_aabb.extend(sensor.position);
    }

    void simulate(int Nsps) {
        quadTree.configuration.splattingStrategy = iterativeNarrowing ?
            quadTree.SPLAT_RAY_WEIGHTED :
            quadTree.SPLAT_RAY;

        quadTree.setAABB(m_aabb);

        for (auto &sensor: m_sensors) {
            simulateSensor(sensor, Nsps, uniform);
        }

        quadTree.build();
        guidingReady = true;
    }

    void simulateGuided(int Nsps) {
        quadTree.configuration.splattingStrategy = iterativeNarrowing ?
            quadTree.SPLAT_RAY_WEIGHTED :
            quadTree.SPLAT_RAY;

        for (auto &sensor: m_sensors) {
            simulateSensor(sensor, Nsps, quadTree);
        }

        quadTree.build();
    }

    [[nodiscard]] const std::vector<Line> &lines() const { return m_lines; }

    [[nodiscard]] AABB<Vec2f> &aabb() { return m_aabb; }

    [[nodiscard]] const AABB<Vec2f> &aabb() const { return m_aabb; }

    [[nodiscard]] const std::vector<Sensor> &sensors() const { return m_sensors; }

private:
    mutable PRNG prng;

    template<typename Sampler>
    Vec2f mixtureSample(const Vec2f &o, Env2D::PRNG &prng, const Sampler &sampler, Float &pdf, Float &guidePdf) {
        Float uniformProb = 0.5f;
        Vec2f d;
        if (prng() < uniformProb) {
            d = uniform.sample(o, prng);
        } else {
            d = sampler.sample(o, prng);
        }

        guidePdf = sampler.pdf(o, d);
        pdf = uniformProb * uniform.pdf(o, d) + (1 - uniformProb) * guidePdf;
        guidePdf += sampler.pdf(o, d * -1);
        return d;
    }

    template<typename Sampler>
    Float measure(Ray &ray, Float throughput, const Sampler &sampler, Env2D::PRNG &localPRNG, int maxDepth, Float pdf) {
        Float contribution = 0;
        if (maxDepth-- <= 0) {
            return contribution;
        }

        auto isect = intersect(ray);
        if (isect.valid()) {
            contribution = throughput * m_lines[isect.line].emission;
            if (m_lines[isect.line].reflect) {
                throughput /= 2 * M_PI;

                Float samplePdf, guidePdf;

                Ray nestedRay;
                nestedRay.o = ray(isect.t);
                nestedRay.d = mixtureSample(nestedRay.o, localPRNG, sampler, samplePdf, guidePdf);
                nestedRay.o += nestedRay.d * 1e-3;
                throughput /= samplePdf;

                //guidePdf = samplePdf;
                contribution += measure(nestedRay, throughput, sampler, localPRNG, maxDepth, guidePdf);
            }

            if (contribution > 0) {
                Vec2f projectedOrigin = ray.o;
                Vec2f projectedDir = ray.d;
                for (int dim = 0; dim < 2; dim++) {
                    if (ray.d[dim] < 0) {
                        projectedOrigin[dim] = (aabb().max[dim] + aabb().min[dim]) - projectedOrigin[dim];
                        projectedDir[dim] = -ray.d[dim];
                    }
                }

                const Float tNear = ((aabb().min - projectedOrigin) / projectedDir).max();
                const Float tFar = ((aabb().max - projectedOrigin) / projectedDir).min();

                if (pdf > 0) {
                    quadTree.splat(ray.o, ray.d, 1e+1, contribution, pdf, localPRNG);
                }
            }
        }

        return contribution;
    }

    template<typename Sampler>
    void simulateSensor(Sensor &sensor, int Nsps, const Sampler &sampler) {
        std::mutex statsMutex;
        double firstMoment = 0;
        double secondMoment = 0;
        long sampleCount = 0;

        //ThreadPool::get().parallel([&](int)
        {
            double localFirstMoment = 0;
            double localSecondMoment = 0;
            const int localSampleCount = Nsps;
            PRNG localPRNG{};
            localPRNG.index = rand();

            {
                std::unique_lock lock(statsMutex);
                //localPRNG.index = sampleCount;
                sampleCount += localSampleCount;
            }

            for (int n = 0; n < localSampleCount; n++) {
                Float throughput(1);

                Float samplePdf, guidePdf;
                Ray ray;
                ray.o = sensor.position;
                //ray.o.x() += 0.005f * (2 * localPRNG() - 1);
                //ray.o.y() += 0.005f * (2 * localPRNG() - 1);
                ray.d = mixtureSample(ray.o, localPRNG, sampler, samplePdf, guidePdf);
                throughput /= samplePdf;

                if (!splatCamera) {
                    guidePdf = 0;
                }
                //guidePdf = samplePdf;
                const Float contribution = measure(ray, throughput, sampler, localPRNG, 2, guidePdf);
                localFirstMoment += contribution;
                localSecondMoment += contribution * contribution;

                //localPRNG.advance();
            }

            {
                std::unique_lock lock(statsMutex);
                firstMoment += localFirstMoment;
                secondMoment += localSecondMoment;
            }
        }
        //);

        sensor.estimate = Float(firstMoment / double(sampleCount));
        sensor.variance = Float(secondMoment / double(sampleCount) - sensor.estimate * sensor.estimate);
    }

    Intersection intersect(Ray &ray) const {
        Intersection isect;
        for (size_t i = 0; i < m_lines.size(); i++) {
            auto &line = m_lines[i];

            const Float det = line.span.x() * ray.d.y() - line.span.y() * ray.d.x();
            if (det == 0) continue;

            const Float idet = 1 / det;

            const Vec2f shift = line.start - ray.o;
            const Float t = idet * Vec2f{-line.span.y(), line.span.x()}.dot(shift);
            if (t > isect.t || t < 0) continue;

            const Float l = idet * Vec2f{-ray.d.y(), ray.d.x()}.dot(shift);
            if (l > 1 || l < 0) continue;

            isect.t = t;
            isect.line = int(i);
        }
        return isect;
    }

    AABB<Vec2f> m_aabb;
    std::vector<Line> m_lines;
    std::vector<Sensor> m_sensors;
};

#pragma once

#include <visualizer/core/platform.hpp>
#include <initializer_list>

#include <cmath>
#include <algorithm>

template<typename Float, int N>
struct Vector {
    static constexpr int Dimensionality = N;

    Float el[N];

    Vector() {
        for (int i = 0; i < N; i++) {
            el[i] = 0;
        }
    }

    explicit Vector(Float v) {
        for (int i = 0; i < N; i++) {
            el[i] = v;
        }
    }

    Vector(std::initializer_list<Float> l) {
        //static_assert(l.size() == N, "incorrect number of components to Vector constructor");

        auto it = l.begin();
        for (int i = 0; i < N; i++) {
            el[i] = *it;
            it++;
        }
    }

    MTL_THREAD Float &operator[](int i) {
        return el[i];
    }

    MTL_THREAD const Float &operator[](int i) const {
        return el[i];
    }

#define MAKE_ACCESSOR(name, index) \
MTL_THREAD Float &name() { \
    static_assert(index < N, "out of bounds"); \
    return this->el[index]; \
} \
MTL_THREAD const Float &name() const { \
    static_assert(index < N, "out of bounds"); \
    return this->el[index]; \
}

    MAKE_ACCESSOR(x, 0)

    MAKE_ACCESSOR(y, 1)

    MAKE_ACCESSOR(z, 2)

    MAKE_ACCESSOR(w, 3)

#undef MAKE_ACCESSOR

    Float lengthSquared() const {
        return dot(*this);
    }

    Float length() const {
        return std::sqrt(lengthSquared());
    }

    Float dot(MTL_THREAD const Vector<Float, N> &other) const {
        Float sum(0);
        for (int i = 0; i < N; i++) {
            sum += el[i] * other[i];
        }
        return sum;
    }

    Vector normalized() const {
        return *this / length();
    }

    Float sum() const {
        Float result(0);
        for (int i = 0; i < N; i++) {
            result += el[i];
        }
        return result;
    }

    MTL_THREAD Vector &operator*=(MTL_THREAD const Float &other) {
        for (int i = 0; i < N; i++) {
            el[i] *= other;
        }
        return *this;
    }

    MTL_THREAD Vector &operator*=(MTL_THREAD const Vector &other) {
        for (int i = 0; i < N; i++) {
            el[i] *= other[i];
        }
        return *this;
    }

    MTL_THREAD Vector &operator/=(MTL_THREAD const Vector &other) {
        for (int i = 0; i < N; i++) {
            el[i] /= other[i];
        }
        return *this;
    }

    MTL_THREAD Vector &operator/=(MTL_THREAD const Float &other) {
        return (*this *= Float(1) / other);
    }

    MTL_THREAD Vector &operator+=(MTL_THREAD const Vector &other) {
        for (int i = 0; i < N; i++) {
            el[i] += other[i];
        }
        return *this;
    }

    MTL_THREAD Vector &operator-=(MTL_THREAD const Vector &other) {
        for (int i = 0; i < N; i++) {
            el[i] -= other[i];
        }
        return *this;
    }

    MTL_THREAD Vector operator*(MTL_THREAD const Float &other) const {
        Vector copy = *this;
        copy *= other;
        return copy;
    }

    MTL_THREAD Vector operator*(MTL_THREAD const Vector &other) const {
        Vector copy = *this;
        copy *= other;
        return copy;
    }

    MTL_THREAD Vector operator/(MTL_THREAD const Vector &other) const {
        Vector copy = *this;
        copy /= other;
        return copy;
    }

    MTL_THREAD Vector operator/(MTL_THREAD const Float &other) const {
        Vector copy = *this;
        copy /= other;
        return copy;
    }

    Vector operator+(MTL_THREAD const Vector &other) const {
        Vector copy = *this;
        copy += other;
        return copy;
    }

    Vector operator-(MTL_THREAD const Vector &other) const {
        Vector copy = *this;
        copy -= other;
        return copy;
    }

    Vector operator-() const {
        Vector copy;
        for (int i = 0; i < N; i++) {
            copy(i) = -(*this)(i);
        }
        return copy;
    }

    friend Vector operator*(MTL_THREAD const Float &lhs, Vector rhs) {
        return rhs *= lhs;
    }

    bool operator==(MTL_THREAD const Vector &other) const {
        for (int i = 0; i < N; i++) {
            if ((*this)(i) != other(i)) {
                return false;
            }
        }
        return true;
    }

    bool operator!=(MTL_THREAD const Vector &other) const {
        return !(*this == other);
    }

    [[nodiscard]] int argmin() const {
        return std::distance(std::begin(el), std::min_element(std::begin(el), std::end(el)));
    }

    [[nodiscard]] int argmax() const {
        return std::distance(std::begin(el), std::max_element(std::begin(el), std::end(el)));
    }

    [[nodiscard]] Float min() const {
        return *std::min_element(std::begin(el), std::end(el));
    }

    [[nodiscard]] Float min(int &arg) const {
        auto it = std::min_element(std::begin(el), std::end(el));
        arg = std::distance(std::begin(el), it);
        return *it;
    }

    [[nodiscard]] Float max() const {
        return *std::max_element(std::begin(el), std::end(el));
    }

    [[nodiscard]] Float max(int &arg) const {
        auto it = std::max_element(std::begin(el), std::end(el));
        arg = std::distance(std::begin(el), it);
        return *it;
    }
};

using Float = float;
using Vec2f = Vector<Float, 2>;

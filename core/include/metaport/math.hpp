// Metaport — basic 3D math used across the tracking stack.
// Convention: right-handed, +Y up, -Z forward (same as OpenXR).
#pragma once

#include <algorithm>
#include <cmath>

namespace metaport {

struct Vec3 {
    float x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    float length() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        float l = length();
        return l > 1e-9f ? *this / l : Vec3{0, 0, 0};
    }
};

inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) { return a + (b - a) * t; }

struct Quat {
    // Hamilton convention, (x, y, z, w) like XrQuaternionf.
    float x = 0, y = 0, z = 0, w = 1;

    constexpr Quat() = default;
    constexpr Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static Quat identity() { return {}; }

    static Quat fromAxisAngle(const Vec3& axis, float angle) {
        Vec3 a = axis.normalized();
        float s = std::sin(angle * 0.5f);
        return {a.x * s, a.y * s, a.z * s, std::cos(angle * 0.5f)};
    }

    // Rotation vector (axis * angle, radians) -> quaternion. Used for gyro integration.
    static Quat fromRotationVector(const Vec3& rv) {
        float angle = rv.length();
        if (angle < 1e-9f) return Quat{rv.x * 0.5f, rv.y * 0.5f, rv.z * 0.5f, 1.0f}.normalized();
        return fromAxisAngle(rv / angle, angle);
    }

    // Shortest rotation that takes direction `from` to direction `to`.
    static Quat fromTo(const Vec3& from, const Vec3& to) {
        Vec3 f = from.normalized(), t = to.normalized();
        float d = f.dot(t);
        if (d < -0.999999f) {
            Vec3 axis = Vec3{1, 0, 0}.cross(f);
            if (axis.length() < 1e-6f) axis = Vec3{0, 1, 0}.cross(f);
            return fromAxisAngle(axis, 3.14159265358979f);
        }
        Vec3 c = f.cross(t);
        return Quat{c.x, c.y, c.z, 1.0f + d}.normalized();
    }

    // Builds an orientation whose local -Z points along `forward` and +Y is as close to `up` as possible.
    static Quat lookRotation(const Vec3& forward, const Vec3& up) {
        Vec3 z = (-forward).normalized();
        Vec3 x = up.cross(z).normalized();
        if (x.length() < 1e-6f) x = Vec3{0, 1, 0}.cross(z).normalized();
        if (x.length() < 1e-6f) x = Vec3{1, 0, 0};
        Vec3 y = z.cross(x);
        return fromBasis(x, y, z);
    }

    // Columns of the rotation matrix are the rotated X, Y, Z axes.
    static Quat fromBasis(const Vec3& X, const Vec3& Y, const Vec3& Z) {
        float m00 = X.x, m01 = Y.x, m02 = Z.x;
        float m10 = X.y, m11 = Y.y, m12 = Z.y;
        float m20 = X.z, m21 = Y.z, m22 = Z.z;
        float tr = m00 + m11 + m22;
        Quat q;
        if (tr > 0) {
            float s = std::sqrt(tr + 1.0f) * 2;
            q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
        } else if (m00 > m11 && m00 > m22) {
            float s = std::sqrt(1.0f + m00 - m11 - m22) * 2;
            q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
        } else if (m11 > m22) {
            float s = std::sqrt(1.0f + m11 - m00 - m22) * 2;
            q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
        } else {
            float s = std::sqrt(1.0f + m22 - m00 - m11) * 2;
            q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
        }
        return q.normalized();
    }

    Quat operator*(const Quat& b) const {
        const Quat& a = *this;
        return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
    }

    Quat conjugate() const { return {-x, -y, -z, w}; }
    Quat inverse() const { return conjugate(); }  // unit quaternions only
    float dot(const Quat& o) const { return x * o.x + y * o.y + z * o.z + w * o.w; }

    Quat normalized() const {
        float l = std::sqrt(x * x + y * y + z * z + w * w);
        return l > 1e-9f ? Quat{x / l, y / l, z / l, w / l} : Quat{};
    }

    Vec3 rotate(const Vec3& v) const {
        Vec3 u{x, y, z};
        Vec3 t = u.cross(v) * 2.0f;
        return v + t * w + u.cross(t);
    }

    // Rotation vector (axis * angle) of this quaternion, angle in [0, pi].
    Vec3 toRotationVector() const {
        Quat q = w < 0 ? Quat{-x, -y, -z, -w} : *this;
        Vec3 v{q.x, q.y, q.z};
        float s = v.length();
        if (s < 1e-9f) return v * 2.0f;
        float angle = 2.0f * std::atan2(s, q.w);
        return v * (angle / s);
    }

    float angleTo(const Quat& o) const {
        float d = std::fabs(dot(o));
        return 2.0f * std::acos(std::min(1.0f, d));
    }

    // Splits this rotation into swing * twist where twist is around `axis`.
    Quat twist(const Vec3& axis) const {
        Vec3 a = axis.normalized();
        Vec3 r{x, y, z};
        Vec3 p = a * r.dot(a);
        Quat t{p.x, p.y, p.z, w};
        float l = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z + t.w * t.w);
        if (l < 1e-9f) return Quat{};
        return t.normalized();
    }
};

inline Quat slerp(const Quat& a, Quat b, float t) {
    float d = a.dot(b);
    if (d < 0) { b = {-b.x, -b.y, -b.z, -b.w}; d = -d; }
    if (d > 0.9995f) {
        return Quat{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t,
                    a.w + (b.w - a.w) * t}.normalized();
    }
    float theta = std::acos(d);
    float s = std::sin(theta);
    float wa = std::sin((1 - t) * theta) / s, wb = std::sin(t * theta) / s;
    return Quat{a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb};
}

struct Pose {
    Quat orientation;
    Vec3 position;

    Pose operator*(const Pose& child) const {
        return {orientation * child.orientation, position + orientation.rotate(child.position)};
    }
    Pose inverse() const {
        Quat inv = orientation.inverse();
        return {inv, inv.rotate(-position)};
    }
    Vec3 transform(const Vec3& p) const { return position + orientation.rotate(p); }
};

// Frame-rate independent exponential smoothing factor: fraction of the gap closed after dt
// for a filter with time constant tau.
inline float expAlpha(float dt, float tau) {
    if (tau <= 0) return 1.0f;
    return 1.0f - std::exp(-dt / tau);
}

inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

}  // namespace metaport

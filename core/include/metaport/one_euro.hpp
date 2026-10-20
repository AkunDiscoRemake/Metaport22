// One Euro filter (Casiez et al., CHI 2012): low jitter when still, low lag when moving.
// Used on raw hand landmarks, which are noisy frame to frame.
#pragma once

#include <cmath>

#include "metaport/math.hpp"

namespace metaport {

class OneEuroFilter {
public:
    explicit OneEuroFilter(float minCutoff = 1.0f, float beta = 0.007f, float dCutoff = 1.0f)
        : minCutoff_(minCutoff), beta_(beta), dCutoff_(dCutoff) {}

    void reset() { initialized_ = false; }

    float filter(float value, float dt) {
        if (!initialized_ || dt <= 0) {
            initialized_ = true;
            prev_ = value;
            dPrev_ = 0;
            return value;
        }
        float dx = (value - prev_) / dt;
        dPrev_ = dPrev_ + alpha(dt, dCutoff_) * (dx - dPrev_);
        float cutoff = minCutoff_ + beta_ * std::fabs(dPrev_);
        prev_ = prev_ + alpha(dt, cutoff) * (value - prev_);
        return prev_;
    }

private:
    static float alpha(float dt, float cutoff) {
        float tau = 1.0f / (2.0f * 3.14159265358979f * cutoff);
        return 1.0f / (1.0f + tau / dt);
    }

    float minCutoff_, beta_, dCutoff_;
    bool initialized_ = false;
    float prev_ = 0, dPrev_ = 0;
};

class OneEuroVec3 {
public:
    explicit OneEuroVec3(float minCutoff = 1.0f, float beta = 0.007f)
        : fx_(minCutoff, beta), fy_(minCutoff, beta), fz_(minCutoff, beta) {}
    void reset() { fx_.reset(); fy_.reset(); fz_.reset(); }
    Vec3 filter(const Vec3& v, float dt) { return {fx_.filter(v.x, dt), fy_.filter(v.y, dt), fz_.filter(v.z, dt)}; }

private:
    OneEuroFilter fx_, fy_, fz_;
};

}  // namespace metaport

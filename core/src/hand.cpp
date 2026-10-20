#include "metaport/hand.hpp"

#include <cmath>

namespace metaport {

namespace {

// Solves a symmetric 3x3 system with Cramer's rule.
bool solve3(const float A[3][3], const float b[3], float x[3]) {
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (std::fabs(det) < 1e-12f) return false;
    for (int c = 0; c < 3; ++c) {
        float M[3][3];
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k) M[r][k] = (k == c) ? b[r] : A[r][k];
        float d = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                  M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                  M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
        x[c] = d / det;
    }
    return true;
}

// OpenCV camera axes (x right, y down, z forward) -> OpenXR camera axes (x right, y up, z back).
Vec3 cvToXr(const Vec3& v) { return {v.x, -v.y, -v.z}; }

Quat boneOrientation(const Vec3& from, const Vec3& to, const Vec3& dorsal) {
    Vec3 fwd = to - from;
    if (fwd.length() < 1e-6f) fwd = Vec3{0, 0, -1};
    return Quat::lookRotation(fwd, dorsal);
}

}  // namespace

bool solveHandTranslation(const HandObservation& obs, const CameraIntrinsics& K, Vec3* translationCv) {
    float A[3][3] = {};
    float b[3] = {};
    for (int i = 0; i < kLandmarkCount; ++i) {
        const Vec3& P = obs.metric[i];
        float a = (obs.image[i].x * K.width - K.cx) / K.fx;
        float c = (obs.image[i].y * K.height - K.cy) / K.fy;
        // Row [1, 0, -a] . T = a*Z - X
        float rhs1 = a * P.z - P.x;
        A[0][0] += 1; A[0][2] += -a; A[2][0] += -a; A[2][2] += a * a;
        b[0] += rhs1; b[2] += -a * rhs1;
        // Row [0, 1, -c] . T = c*Z - Y
        float rhs2 = c * P.z - P.y;
        A[1][1] += 1; A[1][2] += -c; A[2][1] += -c; A[2][2] += c * c;
        b[1] += rhs2; b[2] += -c * rhs2;
    }
    float t[3];
    if (!solve3(A, b, t) || t[2] <= 0.05f) return false;

    if (obs.wristDepthM > 0.05f) {
        // Depth sensor/ARCore depth is less scale-ambiguous; average the two estimates, then
        // re-fit x/y for that depth.
        float tzDepth = obs.wristDepthM - obs.metric[kWrist].z;
        t[2] = 0.5f * (t[2] + tzDepth);
        float sx = 0, sy = 0;
        for (int i = 0; i < kLandmarkCount; ++i) {
            const Vec3& P = obs.metric[i];
            float a = (obs.image[i].x * K.width - K.cx) / K.fx;
            float c = (obs.image[i].y * K.height - K.cy) / K.fy;
            sx += a * (P.z + t[2]) - P.x;
            sy += c * (P.z + t[2]) - P.y;
        }
        t[0] = sx / kLandmarkCount;
        t[1] = sy / kLandmarkCount;
    }
    *translationCv = {t[0], t[1], t[2]};
    return true;
}

bool observationPalmPosition(const HandObservation& obs, const CameraIntrinsics& K, const Pose& worldFromCamera,
                             Vec3* out) {
    Vec3 t;
    if (!solveHandTranslation(obs, K, &t)) return false;
    Vec3 palmCv = lerp(obs.metric[kWrist], obs.metric[kMiddleMcp], 0.5f) + t;
    *out = worldFromCamera.transform(cvToXr(palmCv));
    return true;
}

void buildXrJoints(const std::array<Vec3, kLandmarkCount>& p, Side side, HandJoint out[kHandJointCount]) {
    const Vec3& wrist = p[kWrist];
    Vec3 across = (p[kIndexMcp] - wrist).cross(p[kPinkyMcp] - wrist);
    Vec3 dorsal = (side == kRight ? -across : across).normalized();
    float handSize = (p[kMiddleMcp] - wrist).length();  // ~0.09 m for an adult hand
    float rScale = clampf(handSize / 0.09f, 0.6f, 1.5f);

    auto set = [&](int j, const Vec3& pos, const Vec3& toward, float radius) {
        out[j].position = pos;
        out[j].orientation = boneOrientation(pos, toward, dorsal);
        out[j].radius = radius * rScale;
    };
    auto setTip = [&](int j, const Vec3& pos, const Vec3& prev, float radius) {
        out[j].position = pos;
        out[j].orientation = boneOrientation(prev, pos, dorsal);
        out[j].radius = radius * rScale;
    };

    Vec3 palm = lerp(wrist, p[kMiddleMcp], 0.5f);
    set(kJPalm, palm, p[kMiddleMcp], 0.025f);
    set(kJWrist, wrist, p[kMiddleMcp], 0.02f);

    set(kJThumbMetacarpal, p[kThumbCmc], p[kThumbMcp], 0.013f);
    set(kJThumbProximal, p[kThumbMcp], p[kThumbIp], 0.011f);
    set(kJThumbDistal, p[kThumbIp], p[kThumbTip], 0.010f);
    setTip(kJThumbTip, p[kThumbTip], p[kThumbIp], 0.008f);

    struct Finger { int mcp, xrMeta; float r; };
    const Finger fingers[4] = {
        {kIndexMcp, kJIndexMetacarpal, 0.010f},
        {kMiddleMcp, kJMiddleMetacarpal, 0.010f},
        {kRingMcp, kJRingMetacarpal, 0.009f},
        {kPinkyMcp, kJLittleMetacarpal, 0.008f},
    };
    for (const Finger& f : fingers) {
        Vec3 meta = lerp(wrist, p[f.mcp], 0.1f);
        set(f.xrMeta, meta, p[f.mcp], f.r * 1.1f);
        set(f.xrMeta + 1, p[f.mcp], p[f.mcp + 1], f.r);
        set(f.xrMeta + 2, p[f.mcp + 1], p[f.mcp + 2], f.r * 0.9f);
        set(f.xrMeta + 3, p[f.mcp + 2], p[f.mcp + 3], f.r * 0.85f);
        setTip(f.xrMeta + 4, p[f.mcp + 3], p[f.mcp + 2], f.r * 0.8f);
    }
}

float pinchStrength(const HandJoint j[kHandJointCount], const HandTrackerConfig& cfg) {
    float d = (j[kJThumbTip].position - j[kJIndexTip].position).length();
    return clampf(1.0f - (d - cfg.pinchStart) / (cfg.pinchEnd - cfg.pinchStart), 0.0f, 1.0f);
}

HandTracker::HandTracker(const HandTrackerConfig& cfg) : cfg_(cfg) {
    for (auto& side : filters_)
        for (auto& f : side) f = OneEuroVec3(cfg.jointMinCutoff, cfg.jointBeta);
}

void HandTracker::onObservation(const HandObservation& obs, const CameraIntrinsics& K,
                                const Pose& worldFromCamera, const Pose& head) {
    if (obs.score < cfg_.minScore) return;
    Vec3 t;
    if (!solveHandTranslation(obs, K, &t)) return;

    HandState& h = hands_[obs.side];
    float dt = lastSeen_[obs.side] ? static_cast<float>(obs.timeNs - lastSeen_[obs.side]) * 1e-9f : 0.0f;
    if (dt <= 0 || dt > cfg_.lostTimeoutS) {
        for (auto& f : filters_[obs.side]) f.reset();
        dt = 0;
    }

    std::array<Vec3, kLandmarkCount> world;
    for (int i = 0; i < kLandmarkCount; ++i) {
        Vec3 cam = cvToXr(obs.metric[i] + t);
        world[i] = filters_[obs.side][i].filter(worldFromCamera.transform(cam), dt);
    }

    buildXrJoints(world, obs.side, h.joints);
    h.pinchStrength = pinchStrength(h.joints, cfg_);
    h.confidence = obs.score;
    h.timeNs = obs.timeNs;
    h.active = 1;

    // Aim ray: from an estimated shoulder through the pinch point, like system hand pointers.
    Vec3 pinchPoint = lerp(h.joints[kJIndexProximal].position, h.joints[kJThumbProximal].position, 0.5f);
    Vec3 shoulder = head.transform({obs.side == kRight ? 0.18f : -0.18f, -0.20f, 0.05f});
    Vec3 dir = (pinchPoint - shoulder).normalized();
    h.aim.position = pinchPoint;
    h.aim.orientation = Quat::lookRotation(dir, {0, 1, 0});
    h.aim.timeNs = obs.timeNs;

    lastSeen_[obs.side] = obs.timeNs;
}

void HandTracker::update(int64_t nowNs) {
    for (int s = 0; s < 2; ++s) {
        if (hands_[s].active && static_cast<float>(nowNs - lastSeen_[s]) * 1e-9f > cfg_.lostTimeoutS) {
            hands_[s].active = 0;
            hands_[s].pinchStrength = 0;
        }
    }
}

bool HandTracker::palmPose(Side s, Pose* out) const {
    if (!hands_[s].active) return false;
    out->position = hands_[s].joints[kJPalm].position;
    out->orientation = hands_[s].joints[kJPalm].orientation;
    return true;
}

}  // namespace metaport

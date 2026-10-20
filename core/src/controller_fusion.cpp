#include "metaport/controller_fusion.hpp"

#include <cmath>

namespace metaport {

Quat palmToGripRotation(Side side) {
    // Columns: grip X, Y, Z axes in palm coordinates.
    if (side == kRight) return Quat::fromBasis({0, -1, 0}, {0, 0, -1}, {1, 0, 0});
    return Quat::fromBasis({0, 1, 0}, {0, 0, -1}, {-1, 0, 0});
}

ControllerFusion::ControllerFusion(Side side, const ControllerFusionConfig& cfg)
    : side_(side), cfg_(cfg), palmToGripRotation_((palmToGripRotation(side) * cfg.gripRotationTweak).normalized()) {}

void ControllerFusion::reset() {
    imu_.reset();
    yawAlign_ = Quat{};
    position_ = velocity_ = pending_ = {};
    lastImuNs_ = lastHandNs_ = lastHandArrivalNs_ = 0;
    havePosition_ = yawAligned_ = false;
    source_ = kPosNone;
    history_.clear();
}

Vec3 ControllerFusion::armModel(const Pose& head) const {
    float sign = side_ == kRight ? 1.0f : -1.0f;
    Quat headYaw = head.orientation.twist({0, 1, 0});
    Vec3 shoulder = head.position + headYaw.rotate({sign * 0.17f, -0.22f, 0.05f});
    Vec3 elbow = shoulder + headYaw.rotate({0.0f, -0.26f, -0.08f});
    return elbow + worldOrientation().rotate({0.0f, 0.0f, -0.24f});
}

void ControllerFusion::onImu(const ImuSample& s, const Pose& head) {
    imu_.onSample(s);
    if (!imu_.initialized()) return;
    float dt = lastImuNs_ ? clampf(static_cast<float>(s.timeNs - lastImuNs_) * 1e-9f, 0.0f, 0.05f) : 0.0f;
    lastImuNs_ = s.timeNs;

    if (!havePosition_) {
        position_ = armModel(head);
        havePosition_ = true;
    }

    float sinceHand = lastHandArrivalNs_ ? static_cast<float>(s.timeNs - lastHandArrivalNs_) * 1e-9f : 1e9f;
    if (sinceHand < cfg_.inertialWindowS) {
        source_ = sinceHand < 0.05f ? kPosHandTracked : kPosInertial;
        Vec3 accel = yawAlign_.rotate(imu_.linearAccelWorld());
        velocity_ += accel * dt;
        velocity_ *= std::exp(-dt / cfg_.velocityDampingTau);
        position_ += velocity_ * dt;
    } else {
        source_ = kPosArmModel;
        pending_ = {};
        Vec3 before = position_;
        position_ = lerp(position_, armModel(head), expAlpha(dt, cfg_.armModelBlendTau));
        velocity_ = dt > 0 ? (position_ - before) / dt : Vec3{};
    }

    // Drain the pending correction, speed-limited.
    Vec3 apply = pending_;
    float maxStep = cfg_.maxCorrectionSpeed * dt;
    if (apply.length() > maxStep) apply = apply.normalized() * maxStep;
    if (apply.length() > 0) {
        position_ += apply;
        pending_ -= apply;
        history_.applyAll([&](Snapshot& sn) { sn.position += apply; });
    }

    history_.push(s.timeNs, {position_, worldOrientation()});
}

float ControllerFusion::distanceToPalm(int64_t timeNs, const Vec3& palmPosition) const {
    Snapshot past;
    if (!history_.lookup(timeNs, &past)) return 1e9f;
    return (palmPosition - past.position).length();
}

bool ControllerFusion::onHandObservation(int64_t timeNs, const Pose& palm, float confidence) {
    if (!imu_.initialized() || confidence < cfg_.minConfidence) return false;

    Vec3 measured = palm.transform(cfg_.palmToGripOffset);
    Snapshot past;
    if (!history_.lookup(timeNs, &past)) {
        position_ = measured;
        velocity_ = pending_ = {};
        lastHandNs_ = timeNs;
        lastHandArrivalNs_ = lastImuNs_;
        return true;
    }

    // `pending_` is correction already decided but not yet applied to the history.
    Vec3 error = measured - (past.position + pending_);
    bool wasTracking = source_ == kPosHandTracked || source_ == kPosInertial;
    bool settled = pending_.length() < 0.05f;
    if (wasTracking && settled && error.length() > cfg_.maxHandDistanceM) return false;

    pending_ += error * (cfg_.positionGain * confidence);
    // Only small errors say something about velocity; big ones are re-acquisition offsets.
    if (wasTracking && error.length() < 0.1f) velocity_ += error * (cfg_.velocityGain * confidence);

    // Yaw: the IMU cannot observe it, the palm direction can (noisily). Only the twist about
    // gravity is taken from the hand, tilt stays with the accelerometer.
    Quat measuredGrip = (palm.orientation * palmToGripRotation_).normalized();
    Quat worldError = (measuredGrip * past.orientation.inverse()).normalized();
    Quat yawError = worldError.twist({0, 1, 0});
    float gain = yawAligned_ ? cfg_.yawGain * confidence : 1.0f;
    Quat stepYaw = slerp(Quat{}, yawError, gain);
    yawAlign_ = (stepYaw * yawAlign_).normalized();
    history_.applyAll([&](Snapshot& sn) { sn.orientation = (stepYaw * sn.orientation).normalized(); });
    yawAligned_ = true;

    lastHandNs_ = timeNs;
    lastHandArrivalNs_ = lastImuNs_;
    return true;
}

void ControllerFusion::recenter(const Pose& head) {
    if (!imu_.initialized()) return;
    Vec3 fc = worldOrientation().rotate({0, 0, -1});
    Vec3 fh = head.orientation.rotate({0, 0, -1});
    fc.y = 0;
    fh.y = 0;
    if (fc.length() < 0.1f || fh.length() < 0.1f) return;  // pointing straight up/down
    yawAlign_ = (Quat::fromTo(fc, fh) * yawAlign_).normalized();
    yawAligned_ = true;
}

PoseSample ControllerFusion::grip() const {
    PoseSample s;
    s.orientation = worldOrientation();
    s.position = position_;
    s.linearVelocity = velocity_;
    s.angularVelocity = s.orientation.rotate(imu_.angularVelocityBody());
    s.timeNs = lastImuNs_;
    return s;
}

}  // namespace metaport

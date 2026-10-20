#include "metaport/head_tracker.hpp"

namespace metaport {

void HeadTracker::onGyro(int64_t timeNs, const Vec3& omegaBody) {
    if (lastGyroNs_ != 0 && timeNs > lastGyroNs_) {
        float dt = clampf(static_cast<float>(timeNs - lastGyroNs_) * 1e-9f, 0.0f, 0.05f);
        orientation_ = (orientation_ * Quat::fromRotationVector(omegaBody * dt)).normalized();
    }
    omegaWorld_ = orientation_.rotate(omegaBody);
    lastGyroNs_ = timeNs;
    orientationHistory_.push(timeNs, orientation_);
}

void HeadTracker::onArPose(int64_t timeNs, const Pose& worldFromCamera) {
    Quat past;
    bool havePast = orientationHistory_.lookup(timeNs, &past);

    if (!haveAr_ || !havePast) {
        orientation_ = worldFromCamera.orientation;
        orientationHistory_.clear();
        orientationHistory_.push(timeNs, orientation_);
    } else {
        Quat error = (worldFromCamera.orientation * past.inverse()).normalized();
        float angle = Quat{}.angleTo(error);
        float gain = angle > cfg_.snapAngleRad ? 1.0f : cfg_.orientationGain;
        Quat step = slerp(Quat{}, error, gain);
        orientation_ = (step * orientation_).normalized();
        orientationHistory_.applyAll([&](Quat& q) { q = (step * q).normalized(); });
    }

    if (haveAr_ && timeNs > arTimeNs_) {
        float dt = static_cast<float>(timeNs - arTimeNs_) * 1e-9f;
        Vec3 delta = worldFromCamera.position - arPosition_;
        if (delta.length() > cfg_.snapDistanceM || dt > 0.25f) {
            velocity_ = {};  // relocalisation jump or long gap: don't turn it into velocity
        } else {
            Vec3 measured = delta / dt;
            velocity_ = lerp(velocity_, measured, expAlpha(dt, cfg_.velocityTau));
        }
    }

    arPosition_ = worldFromCamera.position;
    arTimeNs_ = timeNs;
    haveAr_ = true;
    arTracking_ = true;
}

void HeadTracker::onArTrackingLost() {
    arTracking_ = false;
    velocity_ = {};
}

Pose HeadTracker::cameraPose() const {
    Pose p{orientation_, arPosition_};
    if (arTracking_ && lastGyroNs_ > arTimeNs_) {
        float dt = clampf(static_cast<float>(lastGyroNs_ - arTimeNs_) * 1e-9f, 0.0f, 0.1f);
        p.position += velocity_ * dt;
    }
    return p;
}

PoseSample HeadTracker::headSample() const {
    Pose head = cameraPose() * cfg_.cameraToHead;
    PoseSample s;
    s.orientation = head.orientation;
    s.position = head.position;
    // The head point is offset from the camera, so rotation adds tangential velocity.
    Vec3 lever = orientation_.rotate(cfg_.cameraToHead.position);
    s.linearVelocity = (arTracking_ ? velocity_ : Vec3{}) + omegaWorld_.cross(lever);
    s.angularVelocity = omegaWorld_;
    s.timeNs = lastGyroNs_ != 0 ? lastGyroNs_ : arTimeNs_;
    return s;
}

}  // namespace metaport

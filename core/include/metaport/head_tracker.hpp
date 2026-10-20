// Head tracking: ARCore 6DoF poses (~30 Hz, delayed) fused with the phone gyro (200+ Hz).
//
// ARCore gives drift-free position and orientation but at camera rate and with latency.
// Rendering from it directly judders in a headset. The gyro is integrated at full rate and
// ARCore pulls the result back toward truth (with latency compensation), and position is
// extrapolated between camera frames from a filtered velocity.
#pragma once

#include <cstdint>

#include "metaport/history.hpp"
#include "metaport/math.hpp"
#include "metaport/shared_state.hpp"

namespace metaport {

struct HeadTrackerConfig {
    // Fraction of the orientation error removed per ARCore frame.
    float orientationGain = 0.25f;
    // Errors above this are treated as an ARCore relocalisation and applied immediately.
    float snapAngleRad = 0.35f;
    float snapDistanceM = 0.30f;
    // Time constant of the ARCore velocity estimate.
    float velocityTau = 0.06f;
    // Pose of the eye midpoint relative to the ARCore camera (the phone's rear camera).
    // Default: ~9 cm behind and ~2 cm above the lens for a typical Cardboard viewer.
    Pose cameraToHead{Quat{}, Vec3{0.0f, 0.02f, 0.09f}};
};

class HeadTracker {
public:
    explicit HeadTracker(const HeadTrackerConfig& cfg = {}) : cfg_(cfg) {}

    // Gyro sample in the ARCore camera's body frame (rad/s).
    void onGyro(int64_t timeNs, const Vec3& omegaBody);

    // ARCore camera pose (Camera.getPose()) at the camera frame's timestamp.
    void onArPose(int64_t timeNs, const Pose& worldFromCamera);

    // ARCore lost tracking: keep orientation from the gyro, freeze position.
    void onArTrackingLost();

    bool is6Dof() const { return arTracking_; }
    PoseSample headSample() const;
    // Current world_from_camera, position extrapolated to the latest gyro time.
    Pose cameraPose() const;

private:
    HeadTrackerConfig cfg_;
    Quat orientation_;       // world_from_camera
    Vec3 arPosition_;        // last ARCore camera position (at arTimeNs_)
    Vec3 velocity_;
    Vec3 omegaWorld_;
    int64_t lastGyroNs_ = 0;
    int64_t arTimeNs_ = 0;
    bool haveAr_ = false;
    bool arTracking_ = false;
    History<Quat, 128> orientationHistory_;
};

}  // namespace metaport

// 6DoF controller from a 3DoF Joy-Con plus camera hand tracking.
//
// The Joy-Con IMU gives fast, smooth orientation (but no position and drifting yaw). The camera
// sees the hand holding it and gives absolute position (but at ~30 Hz, delayed, noisy and only
// while the hand is in view). Fusing them:
//
//  * Orientation: IMU only, with yaw slowly aligned to the hand's palm direction while visible.
//  * Position: predicted at IMU rate from gravity-free acceleration (short-term only, heavily
//    damped), corrected by each hand observation against the state at the observation's
//    timestamp (latency compensation). Corrections are speed-limited, so re-acquiring a hand
//    after a gap glides back instead of teleporting.
//  * Hand lost: inertial for a short gap, then a smooth blend to an arm model anchored on the
//    head, so the controller never flies away or freezes in mid-air.
#pragma once

#include <cstdint>

#include "metaport/history.hpp"
#include "metaport/joycon.hpp"
#include "metaport/math.hpp"
#include "metaport/shared_state.hpp"

namespace metaport {

// OpenXR defines the grip pose as: +X normal to the palm (into it for the right hand, out of it
// for the left), -Z through the tube formed by the closed fingers from little finger to thumb.
// Palm joints use -Z toward the fingers and +Y out of the back of the hand.
Quat palmToGripRotation(Side side);

struct ControllerFusionConfig {
    // Grip position relative to the palm joint while holding a Joy-Con (palm frame: -Z toward
    // fingers, +Y out of the back of the hand). The Joy-Con sits ~3 cm inside the fist.
    Vec3 palmToGripOffset{0.0f, -0.03f, -0.02f};
    // Extra rotation applied after the standard palm->grip mapping, to tune for how the user
    // holds the Joy-Con (identity = held vertically, stick under the thumb).
    Quat gripRotationTweak{};

    float positionGain = 0.6f;        // fraction of position error applied per observation
    float velocityGain = 4.0f;        // 1/s, velocity correction from the same error
    float maxCorrectionSpeed = 2.5f;  // m/s, caps how fast a large error is absorbed
    float yawGain = 0.05f;            // per observation, yaw alignment toward the palm
    float velocityDampingTau = 0.25f; // s, leak on dead-reckoned velocity
    float inertialWindowS = 0.15f;    // after losing the hand, trust inertial prediction
    float armModelBlendTau = 0.35f;   // s, convergence toward the arm model afterwards
    float minConfidence = 0.5f;
    float maxHandDistanceM = 0.25f;   // hand farther than this from prediction → reject (mis-ID)
};

class ControllerFusion {
public:
    explicit ControllerFusion(Side side, const ControllerFusionConfig& cfg = {});

    void onImu(const ImuSample& s, const Pose& head);
    // Palm pose of the hand holding this controller, world space, at the camera timestamp.
    // Returns false if the hand was rejected (too far from where the controller must be, e.g. the
    // controller was put down or the other hand was mislabelled).
    bool onHandObservation(int64_t timeNs, const Pose& palm, float confidence);
    // Distance between a palm position and where this controller's grip was at `timeNs`.
    float distanceToPalm(int64_t timeNs, const Vec3& palmPosition) const;
    // Sets yaw so the controller points along the head's forward direction.
    void recenter(const Pose& head);
    void reset();

    bool connected() const { return imu_.initialized(); }
    PoseSample grip() const;
    PositionSource positionSource() const { return source_; }

private:
    struct Snapshot {
        Vec3 position;
        Quat orientation;
    };

    Quat worldOrientation() const { return (yawAlign_ * imu_.orientation()).normalized(); }
    Vec3 armModel(const Pose& head) const;

    Side side_;
    ControllerFusionConfig cfg_;
    Quat palmToGripRotation_;  // grip axes in the palm joint frame, incl. the user tweak
    JoyConImu imu_;
    Quat yawAlign_;
    Vec3 position_;
    Vec3 velocity_;
    // Correction not applied yet; drained at maxCorrectionSpeed so large errors glide out.
    Vec3 pending_;
    int64_t lastImuNs_ = 0;
    int64_t lastHandNs_ = 0;        // camera timestamp of the last accepted observation
    int64_t lastHandArrivalNs_ = 0; // IMU time when it arrived (observations are delayed)
    bool havePosition_ = false;
    bool yawAligned_ = false;
    PositionSource source_ = kPosNone;
    History<Snapshot, 256> history_;
};

}  // namespace metaport

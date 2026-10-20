// Hand tracking: converts 21-point hand landmarks seen by the phone's rear camera into world-space
// XR_EXT_hand_tracking joints (26 per hand).
//
// Metaport does not bundle a hand detector. Landmarks come from a HandLandmarkProvider plugged in
// by the service; the planned provider is a bridge to the hand tracking component of the user's
// own Horizon OS firmware (see docs/PORT.md). Without a provider, hands are simply reported as
// not tracked and Joy-Cons fall back to the arm model.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "metaport/math.hpp"
#include "metaport/one_euro.hpp"
#include "metaport/shared_state.hpp"

namespace metaport {

constexpr int kLandmarkCount = 21;

// Landmark indices (the common 21-point hand topology: wrist + 4 per finger).
enum Landmark : int {
    kWrist = 0,
    kThumbCmc = 1, kThumbMcp = 2, kThumbIp = 3, kThumbTip = 4,
    kIndexMcp = 5, kIndexPip = 6, kIndexDip = 7, kIndexTip = 8,
    kMiddleMcp = 9, kMiddlePip = 10, kMiddleDip = 11, kMiddleTip = 12,
    kRingMcp = 13, kRingPip = 14, kRingDip = 15, kRingTip = 16,
    kPinkyMcp = 17, kPinkyPip = 18, kPinkyDip = 19, kPinkyTip = 20,
};

// XrHandJointEXT indices.
enum XrJoint : int {
    kJPalm = 0, kJWrist = 1,
    kJThumbMetacarpal = 2, kJThumbProximal = 3, kJThumbDistal = 4, kJThumbTip = 5,
    kJIndexMetacarpal = 6, kJIndexProximal = 7, kJIndexIntermediate = 8, kJIndexDistal = 9, kJIndexTip = 10,
    kJMiddleMetacarpal = 11, kJMiddleProximal = 12, kJMiddleIntermediate = 13, kJMiddleDistal = 14, kJMiddleTip = 15,
    kJRingMetacarpal = 16, kJRingProximal = 17, kJRingIntermediate = 18, kJRingDistal = 19, kJRingTip = 20,
    kJLittleMetacarpal = 21, kJLittleProximal = 22, kJLittleIntermediate = 23, kJLittleDistal = 24, kJLittleTip = 25,
};

struct CameraIntrinsics {
    float fx = 500, fy = 500, cx = 320, cy = 240;  // pixels
    int width = 640, height = 480;
};

// One detected hand, as delivered by a landmark provider for one camera image.
struct HandObservation {
    int64_t timeNs = 0;          // camera image timestamp
    Side side = kRight;          // handedness as seen by the user (not mirrored)
    float score = 0;             // handedness/presence confidence
    // Normalised image coordinates (0..1) in the camera image's native orientation.
    std::array<Vec3, kLandmarkCount> image{};
    // Metric landmarks around the hand's centre, camera axes (x right, y down, z away).
    std::array<Vec3, kLandmarkCount> metric{};
    // Optional ARCore depth at the wrist pixel, metres (<= 0 when unavailable).
    float wristDepthM = 0;
};

// Source of hand landmarks for each camera frame. Implementations live outside the core (the
// planned one bridges the hand tracking component of the user's own firmware). Returns the hands
// found in the frame; an empty vector simply means no hands this frame.
class HandLandmarkProvider {
public:
    virtual ~HandLandmarkProvider() = default;
    virtual const char* name() const = 0;
    // `rgba` is the camera image in its native orientation, `stride` in bytes.
    virtual std::vector<HandObservation> detect(int64_t timeNs, const uint8_t* rgba, int width, int height,
                                                int stride) = 0;
};

struct HandTrackerConfig {
    float minScore = 0.6f;
    float jointMinCutoff = 1.5f;   // One Euro params for joint positions
    float jointBeta = 0.5f;
    float pinchStart = 0.015f;     // thumb-index tip distance at full pinch (m)
    float pinchEnd = 0.055f;       // distance where pinch strength reaches 0 (m)
    float lostTimeoutS = 0.2f;
};

// Solves the hand's camera-space translation from the metric landmarks and their image
// projections (translation-only least squares; the metric frame is camera aligned).
// Returns false if the system is degenerate.
bool solveHandTranslation(const HandObservation& obs, const CameraIntrinsics& K, Vec3* translationCv);

// Unfiltered world-space palm position of an observation (used to decide which controller a
// hand belongs to before it is fed to the filters).
bool observationPalmPosition(const HandObservation& obs, const CameraIntrinsics& K, const Pose& worldFromCamera,
                             Vec3* out);

// Builds the 26 OpenXR joints in the camera's OpenXR-convention frame (x right, y up, -z forward)
// from 21 camera-space points in the same frame.
void buildXrJoints(const std::array<Vec3, kLandmarkCount>& pts, Side side, HandJoint out[kHandJointCount]);

// Pinch strength 0..1 from thumb/index tip distance.
float pinchStrength(const HandJoint joints[kHandJointCount], const HandTrackerConfig& cfg);

class HandTracker {
public:
    explicit HandTracker(const HandTrackerConfig& cfg = {});

    // `worldFromCamera` is the ARCore camera pose at the same timestamp as the observation.
    void onObservation(const HandObservation& obs, const CameraIntrinsics& K, const Pose& worldFromCamera,
                       const Pose& head);

    // Called every tick; marks hands inactive once they have not been seen for a while.
    void update(int64_t nowNs);

    const HandState& state(Side s) const { return hands_[s]; }

    // Palm pose in world space (position, orientation with -Z toward fingers, +Y dorsal).
    bool palmPose(Side s, Pose* out) const;

private:
    HandTrackerConfig cfg_;
    HandState hands_[2];
    std::array<OneEuroVec3, kLandmarkCount> filters_[2];
    int64_t lastSeen_[2] = {0, 0};
};

}  // namespace metaport

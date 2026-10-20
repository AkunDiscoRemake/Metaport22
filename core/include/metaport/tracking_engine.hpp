// Owns every tracker and produces the TrackingFrame published to OpenXR apps.
// All inputs arrive from the Android service (ARCore thread, sensor thread, hand-landmark
// provider thread); calls are serialised by the caller (see jni_bridge.cpp).
#pragma once

#include <cstdint>

#include "metaport/controller_fusion.hpp"
#include "metaport/hand.hpp"
#include "metaport/head_tracker.hpp"
#include "metaport/history.hpp"
#include "metaport/shared_state.hpp"

namespace metaport {

struct EngineConfig {
    HeadTrackerConfig head;
    HandTrackerConfig hands;
    ControllerFusionConfig controllers;
    // A hand with no Joy-Con on its side drives a virtual Touch controller (pinch = trigger,
    // fist = squeeze) so controller-only apps remain usable.
    bool emulateControllerFromHand = true;
};

class TrackingEngine {
public:
    explicit TrackingEngine(const EngineConfig& cfg = {});

    void onPhoneGyro(int64_t timeNs, const Vec3& omegaCameraFrame);
    void onArPose(int64_t timeNs, const Pose& worldFromCamera);
    void onArTrackingLost();
    void onFloorPlane(float y) { floorY_ = y; }

    void onHandObservation(const HandObservation& obs, const CameraIntrinsics& K);

    void onJoyConImu(Side side, const ImuSample& s);
    // Buttons/axes already mapped to Touch semantics (ControllerState fields other than grip).
    void onJoyConInput(Side side, uint32_t buttons, float trigger, float squeeze, float thumbX, float thumbY);
    void onJoyConDisconnected(Side side);

    void setDisplayInfo(const DisplayInfo& d) { display_ = d; }
    // Derives FOV and eye resolution from the viewer/screen optics and stores them.
    void configureDisplay(const ViewerParams& viewer, const ScreenParams& screen, float ipdMeters, float refreshHz);

    // Makes the current head yaw "forward" and points both Joy-Cons along it.
    void recenter();

    TrackingFrame buildFrame(int64_t nowNs);

private:
    Pose headPose() const;

    EngineConfig cfg_;
    HeadTracker head_;
    HandTracker hands_;
    ControllerFusion controllers_[2];
    ControllerState input_[2];
    bool joyConConnected_[2] = {false, false};  // IMU samples arriving
    bool joyConButtons_[2] = {false, false};    // buttons/sticks arriving (IMU may be unavailable)
    bool handHolding_[2] = {false, false};
    History<Pose, 32> cameraPoses_;
    Pose worldFix_;  // recenter transform applied to everything on output
    DisplayInfo display_;
    float floorY_ = 0;
};

}  // namespace metaport

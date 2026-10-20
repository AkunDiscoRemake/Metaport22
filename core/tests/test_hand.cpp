#include "hand_fixture.hpp"
#include "metaport/controller_fusion.hpp"
#include "test.hpp"

using namespace metaport;

TEST(hand_translation_recovers_depth) {
    CameraIntrinsics K;
    auto xr = fixture::rightHandXr(-0.45f, 0.05f, -0.02f);
    HandObservation obs = fixture::observe(xr, K, kRight);
    Vec3 t;
    EXPECT_TRUE(solveHandTranslation(obs, K, &t));
    Vec3 wristCv = obs.metric[kWrist] + t;
    EXPECT_NEAR(wristCv.x, 0.05, 1e-3);
    EXPECT_NEAR(wristCv.y, 0.02, 1e-3);
    EXPECT_NEAR(wristCv.z, 0.45, 1e-3);
}

TEST(hand_joints_axes_follow_openxr_convention) {
    auto xr = fixture::rightHandXr();
    HandJoint j[kHandJointCount];
    buildXrJoints(xr, kRight, j);
    // Back of the hand faces the camera (+z), fingers point up (+y).
    Vec3 dorsal = j[kJPalm].orientation.rotate({0, 1, 0});
    Vec3 toFingers = j[kJPalm].orientation.rotate({0, 0, -1});
    EXPECT_NEAR(dorsal.z, 1, 0.05);
    EXPECT_NEAR(toFingers.y, 1, 0.05);
    EXPECT_NEAR(j[kJIndexTip].position.y, 0.165, 1e-6);
}

TEST(left_hand_mirror_has_same_dorsal_rule) {
    auto xr = fixture::rightHandXr();
    for (auto& v : xr) v.x = -v.x;  // mirror: left hand, back still facing the camera
    HandJoint j[kHandJointCount];
    buildXrJoints(xr, kLeft, j);
    EXPECT_NEAR(j[kJPalm].orientation.rotate({0, 1, 0}).z, 1, 0.05);
}

TEST(grip_axes_match_openxr_definition) {
    auto xr = fixture::rightHandXr();
    HandJoint j[kHandJointCount];
    buildXrJoints(xr, kRight, j);
    Quat grip = j[kJPalm].orientation * palmToGripRotation(kRight);
    // Right hand: +X into the palm (palm faces away from camera = -z), -Z from little finger to thumb (-x).
    EXPECT_NEAR(grip.rotate({1, 0, 0}).z, -1, 0.1);
    EXPECT_NEAR(grip.rotate({0, 0, -1}).x, -1, 0.1);
}

TEST(pinch_strength) {
    auto xr = fixture::rightHandXr();
    HandJoint j[kHandJointCount];
    HandTrackerConfig cfg;
    buildXrJoints(xr, kRight, j);
    EXPECT_NEAR(pinchStrength(j, cfg), 0, 1e-6);
    xr[kThumbTip] = xr[kIndexTip] + metaport::Vec3{0.01f, 0, 0};
    buildXrJoints(xr, kRight, j);
    EXPECT_NEAR(pinchStrength(j, cfg), 1, 1e-6);
}

TEST(hand_tracker_world_space_and_timeout) {
    CameraIntrinsics K;
    HandTracker tracker;
    Pose camera{Quat::fromAxisAngle({0, 1, 0}, 0.5f), {1, 1.5f, 0}};
    auto xr = fixture::rightHandXr();
    HandObservation obs = fixture::observe(xr, K, kRight);
    obs.timeNs = 1'000'000'000;
    tracker.onObservation(obs, K, camera, camera);
    const HandState& h = tracker.state(kRight);
    EXPECT_TRUE(h.active);
    Vec3 expected = camera.transform(xr[kWrist]);
    EXPECT_NEAR((h.joints[kJWrist].position - expected).length(), 0, 2e-3);
    tracker.update(obs.timeNs + 500'000'000);
    EXPECT_TRUE(!tracker.state(kRight).active);
}

#include "metaport/controller_fusion.hpp"
#include "metaport/tracking_engine.hpp"
#include "test.hpp"

using namespace metaport;

namespace {

constexpr float kG = 9.80665f;
const Pose kHead{Quat{}, {0, 1.6f, 0}};

// IMU sample for a controller with world orientation `q` and world linear acceleration `a`.
ImuSample imuFor(int64_t t, const Quat& q, const Vec3& a = {}, const Vec3& omegaBody = {}) {
    return {t, q.inverse().rotate(a + Vec3{0, kG, 0}), omegaBody};
}

// Palm pose whose grip (palm * palmToGrip) has orientation `grip` and whose grip point is `pos`.
Pose palmFor(Side side, const Quat& grip, const Vec3& pos, const ControllerFusionConfig& cfg) {
    Pose palm;
    palm.orientation = (grip * palmToGripRotation(side).inverse()).normalized();
    palm.position = pos - palm.orientation.rotate(cfg.palmToGripOffset);
    return palm;
}

}  // namespace

TEST(fusion_converges_to_hand_position_and_yaw) {
    ControllerFusionConfig cfg;
    ControllerFusion c(kRight, cfg);
    Quat truth = Quat::fromAxisAngle({0, 1, 0}, 1.0f);  // yaw the IMU cannot know
    Vec3 target{0.2f, 1.2f, -0.4f};
    int64_t t = 0;
    for (int i = 0; i < 400; ++i) {  // 2 s at 200 Hz, hand seen at 30 Hz, 50 ms late
        c.onImu(imuFor(t, truth), kHead);
        if (i % 7 == 0 && t > 50'000'000) c.onHandObservation(t - 50'000'000, palmFor(kRight, truth, target, cfg), 0.9f);
        t += 5'000'000;
    }
    PoseSample g = c.grip();
    EXPECT_NEAR((g.position - target).length(), 0, 0.01);
    EXPECT_NEAR(g.orientation.angleTo(truth), 0, 0.02);
    EXPECT_TRUE(c.positionSource() == kPosHandTracked);
}

TEST(fusion_tracks_moving_hand_despite_latency) {
    ControllerFusionConfig cfg;
    ControllerFusion c(kRight, cfg);
    Quat q;
    Vec3 start{0.2f, 1.2f, -0.4f};
    const float speed = 0.5f;  // m/s along +x, constant
    int64_t t = 0;
    float maxErr = 0;
    for (int i = 0; i < 600; ++i) {
        float secs = t * 1e-9f;
        auto posAt = [&](int64_t ns) { return start + Vec3{speed * ns * 1e-9f, 0, 0}; };
        c.onImu(imuFor(t, q), kHead);  // constant velocity: zero acceleration
        if (i % 7 == 0 && t > 60'000'000) {
            int64_t obsT = t - 60'000'000;
            c.onHandObservation(obsT, palmFor(kRight, q, posAt(obsT), cfg), 0.9f);
        }
        if (secs > 1.5f) maxErr = std::max(maxErr, (c.grip().position - posAt(t)).length());
        t += 5'000'000;
    }
    EXPECT_TRUE(maxErr < 0.03f);
}

TEST(fusion_hand_lost_goes_to_arm_model_without_runaway) {
    ControllerFusionConfig cfg;
    ControllerFusion c(kRight, cfg);
    Quat q;
    Vec3 target{0.2f, 1.2f, -0.4f};
    int64_t t = 0;
    for (int i = 0; i < 200; ++i) {
        c.onImu(imuFor(t, q), kHead);
        if (i % 7 == 0) c.onHandObservation(t, palmFor(kRight, q, target, cfg), 0.9f);
        t += 5'000'000;
    }
    // Hand leaves the camera view and the accelerometer has a 0.3 m/s² bias: naive double
    // integration would drift ~6 m in 2 s.
    for (int i = 0; i < 400; ++i) {
        c.onImu(imuFor(t, q, {0.3f, 0, 0}), kHead);
        t += 5'000'000;
    }
    EXPECT_TRUE(c.positionSource() == kPosArmModel);
    Vec3 p = c.grip().position;
    EXPECT_TRUE((p - kHead.position).length() < 0.9f);  // still within arm's reach
    EXPECT_TRUE(c.grip().linearVelocity.length() < 0.5f);
}

TEST(fusion_reacquire_glides_instead_of_snapping) {
    ControllerFusionConfig cfg;
    ControllerFusion c(kRight, cfg);
    Quat q;
    int64_t t = 0;
    for (int i = 0; i < 400; ++i) {  // no hand at all: arm model
        c.onImu(imuFor(t, q), kHead);
        t += 5'000'000;
    }
    Vec3 far = c.grip().position + Vec3{0.5f, 0.3f, 0};
    float maxJump = 0;
    Vec3 prev = c.grip().position;
    for (int i = 0; i < 200; ++i) {
        c.onImu(imuFor(t, q), kHead);
        if (i % 7 == 0) c.onHandObservation(t, palmFor(kRight, q, far, cfg), 0.9f);
        Vec3 now = c.grip().position;
        maxJump = std::max(maxJump, (now - prev).length());
        prev = now;
        t += 5'000'000;
    }
    // Per 5 ms IMU tick: at most the correction speed limit plus normal motion.
    EXPECT_TRUE(maxJump <= cfg.maxCorrectionSpeed * 0.005f + 2e-3f);
    EXPECT_NEAR((c.grip().position - far).length(), 0, 0.02);
}

TEST(fusion_rejects_far_hand_while_tracking) {
    ControllerFusionConfig cfg;
    ControllerFusion c(kRight, cfg);
    Quat q;
    Vec3 target{0.2f, 1.2f, -0.4f};
    int64_t t = 0;
    for (int i = 0; i < 100; ++i) {
        c.onImu(imuFor(t, q), kHead);
        if (i % 7 == 0) c.onHandObservation(t, palmFor(kRight, q, target, cfg), 0.9f);
        t += 5'000'000;
    }
    c.onImu(imuFor(t, q), kHead);
    EXPECT_TRUE(!c.onHandObservation(t, palmFor(kRight, q, target + Vec3{-0.5f, 0, 0}, cfg), 0.9f));
}

TEST(engine_emulates_controller_from_hand) {
    TrackingEngine engine;
    TrackingFrame f = engine.buildFrame(0);
    EXPECT_TRUE(!f.controllers[kLeft].connected && !f.controllers[kRight].connected);
}

TEST(engine_recenter_zeroes_head_yaw_and_xz) {
    TrackingEngine engine;
    Pose cam{Quat::fromAxisAngle({0, 1, 0}, 1.2f), {2, 1.5f, -3}};
    engine.onPhoneGyro(1'000'000, {});
    engine.onArPose(1'000'000, cam);
    engine.onPhoneGyro(2'000'000, {});
    engine.recenter();
    TrackingFrame f = engine.buildFrame(2'000'000);
    Vec3 fwd = f.head.orientation.rotate({0, 0, -1});
    EXPECT_NEAR(fwd.x, 0, 1e-3);
    EXPECT_NEAR(f.head.position.x, 0, 1e-3);
    EXPECT_NEAR(f.head.position.z, 0, 1e-3);
}

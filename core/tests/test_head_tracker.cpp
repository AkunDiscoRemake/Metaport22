#include "metaport/head_tracker.hpp"
#include "test.hpp"

using namespace metaport;

TEST(head_gyro_fills_between_ar_frames) {
    HeadTrackerConfig cfg;
    cfg.cameraToHead = Pose{};
    HeadTracker h(cfg);
    const float rate = 1.0f;  // rad/s yaw
    int64_t t = 0;
    for (int i = 0; i <= 1000; ++i) {  // 1 s at 1 kHz
        h.onGyro(t, {0, rate, 0});
        if (i % 33 == 0 && t >= 40'000'000) {
            // ARCore frame describing the truth 40 ms in the past
            int64_t at = t - 40'000'000;
            h.onArPose(at, {Quat::fromAxisAngle({0, 1, 0}, rate * at * 1e-9f), {}});
        }
        t += 1'000'000;
    }
    Quat truth = Quat::fromAxisAngle({0, 1, 0}, rate * (t - 1'000'000) * 1e-9f);
    EXPECT_NEAR(h.headSample().orientation.angleTo(truth), 0, 0.01);
    EXPECT_NEAR(h.headSample().angularVelocity.y, rate, 1e-4);
}

TEST(head_corrects_gyro_drift_with_ar) {
    HeadTrackerConfig cfg;
    cfg.cameraToHead = Pose{};
    HeadTracker h(cfg);
    int64_t t = 0;
    for (int i = 0; i <= 3000; ++i) {
        h.onGyro(t, {0, 0.05f, 0});  // pure bias, head is still
        if (i % 33 == 0) h.onArPose(t, {Quat{}, {0, 1.6f, 0}});
        t += 1'000'000;
    }
    EXPECT_TRUE(h.headSample().orientation.angleTo(Quat{}) < 0.01f);
    EXPECT_NEAR(h.headSample().position.y, 1.6, 1e-4);
}

TEST(head_position_extrapolates_with_velocity) {
    HeadTrackerConfig cfg;
    cfg.cameraToHead = Pose{};
    HeadTracker h(cfg);
    int64_t t = 0;
    for (int i = 0; i <= 1000; ++i) {
        h.onGyro(t, {});
        if (i % 33 == 0) h.onArPose(t, {Quat{}, {0.3f * t * 1e-9f, 0, 0}});
        t += 1'000'000;
    }
    float truthX = 0.3f * (t - 1'000'000) * 1e-9f;
    EXPECT_NEAR(h.headSample().position.x, truthX, 0.003);
}

TEST(head_relocalisation_jump_does_not_create_velocity) {
    HeadTracker h;
    h.onGyro(0, {});
    h.onArPose(0, {Quat{}, {0, 0, 0}});
    h.onGyro(33'000'000, {});
    h.onArPose(33'000'000, {Quat{}, {2, 0, 0}});
    EXPECT_NEAR(h.headSample().linearVelocity.x, 0, 1e-6);
}

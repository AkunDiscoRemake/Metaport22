#include <cstring>

#include "metaport/joycon.hpp"
#include "test.hpp"

using namespace metaport;

namespace {
void put16(uint8_t* p, int16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
}  // namespace

TEST(joycon_report_buttons_and_stick) {
    uint8_t d[49] = {};
    d[0] = 0x30;
    d[3] = 0x08 | 0x80;  // A + ZR
    d[4] = 0x02 | 0x10;  // plus + home
    // Right stick fully right (x = center + max), y centered.
    uint16_t x = 2048 + 1400, y = 2048;
    d[9] = x & 0xFF;
    d[10] = ((x >> 8) & 0x0F) | ((y & 0x0F) << 4);
    d[11] = (y >> 4) & 0xFF;
    // IMU sample 0: accel 4096 LSB on z (~1 g: 4096 * 0.244 mg)
    put16(d + 13 + 4, 4096);

    JoyConReport r;
    StickCalibration cal;
    EXPECT_TRUE(parseJoyConReport(d, sizeof d, JoyConSide::kRight, 100'000'000, cal, &r));
    EXPECT_TRUE(r.buttons.a && r.buttons.zr && r.buttons.plus && r.buttons.home);
    EXPECT_TRUE(!r.buttons.b);
    EXPECT_NEAR(r.stickX, 1, 1e-5);
    EXPECT_NEAR(r.stickY, 0, 1e-5);
    // Raw z (up when lying flat) maps to grip +y.
    EXPECT_NEAR(r.imu[0].accel.y, 9.8, 0.1);
    EXPECT_TRUE(r.imu[0].timeNs == 90'000'000 && r.imu[2].timeNs == 100'000'000);

    ControllerState cs;
    mapJoyConToTouch(r, JoyConSide::kRight, &cs);
    EXPECT_TRUE((cs.buttons & kBtnA) && (cs.buttons & kBtnMenu) && (cs.buttons & kBtnSystem));
    EXPECT_NEAR(cs.trigger, 1, 0);
    EXPECT_TRUE(cs.buttons & kTouchThumbstick);
}

TEST(joycon_rejects_short_or_wrong_report) {
    uint8_t d[49] = {};
    d[0] = 0x21;
    JoyConReport r;
    EXPECT_TRUE(!parseJoyConReport(d, sizeof d, JoyConSide::kLeft, 0, {}, &r));
    d[0] = 0x30;
    EXPECT_TRUE(!parseJoyConReport(d, 12, JoyConSide::kLeft, 0, {}, &r));
}

TEST(joycon_imu_converges_to_tilt_and_learns_bias) {
    JoyConImu imu;
    // Controller pitched 30° nose-up, at rest, gyro with a constant 0.02 rad/s bias on x.
    Quat truth = Quat::fromAxisAngle({1, 0, 0}, 0.5236f);
    Vec3 accelBody = truth.inverse().rotate({0, 9.80665f, 0});
    Vec3 bias{0.02f, 0, 0};
    int64_t t = 0;
    for (int i = 0; i < 200 * 10; ++i) {  // 10 s at 200 Hz
        imu.onSample({t, accelBody, bias});
        t += 5'000'000;
    }
    Vec3 up = imu.orientation().inverse().rotate({0, 1, 0});
    EXPECT_NEAR(up.dot(accelBody.normalized()), 1, 1e-3);
    EXPECT_NEAR(imu.gyroBias().x, 0.02, 0.005);
    EXPECT_TRUE(imu.isStill());
    EXPECT_NEAR(imu.linearAccelWorld().length(), 0, 0.05);
}

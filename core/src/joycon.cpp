#include "metaport/joycon.hpp"

#include <cmath>

namespace metaport {

namespace {
constexpr float kGravity = 9.80665f;
constexpr float kAccelScale = 0.000244f * kGravity;               // ±8 G range, per LSB
constexpr float kGyroScale = 0.06103f * 3.14159265358979f / 180;  // ±2000 dps range, per LSB

int16_t le16(const uint8_t* p) { return static_cast<int16_t>(p[0] | (p[1] << 8)); }
}  // namespace

void JoyConImu::reset() {
    q_ = Quat{};
    bias_ = {};
    lastNs_ = 0;
    stillFor_ = 0;
    initialized_ = false;
}

void JoyConImu::onSample(const ImuSample& s) {
    if (!initialized_) {
        // Initial tilt straight from gravity.
        if (s.accel.length() < 1.0f) return;
        q_ = Quat::fromTo(s.accel, {0, 1, 0});
        lastNs_ = s.timeNs;
        initialized_ = true;
        return;
    }
    float dt = static_cast<float>(s.timeNs - lastNs_) * 1e-9f;
    lastNs_ = s.timeNs;
    if (dt <= 0 || dt > 0.1f) return;

    Vec3 gyro = s.gyro - bias_;
    float aNorm = s.accel.length();

    // Rest detection drives bias learning; moving controllers only get gentle tilt correction.
    bool still = gyro.length() < cfg_.stillGyro && std::fabs(aNorm - kGravity) < cfg_.stillAccelTol;
    stillFor_ = still ? stillFor_ + dt : 0.0f;

    Vec3 correction;
    if (aNorm > 1.0f) {
        // Expected gravity direction ("up") in the body frame vs. what the accelerometer reads.
        Vec3 upBody = q_.inverse().rotate({0, 1, 0});
        Vec3 err = (s.accel / aNorm).cross(upBody);
        // Trust the accelerometer less the further it is from 1 g (the controller is being swung).
        float trust = clampf(1.0f - std::fabs(aNorm - kGravity) / 4.0f, 0.0f, 1.0f);
        correction = err * (cfg_.kp * trust);
        if (isStill()) {
            bias_ -= err * (cfg_.ki * dt);
            bias_ += gyro * (0.5f * dt);  // at rest the true rate is ~0: pull bias toward it
        }
    }

    omega_ = gyro;
    q_ = (q_ * Quat::fromRotationVector((gyro + correction) * dt)).normalized();
    linAccel_ = q_.rotate(s.accel) - Vec3{0, kGravity, 0};
}

bool parseJoyConReport(const uint8_t* d, size_t len, JoyConSide side, int64_t reportTimeNs,
                       const StickCalibration& cal, JoyConReport* out) {
    if (len < 49 || d[0] != 0x30) return false;
    JoyConButtons& b = out->buttons;
    uint8_t r = d[3], s = d[4], l = d[5];
    b = {};
    b.y = r & 0x01; b.x = r & 0x02; b.b = r & 0x04; b.a = r & 0x08;
    b.r = r & 0x40; b.zr = r & 0x80;
    b.minus = s & 0x01; b.plus = s & 0x02; b.rStick = s & 0x04; b.lStick = s & 0x08;
    b.home = s & 0x10; b.capture = s & 0x20;
    b.down = l & 0x01; b.up = l & 0x02; b.right = l & 0x04; b.left = l & 0x08;
    b.l = l & 0x40; b.zl = l & 0x80;
    if (side == JoyConSide::kRight) { b.sr = r & 0x10; b.sl = r & 0x20; }
    else { b.sr = l & 0x10; b.sl = l & 0x20; }

    const uint8_t* st = side == JoyConSide::kLeft ? d + 6 : d + 9;
    uint16_t raw[2] = {static_cast<uint16_t>(st[0] | ((st[1] & 0x0F) << 8)),
                       static_cast<uint16_t>((st[1] >> 4) | (st[2] << 4))};
    float axis[2];
    for (int i = 0; i < 2; ++i) {
        float v = static_cast<float>(raw[i]) - cal.center[i];
        v = v >= 0 ? v / cal.maxDelta[i] : v / cal.minDelta[i];
        v = clampf(v, -1.0f, 1.0f);
        // Radial-ish deadzone per axis, rescaled so output still reaches ±1.
        float m = std::fabs(v);
        axis[i] = m < cal.deadzone ? 0.0f : std::copysign((m - cal.deadzone) / (1 - cal.deadzone), v);
    }
    out->stickX = axis[0];
    out->stickY = axis[1];

    // IMU axes (per the reverse-engineering notes, controller lying flat): x toward the rail
    // tip, y toward the rail side, z up. Remap into the remote-style grip frame used by Metaport,
    // where the Joy-Con is held vertically and points forward along its top edge.
    // The axis signs differ between hardware revisions; verify with the calibration screen.
    for (int i = 0; i < 3; ++i) {
        const uint8_t* p = d + 13 + i * 12;
        Vec3 a{le16(p) * kAccelScale, le16(p + 2) * kAccelScale, le16(p + 4) * kAccelScale};
        Vec3 g{le16(p + 6) * kGyroScale, le16(p + 8) * kGyroScale, le16(p + 10) * kGyroScale};
        float sideSign = side == JoyConSide::kRight ? 1.0f : -1.0f;
        auto remap = [&](const Vec3& v) { return Vec3{sideSign * v.y, v.z, -v.x}; };
        out->imu[i].accel = remap(a);
        out->imu[i].gyro = remap(g);
        out->imu[i].timeNs = reportTimeNs - (2 - i) * 5'000'000;
    }
    return true;
}

void mapJoyConToTouch(const JoyConReport& r, JoyConSide side, ControllerState* cs) {
    const JoyConButtons& b = r.buttons;
    uint32_t bits = 0;
    if (side == JoyConSide::kRight) {
        if (b.a) bits |= kBtnA;
        if (b.b || b.x) bits |= kBtnB;
        if (b.plus) bits |= kBtnMenu;
        if (b.home) bits |= kBtnSystem;
        if (b.rStick) bits |= kBtnThumbstick;
        cs->trigger = b.zr ? 1.0f : 0.0f;
        cs->squeeze = b.r ? 1.0f : 0.0f;
    } else {
        if (b.down || b.right) bits |= kBtnA;  // X
        if (b.up || b.left) bits |= kBtnB;     // Y
        if (b.minus) bits |= kBtnMenu;
        if (b.capture) bits |= kBtnSystem;
        if (b.lStick) bits |= kBtnThumbstick;
        cs->trigger = b.zl ? 1.0f : 0.0f;
        cs->squeeze = b.l ? 1.0f : 0.0f;
    }
    // Joy-Cons have no capacitive sensing; report "touched" whenever the thumb is on a control.
    if (bits & kBtnA) bits |= kTouchA;
    if (bits & kBtnB) bits |= kTouchB;
    if (r.stickX != 0 || r.stickY != 0 || (bits & kBtnThumbstick)) bits |= kTouchThumbstick;
    if (cs->trigger > 0) bits |= kTouchTrigger;
    cs->buttons = bits;
    cs->thumbX = r.stickX;
    cs->thumbY = r.stickY;
}

}  // namespace metaport

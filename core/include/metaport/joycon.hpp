// Joy-Con input: IMU orientation filter plus a parser for the standard full input report (0x30).
//
// On stock Android the Joy-Con is read through the kernel hid-nintendo driver (buttons/sticks as
// a gamepad, IMU through InputDevice.getSensorManager()), so the Kotlin side feeds
// `JoyConImu` directly. The raw report parser is used when /dev/hidraw is accessible
// (rooted devices, desktop debugging) and is based on public reverse-engineering notes
// (dekuNukem/Nintendo_Switch_Reverse_Engineering).
#pragma once

#include <cstddef>
#include <cstdint>

#include "metaport/math.hpp"
#include "metaport/shared_state.hpp"

namespace metaport {

struct ImuSample {
    int64_t timeNs = 0;
    Vec3 accel;  // m/s^2, includes gravity (reads +9.81 along "up" at rest), controller frame
    Vec3 gyro;   // rad/s, controller frame
};

// Controller frame used throughout: grip axes of an OpenXR controller held like a remote
// (x right, y up, -z forward along the pointing direction).

// Gravity-referenced orientation filter (Mahony) with automatic gyro-bias estimation while the
// controller rests. Yaw is unobservable from the IMU alone; it is aligned externally.
class JoyConImu {
public:
    struct Config {
        float kp = 1.5f;              // accelerometer correction gain
        float ki = 0.02f;             // bias integration gain
        float stillGyro = 0.04f;      // rad/s threshold for "at rest"
        float stillAccelTol = 0.6f;   // |a| - g tolerance for "at rest", m/s^2
        float stillTime = 0.6f;       // seconds before bias learning kicks in
    };

    JoyConImu() : JoyConImu(Config{}) {}
    explicit JoyConImu(const Config& cfg) : cfg_(cfg) {}

    void onSample(const ImuSample& s);
    void reset();

    bool initialized() const { return initialized_; }
    // world(gravity-aligned, arbitrary yaw)_from_controller
    Quat orientation() const { return q_; }
    Vec3 angularVelocityBody() const { return omega_; }
    // Acceleration in the gravity-aligned frame with gravity removed.
    Vec3 linearAccelWorld() const { return linAccel_; }
    bool isStill() const { return stillFor_ >= cfg_.stillTime; }
    Vec3 gyroBias() const { return bias_; }

private:
    Config cfg_;
    Quat q_;
    Vec3 bias_;
    Vec3 omega_;
    Vec3 linAccel_;
    int64_t lastNs_ = 0;
    float stillFor_ = 0;
    bool initialized_ = false;
};

// --- Raw HID report 0x30 -------------------------------------------------------------------

enum class JoyConSide { kLeft, kRight };

struct JoyConButtons {
    // Right Joy-Con
    bool y = false, x = false, b = false, a = false, r = false, zr = false;
    bool plus = false, home = false, rStick = false;
    // Left Joy-Con
    bool down = false, up = false, right = false, left = false, l = false, zl = false;
    bool minus = false, capture = false, lStick = false;
    // Rail buttons (present on both)
    bool sl = false, sr = false;
};

struct JoyConReport {
    JoyConButtons buttons;
    float stickX = 0, stickY = 0;  // -1..1 after calibration
    ImuSample imu[3];              // three 5 ms samples per report
};

struct StickCalibration {
    // Defaults close to factory values; real values come from SPI flash 0x603D/0x6046.
    uint16_t center[2] = {2048, 2048};
    uint16_t minDelta[2] = {1400, 1400};
    uint16_t maxDelta[2] = {1400, 1400};
    float deadzone = 0.08f;
};

// Parses a 49-byte 0x30 report. `reportTimeNs` is the host timestamp of the report; IMU samples
// are back-dated 5 ms apart. Returns false if this is not a full input report.
bool parseJoyConReport(const uint8_t* data, size_t len, JoyConSide side, int64_t reportTimeNs,
                       const StickCalibration& cal, JoyConReport* out);

// Maps Joy-Con buttons to the Touch-controller bitmask/axes in ControllerState.
void mapJoyConToTouch(const JoyConReport& r, JoyConSide side, ControllerState* cs);

}  // namespace metaport

// Layout of the shared-memory block between the Metaport tracking service (single writer)
// and the OpenXR runtime loaded inside each VR app's process (readers).
//
// The block is plain-old-data so it can live in an ASharedMemory region mapped by two processes.
// Consistency uses a seqlock: the writer bumps `seq` to an odd value, writes, then bumps it to
// even. Readers retry while the value is odd or changed during the copy.
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include "metaport/cardboard_lens.hpp"
#include "metaport/math.hpp"

namespace metaport {

constexpr uint32_t kSharedMagic = 0x4D505432;  // "MPT2"
constexpr uint32_t kSharedVersion = 5;
constexpr int kHandJointCount = 26;            // XR_HAND_JOINT_COUNT_EXT
constexpr int kHapticSlots = 2;

enum Side : int { kLeft = 0, kRight = 1 };

// Bit layout mirrors the inputs of /interaction_profiles/oculus/touch_controller.
enum ButtonBits : uint32_t {
    kBtnA = 1u << 0,        // right: A, left: X
    kBtnB = 1u << 1,        // right: B, left: Y
    kBtnMenu = 1u << 2,     // left only on Touch; Joy-Con "-" / "+"
    kBtnSystem = 1u << 3,   // Joy-Con Home / Capture: reserved for the Metaport overlay
    kBtnThumbstick = 1u << 4,
    kTouchA = 1u << 8,
    kTouchB = 1u << 9,
    kTouchThumbstick = 1u << 10,
    kTouchTrigger = 1u << 11,
};

// Where a controller's position currently comes from. Exposed so the runtime can mark
// XR_SPACE_LOCATION_POSITION_TRACKED_BIT honestly.
enum PositionSource : uint32_t {
    kPosNone = 0,
    kPosHandTracked = 1,   // camera sees the hand holding the Joy-Con
    kPosInertial = 2,      // short dead-reckoning gap after losing the hand
    kPosArmModel = 3,      // hand lost for a while: body model estimate
};

struct PoseSample {
    Quat orientation;
    Vec3 position;
    Vec3 linearVelocity;   // m/s, world
    Vec3 angularVelocity;  // rad/s, world
    int64_t timeNs = 0;    // CLOCK_MONOTONIC, same timebase as XrTime
};

struct ControllerState {
    PoseSample grip;
    uint32_t connected = 0;
    uint32_t orientationValid = 0;
    uint32_t positionSource = kPosNone;
    uint32_t poseFromHand = 0;       // pose comes from the hand (no Joy-Con IMU): aim = hand ray
    uint32_t inputFromHand = 0;      // no Joy-Con at all: pinch = trigger, fist = squeeze
    uint32_t buttons = 0;
    float trigger = 0;
    float squeeze = 0;
    float thumbX = 0, thumbY = 0;
};

struct HandJoint {
    Quat orientation;
    Vec3 position;
    float radius = 0.01f;
};

struct HandState {
    uint32_t active = 0;
    uint32_t holdingController = 0;  // hand is wrapped around a Joy-Con
    float pinchStrength = 0;         // thumb-index, 0..1
    float confidence = 0;
    int64_t timeNs = 0;
    HandJoint joints[kHandJointCount];
    PoseSample aim;                  // pointing ray for pinch-select
};

struct DisplayInfo {
    float ipdMeters = 0.063f;
    // Half-angles in radians, OpenXR XrFovf order: left, right, up, down (left/down negative).
    float fov[2][4] = {{-0.7f, 0.7f, 0.7f, -0.7f}, {-0.7f, 0.7f, 0.7f, -0.7f}};
    uint32_t eyeWidth = 1024, eyeHeight = 1024;
    float refreshHz = 60.0f;
    // Optics used by the compositor; fov/eye size above are derived from these.
    ViewerParams viewer;
    ScreenParams screen;
};

struct TrackingFrame {
    PoseSample head;
    uint32_t headTracking6Dof = 0;  // 0 while ARCore is initialising / lost: 3DoF from gyro only
    float floorHeight = 0;          // y of the detected floor plane in world space
    ControllerState controllers[2];
    HandState hands[2];
    DisplayInfo display;
};

// Written by the runtime, read by the service: rumble requests forwarded to the Joy-Cons.
struct HapticCommand {
    std::atomic<uint32_t> serial;
    float amplitude;
    float frequencyHz;
    int64_t durationNs;
};

struct SharedBlock {
    uint32_t magic;
    uint32_t version;
    std::atomic<uint32_t> seq;
    uint32_t pad_;
    TrackingFrame frame;
    HapticCommand haptics[kHapticSlots];
};

static_assert(std::atomic<uint32_t>::is_always_lock_free, "seqlock needs lock-free atomics for cross-process use");

inline void initSharedBlock(SharedBlock* b) {
    std::memset(static_cast<void*>(b), 0, sizeof(SharedBlock));
    b->magic = kSharedMagic;
    b->version = kSharedVersion;
    b->frame = TrackingFrame{};
}

inline void publishFrame(SharedBlock* b, const TrackingFrame& f) {
    uint32_t s = b->seq.load(std::memory_order_relaxed);
    b->seq.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    std::memcpy(static_cast<void*>(&b->frame), &f, sizeof(TrackingFrame));
    std::atomic_thread_fence(std::memory_order_release);
    b->seq.store(s + 2, std::memory_order_release);
}

// Returns false if the block is not a valid Metaport block or the writer kept it busy.
inline bool readFrame(const SharedBlock* b, TrackingFrame* out, int maxRetries = 64) {
    if (b->magic != kSharedMagic || b->version != kSharedVersion) return false;
    for (int i = 0; i < maxRetries; ++i) {
        uint32_t s1 = b->seq.load(std::memory_order_acquire);
        if (s1 & 1u) continue;
        std::memcpy(static_cast<void*>(out), &b->frame, sizeof(TrackingFrame));
        std::atomic_thread_fence(std::memory_order_acquire);
        uint32_t s2 = b->seq.load(std::memory_order_relaxed);
        if (s1 == s2) return true;
    }
    return false;
}

// Extrapolates a pose sample to `targetNs` using its velocities. Capped to avoid
// overshoot when a source stalls.
inline Pose predictPose(const PoseSample& s, int64_t targetNs, float maxSeconds = 0.05f) {
    float dt = clampf(static_cast<float>(targetNs - s.timeNs) * 1e-9f, 0.0f, maxSeconds);
    Pose p;
    p.position = s.position + s.linearVelocity * dt;
    p.orientation = (Quat::fromRotationVector(s.angularVelocity * dt) * s.orientation).normalized();
    return p;
}

}  // namespace metaport

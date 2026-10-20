#include "metaport/tracking_engine.hpp"

#include <cmath>

namespace metaport {

namespace {

PoseSample transformSample(const Pose& t, const PoseSample& s) {
    PoseSample o = s;
    o.orientation = (t.orientation * s.orientation).normalized();
    o.position = t.transform(s.position);
    o.linearVelocity = t.orientation.rotate(s.linearVelocity);
    o.angularVelocity = t.orientation.rotate(s.angularVelocity);
    return o;
}

// 0 = open hand, 1 = closed fist, from fingertip-to-palm distances.
float fistAmount(const HandState& h) {
    const Vec3& palm = h.joints[kJPalm].position;
    float size = (h.joints[kJMiddleProximal].position - h.joints[kJWrist].position).length();
    if (size < 1e-3f) return 0;
    float sum = 0;
    for (int tip : {kJIndexTip, kJMiddleTip, kJRingTip, kJLittleTip}) sum += (h.joints[tip].position - palm).length();
    float ratio = (sum / 4) / size;  // ~1.6 open, ~0.6 fist
    return clampf((1.4f - ratio) / 0.7f, 0.0f, 1.0f);
}

}  // namespace

TrackingEngine::TrackingEngine(const EngineConfig& cfg)
    : cfg_(cfg),
      head_(cfg.head),
      hands_(cfg.hands),
      controllers_{ControllerFusion(kLeft, cfg.controllers), ControllerFusion(kRight, cfg.controllers)} {
    configureDisplay(ViewerParams{}, ScreenParams{}, 0.063f, 60.0f);
}

Pose TrackingEngine::headPose() const {
    PoseSample s = head_.headSample();
    return {s.orientation, s.position};
}

void TrackingEngine::onPhoneGyro(int64_t timeNs, const Vec3& omega) { head_.onGyro(timeNs, omega); }

void TrackingEngine::onArPose(int64_t timeNs, const Pose& worldFromCamera) {
    head_.onArPose(timeNs, worldFromCamera);
    cameraPoses_.push(timeNs, worldFromCamera);
}

void TrackingEngine::onArTrackingLost() { head_.onArTrackingLost(); }

void TrackingEngine::onHandObservation(const HandObservation& obsIn, const CameraIntrinsics& K) {
    Pose cam;
    if (!cameraPoses_.lookup(obsIn.timeNs, &cam)) cam = head_.cameraPose();

    HandObservation obs = obsIn;
    Side other = obs.side == kLeft ? kRight : kLeft;

    // Hand detectors sometimes swap left/right. With both Joy-Cons tracked, trust geometry over the
    // label: a hand sitting on the other controller belongs to that controller.
    Vec3 palmPos;
    if (joyConConnected_[kLeft] && joyConConnected_[kRight] && observationPalmPosition(obs, K, cam, &palmPos)) {
        float dSame = controllers_[obs.side].distanceToPalm(obs.timeNs, palmPos);
        float dOther = controllers_[other].distanceToPalm(obs.timeNs, palmPos);
        if (dOther < 0.5f * dSame && dOther < cfg_.controllers.maxHandDistanceM) obs.side = other;
    }

    hands_.onObservation(obs, K, cam, headPose());
    const HandState& h = hands_.state(obs.side);
    if (!h.active || h.timeNs != obs.timeNs) return;  // rejected by the hand tracker

    if (joyConConnected_[obs.side]) {
        Pose palm;
        hands_.palmPose(obs.side, &palm);
        handHolding_[obs.side] = controllers_[obs.side].onHandObservation(obs.timeNs, palm, h.confidence);
    }
}

void TrackingEngine::onJoyConImu(Side side, const ImuSample& s) {
    joyConConnected_[side] = true;
    controllers_[side].onImu(s, headPose());
}

void TrackingEngine::onJoyConInput(Side side, uint32_t buttons, float trigger, float squeeze, float thumbX,
                                   float thumbY) {
    joyConButtons_[side] = true;
    ControllerState& in = input_[side];
    in.buttons = buttons;
    in.trigger = trigger;
    in.squeeze = squeeze;
    in.thumbX = thumbX;
    in.thumbY = thumbY;
}

void TrackingEngine::onJoyConDisconnected(Side side) {
    joyConConnected_[side] = false;
    joyConButtons_[side] = false;
    handHolding_[side] = false;
    controllers_[side].reset();
    input_[side] = ControllerState{};
}

void TrackingEngine::recenter() {
    Pose head = headPose();
    Quat yawInv = head.orientation.twist({0, 1, 0}).inverse();
    worldFix_.orientation = yawInv;
    worldFix_.position = yawInv.rotate(Vec3{-head.position.x, 0, -head.position.z});
    for (auto& c : controllers_) c.recenter(head);
}

void TrackingEngine::configureDisplay(const ViewerParams& viewer, const ScreenParams& screen, float ipdMeters,
                                      float refreshHz) {
    CardboardLens lens(viewer, screen);
    DisplayInfo d;
    d.viewer = viewer;
    d.screen = screen;
    d.ipdMeters = ipdMeters;
    d.refreshHz = refreshHz;
    for (int e = 0; e < 2; ++e) {
        EyeFovTan t = lens.fov(e);
        d.fov[e][0] = -std::atan(t.left);
        d.fov[e][1] = std::atan(t.right);
        d.fov[e][2] = std::atan(t.up);
        d.fov[e][3] = -std::atan(t.down);
    }
    int w, h;
    lens.recommendedEyeSize(0, &w, &h);
    d.eyeWidth = static_cast<uint32_t>(w);
    d.eyeHeight = static_cast<uint32_t>(h);
    display_ = d;
}

TrackingFrame TrackingEngine::buildFrame(int64_t nowNs) {
    hands_.update(nowNs);

    TrackingFrame f;
    f.head = transformSample(worldFix_, head_.headSample());
    f.headTracking6Dof = head_.is6Dof() ? 1 : 0;
    f.floorHeight = floorY_ + worldFix_.position.y;
    f.display = display_;

    for (int s = 0; s < 2; ++s) {
        Side side = static_cast<Side>(s);
        HandState h = hands_.state(side);
        if (h.active) {
            for (auto& j : h.joints) {
                j.orientation = (worldFix_.orientation * j.orientation).normalized();
                j.position = worldFix_.transform(j.position);
            }
            h.aim = transformSample(worldFix_, h.aim);
        }
        h.holdingController = h.active && joyConConnected_[s] && handHolding_[s];
        f.hands[s] = h;

        ControllerState& c = f.controllers[s];
        if (joyConConnected_[s] && controllers_[s].connected()) {
            c = input_[s];
            c.grip = transformSample(worldFix_, controllers_[s].grip());
            c.connected = 1;
            c.orientationValid = 1;
            c.positionSource = controllers_[s].positionSource();
        } else if (h.active && (joyConButtons_[s] || cfg_.emulateControllerFromHand)) {
            // No IMU on this side: the hand provides the pose. Inputs come from the Joy-Con if one
            // is connected, otherwise from gestures.
            if (joyConButtons_[s]) {
                c = input_[s];
            } else {
                c = ControllerState{};
                c.trigger = h.pinchStrength;
                c.squeeze = fistAmount(h);
                if (c.trigger > 0.3f) c.buttons |= kTouchTrigger;
                c.inputFromHand = 1;
            }
            c.grip.position = h.joints[kJPalm].position;
            c.grip.orientation = (h.joints[kJPalm].orientation * palmToGripRotation(side)).normalized();
            c.grip.timeNs = h.timeNs;
            c.connected = 1;
            c.orientationValid = 1;
            c.positionSource = kPosHandTracked;
            c.poseFromHand = 1;
        }
    }
    return f;
}

}  // namespace metaport

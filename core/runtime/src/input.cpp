// Action system. Joy-Cons (or hands) are exposed as Oculus Touch controllers, which is the
// profile every Quest app binds, plus the KHR simple controller as a fallback.
#include <algorithm>
#include <cmath>
#include <cstring>

#include "runtime.hpp"

namespace forkxr {

namespace {

constexpr const char* kTouchProfile = "/interaction_profiles/oculus/touch_controller";
constexpr const char* kSimpleProfile = "/interaction_profiles/khr/simple_controller";

struct SourceDef {
    const char* identifier;  // e.g. "trigger"
    const char* component;   // e.g. "value"; "" for pose/haptic/vec2 parents
    Source source;
    InputKind kind;
    int sides;  // bit 0 = left, bit 1 = right
};

constexpr int L = 1, R = 2, LR = 3;

const SourceDef kTouchSources[] = {
    {"x", "click", Source::kButtonAClick, InputKind::kBool, L},
    {"x", "touch", Source::kButtonATouch, InputKind::kBool, L},
    {"y", "click", Source::kButtonBClick, InputKind::kBool, L},
    {"y", "touch", Source::kButtonBTouch, InputKind::kBool, L},
    {"menu", "click", Source::kMenuClick, InputKind::kBool, L},
    {"a", "click", Source::kButtonAClick, InputKind::kBool, R},
    {"a", "touch", Source::kButtonATouch, InputKind::kBool, R},
    {"b", "click", Source::kButtonBClick, InputKind::kBool, R},
    {"b", "touch", Source::kButtonBTouch, InputKind::kBool, R},
    {"squeeze", "value", Source::kSqueezeValue, InputKind::kFloat, LR},
    {"trigger", "value", Source::kTriggerValue, InputKind::kFloat, LR},
    {"trigger", "touch", Source::kTriggerTouch, InputKind::kBool, LR},
    {"thumbstick", "", Source::kThumbstick, InputKind::kVec2, LR},
    {"thumbstick", "x", Source::kThumbstickX, InputKind::kFloat, LR},
    {"thumbstick", "y", Source::kThumbstickY, InputKind::kFloat, LR},
    {"thumbstick", "click", Source::kThumbstickClick, InputKind::kBool, LR},
    {"thumbstick", "touch", Source::kThumbstickTouch, InputKind::kBool, LR},
    {"thumbrest", "touch", Source::kThumbrestTouch, InputKind::kBool, LR},
    {"grip", "pose", Source::kGripPose, InputKind::kPose, LR},
    {"aim", "pose", Source::kAimPose, InputKind::kPose, LR},
    {"haptic", "", Source::kHaptic, InputKind::kHaptic, LR},
};

const SourceDef kSimpleSources[] = {
    {"select", "click", Source::kSelectClick, InputKind::kBool, LR},
    {"menu", "click", Source::kMenuClick, InputKind::kBool, LR},
    {"grip", "pose", Source::kGripPose, InputKind::kPose, LR},
    {"aim", "pose", Source::kAimPose, InputKind::kPose, LR},
    {"haptic", "", Source::kHaptic, InputKind::kHaptic, LR},
};

InputKind kindFor(XrActionType t) {
    switch (t) {
        case XR_ACTION_TYPE_BOOLEAN_INPUT: return InputKind::kBool;
        case XR_ACTION_TYPE_FLOAT_INPUT: return InputKind::kFloat;
        case XR_ACTION_TYPE_VECTOR2F_INPUT: return InputKind::kVec2;
        case XR_ACTION_TYPE_POSE_INPUT: return InputKind::kPose;
        default: return InputKind::kHaptic;
    }
}

// Whether a source of kind `have` can feed an action of kind `want` (OpenXR allows bool<->float).
bool compatible(InputKind have, InputKind want) {
    if (have == want) return true;
    bool scalar = (have == InputKind::kBool || have == InputKind::kFloat);
    return scalar && (want == InputKind::kBool || want == InputKind::kFloat);
}

int subactionIndex(Instance* inst, XrPath p) {
    if (p == XR_NULL_PATH) return 0;
    std::string s;
    if (!inst->paths.lookup(p, &s)) return -1;
    if (s == "/user/hand/left") return 1;
    if (s == "/user/hand/right") return 2;
    return -1;
}

float sourceFloat(Session* s, Side side, Source src) {
    const metaport::ControllerState& c = s->frame.controllers[side];
    if (!c.connected) return 0;
    uint32_t b = c.buttons;
    auto bit = [&](uint32_t m) { return (b & m) ? 1.0f : 0.0f; };
    switch (src) {
        case Source::kTriggerValue: return c.trigger;
        case Source::kTriggerTouch: return bit(metaport::kTouchTrigger);
        case Source::kTriggerClick: return c.trigger > 0.5f ? 1.0f : 0.0f;
        case Source::kSqueezeValue: return c.squeeze;
        case Source::kSqueezeClick: return c.squeeze > 0.5f ? 1.0f : 0.0f;
        case Source::kThumbstickX: return c.thumbX;
        case Source::kThumbstickY: return c.thumbY;
        case Source::kThumbstickClick: return bit(metaport::kBtnThumbstick);
        case Source::kThumbstickTouch: return bit(metaport::kTouchThumbstick);
        case Source::kThumbrestTouch:
            return (b & (metaport::kTouchA | metaport::kTouchB | metaport::kTouchThumbstick)) ? 0.0f : 1.0f;
        case Source::kButtonAClick: return bit(metaport::kBtnA);
        case Source::kButtonATouch: return bit(metaport::kTouchA);
        case Source::kButtonBClick: return bit(metaport::kBtnB);
        case Source::kButtonBTouch: return bit(metaport::kTouchB);
        case Source::kMenuClick: return bit(metaport::kBtnMenu);
        case Source::kSelectClick:
            return c.inputFromHand ? (s->pinchSelect[side] ? 1.0f : 0.0f) : (c.trigger > 0.5f ? 1.0f : 0.0f);
        default: return 0;
    }
}

bool sourceBool(Session* s, Side side, Source src) {
    const metaport::ControllerState& c = s->frame.controllers[side];
    // Pinches are noisy: use the hysteretic state for anything pinch-driven.
    if (c.inputFromHand && (src == Source::kTriggerValue || src == Source::kTriggerClick)) return s->pinchSelect[side];
    return sourceFloat(s, side, src) > 0.5f;
}

const char* localizedName(Source src, Side side) {
    bool r = side == metaport::kRight;
    switch (src) {
        case Source::kTriggerValue: case Source::kTriggerTouch: case Source::kTriggerClick: return r ? "ZR" : "ZL";
        case Source::kSqueezeValue: case Source::kSqueezeClick: return r ? "R" : "L";
        case Source::kThumbstick: case Source::kThumbstickX: case Source::kThumbstickY:
        case Source::kThumbstickTouch: return "Stick";
        case Source::kThumbstickClick: return "Stick Press";
        case Source::kButtonAClick: case Source::kButtonATouch: return r ? "A" : "Down";
        case Source::kButtonBClick: case Source::kButtonBTouch: return r ? "B" : "Up";
        case Source::kMenuClick: return r ? "+" : "-";
        case Source::kSelectClick: return "Pinch / Trigger";
        case Source::kGripPose: return "Grip";
        case Source::kAimPose: return "Aim";
        case Source::kHaptic: return "Rumble";
        default: return "Input";
    }
}

}  // namespace

bool isKnownProfile(const std::string& p) { return p == kTouchProfile || p == kSimpleProfile; }

bool resolveBinding(const std::string& profile, const std::string& path, XrActionType type, Binding* out) {
    const SourceDef* table;
    size_t n;
    if (profile == kTouchProfile) { table = kTouchSources; n = sizeof(kTouchSources) / sizeof(kTouchSources[0]); }
    else if (profile == kSimpleProfile) { table = kSimpleSources; n = sizeof(kSimpleSources) / sizeof(kSimpleSources[0]); }
    else return false;

    // /user/hand/<side>/<input|output>/<identifier>[/<component>]
    const std::string leftPrefix = "/user/hand/left/", rightPrefix = "/user/hand/right/";
    std::string rest;
    Side side;
    if (path.compare(0, leftPrefix.size(), leftPrefix) == 0) { side = metaport::kLeft; rest = path.substr(leftPrefix.size()); }
    else if (path.compare(0, rightPrefix.size(), rightPrefix) == 0) { side = metaport::kRight; rest = path.substr(rightPrefix.size()); }
    else return false;

    bool output = rest.compare(0, 7, "output/") == 0;
    if (!output && rest.compare(0, 6, "input/") != 0) return false;
    rest = rest.substr(output ? 7 : 6);
    std::string identifier = rest, component;
    size_t slash = rest.find('/');
    if (slash != std::string::npos) { identifier = rest.substr(0, slash); component = rest.substr(slash + 1); }

    InputKind want = kindFor(type);
    if (output != (want == InputKind::kHaptic)) return false;
    int sideBit = side == metaport::kLeft ? L : R;

    // With an explicit component the match is exact. Without one, OpenXR implies a component
    // from the action type: click then value for booleans, value then click for floats, and the
    // parent for vector2/pose/haptic. "touch" is never implied.
    const char* preference[3] = {nullptr, nullptr, nullptr};
    if (!component.empty()) preference[0] = component.c_str();
    else if (want == InputKind::kBool) { preference[0] = "click"; preference[1] = "value"; }
    else if (want == InputKind::kFloat) { preference[0] = "value"; preference[1] = "click"; }
    else if (want == InputKind::kPose) preference[0] = "pose";
    else preference[0] = "";

    const SourceDef* best = nullptr;
    for (int p = 0; p < 3 && preference[p] && !best; ++p) {
        for (size_t i = 0; i < n; ++i) {
            const SourceDef& d = table[i];
            if ((d.sides & sideBit) && identifier == d.identifier && std::strcmp(d.component, preference[p]) == 0 &&
                compatible(d.kind, want)) {
                best = &d;
                break;
            }
        }
    }
    if (!best) return false;
    out->side = side;
    out->source = best->source;
    return true;
}

void updateActionStates(Session* s, const XrActionsSyncInfo* info) {
    // Pinch hysteresis for hand-emulated controllers.
    for (int side = 0; side < 2; ++side) {
        const metaport::ControllerState& c = s->frame.controllers[side];
        float pinch = c.inputFromHand ? c.trigger : 0.0f;
        if (!s->pinchSelect[side] && pinch > 0.9f) s->pinchSelect[side] = true;
        else if (s->pinchSelect[side] && pinch < 0.6f) s->pinchSelect[side] = false;
    }

    std::set<ActionSet*> active;
    for (uint32_t i = 0; i < info->countActiveActionSets; ++i)
        active.insert(fromHandle<ActionSet>(info->activeActionSets[i].actionSet));

    XrTime now = nowXrTime();
    for (ActionSet* set : s->attachedSets) {
        for (auto& ap : set->actions) {
            Action* a = ap.get();
            Action::State next[3];
            if (active.count(set)) {
                for (int side = 0; side < 2; ++side) {
                    Action::State& st = next[side + 1];
                    XrPath profile = s->currentProfile[side];
                    auto it = a->suggested.find(profile);
                    if (it == a->suggested.end() || !s->frame.controllers[side].connected) continue;
                    for (const Binding& b : it->second) {
                        if (b.side != side) continue;
                        st.active = true;
                        Side sd = static_cast<Side>(side);
                        switch (a->type) {
                            case XR_ACTION_TYPE_BOOLEAN_INPUT:
                                st.boolValue = st.boolValue || sourceBool(s, sd, b.source);
                                break;
                            case XR_ACTION_TYPE_FLOAT_INPUT: {
                                float v = b.source == Source::kThumbstick ? 0.0f : sourceFloat(s, sd, b.source);
                                if (std::fabs(v) > std::fabs(st.floatValue)) st.floatValue = v;
                                break;
                            }
                            case XR_ACTION_TYPE_VECTOR2F_INPUT:
                                if (b.source == Source::kThumbstick) {
                                    const auto& c = s->frame.controllers[side];
                                    if (c.thumbX * c.thumbX + c.thumbY * c.thumbY >
                                        st.vec2.x * st.vec2.x + st.vec2.y * st.vec2.y)
                                        st.vec2 = {c.thumbX, c.thumbY};
                                }
                                break;
                            default:
                                break;
                        }
                    }
                }
                // Combined (no subaction path): strongest input of either hand.
                Action::State& all = next[0];
                for (int k = 1; k <= 2; ++k) {
                    const Action::State& h = next[k];
                    all.active = all.active || h.active;
                    all.boolValue = all.boolValue || h.boolValue;
                    if (std::fabs(h.floatValue) > std::fabs(all.floatValue)) all.floatValue = h.floatValue;
                    if (h.vec2.x * h.vec2.x + h.vec2.y * h.vec2.y > all.vec2.x * all.vec2.x + all.vec2.y * all.vec2.y)
                        all.vec2 = h.vec2;
                }
            }
            for (int k = 0; k < 3; ++k) {
                Action::State& prev = a->state[k];
                Action::State& cur = next[k];
                bool differs = cur.boolValue != prev.boolValue || cur.floatValue != prev.floatValue ||
                               cur.vec2.x != prev.vec2.x || cur.vec2.y != prev.vec2.y;
                cur.changed = prev.active && cur.active && differs;
                cur.lastChange = differs ? now : prev.lastChange;
                prev = cur;
            }
        }
    }
}

}  // namespace forkxr

using namespace forkxr;

XRAPI_ATTR XrResult XRAPI_CALL xrCreateActionSet(XrInstance instance, const XrActionSetCreateInfo* info, XrActionSet* out) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !out || info->type != XR_TYPE_ACTION_SET_CREATE_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (info->actionSetName[0] == 0) return XR_ERROR_NAME_INVALID;
    for (ActionSet* a : inst->actionSets)
        if (a->name == info->actionSetName) return XR_ERROR_NAME_DUPLICATED;
    auto* set = new ActionSet;
    set->instance = inst;
    set->name = info->actionSetName;
    set->priority = info->priority;
    actionSets().add(set);
    inst->actionSets.insert(set);
    *out = toHandle<XrActionSet>(set);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyActionSet(XrActionSet actionSet) {
    ActionSet* set = fromHandle<ActionSet>(actionSet);
    if (!actionSets().valid(set)) return XR_ERROR_HANDLE_INVALID;
    for (auto& a : set->actions) actions().remove(a.get());
    for (Session* s : set->instance->sessions) {
        auto& v = s->attachedSets;
        v.erase(std::remove(v.begin(), v.end(), set), v.end());
        for (Space* sp : s->spaces)
            for (auto& a : set->actions)
                if (sp->action == a.get()) sp->action = nullptr;
    }
    set->instance->actionSets.erase(set);
    actionSets().remove(set);
    delete set;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrCreateAction(XrActionSet actionSet, const XrActionCreateInfo* info, XrAction* out) {
    ActionSet* set = fromHandle<ActionSet>(actionSet);
    if (!actionSets().valid(set)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !out || info->type != XR_TYPE_ACTION_CREATE_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (set->attached) return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
    if (info->actionName[0] == 0) return XR_ERROR_NAME_INVALID;
    for (auto& a : set->actions)
        if (a->name == info->actionName) return XR_ERROR_NAME_DUPLICATED;
    auto a = std::make_unique<Action>();
    a->set = set;
    a->name = info->actionName;
    a->type = info->actionType;
    for (uint32_t i = 0; i < info->countSubactionPaths; ++i) {
        if (subactionIndex(set->instance, info->subactionPaths[i]) <= 0) return XR_ERROR_PATH_UNSUPPORTED;
        a->subactionPaths.push_back(info->subactionPaths[i]);
    }
    Action* raw = a.get();
    set->actions.push_back(std::move(a));
    actions().add(raw);
    *out = toHandle<XrAction>(raw);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyAction(XrAction action) {
    Action* a = fromHandle<Action>(action);
    if (!actions().valid(a)) return XR_ERROR_HANDLE_INVALID;
    // Actions are owned by their set; the handle simply becomes invalid.
    actions().remove(a);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrSuggestInteractionProfileBindings(XrInstance instance,
                                                                   const XrInteractionProfileSuggestedBinding* sb) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    if (!sb || sb->type != XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING) return XR_ERROR_VALIDATION_FAILURE;
    std::string profile;
    if (!inst->paths.lookup(sb->interactionProfile, &profile)) return XR_ERROR_PATH_INVALID;
    // Unknown profiles (Index, Vive, ...) are accepted and ignored, as the spec requires for
    // profiles the runtime does not emulate.
    bool known = isKnownProfile(profile);

    std::map<Action*, std::vector<Binding>> resolved;
    for (uint32_t i = 0; i < sb->countSuggestedBindings; ++i) {
        Action* a = fromHandle<Action>(sb->suggestedBindings[i].action);
        if (!actions().valid(a)) return XR_ERROR_HANDLE_INVALID;
        if (a->set->attached) return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
        std::string path;
        if (!inst->paths.lookup(sb->suggestedBindings[i].binding, &path)) return XR_ERROR_PATH_INVALID;
        if (!known) continue;
        Binding b;
        if (!resolveBinding(profile, path, a->type, &b)) return XR_ERROR_PATH_UNSUPPORTED;
        b.path = sb->suggestedBindings[i].binding;
        resolved[a].push_back(b);
    }
    if (!known) return XR_SUCCESS;
    // A new suggestion for a profile replaces the previous one for every action.
    for (ActionSet* set : inst->actionSets)
        for (auto& a : set->actions) a->suggested.erase(sb->interactionProfile);
    for (auto& [a, bindings] : resolved) a->suggested[sb->interactionProfile] = bindings;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrAttachSessionActionSets(XrSession session, const XrSessionActionSetsAttachInfo* info) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || info->type != XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (!s->attachedSets.empty()) return XR_ERROR_ACTIONSETS_ALREADY_ATTACHED;
    for (uint32_t i = 0; i < info->countActionSets; ++i) {
        ActionSet* set = fromHandle<ActionSet>(info->actionSets[i]);
        if (!actionSets().valid(set)) return XR_ERROR_HANDLE_INVALID;
    }
    for (uint32_t i = 0; i < info->countActionSets; ++i) {
        ActionSet* set = fromHandle<ActionSet>(info->actionSets[i]);
        set->attached = true;
        s->attachedSets.push_back(set);
    }

    // Choose the profile per hand: Touch if the app bound anything for it, else simple controller.
    XrPath touch = s->instance->paths.intern(kTouchProfile);
    XrPath simple = s->instance->paths.intern(kSimpleProfile);
    bool hasTouch = false, hasSimple = false;
    for (ActionSet* set : s->attachedSets)
        for (auto& a : set->actions) {
            hasTouch |= a->suggested.count(touch) != 0;
            hasSimple |= a->suggested.count(simple) != 0;
        }
    XrPath chosen = hasTouch ? touch : (hasSimple ? simple : XR_NULL_PATH);
    s->currentProfile[0] = s->currentProfile[1] = chosen;
    if (chosen != XR_NULL_PATH) {
        XrEventDataBuffer buf{XR_TYPE_EVENT_DATA_BUFFER};
        auto* e = reinterpret_cast<XrEventDataInteractionProfileChanged*>(&buf);
        e->type = XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED;
        e->next = nullptr;
        e->session = session;
        std::lock_guard<std::mutex> l(s->instance->eventMutex);
        s->instance->events.push_back(buf);
    }
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetCurrentInteractionProfile(XrSession session, XrPath topLevelUserPath,
                                                              XrInteractionProfileState* state) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!state || state->type != XR_TYPE_INTERACTION_PROFILE_STATE) return XR_ERROR_VALIDATION_FAILURE;
    if (s->attachedSets.empty()) return XR_ERROR_ACTIONSET_NOT_ATTACHED;
    int idx = subactionIndex(s->instance, topLevelUserPath);
    if (idx < 0) {
        std::string p;
        s->instance->paths.lookup(topLevelUserPath, &p);
        if (p == "/user/head" || p == "/user/gamepad") { state->interactionProfile = XR_NULL_PATH; return XR_SUCCESS; }
        return XR_ERROR_PATH_UNSUPPORTED;
    }
    if (idx == 0) return XR_ERROR_PATH_INVALID;
    state->interactionProfile = s->frame.controllers[idx - 1].connected ? s->currentProfile[idx - 1] : XR_NULL_PATH;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrSyncActions(XrSession session, const XrActionsSyncInfo* info) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || info->type != XR_TYPE_ACTIONS_SYNC_INFO) return XR_ERROR_VALIDATION_FAILURE;
    for (uint32_t i = 0; i < info->countActiveActionSets; ++i) {
        ActionSet* set = fromHandle<ActionSet>(info->activeActionSets[i].actionSet);
        if (!actionSets().valid(set)) return XR_ERROR_HANDLE_INVALID;
        if (!set->attached) return XR_ERROR_ACTIONSET_NOT_ATTACHED;
    }
    refreshFrame(s);
    updateActionStates(s, info);
    return s->state == XR_SESSION_STATE_FOCUSED ? XR_SUCCESS : XR_SESSION_NOT_FOCUSED;
}

namespace {

template <typename T>
XrResult stateFor(XrSession session, const XrActionStateGetInfo* info, XrActionType type, Action** a, int* idx) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || info->type != XR_TYPE_ACTION_STATE_GET_INFO) return XR_ERROR_VALIDATION_FAILURE;
    *a = fromHandle<Action>(info->action);
    if (!actions().valid(*a)) return XR_ERROR_HANDLE_INVALID;
    if (!(*a)->set->attached) return XR_ERROR_ACTIONSET_NOT_ATTACHED;
    if ((*a)->type != type) return XR_ERROR_ACTION_TYPE_MISMATCH;
    *idx = subactionIndex(s->instance, info->subactionPath);
    if (*idx < 0) return XR_ERROR_PATH_UNSUPPORTED;
    if (*idx > 0) {
        bool declared = false;
        for (XrPath p : (*a)->subactionPaths) declared |= p == info->subactionPath;
        if (!declared) return XR_ERROR_PATH_UNSUPPORTED;
    }
    return XR_SUCCESS;
}

}  // namespace

XRAPI_ATTR XrResult XRAPI_CALL xrGetActionStateBoolean(XrSession session, const XrActionStateGetInfo* info,
                                                       XrActionStateBoolean* state) {
    Action* a;
    int idx;
    XrResult r = stateFor<XrActionStateBoolean>(session, info, XR_ACTION_TYPE_BOOLEAN_INPUT, &a, &idx);
    if (XR_FAILED(r)) return r;
    const Action::State& st = a->state[idx];
    state->currentState = st.boolValue;
    state->changedSinceLastSync = st.changed;
    state->lastChangeTime = st.lastChange;
    state->isActive = st.active;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetActionStateFloat(XrSession session, const XrActionStateGetInfo* info,
                                                     XrActionStateFloat* state) {
    Action* a;
    int idx;
    XrResult r = stateFor<XrActionStateFloat>(session, info, XR_ACTION_TYPE_FLOAT_INPUT, &a, &idx);
    if (XR_FAILED(r)) return r;
    const Action::State& st = a->state[idx];
    state->currentState = st.floatValue;
    state->changedSinceLastSync = st.changed;
    state->lastChangeTime = st.lastChange;
    state->isActive = st.active;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetActionStateVector2f(XrSession session, const XrActionStateGetInfo* info,
                                                        XrActionStateVector2f* state) {
    Action* a;
    int idx;
    XrResult r = stateFor<XrActionStateVector2f>(session, info, XR_ACTION_TYPE_VECTOR2F_INPUT, &a, &idx);
    if (XR_FAILED(r)) return r;
    const Action::State& st = a->state[idx];
    state->currentState = st.vec2;
    state->changedSinceLastSync = st.changed;
    state->lastChangeTime = st.lastChange;
    state->isActive = st.active;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetActionStatePose(XrSession session, const XrActionStateGetInfo* info,
                                                    XrActionStatePose* state) {
    Action* a;
    int idx;
    XrResult r = stateFor<XrActionStatePose>(session, info, XR_ACTION_TYPE_POSE_INPUT, &a, &idx);
    if (XR_FAILED(r)) return r;
    Session* s = fromHandle<Session>(session);
    bool active = false;
    for (int side = 0; side < 2; ++side) {
        if (idx != 0 && idx != side + 1) continue;
        auto it = a->suggested.find(s->currentProfile[side]);
        if (it == a->suggested.end() || !s->frame.controllers[side].connected) continue;
        for (const Binding& b : it->second) active |= b.side == side;
    }
    state->isActive = active;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateBoundSourcesForAction(XrSession session,
                                                                const XrBoundSourcesForActionEnumerateInfo* info,
                                                                uint32_t capacity, uint32_t* count, XrPath* sources) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !count) return XR_ERROR_VALIDATION_FAILURE;
    Action* a = fromHandle<Action>(info->action);
    if (!actions().valid(a)) return XR_ERROR_HANDLE_INVALID;
    if (!a->set->attached) return XR_ERROR_ACTIONSET_NOT_ATTACHED;
    std::vector<XrPath> out;
    for (int side = 0; side < 2; ++side) {
        auto it = a->suggested.find(s->currentProfile[side]);
        if (it == a->suggested.end()) continue;
        for (const Binding& b : it->second)
            if (b.side == side && std::find(out.begin(), out.end(), b.path) == out.end()) out.push_back(b.path);
    }
    *count = static_cast<uint32_t>(out.size());
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < out.size()) return XR_ERROR_SIZE_INSUFFICIENT;
    std::copy(out.begin(), out.end(), sources);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetInputSourceLocalizedName(XrSession session, const XrInputSourceLocalizedNameGetInfo* info,
                                                             uint32_t capacity, uint32_t* count, char* buffer) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !count) return XR_ERROR_VALIDATION_FAILURE;
    std::string path;
    if (!s->instance->paths.lookup(info->sourcePath, &path)) return XR_ERROR_PATH_INVALID;
    std::string name;
    Side side = path.find("/right/") != std::string::npos ? metaport::kRight : metaport::kLeft;
    if (info->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_USER_PATH_BIT)
        name += side == metaport::kRight ? "Right Hand" : "Left Hand";
    if (info->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_INTERACTION_PROFILE_BIT)
        name += std::string(name.empty() ? "" : " ") + "Joy-Con";
    if (info->whichComponents & XR_INPUT_SOURCE_LOCALIZED_NAME_COMPONENT_BIT) {
        Binding b{};
        std::string profile;
        s->instance->paths.lookup(s->currentProfile[side], &profile);
        Source src = Source::kNone;
        for (XrActionType t : {XR_ACTION_TYPE_FLOAT_INPUT, XR_ACTION_TYPE_BOOLEAN_INPUT, XR_ACTION_TYPE_VECTOR2F_INPUT,
                               XR_ACTION_TYPE_POSE_INPUT, XR_ACTION_TYPE_VIBRATION_OUTPUT})
            if (resolveBinding(profile, path, t, &b)) { src = b.source; break; }
        name += std::string(name.empty() ? "" : " ") + localizedName(src, side);
    }
    *count = static_cast<uint32_t>(name.size() + 1);
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < name.size() + 1) return XR_ERROR_SIZE_INSUFFICIENT;
    std::memcpy(buffer, name.c_str(), name.size() + 1);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrApplyHapticFeedback(XrSession session, const XrHapticActionInfo* info,
                                                     const XrHapticBaseHeader* haptic) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !haptic) return XR_ERROR_VALIDATION_FAILURE;
    Action* a = fromHandle<Action>(info->action);
    if (!actions().valid(a)) return XR_ERROR_HANDLE_INVALID;
    if (a->type != XR_ACTION_TYPE_VIBRATION_OUTPUT) return XR_ERROR_ACTION_TYPE_MISMATCH;
    if (!a->set->attached) return XR_ERROR_ACTIONSET_NOT_ATTACHED;
    if (haptic->type != XR_TYPE_HAPTIC_VIBRATION) return XR_ERROR_VALIDATION_FAILURE;
    auto* v = reinterpret_cast<const XrHapticVibration*>(haptic);
    int idx = subactionIndex(s->instance, info->subactionPath);
    if (idx < 0) return XR_ERROR_PATH_UNSUPPORTED;
    int64_t duration = v->duration <= 0 ? 20'000'000 : v->duration;  // XR_MIN_HAPTIC_DURATION -> 20 ms
    float freq = v->frequency == XR_FREQUENCY_UNSPECIFIED ? 160.0f : v->frequency;
    for (int side = 0; side < 2; ++side) {
        if (idx != 0 && idx != side + 1) continue;
        auto it = a->suggested.find(s->currentProfile[side]);
        if (it == a->suggested.end()) continue;
        for (const Binding& b : it->second)
            if (b.side == side && b.source == Source::kHaptic)
                s->instance->tracking->requestHaptic(static_cast<Side>(side), metaport::clampf(v->amplitude, 0, 1), freq, duration);
    }
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrStopHapticFeedback(XrSession session, const XrHapticActionInfo* info) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info) return XR_ERROR_VALIDATION_FAILURE;
    Action* a = fromHandle<Action>(info->action);
    if (!actions().valid(a)) return XR_ERROR_HANDLE_INVALID;
    if (a->type != XR_ACTION_TYPE_VIBRATION_OUTPUT) return XR_ERROR_ACTION_TYPE_MISMATCH;
    int idx = subactionIndex(s->instance, info->subactionPath);
    for (int side = 0; side < 2; ++side)
        if (idx == 0 || idx == side + 1) s->instance->tracking->requestHaptic(static_cast<Side>(side), 0, 0, 0);
    return XR_SUCCESS;
}

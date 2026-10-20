// Session lifecycle, frame loop, reference/action spaces and views.
#include <cstring>
#include <thread>

#include "runtime.hpp"

namespace forkxr {

void refreshFrame(Session* s) {
    TrackingFrame f;
    if (s->instance->tracking->read(&f)) s->frame = f;
}

Pose headPoseAt(Session* s, XrTime time) { return metaport::predictPose(s->frame.head, time); }

bool controllerPose(Session* s, Side side, Source src, XrTime time, Pose* out, bool* positionTracked) {
    const metaport::ControllerState& c = s->frame.controllers[side];
    if (!c.connected || !c.orientationValid) return false;
    Pose grip = metaport::predictPose(c.grip, time);
    *positionTracked = c.positionSource == metaport::kPosHandTracked || c.positionSource == metaport::kPosInertial;
    if (src == Source::kGripPose) {
        *out = grip;
        return true;
    }
    if (src == Source::kAimPose) {
        if (c.poseFromHand) {
            *out = metaport::predictPose(s->frame.hands[side].aim, time);
        } else {
            // A Joy-Con held like a remote points along the grip's -Z; the ray starts at its tip.
            *out = grip * Pose{Quat{}, Vec3{0, 0, -0.06f}};
        }
        return true;
    }
    return false;
}

namespace {

Side sideForSubaction(Instance* inst, XrPath p, bool* ok) {
    std::string s;
    *ok = true;
    if (inst->paths.lookup(p, &s)) {
        if (s == "/user/hand/left") return metaport::kLeft;
        if (s == "/user/hand/right") return metaport::kRight;
    }
    *ok = false;
    return metaport::kLeft;
}

bool referenceSpaceSupported(Instance* inst, XrReferenceSpaceType t) {
    switch (t) {
        case XR_REFERENCE_SPACE_TYPE_VIEW:
        case XR_REFERENCE_SPACE_TYPE_LOCAL:
        case XR_REFERENCE_SPACE_TYPE_STAGE:
            return true;
        case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR:
            return XR_VERSION_MINOR(inst->apiVersion) >= 1 || inst->hasExtension(XR_EXT_LOCAL_FLOOR_EXTENSION_NAME);
        default:
            return false;
    }
}

// The first pose source an action is bound to for a given hand in the current profile.
bool actionPoseSource(Session* s, Action* a, Side side, Source* out) {
    XrPath profile = s->currentProfile[side];
    auto it = a->suggested.find(profile);
    if (it == a->suggested.end()) return false;
    for (const Binding& b : it->second) {
        if (b.side == side && (b.source == Source::kGripPose || b.source == Source::kAimPose)) {
            *out = b.source;
            return true;
        }
    }
    return false;
}

}  // namespace

bool locateInWorld(Session* s, Space* space, XrTime time, Pose* out, bool* positionTracked) {
    *positionTracked = true;
    if (space->kind == SpaceKind::kReference) {
        Pose base;
        switch (space->refType) {
            case XR_REFERENCE_SPACE_TYPE_VIEW:
                base = headPoseAt(s, time);
                *positionTracked = s->frame.headTracking6Dof != 0;
                break;
            case XR_REFERENCE_SPACE_TYPE_STAGE:
            case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR:
                base.position = {0, s->frame.floorHeight, 0};
                break;
            default:  // LOCAL
                break;
        }
        *out = base * space->offset;
        return true;
    }

    Action* a = space->action;
    if (!a || !a->set->attached) return false;
    bool ok;
    Side side = sideForSubaction(s->instance, space->subaction, &ok);
    Side candidates[2] = {side, side};
    int n = 1;
    if (!ok) { candidates[0] = metaport::kLeft; candidates[1] = metaport::kRight; n = 2; }  // any hand
    for (int i = 0; i < n; ++i) {
        Source src;
        if (!actionPoseSource(s, a, candidates[i], &src)) continue;
        Pose p;
        if (controllerPose(s, candidates[i], src, time, &p, positionTracked)) {
            *out = p * space->offset;
            return true;
        }
    }
    return false;
}

}  // namespace forkxr

using namespace forkxr;

#ifdef __ANDROID__
namespace {
bool g_graphicsRequirementsQueried = false;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetOpenGLESGraphicsRequirementsKHR(XrInstance instance, XrSystemId systemId,
                                                                    XrGraphicsRequirementsOpenGLESKHR* req) {
    if (!instances().valid(fromHandle<Instance>(instance))) return XR_ERROR_HANDLE_INVALID;
    if (systemId != kSystemId) return XR_ERROR_SYSTEM_INVALID;
    if (!req) return XR_ERROR_VALIDATION_FAILURE;
    req->minApiVersionSupported = XR_MAKE_VERSION(3, 0, 0);
    req->maxApiVersionSupported = XR_MAKE_VERSION(3, 2, 0);
    g_graphicsRequirementsQueried = true;
    return XR_SUCCESS;
}
#endif

XRAPI_ATTR XrResult XRAPI_CALL xrCreateSession(XrInstance instance, const XrSessionCreateInfo* info, XrSession* out) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !out || info->type != XR_TYPE_SESSION_CREATE_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (info->systemId != kSystemId) return XR_ERROR_SYSTEM_INVALID;
    if (!inst->sessions.empty()) return XR_ERROR_LIMIT_REACHED;

    auto s = std::make_unique<Session>();
    s->instance = inst;
    refreshFrame(s.get());

    bool haveGraphics = false;
#ifdef __ANDROID__
    for (auto* n = static_cast<const XrBaseInStructure*>(info->next); n; n = n->next) {
        if (n->type == XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR) {
            if (!g_graphicsRequirementsQueried) return XR_ERROR_GRAPHICS_REQUIREMENTS_CALL_MISSING;
            auto* g = reinterpret_cast<const XrGraphicsBindingOpenGLESAndroidKHR*>(n);
            s->eglDisplay = g->display;
            s->eglContext = g->context;
            s->compositor = createCardboardCompositor(static_cast<JavaVM*>(inst->javaVm),
                                                      static_cast<jobject>(inst->activity), g->display, g->context,
                                                      s->frame.display);
            if (!s->compositor) return XR_ERROR_INITIALIZATION_FAILED;
            haveGraphics = true;
        }
    }
#endif
    if (!haveGraphics) {
        if (!inst->hasExtension(XR_MND_HEADLESS_EXTENSION_NAME)) return XR_ERROR_GRAPHICS_DEVICE_INVALID;
        s->headless = true;
        s->compositor = createNullCompositor(s->frame.display.refreshHz);
    }

    Session* raw = s.release();
    sessions().add(raw);
    inst->sessions.insert(raw);
    XrTime now = nowXrTime();
    inst->pushSessionState(raw, XR_SESSION_STATE_IDLE, now);
    inst->pushSessionState(raw, XR_SESSION_STATE_READY, now);
    *out = toHandle<XrSession>(raw);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroySession(XrSession session) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    for (Space* sp : std::set<Space*>(s->spaces)) xrDestroySpace(toHandle<XrSpace>(sp));
    for (Swapchain* sc : std::set<Swapchain*>(s->swapchains)) xrDestroySwapchain(toHandle<XrSwapchain>(sc));
    for (HandTracker* h : std::set<HandTracker*>(s->handTrackers)) {
        handTrackers().remove(h);
        delete h;
    }
    s->instance->sessions.erase(s);
    {
        // Drop queued events that reference this session.
        std::lock_guard<std::mutex> l(s->instance->eventMutex);
        auto& q = s->instance->events;
        for (auto it = q.begin(); it != q.end();) {
            auto* e = reinterpret_cast<const XrEventDataSessionStateChanged*>(&*it);
            if (e->type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED && e->session == session) it = q.erase(it);
            else ++it;
        }
    }
    sessions().remove(s);
    delete s;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrBeginSession(XrSession session, const XrSessionBeginInfo* info) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || info->type != XR_TYPE_SESSION_BEGIN_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (s->running) return XR_ERROR_SESSION_RUNNING;
    if (s->state != XR_SESSION_STATE_READY) return XR_ERROR_SESSION_NOT_READY;
    if (info->primaryViewConfigurationType != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO)
        return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    s->running = true;
    s->viewConfig = info->primaryViewConfigurationType;
    XrTime now = nowXrTime();
    s->instance->pushSessionState(s, XR_SESSION_STATE_SYNCHRONIZED, now);
    s->instance->pushSessionState(s, XR_SESSION_STATE_VISIBLE, now);
    s->instance->pushSessionState(s, XR_SESSION_STATE_FOCUSED, now);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrRequestExitSession(XrSession session) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!s->running) return XR_ERROR_SESSION_NOT_RUNNING;
    s->exitRequested = true;
    XrTime now = nowXrTime();
    if (s->state == XR_SESSION_STATE_FOCUSED) s->instance->pushSessionState(s, XR_SESSION_STATE_VISIBLE, now);
    s->instance->pushSessionState(s, XR_SESSION_STATE_SYNCHRONIZED, now);
    s->instance->pushSessionState(s, XR_SESSION_STATE_STOPPING, now);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEndSession(XrSession session) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!s->running) return XR_ERROR_SESSION_NOT_RUNNING;
    if (s->state != XR_SESSION_STATE_STOPPING) return XR_ERROR_SESSION_NOT_STOPPING;
    s->running = false;
    s->frameBegun = false;
    XrTime now = nowXrTime();
    s->instance->pushSessionState(s, XR_SESSION_STATE_IDLE, now);
    if (s->exitRequested) s->instance->pushSessionState(s, XR_SESSION_STATE_EXITING, now);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrWaitFrame(XrSession session, const XrFrameWaitInfo*, XrFrameState* state) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!state || state->type != XR_TYPE_FRAME_STATE) return XR_ERROR_VALIDATION_FAILURE;
    if (!s->running) return XR_ERROR_SESSION_NOT_RUNNING;

    int64_t period = s->compositor->framePeriodNs();
    XrTime now = nowXrTime();
    if (s->nextDisplayTime == 0) {
        int64_t vsync = s->compositor->lastVsyncNs();
        s->nextDisplayTime = (vsync ? vsync : now) + 2 * period;
    } else {
        s->nextDisplayTime += period;
    }
    // Never fall behind: the frame we hand out must still be at least one period away.
    while (s->nextDisplayTime < now + period) s->nextDisplayTime += period;
    // Throttle the app to one frame per vsync: wake ~2 periods before its display time.
    XrTime wake = s->nextDisplayTime - 2 * period;
    if (wake > now) std::this_thread::sleep_for(std::chrono::nanoseconds(wake - now));

    refreshFrame(s);
    s->framesWaited++;
    state->predictedDisplayTime = s->nextDisplayTime;
    state->predictedDisplayPeriod = period;
    state->shouldRender = (s->state == XR_SESSION_STATE_VISIBLE || s->state == XR_SESSION_STATE_FOCUSED) ? XR_TRUE : XR_FALSE;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrBeginFrame(XrSession session, const XrFrameBeginInfo*) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!s->running) return XR_ERROR_SESSION_NOT_RUNNING;
    if (s->framesWaited == 0) return XR_ERROR_CALL_ORDER_INVALID;
    bool discarded = s->frameBegun;
    s->frameBegun = true;
    return discarded ? XR_FRAME_DISCARDED : XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEndFrame(XrSession session, const XrFrameEndInfo* info) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || info->type != XR_TYPE_FRAME_END_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (!s->running) return XR_ERROR_SESSION_NOT_RUNNING;
    if (!s->frameBegun) return XR_ERROR_CALL_ORDER_INVALID;
    if (info->environmentBlendMode != XR_ENVIRONMENT_BLEND_MODE_OPAQUE) return XR_ERROR_ENVIRONMENT_BLEND_MODE_UNSUPPORTED;
    if (info->displayTime <= 0) return XR_ERROR_TIME_INVALID;
    s->frameBegun = false;

    for (uint32_t i = 0; i < info->layerCount; ++i) {
        const XrCompositionLayerBaseHeader* layer = info->layers[i];
        if (!layer) return XR_ERROR_LAYER_INVALID;
        if (layer->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) continue;  // quads etc. not composited yet
        auto* proj = reinterpret_cast<const XrCompositionLayerProjection*>(layer);
        if (proj->viewCount != 2) return XR_ERROR_VALIDATION_FAILURE;
        Space* base = fromHandle<Space>(proj->space);
        if (!spaces().valid(base)) return XR_ERROR_HANDLE_INVALID;

        Pose baseWorld;
        bool tracked;
        if (!locateInWorld(s, base, info->displayTime, &baseWorld, &tracked)) continue;

        EyeSubmission eyes[2];
        for (int e = 0; e < 2; ++e) {
            const XrCompositionLayerProjectionView& v = proj->views[e];
            Swapchain* sc = fromHandle<Swapchain>(v.subImage.swapchain);
            if (!swapchains().valid(sc)) return XR_ERROR_HANDLE_INVALID;
            if (!s->headless && sc->lastReleased < 0) return XR_ERROR_LAYER_INVALID;  // nothing rendered yet
            eyes[e].texture = sc->lastReleased >= 0 && !sc->textures.empty() ? sc->textures[sc->lastReleased] : 0;
            eyes[e].isArray = sc->isArray;
            eyes[e].srgb = sc->info.format == 0x8C43;  // GL_SRGB8_ALPHA8
            eyes[e].arrayIndex = v.subImage.imageArrayIndex;
            eyes[e].rect = v.subImage.imageRect;
            eyes[e].texWidth = sc->info.width;
            eyes[e].texHeight = sc->info.height;
            eyes[e].pose = toXr(baseWorld * fromXr(v.pose));
            eyes[e].fov = v.fov;
        }
        refreshFrame(s);  // newest tracking for reprojection
        s->compositor->submit(eyes, info->displayTime, headPoseAt(s, info->displayTime));
        break;  // one projection layer is composited
    }
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrLocateViews(XrSession session, const XrViewLocateInfo* info, XrViewState* viewState,
                                             uint32_t capacity, uint32_t* count, XrView* views) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !viewState || !count) return XR_ERROR_VALIDATION_FAILURE;
    if (info->viewConfigurationType != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO)
        return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    if (info->displayTime <= 0) return XR_ERROR_TIME_INVALID;
    Space* base = fromHandle<Space>(info->space);
    if (!spaces().valid(base)) return XR_ERROR_HANDLE_INVALID;
    *count = 2;
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < 2) return XR_ERROR_SIZE_INSUFFICIENT;

    Pose baseWorld;
    bool baseTracked;
    if (!locateInWorld(s, base, info->displayTime, &baseWorld, &baseTracked)) {
        viewState->viewStateFlags = 0;
        return XR_SUCCESS;
    }
    Pose head = headPoseAt(s, info->displayTime);
    Pose rel = baseWorld.inverse() * head;
    float halfIpd = s->frame.display.ipdMeters * 0.5f;
    for (int e = 0; e < 2; ++e) {
        Pose eye = rel * Pose{Quat{}, Vec3{e == 0 ? -halfIpd : halfIpd, 0, 0}};
        views[e].pose = toXr(eye);
        const float* f = s->frame.display.fov[e];
        views[e].fov = {f[0], f[1], f[2], f[3]};
    }
    viewState->viewStateFlags = XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_ORIENTATION_TRACKED_BIT |
                                XR_VIEW_STATE_POSITION_VALID_BIT;
    if (s->frame.headTracking6Dof) viewState->viewStateFlags |= XR_VIEW_STATE_POSITION_TRACKED_BIT;
    return XR_SUCCESS;
}

// --- Spaces ---------------------------------------------------------------------------------------

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateReferenceSpaces(XrSession session, uint32_t capacity, uint32_t* count,
                                                          XrReferenceSpaceType* out) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!count) return XR_ERROR_VALIDATION_FAILURE;
    std::vector<XrReferenceSpaceType> types = {XR_REFERENCE_SPACE_TYPE_VIEW, XR_REFERENCE_SPACE_TYPE_LOCAL,
                                               XR_REFERENCE_SPACE_TYPE_STAGE};
    if (referenceSpaceSupported(s->instance, XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR))
        types.push_back(XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR);
    *count = static_cast<uint32_t>(types.size());
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < types.size()) return XR_ERROR_SIZE_INSUFFICIENT;
    std::memcpy(out, types.data(), types.size() * sizeof(XrReferenceSpaceType));
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrCreateReferenceSpace(XrSession session, const XrReferenceSpaceCreateInfo* info,
                                                      XrSpace* out) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !out || info->type != XR_TYPE_REFERENCE_SPACE_CREATE_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (!referenceSpaceSupported(s->instance, info->referenceSpaceType)) return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
    auto* sp = new Space;
    sp->session = s;
    sp->kind = SpaceKind::kReference;
    sp->refType = info->referenceSpaceType;
    sp->offset = fromXr(info->poseInReferenceSpace);
    spaces().add(sp);
    s->spaces.insert(sp);
    *out = toHandle<XrSpace>(sp);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetReferenceSpaceBoundsRect(XrSession session, XrReferenceSpaceType type,
                                                             XrExtent2Df* bounds) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!referenceSpaceSupported(s->instance, type)) return XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
    if (!bounds) return XR_ERROR_VALIDATION_FAILURE;
    // No guardian/boundary on a phone headset: report unknown bounds.
    bounds->width = 0;
    bounds->height = 0;
    return XR_SPACE_BOUNDS_UNAVAILABLE;
}

XRAPI_ATTR XrResult XRAPI_CALL xrCreateActionSpace(XrSession session, const XrActionSpaceCreateInfo* info, XrSpace* out) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !out || info->type != XR_TYPE_ACTION_SPACE_CREATE_INFO) return XR_ERROR_VALIDATION_FAILURE;
    Action* a = fromHandle<Action>(info->action);
    if (!actions().valid(a)) return XR_ERROR_HANDLE_INVALID;
    if (a->type != XR_ACTION_TYPE_POSE_INPUT) return XR_ERROR_ACTION_TYPE_MISMATCH;
    if (info->subactionPath != XR_NULL_PATH) {
        bool found = false;
        for (XrPath p : a->subactionPaths) found |= p == info->subactionPath;
        if (!found) return XR_ERROR_PATH_UNSUPPORTED;
    }
    auto* sp = new Space;
    sp->session = s;
    sp->kind = SpaceKind::kAction;
    sp->action = a;
    sp->subaction = info->subactionPath;
    sp->offset = fromXr(info->poseInActionSpace);
    spaces().add(sp);
    s->spaces.insert(sp);
    *out = toHandle<XrSpace>(sp);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrLocateSpace(XrSpace space, XrSpace baseSpace, XrTime time, XrSpaceLocation* location) {
    Space* sp = fromHandle<Space>(space);
    Space* base = fromHandle<Space>(baseSpace);
    if (!spaces().valid(sp) || !spaces().valid(base)) return XR_ERROR_HANDLE_INVALID;
    if (!location || location->type != XR_TYPE_SPACE_LOCATION) return XR_ERROR_VALIDATION_FAILURE;
    if (time <= 0) return XR_ERROR_TIME_INVALID;
    Session* s = sp->session;

    XrSpaceVelocity* velocity = nullptr;
    for (auto* n = static_cast<XrBaseOutStructure*>(location->next); n; n = n->next)
        if (n->type == XR_TYPE_SPACE_VELOCITY) velocity = reinterpret_cast<XrSpaceVelocity*>(n);

    Pose a, b;
    bool ta, tb;
    location->locationFlags = 0;
    if (velocity) velocity->velocityFlags = 0;
    if (!locateInWorld(s, sp, time, &a, &ta) || !locateInWorld(s, base, time, &b, &tb)) return XR_SUCCESS;

    Pose rel = b.inverse() * a;
    location->pose = toXr(rel);
    location->locationFlags = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT |
                              XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    if (ta && tb) location->locationFlags |= XR_SPACE_LOCATION_POSITION_TRACKED_BIT;

    if (velocity) {
        // Finite difference over 5 ms keeps this correct for every combination of space types.
        const XrTime dtNs = 5'000'000;
        Pose a2, b2;
        bool t1, t2;
        if (locateInWorld(s, sp, time + dtNs, &a2, &t1) && locateInWorld(s, base, time + dtNs, &b2, &t2)) {
            Pose rel2 = b2.inverse() * a2;
            float dt = dtNs * 1e-9f;
            Vec3 lv = (rel2.position - rel.position) / dt;
            Vec3 av = (rel2.orientation * rel.orientation.inverse()).toRotationVector() / dt;
            velocity->linearVelocity = {lv.x, lv.y, lv.z};
            velocity->angularVelocity = {av.x, av.y, av.z};
            velocity->velocityFlags = XR_SPACE_VELOCITY_LINEAR_VALID_BIT | XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
        }
    }
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrLocateSpaces(XrSession session, const XrSpacesLocateInfo* info,
                                              XrSpaceLocations* locations) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !locations || info->spaceCount != locations->locationCount) return XR_ERROR_VALIDATION_FAILURE;
    for (uint32_t i = 0; i < info->spaceCount; ++i) {
        XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
        XrResult r = xrLocateSpace(info->spaces[i], info->baseSpace, info->time, &loc);
        if (XR_FAILED(r)) return r;
        locations->locations[i].locationFlags = loc.locationFlags;
        locations->locations[i].pose = loc.pose;
    }
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroySpace(XrSpace space) {
    Space* sp = fromHandle<Space>(space);
    if (!spaces().valid(sp)) return XR_ERROR_HANDLE_INVALID;
    sp->session->spaces.erase(sp);
    spaces().remove(sp);
    delete sp;
    return XR_SUCCESS;
}

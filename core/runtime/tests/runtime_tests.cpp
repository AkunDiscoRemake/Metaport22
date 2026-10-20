// End-to-end test of the runtime the way the OpenXR loader drives it: dlopen, negotiate, then
// a headless session fed by a shared-memory block written like the Metaport service does.
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include <openxr/openxr.h>
#include <openxr/openxr_loader_negotiation.h>

#include "metaport/shared_state.hpp"

namespace {

int g_failures = 0;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)
#define CHECK_XR(expr)                                                       \
    do {                                                                     \
        XrResult r_ = (expr);                                                \
        if (XR_FAILED(r_)) {                                                 \
            std::printf("  FAIL %s:%d: %s -> %d\n", __FILE__, __LINE__, #expr, r_); \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)

PFN_xrGetInstanceProcAddr gipa = nullptr;

template <typename T>
T fn(XrInstance inst, const char* name) {
    PFN_xrVoidFunction f = nullptr;
    gipa(inst, name, &f);
    if (!f) std::printf("  missing %s\n", name);
    return reinterpret_cast<T>(f);
}

XrTime now() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<XrTime>(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
}

std::vector<XrSessionState> drainStates(XrInstance inst, PFN_xrPollEvent poll) {
    std::vector<XrSessionState> out;
    XrEventDataBuffer e{XR_TYPE_EVENT_DATA_BUFFER};
    while (poll(inst, &e) == XR_SUCCESS) {
        if (e.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED)
            out.push_back(reinterpret_cast<XrEventDataSessionStateChanged*>(&e)->state);
        e = {XR_TYPE_EVENT_DATA_BUFFER};
    }
    return out;
}

}  // namespace

int main() {
    // --- Fake Metaport service: a shared block in a temp file ---
    char shmPath[] = "/tmp/metaport_shm_XXXXXX";
    int fd = mkstemp(shmPath);
    if (fd < 0 || ftruncate(fd, sizeof(metaport::SharedBlock)) != 0) return 2;
    auto* block = static_cast<metaport::SharedBlock*>(
        mmap(nullptr, sizeof(metaport::SharedBlock), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    metaport::initSharedBlock(block);
    setenv("METAPORT_SHM_PATH", shmPath, 1);

    metaport::TrackingFrame frame;
    frame.head.position = {0, 1.6f, 0};
    frame.headTracking6Dof = 1;
    frame.floorHeight = 0;
    auto& rc = frame.controllers[metaport::kRight];
    rc.connected = 1;
    rc.orientationValid = 1;
    rc.positionSource = metaport::kPosHandTracked;
    rc.grip.position = {0.2f, 1.2f, -0.4f};
    rc.trigger = 0.8f;
    rc.thumbX = 0.5f;
    rc.buttons = metaport::kBtnA | metaport::kTouchA;
    auto& lh = frame.hands[metaport::kLeft];
    lh.active = 1;
    for (auto& j : lh.joints) j.position = {-0.2f, 1.2f, -0.4f};
    frame.controllers[metaport::kLeft].connected = 0;
    metaport::publishFrame(block, frame);

    // --- Loader side ---
    void* lib = dlopen(FORKXR_RUNTIME_PATH, RTLD_NOW | RTLD_LOCAL);
    if (!lib) { std::printf("dlopen failed: %s\n", dlerror()); return 2; }
    CHECK(dlsym(lib, "xrCreateInstance") == nullptr);  // only the negotiation entry is exported
    auto negotiate = reinterpret_cast<PFN_xrNegotiateLoaderRuntimeInterface>(dlsym(lib, "xrNegotiateLoaderRuntimeInterface"));
    CHECK(negotiate != nullptr);
    XrNegotiateLoaderInfo li{XR_LOADER_INTERFACE_STRUCT_LOADER_INFO, XR_LOADER_INFO_STRUCT_VERSION, sizeof(XrNegotiateLoaderInfo),
                             1, XR_CURRENT_LOADER_RUNTIME_VERSION, XR_MAKE_VERSION(1, 0, 0), XR_MAKE_VERSION(1, 1, 0xFFFFFFFF)};
    XrNegotiateRuntimeRequest rr{XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST, XR_RUNTIME_INFO_STRUCT_VERSION,
                                 sizeof(XrNegotiateRuntimeRequest)};
    CHECK_XR(negotiate(&li, &rr));
    gipa = rr.getInstanceProcAddr;
    CHECK(gipa != nullptr);

    auto createInstance = fn<PFN_xrCreateInstance>(XR_NULL_HANDLE, "xrCreateInstance");
    const char* exts[] = {XR_MND_HEADLESS_EXTENSION_NAME, XR_EXT_HAND_TRACKING_EXTENSION_NAME};
    XrInstanceCreateInfo ici{XR_TYPE_INSTANCE_CREATE_INFO};
    std::strcpy(ici.applicationInfo.applicationName, "metaport-test");
    ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ici.enabledExtensionCount = 2;
    ici.enabledExtensionNames = exts;
    XrInstance inst = XR_NULL_HANDLE;
    CHECK_XR(createInstance(&ici, &inst));

    const char* bogus[] = {"XR_FB_passthrough"};
    XrInstanceCreateInfo bad = ici;
    bad.enabledExtensionCount = 1;
    bad.enabledExtensionNames = bogus;
    XrInstance badInst;
    CHECK(createInstance(&bad, &badInst) == XR_ERROR_EXTENSION_NOT_PRESENT);

    auto getSystem = fn<PFN_xrGetSystem>(inst, "xrGetSystem");
    auto stringToPath = fn<PFN_xrStringToPath>(inst, "xrStringToPath");
    auto createSession = fn<PFN_xrCreateSession>(inst, "xrCreateSession");
    auto poll = fn<PFN_xrPollEvent>(inst, "xrPollEvent");
    auto beginSession = fn<PFN_xrBeginSession>(inst, "xrBeginSession");
    auto createActionSet = fn<PFN_xrCreateActionSet>(inst, "xrCreateActionSet");
    auto createAction = fn<PFN_xrCreateAction>(inst, "xrCreateAction");
    auto suggest = fn<PFN_xrSuggestInteractionProfileBindings>(inst, "xrSuggestInteractionProfileBindings");
    auto attach = fn<PFN_xrAttachSessionActionSets>(inst, "xrAttachSessionActionSets");
    auto sync = fn<PFN_xrSyncActions>(inst, "xrSyncActions");
    auto getFloat = fn<PFN_xrGetActionStateFloat>(inst, "xrGetActionStateFloat");
    auto getBool = fn<PFN_xrGetActionStateBoolean>(inst, "xrGetActionStateBoolean");
    auto getVec2 = fn<PFN_xrGetActionStateVector2f>(inst, "xrGetActionStateVector2f");
    auto createRefSpace = fn<PFN_xrCreateReferenceSpace>(inst, "xrCreateReferenceSpace");
    auto createActionSpace = fn<PFN_xrCreateActionSpace>(inst, "xrCreateActionSpace");
    auto locateSpace = fn<PFN_xrLocateSpace>(inst, "xrLocateSpace");
    auto locateViews = fn<PFN_xrLocateViews>(inst, "xrLocateViews");
    auto waitFrame = fn<PFN_xrWaitFrame>(inst, "xrWaitFrame");
    auto beginFrame = fn<PFN_xrBeginFrame>(inst, "xrBeginFrame");
    auto endFrame = fn<PFN_xrEndFrame>(inst, "xrEndFrame");
    auto requestExit = fn<PFN_xrRequestExitSession>(inst, "xrRequestExitSession");
    auto endSession = fn<PFN_xrEndSession>(inst, "xrEndSession");
    auto destroyInstance = fn<PFN_xrDestroyInstance>(inst, "xrDestroyInstance");
    auto createHand = fn<PFN_xrCreateHandTrackerEXT>(inst, "xrCreateHandTrackerEXT");
    auto locateJoints = fn<PFN_xrLocateHandJointsEXT>(inst, "xrLocateHandJointsEXT");
    PFN_xrVoidFunction refresh = nullptr;
    CHECK(gipa(inst, "xrGetDisplayRefreshRateFB", &refresh) == XR_ERROR_FUNCTION_UNSUPPORTED);  // ext not enabled

    XrSystemGetInfo sgi{XR_TYPE_SYSTEM_GET_INFO};
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XrSystemId sys;
    CHECK_XR(getSystem(inst, &sgi, &sys));

    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
    sci.systemId = sys;
    XrSession session;
    CHECK_XR(createSession(inst, &sci, &session));
    auto states = drainStates(inst, poll);
    CHECK(states.size() == 2 && states[0] == XR_SESSION_STATE_IDLE && states[1] == XR_SESSION_STATE_READY);

    // --- Actions bound for Touch ---
    XrPath left, right, touch;
    stringToPath(inst, "/user/hand/left", &left);
    stringToPath(inst, "/user/hand/right", &right);
    stringToPath(inst, "/interaction_profiles/oculus/touch_controller", &touch);
    XrPath bad1;
    CHECK(stringToPath(inst, "/user//hand", &bad1) == XR_ERROR_PATH_FORMAT_INVALID);

    XrActionSetCreateInfo asci{XR_TYPE_ACTION_SET_CREATE_INFO};
    std::strcpy(asci.actionSetName, "gameplay");
    std::strcpy(asci.localizedActionSetName, "Gameplay");
    XrActionSet set;
    CHECK_XR(createActionSet(inst, &asci, &set));

    XrPath hands[2] = {left, right};
    auto makeAction = [&](const char* name, XrActionType type) {
        XrActionCreateInfo aci{XR_TYPE_ACTION_CREATE_INFO};
        std::strcpy(aci.actionName, name);
        std::strcpy(aci.localizedActionName, name);
        aci.actionType = type;
        aci.countSubactionPaths = 2;
        aci.subactionPaths = hands;
        XrAction a;
        CHECK_XR(createAction(set, &aci, &a));
        return a;
    };
    XrAction trigger = makeAction("trigger", XR_ACTION_TYPE_FLOAT_INPUT);
    XrAction fire = makeAction("fire", XR_ACTION_TYPE_BOOLEAN_INPUT);
    XrAction aButton = makeAction("a_button", XR_ACTION_TYPE_BOOLEAN_INPUT);
    XrAction move = makeAction("move", XR_ACTION_TYPE_VECTOR2F_INPUT);
    XrAction grip = makeAction("grip_pose", XR_ACTION_TYPE_POSE_INPUT);

    std::vector<XrActionSuggestedBinding> b;
    auto bind = [&](XrAction a, const char* path) {
        XrPath p;
        stringToPath(inst, path, &p);
        b.push_back({a, p});
    };
    bind(trigger, "/user/hand/right/input/trigger/value");
    bind(trigger, "/user/hand/left/input/trigger/value");
    bind(fire, "/user/hand/right/input/trigger");  // implied component, bool from float
    bind(aButton, "/user/hand/right/input/a/click");
    bind(move, "/user/hand/right/input/thumbstick");
    bind(grip, "/user/hand/right/input/grip/pose");
    XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    sb.interactionProfile = touch;
    sb.countSuggestedBindings = static_cast<uint32_t>(b.size());
    sb.suggestedBindings = b.data();
    CHECK_XR(suggest(inst, &sb));

    // A binding the profile does not have must be rejected.
    XrPath badBinding;
    stringToPath(inst, "/user/hand/left/input/a/click", &badBinding);  // A is right-only
    XrActionSuggestedBinding badB{aButton, badBinding};
    XrInteractionProfileSuggestedBinding sbBad = sb;
    sbBad.countSuggestedBindings = 1;
    sbBad.suggestedBindings = &badB;
    CHECK(suggest(inst, &sbBad) == XR_ERROR_PATH_UNSUPPORTED);

    XrSessionActionSetsAttachInfo ai{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    ai.countActionSets = 1;
    ai.actionSets = &set;
    CHECK_XR(attach(session, &ai));

    XrSessionBeginInfo sbi{XR_TYPE_SESSION_BEGIN_INFO};
    sbi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    CHECK_XR(beginSession(session, &sbi));
    states = drainStates(inst, poll);
    CHECK(!states.empty() && states.back() == XR_SESSION_STATE_FOCUSED);

    XrActiveActionSet active{set, XR_NULL_PATH};
    XrActionsSyncInfo si{XR_TYPE_ACTIONS_SYNC_INFO};
    si.countActiveActionSets = 1;
    si.activeActionSets = &active;
    CHECK(sync(session, &si) == XR_SUCCESS);

    XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
    gi.action = trigger;
    gi.subactionPath = right;
    XrActionStateFloat fs{XR_TYPE_ACTION_STATE_FLOAT};
    CHECK_XR(getFloat(session, &gi, &fs));
    CHECK(fs.isActive && std::fabs(fs.currentState - 0.8f) < 1e-6f);
    gi.subactionPath = left;  // no controller on the left
    CHECK_XR(getFloat(session, &gi, &fs));
    CHECK(!fs.isActive);

    gi.action = fire;
    gi.subactionPath = XR_NULL_PATH;
    XrActionStateBoolean bs{XR_TYPE_ACTION_STATE_BOOLEAN};
    CHECK_XR(getBool(session, &gi, &bs));
    CHECK(bs.isActive && bs.currentState);
    gi.action = aButton;
    CHECK_XR(getBool(session, &gi, &bs));
    CHECK(bs.currentState);
    gi.action = move;
    XrActionStateVector2f vs{XR_TYPE_ACTION_STATE_VECTOR2F};
    CHECK_XR(getVec2(session, &gi, &vs));
    CHECK(std::fabs(vs.currentState.x - 0.5f) < 1e-6f);
    gi.action = trigger;
    CHECK(getBool(session, &gi, &bs) == XR_ERROR_ACTION_TYPE_MISMATCH);

    // Release the trigger: changedSinceLastSync must report it.
    frame.controllers[metaport::kRight].trigger = 0;
    metaport::publishFrame(block, frame);
    sync(session, &si);
    gi.action = fire;
    CHECK_XR(getBool(session, &gi, &bs));
    CHECK(!bs.currentState && bs.changedSinceLastSync);

    // --- Spaces ---
    XrReferenceSpaceCreateInfo rsci{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rsci.poseInReferenceSpace.orientation.w = 1;
    rsci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    XrSpace local;
    CHECK_XR(createRefSpace(session, &rsci, &local));
    XrSpace floorSpace;
    rsci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR;  // 1.0 app without XR_EXT_local_floor
    CHECK(createRefSpace(session, &rsci, &floorSpace) == XR_ERROR_REFERENCE_SPACE_UNSUPPORTED);

    XrActionSpaceCreateInfo aci{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    aci.action = grip;
    aci.subactionPath = right;
    aci.poseInActionSpace.orientation.w = 1;
    XrSpace gripSpace;
    CHECK_XR(createActionSpace(session, &aci, &gripSpace));

    XrTime t = now();
    XrSpaceVelocity vel{XR_TYPE_SPACE_VELOCITY};
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION, &vel};
    CHECK_XR(locateSpace(gripSpace, local, t, &loc));
    CHECK(loc.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT);
    CHECK(std::fabs(loc.pose.position.z + 0.4f) < 1e-5f);
    CHECK(vel.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT);

    XrViewLocateInfo vli{XR_TYPE_VIEW_LOCATE_INFO};
    vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
    vli.displayTime = t;
    vli.space = local;
    XrViewState vst{XR_TYPE_VIEW_STATE};
    XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
    uint32_t nViews = 0;
    CHECK_XR(locateViews(session, &vli, &vst, 2, &nViews, views));
    CHECK(nViews == 2);
    CHECK(std::fabs((views[1].pose.position.x - views[0].pose.position.x) - frame.display.ipdMeters) < 1e-5f);
    CHECK(std::fabs(views[0].pose.position.y - 1.6f) < 1e-5f);

    // --- Hand tracking ---
    XrHandTrackerCreateInfoEXT hci{XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
    hci.hand = XR_HAND_LEFT_EXT;
    hci.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
    XrHandTrackerEXT handTracker;
    CHECK_XR(createHand(session, &hci, &handTracker));
    XrHandJointLocationEXT joints[XR_HAND_JOINT_COUNT_EXT];
    XrHandJointLocationsEXT jl{XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
    jl.jointCount = XR_HAND_JOINT_COUNT_EXT;
    jl.jointLocations = joints;
    XrHandJointsLocateInfoEXT jli{XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
    jli.baseSpace = local;
    jli.time = t;
    CHECK_XR(locateJoints(handTracker, &jli, &jl));
    CHECK(jl.isActive);
    CHECK(std::fabs(joints[XR_HAND_JOINT_WRIST_EXT].pose.position.x + 0.2f) < 1e-5f);

    // --- Frame loop ---
    XrFrameState fstate{XR_TYPE_FRAME_STATE};
    XrTime lastDisplay = 0;
    for (int i = 0; i < 3; ++i) {
        CHECK_XR(waitFrame(session, nullptr, &fstate));
        CHECK(fstate.predictedDisplayTime > lastDisplay && fstate.shouldRender);
        lastDisplay = fstate.predictedDisplayTime;
        CHECK_XR(beginFrame(session, nullptr));
        XrFrameEndInfo fei{XR_TYPE_FRAME_END_INFO};
        fei.displayTime = fstate.predictedDisplayTime;
        fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        CHECK_XR(endFrame(session, &fei));
    }
    XrFrameEndInfo fei{XR_TYPE_FRAME_END_INFO};
    fei.displayTime = lastDisplay;
    fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    CHECK(endFrame(session, &fei) == XR_ERROR_CALL_ORDER_INVALID);

    // --- Shutdown sequence ---
    CHECK(endSession(session) == XR_ERROR_SESSION_NOT_STOPPING);
    CHECK_XR(requestExit(session));
    states = drainStates(inst, poll);
    CHECK(!states.empty() && states.back() == XR_SESSION_STATE_STOPPING);
    CHECK_XR(endSession(session));
    states = drainStates(inst, poll);
    CHECK(states.size() == 2 && states[1] == XR_SESSION_STATE_EXITING);

    CHECK_XR(destroyInstance(inst));
    dlclose(lib);
    munmap(block, sizeof(metaport::SharedBlock));
    close(fd);
    unlink(shmPath);

    std::printf("runtime tests: %s (%d failures)\n", g_failures ? "FAIL" : "OK", g_failures);
    return g_failures ? 1 : 0;
}

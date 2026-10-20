// Metaport OpenXR runtime — internal object model.
//
// The runtime is loaded by the OpenXR loader inside the VR app's process. It reads the fused
// tracking state that the Metaport service publishes in shared memory and presents the app's
// frames through the Cardboard compositor.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef __ANDROID__
#include <jni.h>
#define XR_USE_PLATFORM_ANDROID
#define XR_USE_GRAPHICS_API_OPENGL_ES
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#endif

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include "metaport/math.hpp"
#include "metaport/shared_state.hpp"

namespace mpxr {

using metaport::Pose;
using metaport::Quat;
using metaport::Side;
using metaport::TrackingFrame;
using metaport::Vec3;

constexpr XrSystemId kSystemId = 1;
constexpr const char* kRuntimeName = "Metaport (Cardboard)";
constexpr const char* kSystemName = "Metaport Cardboard Headset";

// ---------------------------------------------------------------------------------------------
// Tracking source: where the runtime gets the TrackingFrame from.

class TrackingSource {
public:
    virtual ~TrackingSource() = default;
    virtual bool read(TrackingFrame* out) = 0;
    virtual void requestHaptic(Side side, float amplitude, float frequencyHz, int64_t durationNs) = 0;
};

// Maps the Metaport service's block. On Android the fd comes from the service's ContentProvider
// (see tracking_source.cpp); on desktop builds from the file named by $METAPORT_SHM_PATH.
std::unique_ptr<TrackingSource> createTrackingSource(void* javaVm, void* androidContext);

// ---------------------------------------------------------------------------------------------
// Compositor: receives finished eye images.

struct EyeSubmission {
    uint32_t texture = 0;     // GL texture name (0 when headless)
    uint32_t arrayIndex = 0;  // layer of a GL_TEXTURE_2D_ARRAY swapchain, 0 otherwise
    bool isArray = false;
    bool srgb = false;        // GL_SRGB8_ALPHA8: sampled values are linear and must be re-encoded
    XrRect2Di rect{};
    uint32_t texWidth = 0, texHeight = 0;
    XrPosef pose{};
    XrFovf fov{};
};

class Compositor {
public:
    virtual ~Compositor() = default;
    // Called from xrEndFrame on the app's GL thread. `eyes[i].pose` is the world pose the app
    // rendered with; `latestHead` is the freshest prediction for the same display time, used for
    // rotational reprojection ("timewarp").
    virtual void submit(const EyeSubmission eyes[2], XrTime displayTime, const Pose& latestHead) = 0;
    // Nanoseconds between vsyncs and the timestamp of the most recent one.
    virtual int64_t framePeriodNs() const = 0;
    virtual int64_t lastVsyncNs() const = 0;
};

std::unique_ptr<Compositor> createNullCompositor(float refreshHz);
#ifdef __ANDROID__
std::unique_ptr<Compositor> createCardboardCompositor(JavaVM* vm, jobject activity, EGLDisplay display,
                                                      EGLContext appContext, const metaport::DisplayInfo& info);
#endif

// ---------------------------------------------------------------------------------------------
// Paths

class PathTable {
public:
    XrPath intern(const std::string& s);
    bool lookup(XrPath p, std::string* out) const;

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, XrPath> byName_;
    std::vector<std::string> names_ = {""};  // XR_NULL_PATH = 0
};

// ---------------------------------------------------------------------------------------------
// Input model

enum class InputKind { kBool, kFloat, kVec2, kPose, kHaptic };

// A physical input on the virtual controllers (see input.cpp for the binding table).
enum class Source {
    kNone,
    kTriggerValue, kTriggerTouch, kTriggerClick,
    kSqueezeValue, kSqueezeClick,
    kThumbstick, kThumbstickX, kThumbstickY, kThumbstickClick, kThumbstickTouch, kThumbrestTouch,
    kButtonAClick, kButtonATouch, kButtonBClick, kButtonBTouch,  // A/B right, X/Y left
    kMenuClick, kSelectClick,
    kGripPose, kAimPose,
    kHaptic,
};

struct Binding {
    Side side;
    Source source;
    XrPath path = XR_NULL_PATH;  // the binding path as the app wrote it
};

struct Instance;
struct Session;
struct ActionSet;

struct Action {
    ActionSet* set = nullptr;
    std::string name;
    XrActionType type{};
    std::vector<XrPath> subactionPaths;
    // interaction profile path -> bindings suggested for it
    std::map<XrPath, std::vector<Binding>> suggested;

    struct State {
        bool boolValue = false;
        float floatValue = 0;
        XrVector2f vec2{0, 0};
        bool active = false;
        bool changed = false;
        XrTime lastChange = 0;
    };
    // [0] = combined (XR_NULL_PATH), [1] = left, [2] = right
    State state[3];
};

struct ActionSet {
    Instance* instance = nullptr;
    std::string name;
    uint32_t priority = 0;
    bool attached = false;
    std::vector<std::unique_ptr<Action>> actions;
};

enum class SpaceKind { kReference, kAction };

struct Space {
    Session* session = nullptr;
    SpaceKind kind{};
    XrReferenceSpaceType refType{};
    Action* action = nullptr;
    XrPath subaction = XR_NULL_PATH;
    Pose offset;  // poseInReferenceSpace / poseInActionSpace
};

struct Swapchain {
    Session* session = nullptr;
    XrSwapchainCreateInfo info{};
    std::vector<uint32_t> textures;
    uint32_t next = 0;
    std::deque<uint32_t> acquired;
    int32_t lastReleased = -1;
    bool isArray = false;
};

struct HandTracker {
    Session* session = nullptr;
    Side side{};
};

struct Session {
    Instance* instance = nullptr;
    XrSessionState state = XR_SESSION_STATE_UNKNOWN;
    bool running = false;
    bool exitRequested = false;
    bool headless = false;
    XrViewConfigurationType viewConfig = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

    std::vector<ActionSet*> attachedSets;
    XrPath currentProfile[2] = {XR_NULL_PATH, XR_NULL_PATH};
    TrackingFrame frame;  // snapshot taken by xrSyncActions / xrWaitFrame

    std::unique_ptr<Compositor> compositor;
    std::set<Space*> spaces;
    std::set<Swapchain*> swapchains;
    std::set<HandTracker*> handTrackers;

    XrTime nextDisplayTime = 0;
    bool frameBegun = false;
    int64_t framesWaited = 0;
    bool pinchSelect[2] = {false, false};  // hysteresis for hand-emulated select
#ifdef __ANDROID__
    EGLDisplay eglDisplay = EGL_NO_DISPLAY;
    EGLContext eglContext = EGL_NO_CONTEXT;
#endif
};

struct Instance {
    std::string appName;
    uint32_t appVersion = 0;
    XrVersion apiVersion = 0;
    std::set<std::string> enabledExtensions;
    PathTable paths;
    std::unique_ptr<TrackingSource> tracking;
    std::mutex eventMutex;
    std::deque<XrEventDataBuffer> events;
    std::set<Session*> sessions;
    std::set<ActionSet*> actionSets;
    bool systemQueried = false;
    void* javaVm = nullptr;
    void* activity = nullptr;

    bool hasExtension(const char* name) const { return enabledExtensions.count(name) != 0; }
    void pushSessionState(Session* s, XrSessionState state, XrTime time);
};

// ---------------------------------------------------------------------------------------------
// Handle registry: every handle we hand out is a pointer we can validate.

template <typename T>
class Registry {
public:
    void add(T* p) { std::lock_guard<std::mutex> l(m_); live_.insert(p); }
    void remove(T* p) { std::lock_guard<std::mutex> l(m_); live_.erase(p); }
    bool valid(T* p) { std::lock_guard<std::mutex> l(m_); return live_.count(p) != 0; }

private:
    std::mutex m_;
    std::set<T*> live_;
};

Registry<Instance>& instances();
Registry<Session>& sessions();
Registry<Space>& spaces();
Registry<ActionSet>& actionSets();
Registry<Action>& actions();
Registry<Swapchain>& swapchains();
Registry<HandTracker>& handTrackers();

template <typename H, typename T>
H toHandle(T* p) { return reinterpret_cast<H>(p); }
template <typename T, typename H>
T* fromHandle(H h) { return reinterpret_cast<T*>(h); }

// ---------------------------------------------------------------------------------------------
// Helpers shared between translation units.

XrTime nowXrTime();
inline XrPosef toXr(const Pose& p) {
    return {{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w},
            {p.position.x, p.position.y, p.position.z}};
}
inline Pose fromXr(const XrPosef& p) {
    return {Quat{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}.normalized(),
            Vec3{p.position.x, p.position.y, p.position.z}};
}

// World-space pose of a space at `time`. Returns false if it cannot be located.
bool locateInWorld(Session* s, Space* space, XrTime time, Pose* out, bool* positionTracked);
// Pose of a controller source (grip/aim/palm) for one hand in world space.
bool controllerPose(Session* s, Side side, Source src, XrTime time, Pose* out, bool* positionTracked);
Pose headPoseAt(Session* s, XrTime time);
void refreshFrame(Session* s);

// Implemented in input.cpp; resolves a binding string like "/user/hand/left/input/trigger/value".
bool resolveBinding(const std::string& profile, const std::string& path, XrActionType type, Binding* out);
bool isKnownProfile(const std::string& profile);
void updateActionStates(Session* s, const XrActionsSyncInfo* info);

}  // namespace mpxr

// Entry points implemented across the translation units (OpenXR signatures).
#define MPXR_FUNCTIONS(_)                         \
    _(xrGetInstanceProcAddr)                      \
    _(xrEnumerateApiLayerProperties)              \
    _(xrEnumerateInstanceExtensionProperties)     \
    _(xrCreateInstance)                           \
    _(xrDestroyInstance)                          \
    _(xrGetInstanceProperties)                    \
    _(xrPollEvent)                                \
    _(xrResultToString)                           \
    _(xrStructureTypeToString)                    \
    _(xrGetSystem)                                \
    _(xrGetSystemProperties)                      \
    _(xrEnumerateEnvironmentBlendModes)           \
    _(xrCreateSession)                            \
    _(xrDestroySession)                           \
    _(xrEnumerateReferenceSpaces)                 \
    _(xrCreateReferenceSpace)                     \
    _(xrGetReferenceSpaceBoundsRect)              \
    _(xrCreateActionSpace)                        \
    _(xrLocateSpace)                              \
    _(xrLocateSpaces)                             \
    _(xrDestroySpace)                             \
    _(xrEnumerateViewConfigurations)              \
    _(xrGetViewConfigurationProperties)           \
    _(xrEnumerateViewConfigurationViews)          \
    _(xrEnumerateSwapchainFormats)                \
    _(xrCreateSwapchain)                          \
    _(xrDestroySwapchain)                         \
    _(xrEnumerateSwapchainImages)                 \
    _(xrAcquireSwapchainImage)                    \
    _(xrWaitSwapchainImage)                       \
    _(xrReleaseSwapchainImage)                    \
    _(xrBeginSession)                             \
    _(xrEndSession)                               \
    _(xrRequestExitSession)                       \
    _(xrWaitFrame)                                \
    _(xrBeginFrame)                               \
    _(xrEndFrame)                                 \
    _(xrLocateViews)                              \
    _(xrStringToPath)                             \
    _(xrPathToString)                             \
    _(xrCreateActionSet)                          \
    _(xrDestroyActionSet)                         \
    _(xrCreateAction)                             \
    _(xrDestroyAction)                            \
    _(xrSuggestInteractionProfileBindings)        \
    _(xrAttachSessionActionSets)                  \
    _(xrGetCurrentInteractionProfile)             \
    _(xrGetActionStateBoolean)                    \
    _(xrGetActionStateFloat)                      \
    _(xrGetActionStateVector2f)                   \
    _(xrGetActionStatePose)                       \
    _(xrSyncActions)                              \
    _(xrEnumerateBoundSourcesForAction)           \
    _(xrGetInputSourceLocalizedName)              \
    _(xrApplyHapticFeedback)                      \
    _(xrStopHapticFeedback)                       \
    _(xrCreateHandTrackerEXT)                     \
    _(xrDestroyHandTrackerEXT)                    \
    _(xrLocateHandJointsEXT)                      \
    _(xrEnumerateDisplayRefreshRatesFB)           \
    _(xrGetDisplayRefreshRateFB)                  \
    _(xrRequestDisplayRefreshRateFB)

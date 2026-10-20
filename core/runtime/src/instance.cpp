// Loader negotiation, instance, system and event functions.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <openxr/openxr_loader_negotiation.h>
#include <openxr/openxr_reflection.h>

#include "runtime.hpp"

namespace forkxr {

#define DEFINE_REGISTRY(T, name) \
    Registry<T>& name() {         \
        static Registry<T> r;     \
        return r;                 \
    }
DEFINE_REGISTRY(Instance, instances)
DEFINE_REGISTRY(Session, sessions)
DEFINE_REGISTRY(Space, spaces)
DEFINE_REGISTRY(ActionSet, actionSets)
DEFINE_REGISTRY(Action, actions)
DEFINE_REGISTRY(Swapchain, swapchains)
DEFINE_REGISTRY(HandTracker, handTrackers)
#undef DEFINE_REGISTRY

XrTime nowXrTime() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<XrTime>(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
}

XrPath PathTable::intern(const std::string& s) {
    std::lock_guard<std::mutex> l(mutex_);
    auto it = byName_.find(s);
    if (it != byName_.end()) return it->second;
    XrPath p = names_.size();
    names_.push_back(s);
    byName_[s] = p;
    return p;
}

bool PathTable::lookup(XrPath p, std::string* out) const {
    std::lock_guard<std::mutex> l(mutex_);
    if (p == XR_NULL_PATH || p >= names_.size()) return false;
    *out = names_[p];
    return true;
}

void Instance::pushSessionState(Session* s, XrSessionState newState, XrTime time) {
    s->state = newState;
    XrEventDataBuffer buf{XR_TYPE_EVENT_DATA_BUFFER};
    auto* e = reinterpret_cast<XrEventDataSessionStateChanged*>(&buf);
    e->type = XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED;
    e->next = nullptr;
    e->session = toHandle<XrSession>(s);
    e->state = newState;
    e->time = time;
    std::lock_guard<std::mutex> l(eventMutex);
    events.push_back(buf);
}

namespace {

struct ExtensionInfo {
    const char* name;
    uint32_t version;
};

const ExtensionInfo kExtensions[] = {
#ifdef __ANDROID__
    {XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME, XR_KHR_android_create_instance_SPEC_VERSION},
    {XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME, XR_KHR_opengl_es_enable_SPEC_VERSION},
#endif
    {XR_MND_HEADLESS_EXTENSION_NAME, XR_MND_headless_SPEC_VERSION},
    {XR_EXT_HAND_TRACKING_EXTENSION_NAME, XR_EXT_hand_tracking_SPEC_VERSION},
    {XR_EXT_LOCAL_FLOOR_EXTENSION_NAME, XR_EXT_local_floor_SPEC_VERSION},
    {XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME, XR_FB_display_refresh_rate_SPEC_VERSION},
};

// A few enum names are longer than the fixed OpenXR buffers; truncation is intended.
void copyName(char* dst, size_t size, const char* src) {
    size_t n = std::min(size - 1, std::strlen(src));
    std::memcpy(dst, src, n);
    dst[n] = 0;
}

bool isSupportedExtension(const char* name) {
    for (const auto& e : kExtensions)
        if (std::strcmp(e.name, name) == 0) return true;
    return false;
}

template <typename T>
XrResult enumerate(const std::vector<T>& items, uint32_t capacity, uint32_t* count, T* out) {
    if (!count) return XR_ERROR_VALIDATION_FAILURE;
    *count = static_cast<uint32_t>(items.size());
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < items.size()) return XR_ERROR_SIZE_INSUFFICIENT;
    for (size_t i = 0; i < items.size(); ++i) out[i] = items[i];
    return XR_SUCCESS;
}

}  // namespace

}  // namespace forkxr

using namespace forkxr;

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateApiLayerProperties(uint32_t, uint32_t* countOutput, XrApiLayerProperties*) {
    if (!countOutput) return XR_ERROR_VALIDATION_FAILURE;
    *countOutput = 0;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateInstanceExtensionProperties(const char* layerName, uint32_t capacity,
                                                                      uint32_t* countOutput,
                                                                      XrExtensionProperties* props) {
    if (layerName) return XR_ERROR_API_LAYER_NOT_PRESENT;
    if (!countOutput) return XR_ERROR_VALIDATION_FAILURE;
    uint32_t n = sizeof(kExtensions) / sizeof(kExtensions[0]);
    *countOutput = n;
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < n) return XR_ERROR_SIZE_INSUFFICIENT;
    for (uint32_t i = 0; i < n; ++i) {
        if (props[i].type != XR_TYPE_EXTENSION_PROPERTIES) return XR_ERROR_VALIDATION_FAILURE;
        std::strncpy(props[i].extensionName, kExtensions[i].name, XR_MAX_EXTENSION_NAME_SIZE - 1);
        props[i].extensionName[XR_MAX_EXTENSION_NAME_SIZE - 1] = 0;
        props[i].extensionVersion = kExtensions[i].version;
    }
    return XR_SUCCESS;
}

#ifdef __ANDROID__
namespace {
JavaVM* g_vm = nullptr;
jobject g_context = nullptr;
}  // namespace

XRAPI_ATTR XrResult XRAPI_CALL xrInitializeLoaderKHR(const XrLoaderInitInfoBaseHeaderKHR* info) {
    if (!info || info->type != XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR) return XR_ERROR_VALIDATION_FAILURE;
    auto* a = reinterpret_cast<const XrLoaderInitInfoAndroidKHR*>(info);
    g_vm = static_cast<JavaVM*>(a->applicationVM);
    JNIEnv* env = nullptr;
    if (g_vm && g_vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) == JNI_OK && a->applicationContext) {
        if (g_context) env->DeleteGlobalRef(g_context);
        g_context = env->NewGlobalRef(static_cast<jobject>(a->applicationContext));
    }
    return XR_SUCCESS;
}
#endif

XRAPI_ATTR XrResult XRAPI_CALL xrCreateInstance(const XrInstanceCreateInfo* info, XrInstance* out) {
    if (!info || !out || info->type != XR_TYPE_INSTANCE_CREATE_INFO) return XR_ERROR_VALIDATION_FAILURE;
    XrVersion v = info->applicationInfo.apiVersion;
    if (XR_VERSION_MAJOR(v) != 1 || XR_VERSION_MINOR(v) > 1) return XR_ERROR_API_VERSION_UNSUPPORTED;
    if (info->enabledApiLayerCount > 0) return XR_ERROR_API_LAYER_NOT_PRESENT;
    for (uint32_t i = 0; i < info->enabledExtensionCount; ++i)
        if (!isSupportedExtension(info->enabledExtensionNames[i])) return XR_ERROR_EXTENSION_NOT_PRESENT;

    auto inst = std::make_unique<Instance>();
    inst->appName = info->applicationInfo.applicationName;
    inst->appVersion = info->applicationInfo.applicationVersion;
    inst->apiVersion = v;
    for (uint32_t i = 0; i < info->enabledExtensionCount; ++i) inst->enabledExtensions.insert(info->enabledExtensionNames[i]);

#ifdef __ANDROID__
    for (auto* n = static_cast<const XrBaseInStructure*>(info->next); n; n = n->next) {
        if (n->type == XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR) {
            auto* a = reinterpret_cast<const XrInstanceCreateInfoAndroidKHR*>(n);
            if (!g_vm) g_vm = static_cast<JavaVM*>(a->applicationVM);
            inst->activity = a->applicationActivity;
        }
    }
    inst->javaVm = g_vm;
    inst->tracking = createTrackingSource(g_vm, g_context ? g_context : inst->activity);
#else
    inst->tracking = createTrackingSource(nullptr, nullptr);
#endif
    if (!inst->tracking) return XR_ERROR_RUNTIME_UNAVAILABLE;  // Metaport service not running

    Instance* raw = inst.release();
    instances().add(raw);
    *out = toHandle<XrInstance>(raw);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyInstance(XrInstance instance) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    for (Session* s : std::set<Session*>(inst->sessions)) xrDestroySession(toHandle<XrSession>(s));
    for (ActionSet* a : std::set<ActionSet*>(inst->actionSets)) xrDestroyActionSet(toHandle<XrActionSet>(a));
    instances().remove(inst);
    delete inst;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetInstanceProperties(XrInstance instance, XrInstanceProperties* props) {
    if (!instances().valid(fromHandle<Instance>(instance))) return XR_ERROR_HANDLE_INVALID;
    if (!props || props->type != XR_TYPE_INSTANCE_PROPERTIES) return XR_ERROR_VALIDATION_FAILURE;
    props->runtimeVersion = XR_MAKE_VERSION(0, 1, 0);
    std::strncpy(props->runtimeName, kRuntimeName, XR_MAX_RUNTIME_NAME_SIZE - 1);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrPollEvent(XrInstance instance, XrEventDataBuffer* buffer) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    if (!buffer) return XR_ERROR_VALIDATION_FAILURE;
    std::lock_guard<std::mutex> l(inst->eventMutex);
    if (inst->events.empty()) return XR_EVENT_UNAVAILABLE;
    *buffer = inst->events.front();
    inst->events.pop_front();
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrResultToString(XrInstance, XrResult value, char buffer[XR_MAX_RESULT_STRING_SIZE]) {
#define RESULT_CASE(name, val) \
    case name: copyName(buffer, XR_MAX_RESULT_STRING_SIZE, #name); return XR_SUCCESS;
    switch (value) {
        XR_LIST_ENUM_XrResult(RESULT_CASE)
        default:
            std::snprintf(buffer, XR_MAX_RESULT_STRING_SIZE, "%s_%d", value < 0 ? "XR_UNKNOWN_FAILURE" : "XR_UNKNOWN_SUCCESS",
                          static_cast<int>(value));
            return XR_SUCCESS;
    }
#undef RESULT_CASE
}

XRAPI_ATTR XrResult XRAPI_CALL xrStructureTypeToString(XrInstance, XrStructureType value,
                                                       char buffer[XR_MAX_STRUCTURE_NAME_SIZE]) {
#define TYPE_CASE(name, val) \
    case name: copyName(buffer, XR_MAX_STRUCTURE_NAME_SIZE, #name); return XR_SUCCESS;
    switch (value) {
        XR_LIST_ENUM_XrStructureType(TYPE_CASE)
        default:
            std::snprintf(buffer, XR_MAX_STRUCTURE_NAME_SIZE, "XR_UNKNOWN_STRUCTURE_TYPE_%d", static_cast<int>(value));
            return XR_SUCCESS;
    }
#undef TYPE_CASE
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetSystem(XrInstance instance, const XrSystemGetInfo* info, XrSystemId* systemId) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !systemId || info->type != XR_TYPE_SYSTEM_GET_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (info->formFactor != XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY) return XR_ERROR_FORM_FACTOR_UNSUPPORTED;
    inst->systemQueried = true;
    *systemId = kSystemId;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetSystemProperties(XrInstance instance, XrSystemId systemId, XrSystemProperties* props) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    if (systemId != kSystemId) return XR_ERROR_SYSTEM_INVALID;
    if (!props || props->type != XR_TYPE_SYSTEM_PROPERTIES) return XR_ERROR_VALIDATION_FAILURE;
    props->systemId = kSystemId;
    props->vendorId = 0;
    std::strncpy(props->systemName, kSystemName, XR_MAX_SYSTEM_NAME_SIZE - 1);
    props->graphicsProperties.maxLayerCount = 1;
    props->graphicsProperties.maxSwapchainImageWidth = 4096;
    props->graphicsProperties.maxSwapchainImageHeight = 4096;
    props->trackingProperties.orientationTracking = XR_TRUE;
    props->trackingProperties.positionTracking = XR_TRUE;  // ARCore 6DoF
    for (auto* n = static_cast<XrBaseOutStructure*>(props->next); n; n = n->next) {
        if (n->type == XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT && inst->hasExtension(XR_EXT_HAND_TRACKING_EXTENSION_NAME))
            reinterpret_cast<XrSystemHandTrackingPropertiesEXT*>(n)->supportsHandTracking = XR_TRUE;
    }
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateEnvironmentBlendModes(XrInstance instance, XrSystemId systemId,
                                                                XrViewConfigurationType viewConfig, uint32_t capacity,
                                                                uint32_t* count, XrEnvironmentBlendMode* modes) {
    if (!instances().valid(fromHandle<Instance>(instance))) return XR_ERROR_HANDLE_INVALID;
    if (systemId != kSystemId) return XR_ERROR_SYSTEM_INVALID;
    if (viewConfig != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    return enumerate<XrEnvironmentBlendMode>({XR_ENVIRONMENT_BLEND_MODE_OPAQUE}, capacity, count, modes);
}

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateViewConfigurations(XrInstance instance, XrSystemId systemId, uint32_t capacity,
                                                             uint32_t* count, XrViewConfigurationType* types) {
    if (!instances().valid(fromHandle<Instance>(instance))) return XR_ERROR_HANDLE_INVALID;
    if (systemId != kSystemId) return XR_ERROR_SYSTEM_INVALID;
    return enumerate<XrViewConfigurationType>({XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO}, capacity, count, types);
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetViewConfigurationProperties(XrInstance instance, XrSystemId systemId,
                                                                XrViewConfigurationType type,
                                                                XrViewConfigurationProperties* props) {
    if (!instances().valid(fromHandle<Instance>(instance))) return XR_ERROR_HANDLE_INVALID;
    if (systemId != kSystemId) return XR_ERROR_SYSTEM_INVALID;
    if (type != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    if (!props) return XR_ERROR_VALIDATION_FAILURE;
    props->viewConfigurationType = type;
    props->fovMutable = XR_FALSE;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateViewConfigurationViews(XrInstance instance, XrSystemId systemId,
                                                                 XrViewConfigurationType type, uint32_t capacity,
                                                                 uint32_t* count, XrViewConfigurationView* views) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    if (systemId != kSystemId) return XR_ERROR_SYSTEM_INVALID;
    if (type != XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO) return XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    if (!count) return XR_ERROR_VALIDATION_FAILURE;
    *count = 2;
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < 2) return XR_ERROR_SIZE_INSUFFICIENT;
    TrackingFrame f;
    inst->tracking->read(&f);
    for (int i = 0; i < 2; ++i) {
        views[i].recommendedImageRectWidth = f.display.eyeWidth;
        views[i].recommendedImageRectHeight = f.display.eyeHeight;
        views[i].maxImageRectWidth = 4096;
        views[i].maxImageRectHeight = 4096;
        views[i].recommendedSwapchainSampleCount = 1;
        views[i].maxSwapchainSampleCount = 4;
    }
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrStringToPath(XrInstance instance, const char* str, XrPath* path) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    if (!str || !path) return XR_ERROR_VALIDATION_FAILURE;
    size_t len = std::strlen(str);
    if (len == 0 || str[0] != '/' || str[len - 1] == '/' || len >= XR_MAX_PATH_LENGTH) return XR_ERROR_PATH_FORMAT_INVALID;
    for (size_t i = 0; i < len; ++i) {
        char c = str[i];
        bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '/';
        if (!ok || (c == '/' && i + 1 < len && str[i + 1] == '/')) return XR_ERROR_PATH_FORMAT_INVALID;
    }
    *path = inst->paths.intern(str);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrPathToString(XrInstance instance, XrPath path, uint32_t capacity, uint32_t* count,
                                              char* buffer) {
    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;
    std::string s;
    if (!inst->paths.lookup(path, &s)) return XR_ERROR_PATH_INVALID;
    if (!count) return XR_ERROR_VALIDATION_FAILURE;
    *count = static_cast<uint32_t>(s.size() + 1);
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < s.size() + 1) return XR_ERROR_SIZE_INSUFFICIENT;
    std::memcpy(buffer, s.c_str(), s.size() + 1);
    return XR_SUCCESS;
}

// --- XR_FB_display_refresh_rate: Cardboard runs at the phone's refresh rate ------------------

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateDisplayRefreshRatesFB(XrSession session, uint32_t capacity, uint32_t* count,
                                                                float* rates) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    return enumerate<float>({s->frame.display.refreshHz}, capacity, count, rates);
}

XRAPI_ATTR XrResult XRAPI_CALL xrGetDisplayRefreshRateFB(XrSession session, float* rate) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!rate) return XR_ERROR_VALIDATION_FAILURE;
    *rate = s->frame.display.refreshHz;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrRequestDisplayRefreshRateFB(XrSession session, float rate) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (rate != 0 && std::fabs(rate - s->frame.display.refreshHz) > 0.5f) return XR_ERROR_DISPLAY_REFRESH_RATE_UNSUPPORTED_FB;
    return XR_SUCCESS;
}

// --- Dispatch -----------------------------------------------------------------------------------

namespace {

bool requiresExtension(const char* fn, const char** ext) {
    struct Entry { const char* fn; const char* ext; };
    static const Entry kTable[] = {
        {"xrCreateHandTrackerEXT", XR_EXT_HAND_TRACKING_EXTENSION_NAME},
        {"xrDestroyHandTrackerEXT", XR_EXT_HAND_TRACKING_EXTENSION_NAME},
        {"xrLocateHandJointsEXT", XR_EXT_HAND_TRACKING_EXTENSION_NAME},
        {"xrEnumerateDisplayRefreshRatesFB", XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME},
        {"xrGetDisplayRefreshRateFB", XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME},
        {"xrRequestDisplayRefreshRateFB", XR_FB_DISPLAY_REFRESH_RATE_EXTENSION_NAME},
#ifdef __ANDROID__
        {"xrGetOpenGLESGraphicsRequirementsKHR", XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME},
#endif
    };
    for (const auto& e : kTable)
        if (std::strcmp(e.fn, fn) == 0) { *ext = e.ext; return true; }
    return false;
}

}  // namespace

#ifdef __ANDROID__
XRAPI_ATTR XrResult XRAPI_CALL xrGetOpenGLESGraphicsRequirementsKHR(XrInstance, XrSystemId,
                                                                    XrGraphicsRequirementsOpenGLESKHR*);
#endif

XRAPI_ATTR XrResult XRAPI_CALL xrGetInstanceProcAddr(XrInstance instance, const char* name, PFN_xrVoidFunction* function) {
    if (!name || !function) return XR_ERROR_VALIDATION_FAILURE;
    *function = nullptr;

    // Functions callable without an instance.
    if (instance == XR_NULL_HANDLE) {
        if (std::strcmp(name, "xrEnumerateInstanceExtensionProperties") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(xrEnumerateInstanceExtensionProperties);
        else if (std::strcmp(name, "xrEnumerateApiLayerProperties") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(xrEnumerateApiLayerProperties);
        else if (std::strcmp(name, "xrCreateInstance") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(xrCreateInstance);
#ifdef __ANDROID__
        else if (std::strcmp(name, "xrInitializeLoaderKHR") == 0)
            *function = reinterpret_cast<PFN_xrVoidFunction>(xrInitializeLoaderKHR);
#endif
        return *function ? XR_SUCCESS : XR_ERROR_HANDLE_INVALID;
    }

    Instance* inst = fromHandle<Instance>(instance);
    if (!instances().valid(inst)) return XR_ERROR_HANDLE_INVALID;

    const char* ext = nullptr;
    if (requiresExtension(name, &ext) && !inst->hasExtension(ext)) return XR_ERROR_FUNCTION_UNSUPPORTED;

#define LOOKUP(fn) \
    if (std::strcmp(name, #fn) == 0) { *function = reinterpret_cast<PFN_xrVoidFunction>(fn); return XR_SUCCESS; }
    MPXR_FUNCTIONS(LOOKUP)
#ifdef __ANDROID__
    LOOKUP(xrGetOpenGLESGraphicsRequirementsKHR)
#endif
#undef LOOKUP
    return XR_ERROR_FUNCTION_UNSUPPORTED;
}

extern "C" __attribute__((visibility("default"))) XRAPI_ATTR XrResult XRAPI_CALL
xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo* loaderInfo, XrNegotiateRuntimeRequest* request) {
    if (!loaderInfo || !request || loaderInfo->structType != XR_LOADER_INTERFACE_STRUCT_LOADER_INFO ||
        request->structType != XR_LOADER_INTERFACE_STRUCT_RUNTIME_REQUEST)
        return XR_ERROR_INITIALIZATION_FAILED;
    if (loaderInfo->minInterfaceVersion > XR_CURRENT_LOADER_RUNTIME_VERSION ||
        loaderInfo->maxInterfaceVersion < XR_CURRENT_LOADER_RUNTIME_VERSION)
        return XR_ERROR_INITIALIZATION_FAILED;
    if (XR_VERSION_MAJOR(loaderInfo->minApiVersion) > 1 || XR_VERSION_MAJOR(loaderInfo->maxApiVersion) < 1)
        return XR_ERROR_INITIALIZATION_FAILED;
    request->runtimeInterfaceVersion = XR_CURRENT_LOADER_RUNTIME_VERSION;
    request->runtimeApiVersion = XR_MAKE_VERSION(1, 1, 0);
    request->getInstanceProcAddr = xrGetInstanceProcAddr;
    return XR_SUCCESS;
}

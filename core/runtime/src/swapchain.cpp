// OpenGL ES swapchains. Images are plain GL textures created in the app's context; the
// Cardboard compositor shares that context and samples them in xrEndFrame.
#include <algorithm>

#include "runtime.hpp"

using namespace forkxr;

namespace {

#ifdef __ANDROID__
const int64_t kFormats[] = {
    0x8C43,  // GL_SRGB8_ALPHA8 (preferred: the compositor writes to an sRGB-correct surface)
    0x8058,  // GL_RGBA8
    0x88F0,  // GL_DEPTH24_STENCIL8
    0x81A6,  // GL_DEPTH_COMPONENT24
    0x8CAC,  // GL_DEPTH_COMPONENT32F
};
constexpr uint32_t kImageCount = 3;
#endif

}  // namespace

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateSwapchainFormats(XrSession session, uint32_t capacity, uint32_t* count,
                                                           int64_t* formats) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!count) return XR_ERROR_VALIDATION_FAILURE;
#ifdef __ANDROID__
    uint32_t n = s->headless ? 0 : static_cast<uint32_t>(sizeof(kFormats) / sizeof(kFormats[0]));
    *count = n;
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < n) return XR_ERROR_SIZE_INSUFFICIENT;
    std::copy(kFormats, kFormats + n, formats);
#else
    (void)capacity;
    (void)formats;
    *count = 0;
#endif
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrCreateSwapchain(XrSession session, const XrSwapchainCreateInfo* info, XrSwapchain* out) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !out || info->type != XR_TYPE_SWAPCHAIN_CREATE_INFO) return XR_ERROR_VALIDATION_FAILURE;
    if (s->headless) return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
#ifdef __ANDROID__
    if (std::find(std::begin(kFormats), std::end(kFormats), info->format) == std::end(kFormats))
        return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
    if (info->faceCount != 1 || info->mipCount < 1 || info->arraySize < 1 || info->width == 0 || info->height == 0)
        return XR_ERROR_FEATURE_UNSUPPORTED;
    if (eglGetCurrentContext() == EGL_NO_CONTEXT) return XR_ERROR_GRAPHICS_DEVICE_INVALID;

    auto* sc = new Swapchain;
    sc->session = s;
    sc->info = *info;
    sc->isArray = info->arraySize > 1;
    GLenum target = sc->isArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D;
    sc->textures.resize(kImageCount);
    glGenTextures(kImageCount, sc->textures.data());
    for (uint32_t tex : sc->textures) {
        glBindTexture(target, tex);
        if (sc->isArray)
            glTexStorage3D(target, info->mipCount, static_cast<GLenum>(info->format), info->width, info->height, info->arraySize);
        else
            glTexStorage2D(target, info->mipCount, static_cast<GLenum>(info->format), info->width, info->height);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(target, 0);
    if (glGetError() != GL_NO_ERROR) {
        glDeleteTextures(kImageCount, sc->textures.data());
        delete sc;
        return XR_ERROR_RUNTIME_FAILURE;
    }
    swapchains().add(sc);
    s->swapchains.insert(sc);
    *out = toHandle<XrSwapchain>(sc);
    return XR_SUCCESS;
#else
    return XR_ERROR_SWAPCHAIN_FORMAT_UNSUPPORTED;
#endif
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroySwapchain(XrSwapchain swapchain) {
    Swapchain* sc = fromHandle<Swapchain>(swapchain);
    if (!swapchains().valid(sc)) return XR_ERROR_HANDLE_INVALID;
#ifdef __ANDROID__
    if (!sc->textures.empty() && eglGetCurrentContext() != EGL_NO_CONTEXT)
        glDeleteTextures(static_cast<GLsizei>(sc->textures.size()), sc->textures.data());
#endif
    sc->session->swapchains.erase(sc);
    swapchains().remove(sc);
    delete sc;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrEnumerateSwapchainImages(XrSwapchain swapchain, uint32_t capacity, uint32_t* count,
                                                          XrSwapchainImageBaseHeader* images) {
    Swapchain* sc = fromHandle<Swapchain>(swapchain);
    if (!swapchains().valid(sc)) return XR_ERROR_HANDLE_INVALID;
    if (!count) return XR_ERROR_VALIDATION_FAILURE;
    *count = static_cast<uint32_t>(sc->textures.size());
    if (capacity == 0) return XR_SUCCESS;
    if (capacity < sc->textures.size()) return XR_ERROR_SIZE_INSUFFICIENT;
#ifdef __ANDROID__
    auto* gl = reinterpret_cast<XrSwapchainImageOpenGLESKHR*>(images);
    for (size_t i = 0; i < sc->textures.size(); ++i) {
        if (gl[i].type != XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR) return XR_ERROR_VALIDATION_FAILURE;
        gl[i].image = sc->textures[i];
    }
#else
    (void)images;
#endif
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrAcquireSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageAcquireInfo*,
                                                       uint32_t* index) {
    Swapchain* sc = fromHandle<Swapchain>(swapchain);
    if (!swapchains().valid(sc)) return XR_ERROR_HANDLE_INVALID;
    if (!index) return XR_ERROR_VALIDATION_FAILURE;
    if (sc->acquired.size() >= sc->textures.size()) return XR_ERROR_CALL_ORDER_INVALID;
    *index = sc->next;
    sc->acquired.push_back(sc->next);
    sc->next = (sc->next + 1) % static_cast<uint32_t>(sc->textures.size());
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrWaitSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageWaitInfo*) {
    Swapchain* sc = fromHandle<Swapchain>(swapchain);
    if (!swapchains().valid(sc)) return XR_ERROR_HANDLE_INVALID;
    if (sc->acquired.empty()) return XR_ERROR_CALL_ORDER_INVALID;
    // The compositor waits on a fence before sampling, so images are always ready to render into.
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrReleaseSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageReleaseInfo*) {
    Swapchain* sc = fromHandle<Swapchain>(swapchain);
    if (!swapchains().valid(sc)) return XR_ERROR_HANDLE_INVALID;
    if (sc->acquired.empty()) return XR_ERROR_CALL_ORDER_INVALID;
    sc->lastReleased = static_cast<int32_t>(sc->acquired.front());
    sc->acquired.pop_front();
    return XR_SUCCESS;
}

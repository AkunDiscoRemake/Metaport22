// Cardboard compositor: draws the app's eye images to the phone screen through the lens
// distortion mesh, with rotation-only reprojection to the latest head pose.
//
// The runtime lives inside the VR app's process, which has no window of ours. Like Monado's
// Android runtime, we load a small helper class from the Metaport APK
// (org.metaport.runtime.CompositorSurface) through createPackageContext(); it adds a
// full-screen SurfaceView to the app's activity and reports Choreographer vsync times.
#include <android/log.h>
#include <android/native_window_jni.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>

#include <cmath>
#include <cstddef>
#include <vector>

#include "../runtime.hpp"
#include "metaport/cardboard_lens.hpp"

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "MetaportXR", __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "MetaportXR", __VA_ARGS__)

namespace mpxr {

namespace {

constexpr const char* kMetaportPackage = "org.metaport";
constexpr const char* kHelperClass = "org.metaport.runtime.CompositorSurface";

const char* kVertexShader = R"(#version 300 es
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aTan;
uniform mat3 uWarp;   // render-eye-from-current-eye rotation
uniform vec4 uFov;    // tan of the app's fov: left, right, down, up (left/down negative)
uniform vec4 uRect;   // sub-image offset and scale in texture coordinates
out vec2 vUv;
void main() {
    vec3 d = uWarp * vec3(aTan, -1.0);
    vec2 t = d.xy / -d.z;
    vec2 uv = (t - uFov.xz) / (uFov.yw - uFov.xz);
    vUv = uRect.xy + uv * uRect.zw;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

const char* kFragment2D = R"(#version 300 es
precision mediump float;
in vec2 vUv;
uniform sampler2D uTex;
uniform vec4 uRect;
uniform float uEncode;  // 1 = sampled values are linear (sRGB texture): re-encode for the screen
out vec4 outColor;
void main() {
    vec2 lo = uRect.xy, hi = uRect.xy + uRect.zw;
    if (any(lessThan(vUv, lo)) || any(greaterThan(vUv, hi))) { outColor = vec4(0, 0, 0, 1); return; }
    vec3 c = texture(uTex, vUv).rgb;
    outColor = vec4(mix(c, pow(c, vec3(1.0 / 2.2)), uEncode), 1.0);
}
)";

const char* kFragmentArray = R"(#version 300 es
precision mediump float;
precision mediump sampler2DArray;
in vec2 vUv;
uniform sampler2DArray uTex;
uniform float uLayer;
uniform vec4 uRect;
uniform float uEncode;
out vec4 outColor;
void main() {
    vec2 lo = uRect.xy, hi = uRect.xy + uRect.zw;
    if (any(lessThan(vUv, lo)) || any(greaterThan(vUv, hi))) { outColor = vec4(0, 0, 0, 1); return; }
    vec3 c = texture(uTex, vec3(vUv, uLayer)).rgb;
    outColor = vec4(mix(c, pow(c, vec3(1.0 / 2.2)), uEncode), 1.0);
}
)";

GLuint compile(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        LOGE("shader: %s", log);
    }
    return s;
}

struct Program {
    GLuint id = 0;
    GLint warp, fov, rect, encode, layer, tex;

    void build(const char* fs) {
        id = glCreateProgram();
        GLuint v = compile(GL_VERTEX_SHADER, kVertexShader), f = compile(GL_FRAGMENT_SHADER, fs);
        glAttachShader(id, v);
        glAttachShader(id, f);
        glLinkProgram(id);
        glDeleteShader(v);
        glDeleteShader(f);
        warp = glGetUniformLocation(id, "uWarp");
        fov = glGetUniformLocation(id, "uFov");
        rect = glGetUniformLocation(id, "uRect");
        encode = glGetUniformLocation(id, "uEncode");
        layer = glGetUniformLocation(id, "uLayer");
        tex = glGetUniformLocation(id, "uTex");
    }
};

class CardboardCompositor : public Compositor {
public:
    CardboardCompositor(JavaVM* vm, EGLDisplay display, const metaport::DisplayInfo& info)
        : vm_(vm), display_(display), info_(info) {}

    ~CardboardCompositor() override {
        if (surface_ != EGL_NO_SURFACE) eglDestroySurface(display_, surface_);
        if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
        if (window_) ANativeWindow_release(window_);
        JNIEnv* env = env_();
        if (env && helper_) {
            jmethodID detach = env->GetMethodID(helperClass_, "detach", "()V");
            env->CallVoidMethod(helper_, detach);
            env->DeleteGlobalRef(helper_);
            env->DeleteGlobalRef(helperClass_);
        }
    }

    bool init(jobject activity, EGLContext appContext) {
        JNIEnv* env = env_();
        if (!env || !activity || !loadHelper(env, activity)) return false;

        jmethodID wait = env->GetMethodID(helperClass_, "waitForSurface", "(J)Landroid/view/Surface;");
        jobject surface = env->CallObjectMethod(helper_, wait, static_cast<jlong>(3000));
        if (env->ExceptionCheck() || !surface) {
            env->ExceptionClear();
            LOGE("compositor surface not available");
            return false;
        }
        window_ = ANativeWindow_fromSurface(env, surface);
        vsyncField_ = env->GetMethodID(helperClass_, "lastVsyncNanos", "()J");
        periodField_ = env->GetMethodID(helperClass_, "framePeriodNanos", "()J");

        const EGLint cfgAttribs[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR, EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
                                     EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_NONE};
        EGLConfig cfg;
        EGLint n = 0;
        if (!eglChooseConfig(display_, cfgAttribs, &cfg, 1, &n) || n == 0) return false;
        const EGLint ctxAttribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
        context_ = eglCreateContext(display_, cfg, appContext, ctxAttribs);  // shares the app's textures
        surface_ = eglCreateWindowSurface(display_, cfg, window_, nullptr);
        if (context_ == EGL_NO_CONTEXT || surface_ == EGL_NO_SURFACE) return false;

        Saved saved = save();
        eglMakeCurrent(display_, surface_, surface_, context_);
        eglSwapInterval(display_, 1);
        program2D_.build(kFragment2D);
        programArray_.build(kFragmentArray);
        buildMeshes();
        restore(saved);
        LOGI("Cardboard compositor ready (%dx%d)", ANativeWindow_getWidth(window_), ANativeWindow_getHeight(window_));
        return true;
    }

    void submit(const EyeSubmission eyes[2], XrTime, const Pose& latestHead) override {
        // Everything the app rendered so far must finish before we sample it.
        GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        glFlush();
        Saved saved = save();
        eglMakeCurrent(display_, surface_, surface_, context_);
        glWaitSync(fence, 0, GL_TIMEOUT_IGNORED);
        glDeleteSync(fence);

        glViewport(0, 0, ANativeWindow_getWidth(window_), ANativeWindow_getHeight(window_));
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);

        for (int e = 0; e < 2; ++e) {
            const EyeSubmission& eye = eyes[e];
            if (!eye.texture) continue;
            Program& p = eye.isArray ? programArray_ : program2D_;
            glUseProgram(p.id);

            // Rotation from the current eye frame to the frame the app rendered with.
            Quat rendered = fromXr(eye.pose).orientation;
            Quat warp = (rendered.inverse() * latestHead.orientation).normalized();
            Vec3 cx = warp.rotate({1, 0, 0}), cy = warp.rotate({0, 1, 0}), cz = warp.rotate({0, 0, 1});
            const float columns[9] = {cx.x, cx.y, cx.z, cy.x, cy.y, cy.z, cz.x, cz.y, cz.z};
            glUniformMatrix3fv(p.warp, 1, GL_FALSE, columns);
            glUniform4f(p.fov, std::tan(eye.fov.angleLeft), std::tan(eye.fov.angleRight), std::tan(eye.fov.angleDown),
                        std::tan(eye.fov.angleUp));
            float tw = static_cast<float>(eye.texWidth), th = static_cast<float>(eye.texHeight);
            glUniform4f(p.rect, eye.rect.offset.x / tw, eye.rect.offset.y / th, eye.rect.extent.width / tw,
                        eye.rect.extent.height / th);
            glUniform1f(p.encode, eye.srgb ? 1.0f : 0.0f);
            if (eye.isArray) glUniform1f(p.layer, static_cast<float>(eye.arrayIndex));
            glUniform1i(p.tex, 0);
            glActiveTexture(GL_TEXTURE0);
            GLenum target = eye.isArray ? GL_TEXTURE_2D_ARRAY : GL_TEXTURE_2D;
            glBindTexture(target, eye.texture);

            glBindBuffer(GL_ARRAY_BUFFER, vbo_[e]);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_[e]);
            glEnableVertexAttribArray(0);
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(metaport::MeshVertex),
                                  reinterpret_cast<void*>(offsetof(metaport::MeshVertex, x)));
            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(metaport::MeshVertex),
                                  reinterpret_cast<void*>(offsetof(metaport::MeshVertex, tx)));
            glDrawElements(GL_TRIANGLES, indexCount_[e], GL_UNSIGNED_SHORT, nullptr);
            glBindTexture(target, 0);
        }
        eglSwapBuffers(display_, surface_);
        restore(saved);
    }

    int64_t framePeriodNs() const override {
        JNIEnv* env = env_();
        jlong p = env ? env->CallLongMethod(helper_, periodField_) : 0;
        return p > 0 ? p : static_cast<int64_t>(1e9 / info_.refreshHz);
    }

    int64_t lastVsyncNs() const override {
        JNIEnv* env = env_();
        return env ? env->CallLongMethod(helper_, vsyncField_) : 0;
    }

private:
    struct Saved {
        EGLSurface draw, read;
        EGLContext ctx;
    };
    Saved save() const { return {eglGetCurrentSurface(EGL_DRAW), eglGetCurrentSurface(EGL_READ), eglGetCurrentContext()}; }
    void restore(const Saved& s) const { eglMakeCurrent(display_, s.draw, s.read, s.ctx); }

    JNIEnv* env_() const {
        JNIEnv* env = nullptr;
        if (vm_->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) == JNI_OK) return env;
        // The app's render thread stays attached for its lifetime.
        return vm_->AttachCurrentThread(&env, nullptr) == JNI_OK ? env : nullptr;
    }

    bool loadHelper(JNIEnv* env, jobject activity) {
        jclass ctxCls = env->FindClass("android/content/Context");
        jmethodID createPkg = env->GetMethodID(ctxCls, "createPackageContext", "(Ljava/lang/String;I)Landroid/content/Context;");
        const jint CONTEXT_INCLUDE_CODE = 1, CONTEXT_IGNORE_SECURITY = 2;
        jobject pkgCtx = env->CallObjectMethod(activity, createPkg, env->NewStringUTF(kMetaportPackage),
                                               CONTEXT_INCLUDE_CODE | CONTEXT_IGNORE_SECURITY);
        if (env->ExceptionCheck() || !pkgCtx) { env->ExceptionClear(); return false; }
        jmethodID getCl = env->GetMethodID(ctxCls, "getClassLoader", "()Ljava/lang/ClassLoader;");
        jobject loader = env->CallObjectMethod(pkgCtx, getCl);
        jclass clCls = env->FindClass("java/lang/ClassLoader");
        jmethodID loadClass = env->GetMethodID(clCls, "loadClass", "(Ljava/lang/String;)Ljava/lang/Class;");
        auto cls = static_cast<jclass>(env->CallObjectMethod(loader, loadClass, env->NewStringUTF(kHelperClass)));
        if (env->ExceptionCheck() || !cls) { env->ExceptionClear(); return false; }
        jmethodID attach = env->GetStaticMethodID(cls, "attach", "(Landroid/app/Activity;)Lorg/metaport/runtime/CompositorSurface;");
        jobject helper = env->CallStaticObjectMethod(cls, attach, activity);
        if (env->ExceptionCheck() || !helper) { env->ExceptionClear(); return false; }
        helperClass_ = static_cast<jclass>(env->NewGlobalRef(cls));
        helper_ = env->NewGlobalRef(helper);
        return true;
    }

    void buildMeshes() {
        metaport::CardboardLens lens(info_.viewer, info_.screen);
        glGenBuffers(2, vbo_);
        glGenBuffers(2, ibo_);
        for (int e = 0; e < 2; ++e) {
            std::vector<metaport::MeshVertex> v;
            std::vector<unsigned short> idx;
            lens.buildMesh(e, 48, 48, &v, &idx);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_[e]);
            glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(v[0]), v.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_[e]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(idx[0]), idx.data(), GL_STATIC_DRAW);
            indexCount_[e] = static_cast<GLsizei>(idx.size());
        }
    }

    JavaVM* vm_;
    EGLDisplay display_;
    metaport::DisplayInfo info_;
    jclass helperClass_ = nullptr;
    jobject helper_ = nullptr;
    jmethodID vsyncField_ = nullptr, periodField_ = nullptr;
    ANativeWindow* window_ = nullptr;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;
    Program program2D_, programArray_;
    GLuint vbo_[2] = {0, 0}, ibo_[2] = {0, 0};
    GLsizei indexCount_[2] = {0, 0};
};

}  // namespace

std::unique_ptr<Compositor> createCardboardCompositor(JavaVM* vm, jobject activity, EGLDisplay display,
                                                      EGLContext appContext, const metaport::DisplayInfo& info) {
    if (!vm) return nullptr;
    auto c = std::make_unique<CardboardCompositor>(vm, display, info);
    if (!c->init(activity, appContext)) return nullptr;
    return c;
}

}  // namespace mpxr

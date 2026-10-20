// Access to the shared tracking block published by the Metaport service.
//
// Android: the service exposes the block through its ContentProvider
// (content://org.metaport.tracking/block); `call("open")` returns a Bundle with an
// android.os.SharedMemory, which is mapped here with ASharedMemory_dupFromJava.
// Desktop: $METAPORT_SHM_PATH names a file holding the block (used by tests and the
// replay tool).
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdlib>

#ifdef __ANDROID__
#include <android/sharedmem_jni.h>
#endif

#include "runtime.hpp"

namespace forkxr {

namespace {

class SharedMemorySource : public TrackingSource {
public:
    SharedMemorySource(int fd, size_t size) : fd_(fd), size_(size) {
        void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        block_ = p == MAP_FAILED ? nullptr : static_cast<metaport::SharedBlock*>(p);
    }
    ~SharedMemorySource() override {
        if (block_) munmap(block_, size_);
        if (fd_ >= 0) close(fd_);
    }
    bool ok() const { return block_ != nullptr && block_->magic == metaport::kSharedMagic; }

    bool read(TrackingFrame* out) override { return metaport::readFrame(block_, out); }

    void requestHaptic(Side side, float amplitude, float frequencyHz, int64_t durationNs) override {
        metaport::HapticCommand& c = block_->haptics[side];
        c.amplitude = amplitude;
        c.frequencyHz = frequencyHz;
        c.durationNs = durationNs;
        c.serial.fetch_add(1, std::memory_order_release);  // the service polls for a new serial
    }

private:
    int fd_;
    size_t size_;
    metaport::SharedBlock* block_ = nullptr;
};

#ifdef __ANDROID__
int openFromProvider(JavaVM* vm, jobject context) {
    if (!vm || !context) return -1;
    JNIEnv* env = nullptr;
    bool attached = false;
    if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
        if (vm->AttachCurrentThread(&env, nullptr) != JNI_OK) return -1;
        attached = true;
    }
    int fd = -1;
    jclass ctxCls = env->GetObjectClass(context);
    jmethodID getResolver = env->GetMethodID(ctxCls, "getContentResolver", "()Landroid/content/ContentResolver;");
    jobject resolver = env->CallObjectMethod(context, getResolver);
    jclass uriCls = env->FindClass("android/net/Uri");
    jmethodID parse = env->GetStaticMethodID(uriCls, "parse", "(Ljava/lang/String;)Landroid/net/Uri;");
    jobject uri = env->CallStaticObjectMethod(uriCls, parse, env->NewStringUTF("content://org.metaport.tracking"));
    jclass resCls = env->GetObjectClass(resolver);
    jmethodID call = env->GetMethodID(resCls, "call",
                                      "(Landroid/net/Uri;Ljava/lang/String;Ljava/lang/String;Landroid/os/Bundle;)Landroid/os/Bundle;");
    jobject bundle = env->CallObjectMethod(resolver, call, uri, env->NewStringUTF("open"), nullptr, nullptr);
    if (env->ExceptionCheck()) {
        env->ExceptionClear();  // provider missing: Metaport app not installed or service stopped
        bundle = nullptr;
    }
    if (bundle) {
        jclass bCls = env->GetObjectClass(bundle);
        jmethodID getParcelable = env->GetMethodID(bCls, "getParcelable", "(Ljava/lang/String;)Landroid/os/Parcelable;");
        jobject shm = env->CallObjectMethod(bundle, getParcelable, env->NewStringUTF("block"));
        if (shm && !env->ExceptionCheck()) fd = ASharedMemory_dupFromJava(env, shm);
        if (env->ExceptionCheck()) env->ExceptionClear();
    }
    if (attached) vm->DetachCurrentThread();
    return fd;
}
#endif

}  // namespace

std::unique_ptr<TrackingSource> createTrackingSource(void* javaVm, void* context) {
    int fd = -1;
#ifdef __ANDROID__
    fd = openFromProvider(static_cast<JavaVM*>(javaVm), static_cast<jobject>(context));
#else
    (void)javaVm;
    (void)context;
    if (const char* path = std::getenv("METAPORT_SHM_PATH")) fd = open(path, O_RDWR);
#endif
    if (fd < 0) return nullptr;
    auto src = std::make_unique<SharedMemorySource>(fd, sizeof(metaport::SharedBlock));
    if (!src->ok()) return nullptr;
    return src;
}

// --- Null compositor: paces frames on a clock, displays nothing (headless sessions, tests) ----

namespace {

class NullCompositor : public Compositor {
public:
    explicit NullCompositor(float hz) : period_(static_cast<int64_t>(1e9 / (hz > 1 ? hz : 60))) {}
    void submit(const EyeSubmission*, XrTime, const Pose&) override {}
    int64_t framePeriodNs() const override { return period_; }
    int64_t lastVsyncNs() const override { return 0; }

private:
    int64_t period_;
};

}  // namespace

std::unique_ptr<Compositor> createNullCompositor(float refreshHz) { return std::make_unique<NullCompositor>(refreshHz); }

}  // namespace forkxr

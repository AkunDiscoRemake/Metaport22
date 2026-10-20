Test fixtures for `ElfTest`: tiny arm64 shared libraries built from Metaport's own sources with
the Android NDK (`aarch64-linux-android29-clang -shared -nostdlib`). No third-party code.

- `libbar.so`: exports `bar`.
- `libfoo.so`: needs `libbar.so` and `liblog.so`, imports `bar` and `__android_log_print`, exports `foo`.
- `libvrapp.so`: needs `libvrplatform.so` (deliberately absent), imports `vr_platform_init`.

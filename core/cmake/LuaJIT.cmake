# Builds LuaJIT (MIT, https://luajit.org) as a static library with its own Makefile, for the host
# or cross-compiled for Android with the NDK toolchain CMake is already using.
# Defines the imported target `luajit::luajit`.
include(ExternalProject)

set(FORKXR_LUAJIT_COMMIT dd3b24369ddfaa83c78cd6fdd91d157c8968ec8d CACHE STRING "LuaJIT commit")
set(_lj_src ${CMAKE_BINARY_DIR}/luajit-src)
set(_lj_lib ${_lj_src}/src/libluajit.a)

set(_lj_make_args -j8 BUILDMODE=static amalg)
if(ANDROID)
    # LuaJIT builds a host tool (buildvm) first, so it needs a host compiler too.
    get_filename_component(_ndk_bin ${CMAKE_C_COMPILER} DIRECTORY)
    set(_target_cc "${CMAKE_C_COMPILER} --target=${CMAKE_C_COMPILER_TARGET} -fPIC")
    list(APPEND _lj_make_args
        HOST_CC=cc
        CROSS=${_ndk_bin}/llvm-
        "STATIC_CC=${_target_cc}"
        "DYNAMIC_CC=${_target_cc}"
        "TARGET_LD=${_target_cc}"
        "TARGET_AR=${_ndk_bin}/llvm-ar rcus"
        TARGET_STRIP=${_ndk_bin}/llvm-strip
        TARGET_SYS=Linux)
else()
    list(APPEND _lj_make_args "XCFLAGS=-fPIC")
endif()

ExternalProject_Add(luajit_build
    GIT_REPOSITORY https://github.com/LuaJIT/LuaJIT.git
    GIT_TAG ${FORKXR_LUAJIT_COMMIT}
    SOURCE_DIR ${_lj_src}
    BUILD_IN_SOURCE TRUE
    CONFIGURE_COMMAND ""
    BUILD_COMMAND make ${_lj_make_args}
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${_lj_lib}
    UPDATE_DISCONNECTED TRUE)

file(MAKE_DIRECTORY ${_lj_src}/src)
add_library(luajit::luajit STATIC IMPORTED GLOBAL)
set_target_properties(luajit::luajit PROPERTIES
    IMPORTED_LOCATION ${_lj_lib}
    INTERFACE_INCLUDE_DIRECTORIES ${_lj_src}/src)
add_dependencies(luajit::luajit luajit_build)
if(NOT ANDROID)
    set_property(TARGET luajit::luajit APPEND PROPERTY INTERFACE_LINK_LIBRARIES m ${CMAKE_DL_LIBS})
else()
    set_property(TARGET luajit::luajit APPEND PROPERTY INTERFACE_LINK_LIBRARIES m dl)
endif()

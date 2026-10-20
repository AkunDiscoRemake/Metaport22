# Metaport OpenXR runtime (libmetaport_openxr.so).
include(FetchContent)

if(NOT METAPORT_OPENXR_INCLUDE_DIR)
    # Headers only: the Khronos loader itself is never built or shipped by Metaport.
    FetchContent_Declare(openxr_sdk
        GIT_REPOSITORY https://github.com/KhronosGroup/OpenXR-SDK.git
        GIT_TAG release-1.1.65
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR headers-only-no-build)
    FetchContent_MakeAvailable(openxr_sdk)
    set(METAPORT_OPENXR_INCLUDE_DIR ${openxr_sdk_SOURCE_DIR}/include)
endif()

set(RUNTIME_SOURCES
    src/instance.cpp
    src/session.cpp
    src/input.cpp
    src/swapchain.cpp
    src/hand_tracking.cpp
    src/tracking_source.cpp
)
if(ANDROID)
    list(APPEND RUNTIME_SOURCES src/android/cardboard_compositor.cpp)
endif()

add_library(metaport_openxr SHARED ${RUNTIME_SOURCES})
target_include_directories(metaport_openxr PRIVATE ${METAPORT_OPENXR_INCLUDE_DIR} src)
target_compile_definitions(metaport_openxr PRIVATE XR_EXTENSION_PROTOTYPES)
target_link_libraries(metaport_openxr PRIVATE metaport_tracking)
# Only xrNegotiateLoaderRuntimeInterface is exported. Everything else stays hidden so our xr*
# symbols never interpose with (or get interposed by) the app's own libopenxr_loader.so.
set_target_properties(metaport_openxr PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
if(NOT MSVC)
    target_compile_options(metaport_openxr PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
    target_link_options(metaport_openxr PRIVATE -Wl,-Bsymbolic -Wl,--no-undefined -Wl,--exclude-libs,ALL)
endif()
if(ANDROID)
    target_link_libraries(metaport_openxr PRIVATE android log EGL GLESv3)
endif()

if(METAPORT_BUILD_TESTS AND NOT ANDROID)
    add_executable(metaport_runtime_tests tests/runtime_tests.cpp)
    target_include_directories(metaport_runtime_tests PRIVATE ${METAPORT_OPENXR_INCLUDE_DIR})
    target_link_libraries(metaport_runtime_tests PRIVATE metaport_tracking ${CMAKE_DL_LIBS})
    target_compile_definitions(metaport_runtime_tests PRIVATE
        METAPORT_RUNTIME_PATH="$<TARGET_FILE:metaport_openxr>")
    add_dependencies(metaport_runtime_tests metaport_openxr)
    add_test(NAME metaport_runtime_tests COMMAND metaport_runtime_tests)
endif()

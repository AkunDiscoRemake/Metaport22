cmake_minimum_required(VERSION 3.18)
project(metaport_core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(METAPORT_BUILD_TESTS "Build host unit tests" ON)
option(METAPORT_BUILD_RUNTIME "Build the OpenXR runtime library" ON)

add_library(metaport_tracking STATIC
    src/head_tracker.cpp
    src/hand.cpp
    src/joycon.cpp
    src/controller_fusion.cpp
    src/tracking_engine.cpp
    src/cardboard_lens.cpp
)
target_include_directories(metaport_tracking PUBLIC include)
if(NOT MSVC)
    target_compile_options(metaport_tracking PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

if(METAPORT_BUILD_RUNTIME)
    add_subdirectory(runtime)
endif()

if(METAPORT_BUILD_TESTS AND NOT ANDROID)
    enable_testing()
    add_executable(metaport_tests
        tests/test_main.cpp
        tests/test_math.cpp
        tests/test_shared_state.cpp
        tests/test_hand.cpp
        tests/test_joycon.cpp
        tests/test_controller_fusion.cpp
        tests/test_head_tracker.cpp
        tests/test_cardboard_lens.cpp
    )
    target_link_libraries(metaport_tests PRIVATE metaport_tracking)
    add_test(NAME metaport_tests COMMAND metaport_tests)
endif()

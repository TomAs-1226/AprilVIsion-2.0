cmake_minimum_required(VERSION 3.16)
project(frc_vision VERSION 2.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# Build Options
# ============================================================================
option(BUILD_ORANGEPI "Build for Orange Pi 5 (RK3588)" OFF)
option(BUILD_MAC_SIM "Build Mac Simulator" OFF)

# Auto-detect platform if not specified
if(NOT BUILD_ORANGEPI AND NOT BUILD_MAC_SIM)
    if(APPLE)
        set(BUILD_MAC_SIM ON)
        message(STATUS "Auto-detected macOS - building Mac Simulator")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(BUILD_ORANGEPI ON)
        message(STATUS "Auto-detected ARM64 - building for Orange Pi")
    else()
        # Default to Orange Pi for Linux x86 (cross-compile scenario)
        set(BUILD_ORANGEPI ON)
        message(STATUS "Defaulting to Orange Pi build")
    endif()
endif()

# Build type defaults to Release for performance
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ============================================================================
# Platform-specific compiler flags
# ============================================================================
if(APPLE)
    # macOS with Apple Clang
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pthread")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

    # Find Homebrew prefix for dependencies
    execute_process(
        COMMAND brew --prefix
        OUTPUT_VARIABLE HOMEBREW_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(HOMEBREW_PREFIX)
        list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}")
        include_directories("${HOMEBREW_PREFIX}/include")
        link_directories("${HOMEBREW_PREFIX}/lib")
    endif()

    message(STATUS "Building for macOS with Apple Clang")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    # ARM64 (Orange Pi 5 / RK3588S)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -pthread")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=armv8-a+crc+crypto -mtune=cortex-a76 -DNDEBUG")
    message(STATUS "Building for ARM64 with Cortex-A76 optimizations (RK3588S)")
else()
    # x86_64 Linux
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -pthread")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
endif()

# ============================================================================
# Find required packages
# ============================================================================
find_package(PkgConfig REQUIRED)

# OpenCV - with highgui for Mac simulator
if(BUILD_MAC_SIM)
    find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio imgcodecs highgui calib3d)
else()
    find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio imgcodecs calib3d)
endif()

find_package(Threads REQUIRED)

# yaml-cpp
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# ============================================================================
# AprilTag library - FetchContent for deterministic builds
# ============================================================================
include(FetchContent)

FetchContent_Declare(
    apriltag
    GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
    GIT_TAG v3.4.2
    GIT_SHALLOW TRUE
)

FetchContent_GetProperties(apriltag)
if(NOT apriltag_POPULATED)
    FetchContent_Populate(apriltag)

    # Build apriltag as static library
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory(${apriltag_SOURCE_DIR} ${apriltag_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# ============================================================================
# WPILib ntcore - FetchContent for NetworkTables 4
# ============================================================================
FetchContent_Declare(
    allwpilib
    GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
    GIT_TAG v2025.1.1
    GIT_SHALLOW TRUE
)

FetchContent_GetProperties(allwpilib)
if(NOT allwpilib_POPULATED)
    FetchContent_Populate(allwpilib)

    # Only build ntcore and its dependencies
    set(WITH_JAVA OFF CACHE BOOL "" FORCE)
    set(WITH_CSCORE OFF CACHE BOOL "" FORCE)
    set(WITH_WPIMATH OFF CACHE BOOL "" FORCE)
    set(WITH_WPILIB OFF CACHE BOOL "" FORCE)
    set(WITH_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(WITH_TESTS OFF CACHE BOOL "" FORCE)
    set(WITH_GUI OFF CACHE BOOL "" FORCE)
    set(WITH_SIMULATION_MODULES OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

    add_subdirectory(${allwpilib_SOURCE_DIR} ${allwpilib_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# ============================================================================
# cpp-httplib - Header-only, vendored
# ============================================================================
FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.15.3
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(httplib)

# ============================================================================
# nlohmann/json - Header-only for JSON parsing
# ============================================================================
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(json)

# ============================================================================
# Core library (shared between Orange Pi and Mac Sim)
# ============================================================================
set(CORE_SOURCES
    src/config.cpp
    src/camera.cpp
    src/detector.cpp
    src/tracker.cpp
    src/pose.cpp
    src/fusion.cpp
    src/nt_publisher.cpp
    src/web_server.cpp
    src/field_layout.cpp
)

set(CORE_HEADERS
    src/types.hpp
    src/ring_buffer.hpp
    src/config.hpp
    src/camera.hpp
    src/detector.hpp
    src/tracker.hpp
    src/pose.hpp
    src/fusion.hpp
    src/nt_publisher.hpp
    src/web_server.hpp
    src/field_layout.hpp
    src/platform/frame_source.hpp
)

# Create core library
add_library(frc_vision_core STATIC ${CORE_SOURCES})

target_include_directories(frc_vision_core PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${apriltag_SOURCE_DIR}
    ${httplib_SOURCE_DIR}
)

target_link_libraries(frc_vision_core PUBLIC
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    apriltag
    ntcore
    wpiutil
    wpinet
    nlohmann_json::nlohmann_json
    Threads::Threads
)

# Link atomic on non-Apple platforms
if(NOT APPLE)
    target_link_libraries(frc_vision_core PUBLIC atomic)
endif()

# ============================================================================
# Orange Pi Build (Production)
# ============================================================================
if(BUILD_ORANGEPI)
    add_executable(frc_vision src/main.cpp)

    target_link_libraries(frc_vision PRIVATE frc_vision_core)

    # Copy web assets and config to build directory
    add_custom_command(TARGET frc_vision POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/web $<TARGET_FILE_DIR:frc_vision>/web
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/config $<TARGET_FILE_DIR:frc_vision>/config
        COMMENT "Copying web assets and config files..."
    )

    # Install targets
    install(TARGETS frc_vision RUNTIME DESTINATION bin)
    install(DIRECTORY web/ DESTINATION share/frc_vision/web)
    install(DIRECTORY config/ DESTINATION share/frc_vision/config)

    if(EXISTS ${CMAKE_SOURCE_DIR}/deploy/frc_vision.service)
        install(FILES deploy/frc_vision.service DESTINATION lib/systemd/system)
    endif()

    message(STATUS "Orange Pi build: frc_vision executable")
endif()

# ============================================================================
# Mac Simulator Build
# ============================================================================
if(BUILD_MAC_SIM)
    set(SIM_SOURCES
        src/sim/sim_main.cpp
        src/sim/simulator.cpp
        src/sim/robot_dynamics.cpp
        src/sim/field_renderer.cpp
        src/sim/auto_align.cpp
    )

    set(SIM_HEADERS
        src/sim/sim_types.hpp
        src/sim/simulator.hpp
        src/sim/robot_dynamics.hpp
        src/sim/field_renderer.hpp
        src/sim/auto_align.hpp
    )

    add_executable(frc_vision_sim ${SIM_SOURCES})

    target_include_directories(frc_vision_sim PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/sim
    )

    target_link_libraries(frc_vision_sim PRIVATE
        frc_vision_core
        ${OpenCV_LIBS}
    )

    # macOS-specific framework for webcam access
    if(APPLE)
        target_link_libraries(frc_vision_sim PRIVATE
            "-framework CoreFoundation"
            "-framework CoreMedia"
            "-framework CoreVideo"
            "-framework AVFoundation"
        )

        # Define macro for macOS-specific code
        target_compile_definitions(frc_vision_sim PRIVATE __APPLE__)
    endif()

    # Copy assets and config to build directory
    add_custom_command(TARGET frc_vision_sim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:frc_vision_sim>/assets
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/config $<TARGET_FILE_DIR:frc_vision_sim>/config
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_SOURCE_DIR}/web $<TARGET_FILE_DIR:frc_vision_sim>/web
        COMMENT "Copying simulator assets, config, and web files..."
    )

    message(STATUS "Mac Simulator build: frc_vision_sim executable")
endif()

# ============================================================================
# Build Summary
# ============================================================================
message(STATUS "========================================")
message(STATUS "FRC Vision Coprocessor v${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Orange Pi build: ${BUILD_ORANGEPI}")
message(STATUS "Mac Simulator build: ${BUILD_MAC_SIM}")
message(STATUS "========================================")

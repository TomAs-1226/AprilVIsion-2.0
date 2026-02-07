cmake_minimum_required(VERSION 3.16)
project(frc_vision VERSION 2.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build type defaults to Release for performance
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Optimization flags for ARM64
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=armv8-a+crc+crypto -mtune=cortex-a76 -DNDEBUG")
    message(STATUS "Building for ARM64 with Cortex-A76 optimizations (RK3588S)")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
endif()

# Common flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -pthread")

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio imgcodecs)
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
    GIT_TAG v2026.2.1
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
# Main executable
# ============================================================================
set(SOURCES
    src/main.cpp
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

add_executable(frc_vision ${SOURCES})

target_include_directories(frc_vision PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${apriltag_SOURCE_DIR}
    ${httplib_SOURCE_DIR}
)

target_link_libraries(frc_vision PRIVATE
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    apriltag
    ntcore
    wpiutil
    wpinet
    nlohmann_json::nlohmann_json
    Threads::Threads
    atomic
)

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
install(FILES deploy/frc_vision.service DESTINATION lib/systemd/system)

message(STATUS "FRC Vision Coprocessor v${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

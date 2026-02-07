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

# Option to use prebuilt WPILib (much faster!)
option(USE_PREBUILT_WPILIB "Use prebuilt WPILib libraries instead of building from source" ON)

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
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio imgcodecs calib3d)
find_package(Threads REQUIRED)

# yaml-cpp
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# ============================================================================
# AprilTag library
# ============================================================================
include(FetchContent)

set(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
    apriltag
    GIT_REPOSITORY "https://github.com/AprilRobotics/apriltag.git"
    GIT_TAG        "v3.4.2"
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(apriltag)

# ============================================================================
# WPILib ntcore
# ============================================================================
if(USE_PREBUILT_WPILIB)
    # Use prebuilt libraries (fast!)
    set(WPILIB_DIR "${CMAKE_SOURCE_DIR}/third_party/wpilib")

    if(NOT EXISTS "${WPILIB_DIR}/lib")
        message(STATUS "Prebuilt WPILib not found, downloading...")
        execute_process(
            COMMAND bash "${CMAKE_SOURCE_DIR}/scripts/download_wpilib.sh"
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            RESULT_VARIABLE DOWNLOAD_RESULT
        )
        if(NOT DOWNLOAD_RESULT EQUAL 0)
            message(FATAL_ERROR "Failed to download WPILib. Run: ./scripts/download_wpilib.sh")
        endif()
    endif()

    # Find the libraries
    find_library(NTCORE_LIB NAMES ntcore libntcore PATHS "${WPILIB_DIR}/lib" NO_DEFAULT_PATH)
    find_library(WPIUTIL_LIB NAMES wpiutil libwpiutil PATHS "${WPILIB_DIR}/lib" NO_DEFAULT_PATH)
    find_library(WPINET_LIB NAMES wpinet libwpinet PATHS "${WPILIB_DIR}/lib" NO_DEFAULT_PATH)

    if(NOT NTCORE_LIB OR NOT WPIUTIL_LIB OR NOT WPINET_LIB)
        message(FATAL_ERROR "WPILib libraries not found in ${WPILIB_DIR}/lib. Run: ./scripts/download_wpilib.sh")
    endif()

    set(WPILIB_INCLUDE_DIRS "${WPILIB_DIR}/include")
    set(WPILIB_LIBRARIES ${NTCORE_LIB} ${WPINET_LIB} ${WPIUTIL_LIB})

    message(STATUS "Using prebuilt WPILib from ${WPILIB_DIR}")
else()
    # Build from source (slow but guaranteed to work)
    message(STATUS "Building WPILib from source (this takes 15-30 minutes on ARM)...")

    FetchContent_Declare(
        allwpilib
        GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib.git
        GIT_TAG v2026.2.1
        GIT_SHALLOW TRUE
    )

    FetchContent_GetProperties(allwpilib)
    if(NOT allwpilib_POPULATED)
        FetchContent_Populate(allwpilib)

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

    set(WPILIB_INCLUDE_DIRS
        "${allwpilib_SOURCE_DIR}/ntcore/src/main/native/include"
        "${allwpilib_SOURCE_DIR}/ntcore/src/generated/main/native/include"
        "${allwpilib_SOURCE_DIR}/wpiutil/src/main/native/include"
        "${allwpilib_SOURCE_DIR}/wpiutil/src/main/native/thirdparty/fmtlib/include"
        "${allwpilib_SOURCE_DIR}/wpiutil/src/main/native/thirdparty/llvm/include"
        "${allwpilib_SOURCE_DIR}/wpiutil/src/main/native/thirdparty/json/include"
        "${allwpilib_SOURCE_DIR}/wpiutil/src/main/native/thirdparty/expected/include"
        "${allwpilib_SOURCE_DIR}/wpiutil/src/main/native/thirdparty/memory/include"
        "${allwpilib_SOURCE_DIR}/wpinet/src/main/native/include"
        "${allwpilib_SOURCE_DIR}/wpinet/src/main/native/thirdparty/libuv/include"
    )
    set(WPILIB_LIBRARIES ntcore wpinet wpiutil)
endif()

# ============================================================================
# cpp-httplib - Header-only
# ============================================================================
FetchContent_Declare(
    httplib
    GIT_REPOSITORY "https://github.com/yhirose/cpp-httplib.git"
    GIT_TAG        "v0.15.3"
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(httplib)

# ============================================================================
# nlohmann/json - Header-only
# ============================================================================
FetchContent_Declare(
    json
    GIT_REPOSITORY "https://github.com/nlohmann/json.git"
    GIT_TAG        "v3.11.3"
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(json)

# ============================================================================
# Main executable
# ============================================================================
set(SOURCES
    src/main.cpp
    src/config.cpp
    src/camera.cpp
    src/calibration.cpp
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
    ${WPILIB_INCLUDE_DIRS}
)

target_link_libraries(frc_vision PRIVATE
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    apriltag
    ${WPILIB_LIBRARIES}
    nlohmann_json::nlohmann_json
    Threads::Threads
    atomic
    dl
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

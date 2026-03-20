cmake_minimum_required(VERSION 3.20)
project(baddy_vision CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(WITH_SPINNAKER "Build with Spinnaker SDK for camera support" OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS
    core
    imgproc
    calib3d
    highgui
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CORE_SOURCES
    src/core/config.cpp
    src/core/birdie_dynamics.cpp
    src/core/ekf.cpp
    src/core/landing_predictor.cpp
    src/core/detector.cpp
    src/core/stereo.cpp
    src/core/tracker.cpp
    src/core/timing.cpp
)

add_library(baddy_core STATIC ${CORE_SOURCES})
target_include_directories(baddy_core PUBLIC src)
target_link_libraries(baddy_core PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
)

# replay_trajectory — works without cameras
add_executable(replay_trajectory src/replay_trajectory.cpp)
target_link_libraries(replay_trajectory PRIVATE baddy_core)

if(WITH_SPINNAKER)
    # Spinnaker SDK does not ship a CMake find module.
    # Set SPINNAKER_DIR to the SDK install prefix if it is not in a standard location.
    set(SPINNAKER_DIR "/opt/spinnaker" CACHE PATH "Spinnaker SDK install prefix")

    find_library(SPINNAKER_LIB Spinnaker HINTS ${SPINNAKER_DIR}/lib)
    find_path(SPINNAKER_INCLUDE Spinnaker.h HINTS ${SPINNAKER_DIR}/include/spinnaker ${SPINNAKER_DIR}/include)

    if(NOT SPINNAKER_LIB OR NOT SPINNAKER_INCLUDE)
        message(FATAL_ERROR "Spinnaker SDK not found. Set SPINNAKER_DIR or disable WITH_SPINNAKER.")
    endif()

    add_library(baddy_camera STATIC src/core/camera.cpp)
    target_include_directories(baddy_camera PUBLIC src ${SPINNAKER_INCLUDE})
    target_link_libraries(baddy_camera PUBLIC baddy_core ${SPINNAKER_LIB})

    add_executable(capture_trajectories src/capture_trajectories.cpp)
    target_link_libraries(capture_trajectories PRIVATE baddy_camera)

    add_executable(realtime src/realtime.cpp)
    target_link_libraries(realtime PRIVATE baddy_camera)

    add_executable(preview src/preview.cpp)
    target_include_directories(preview PRIVATE ${SPINNAKER_INCLUDE})
    target_link_libraries(preview PRIVATE baddy_core ${SPINNAKER_LIB})

    add_executable(dump_nodes src/dump_nodes.cpp)
    target_include_directories(dump_nodes PRIVATE ${SPINNAKER_INCLUDE})
    target_link_libraries(dump_nodes PRIVATE ${SPINNAKER_LIB})
endif()
cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM3)
option(ORB_SLAM3_WITH_CUDA_FAST "Build optional exact FAST-9 CUDA backend" OFF)
set(ORB_SLAM3_CUDA_ARCH "87" CACHE STRING "CUDA architecture for the optional FAST backend")


option(ORB_SLAM3_WITH_PX4 "Build optional ROS 2 / PX4 1.17 VIO bridge" OFF)
option(ORB_SLAM3_NATIVE_ARCH "Use the build machine CPU in optimized configurations" ON)
option(ORB_SLAM3_BUILD_DATASET_EXAMPLES "Build dataset examples" ON)
option(ORB_SLAM3_BUILD_CAMERA_EXAMPLES "Build RealSense capture examples when the SDK is available" ON)
option(ORB_SLAM3_BUILD_LEGACY_EXAMPLES "Build Examples_old programs" OFF)
option(ORB_SLAM3_REQUIRE_REALSENSE "Fail configuration if the RealSense SDK is missing" OFF)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

include(cmake/CompilerOptions.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Keep the legacy macro: examples use it to select std::chrono::steady_clock.
add_compile_definitions(COMPILEDWITHC11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Boost CONFIG QUIET COMPONENTS serialization)
if(NOT Boost_FOUND)
    find_package(Boost REQUIRED COMPONENTS serialization)
endif()
find_package(OpenSSL REQUIRED COMPONENTS Crypto)
find_package(Threads REQUIRED)
find_package(Pangolin REQUIRED)
if(ORB_SLAM3_REQUIRE_REALSENSE)
    find_package(realsense2 REQUIRED)
else()
    find_package(realsense2 QUIET)
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/InputRecording.cc
src/RollingInputRecording.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
Eigen3::Eigen
${Pangolin_LIBRARIES}
DBoW2
g2o
Boost::serialization
OpenSSL::Crypto
Threads::Threads
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Pangolin installations without their own RUNPATH need transitive lookup.
    target_link_options(${PROJECT_NAME} PRIVATE "LINKER:--disable-new-dtags")
endif()

# The core uses no RealSense API. Only capture executables link its SDK.
function(orb_link_realsense target)
    if(TARGET realsense2::realsense2)
        target_link_libraries(${target} realsense2::realsense2)
    else()
        target_include_directories(${target} PRIVATE ${realsense2_INCLUDE_DIR} ${realsense_INCLUDE_DIR})
        target_link_libraries(${target} ${realsense2_LIBRARY})
    endif()
endfunction()


if(ORB_SLAM3_WITH_CUDA_FAST)
    include(CheckLanguage)
    check_language(CUDA)
    if(NOT CMAKE_CUDA_COMPILER)
        message(FATAL_ERROR "ORB_SLAM3_WITH_CUDA_FAST requires a CUDA compiler")
    endif()
    enable_language(CUDA)
    add_library(orbslam3_cuda_fast STATIC src/CudaFast.cu)
    set_target_properties(orbslam3_cuda_fast PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON POSITION_INDEPENDENT_CODE ON CUDA_ARCHITECTURES "${ORB_SLAM3_CUDA_ARCH}")
    target_compile_options(orbslam3_cuda_fast PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-O3;-gencode=arch=compute_${ORB_SLAM3_CUDA_ARCH},code=sm_${ORB_SLAM3_CUDA_ARCH}>)
    target_link_libraries(orbslam3_cuda_fast ${OpenCV_LIBS})
    target_link_libraries(${PROJECT_NAME} orbslam3_cuda_fast)
    set_property(SOURCE src/ORBextractor.cc APPEND PROPERTY COMPILE_DEFINITIONS ORB_SLAM3_WITH_CUDA_FAST=1)
    if(ORB_SLAM3_BUILD_TESTS)
        add_executable(cuda_extractor_regression tests/cuda_extractor_regression.cc)
        target_link_libraries(cuda_extractor_regression ${PROJECT_NAME} orbslam3_cuda_fast)
        set_target_properties(cuda_extractor_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
        add_test(NAME cuda_extractor_regression COMMAND cuda_extractor_regression)
    endif()
endif()

# Build examples

option(ORB_SLAM3_BUILD_TESTS "Build atlas save regression tests" OFF)
if(ORB_SLAM3_BUILD_TESTS)
    enable_testing()
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
        add_test(NAME recovery_launcher_paths
            COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/launcher_path_regression.py)
        add_test(NAME vio_energy_regression
            COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/test_vio_energy.py)
    endif()
    add_executable(featureless_imu_regression tests/featureless_imu_regression.cc)
    target_link_libraries(featureless_imu_regression ${PROJECT_NAME})
    set_target_properties(featureless_imu_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME featureless_imu_regression COMMAND featureless_imu_regression)
    add_executable(decxin_recording_replay EXCLUDE_FROM_ALL tests/decxin_recording_replay.cc)
    target_link_libraries(decxin_recording_replay ${PROJECT_NAME})
    set_target_properties(decxin_recording_replay PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_executable(stereo_resize_geometry_regression tests/stereo_resize_geometry_regression.cc)
    target_link_libraries(stereo_resize_geometry_regression ${PROJECT_NAME})
    set_target_properties(stereo_resize_geometry_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME stereo_resize_geometry_regression COMMAND stereo_resize_geometry_regression
        ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial/DECXIN_Camera.yaml)
    add_test(NAME stereo_resize_geometry_light_regression COMMAND stereo_resize_geometry_regression
        ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial/DECXIN_Camera_Light.yaml)
    add_executable(stereo_match_regression tests/stereo_match_regression.cc)
    target_link_libraries(stereo_match_regression ${PROJECT_NAME})
    set_target_properties(stereo_match_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME stereo_match_regression COMMAND stereo_match_regression)
    add_executable(extractor_stage_benchmark EXCLUDE_FROM_ALL tests/extractor_stage_benchmark.cc)
    target_link_libraries(extractor_stage_benchmark ${PROJECT_NAME})
    set_target_properties(extractor_stage_benchmark PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_executable(feature_profile_benchmark EXCLUDE_FROM_ALL tests/feature_profile_benchmark.cc)
    target_link_libraries(feature_profile_benchmark ${PROJECT_NAME})
    set_target_properties(feature_profile_benchmark PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_executable(tracking_window_regression tests/tracking_window_regression.cc)
    target_link_libraries(tracking_window_regression ${PROJECT_NAME})
    set_target_properties(tracking_window_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME tracking_window_regression COMMAND tracking_window_regression
        ${PROJECT_SOURCE_DIR}/Vocabulary/ORBvoc.txt ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial/DECXIN_Camera.yaml enforce)
    add_executable(rolling_input_regression tests/rolling_input_regression.cc)
    target_link_libraries(rolling_input_regression ${PROJECT_NAME})
    set_target_properties(rolling_input_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME rolling_input_regression COMMAND rolling_input_regression)
    add_executable(input_recording_regression tests/input_recording_regression.cc)
    target_link_libraries(input_recording_regression ${PROJECT_NAME})
    set_target_properties(input_recording_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME input_recording_regression COMMAND input_recording_regression)
    add_executable(imu_settings_regression tests/imu_settings_regression.cc)
    target_link_libraries(imu_settings_regression ${PROJECT_NAME})
    set_target_properties(imu_settings_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME imu_settings_regression COMMAND imu_settings_regression ${PROJECT_SOURCE_DIR})
    add_executable(atlas_save_regression tests/atlas_save_regression.cc)
    target_link_libraries(atlas_save_regression ${PROJECT_NAME})
    set_target_properties(atlas_save_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME atlas_save_regression COMMAND atlas_save_regression)
    add_executable(recovery_report_regression tests/recovery_report_regression.cc)
    set_target_properties(recovery_report_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME recovery_report_regression COMMAND recovery_report_regression)
    # Console workflow test uses the archive/recovery profile; flight policy has
    # its own lifecycle regression below.
    file(READ ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial/DECXIN_Camera.yaml console_settings)
    string(REGEX REPLACE "Tracking.preserve_mature_map: [01]" "Tracking.preserve_mature_map: 0" console_settings "${console_settings}")
    file(WRITE ${CMAKE_BINARY_DIR}/console_test_settings.yaml "${console_settings}")
    add_executable(flight_map_policy_regression tests/flight_map_policy_regression.cc)
    target_link_libraries(flight_map_policy_regression ${PROJECT_NAME})
    set_target_properties(flight_map_policy_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME flight_map_policy_regression COMMAND flight_map_policy_regression
        ${PROJECT_SOURCE_DIR}/Vocabulary/ORBvoc.txt ${CMAKE_BINARY_DIR}/console_test_settings.yaml)
    set_tests_properties(flight_map_policy_regression PROPERTIES TIMEOUT 60)
    add_executable(recovery_console_regression tests/recovery_console_regression.cc)
    target_link_libraries(recovery_console_regression ${PROJECT_NAME})
    set_target_properties(recovery_console_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_test(NAME recovery_console_regression COMMAND recovery_console_regression
        ${PROJECT_SOURCE_DIR}/Vocabulary/ORBvoc.txt ${CMAKE_BINARY_DIR}/console_test_settings.yaml)
    add_executable(stereo_inertial_save_replay tests/stereo_inertial_save_replay.cc)
    target_link_libraries(stereo_inertial_save_replay ${PROJECT_NAME})
    set_target_properties(stereo_inertial_save_replay PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    add_executable(d435i_recovery_replay tests/d435i_recovery_replay.cc)
    target_link_libraries(d435i_recovery_replay ${PROJECT_NAME})
    set_target_properties(d435i_recovery_replay PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    if(realsense2_FOUND AND ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
        add_executable(d435i_capture_slam tests/d435i_capture_slam.cc)
        target_link_libraries(d435i_capture_slam ${PROJECT_NAME})
        orb_link_realsense(d435i_capture_slam)
        set_target_properties(d435i_capture_slam PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
    endif()
endif()

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples/RGB-D)

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(rgbd_tum
            Examples/RGB-D/rgbd_tum.cc)
    target_link_libraries(rgbd_tum ${PROJECT_NAME})
endif()

if(realsense2_FOUND)
    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(rgbd_realsense_D435i
                Examples/RGB-D/rgbd_realsense_D435i.cc)
        target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
    orb_link_realsense(rgbd_realsense_D435i)
endif()
endif()


# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples/RGB-D-Inertial)

if(realsense2_FOUND)
    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(rgbd_inertial_realsense_D435i
                Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
        target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
    orb_link_realsense(rgbd_inertial_realsense_D435i)
endif()
endif()

#Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples/Stereo)

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(stereo_kitti
            Examples/Stereo/stereo_kitti.cc)
    target_link_libraries(stereo_kitti ${PROJECT_NAME})
endif()

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(stereo_euroc
            Examples/Stereo/stereo_euroc.cc)
    target_link_libraries(stereo_euroc ${PROJECT_NAME})
endif()

if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(stereo_uvc
            Examples/Stereo/stereo_uvc.cc)
    target_link_libraries(stereo_uvc ${PROJECT_NAME})
endif()

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(stereo_tum_vi
            Examples/Stereo/stereo_tum_vi.cc)
    target_link_libraries(stereo_tum_vi ${PROJECT_NAME})
endif()

if(realsense2_FOUND)
    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(stereo_realsense_t265
                Examples/Stereo/stereo_realsense_t265.cc)
        target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})
    orb_link_realsense(stereo_realsense_t265)
endif()

    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(stereo_realsense_D435i
                Examples/Stereo/stereo_realsense_D435i.cc)
        target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
    orb_link_realsense(stereo_realsense_D435i)
endif()
endif()

#Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples/Monocular)

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(mono_tum
            Examples/Monocular/mono_tum.cc)
    target_link_libraries(mono_tum ${PROJECT_NAME})
endif()

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(mono_kitti
            Examples/Monocular/mono_kitti.cc)
    target_link_libraries(mono_kitti ${PROJECT_NAME})
endif()

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(mono_euroc
            Examples/Monocular/mono_euroc.cc)
    target_link_libraries(mono_euroc ${PROJECT_NAME})
endif()

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(mono_tum_vi
            Examples/Monocular/mono_tum_vi.cc)
    target_link_libraries(mono_tum_vi ${PROJECT_NAME})
endif()

if(realsense2_FOUND)
    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(mono_realsense_t265
                Examples/Monocular/mono_realsense_t265.cc)
        target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})
    orb_link_realsense(mono_realsense_t265)
endif()

    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(mono_realsense_D435i
                Examples/Monocular/mono_realsense_D435i.cc)
        target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
    orb_link_realsense(mono_realsense_D435i)
endif()
endif()

#Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples/Monocular-Inertial)

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(mono_inertial_euroc
            Examples/Monocular-Inertial/mono_inertial_euroc.cc)
    target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})
endif()

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(mono_inertial_tum_vi
            Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
    target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})
endif()

if(realsense2_FOUND)
    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(mono_inertial_realsense_t265
                Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
        target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})
    orb_link_realsense(mono_inertial_realsense_t265)
endif()

    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(mono_inertial_realsense_D435i
                Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
        target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
    orb_link_realsense(mono_inertial_realsense_D435i)
endif()
endif()

#Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples/Stereo-Inertial)

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(stereo_inertial_euroc
            Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
    target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})
endif()

if(ORB_SLAM3_BUILD_DATASET_EXAMPLES)
    add_executable(stereo_inertial_tum_vi
            Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
    target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})
endif()

if(realsense2_FOUND)
    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(stereo_inertial_realsense_t265
                Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
        target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})
    orb_link_realsense(stereo_inertial_realsense_t265)
endif()

    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(stereo_inertial_realsense_D435i
                Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
        target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
    orb_link_realsense(stereo_inertial_realsense_D435i)
endif()
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples/Calibration)
if(realsense2_FOUND)
    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(recorder_realsense_D435i
                Examples/Calibration/recorder_realsense_D435i.cc)
        target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})
    orb_link_realsense(recorder_realsense_D435i)
endif()

    if(ORB_SLAM3_BUILD_CAMERA_EXAMPLES)
    add_executable(recorder_realsense_T265
                Examples/Calibration/recorder_realsense_T265.cc)
        target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
    orb_link_realsense(recorder_realsense_T265)
endif()
endif()

#Old examples
if(ORB_SLAM3_BUILD_LEGACY_EXAMPLES)


# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples_old/RGB-D)

add_executable(rgbd_tum_old
        Examples_old/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i_old
            Examples_old/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i_old ${PROJECT_NAME})
orb_link_realsense(rgbd_realsense_D435i_old)
endif()


# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples_old/RGB-D-Inertial)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i_old
            Examples_old/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i_old ${PROJECT_NAME})
orb_link_realsense(rgbd_inertial_realsense_D435i_old)
endif()

#Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples_old/Stereo)

add_executable(stereo_kitti_old
        Examples_old/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti_old ${PROJECT_NAME})

add_executable(stereo_euroc_old
        Examples_old/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc_old ${PROJECT_NAME})

add_executable(stereo_tum_vi_old
        Examples_old/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265_old
            Examples_old/Stereo/stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265_old ${PROJECT_NAME})
orb_link_realsense(stereo_realsense_t265_old)

    add_executable(stereo_realsense_D435i_old
            Examples_old/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i_old ${PROJECT_NAME})
orb_link_realsense(stereo_realsense_D435i_old)
endif()

#Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples_old/Monocular)

add_executable(mono_tum_old
        Examples_old/Monocular/mono_tum.cc)
target_link_libraries(mono_tum_old ${PROJECT_NAME})

add_executable(mono_kitti_old
        Examples_old/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti_old ${PROJECT_NAME})

add_executable(mono_euroc_old
        Examples_old/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc_old ${PROJECT_NAME})

add_executable(mono_tum_vi_old
        Examples_old/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_realsense_t265_old
            Examples_old/Monocular/mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265_old ${PROJECT_NAME})
orb_link_realsense(mono_realsense_t265_old)

    add_executable(mono_realsense_D435i_old
            Examples_old/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i_old ${PROJECT_NAME})
orb_link_realsense(mono_realsense_D435i_old)
endif()

#Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples_old/Monocular-Inertial)

add_executable(mono_inertial_euroc_old
        Examples_old/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc_old ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi_old
        Examples_old/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265_old
            Examples_old/Monocular-Inertial/mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265_old ${PROJECT_NAME})
orb_link_realsense(mono_inertial_realsense_t265_old)

    add_executable(mono_inertial_realsense_D435i_old
            Examples_old/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i_old ${PROJECT_NAME})
orb_link_realsense(mono_inertial_realsense_D435i_old)
endif()

#Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Examples_old/Stereo-Inertial)

add_executable(stereo_inertial_euroc_old
        Examples_old/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc_old ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi_old
        Examples_old/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265_old
            Examples_old/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265_old ${PROJECT_NAME})
orb_link_realsense(stereo_inertial_realsense_t265_old)

    add_executable(stereo_inertial_realsense_D435i_old
            Examples_old/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i_old ${PROJECT_NAME})
orb_link_realsense(stereo_inertial_realsense_D435i_old)
endif()


endif()

# Conversion/gating tests remain available without a ROS installation.
if(ORB_SLAM3_WITH_PX4 OR ORB_SLAM3_BUILD_TESTS)
    add_library(orbslam3_px4_core STATIC src/Px4Vio.cc)
    target_link_libraries(orbslam3_px4_core ${OpenCV_LIBS})
    if(ORB_SLAM3_BUILD_TESTS)
        add_executable(px4_vio_regression tests/px4_vio_regression.cc)
        target_link_libraries(px4_vio_regression orbslam3_px4_core)
        set_target_properties(px4_vio_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
        add_test(NAME px4_vio_regression COMMAND px4_vio_regression ${PROJECT_SOURCE_DIR})
        add_executable(px4_recording_gate_regression tests/px4_recording_gate_regression.cc)
        target_link_libraries(px4_recording_gate_regression orbslam3_px4_core)
        set_target_properties(px4_recording_gate_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
        add_test(NAME px4_recording_gate_regression COMMAND px4_recording_gate_regression
            ${PROJECT_SOURCE_DIR}/config/px4_vio_local.yaml)
        add_executable(px4_sortie_regression tests/px4_sortie_regression.cc)
        target_link_libraries(px4_sortie_regression orbslam3_px4_core)
        set_target_properties(px4_sortie_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
        add_test(NAME px4_sortie_regression COMMAND px4_sortie_regression)
    endif()
endif()
if(ORB_SLAM3_WITH_PX4)
    include(cmake/RosDependencies.cmake)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(px4_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    add_library(orbslam3_px4_imu STATIC src/Px4Imu.cc)
    target_link_libraries(orbslam3_px4_imu orbslam3_px4_core)
    orb_target_ros_dependencies(orbslam3_px4_imu rclcpp px4_msgs)
    target_compile_definitions(orbslam3_px4_imu PUBLIC ORB_SLAM3_WITH_PX4=1)
    add_library(orbslam3_px4 STATIC src/Px4Bridge.cc)
    target_compile_definitions(orbslam3_px4 PUBLIC ORB_SLAM3_WITH_PX4=1)
    target_link_libraries(orbslam3_px4 orbslam3_px4_core)
    orb_target_ros_dependencies(orbslam3_px4 rclcpp px4_msgs std_msgs)
    if(TARGET stereo_uvc)
        target_link_libraries(stereo_uvc orbslam3_px4 orbslam3_px4_imu)
    endif()
    if(ORB_SLAM3_BUILD_TESTS)
        add_executable(px4_imu_buffer_regression tests/px4_imu_buffer_regression.cc)
        set_target_properties(px4_imu_buffer_regression PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
        add_test(NAME px4_imu_buffer_regression COMMAND px4_imu_buffer_regression)
        add_executable(px4_imu_probe tests/px4_imu_probe.cc)
        target_link_libraries(px4_imu_probe orbslam3_px4_imu)
        set_target_properties(px4_imu_probe PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
        add_test(NAME px4_imu_ros_regression COMMAND /usr/bin/python3
            ${PROJECT_SOURCE_DIR}/tests/px4_imu_ros_regression.py $<TARGET_FILE:px4_imu_probe>)
        set_tests_properties(px4_imu_ros_regression PROPERTIES TIMEOUT 25 RUN_SERIAL TRUE)
    endif()
    if(TARGET stereo_inertial_realsense_D435i)
        target_link_libraries(stereo_inertial_realsense_D435i orbslam3_px4)
    endif()
    if(ORB_SLAM3_BUILD_TESTS)
        target_link_libraries(d435i_recovery_replay orbslam3_px4)
        add_executable(px4_bridge_fixture tests/px4_bridge_fixture.cc)
        target_link_libraries(px4_bridge_fixture orbslam3_px4)
        set_target_properties(px4_bridge_fixture PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)
        add_test(NAME px4_bridge_ros_regression COMMAND /usr/bin/python3
            ${PROJECT_SOURCE_DIR}/tests/px4_bridge_ros_regression.py $<TARGET_FILE:px4_bridge_fixture>)
        set_tests_properties(px4_bridge_ros_regression PROPERTIES TIMEOUT 50 RUN_SERIAL TRUE)
        add_test(NAME px4_sortie_ros_regression COMMAND /usr/bin/python3
            ${PROJECT_SOURCE_DIR}/tests/px4_sortie_ros_regression.py $<TARGET_FILE:px4_bridge_fixture>)
        set_tests_properties(px4_sortie_ros_regression PROPERTIES TIMEOUT 55 RUN_SERIAL TRUE)
        add_test(NAME vio_energy_ros_regression COMMAND /usr/bin/python3
            ${PROJECT_SOURCE_DIR}/tests/vio_energy_ros_regression.py)
        set_tests_properties(vio_energy_ros_regression PROPERTIES TIMEOUT 25 RUN_SERIAL TRUE)
    endif()
endif()

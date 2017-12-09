cmake_minimum_required(VERSION 2.8.3)

#set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11" )

project(RAIN-VIO)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(OpenCV_DIR "/home/rain/workspace/library/opencv-3.2.0/install/share/OpenCV")
#set(Eigen3_DIR "/home/rain/workspace/Library/eigen3")


find_package(OpenCV REQUIRED)
#if(NOT OpenCV_FOUND)
#    find_package(OpenCV 2.4.3 QUIET)
#    if(NOT OpenCV_FOUND)
#        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#    endif()
#endif()

find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)

include_directories( "/usr/include/eigen3/")

include_directories(
    include
    ${Opencv_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    BEFORE ${GLOG_INCLUDE_DIRS}
)

# add source file
add_library(${PROJECT_NAME} SHARED
    src/Frame.cpp
    src/System.cpp
    src/KeyFrame.cpp
    src/Tracking.cpp
    src/Camera.cpp
    src/Feature.cpp
    src/Initializer.cpp
    src/Map.cpp
    src/BenchMark.cpp
    src/Converter.cpp
    src/Viewer.cpp
    src/Optimizer.cpp
    src/Parameters.cpp
    src/MapDrawer.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
    ${Pangolin_LIBRARIES}
)

add_executable(mono_euroc
    Example/mono_euroc.cpp
)

target_link_libraries(mono_euroc
    ${PROJECT_NAME}
)

add_executable(test_glog test/test_glog.cpp )
target_link_libraries(test_glog ${PROJECT_NAME})

add_executable(test_pangolin test/test_pangolin.cpp)
target_link_libraries(test_pangolin ${PROJECT_NAME})

add_executable(test_pnp test/test_pnp/test_OpencvPnP.cpp)
target_link_libraries(test_pnp ${PROJECT_NAME})


add_executable(test_BA
    test/test_ceres/test_BA.cpp
    test/test_ceres/pose_local_parameterization.cpp
    test/test_ceres/pose_local_parameterization.h
    test/test_ceres/se3.h)
target_link_libraries(test_BA ${PROJECT_NAME})

add_executable(test_converter  test/test_src/test_converter.cpp)
target_link_libraries(test_converter ${PROJECT_NAME})

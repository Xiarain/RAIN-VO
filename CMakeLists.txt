cmake_minimum_required(VERSION 2.8.3)

#set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11" )

project(RAIN-VIO)

set(OpenCV_DIR "/home/rain/workspace/Library/opencv3/share/OpenCV")
#set(Eigen3_DIR "/home/rain/workspace/Library/eigen3")


find_package(OpenCV)
#if(NOT OpenCV_FOUND)
#    find_package(OpenCV 2.4.3 QUIET)
#    if(NOT OpenCV_FOUND)
#        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
#    endif()
#endif()


find_package(Ceres REQUIRED)

include_directories( "/usr/include/eigen3/")

include_directories(
    include
    ../include
    ${Opencv_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

# 添加源文件
#add_library(${PROJECT_NAME} SHARED
#    src/Frame.cpp
#    src/System.cpp)

add_executable(VIO
    src/main.cpp
    src/Frame.cpp
    src/System.cpp
    src/KeyFrame.cpp
    src/Tracking.cpp
    src/Camera.cpp
    src/Feature.cpp
    src/Initializer.cpp
    src/Map.cpp)

target_link_libraries(VIO
    ${OpenCV_LIBRARIES}
    ${CERES_LIBRARIES}
)




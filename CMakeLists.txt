cmake_minimum_required(VERSION 3.0.0)
project(Visualizer VERSION 0.1.0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(image_transport REQUIRED)
include_directories(${image_transport_INCLUDE_DIRS})

find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})

find_package(pcl_ros REQUIRED)
include_directories(${pcl_ros_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O0 -g3 -Wall -fPIC)
    file(GLOB_RECURSE SOURCE_PATH ${PROJECT_SOURCE_DIR}/src/*.cpp)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_PATH})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${roscpp_LIBRARIES} ${image_transport_LIBRARIES} ${cv_bridge_LIBRARIES} ${pcl_ros_LIBRARIES} libjsoncpp.so)
elseif (CMAKE_BUILD_TYPE MATCHES Release)
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O3 -Wall -fPIC)
    file(GLOB_RECURSE SOURCE_PATH ${PROJECT_SOURCE_DIR}/src/*.cpp)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_PATH})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${roscpp_LIBRARIES} ${image_transport_LIBRARIES} ${cv_bridge_LIBRARIES} ${pcl_ros_LIBRARIES} libjsoncpp.so)
else()
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O3 -Wall -fPIC)   
endif()

add_executable(CameraVisualizer ${PROJECT_SOURCE_DIR}/app/run_CameraVisualizer.cpp)
target_link_libraries(CameraVisualizer ${PROJECT_NAME} ${roscpp_LIBRARIES} libjsoncpp.so)

add_executable(LidarVisualizer ${PROJECT_SOURCE_DIR}/app/run_LidarVisualizer.cpp)
target_link_libraries(LidarVisualizer ${PROJECT_NAME} ${roscpp_LIBRARIES} libjsoncpp.so)
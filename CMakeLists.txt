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

find_package(message_filters REQUIRED)
include_directories(${message_filters_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O0 -g3 -Wall -fPIC)
    file(GLOB_RECURSE SOURCE_PATH ${PROJECT_SOURCE_DIR}/src/*.cpp)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_PATH})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${roscpp_LIBRARIES} ${message_filters_LIBRARIES} libjsoncpp.so)
elseif (CMAKE_BUILD_TYPE MATCHES Release)
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O3 -Wall -fPIC)
    file(GLOB_RECURSE SOURCE_PATH ${PROJECT_SOURCE_DIR}/src/*.cpp)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_PATH})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${roscpp_LIBRARIES} ${message_filters_LIBRARIES} libjsoncpp.so)
else()
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O3 -Wall -fPIC)   
endif()

add_executable(visualizer ${PROJECT_SOURCE_DIR}/app/main.cpp)
target_link_libraries(visualizer ${PROJECT_NAME} ${roscpp_LIBRARIES} libjsoncpp.so)

add_executable(CameraVisualizer ${PROJECT_SOURCE_DIR}/app/run_CameraVisualizer.cpp)
target_link_libraries(CameraVisualizer ${PROJECT_NAME} ${roscpp_LIBRARIES} libjsoncpp.so)

add_executable(LidarVisualizer ${PROJECT_SOURCE_DIR}/app/run_LidarVisualizer.cpp)
target_link_libraries(LidarVisualizer ${PROJECT_NAME} ${roscpp_LIBRARIES} libjsoncpp.so)

add_executable(MapVisualizer ${PROJECT_SOURCE_DIR}/app/run_MapVisualizer.cpp)
target_link_libraries(MapVisualizer ${PROJECT_NAME} ${roscpp_LIBRARIES} libjsoncpp.so)

add_executable(RoadVisualizer ${PROJECT_SOURCE_DIR}/app/run_RoadVisualizer.cpp)
target_link_libraries(RoadVisualizer ${PROJECT_NAME} ${roscpp_LIBRARIES} libjsoncpp.so)
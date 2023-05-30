cmake_minimum_required(VERSION 3.0.0)
project(CameraVisualizer VERSION 0.1.0)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

find_package(image_transport REQUIRED)
include_directories(${image_transport_INCLUDE_DIRS})

find_package(cv_bridge REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O0 -g3 -Wall -fPIC)
    file(GLOB_RECURSE SOURCE_PATH ${PROJECT_SOURCE_DIR}/src/*.cpp)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_PATH})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${roscpp_LIBRARIES} ${image_transport_LIBRARIES} ${cv_bridge_LIBRARIES} libjsoncpp.so)
    
    add_executable(CameraVisualizerApp ${PROJECT_SOURCE_DIR}/app/main.cpp)
    target_link_libraries(CameraVisualizerApp ${PROJECT_NAME})
elseif (CMAKE_BUILD_TYPE MATCHES Release)
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O3 -Wall -fPIC)
    file(GLOB_RECURSE SOURCE_PATH ${PROJECT_SOURCE_DIR}/src/*.cpp)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_PATH})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${roscpp_LIBRARIES} ${image_transport_LIBRARIES} ${cv_bridge_LIBRARIES} libjsoncpp.so)
    
    add_executable(CameraVisualizerApp ${PROJECT_SOURCE_DIR}/app/main.cpp)
    target_link_libraries(CameraVisualizerApp ${PROJECT_NAME})
else()
    set(DEFAULT_COMPILE -std=c++14 -fpermissive -O3 -Wall -fPIC)
    
    add_executable(CameraVisualizerApp ${PROJECT_SOURCE_DIR}/app/main.cpp)
    target_link_libraries(CameraVisualizerApp ${PROJECT_SOURCE_DIR}/lib/lib${PROJECT_NAME}.so ${roscpp_LIBRARIES} libjsoncpp.so)
endif()



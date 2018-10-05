cmake_minimum_required(VERSION 2.8)
project(vs_common)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -Werror")
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    src
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB LIB_SRC src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${LIB_SRC})
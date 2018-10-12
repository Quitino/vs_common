cmake_minimum_required(VERSION 2.8)
project(vs_common)

set(BUILD_TEST 1)
set(BUILD_GTEST 1)

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

file(GLOB HEADERS src/*.h src/*.hpp)
install(FILES ${HEADERS} DESTINATION /usr/local/include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION /usr/local/lib)

if(${BUILD_TEST})
    add_subdirectory(test)
endif()

if(${BUILD_GTEST})
    add_subdirectory(gtest)
endif()
file(GLOB cpp_samples RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

foreach(sample_filename ${cpp_samples})
    get_filename_component(sample ${sample_filename} NAME_WE)
    add_executable(${sample} ${sample_filename})
    target_link_libraries(${sample} ${PROJECT_NAME} ${OpenCV_LIBS} pthread)
endforeach()

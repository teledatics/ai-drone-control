cmake_minimum_required(VERSION 3.10.0)	#telling CMake
project(face_tracker_test VERSION 1.0.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
add_definitions(${GCC_COMPILE_FLAGS})

add_executable(face_tracker_test ${PROJECT_SOURCE_DIR}/face_tracker_test.cc)   #must be in the same directory, else cmake seems to have difficulty finding it
include_directories(../)
include_directories(../../filters/include)
target_link_libraries(face_tracker_test gtest pthread )  #must have pthread no matter what your project is!!, also link the binary to the lib.

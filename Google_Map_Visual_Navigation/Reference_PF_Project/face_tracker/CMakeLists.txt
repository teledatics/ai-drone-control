cmake_minimum_required(VERSION 3.10.0)	#telling CMake
project(face_tracker VERSION 1.0.0)

find_package(PythonInterp 3.6 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
add_subdirectory(pybind11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3 -fvisibility=hidden") 
set(CMAKE_CXX_STANDARD 14)
add_definitions(${GCC_COMPILE_FLAGS})

pybind11_add_module(face_tracker face_tracker.cpp)
target_include_directories(face_tracker PUBLIC ../filters/include)

add_definitions( -D_DEBUG )

add_custom_command(TARGET PUBLIC face_tracker  POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/face_tracker/face_tracker.cpython* ${CMAKE_BINARY_DIR}/../face_tracker/face_tracker.so)

# add_subdirectory(gtest)

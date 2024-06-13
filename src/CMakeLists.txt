cmake_minimum_required(VERSION 3.01)

project(SATController)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_FLAGS "-static-libgcc -static-libstdc++ -static")

add_executable(satcontroller main.cpp)

target_link_libraries(
  satcontroller)

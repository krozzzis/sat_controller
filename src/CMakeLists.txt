cmake_minimum_required(VERSION 3.23)

project(SATController)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(satcontroller main.cpp)

add_library(Controller Controller.cpp)
add_library(Params Params.cpp)
add_library(Vec Vec.cpp)

target_link_libraries(
  satcontroller
  PRIVATE Controller
          Params
          Vec)

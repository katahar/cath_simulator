cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(catheter_simulator)


# find_package(Eigen3 3.3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )

add_subdirectory(./fssimplewindow/src ${CMAKE_BINARY_DIR}/fssimplewindow)
# add_subdirectory(./eigen-3.4.0 ${CMAKE_BINARY_DIR}/eigen)

add_executable(cath_sim MACOSX_BUNDLE cath_sim.cpp)
target_link_libraries(cath_sim fssimplewindow )

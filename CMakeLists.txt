cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(catheter_simulator)



add_subdirectory(./fssimplewindow/src ${CMAKE_BINARY_DIR}/fssimplewindow)

add_executable(cath_sim MACOSX_BUNDLE cath_sim.cpp)
target_link_libraries(cath_sim fssimplewindow)

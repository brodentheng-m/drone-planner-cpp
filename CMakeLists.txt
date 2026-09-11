cmake_minimum_required(VERSION 3.20)
project(DronePlannerCpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(engine STATIC
  src/aero/AeroEngine.cpp
  src/sim/Eval.cpp
  src/sim/Simulator.cpp
  src/sim/Obstacle.cpp
  src/codegen/CodeGenerator.cpp
)
target_include_directories(engine PUBLIC src)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(engine PRIVATE -Wall -Wextra)
endif()

enable_testing()
add_executable(aero_test src/aero/aero_test.cpp)
target_link_libraries(aero_test PRIVATE engine)
add_test(NAME aero_test COMMAND aero_test)

add_executable(sim_test src/sim/sim_test.cpp)
target_link_libraries(sim_test PRIVATE engine)
add_test(NAME sim_test COMMAND sim_test)

add_executable(codegen_test src/codegen/codegen_test.cpp)
target_link_libraries(codegen_test PRIVATE engine)
add_test(NAME codegen_test COMMAND codegen_test)

add_executable(obstacle_test src/sim/obstacle_test.cpp)
target_link_libraries(obstacle_test PRIVATE engine)
add_test(NAME obstacle_test COMMAND obstacle_test)

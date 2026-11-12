cmake_minimum_required(VERSION 3.20)
project(wro_simulator LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ORIGINAL ${CMAKE_SOURCE_DIR}/original/WRO2026Senior-new_drivebase_shuga/main)
add_executable(wro-sim-engine
  simulator/native/main.cpp
  simulator/native/virtual_arduino.cpp
  ${ORIGINAL}/src/motion_lib.cpp
  ${ORIGINAL}/src/servo_lib.cpp
  ${ORIGINAL}/src/pi_color_lib.cpp
  ${ORIGINAL}/src/move_function.cpp
  ${ORIGINAL}/src/extra.cpp
  ${ORIGINAL}/src/main.cpp)
target_include_directories(wro-sim-engine PRIVATE
  simulator/native/include ${ORIGINAL}/include)
if(MSVC)
  target_compile_options(wro-sim-engine PRIVATE /W4 /EHsc)
else()
  target_compile_options(wro-sim-engine PRIVATE -Wall -Wextra -Wno-unused-parameter)
  target_link_libraries(wro-sim-engine PRIVATE pthread)
endif()

enable_testing()
add_test(NAME native_demo COMMAND wro-sim-engine --program demo --quiet)
add_test(NAME native_reverse COMMAND wro-sim-engine --program reverse --quiet)

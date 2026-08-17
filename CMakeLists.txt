cmake_minimum_required(VERSION 3.18)
project(av_stack_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(MSVC)
  add_compile_options(/O2 /W3)
else()
  add_compile_options(-O2 -Wall)
endif()

# --- av_stack's own includes (the ad/* headers are bundled here too) ---
include_directories(${CMAKE_SOURCE_DIR}/cpp/include)

# SpeedGovernor has a .cpp, so build it as a static library
add_library(ad_safety STATIC cpp/src/safety/speed_governor.cpp)
target_include_directories(ad_safety PUBLIC ${CMAKE_SOURCE_DIR}/cpp/include)
if(MSVC)
  target_compile_options(ad_safety PRIVATE /W3)
else()
  target_compile_options(ad_safety PRIVATE -Wall)
endif()

# C++ verification test
add_executable(control_test cpp/src/control_test.cpp)

# pybind11 module: control_cpp
#
# `pip install pybind11` puts the CMake config under the interpreter's
# site-packages, which CMake does not search, so a bare find_package(pybind11)
# fails even though pybind11 is installed -- the configure step has never
# succeeded in CI for this reason. Ask the interpreter where its config lives
# before giving up, so `pip install pybind11 && cmake -S . -B build` just works.
#
# Development.Module is requested alongside Interpreter because pybind11 3.x
# reuses whatever Python target it finds; with Interpreter alone its
# pybind11_add_module() fails inside python_add_library().
if(NOT pybind11_DIR)
  find_package(Python COMPONENTS Interpreter Development.Module QUIET)
  if(Python_EXECUTABLE)
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -c
              "import pybind11; print(pybind11.get_cmake_dir())"
      OUTPUT_VARIABLE _pybind11_cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      RESULT_VARIABLE _pybind11_query)
    if(_pybind11_query EQUAL 0 AND EXISTS "${_pybind11_cmake_dir}")
      set(pybind11_DIR "${_pybind11_cmake_dir}")
      message(STATUS "pybind11 CMake config from the interpreter: ${pybind11_DIR}")
    endif()
  endif()
endif()
find_package(pybind11 REQUIRED)
pybind11_add_module(control_cpp cpp/src/bindings_control.cpp)

# pybind11 module: world_cpp (multi-object tracker / world model)
pybind11_add_module(world_cpp cpp/src/bindings_world.cpp)

# pybind11 module: localization_cpp
pybind11_add_module(localization_cpp cpp/src/bindings_localization.cpp)

# pybind11 module: safety_cpp
pybind11_add_module(safety_cpp cpp/src/bindings_safety.cpp)
target_link_libraries(safety_cpp PRIVATE ad_safety)

# ---------------------------------------------------------------------------
# New C++-first pipeline modules (Phase 2 bindings)
# ---------------------------------------------------------------------------
pybind11_add_module(scenario_cpp    cpp/src/bindings_scenario.cpp)
pybind11_add_module(sensors_cpp     cpp/src/bindings_sensors.cpp)
pybind11_add_module(perception_cpp  cpp/src/bindings_perception.cpp)
pybind11_add_module(planning_cpp    cpp/src/bindings_planning.cpp)
pybind11_add_module(guardrail_cpp   cpp/src/bindings_guardrail.cpp)
pybind11_add_module(occupancy_cpp   cpp/src/bindings_occupancy.cpp)

# Hybrid arbiter: confidence-weighted NN/MPC fusion
pybind11_add_module(hybrid_cpp      cpp/src/bindings_hybrid.cpp)

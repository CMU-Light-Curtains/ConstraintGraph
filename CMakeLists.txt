# Minimum version supported to be able to do "set(CMAKE_CXX_STANDARD 17)"
cmake_minimum_required(VERSION 3.13.5)

# Use C++ 14 standard for the whole project
set(CMAKE_CXX_STANDARD 14)

# Options to create bindings.
option(BIND_PY "Create python bindings"     OFF)
option(BIND_JS "Create javascript bindings" OFF)

if (BIND_PY AND BIND_JS)
    message(FATAL_ERROR "Both -DBIND_PY and -DBIND_JS cannot be ON. Bindings can only be generated for one language.")
endif()

# Set Emscripten toolchain file for JS bindings.
if (BIND_JS)
    set (CMAKE_TOOLCHAIN_FILE "$ENV{EMSDK}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
endif()

find_package(Eigen3)

# ======================================================================================================================
# Library: planner_cpp
# ======================================================================================================================

# Specify directories containing header files
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        include/planner
)

file(GLOB planner_cpp_SRC
        "include/planner/base.h"
        "include/planner/v1.h"
        "include/planner/v2.h"
        "src/base.cpp"
        "src/v1.cpp"
        "src/v2.cpp"
        )
add_library(planner_cpp ${planner_cpp_SRC})
set_property(TARGET planner_cpp PROPERTY POSITION_INDEPENDENT_CODE ON)

# ======================================================================================================================
# Library: planner_py
# ======================================================================================================================

if(BIND_PY)
    find_package(PythonLibs 3)
    include_directories(${PYTHON_INCLUDE_DIRS})

    add_subdirectory(lib/pybind11)
    pybind11_add_module(planner_py src/bind/py.cpp)
    target_link_libraries(planner_py PRIVATE pybind11::module planner_cpp)
endif()

# ======================================================================================================================
# Executable: planner_js
# ======================================================================================================================

if(BIND_JS)
    add_executable(planner_js src/bind/js.cpp)
    target_link_libraries(planner_js PRIVATE planner_cpp)
    set_target_properties(planner_js PROPERTIES LINK_FLAGS "-s DEMANGLE_SUPPORT=1 -s ALLOW_MEMORY_GROWTH --bind")
endif()

# ======================================================================================================================

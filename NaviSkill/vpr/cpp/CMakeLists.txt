cmake_minimum_required(VERSION 3.16)
project(vrr_vpr VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------

# Python3 — must match the environment where vrr_vpr is installed.
# Pass -DPython3_EXECUTABLE=/path/to/python to pin a specific interpreter.
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# pybind11 — try system / pip-installed pybind11 first, then fetch.
find_package(pybind11 2.11 QUIET)
if(NOT pybind11_FOUND)
    message(STATUS "pybind11 not found via find_package, trying pybind11_DIR from pip...")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
        OUTPUT_VARIABLE _pybind11_cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _pybind11_result
    )
    if(_pybind11_result EQUAL 0)
        set(pybind11_DIR "${_pybind11_cmake_dir}")
        find_package(pybind11 2.11 REQUIRED)
    else()
        message(FATAL_ERROR
            "pybind11 not found. Install it:\n"
            "  pip install pybind11   OR   apt install pybind11-dev")
    endif()
endif()

# --------------------------------------------------------------------------
# Library target: vrr_vpr
# --------------------------------------------------------------------------

add_library(vrr_vpr
    src/vrr_vpr.cpp
)

target_include_directories(vrr_vpr
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(vrr_vpr
    PUBLIC
        pybind11::embed
        Python3::NumPy
)

# --------------------------------------------------------------------------
# Example executable
# --------------------------------------------------------------------------

option(VRR_BUILD_EXAMPLE "Build the example program" ON)

if(VRR_BUILD_EXAMPLE)
    add_executable(vrr_vpr_example example/main.cpp)
    target_link_libraries(vrr_vpr_example PRIVATE vrr_vpr)
endif()

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------

include(GNUInstallDirs)

install(TARGETS vrr_vpr
    EXPORT vrr_vpr-targets
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT vrr_vpr-targets
    FILE vrr_vpr-config.cmake
    NAMESPACE vrr::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vrr_vpr
)

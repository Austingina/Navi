# 3.24 for CMAKE_CUDA_ARCHITECTURES=native, used below in place of the removed
# FindCUDA/cuda_add_library pair. Ubuntu 24.04 ships 3.28.
cmake_minimum_required(VERSION 3.24)
project(bbs3d)

## find thirdparty/Eigen or Eigen3 package
option(USE_THIRDPARTY_EIGEN "Include thirdparty eigen" OFF)
file(GLOB entries "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen/*")
list(LENGTH entries num_entries)
message(STATUS "num_entries in thirdparty/Eigen: ${num_entries}")
if(num_entries EQUAL 0)
  find_package(Eigen3 REQUIRED)
else()
  set(USE_THIRDPARTY_EIGEN ON)
endif()

## find OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## find CUDA
option(BUILD_CUDA "Build GPU ver" ON)
if (BUILD_CUDA)
  # Has to be set before enable_language(CUDA), which otherwise leaves nvcc on
  # its built-in default (sm_52) -- wrong for every GPU this runs on.
  #
  # "native" is deliberately avoided. It asks nvcc to resolve the arch off the
  # local driver, which fails in both directions: with no driver visible (the
  # normal state inside a build container) nvcc silently falls back to sm_52,
  # and when the GPU is newer than the toolkit it errors out instead -- e.g. a
  # dev box here pairs an RTX 5090 (sm_120) with CUDA 12.0, whose nvcc only
  # knows up to compute_90.
  #
  # So: name real archs for the boards, plus one -virtual entry that ships PTX
  # for anything newer to JIT from. On an Orin pass
  # -DCMAKE_CUDA_ARCHITECTURES=87.
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      set(CMAKE_CUDA_ARCHITECTURES 110)  # Jetson Thor, Blackwell
    else()
      set(CMAKE_CUDA_ARCHITECTURES 75-real 86-real 90-virtual)  # x86 dev boxes
    endif()
  endif()
  message(STATUS "CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()

# Common include directories
if(USE_THIRDPARTY_EIGEN)
  include_directories(thirdparty/Eigen)
  message(STATUS "Use thirdparty Eigen")
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
  message(STATUS "Use EIGEN3_INCLUDE_DIR")
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/include>
  $<INSTALL_INTERFACE:include>
)

add_library(cpu_bbs3d
  SHARED
  bbs3d/src/cpu_bbs3d/bbs3d.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp
)

install(TARGETS cpu_bbs3d DESTINATION lib)

if(BUILD_CUDA)
  add_library(gpu_bbs3d
    SHARED
    bbs3d/src/gpu_bbs3d/bbs3d.cu
    bbs3d/src/gpu_bbs3d/calc_score.cu
    bbs3d/src/gpu_bbs3d/voxelmaps.cu
    bbs3d/src/gpu_bbs3d/stream_manager/check_error.cu
    bbs3d/src/gpu_bbs3d/voxelmaps_io.cu
  )

  # nvcc's default standard has moved over releases (CUDA 13 no longer accepts
  # anything below 17); pin it so the Eigen headers see the same one everywhere.
  set_target_properties(gpu_bbs3d PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
  target_link_libraries(gpu_bbs3d PUBLIC CUDA::cudart)

  install(TARGETS gpu_bbs3d DESTINATION lib)
endif()

install(DIRECTORY bbs3d/include/ DESTINATION include)
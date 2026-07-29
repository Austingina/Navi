cmake_minimum_required(VERSION 3.18)
project(test)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU ver" ON)

find_package(cpu_bbs3d REQUIRED)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS filesystem system)

if (OpenMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (BUILD_CUDA)
  find_package(gpu_bbs3d REQUIRED)
  # No .cu here, only the gpu_bbs3d headers, so the toolkit include/link dirs
  # are all that is needed -- no enable_language(CUDA).
  find_package(CUDAToolkit REQUIRED)
endif()


# Common include directories
find_package(PCL REQUIRED)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

# Common libraries
set(COMMON_LIBRARIES
  ${PCL_LIBRARIES}
  yaml-cpp
)

# cpu_test
add_executable(cpu_test
  src/cpu_test.cpp
)

target_link_libraries(cpu_test
  ${COMMON_LIBRARIES}
  ${cpu_bbs3d_LIBRARY}
)

# voxelmaps saver
add_executable(saver
  src/voxelmaps_saver.cpp
)

target_link_libraries(saver
  ${cpu_bbs3d_LIBRARY}
  ${Boost_LIBRARIES}
)

# CUDA
if(BUILD_CUDA)
  # gpu_test
  add_executable(gpu_test
      src/gpu_test.cpp
  )

  target_link_libraries(gpu_test
    ${COMMON_LIBRARIES}
    ${gpu_bbs3d_LIBRARY}
    CUDA::cudart
  )
endif()
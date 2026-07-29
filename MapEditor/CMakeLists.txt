# 3.24 for CMAKE_CUDA_ARCHITECTURES=native. Ubuntu 24.04 ships 3.28.
cmake_minimum_required(VERSION 3.24)

# CUDA is enabled below rather than here, because the architecture has to be
# chosen from CMAKE_SYSTEM_PROCESSOR, which project() is what defines.
project(MapEditor VERSION 1.0.0 LANGUAGES CXX)

# Without this, enabling the CUDA language leaves nvcc on its built-in default
# (sm_52). "native" is not used because nvcc resolves it off the local driver,
# which silently falls back to that default when no driver is visible and errors
# out when the GPU is newer than the toolkit. See the same block in
# ros2_ws/src/3rdparty/3d_bbs/CMakeLists.txt. On an Orin pass
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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ── Dependencies ──────────────────────────────────────────────────────
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CUDAToolkit REQUIRED)

# 3D-BBS (gpu_bbs3d)
find_path(gpu_bbs3d_INCLUDE_DIRS gpu_bbs3d HINTS /usr/local/include)
find_library(gpu_bbs3d_LIBRARY NAMES gpu_bbs3d HINTS /usr/local/lib)
if(NOT gpu_bbs3d_LIBRARY)
    message(FATAL_ERROR "gpu_bbs3d library not found. Build and install 3d_bbs first.")
endif()
message(STATUS "gpu_bbs3d: ${gpu_bbs3d_LIBRARY}")
message(STATUS "gpu_bbs3d includes: ${gpu_bbs3d_INCLUDE_DIRS}")

# PCL 会拉入 VTK 定义，其中可能包含 -isystem 标志，清理掉
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# ── Sources ───────────────────────────────────────────────────────────
set(CORE_SOURCES
    src/core/Types.h
    src/core/Wgs84.h
    src/core/SubMap.h          src/core/SubMap.cpp
    src/core/MapProject.h      src/core/MapProject.cpp
    src/core/VprDatabaseData.h src/core/VprDatabaseData.cpp
    src/core/OsmData.h         src/core/OsmData.cpp
    src/core/SceneModel.h      src/core/SceneModel.cpp
    src/core/Scene.h           src/core/Scene.cpp
)

set(UI_SOURCES
    src/ui/MainWindow.h        src/ui/MainWindow.cpp
    src/ui/PointCloudViewer.h  src/ui/PointCloudViewer.cpp
    src/ui/SubmapPanel.h       src/ui/SubmapPanel.cpp
    src/ui/ScenePanel.h        src/ui/ScenePanel.cpp
    src/ui/RoutePanel.h        src/ui/RoutePanel.cpp
    src/ui/PropertiesPanel.h   src/ui/PropertiesPanel.cpp
    src/ui/AddSubmapDialog.h   src/ui/AddSubmapDialog.cpp
    src/ui/AlignSubmapDialog.h src/ui/AlignSubmapDialog.cpp
    src/ui/PortalEditorDialog.h src/ui/PortalEditorDialog.cpp
    src/ui/BuildVprDatabaseDialog.h src/ui/BuildVprDatabaseDialog.cpp
    src/ui/VprDatabaseInfoDialog.h  src/ui/VprDatabaseInfoDialog.cpp
    src/ui/VprImageViewer.h         src/ui/VprImageViewer.cpp
    src/ui/SceneAlignView.h         src/ui/SceneAlignView.cpp
    src/ui/SceneAlignDialog.h       src/ui/SceneAlignDialog.cpp
)

set(ALGO_SOURCES
    src/algorithms/VoxelGridFilter.h  src/algorithms/VoxelGridFilter.cpp
    src/algorithms/CloudMatcher.h     src/algorithms/CloudMatcher.cpp
    src/algorithms/BBS3DWrapper.h     src/algorithms/BBS3DWrapper.cu
)

set(ALL_SOURCES
    src/main.cpp
    ${CORE_SOURCES}
    ${UI_SOURCES}
    ${ALGO_SOURCES}
)

# ── Target ────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME} ${ALL_SOURCES})

set_source_files_properties(src/algorithms/BBS3DWrapper.cu PROPERTIES LANGUAGE CUDA)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${gpu_bbs3d_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME} PRIVATE ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt5::Widgets
    Qt5::OpenGL
    OpenGL::GL
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    ${gpu_bbs3d_LIBRARY}
    CUDA::cudart
)

# ── Install ───────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

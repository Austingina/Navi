cmake_minimum_required(VERSION 3.16)
project(MapEditor VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ── Dependencies ──────────────────────────────────────────────────────
find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)

# PCL 会拉入 VTK 定义，其中可能包含 -isystem 标志，清理掉
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# ── Sources ───────────────────────────────────────────────────────────
set(CORE_SOURCES
    src/core/Types.h
    src/core/SubMap.h          src/core/SubMap.cpp
    src/core/MapProject.h      src/core/MapProject.cpp
)

set(UI_SOURCES
    src/ui/MainWindow.h        src/ui/MainWindow.cpp
    src/ui/PointCloudViewer.h  src/ui/PointCloudViewer.cpp
    src/ui/SubmapPanel.h       src/ui/SubmapPanel.cpp
    src/ui/PropertiesPanel.h   src/ui/PropertiesPanel.cpp
    src/ui/AddSubmapDialog.h   src/ui/AddSubmapDialog.cpp
    src/ui/AlignSubmapDialog.h src/ui/AlignSubmapDialog.cpp
)

set(ALGO_SOURCES
    src/algorithms/VoxelGridFilter.h  src/algorithms/VoxelGridFilter.cpp
    src/algorithms/CloudMatcher.h     src/algorithms/CloudMatcher.cpp
)

set(ALL_SOURCES
    src/main.cpp
    ${CORE_SOURCES}
    ${UI_SOURCES}
    ${ALGO_SOURCES}
)

# ── Target ────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME} ${ALL_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_directories(${PROJECT_NAME} PRIVATE ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt5::Widgets
    Qt5::OpenGL
    OpenGL::GL
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)

# ── Install ───────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

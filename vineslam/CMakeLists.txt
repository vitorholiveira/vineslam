cmake_minimum_required(VERSION 3.5)
project(vineslam)
include(CheckLanguage)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# General packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

# CUDA packages
check_language(CUDA)
if (CMAKE_CUDA_COMPILER)
        find_package(CUDA REQUIRED)
        set(CUDA_NVCC_FLAGS "-arch=compute_62" CACHE STRING "nvcc flags" FORCE)
        set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
        add_definitions(-DUSE_CUDA)
endif()

# Include header directory
include_directories(include)

# Define C++ libraries
add_library(vineslam
        src/mapping/landmark_mapping.cpp
        src/mapping/landmark_ekf.cpp
        src/mapping/lidar_mapping.cpp
        src/mapping/lidar_mapping_omnidirectional.cpp
        src/mapping/lidar_mapping_livox.cpp
        src/mapping/occupancy_map.cpp
        src/mapping/topological_map.cpp
        src/mapping/elevation_map.cpp
        src/localization/localizer.cpp
        src/localization/pf.cpp
        src/interface/localization_mapping_interface.cpp
        src/map_io/map_writer.cpp
        src/map_io/map_parser.cpp
        src/map_io/topological_map_writer.cpp
        src/map_io/topological_map_parser.cpp
        src/map_io/elevation_map_writer.cpp
        src/map_io/elevation_map_parser.cpp
        include/vineslam/extern/thread_pool.cpp)

target_include_directories(vineslam
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Define CUDA libraries
if (CMAKE_CUDA_COMPILER)
cuda_add_library(gpu_lib STATIC
        kernels/filters/ransac.cu)

# Target and export package dependencies
ament_export_targets(export_gpu_lib HAS_LIBRARY_TARGET)
endif()

ament_export_targets(export_vineslam HAS_LIBRARY_TARGET)
ament_export_dependencies(
        ament_cmake
        Eigen3
        Boost
        OpenCV
)

ament_target_dependencies(vineslam
        ament_cmake
        OpenCV
        ${Eigen3_LIBRARIES}
        ${Boost_LIBRARIES})

if (CMAKE_CUDA_COMPILER)
  ament_target_dependencies(gpu_lib
          OpenCV)
  target_link_libraries(vineslam gpu_lib)
  install(TARGETS gpu_lib
          EXPORT export_gpu_lib
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin)
endif()

install(TARGETS vineslam
        EXPORT export_vineslam
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        RUNTIME DESTINATION bin)

install(DIRECTORY include/vineslam
        DESTINATION include/)

ament_package()

cmake_minimum_required(VERSION 3.0.2)
project(surface_scan_demo)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

if(POLICY CMP0091)
    # https://stackoverflow.com/a/56490614
    cmake_policy(SET CMP0091 NEW)
endif()

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
# This needs cmake_policy(SET CMP0091 NEW)
if (STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)

find_package(Open3D HINTS ${CMAKE_INSTALL_PREFIX}/lib/cmake)

find_package(octomap REQUIRED)

find_package(OpenMP)


include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
include_directories(${OCTOVIS_INCLUDE_DIRS})
link_libraries(${OCTOVIS_LIBRARIES})


add_executable(${PROJECT_NAME}_node src/surface_scan_demo.cpp)

target_link_libraries(${PROJECT_NAME}_node PUBLIC
   Open3D::Open3D
   octomap
   OpenMP::OpenMP_CXX
 )


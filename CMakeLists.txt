cmake_minimum_required(VERSION 3.18)

# Enable C and C++ languages
enable_language(C)
enable_language(CXX) 

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(Shingle_Optimization)

# Find Open3D
find_package(Open3D REQUIRED)

# Include Eigen
include_directories(/usr/include/eigen3)

# Find VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

#  set the MPI_INCLUDE_PATH
find_package(MPI REQUIRED)
include_directories(${MPI_INCLUDE_PATH})

# Find PCL package
find_package(PCL 1.14 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add your source files
add_executable(${PROJECT_NAME} src/main.cpp src/PointCloudProcessor.cpp)

# Link Open3D
target_link_libraries(${PROJECT_NAME} PUBLIC Open3D::Open3D ${PCL_LIBRARIES})



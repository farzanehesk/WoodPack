cmake_minimum_required(VERSION 3.18)
project(WoodPack LANGUAGES C CXX)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Open3D
find_package(Open3D REQUIRED)

# Include Eigen
include_directories(/usr/include/eigen3)

# Find VTK
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    FiltersCore
    IOImage
)

# Find MPI
find_package(MPI REQUIRED)
include_directories(${MPI_INCLUDE_PATH})

# Main executable
add_executable(${PROJECT_NAME}
    src/main.cpp 
    src/PointCloudProcessor.cpp 
    src/GeometryProcessor.cpp
    src/utils.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC Open3D::Open3D ${VTK_LIBRARIES} MPI::MPI_CXX)
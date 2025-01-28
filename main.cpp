#include <iostream>
#include <open3d/Open3D.h>



#include "PointCloudProcessor.hpp"
#include "custom_types.hpp"

int main() {
    std::cout << "Open3D is successfully installed!" << std::endl;


    // load pointcloud
    PC_o3d_ptr pc = std::make_shared<open3d::geometry::PointCloud>();

    PointCloudVisualizer visualizer;
    visualizer.setPointer(pc);
    std::cout << "Point cloud set via setPointer.'\n";

    PointCloudProcessor processor;
    processor.setPointer(pc);

    
    if (processor.loadPointCloud("1694762329.600894788.pcd"))
    {
        std::cout << "Point cloud loaded successfully. '\n";
        visualizer.visualizerPointCloud();
    }
    else
    {
        std::cerr << "Error loading point cloud.'\n";
    }












    return 0;
}
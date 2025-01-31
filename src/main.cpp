#include <iostream>
#include <open3d/Open3D.h>



#include "../include/PointCloudProcessor.hpp"
#include "../include/custom_types.hpp"

int main() {
    

    // Create shared point cloud
    std::shared_ptr<open3d::geometry::PointCloud> pc = std::make_shared<open3d::geometry::PointCloud>();

    // Create PointCloudPerception (which includes Processor + Visualizer)
    PointCloudPerception perception;
    perception.setPointCloud(pc);  

    // Load parameters into PointCloudPerception (which is also a Processor)
    perception.loadParameters("config/config.txt");  

    std::cout << "Voxel size after loading: " << perception.voxel_size_ << std::endl;

    // Load the point cloud 
    if (!perception.loadPointCloud("test.ply")) {
        std::cerr << "Failed to load point cloud'\n";
        return -1;
    }



    // Refine point cloud
    // std::cout << "Refining the point cloud...\n";
    // if (perception.refinePointCloud()) 
    // {
    //     std::cout << "Point cloud refinement completed.\n";
    // } 
    // else 
    // {
    //     std::cerr << "Point cloud refinement failed.\n";
    // }



    // segment the plane
    perception.segmentAndRemovePlane();
    








    return 0;
}
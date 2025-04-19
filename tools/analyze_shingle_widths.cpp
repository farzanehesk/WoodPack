#include <iostream>
#include <fstream>
#include <filesystem>
#include <open3d/Open3D.h>
#include "../include/PointCloudProcessor.hpp"
#include "../include/GeometryProcessor.hpp"
#include "../include/custom_types.hpp"


int main() {


    PointCloudProcessor processor;

    // Provide the path relative to the current directory
    std::string ply_file = "./data/scans/shingle_rows/row1.ply";  // Modify this path as needed
    
    // Load the point cloud
    bool success = processor.loadPointCloud(ply_file, true);  // Flip Z-axis if necessary

    if (!success) {
        std::cerr << "Failed to load point cloud.\n";
        return 1;
    }

    
    
    // Count the number of points in the loaded point cloud
    size_t num_points = processor.getPointCloud()->points_.size();
    
    std::cout << "Loaded point cloud contains " << num_points << " points." << std::endl;





    return 0;


    
}
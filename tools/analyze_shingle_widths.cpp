#include <iostream>
#include <fstream>
#include <filesystem>
#include <open3d/Open3D.h>
#include "../include/PointCloudProcessor.hpp"
#include "../include/GeometryProcessor.hpp"
#include "../include/custom_types.hpp"


int main() {

    
//     PointCloudProcessor processor;

//     // Provide the path relative to the current directory
//     std::string ply_file = "../data/scans/shingle_rows/row1.ply";  // Modify this path as needed
    
//     // Load the point cloud
//     bool success = processor.loadPointCloud(ply_file, true);  // Flip Z-axis if necessary

//     if (!success) {
//         std::cerr << "Failed to load point cloud.\n";
//         return 1;
//     }



//     // After loading, print the number of points in the point cloud using the getter
//     auto point_cloud = processor.getPointCloud();
//     if (point_cloud && !point_cloud->IsEmpty()) {
//         std::cout << "Number of points in the point cloud: " << point_cloud->points_.size() << std::endl;
//     } else {
//         std::cerr << "Point cloud is empty or not loaded properly.\n";
//     }


    return 0;
    // /Shingle_Optimization/build$ ./analyze_shingle_widths

    
}
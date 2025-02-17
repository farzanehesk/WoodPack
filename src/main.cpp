#include <iostream>
#include <open3d/Open3D.h>



#include "../include/PointCloudProcessor.hpp"
#include "../include/custom_types.hpp"
#include "../include/GeometryProcessor.hpp"

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
    if (!perception.loadPointCloud("1694762329.600894788.pcd")) {
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


    // store the original point cloud
    auto original_pc = std::make_shared<open3d::geometry::PointCloud>(*perception.getPointCloud());

    // segment the plane
    perception.segmentAndRemovePlane();
    
    // cluster the elements
    perception.EuclideanClustering();


    // retrieve the clustered pointclouds
    std::vector <PC_o3d_ptr> clusters = perception.getClusters();


    // instantiate geometryprocessor
    GeometryProcessor geom_processor;
    auto bounding_boxes = geom_processor.computeOrientedBoundingBoxes(clusters);

    // 
    //geom_processor.visualizeBoundingBoxes(clusters, bounding_boxes);
    geom_processor.visualizeBoundingBoxesAndOriginalPc(original_pc , bounding_boxes);


    // get a vector of width of all shingles

    // Extract widths of bounding boxes
    auto widths = geom_processor.getWidthsOfBoundingBoxes(bounding_boxes);

    // Print widths
    for (size_t i = 0; i < widths.size(); ++i) {
        std::cout << "Width of bounding box " << i << ": " << widths[i] << std::endl;
    }

    
    // crete a virtual first row of shingles with e=random width 
    std::vector <Rectangle> first_row_shingles = geom_processor.generateRandomRectangles(20);
    geom_processor.printRectangles(first_row_shingles);

    // find the next best element for the second row, from the available shingles







    return 0;
}
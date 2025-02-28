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
    auto dimensions = geom_processor.getDimensionsOfBoundingBoxes(bounding_boxes);



    auto upper_rectangles = geom_processor.extractUpperRectangles(bounding_boxes);
    geom_processor.visualizeRectangles(upper_rectangles , original_pc);
    
    geom_processor.visualizeRectangleEdgesWithLabels(upper_rectangles);

    auto planes = geom_processor.getPlanesFromBoundingBoxes(bounding_boxes, false);
    geom_processor.visualizePlanesOnBoundingBoxes(bounding_boxes, planes,original_pc );

    // // Print dimensions
    for (size_t i = 0; i < dimensions.size(); ++i) {
        std::cout << "Dimensions of bounding box " << i << ": "
                << "Heigt = " << dimensions[i][0] << ", "
                << "Width = " << dimensions[i][1] << ", "
                << "Length = " << dimensions[i][2] << std::endl;
    }


    

    // --- Create and visualize random rectangles ---
    auto random_rectangles = geom_processor.createRandomRectangles(10);  // Create 10 random rectangles
    geom_processor.visualizeRectangles(random_rectangles, original_pc);


    // Create bounding boxes from rectangles

    auto random_bbox = geom_processor.createBoundingBoxFromRectangle(random_rectangles, 0.01);


    // Create the first row of shingles
    auto first_row_of_shingles = geom_processor.arrangeFirstShingleRow(random_bbox , 0.003 , 1);
    geom_processor.visualize_bounding_boxes(first_row_of_shingles);
    
   


    // // find the next best element for the second row, from the available shingles
    






    return 0;
}
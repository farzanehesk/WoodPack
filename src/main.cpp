#include <iostream>
#include <open3d/Open3D.h>



#include "../include/PointCloudProcessor.hpp"
#include "../include/custom_types.hpp"
#include "../include/GeometryProcessor.hpp"

int main() {
    
    // Create shared point cloud (this will be updated later)
    std::shared_ptr<open3d::geometry::PointCloud> pc = std::make_shared<open3d::geometry::PointCloud>();

    // Create PointCloudPerception (which includes Processor + Visualizer)
    PointCloudPerception perception;
    perception.setPointCloud(pc);  

    // Load parameters into PointCloudPerception (which is also a Processor)
    perception.loadParameters("config/config.txt");  

    std::cout << "Voxel size after loading: " << perception.voxel_size_ << std::endl;

    // Load the point clouds from the "data/scans" folder (without merging yet)
    std::string folder = "data/scans";  // specify the folder path
    auto all_point_clouds = perception.loadPointClouds(folder);

    if (all_point_clouds.empty()) {
        std::cerr << "Failed to load point clouds.\n";
        return -1;
    }

    std::cout << "Successfully loaded " << all_point_clouds.size() << " point clouds from folder: " << folder << std::endl;

    // Now process the point clouds (refine, segment, and merge them)
    perception.processPointClouds(all_point_clouds);



    //
    perception.logOriginalPointCloud();
    

    // store the original point cloud
    auto original_pc = std::make_shared<open3d::geometry::PointCloud>(*perception.getPointCloud());


    // cluster the elements
    perception.EuclideanClustering();


    // retrieve the clustered pointclouds
    std::vector <PC_o3d_ptr> clusters = perception.getClusters();


    // instantiate geometryprocessor
    GeometryProcessor geom_processor;
    auto shingles_bbx = geom_processor.computeOrientedBoundingBoxes(clusters);
    //auto shingles_bbx = geom_processor.computeMinimalOrientedBoundingBoxes(clusters);
    geom_processor.VisualizeBoundingBoxesAxis(shingles_bbx);


    //geom_processor.visualizeBoundingBoxes(clusters, bounding_boxes);
    geom_processor.visualizeBoundingBoxesAndOriginalPc(original_pc , shingles_bbx);


    // Extract widths of bounding boxes
    auto dimensions = geom_processor.getDimensionsOfBoundingBoxes(shingles_bbx);



    // auto upper_rectangles = geom_processor.extractUpperRectangles(shingles_bbx);
    // geom_processor.visualizeRectangles(upper_rectangles , original_pc);
    // geom_processor.visualizeRectangleEdgesWithLabels(upper_rectangles);

    auto planes = geom_processor.getPlanesFromBoundingBoxes(shingles_bbx, true);
    geom_processor.visualizePlanesOnBoundingBoxes(shingles_bbx, planes,original_pc );
    

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
    auto random_bbox = geom_processor.createBoundingBoxFromRectangle(random_rectangles, 0.002);

    // Create the first row of shingles with generated random boxes

    double gap = 0.003;       // 3mm gap
    double max_length = 1.0;  // Ensure row is at least 1m long
    double rotation_angle = 15; 
    // 10 degrees in radians

    auto first_row_of_shingles = geom_processor.arrangeFirstShingleRow(random_bbox , gap , max_length ,rotation_angle );
    geom_processor.visualize_bounding_boxes(first_row_of_shingles);
    

    // place the first shingles_bbx on top of the first shingle row, with a specified vertical overlap



   
    // find the next best element for the second row, from the available shingles

    






    return 0;
}
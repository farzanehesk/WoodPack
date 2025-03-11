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
    perception.logOriginalPointCloud();
    

    // store the original point cloud
    auto original_pc = std::make_shared<open3d::geometry::PointCloud>(*perception.getPointCloud());


    // cluster the elements
    perception.EuclideanClustering(false);


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

    // // Print dimensions
    for (size_t i = 0; i < dimensions.size(); ++i) {
        std::cout << "Dimensions of bounding box " << i << ": "
                << "Heigt = " << dimensions[i][0] << ", "
                << "Width = " << dimensions[i][1] << ", "
                << "Length = " << dimensions[i][2] << std::endl;
    }


    // auto upper_rectangles = geom_processor.extractUpperRectangles(shingles_bbx);
    // geom_processor.visualizeRectangles(upper_rectangles , original_pc);
    // geom_processor.visualizeRectangleEdgesWithLabels(upper_rectangles);
    // auto planes = geom_processor.getPlanesFromBoundingBoxes(shingles_bbx, true);
    // geom_processor.visualizePlanesOnBoundingBoxes(shingles_bbx, planes,original_pc );
    


    ///////////////////////////////////////////////////////////
    // First Row
    auto rect_first_row = geom_processor.createRandomRectangles(10 , 0.25);  // Create 10 random rectangles
    geom_processor.visualizeRectangles(rect_first_row, original_pc);

    // Create bounding boxes from rectangles
    auto bbx_first_row = geom_processor.createBoundingBoxFromRectangle(rect_first_row, 0.002);

    // Create the first row of shingles with generated random boxes

    double gap = 0.003;       // 3mm gap
    double max_length = 1.0;  // Ensure row is at least 1m long
    double rotation_angle = 15; 
    // 10 degrees in radians

    auto first_row_of_shingles = geom_processor.arrangeFirstShingleRow(bbx_first_row , gap , max_length ,rotation_angle );
    geom_processor.visualize_bounding_boxes(first_row_of_shingles);
    

    ///////////////////////////////////////////////////////////
    // Second Row
    //     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> first_row_aligned;
    // for (auto& bbox : first_row_of_shingles) {
    //     geom_processor.alignBoxToXYPlane(bbox); // This aligns the bounding box to the XY plane
    //     first_row_aligned.push_back(bbox);
    // }
    // geom_processor.visualize_bounding_boxes(first_row_aligned);

    auto rect_second_row = geom_processor.createRandomRectangles(20 , 0.35 );  // Create 10 random rectangles
    auto bbx_second_row = geom_processor.createBoundingBoxFromRectangle(rect_second_row, 0.002);
    //geom_processor.visualize_bounding_boxes(bbx_second_row);

    auto second_row_sorted = geom_processor.findNextBestShingles(first_row_of_shingles ,bbx_second_row , 0.03 , gap ,max_length  );
    
     auto second_row_of_shingles = geom_processor.arrangeSecondShingleRow(
        first_row_of_shingles,
        second_row_sorted,
        0.003,
        max_length,
        rotation_angle
     );
     
     geom_processor.visualizeShingleRows(first_row_of_shingles ,second_row_of_shingles );


    // place the first shingles_bbx on top of the first shingle row, with a specified vertical overlap
    // 1. list of shingle_bbx 
    // 2. list of first_row_shingles
    // 3. transform shingle_bbx 
    // 4. crop a region around it to find shingles below it , 
    // 5. do intersection and based on approximity, find the shingles previously arranged in those positions
    // 4. calculate its right-edge distance with the upmost right-edge of shingle in the row below
    // 5. search in the list of shingle_bbx  to find the next best object based on width to fullfill criteria to have 3 cm stagger
    // repeat the loop



    // 

   
    // find the next best element for the second row, from the available shingles

    






    return 0;
}
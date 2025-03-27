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

    // std::cout << "Voxel size after loading: " << perception.voxel_size_ << std::endl;

    // Load the point clouds from the "data/scans" folder (without merging yet)
    std::string folder = "data/scans";  // specify the folder path
    std::string export_folder = "data/export";
    // auto all_point_clouds = perception.loadPointClouds(folder);

    // if (all_point_clouds.empty()) {
    //     std::cerr << "Failed to load point clouds.\n";
    //     return -1;
    // }

    // std::cout << "Successfully loaded " << all_point_clouds.size() << " point clouds from folder: " << folder << std::endl;

    // Now process the point clouds (refine, segment, and merge them)
    // perception.processPointClouds(all_point_clouds);
    // perception.logOriginalPointCloud();
    

    // // store the original point cloud
    auto original_pc = std::make_shared<open3d::geometry::PointCloud>(*perception.getPointCloud());


    // // cluster the elements
    // perception.EuclideanClustering(false);


    // // retrieve the clustered pointclouds
    // std::vector <PC_o3d_ptr> clusters = perception.getClusters();


    // // instantiate geometryprocessor
    GeometryProcessor geom_processor;
    // auto shingles_bbx = geom_processor.computeOrientedBoundingBoxes(clusters);
    // //auto shingles_bbx = geom_processor.computeMinimalOrientedBoundingBoxes(clusters);
    // geom_processor.VisualizeBoundingBoxesAxis(shingles_bbx);


    // //geom_processor.visualizeBoundingBoxes(clusters, bounding_boxes);
    // geom_processor.visualizeBoundingBoxesAndOriginalPc(original_pc , shingles_bbx);


    // // Extract widths of bounding boxes
    // auto dimensions = geom_processor.getDimensionsOfBoundingBoxes(shingles_bbx);

    // // // Print dimensions
    // for (size_t i = 0; i < dimensions.size(); ++i) {
    //     std::cout << "Dimensions of bounding box " << i << ": "
    //             << "Heigt = " << dimensions[i][0] << ", "
    //             << "Width = " << dimensions[i][1] << ", "
    //             << "Length = " << dimensions[i][2] << std::endl;
    // }


    // auto upper_rectangles = geom_processor.extractUpperRectangles(shingles_bbx);
    // geom_processor.visualizeRectangles(upper_rectangles , original_pc);
    // geom_processor.visualizeRectangleEdgesWithLabels(upper_rectangles);
    // auto planes = geom_processor.getPlanesFromBoundingBoxes(shingles_bbx, true);
    // geom_processor.visualizePlanesOnBoundingBoxes(shingles_bbx, planes,original_pc );
    


    ///////////////////////////////////////////////////////////
    // First Row
    auto rect_first_row = geom_processor.createRandomRectangles(40 , 0.25);  // Create 10 random rectangles
    geom_processor.visualizeRectangles(rect_first_row, original_pc);

    // Create bounding boxes from rectangles
    auto bbx_first_row = geom_processor.createBoundingBoxFromRectangle(rect_first_row, 0.002);

    /////////////////////////////////////////////////////////////
    // Create the first row of shingles with generated random boxes
    double gap = 0.003;       // 3mm gap
    double max_length = 1.0;  // Ensure row is at least 1m long
    double rotation_angle = 5; 
    // 10 degrees in radians

    auto first_row_of_shingles = geom_processor.arrangeFirstShingleRow(bbx_first_row , gap , max_length ,rotation_angle );
    geom_processor.visualize_bounding_boxes(first_row_of_shingles);
    

    /////////////////////////////////////////////////////////////
    // Create the second row of shingles with generated random boxes
    auto rect_second_row = geom_processor.createRandomRectangles(20 , 0.35 );  // Create 10 random rectangles
    geom_processor.visualizeRectangles(rect_second_row);
    auto bbx_second_row = geom_processor.createBoundingBoxFromRectangle(rect_second_row, 0.002);
    //geom_processor.visualize_bounding_boxes(bbx_second_row);



//////////////////////////////////////////////////////////////////////
    auto second_row_sorted = geom_processor.findNextBestShingles(first_row_of_shingles ,bbx_second_row , 0.03 , gap ,max_length);


    // auto first_box_second_row = geom_processor.alignAndShiftFirstBox(first_row_of_shingles ,bbx_second_row , gap , max_length , 0 );
    // geom_processor.visualizeShingleRows(first_row_of_shingles ,{first_box_second_row} );


   
    
    auto second_row_of_shingles = geom_processor.arrangeShingleRow(
        first_row_of_shingles,
        second_row_sorted,
        0.003,
        max_length,
        rotation_angle,
        0);

    geom_processor.visualizeShingleRows(first_row_of_shingles ,second_row_of_shingles );
///////////////////////////////////////////////////////////////////////////////////////

    //
    std::cout << "starting third and forth row'\n\n\n\n" ;
    auto rect_third_and_forth_row = geom_processor.createRandomRectangles(30 , 0.45 );  // Create 10 random rectangles
    geom_processor.visualizeRectangles(rect_third_and_forth_row);
    auto bbx_third_and_forth_row = geom_processor.createBoundingBoxFromRectangle(rect_third_and_forth_row, 0.002);
    

    //auto third_forth_row_sorted = geom_processor.findNextBestShinglesForMultipleRows(second_row_of_shingles , bbx_third_and_forth_row , 3 , 0.03 , gap , max_length ) ;
    auto third_forth_row_sorted = geom_processor.findNextBestShinglesForMultipleRows(second_row_of_shingles , bbx_third_and_forth_row ,3 ,  0.03 , gap , max_length);




    // // Now pass it to the visualization function
    geom_processor.visualizeAllShingleRows(third_forth_row_sorted);











    
    



    // export
    geom_processor.exportBoundingBoxes(first_row_of_shingles ,export_folder , "first_row_" );
    geom_processor.exportBoundingBoxes(second_row_of_shingles ,export_folder , "second_row_" );
    geom_processor.exportBoundingBoxes(third_forth_row_sorted[0] ,export_folder , "third_row_" );



    return 0;
}


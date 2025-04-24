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
    std::string folder = "data/scans/shingles";  // specify the folder path
    std::string export_folder = "data/export";
    auto all_point_clouds = perception.loadPointClouds(folder);

    if (all_point_clouds.empty()) {
        std::cerr << "Failed to load point clouds.\n";
        return -1;
    }

    std::cout << "Successfully loaded " << all_point_clouds.size() << " point clouds from folder: " << folder << std::endl;

    //// Now process the point clouds (refine, segment, and merge them)
    perception.processPointClouds(all_point_clouds);
    perception.logOriginalPointCloud();
    




    // // store the original point cloud
    auto original_pc = std::make_shared<open3d::geometry::PointCloud>(*perception.getPointCloud());


    // // cluster the elements
    perception.EuclideanClustering(false);


    // // retrieve the clustered pointclouds
    std::vector <PC_o3d_ptr> clusters = perception.getClusters();


    /// former method that worked
    // // instantiate geometryprocessor
    GeometryProcessor geom_processor;
    // auto shingles_bbx = geom_processor.computeOrientedBoundingBoxes(clusters);
    // geom_processor.VisualizeBoundingBoxesAxis(shingles_bbx);
    ///

/// new method
    auto box_cloud_pairs = geom_processor.computeOrientedBoundingBoxesWithClouds(clusters , true);
    // extract just the boxes for sorting/arranging
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> shingle_ptrs;
    for (const auto& [box, _] : box_cloud_pairs)
        shingle_ptrs.push_back(std::make_shared<open3d::geometry::OrientedBoundingBox>(box));

/////

    std::vector<open3d::geometry::OrientedBoundingBox> boxes;
    boxes.reserve(shingle_ptrs.size());

    for (const auto& ptr : shingle_ptrs) {
        if (ptr) {
            boxes.push_back(*ptr);  // Dereference and copy the value
        }
    }

    // Now pass it to the function
    geom_processor.VisualizeBoundingBoxesAxis(boxes);

/////




    // //geom_processor.visualizeBoundingBoxes(clusters, bounding_boxes);
    // geom_processor.visualizeBoundingBoxesAndOriginalPc(original_pc , shingles_bbx);


    // // // Extract widths of bounding boxes
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
    auto bbx_first_row = geom_processor.createBoundingBoxes(20 , 0.25 , false);
    geom_processor.visualize_bounding_boxes(bbx_first_row);

    /////////////////////////////////////////////////////////////
    // Create the first row of shingles with generated random boxes
    double gap = 0.003;       // 3mm gap
    double max_length = 1.0;  // Ensure row is at least 1m long
    double rotation_angle = 9; 
    // 10 degrees in radians

    auto first_row_of_shingles = geom_processor.arrangeFirstShingleRow(bbx_first_row , gap , max_length ,rotation_angle );
    geom_processor.visualize_bounding_boxes(first_row_of_shingles);
    

    /////////////////////////////////////////////////////////////
    //Second row 
    auto bbx_second_row = geom_processor.createBoundingBoxes(45 , 0.35, false );  // Create 10 random rectangles
    geom_processor.visualize_bounding_boxes(bbx_second_row);
    //
    auto second_row_sorted = geom_processor.findNextBestShingles(first_row_of_shingles ,bbx_second_row , 0.03 , gap ,max_length, true);
    //
    auto second_row_of_shingles = geom_processor.arrangeShingleRow(
        first_row_of_shingles,
        second_row_sorted,
        0.003,
        max_length,
        rotation_angle,
        0);
    //
    geom_processor.visualizeShingleRows(first_row_of_shingles ,second_row_of_shingles );


    ///////////////////////////////////////////////////////////////////////////////////////
    // 3-4-5-6 th rows
    std::cout << "starting third and forth row'\n\n\n\n" ;
    auto bbx_third_and_forth_row = geom_processor.createBoundingBoxes(45 , 0.45, false );  // Create 10 random rectangles
    // std::cout << "Number of RECTANGLES in rect_third_and_forth_row: " << bbx_third_and_forth_row.size() << std::endl;


    /// former method that worked
    // // Convert std::vector<OrientedBoundingBox> to std::vector<std::shared_ptr<OrientedBoundingBox>>
    // std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> shingle_ptrs;
    // for (const auto& box : shingles_bbx) {
    // //     shingle_ptrs.push_back(std::make_shared<open3d::geometry::OrientedBoundingBox>(box));
    // }
    // geom_processor.visualize_bounding_boxes(shingle_ptrs);
    ///




    auto sorted_boxes = geom_processor.findNextBestShinglesForMultipleRows(second_row_of_shingles , shingle_ptrs ,5  ,  0.03 , gap , max_length);
    geom_processor.visualizeAllShingleRows(sorted_boxes);

    //
   auto arranged_boxes = geom_processor.arrangeMultipleShingleRows(second_row_of_shingles ,sorted_boxes , gap , max_length , rotation_angle , 0.02 , -0.11 );
   std::cout << "Number of rows in third_forth_row_: " << arranged_boxes.size() << std::endl;


    ////////////////////////////////////
    auto arranged_clouds = geom_processor.alignPointCloudsToArrangedBoxes(arranged_boxes, box_cloud_pairs);
    std::cout << "Number of arranged shingles: " << arranged_clouds.size() << std::endl;
    geom_processor.visualizePointClouds(arranged_clouds , nullptr);

    ///////////////////////////////////////////////////////////////////////////////////////
    // 7 - 8 - 9 - 10: 



    // Create a vector to hold all the rows
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> combined_rows;

    // Add the first and second rows to the combined rows
    combined_rows.push_back(first_row_of_shingles);
    combined_rows.push_back(second_row_of_shingles);

    // Add all rows from the third_forth_row to the combined rows
    for (const auto& row : arranged_boxes) {
        combined_rows.push_back(row);
    }

    // for (const auto& row : third_forth_row_sorted) {
    //     combined_rows.push_back(row);
    // }



        // Load the point cloud
    auto sub_structure = perception.loadPointCloud("data/scans/substructure.ply" , false);

    if (sub_structure) {
        // Visualize the stored point cloud
        perception.visualizerPointCloud();
    } else {
        std::cerr << "Failed to load or process point cloud." << std::endl;
    }
    auto sub_structure_pc = perception.getPointCloud();


    std::cout << "Number of rows in combined_rows: " << combined_rows.size() << std::endl;

    geom_processor.visualizeAllShingleRows(combined_rows);
    geom_processor.visualizeShingleMeshes(combined_rows ,sub_structure_pc);
    geom_processor.visualizePointClouds(arranged_clouds ,sub_structure_pc );





    // 1. third forth row z overlap : done 
    // 2. vertical overlaps : done
    // 5. create mesh to visualize boxes : done
    // 3. insert the substrycture point cloud under arranged shingles : done



    // 4. check the angles based on the prototype built by craftspeople : done
    // 5. use the actual shingles for arrangement : done
    // 
    // 6. import the substructure designed 3d model for the process
    // 7. correct the thickness of shingles bbx to be consistent: done
    // 8. check the length limit
    // 9. see if we can get shorter shingles
    

    


    



    




    
    



//     // export
    geom_processor.exportBoundingBoxes(first_row_of_shingles ,export_folder ,{0, 0, 1} ,  "first_row_" );
    geom_processor.exportBoundingBoxes(second_row_of_shingles ,export_folder ,{0, 1, 0}, "second_row_" );
    geom_processor.exportBoundingBoxes(arranged_boxes[0] ,export_folder ,{1, 0, 0}, "third_row_" );
    geom_processor.exportBoundingBoxes(arranged_boxes[1] ,export_folder , {0, 0, 1} ,"forth_row_" );
    geom_processor.exportBoundingBoxes(arranged_boxes[2] ,export_folder ,{0, 1, 0}, "fifth_row_" );
    geom_processor.exportBoundingBoxes(arranged_boxes[3] ,export_folder ,{1, 0, 0}, "sixth_row_" );




    return 0;
}


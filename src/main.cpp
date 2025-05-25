#include <iostream>
#include <open3d/Open3D.h>



#include "../include/PointCloudProcessor.hpp"
#include "../include/custom_types.hpp"
#include "../include/GeometryProcessor.hpp"
#include <filesystem>

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




    ////////////////////////////////////////


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
    double max_length = 0.80;  // Ensure row is at least 1m long
    double rotation_angle = 9; 
    // 10 degrees in radians

    auto first_row_of_shingles = geom_processor.arrangeFirstShingleRow(bbx_first_row , gap , max_length ,rotation_angle );
    geom_processor.visualize_bounding_boxes(first_row_of_shingles);
    



    /////////////////////////////////////////////////////////////
    //Second row 
    auto bbx_second_row = geom_processor.createBoundingBoxes(45 , 0.35, false );  // Create 10 random rectangles
    geom_processor.visualize_bounding_boxes(bbx_second_row);
    //
    auto second_row_sorted = geom_processor.findNextBestShingles(first_row_of_shingles ,bbx_second_row , 0.03 , gap ,max_length, false);
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




    auto sorted_boxes = geom_processor.findNextBestShinglesForMultipleRows(second_row_of_shingles , shingle_ptrs ,10  ,  0.03 , gap , max_length);
    geom_processor.visualizeAllShingleRows(sorted_boxes);

    //
   auto arranged_boxes = geom_processor.arrangeMultipleShingleRows(second_row_of_shingles ,sorted_boxes , gap , max_length , rotation_angle , 0.02 , -0.11 );
   std::cout << "Number of rows in third_forth_row_: " << arranged_boxes.size() << std::endl;


    ////////////////////////////////////
    auto tuple_pairs = geom_processor.convertPairsToTuples(box_cloud_pairs);
    auto result = geom_processor.alignPointCloudsToArrangedBoxes(arranged_boxes, tuple_pairs);
    auto arranged_clouds = result.first;  // Extract the aligned clouds (arranged_clouds)
    auto box_to_shingle_id = result.second;  // Extract the shingle ID mapping (optional)    
    std::cout << "Number of arranged shingles: " << arranged_clouds.size() << std::endl;
    geom_processor.visualizePointClouds(arranged_clouds , nullptr , true, "output/arranged clouds.png");



    // test this
auto full_cloud = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& [box, cloud] : box_cloud_pairs) {
        *full_cloud += *cloud; // Merge all original clouds
    }
    if (!full_cloud->HasColors()) {
        full_cloud->colors_.resize(full_cloud->points_.size(), Eigen::Vector3d(0.5, 0.5, 0.5)); // Gray
    }

    // Construct corresponding shingles (as before)
    std::vector<PC_o3d_ptr> corresponding_shingles;
    corresponding_shingles.reserve(arranged_clouds.size());
    for (const auto& row : arranged_boxes) {
        for (const auto& arranged_box_ptr : row) {
            auto it = box_to_shingle_id.find(arranged_box_ptr);
            if (it == box_to_shingle_id.end()) {
                std::cerr << "Warning: No shingle ID found for arranged box.\n";
                continue;
            }
            int shingle_id = it->second;
            if (shingle_id >= 0 && shingle_id < static_cast<int>(box_cloud_pairs.size())) {
                corresponding_shingles.push_back(box_cloud_pairs[shingle_id].second);
            } else {
                std::cerr << "Warning: Invalid shingle ID " << shingle_id << " for arranged box.\n";
            }
        }
    }

    // Visualize and export corresponding shingles one by one
    if (!corresponding_shingles.empty()) {
        std::cout << "Visualizing and exporting corresponding original shingles one by one...\n";
        std::filesystem::create_directories("output/corresponding_shingles");
        std::filesystem::create_directories("output/corresponding_shingles/ply");

        for (size_t i = 0; i < corresponding_shingles.size(); ++i) {
            // Visualize the single corresponding shingle with the full cloud in the background (top-down view applied)
            std::string screenshot_path = "output/corresponding_shingles/shingle_" + std::to_string(i) + ".png";
            std::cout << "Visualizing shingle " << i << " (saving to " << screenshot_path << ")\n";
            geom_processor.visualizeSingleShingle(corresponding_shingles[i], full_cloud, true, screenshot_path);

            // Export the single corresponding shingle to a PLY file
            std::vector<PC_o3d_ptr> single_shingle_for_export = {corresponding_shingles[i]};
            geom_processor.exportPointClouds(single_shingle_for_export, "output/corresponding_shingles/ply", "shingle_" + std::to_string(i) + "_");
        }
    } else {
        std::cerr << "No corresponding shingles to visualize.\n";
    }
    ////////

    ///////////////////////////////////////////////////////////////////////////////////////



    // Create a vector to hold all the rows
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> combined_rows;

    // Add the first and second rows to the combined rows
    combined_rows.push_back(first_row_of_shingles);
    combined_rows.push_back(second_row_of_shingles);

    // Add all rows from the third_forth_row to the combined rows
    for (const auto& row : arranged_boxes) {
        combined_rows.push_back(row);
    }


        // Load the point cloud
    auto sub_structure_pc = perception.loadPointCloud("data/scans/structure.ply", false);

    if (sub_structure_pc && !sub_structure_pc->IsEmpty()) {
        // Visualize the stored point cloud
        //perception.visualizerPointCloud(); // This still uses pc_ptr_, but sub_structure_pc is independent

        // Check colors before passing
        if (sub_structure_pc->HasColors()) {
            std::cout << "sub_structure_pc has " << sub_structure_pc->colors_.size() << " colors." << std::endl;
        } else {
            std::cerr << "sub_structure_pc has no colors." << std::endl;
        }
        //geom_processor.visualizePointClouds(arranged_clouds, sub_structure_pc , true, "output/sh2.png");



        //test this
            // Visualize arranged clouds incrementally with substructure
        if (!arranged_clouds.empty() && sub_structure_pc) {
            std::vector<PC_o3d_ptr> accumulated_clouds; // To accumulate clouds over iterations

            for (size_t i = 0; i < arranged_clouds.size(); ++i) {
                // Add the current cloud to the accumulated list
                accumulated_clouds.push_back(arranged_clouds[i]);

                // Visualize the accumulated clouds with the substructure
                std::string png_path = "output/assembly/sh_" + std::to_string(i) + ".png";
                std::cout << "Visualizing up to cloud " << i << " (saving to " << png_path << ")\n";
                Eigen::Vector3d custom_lookat(0.0, 0.0, 0.0);
                if (sub_structure_pc && !sub_structure_pc->points_.empty()) {
                    auto bbox = sub_structure_pc->GetAxisAlignedBoundingBox();
                    custom_lookat = bbox.GetCenter();
                }
                geom_processor.visualizePointClouds(
                    accumulated_clouds,
                    sub_structure_pc,
                    true,
                    png_path
                );

                //


            }
        } else {
            std::cerr << "No arranged clouds or substructure to visualize.\n";
        }


    // Visualize shingle meshes incrementally by box with substructure, preserving row colors
        if (!combined_rows.empty() && sub_structure_pc) {
            // Count total number of boxes for iteration
            size_t total_boxes = 0;
            for (const auto& row : combined_rows) {
                total_boxes += row.size();
            }

            // Initialize accumulated_rows with the same number of rows as combined_rows
            std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> accumulated_rows(combined_rows.size());
            std::filesystem::create_directories("output");

            size_t box_counter = 0; // To track total boxes processed
            for (size_t row_idx = 0; row_idx < combined_rows.size() && box_counter < total_boxes; ++row_idx) {
                const auto& row = combined_rows[row_idx];
                for (size_t box_idx = 0; box_idx < row.size() && box_counter < total_boxes; ++box_idx) {
                    // Add the current box to its corresponding row in accumulated_rows
                    accumulated_rows[row_idx].push_back(row[box_idx]);

                    // Visualize the accumulated rows with the substructure
                    std::string png_path = "output/assembly/box_" + std::to_string(box_counter) + ".png";
                    std::cout << "Visualizing up to box " << box_counter << " (saving to " << png_path << ")\n";
                    geom_processor.visualizeShingleMeshes(
                        accumulated_rows,
                        sub_structure_pc,
                        true,
                        png_path
                    );

                    box_counter++;
                }
            }
        } else {
            std::cerr << "No combined rows or substructure to visualize.\n";
        }
        
        }
    auto sub_structure_pc_copy = std::make_shared<open3d::geometry::PointCloud>(*sub_structure_pc);



    std::cout << "Number of rows in combined_rows: " << combined_rows.size() << std::endl;

    geom_processor.visualizeAllShingleRows(combined_rows);
    //geom_processor.visualizeShingleMeshes(combined_rows ,sub_structure_pc  );
    geom_processor.visualizeShingleMeshes(combined_rows, sub_structure_pc_copy, true, "output/my_shingles_visualization.png");
    geom_processor.visualizePointClouds(arranged_clouds ,sub_structure_pc, false, "output/my_shingles_visualization.png" );

    //


    geom_processor.visualizeArrangedCloudCorrespondence(all_point_clouds, arranged_clouds, arranged_boxes, box_cloud_pairs);









    


    



    




    
    



//     // export
    geom_processor.exportBoundingBoxes(first_row_of_shingles ,export_folder ,{0, 0, 1} ,  "first_row_" );
    geom_processor.exportBoundingBoxes(second_row_of_shingles ,export_folder ,{0, 1, 0}, "second_row_" );
    geom_processor.exportBoundingBoxes(arranged_boxes[0] ,export_folder ,{1, 0, 0}, "third_row_" );
    geom_processor.exportBoundingBoxes(arranged_boxes[1] ,export_folder , {0, 0, 1} ,"forth_row_" );
    geom_processor.exportBoundingBoxes(arranged_boxes[2] ,export_folder ,{0, 1, 0}, "fifth_row_" );
    geom_processor.exportBoundingBoxes(arranged_boxes[3] ,export_folder ,{1, 0, 0}, "sixth_row_" );

    geom_processor.exportPointClouds(arranged_clouds ,export_folder , "test_" );
    


    return 0;
}


#include <iostream>
#include <open3d/Open3D.h>
#include "../include/PointCloudProcessor.hpp"
#include "../include/custom_types.hpp"
#include "../include/GeometryProcessor.hpp"
#include "../include/utils.hpp"
#include <filesystem>

int main() {
    // Initialize point cloud and perception
    auto pc = std::make_shared<open3d::geometry::PointCloud>();
    PointCloudPerception perception;
    perception.setPointCloud(pc);
    perception.loadParameters("config/config.txt");

    // Load point clouds
    std::string folder = "data/scans/shingles";
    std::string export_folder = "data/export";
    auto all_point_clouds = perception.loadPointClouds(folder);
    if (all_point_clouds.empty()) {
        std::cerr << "Failed to load point clouds.\n";
        return -1;
    }
    std::cout << "Loaded " << all_point_clouds.size() << " point clouds from " << folder << std::endl;

    // Process point clouds
    perception.processPointClouds(all_point_clouds);
    perception.logOriginalPointCloud();
    auto original_pc = std::make_shared<open3d::geometry::PointCloud>(*perception.getPointCloud());

    // Cluster point clouds
    perception.EuclideanClustering(false);
    auto clusters = perception.getClusters();

    // Compute bounding boxes with clouds
    GeometryProcessor geom_processor;
    auto box_cloud_pairs = geom_processor.computeOrientedBoundingBoxesWithClouds(clusters, true);

    // Extract bounding boxes
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> shingle_ptrs;
    for (const auto& [box, _] : box_cloud_pairs) {
        shingle_ptrs.push_back(std::make_shared<open3d::geometry::OrientedBoundingBox>(box));
    }

    std::vector<open3d::geometry::OrientedBoundingBox> boxes;
    boxes.reserve(shingle_ptrs.size());
    for (const auto& ptr : shingle_ptrs) {
        if (ptr) {
            boxes.push_back(*ptr);
        }
    }

    // Export and visualize bounding boxes
    Eigen::Vector3d color(0.0, 0.0, 1.0); // Blue
    constexpr double scale_factor = 1000.0; // Meters to mm
    std::string prefix = "boundingbox_";
    geom_processor.exportBoundingBoxesAsPolylines(boxes, export_folder, color, prefix, scale_factor);
    geom_processor.VisualizeBoundingBoxesAxis(boxes);

    // First Row Arrangement (25 cm)
    constexpr double gap = 0.003; // 3mm
    constexpr double max_length = 0.80; // 800mm
    constexpr double rotation_angle = 8; // Degrees
    auto bbx_first_row = geom_processor.createBoundingBoxes(20, 0.25, false);
    geom_processor.visualize_bounding_boxes(bbx_first_row);
    auto first_row_of_shingles = geom_processor.arrangeFirstShingleRow(bbx_first_row, gap, max_length, rotation_angle);
    geom_processor.visualize_bounding_boxes(first_row_of_shingles);

    // Second Row Arrangement (35 cm)
    auto bbx_second_row = geom_processor.createBoundingBoxes(45, 0.35, false);
    geom_processor.visualize_bounding_boxes(bbx_second_row);
    auto second_row_sorted = geom_processor.findNextBestShingles(first_row_of_shingles, bbx_second_row, 0.03, gap, max_length, false);
    auto second_row_of_shingles = geom_processor.arrangeShingleRow(first_row_of_shingles, second_row_sorted, gap, max_length, rotation_angle, 0);
    geom_processor.visualizeShingleRows(first_row_of_shingles, second_row_of_shingles);

    // Third Row Up to Second-to-Last
    std::cout << "Starting third and fourth rows\n";
    auto bbx_third_and_fourth_row = geom_processor.createBoundingBoxes(45, 0.45, false);
    auto sorted_boxes = geom_processor.findNextBestShinglesForMultipleRows(second_row_of_shingles, shingle_ptrs, 11, 0.03, gap, max_length);
    geom_processor.visualizeAllShingleRows(sorted_boxes);
    auto arranged_boxes = geom_processor.arrangeMultipleShingleRows(second_row_of_shingles, sorted_boxes, gap, max_length, rotation_angle, 0.02, -0.11);
    std::cout << "Number of rows in third_and_fourth_row: " << arranged_boxes.size() << std::endl;

    // Last Two Rows
    std::cout << "Starting last rows\n";
    auto bbx_last_rows = geom_processor.createBoundingBoxes(20, 0.35, false);
    auto last_vector = arranged_boxes.back();
    auto sorted_boxes_last_rows = geom_processor.findNextBestShinglesForMultipleRows(last_vector, bbx_last_rows, 3, 0.03, gap, max_length);
    geom_processor.visualizeAllShingleRows(sorted_boxes_last_rows);
    auto arranged_boxes_last_rows = geom_processor.arrangeLastTwoShingleRows(last_vector, sorted_boxes_last_rows, gap, max_length, rotation_angle, -0.11);
    std::cout << "Number of rows in last_rows: " << arranged_boxes_last_rows.size() << std::endl;

    // Combine All Rows
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> combined_rows;
    combined_rows.push_back(first_row_of_shingles);
    combined_rows.push_back(second_row_of_shingles);
    for (const auto& row : arranged_boxes) {
        combined_rows.push_back(row);
    }
    for (const auto& row : arranged_boxes_last_rows) {
        combined_rows.push_back(row);
    }
    geom_processor.visualizeAllShingleRows(combined_rows);

    // Align Shingle Point Clouds to Arranged Boxes
    auto tuple_pairs = geom_processor.convertPairsToTuples(box_cloud_pairs);
    auto result = geom_processor.alignPointCloudsToArrangedBoxes(arranged_boxes, tuple_pairs);
    auto arranged_clouds = result.first;
    auto box_to_shingle_id = result.second;
    if (arranged_clouds.empty()) {
        std::cerr << "No arranged clouds generated.\n";
        return -1;
    }
    std::cout << "Number of arranged shingles: " << arranged_clouds.size() << std::endl;
    geom_processor.visualizePointClouds(arranged_clouds, nullptr, true, "output/arranged_clouds.png");

    // Construct and Visualize Corresponding Shingles
    auto full_cloud = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& [box, cloud] : box_cloud_pairs) {
        *full_cloud += *cloud;
    }
    if (!full_cloud->HasColors()) {
        full_cloud->colors_.resize(full_cloud->points_.size(), Eigen::Vector3d(0.5, 0.5, 0.5)); // Gray
    }

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

    if (!corresponding_shingles.empty()) {
        geom_processor.visualizeAndExportCorrespondingShingles(corresponding_shingles, full_cloud);
    } else {
        std::cerr << "No corresponding shingles to visualize.\n";
    }

    // Load and Visualize Sub-Structure
    auto sub_structure_pc = perception.loadPointCloud("data/scans/structure.ply", false);
    if (!sub_structure_pc || sub_structure_pc->IsEmpty()) {
        std::cerr << "Failed to load sub-structure point cloud.\n";
        return -1;
    }
    auto sub_structure_pc_copy = std::make_shared<open3d::geometry::PointCloud>(*sub_structure_pc);
    geom_processor.visualizeShingleMeshes(combined_rows, sub_structure_pc_copy, true, "output/my_shingles_boxes_visualization.png");
    geom_processor.visualizePointClouds(arranged_clouds, sub_structure_pc, true, "output/my_shingles_visualization.png");
    geom_processor.visualizeArrangedCloudCorrespondence(all_point_clouds, arranged_clouds, arranged_boxes, box_cloud_pairs);

    // Export Results
    auto colors = generateColorGradient(combined_rows.size());
    for (size_t i = 0; i < combined_rows.size(); ++i) {
        std::string prefix = "row_" + std::to_string(i) + "_";
        geom_processor.exportBoundingBoxes(combined_rows[i], export_folder, colors[i], prefix);
    }
    geom_processor.exportPointClouds(arranged_clouds, export_folder, "test_");

    return 0;
}
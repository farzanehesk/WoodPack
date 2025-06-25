#include "iostream"
#include <fstream>
#include <string>
#include <filesystem>

#include "../include/PointCloudProcessor.hpp"

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
// PointCloudProcessor

//constructor fro PointCloudProcessor
PointCloudProcessor::PointCloudProcessor ()
: voxel_size_(0.004), nb_neighbors_(20), std_ratio_(2.0), verbose_(false)
{

    std::cout << "PointCloudProcessor initialized." << '\n';

}


// destructor
PointCloudProcessor::~PointCloudProcessor()
{
    std::cout << "PPointCloudProcessor destroyed. \n" ;

}



//1. Method to check if the point cloud is valid
bool PointCloudProcessor::checkPointCloud() const 
{
    if (pc_ptr_ == nullptr || pc_ptr_->IsEmpty()) 
    {
        std::cerr << "Error: Point cloud is empty!" << std::endl;
        return false;
    }

    auto points = pc_ptr_->points_;
    Eigen::Vector3d min_point = points[0];
    Eigen::Vector3d max_point = points[0];
    for (const auto& point : points) {
        min_point = min_point.cwiseMin(point);
        max_point = max_point.cwiseMax(point);
    }

    if ((max_point - min_point).norm() < voxel_size_) {
        std::cerr << "Error: Point cloud scale is too small for the given voxel size!" << std::endl;
        return false;
    }

    return true;
}


// 2. Method to log a message
void PointCloudProcessor::log(const std::string& message) const {
    if (verbose_) { // Assuming `verbose_` is a member variable
        std::cout << message << std::endl;
    }
}

// 3. Method to log the original point cloud size
void PointCloudProcessor::logOriginalPointCloud() const {
    log("Number of points in original point cloud: " + std::to_string(pc_ptr_->points_.size()));
}

// 4. Method to log the point cloud size
void PointCloudProcessor::logPointCloudSize(const std::string& stage, const std::shared_ptr<open3d::geometry::PointCloud>& cloud) const 
{
if (cloud) 
{
    log("Number of points in " + stage + " point cloud: " + std::to_string(cloud->points_.size()));
} 
else 
{
    log("Point cloud is empty at stage: " + stage);
}
}



// 5. Load parameters 
void PointCloudProcessor::loadParameters(const std::string& config_file_name = "config/config.txt") {
    std::ifstream config_file(config_file_name);
    
    if (!config_file.is_open()) {
        std::cerr << "Error: Could not open config file." << std::endl;
        return;
    }

    std::string line;
    while (std::getline(config_file, line)) {
        size_t delimiter_pos = line.find('=');
        if (delimiter_pos != std::string::npos) {
            std::string key = line.substr(0, delimiter_pos);
            std::string value = line.substr(delimiter_pos + 1);

            // Simple assignment for known parameters
            if (key == "verbose") {
                verbose_ = (value == "true");
            }
            else if (key == "voxel_size") {
                voxel_size_ = std::stod(value);  // Convert string to double
                std::cout << "Loaded voxel size: " << voxel_size_ << std::endl; // Debugging
            }

            else if (key == "nb_neighbors") {
                nb_neighbors_ = std::stoi(value);  // Convert string to int
            }
            else if (key == "std_ratio") {
                std_ratio_ = std::stod(value);  // Convert string to double
            }

            else if (key == "ransac_n") {
                ransac_n_ = std::stoi(value);
            }
            else if (key == "num_iterations") {
                num_iterations_ = std::stoi(value);
            }
            // ✅ New Clustering Parameters
            else if (key == "cluster_tolerance") {
                cluster_tolerance_ = std::stod(value);
                std::cout << "Loaded cluster_tolerance: " << cluster_tolerance_ << std::endl;
            } else if (key == "min_cluster_size") {
                min_cluster_size_ = std::stoi(value);
                std::cout << "Loaded min_cluster_size: " << min_cluster_size_ << std::endl;
            }

        }
    }

    config_file.close();
}



// 6. Method to load a point cloud from a file within the 'data' directory
// bool PointCloudProcessor::loadPointCloud(const std::string& filename)
// {
//     if (!pc_ptr_) {
//         pc_ptr_ = std::make_shared<open3d::geometry::PointCloud>();
//     }
//     std::string filepath="data/"+filename ;
//     if (open3d::io::ReadPointCloud(filepath , *pc_ptr_))
//     {
//         std::cout << "Successfully loaded point cloud from " <<filepath <<'\n';
        
//         // Visualize the loaded point cloud
//         if (!pc_ptr_->IsEmpty()) {
//             std::cout << "Visualizing the loaded point cloud...\n";
//             open3d::visualization::DrawGeometries({pc_ptr_}, "Loaded Point Cloud", 800, 600);
//         } 
//         else {
//             std::cerr << "Loaded point cloud is empty.\n";
//         }

//         return true;

//     }
//     else
//     {
//         std::cerr << "Failed to load point cloud from " << filepath << '\n';
//         return false;
//     }

// }

// bool PointCloudProcessor::loadPointCloud(const std::string& filename)
// {
//     if (!pc_ptr_) {
//         pc_ptr_ = std::make_shared<open3d::geometry::PointCloud>();
//     }
//     std::string filepath = "data/" + filename;
    
//     if (open3d::io::ReadPointCloud(filepath, *pc_ptr_))
//     {
//         std::cout << "Successfully loaded point cloud from " << filepath << '\n';

//         // Check the scale of the point cloud
//         if (!pc_ptr_->IsEmpty()) {
//             // Get the extent (bounding box size) of the point cloud
//             auto bbox = pc_ptr_->GetAxisAlignedBoundingBox();
//             Eigen::Vector3d min = bbox.min_bound_; // Access the min_bound_ point
//             Eigen::Vector3d max = bbox.max_bound_; // Access the max_bound_ point

//             Eigen::Vector3d extent = max - min;  // Calculate the extent
            
//             std::cout << "Point cloud extent: " << extent.transpose() << std::endl;

//             // If the extent is in millimeters or centimeters, scale to meters
//             double scale_factor = 1.0;
//             if (extent.x() > 1000 || extent.y() > 1000 || extent.z() > 1000) {
//                 // Likely in millimeters, scale to meters
//                 scale_factor = 0.001;
//                 std::cout << "Point cloud scale is in millimeters. Scaling to meters." << std::endl;
//             } 
//             else if (extent.x() > 100 || extent.y() > 100 || extent.z() > 100) {
//                 // Likely in centimeters, scale to meters
//                 scale_factor = 0.01;
//                 std::cout << "Point cloud scale is in centimeters. Scaling to meters." << std::endl;
//             }

//             // Scale the points by the determined factor
//             for (auto& point : pc_ptr_->points_) {
//                 point *= scale_factor;
//             }

//             // Visualize the scaled point cloud
//             std::cout << "Visualizing the scaled point cloud...\n";
//             open3d::visualization::DrawGeometries({pc_ptr_}, "Loaded Point Cloud (Scaled)", 800, 600);
//         }
//         else {
//             std::cerr << "Loaded point cloud is empty.\n";
//         }

//         return true;
//     }
//     else {
//         std::cerr << "Failed to load point cloud from " << filepath << '\n';
//         return false;
//     }
// }



// bool PointCloudProcessor::loadPointCloud(const std::string& filename, bool flip_z)
// {
//     if (!pc_ptr_) {
//         pc_ptr_ = std::make_shared<open3d::geometry::PointCloud>();
//     }
//     std::string filepath = "data/" + filename;
    
//     if (open3d::io::ReadPointCloud(filepath, *pc_ptr_))
//     {
//         std::cout << "Successfully loaded point cloud from " << filepath << '\n';

//         // Check the scale of the point cloud
//         if (!pc_ptr_->IsEmpty()) {
//             auto bbox = pc_ptr_->GetAxisAlignedBoundingBox();
//             Eigen::Vector3d min = bbox.min_bound_;
//             Eigen::Vector3d max = bbox.max_bound_;
//             Eigen::Vector3d extent = max - min;
            
//             std::cout << "Point cloud extent: " << extent.transpose() << std::endl;

//             // If the extent is in millimeters or centimeters, scale to meters
//             double scale_factor = 1.0;
//             if (extent.x() > 1000 || extent.y() > 1000 || extent.z() > 1000) {
//                 scale_factor = 0.001; // Convert mm to meters
//                 std::cout << "Point cloud scale is in millimeters. Scaling to meters." << std::endl;
//             } 
//             else if (extent.x() > 100 || extent.y() > 100 || extent.z() > 100) {
//                 scale_factor = 0.01; // Convert cm to meters
//                 std::cout << "Point cloud scale is in centimeters. Scaling to meters." << std::endl;
//             }

//             // Scale and Flip the Point Cloud
//             for (auto& point : pc_ptr_->points_) {
//                 point *= scale_factor;  // Scale first
//                 point.z() = -point.z(); // Flip Z-axis
//             }

//             std::cout << "Flipping point cloud along the Z-axis...\n";

//             // Visualize the flipped point cloud
//             std::cout << "Visualizing the flipped and scaled point cloud...\n";
//             open3d::visualization::DrawGeometries({pc_ptr_}, "Flipped Point Cloud", 800, 600);
//         }
//         else {
//             std::cerr << "Loaded point cloud is empty.\n";
//         }

//         return true;
//     }
//     else {
//         std::cerr << "Failed to load point cloud from " << filepath << '\n';
//         return false;
//     }
// }

// bool PointCloudProcessor::loadPointCloud(const std::string& filename, bool flip_z)
// {
//     if (!pc_ptr_) {
//         pc_ptr_ = std::make_shared<open3d::geometry::PointCloud>();
//     }
//     //std::string filepath = "data/" + filename;
//     std::string filepath =  filename;
    
//     if (open3d::io::ReadPointCloud(filepath, *pc_ptr_))
//     {
//         std::cout << "Successfully loaded point cloud from " << filepath << '\n';

//         // Check the scale of the point cloud
//         if (!pc_ptr_->IsEmpty()) {
//             auto bbox = pc_ptr_->GetAxisAlignedBoundingBox();
//             Eigen::Vector3d min = bbox.min_bound_;
//             Eigen::Vector3d max = bbox.max_bound_;
//             Eigen::Vector3d extent = max - min;
            
//             std::cout << "Point cloud extent: " << extent.transpose() << std::endl;

//             // If the extent is in millimeters or centimeters, scale to meters
//             double scale_factor = 1.0;
//             if (extent.x() > 1000 || extent.y() > 1000 || extent.z() > 1000) {
//                 scale_factor = 0.001; // Convert mm to meters
//                 std::cout << "Point cloud scale is in millimeters. Scaling to meters." << std::endl;
//             } 
//             else if (extent.x() > 100 || extent.y() > 100 || extent.z() > 100) {
//                 scale_factor = 0.01; // Convert cm to meters
//                 std::cout << "Point cloud scale is in centimeters. Scaling to meters." << std::endl;
//             }

//             // Scale the point cloud
//             for (auto& point : pc_ptr_->points_) {
//                 point *= scale_factor;
                
//                 // Flip Z-axis only if the user chooses to do so
//                 if (flip_z) {
//                     point.z() = -point.z();
//                 }
//             }

//             if (flip_z) {
//                 std::cout << "Flipping point cloud along the Z-axis...\n";
//             }

//             // Compute and print the center of the scaled (and flipped) point cloud
//             Eigen::Vector3d center = pc_ptr_->GetCenter();
//             std::cout << "Center of the processed point cloud: " << center.transpose() << std::endl;

//             // Visualize the scaled (and possibly flipped) point cloud
//             std::cout << "Visualizing the processed point cloud...\n";
//             open3d::visualization::DrawGeometries({pc_ptr_}, "Processed Point Cloud", 800, 600);
//         }
//         else {
//             std::cerr << "Loaded point cloud is empty.\n";
//         }

//         return true;
//     }
//     else {
//         std::cerr << "Failed to load point cloud from " << filepath << '\n';
//         return false;
//     }
// }

std::shared_ptr<open3d::geometry::PointCloud> PointCloudProcessor::loadPointCloud(
    const std::string& filename, bool flip_z)
{
    // Create a new point cloud
    auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
    std::string filepath = filename;

    if (open3d::io::ReadPointCloud(filepath, *point_cloud))
    {
        std::cout << "Successfully loaded point cloud from " << filepath << '\n';

        // Check if colors are loaded
        if (point_cloud->HasColors()) {
            std::cout << "Point cloud has " << point_cloud->colors_.size() << " colors." << std::endl;
        } else {
            std::cerr << "Point cloud has no colors." << std::endl;
        }

        // Check the scale of the point cloud
        if (!point_cloud->IsEmpty()) {
            auto bbox = point_cloud->GetAxisAlignedBoundingBox();
            Eigen::Vector3d min = bbox.min_bound_;
            Eigen::Vector3d max = bbox.max_bound_;
            Eigen::Vector3d extent = max - min;

            std::cout << "Point cloud extent: " << extent.transpose() << std::endl;

            // If the extent is in millimeters or centimeters, scale to meters
            double scale_factor = 1.0;
            if (extent.x() > 1000 || extent.y() > 1000 || extent.z() > 1000) {
                scale_factor = 0.001; // Convert mm to meters
                std::cout << "Point cloud scale is in millimeters. Scaling to meters." << std::endl;
            }
            else if (extent.x() > 100 || extent.y() > 100 || extent.z() > 100) {
                scale_factor = 0.01; // Convert cm to meters
                std::cout << "Point cloud scale is in centimeters. Scaling to meters." << std::endl;
            }

            // Scale the point cloud
            for (auto& point : point_cloud->points_) {
                point *= scale_factor;

                // Flip Z-axis only if the user chooses to do so
                if (flip_z) {
                    point.z() = -point.z();
                }
            }

            if (flip_z) {
                std::cout << "Flipping point cloud along the Z-axis...\n";
            }

            // Compute and print the center of the scaled (and flipped) point cloud
            Eigen::Vector3d center = point_cloud->GetCenter();
            std::cout << "Center of the processed point cloud: " << center.transpose() << std::endl;

            // Visualize the scaled (and possibly flipped) point cloud
            std::cout << "Visualizing the processed point cloud...\n";
            open3d::visualization::DrawGeometries({point_cloud}, "Processed Point Cloud", 800, 600);
        }
        else {
            std::cerr << "Loaded point cloud is empty.\n";
            return nullptr; // Return null if empty
        }

        // Optionally store in pc_ptr_ if needed elsewhere
        pc_ptr_ = point_cloud;
        return point_cloud;
    }
    else {
        std::cerr << "Failed to load point cloud from " << filepath << '\n';
        return nullptr; // Return null on failure
    }
}






/////////////////////////////////////////////////////////

std::vector<std::shared_ptr<open3d::geometry::PointCloud>> PointCloudProcessor::loadPointClouds(const std::string& folder)
{
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> all_point_clouds;
    Eigen::Vector3d translation_offset(0.0, 0.0, 0.0);  // To keep track of translation

    // Iterate over all PLY files in the specified folder
    for (const auto& entry : std::filesystem::directory_iterator(folder)) {
        if (entry.is_regular_file() && entry.path().extension() == ".ply") {
            std::string filename = entry.path().filename().string();
            std::cout << "Loading point cloud from " << filename << "...\n";

            auto pc = std::make_shared<open3d::geometry::PointCloud>();
            if (open3d::io::ReadPointCloud(entry.path().string(), *pc)) {
                std::cout << "Successfully loaded point cloud from " << filename << '\n';

                // Check the scale of the point cloud
                if (!pc->IsEmpty()) {
                    auto bbox = pc->GetAxisAlignedBoundingBox();
                    Eigen::Vector3d min = bbox.min_bound_;
                    Eigen::Vector3d max = bbox.max_bound_;
                    Eigen::Vector3d extent = max - min;

                    std::cout << "Point cloud extent: " << extent.transpose() << std::endl;

                    // Scale the point cloud
                    double scale_factor = 1.0;
                    if (extent.x() > 1000 || extent.y() > 1000 || extent.z() > 1000) {
                        scale_factor = 0.001; // Convert mm to meters
                        std::cout << "Point cloud scale is in millimeters. Scaling to meters." << std::endl;
                    } 
                    else if (extent.x() > 100 || extent.y() > 100 || extent.z() > 100) {
                        scale_factor = 0.01; // Convert cm to meters
                        std::cout << "Point cloud scale is in centimeters. Scaling to meters." << std::endl;
                    }

                    // Apply scaling and flip the Z-axis
                    for (auto& point : pc->points_) {
                        point *= scale_factor;  // Scale first
                        point.z() = -point.z(); // Flip Z-axis
                    }

                    std::cout << "Flipping point cloud along the Z-axis...\n";

                    // Translate the point cloud based on the current offset
                    for (auto& point : pc->points_) {
                        point += translation_offset;
                    }

                    // Update the translation offset for the next point cloud (along X-axis)
                    translation_offset.x() += extent.x() * scale_factor; // Move next cloud along X-axis

                    all_point_clouds.push_back(pc);
                } else {
                    std::cerr << "Loaded point cloud is empty.\n";
                }
            } else {
                std::cerr << "Failed to load point cloud from " << filename << '\n';
            }
        }
    }

    return all_point_clouds;
}



///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
// PointCloudVisualizer

// Constructor for PointCloudVisualizer
PointCloudVisualizer::PointCloudVisualizer() : PointCloudProcessor()
{
    std::cout << "PointCloudVisualizer initialized." <<'\n';
}

// Destructor for PointCloudVisualizer
PointCloudVisualizer::~PointCloudVisualizer()
{
    std::cout << "PointCloudVisualizer destroyed." << '\n';
}

// Visualize a single point cloud

// void PointCloudVisualizer::visualizerPointCloud()
// {
//     auto pc = getPointCloud();
//     if (pc && !pc->IsEmpty())
//     {
//         std::shared_ptr <open3d::visualization::Visualizer> vis =
//             std::make_shared <open3d::visualization::Visualizer>();
//         vis->CreateVisualizerWindow("Open3D Point Cloud Viewer", 840, 680, 50, 50);
//         vis->AddGeometry(pc, true);
//         vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // Set white background
//         vis->GetRenderOption().point_size_ = 2.0; // Set point size
//         vis->Run();
//         vis->DestroyVisualizerWindow();
//     }
//     else
//     {
//         std::cerr << "Point cloud is empty or null. Cannot visualize."<< '\n';

//     }
// }

// void PointCloudVisualizer::visualizerPointCloud()
// {
//     auto pc = getPointCloud();
//     if (pc && !pc->IsEmpty())
//     {
//         std::shared_ptr<open3d::visualization::Visualizer> vis =
//             std::make_shared<open3d::visualization::Visualizer>();
//         vis->CreateVisualizerWindow("Open3D Point Cloud Viewer", 840, 680, 50, 50);
//         vis->AddGeometry(pc, true);

//         // Set options
//         vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // white background
//         vis->GetRenderOption().point_size_ = 2.0; // point size

//         // Set view control
//         auto& view_control = vis->GetViewControl();
//         view_control.SetFront(Eigen::Vector3d(0.5, 0.5, 0.7).normalized());
//         view_control.SetUp(Eigen::Vector3d(0.0, 0.0, 1.0));
//         view_control.SetLookat(pc->GetCenter());
//         view_control.SetZoom(0.5);

//         // Run the visualizer
//         vis->PollEvents();  // Needed to update the view
//         vis->UpdateRender(); // Update rendering

//         // Save screenshot
//         vis->CaptureScreenImage("output/point_cloud_screenshot.png"); // Save in output folder

//         vis->Run();
//         vis->DestroyVisualizerWindow();
//     }
//     else
//     {
//         std::cerr << "Point cloud is empty or null. Cannot visualize." << '\n';
//     }
// }

void PointCloudVisualizer::visualizerPointCloud(double zoom = 0.5)
{
    auto pc = getPointCloud();
    if (pc && !pc->IsEmpty())
    {
        std::shared_ptr<open3d::visualization::Visualizer> vis =
            std::make_shared<open3d::visualization::Visualizer>();
        vis->CreateVisualizerWindow("Open3D Point Cloud Viewer", 1920, 1080, 50, 50);
        vis->AddGeometry(pc, true);

        // Set options
        vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // white background
        vis->GetRenderOption().point_size_ = 2.0; // point size

        // Set top-down orthographic view control
        auto& view_control = vis->GetViewControl();
        view_control.SetFront(Eigen::Vector3d(0.0, 0.0, -1.0)); // Look down Z-axis
        view_control.SetUp(Eigen::Vector3d(0.0, 1.0, 0.0)); // Y-axis is up
        view_control.SetLookat(pc->GetCenter()); // Center of point cloud
        view_control.SetZoom(zoom); // Use parameter, default 0.5

        // Run the visualizer
        vis->PollEvents();  // Needed to update the view
        vis->UpdateRender(); // Update rendering

        // Create output directory if it doesn't exist
        //std::filesystem::create_directories("output");

        // Generate dynamic filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "output/point_cloud_screenshot_"
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << ".png";
        std::string filename = ss.str();

        // Save screenshot
        vis->CaptureScreenImage(filename); // Save with dynamic filename

        vis->Run();
        vis->DestroyVisualizerWindow();
    }
    else
    {
        std::cerr << "Point cloud is empty or null. Cannot visualize." << '\n';
    }
}

////////////////////////////////////////

// void PointCloudVisualizer::visualizerClusters(const std::vector<PC_o3d_ptr>& clusters, bool keep_original_color) {
//     std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;

//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> dis(0.0, 1.0);  // RGB range between 0 and 1

//     for (size_t i = 0; i < clusters.size(); i++) {
//         auto cluster = clusters[i];

//         if (!keep_original_color || cluster->colors_.empty()) {
//             // Assign random color
//             Eigen::Vector3d color(dis(gen), dis(gen), dis(gen));
//             cluster->colors_.clear();
//             cluster->colors_.resize(cluster->points_.size(), color);
//         }

//         geometries.push_back(cluster);
//     }

//     open3d::visualization::DrawGeometries(geometries, "Clustered Point Cloud");
// }

// void PointCloudVisualizer::visualizerClusters(const std::vector<PC_o3d_ptr>& clusters, bool keep_original_color) {
//     // Validate input
//     if (clusters.empty()) {
//         std::cerr << "No clusters to visualize." << '\n';
//         return;
//     }

//     std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;

//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> dis(0.0, 1.0);  // RGB range between 0 and 1

//     // Log cluster details
//     std::cout << "Visualizing " << clusters.size() << " clusters." << '\n';
//     for (size_t i = 0; i < clusters.size(); ++i) {
//         std::cout << "Cluster " << i << ": " << clusters[i]->points_.size() << " points." << '\n';
//     }

//     for (size_t i = 0; i < clusters.size(); i++) {
//         auto cluster = clusters[i];

//         if (!cluster->points_.empty()) {
//             if (!keep_original_color || cluster->colors_.empty()) {
//                 // Assign random color
//                 Eigen::Vector3d color(dis(gen), dis(gen), dis(gen));
//                 cluster->colors_.clear();
//                 cluster->colors_.resize(cluster->points_.size(), color);
//             }
//             geometries.push_back(cluster);
//         } else {
//             std::cerr << "Cluster " << i << " is empty, skipping." << '\n';
//         }
//     }

//     // Check if any valid geometries were added
//     if (geometries.empty()) {
//         std::cerr << "No valid clusters to visualize." << '\n';
//         return;
//     }

//     // Visualize and save screenshot
//     {
//         std::shared_ptr<open3d::visualization::Visualizer> vis =
//             std::make_shared<open3d::visualization::Visualizer>();
//         vis->CreateVisualizerWindow("Clustered Point Cloud", 1920, 1080, 50, 50);

//         // Add all geometries
//         for (const auto& geom : geometries) {
//             vis->AddGeometry(geom, true);
//         }

//         // Set options
//         vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // White background
//         vis->GetRenderOption().point_size_ = 2.0; // Point size

//         // Compute centroid and max extent of all cluster points
//         Eigen::Vector3d centroid(0.0, 0.0, 0.0);
//         double max_extent = 0.0;
//         size_t total_points = 0;
//         for (const auto& cluster : clusters) {
//             if (!cluster->points_.empty()) {
//                 auto bbox = cluster->GetAxisAlignedBoundingBox();
//                 centroid += bbox.GetCenter() * cluster->points_.size();
//                 total_points += cluster->points_.size();
//                 max_extent = std::max(max_extent, bbox.GetExtent().maxCoeff());
//             }
//         }
//         if (total_points > 0) {
//             centroid /= total_points;
//         }

//         // Set view control (top-down orthographic)
//         auto& view_control = vis->GetViewControl();
//         view_control.SetFront(Eigen::Vector3d(0.0, 0.0, -1.0)); // Look down Z-axis
//         view_control.SetUp(Eigen::Vector3d(0.0, 1.0, 0.0)); // Y-axis is up
//         view_control.SetLookat(centroid); // Center of clusters
//         // Dynamic zoom based on extent
//         double zoom = 0.8 / (1.0 + max_extent);
//         view_control.SetZoom(std::min(zoom, 1.0)); // Cap at 1.0 to avoid over-zooming

//         // Ensure rendering
//         vis->PollEvents();
//         vis->UpdateRender();
//         vis->PollEvents(); // Additional call for rendering completion

//         // Create output directory if it doesn't exist
//         //std::filesystem::create_directories("output");

//         // Generate dynamic filename with timestamp
//         auto now = std::chrono::system_clock::now();
//         auto time_t = std::chrono::system_clock::to_time_t(now);
//         std::stringstream ss;
//         ss << "output/clustered_point_cloud_"
//            << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
//            << ".png";
//         std::string filename = ss.str();

//         // Save screenshot
//         vis->CaptureScreenImage(filename);

//         // Clean up
//         vis->Run();
//         vis->DestroyVisualizerWindow();
//     }
// }


// Helper function to convert HSV to RGB
Eigen::Vector3d HSVToRGB(double h, double s, double v) {
    // Hue: 0-360, Saturation/Value: 0-1
    double c = v * s; // Chroma
    double h_prime = h / 60.0;
    double x = c * (1.0 - std::abs(std::fmod(h_prime, 2.0) - 1.0));
    double m = v - c;

    double r = 0.0, g = 0.0, b = 0.0;
    if (h_prime >= 0.0 && h_prime < 1.0) {
        r = c; g = x; b = 0.0;
    } else if (h_prime < 2.0) {
        r = x; g = c; b = 0.0;
    } else if (h_prime < 3.0) {
        r = 0.0; g = c; b = x;
    } else if (h_prime < 4.0) {
        r = 0.0; g = x; b = c;
    } else if (h_prime < 5.0) {
        r = x; g = 0.0; b = c;
    } else if (h_prime < 6.0) {
        r = c; g = 0.0; b = x;
    }

    return Eigen::Vector3d(r + m, g + m, b + m);
}

void PointCloudVisualizer::visualizerClusters(const std::vector<PC_o3d_ptr>& clusters, bool keep_original_color) {
    // Validate input
    if (clusters.empty()) {
        std::cerr << "No clusters to visualize." << '\n';
        return;
    }

    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> hue_dis(0.0, 360.0); // Hue: 0-360 degrees
    std::uniform_real_distribution<double> sat_dis(0.7, 1.0);   // Saturation: 0.7-1.0
    std::uniform_real_distribution<double> val_dis(0.7, 0.9);   // Value: 0.7-0.9

    // Log cluster details
    std::cout << "Visualizing " << clusters.size() << " clusters (keep_original_color = " 
              << (keep_original_color ? "true" : "false") << ")." << '\n';
    for (size_t i = 0; i < clusters.size(); ++i) {
        std::cout << "Cluster " << i << ": " << clusters[i]->points_.size() << " points, "
                  << "has colors: " << (!clusters[i]->colors_.empty() ? "Yes" : "No") << '\n';
    }

    for (size_t i = 0; i < clusters.size(); i++) {
        auto cluster = clusters[i];

        if (!cluster->points_.empty()) {
            if (!keep_original_color || cluster->colors_.empty()) {
                // Assign vibrant random color using HSV
                double hue = hue_dis(gen);
                double sat = sat_dis(gen);
                double val = val_dis(gen);
                Eigen::Vector3d color = HSVToRGB(hue, sat, val);
                cluster->colors_.clear();
                cluster->colors_.resize(cluster->points_.size(), color);
            }
            geometries.push_back(cluster);
        } else {
            std::cerr << "Cluster " << i << " is empty, skipping." << '\n';
        }
    }

    // Check if any valid geometries were added
    if (geometries.empty()) {
        std::cerr << "No valid clusters to visualize." << '\n';
        return;
    }

    // Visualize and save screenshot
    {
        std::shared_ptr<open3d::visualization::Visualizer> vis =
            std::make_shared<open3d::visualization::Visualizer>();
        vis->CreateVisualizerWindow("Clustered Point Cloud", 1920, 1080, 50, 50);

        // Add all geometries
        for (const auto& geom : geometries) {
            vis->AddGeometry(geom, true);
        }

        // Set options
        vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // White background
        vis->GetRenderOption().point_size_ = 2.0; // Point size

        // Compute centroid and max extent of all cluster points
        Eigen::Vector3d centroid(0.0, 0.0, 0.0);
        double max_extent = 0.0;
        size_t total_points = 0;
        for (const auto& cluster : clusters) {
            if (!cluster->points_.empty()) {
                auto bbox = cluster->GetAxisAlignedBoundingBox();
                centroid += bbox.GetCenter() * cluster->points_.size();
                total_points += cluster->points_.size();
                max_extent = std::max(max_extent, bbox.GetExtent().maxCoeff());
            }
        }
        if (total_points > 0) {
            centroid /= total_points;
        }
        std::cout << "Centroid: " << centroid.transpose() << ", Max extent: " << max_extent << '\n';

        // Set view control (top-down orthographic)
        auto& view_control = vis->GetViewControl();
        view_control.SetFront(Eigen::Vector3d(0.0, 0.0, -1.0)); // Look down Z-axis
        view_control.SetUp(Eigen::Vector3d(0.0, 1.0, 0.0)); // Y-axis is up
        view_control.SetLookat(centroid); // Center of clusters
        // Dynamic zoom based on extent
        double zoom = 0.5 / (1.0 + max_extent);
        view_control.SetZoom(std::min(zoom, 1.0)); // Cap at 1.0 to avoid over-zooming

        // Ensure rendering
        vis->PollEvents();
        vis->UpdateRender();
        vis->PollEvents(); // Additional call for rendering completion


        // Generate dynamic filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "output/clustered_point_cloud_"
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << ".png";
        std::string filename = ss.str();

        // Save screenshot
        vis->CaptureScreenImage(filename);

        // Clean up
        vis->Run();
        vis->DestroyVisualizerWindow(); // Closes immediately
    }
}


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
// PointCloudPerception

 //constructor and destructor
PointCloudPerception::PointCloudPerception() : PointCloudProcessor(), PointCloudVisualizer() {
    std::cout << "PointCloudPerception initialized.\n";
    std::cout << "Voxel size in PointCloudPerception constructor: " << voxel_size_ << std::endl;
}

PointCloudPerception::~PointCloudPerception() {
    std::cout << "PointCloudPerception destroyed.\n";
}



/////////////////////////////////////////////////////////////
// 1.refinePointCloud
bool PointCloudPerception::refinePointCloud()
{
    try
    {
        // Get the point cloud using the getter method
        auto pc = getPointCloud();
        std::cout << "Loaded voxel size for refinement: " << voxel_size_ << std::endl;
        //// Code that might throw exceptions
        if (!checkPointCloud()) return false;
        log("Starting point cloud preprocessing...");
        logOriginalPointCloud();

        // Downsample the point cloud
        log("Using voxel size: " + std::to_string(voxel_size_));  // Log voxel size
        auto downsampled_pc_ptr = pc->VoxelDownSample(voxel_size_);
        logPointCloudSize("downsampled", downsampled_pc_ptr);

        // Print the bounding box and extent
        auto bbox = downsampled_pc_ptr->GetAxisAlignedBoundingBox();
        auto extent = bbox.GetExtent();  // Get the dimensions of the bounding box (x, y, z)
        std::cout << "Point cloud extent: " << extent.transpose() << std::endl;

        // Outlier removal
        auto outlier_removed_pc_ptr = downsampled_pc_ptr->RemoveStatisticalOutliers (nb_neighbors_ , std_ratio_);
        auto inlier_indices = std::get<1>(outlier_removed_pc_ptr);// Extract inlier indices
        auto inlier_pc_ptr = downsampled_pc_ptr->SelectByIndex(inlier_indices);
        

        logPointCloudSize("filtered" , inlier_pc_ptr);
        log("Point cloud preprocessing completed.");

        // Update the class member with the refined point cloud
        setPointCloud(inlier_pc_ptr);

        // Visualize the refined point cloud
        log("Visualizing the refined point cloud...");
        //visualizerPointCloud();

        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


/////////////////////////////////////////////////////////////////////
// 2. segmentAndRemovePlane
// void PointCloudPerception::segmentAndRemovePlane() {  // Make sure this matches
//     if (!checkPointCloud()) {
//         return;
//     }

//     auto pc = getPointCloud();

//     // Step 1: Segment the largest plane using RANSAC
//     auto [plane_model, inlier_indices] = pc->SegmentPlane(0.006, ransac_n_, num_iterations_);

//     log("Detected plane equation: " +
//         std::to_string(plane_model(0)) + "x + " +
//         std::to_string(plane_model(1)) + "y + " +
//         std::to_string(plane_model(2)) + "z + " +
//         std::to_string(plane_model(3)) + " = 0");

//     // Step 2: Extract inliers (plane points) and outliers (remaining points)
//     auto plane_pc = pc->SelectByIndex(inlier_indices);
//     auto non_plane_pc = pc->SelectByIndex(inlier_indices, true); // true -> invert selection

//     logPointCloudSize("Plane (removed)", plane_pc);
//     logPointCloudSize("Remaining after plane removal", non_plane_pc);
//     open3d::visualization::DrawGeometries({plane_pc}, "Segmented Plane");

//     // Step 3: Update the class member with the processed point cloud
//     setPointCloud(non_plane_pc);
//     visualizerPointCloud();
// }


void PointCloudPerception::segmentAndRemovePlane() {
    if (!checkPointCloud()) {
        return;
    }

    auto pc = getPointCloud();

    // Step 1: Segment the largest plane using RANSAC
    auto [plane_model, inlier_indices] = pc->SegmentPlane(0.006, ransac_n_, num_iterations_);

    log("Detected plane equation: " +
        std::to_string(plane_model(0)) + "x + " +
        std::to_string(plane_model(1)) + "y + " +
        std::to_string(plane_model(2)) + "z + " +
        std::to_string(plane_model(3)) + " = 0");

    // Step 2: Extract inliers (plane points) and outliers (remaining points)
    auto plane_pc = pc->SelectByIndex(inlier_indices);
    auto non_plane_pc = pc->SelectByIndex(inlier_indices, true); // true -> invert selection

    logPointCloudSize("Plane (removed)", plane_pc);
    logPointCloudSize("Remaining after plane removal", non_plane_pc);

    // // Visualize and save screenshot of segmented plane
    // {
    //     std::shared_ptr<open3d::visualization::Visualizer> vis =
    //         std::make_shared<open3d::visualization::Visualizer>();
    //     vis->CreateVisualizerWindow("Segmented Plane", 1920, 1080, 50, 50);
    //     vis->AddGeometry(plane_pc, true);

    //     // Set options (consistent with visualizerPointCloud)
    //     vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // white background
    //     vis->GetRenderOption().point_size_ = 2.0; // point size

        //         // Set view control (top-down orthographic)
        // auto& view_control = vis->GetViewControl();
        // view_control.SetFront(Eigen::Vector3d(0.0, 0.0, -1.0)); // Look down Z-axis
        // view_control.SetUp(Eigen::Vector3d(0.0, 1.0, 0.0)); // Y-axis is up
        // view_control.SetLookat(pc->GetCenter()); // Center of point cloud
        // view_control.SetZoom(0.8); // Adjusted for top-down fit
        // //view_control.ChangeProjection(open3d::visualization::ViewControl::ProjectionMode::Orthographic);

        // // Render and capture screenshot
        // vis->PollEvents();  // Update the view
        // vis->UpdateRender(); // Update rendering

        // // Create output directory if it doesn't exist
        // std::filesystem::create_directories("output");

        // // Generate dynamic filename with timestamp
        // auto now = std::chrono::system_clock::now();
        // auto time_t = std::chrono::system_clock::to_time_t(now);
        // std::stringstream ss;
        // ss << "output/segmented_plane_"
        //    << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
        //    << ".png";
        // std::string filename = ss.str();

        // // Save screenshot
        // vis->CaptureScreenImage(filename);

        // Clean up 
    //     vis->Run();
    //     vis->DestroyVisualizerWindow();

    // }

    // Step 3: Update the class member with the processed point cloud
    setPointCloud(non_plane_pc);
    //visualizerPointCloud();
}



////////////////////////////////////////////////////////////////////////////
 // 3. Method to process all loaded pointclouds (refine > segment > merge)
void PointCloudPerception::processPointClouds(const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& all_point_clouds) {
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> non_plane_point_clouds;

    // Iterate through the list of point clouds
    for (auto& pc : all_point_clouds) {
        // Set the current point cloud to be processed
        setPointCloud(pc);

        // Refine the point cloud
        refinePointCloud();

        // Segment and remove the plane
        segmentAndRemovePlane();

        // After segmenting, retrieve the non-plane point cloud
        auto non_plane_pc = getPointCloud(); // This will be the point cloud after plane removal
        non_plane_point_clouds.push_back(non_plane_pc); // Add to the list of non-plane point clouds
    }

    // Now merge all non-plane point clouds into one
    auto merged_pc = std::make_shared<open3d::geometry::PointCloud>();
    for (const auto& pc : non_plane_point_clouds) {
        *merged_pc += *pc; // Merge each non-plane point cloud
    }

    // Set the merged point cloud
    setPointCloud(merged_pc);

    // Optionally, visualize the merged point cloud
    // open3d::visualization::DrawGeometries({merged_pc}, "Merged Point Cloud", 800, 600);
    visualizerPointCloud();
}



////////////////////////////////////////////////////////////////////
// 4. Method to perform Euclidean Clustering
void PointCloudPerception::EuclideanClustering(bool debug) {
    if (!checkPointCloud()) {
        return;
    }

    auto pc = getPointCloud();
    log("Starting Euclidean Clustering...");

    // Perform clustering
    std::vector<int> labels = pc->ClusterDBSCAN(
        cluster_tolerance_,   // Tolerance (epsilon)
        min_cluster_size_,    // Minimum cluster size
        false                 // Print progress
    );

    // Find the number of clusters (max label + 1)
    int max_label = *std::max_element(labels.begin(), labels.end());
    std::cout << "Number of clusters found: " << max_label + 1 << std::endl;

    // Create a vector of point cloud clusters
    std::vector<PC_o3d_ptr> cluster_clouds(max_label + 1);

    // Initialize clusters
    for (int i = 0; i <= max_label; ++i) {
        cluster_clouds[i] = std::make_shared<open3d::geometry::PointCloud>();
    }

    // Assign points to respective clusters
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {  // Ignore noise (-1)
            cluster_clouds[labels[i]]->points_.push_back(pc->points_[i]);
            if (!pc->colors_.empty()) {  // Preserve colors if available
                cluster_clouds[labels[i]]->colors_.push_back(pc->colors_[i]);
            }
        }
    }

    // Store clusters in class member
    setClusters(cluster_clouds);

    log("Clustering completed. " + std::to_string(cluster_clouds.size()) + " clusters detected.");

    //     // Visualize clustered point cloud
    visualizerClusters(cluster_clouds , true);
    //visualizerClusters(cluster_clouds , true);


    // Optional: Visualize all clusters separately
    if (debug)
    {
        for (size_t i = 0; i < cluster_clouds.size(); ++i) {
            std::cout << "Cluster " << i << " has " << cluster_clouds[i]->points_.size() << " points." << std::endl;
            open3d::visualization::DrawGeometries({cluster_clouds[i]}, "Cluster " + std::to_string(i));
        }
    }
}

    






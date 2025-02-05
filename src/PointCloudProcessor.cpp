#include "iostream"
#include <fstream>
#include <string>

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
bool PointCloudProcessor::loadPointCloud(const std::string& filename)
{
    if (!pc_ptr_) {
        pc_ptr_ = std::make_shared<open3d::geometry::PointCloud>();
    }
    std::string filepath="data/"+filename ;
    if (open3d::io::ReadPointCloud(filepath , *pc_ptr_))
    {
        std::cout << "Successfully loaded point cloud from " <<filepath <<'\n';
        
        // Visualize the loaded point cloud
        if (!pc_ptr_->IsEmpty()) {
            std::cout << "Visualizing the loaded point cloud...\n";
            open3d::visualization::DrawGeometries({pc_ptr_}, "Loaded Point Cloud", 800, 600);
        } 
        else {
            std::cerr << "Loaded point cloud is empty.\n";
        }

        return true;

    }
    else
    {
        std::cerr << "Failed to load point cloud from " << filepath << '\n';
        return false;
    }

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

void PointCloudVisualizer::visualizerPointCloud()
{
    auto pc = getPointCloud();
    if (pc && !pc->IsEmpty())
    {
        std::shared_ptr <open3d::visualization::Visualizer> vis =
            std::make_shared <open3d::visualization::Visualizer>();
        vis->CreateVisualizerWindow("Open3D Point Cloud Viewer", 840, 680, 50, 50);
        vis->AddGeometry(pc, true);
        vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // Set white background
        vis->GetRenderOption().point_size_ = 2.0; // Set point size
        vis->Run();
        vis->DestroyVisualizerWindow();
    }
    else
    {
        std::cerr << "Point cloud is empty or null. Cannot visualize."<< '\n';

    }
}


    // visualizerClusters
void PointCloudVisualizer::visualizerClusters(const std::vector<PC_o3d_ptr>& clusters) {
    // Visualize the clusters passed to the function
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;

    // Random color generator (you can use a different approach to assign specific colors)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);  // RGB range between 0 and 1

    // Iterate over clusters and assign a random color to each one
    for (size_t i = 0; i < clusters.size(); i++) {
        auto cluster = clusters[i];
        
        // Assign a single random color to each cluster
        Eigen::Vector3d color(dis(gen), dis(gen), dis(gen));  // Random RGB color
        
        // Assign this color to all points in the cluster
        cluster->colors_.clear();  // Clear any existing colors
        cluster->colors_.resize(cluster->points_.size(), color);  // Assign color to all points in the cluster
        
        // Add this cluster to the list of geometries to visualize
        geometries.push_back(cluster);
    }

    // Visualize all clusters with distinct colors
    open3d::visualization::DrawGeometries(geometries, "Clustered Point Cloud");
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
        visualizerPointCloud();

        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

// 2. segmentAndRemovePlane
void PointCloudPerception::segmentAndRemovePlane() {  // Make sure this matches
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
    open3d::visualization::DrawGeometries({plane_pc}, "Segmented Plane");

    // Step 3: Update the class member with the processed point cloud
    setPointCloud(non_plane_pc);
    visualizerPointCloud();
}



// 3. Method to perform Euclidean Clustering
// void PointCloudPerception::EuclideanClustering() {
//     if (!checkPointCloud()) {
//         return;
//     }

//     auto pc = getPointCloud();
//     log("Starting Euclidean Clustering...");

//     // Ensure the point cloud is not empty
//     if (pc->points_.empty()) {
//         log("Point cloud is empty, cannot perform clustering.");
//         return;
//     }

//     // Perform clustering
//     std::vector<int> labels = pc->ClusterDBSCAN(cluster_tolerance_, min_cluster_size_, false);

//     // Determine number of clusters
//     int num_clusters = *std::max_element(labels.begin(), labels.end()) + 1;
//     log("Number of clusters found: " + std::to_string(num_clusters));

//     // Create colored point clouds for visualization
//     std::vector<Eigen::Vector3d> cluster_colors;
//     for (int i = 0; i < num_clusters; ++i) {
//         cluster_colors.push_back(Eigen::Vector3d::Random().cwiseAbs()); // Random colors for clusters
//     }

//     for (size_t i = 0; i < labels.size(); ++i) {
//         if (labels[i] >= 0) {
//             pc->colors_[i] = cluster_colors[labels[i]];
//         } else {
//             pc->colors_[i] = Eigen::Vector3d(0.5, 0.5, 0.5); // Gray for unclustered points
//         }
//     }

//     // Visualize clustered point cloud
//     open3d::visualization::DrawGeometries({pc}, "Euclidean Clustering Result");
// }




void PointCloudPerception::EuclideanClustering() {
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
    visualizerClusters(cluster_clouds);

    // Optional: Visualize all clusters separately
    for (size_t i = 0; i < cluster_clouds.size(); ++i) {
        std::cout << "Cluster " << i << " has " << cluster_clouds[i]->points_.size() << " points." << std::endl;
        open3d::visualization::DrawGeometries({cluster_clouds[i]}, "Cluster " + std::to_string(i));
    }
}

    






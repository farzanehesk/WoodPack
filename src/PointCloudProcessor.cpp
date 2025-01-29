#include "iostream"
#include <fstream>
#include <string>

#include "../include/PointCloudProcessor.hpp"

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
// PointCloudProcessor

//constructor fro PointCloudProcessor
PointCloudProcessor::PointCloudProcessor ()
{
    std::cout << "PointCloudProcessor initialized." << '\n';
}


// destructor
PointCloudProcessor::~PointCloudProcessor()
{
    std::cout << "PPointCloudProcessor destroyed. \n" ;

}


// 1. Load parameters 
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
            }
        }
    }

    config_file.close();
}



// 2. Method to load a point cloud from a file within the 'data' directory
bool PointCloudProcessor::loadPointCloud(const std::string& filename)
{
    if (!pc_ptr_) {
        pc_ptr_ = std::make_shared<open3d::geometry::PointCloud>();
    }
    std::string filepath="data/"+filename ;
    if (open3d::io::ReadPointCloud(filepath , *pc_ptr_))
    {
        std::cout << "Successfully loaded point cloud from " <<filepath <<'\n';
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




///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
// PointCloudPerception

 //constructor and destructor
PointCloudPerception::PointCloudPerception() : PointCloudProcessor(), PointCloudVisualizer() {
    std::cout << "PointCloudPerception initialized.\n";
}

PointCloudPerception::~PointCloudPerception() {
    std::cout << "PointCloudPerception destroyed.\n";
}


// // 1.PointCloudPerception
bool PointCloudPerception::refinePointCloud()
{

    try
    {

        // Get the point cloud using the getter method
        auto pc = getPointCloud();

        //// Code that might throw exceptions
        if (!checkPointCloud()) return false;


        log("Starting point cloud preprocessing...");
        logOriginalPointCloud();

        

        // Downsample the point cloud
        auto downsampled_pc_ptr = pc->VoxelDownSample(voxel_size_);
        logPointCloudSize("downsampled", downsampled_pc_ptr);




    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    



    



}


#include "iostream"
#include "PointCloudProcessor.hpp"

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


// Method to load a point cloud from a file within the 'data' directory
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
PointCloudVisualizer::PointCloudVisualizer()
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

bool PointCloudPerception::refinePointCloud()
{
    auto pc = getPointCloud();
}


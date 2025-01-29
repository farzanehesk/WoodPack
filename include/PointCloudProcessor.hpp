#ifndef POINTCLOUDPROCESSOR_H
#define POINTCLOUDPROCESSOR_H


#include "memory"
#include "vector"
#include "open3d/Open3D.h"
#include "../include/PointCloudProcessor.hpp"
#include "../include/custom_types.hpp"


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
class PointCloudProcessor 
{
    private:
        // Members to hold the point cloud and clusters
        PC_o3d_ptr pc_ptr_ ; // Internal shared pointer to the point cloud
        std::vector <PC_o3d_ptr> clusters_ptr_ ;





    public:


        bool verbose_;


        // function parameters
        double voxel_size_;  // Voxel size for downsampling
        int nb_neighbors_;
        double std_ratio_;

    
        // Constructor and Destructor
        PointCloudProcessor ();
        ~PointCloudProcessor();

        // Setter method to set the pointer

        void setPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& pc_ptr) {
            pc_ptr_ = pc_ptr;
        }


        // Getter method to retrieve the point cloud pointer
        PC_o3d_ptr getPointCloud() const
        {
            return pc_ptr_;
        }



        void resetPointers() {
            pc_ptr_ = nullptr;
            clusters_ptr_.clear();
        } 

        // Method to check if the point cloud is valid
        bool checkPointCloud() const {
            if (pc_ptr_ == nullptr || pc_ptr_->IsEmpty()) {
                std::cerr << "Error: Point cloud is empty!" << std::endl;
                return false;
            }

            // Check the scale of the point cloud
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

        // Method to log a message (logMessage is used here)
        void log(const std::string& message) const {
            if (verbose_) { // Assuming `verbose_` is a member variable
                std::cout << message << std::endl;
            }
        }
        // Method to log the original point cloud size
        void logOriginalPointCloud() const {
            log("Number of points in original point cloud: " + std::to_string(pc_ptr_->points_.size()));
        }

        // 
        void logPointCloudSize(const std::string& stage, const std::shared_ptr<open3d::geometry::PointCloud>& cloud) const {
        if (cloud) 
        {
            log("Number of points in " + stage + " point cloud: " + std::to_string(cloud->points_.size()));
        } 
        else 
        {
            log("Point cloud is empty at stage: " + stage);
        }
}


        





        // 1. Load parameters (simulating loading from a configuration)
        void loadParameters(const std::string& config_file_name);

        // 2. Method to load a point cloud from a file within the 'data' directory
        bool loadPointCloud (const std::string& filename);






};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


// Visualization class inheriting from PointCloudProcessor
class PointCloudVisualizer:  public virtual PointCloudProcessor
{
    public:
    //constructor and destructor
    PointCloudVisualizer();
    ~PointCloudVisualizer();
    
    //Member Functions
    void visualizerPointCloud();
    //void visualizerClusters();


};


class PointCloudPerception : public virtual PointCloudProcessor , public PointCloudVisualizer
{

    public:
        // Constructor and Destructor
        PointCloudPerception();
        ~PointCloudPerception();

        // 1. Method to refine point clouds
        bool refinePointCloud();


};








#endif // POINTCLOUDPROCESSOR_H
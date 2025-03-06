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
        double voxel_size_;  
        int nb_neighbors_;
        double std_ratio_;
        int ransac_n_;
        int num_iterations_;
        double cluster_tolerance_;
        int min_cluster_size_;

    
        // Constructor and Destructor
        PointCloudProcessor ();
        ~PointCloudProcessor();

        // Setter method to set the pointer
        void setPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& pc_ptr) { pc_ptr_ = pc_ptr; }

        // Getter method to retrieve the point cloud pointer
        PC_o3d_ptr getPointCloud() const{ return pc_ptr_; }

        // Setter: Store clusters
        void setClusters(const std::vector<PC_o3d_ptr>& clusters) { clusters_ptr_ = clusters; }

        // Getter: Retrieve clusters
        std::vector<PC_o3d_ptr> getClusters() const { return clusters_ptr_; }

        void resetPointers() {
            pc_ptr_ = nullptr;
            clusters_ptr_.clear();
        } 

        // 1. Method to check if the point cloud is valid
        bool checkPointCloud() const;

        // 2. Method to log a message
        void log(const std::string& message) const;

        // 3. Method to log the original point cloud size
        void logOriginalPointCloud() const;


        // 
        
       
        // 4. Method to log the point cloud size
        void logPointCloudSize(const std::string& stage, const std::shared_ptr<open3d::geometry::PointCloud>& cloud) const;

        // 5. Load parameters (simulating loading from a configuration)
        void loadParameters(const std::string& config_file_name);

        // 6. Method to load a point cloud from a file within the 'data' directory
        bool loadPointCloud (const std::string& filename);

        // 7. ImportAndMergeScans
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> loadPointClouds(const std::string& folder);



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
    //
    
    void visualizerClusters(const std::vector<PC_o3d_ptr>& clusters);  // Visualize clusters


    //void visualizeMultiplePointClouds();

};


class PointCloudPerception : public virtual PointCloudProcessor , public PointCloudVisualizer
{

    public:
        // Constructor and Destructor
        PointCloudPerception();
        ~PointCloudPerception();



        // 1. Method to refine point clouds
        bool refinePointCloud();

        // 2. Method to segmentAndRemovePlane
        void segmentAndRemovePlane(); 

        // 3. Method to process all loaded pointclouds (refine > segment > merge)
        void processPointClouds(const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& all_point_clouds);

        // 4. Method to perform Euclidean Clustering
        void EuclideanClustering(bool debug);


};








#endif // POINTCLOUDPROCESSOR_H
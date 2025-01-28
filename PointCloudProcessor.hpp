#ifndef POINTCLOUDPROCESSOR_H
#define POINTCLOUDPROCESSOR_H


#include "memory"
#include "vector"
#include "open3d/Open3D.h"
#include "custom_types.hpp"


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
class PointCloudProcessor 
{
    private:
        // Members to hold the point cloud and clusters
        PC_o3d_ptr pc_ptr_ ; // Internal shared pointer to the point cloud
        std::vector <PC_o3d_ptr> clusters_ptr_ ;

    public:
    
        // Constructor and Destructor
        PointCloudProcessor ();
        ~PointCloudProcessor();

        // Setter method to set the pointer

        void setPointer(const std::shared_ptr<open3d::geometry::PointCloud>& pc_ptr) {
            pc_ptr_ = pc_ptr;
        }


        // Getter method to retrieve the point cloud pointer
        PC_o3d_ptr getPOintCloud()
        {
            return pc_ptr_;
        }



        void resetPointers() {
            pc_ptr_ = nullptr;
            clusters_ptr_.clear();
        } 

        // Method to load a point cloud from a file within the 'data' directory
        bool loadPointCloud (const std::string& filename);




};

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////


// Visualization class inheriting from PointCloudProcessor
class PointCloudVisualizer:public PointCloudProcessor
{
    public:
    //constructor and destructor
    PointCloudVisualizer();
    ~PointCloudVisualizer();
    
    //Member Functions
    void visualizerPointCloud();
    //void visualizerClusters();


};











#endif // POINTCLOUDPROCESSOR_H
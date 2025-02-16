#ifndef GEOMETRYPROCESSOR_H
#define GEOMETRYPROCESSOR_H


#include <open3d/Open3D.h>
#include <vector>


// Forward declaration of the PointCloud type
using PC_o3d_ptr = std::shared_ptr<open3d::geometry::PointCloud>;


class GeometryProcessor
{



public:
    // Constructor and Destructor
    GeometryProcessor();
    ~GeometryProcessor();

    // 1. Method to compute oriented bounding boxes for a vector of point clouds
    std::vector<open3d::geometry::OrientedBoundingBox> computeOrientedBoundingBoxes(
        const std::vector<PC_o3d_ptr>& clusters);

    // 2. Method to visualize clusters with bb
    void visualizeBoundingBoxes (
        const std::vector<PC_o3d_ptr>& clusters, 
        const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) ;
        






};




#endif //GEOMETRYPROCESSOR_H
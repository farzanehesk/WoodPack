#ifndef GEOMETRYPROCESSOR_H
#define GEOMETRYPROCESSOR_H


#include <open3d/Open3D.h>
#include <vector>
#include "custom_types.hpp"


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

    // 3. Method to compute axis-aligned bounding boxes for a vector of point clouds
    // std::vector<open3d::geometry::AxisAlignedBoundingBox> computeAxisAlignedBoundingBoxes(
    //     const std::vector<PC_o3d_ptr>& clusters);

    // 4. Method to visualize clusters with AABBs
    // void visualizeBoundingBoxes(
    //     const std::vector<PC_o3d_ptr>& clusters,
    //     const std::vector<open3d::geometry::AxisAlignedBoundingBox>& bounding_boxes);


    // 5. Method to visualize bounding boxes on the original poitn cloud
    void visualizeBoundingBoxesAndOriginalPc(
        const std::shared_ptr<open3d::geometry::PointCloud>& original_pc,
        const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes);

    // 6.Method to getWidthsOfBoundingBoxes
    std::vector<double> getWidthsOfBoundingBoxes(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes);


    // 7. Method to generate a list of random rectangles
    std::vector <Rectangle> generateRandomRectangles (int count);


    // 8. Method to print rectangles
    void printRectangles(const std::vector<Rectangle>& rectangles);







};




#endif //GEOMETRYPROCESSOR_H
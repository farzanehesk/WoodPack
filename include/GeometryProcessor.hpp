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
    // void visualizeBoundingBoxes (
    //     const std::vector<PC_o3d_ptr>& clusters, 
    //     const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) ;

    // 3. Method to compute axis-aligned bounding boxes for a vector of point clouds
    // std::vector<open3d::geometry::AxisAlignedBoundingBox> computeAxisAlignedBoundingBoxes(
    //     const std::vector<PC_o3d_ptr>& clusters);

    // 4. Method to visualize clusters with AABBs
    // void visualizeBoundingBoxes(
    //     const std::vector<PC_o3d_ptr>& clusters,
    //     const std::vector<open3d::geometry::AxisAlignedBoundingBox>& bounding_boxes);


    // 5. visualize bounding boxes on the original poitn cloud
    void visualizeBoundingBoxes(
        const std::shared_ptr<open3d::geometry::PointCloud>& original_pc,
        const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes);
        // {

        // // create a vector to store geometries
        // std::vector <std::shared_ptr <const open3d::geometry::Geometry>> geometries ;

        // // add the original point cloud
        // geometries.push_back(original_pc);


        // // add all bounding boxes
        // for (const auto& obb : bounding_boxes)
        // {
        //     auto obb_mesh = open3d::geometry::TriangleMesh::CreateFromOrientedBoundingBox(obb);
        //     obb_mesh->PaintUniformColor({1.0 , 0.0 , 0.0});
        //     geometries.push_back(obb_mesh);
        // }

        // //open3d visualization
        // open3d::visualization::DrawGeometries(geometries,  "Bounding Boxes on original point cloud"); 

        // }








};




#endif //GEOMETRYPROCESSOR_H
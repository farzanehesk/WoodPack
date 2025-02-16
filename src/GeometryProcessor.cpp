#include "../include/GeometryProcessor.hpp"
#include <open3d/Open3D.h>
#include <iostream>

 GeometryProcessor::GeometryProcessor() {} 
 GeometryProcessor::~GeometryProcessor() {} 

 std::vector<open3d::geometry::OrientedBoundingBox> GeometryProcessor::computeOrientedBoundingBoxes(  
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& clusters) 
 { 
 std::vector<open3d::geometry::OrientedBoundingBox> bounding_boxes; 
 for (const auto& cluster : clusters) 
    { 
    if (cluster->points_.empty()) 
    { 
        continue; // Skip empty clusters 
    } 

    auto obb = cluster->GetOrientedBoundingBox(); // Compute OBB 
    obb.color_ = Eigen::Vector3d(1, 0, 0); // Set OBB color (red) 
    bounding_boxes.push_back(obb); 
    } 
    
return bounding_boxes; 
}


///
// Visualize clusters with precomputed bounding boxes 

void GeometryProcessor::visualizeBoundingBoxes( 
const std::vector<PC_o3d_ptr>& clusters, 
const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) 
{ 
std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometry_list; 

for (size_t i = 0; i < clusters.size(); ++i) 
{ 
if (!clusters[i]->points_.empty()) 
{ 
geometry_list.push_back(clusters[i]); // Add cluster point cloud 
auto obb = std::make_shared<open3d::geometry::OrientedBoundingBox>(bounding_boxes[i]); 
obb->color_ = Eigen::Vector3d(1, 0, 0); // Red color for bounding boxes 
geometry_list.push_back(obb); 
} 
}
 // Open3D visualization
open3d::visualization::DrawGeometries(geometry_list, "Cluster Bounding Boxes"); 
}


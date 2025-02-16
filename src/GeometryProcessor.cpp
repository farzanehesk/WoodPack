#include "../include/GeometryProcessor.hpp"
#include <open3d/Open3D.h>
#include <iostream>
#include <Eigen/Eigenvalues> // For SelfAdjointEigenSolver


 GeometryProcessor::GeometryProcessor() {} 
 GeometryProcessor::~GeometryProcessor() {} 

//  std::vector<open3d::geometry::OrientedBoundingBox> GeometryProcessor::computeOrientedBoundingBoxes(  
//     const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& clusters) 
//  { 
//  std::vector<open3d::geometry::OrientedBoundingBox> bounding_boxes; 
//  for (const auto& cluster : clusters) 
//     { 
//     if (cluster->points_.empty()) 
//     { 
//         continue; // Skip empty clusters 
//     } 

//     auto obb = cluster->GetOrientedBoundingBox(); // Compute OBB 
//     obb.color_ = Eigen::Vector3d(1, 0, 0); // Set OBB color (red) 
//     bounding_boxes.push_back(obb); 
//     } 
    
// return bounding_boxes; 
// }






std::vector<open3d::geometry::OrientedBoundingBox> GeometryProcessor::computeOrientedBoundingBoxes(
    const std::vector<PC_o3d_ptr>& clusters) 
{
 std::vector<open3d::geometry::OrientedBoundingBox> bounding_boxes; 
 bounding_boxes.reserve(clusters.size()); 

for (const auto& cluster : clusters)
    { if (!cluster->points_.empty()) 
    {
        // Convert Open3D point cloud to Eigen matrix
        Eigen::MatrixXd data(cluster->points_.size(),3);
        for (size_t i = 0; i < cluster->points_.size(); ++i) { data.row(i) = cluster->points_[i].transpose(); } 


        // Step 1: Compute Mean (Centroid) 
        Eigen::Vector3d mean = data.colwise().mean(); 

        // Step 2: Compute Covariance Matrix 
        Eigen::MatrixXd centered = data.rowwise() - mean.transpose(); 
        Eigen::Matrix3d covariance = (centered.adjoint() * centered) / double(cluster->points_.size()); 

        // Step 3: Perform PCA (Eigen Decomposition)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance); 
        Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors(); // Columns are principal axes 

        // Step 4: Ensure right-handed coordinate system 
        if(eigenvectors.determinant() < 0) 
        {
            eigenvectors.col(0) = -eigenvectors.col(0);
        } 

        // Step 5: Transform points to PCA-aligned space 
        std::vector<Eigen::Vector3d> transformed_points; 
        transformed_points.reserve(cluster->points_.size()); 
        for (const auto& point : cluster->points_){ transformed_points.push_back(eigenvectors.transpose() * (point - mean)); }

        // Step 6: Compute min/max bounds in PCA-aligned space 
        Eigen::Vector3d min_bound = transformed_points.front(); 
        Eigen::Vector3d max_bound = transformed_points.front();
        for (const auto& p : transformed_points) { 
        min_bound = min_bound.cwiseMin(p); 
        max_bound = max_bound.cwiseMax(p); } 

        // Step 7: Create OBB with PCA rotation 
        auto obb = open3d::geometry::OrientedBoundingBox(); 
        obb.center_ = mean; obb.extent_ = (
        max_bound - min_bound).cwiseAbs(); 
        obb.R_ = eigenvectors;  // Assign PCA rotation matrix 
        bounding_boxes.push_back(obb);

    }
    } 
return bounding_boxes; 
}







///
// // Visualize clusters with precomputed bounding boxes 

// void GeometryProcessor::visualizeBoundingBoxes( 
// const std::vector<PC_o3d_ptr>& clusters, 
// const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) 
// { 
// std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometry_list; 

// for (size_t i = 0; i < clusters.size(); ++i) 
// { 
// if (!clusters[i]->points_.empty()) 
// { 
// geometry_list.push_back(clusters[i]); // Add cluster point cloud 
// auto obb = std::make_shared<open3d::geometry::OrientedBoundingBox>(bounding_boxes[i]); 
// obb->color_ = Eigen::Vector3d(1, 0, 0); // Red color for bounding boxes 
// geometry_list.push_back(obb); 
// } 
// }
//  // Open3D visualization
// open3d::visualization::DrawGeometries(geometry_list, "Cluster Bounding Boxes"); 
// }




///////////////////////////////////////////
// void visualizeBoundingBoxes(
//         const std::shared_ptr<open3d::geometry::PointCloud>& original_pc,
//         const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes)
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


// // 3. Compute axis-aligned bounding boxes for a vector of point clouds
// std::vector<open3d::geometry::AxisAlignedBoundingBox> GeometryProcessor::computeAxisAlignedBoundingBoxes(
//     const std::vector<PC_o3d_ptr>& clusters) {
//     std::vector<open3d::geometry::AxisAlignedBoundingBox> bounding_boxes;

//     for (const auto& cluster : clusters) {
//         if (cluster->points_.empty()) {
//             continue; // Skip empty clusters
//         }

//         // Compute AABB
//         auto aabb = cluster->GetAxisAlignedBoundingBox();
//         aabb.color_ = Eigen::Vector3d(1, 0, 0); // Set AABB color (red)
//         bounding_boxes.push_back(aabb);
//     }

//     return bounding_boxes;
// }

// // 4. Visualize clusters with precomputed AABBs
// void GeometryProcessor::visualizeBoundingBoxes(
//     const std::vector<PC_o3d_ptr>& clusters,
//     const std::vector<open3d::geometry::AxisAlignedBoundingBox>& bounding_boxes) 
//     {
//     std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometry_list;

//     for (size_t i = 0; i < clusters.size(); ++i) {
//         if (!clusters[i]->points_.empty()) {
//             geometry_list.push_back(clusters[i]); // Add cluster point cloud
//             auto aabb = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(bounding_boxes[i]);
//             aabb->color_ = Eigen::Vector3d(1, 0, 0); // Red color for bounding boxes
//             geometry_list.push_back(aabb);
//         }
//     }

//     // Open3D visualization
//     open3d::visualization::DrawGeometries(geometry_list, "Cluster AABBs", 640, 480, 50, 50, true);
// }
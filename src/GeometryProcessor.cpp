#include "../include/GeometryProcessor.hpp"
#include <open3d/Open3D.h>
#include <iostream>
#include <Eigen/Eigenvalues> // For SelfAdjointEigenSolver
#include <cmath>
#include <Eigen/Dense>
#include <random>
#include <chrono>
#include <iomanip>



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





// 1. Method to compute oriented bounding boxes for a vector of point clouds 
// Implementing PCA-based Oriented Bounding Box (OBB) computation for better alignment with object orientation
// std::vector<open3d::geometry::OrientedBoundingBox> GeometryProcessor::computeOrientedBoundingBoxes(
//     const std::vector<PC_o3d_ptr>& clusters) 
// {
//  std::vector<open3d::geometry::OrientedBoundingBox> bounding_boxes; 
//  bounding_boxes.reserve(clusters.size()); 

// for (const auto& cluster : clusters)
//     { if (!cluster->points_.empty()) 
//     {
//         // Convert Open3D point cloud to Eigen matrix
//         Eigen::MatrixXd data(cluster->points_.size(),3);
//         for (size_t i = 0; i < cluster->points_.size(); ++i) { data.row(i) = cluster->points_[i].transpose(); } 


//         // Step 1: Compute Mean (Centroid) 
//         Eigen::Vector3d mean = data.colwise().mean(); 

//         // Step 2: Compute Covariance Matrix 
//         Eigen::MatrixXd centered = data.rowwise() - mean.transpose(); 
//         Eigen::Matrix3d covariance = (centered.adjoint() * centered) / double(cluster->points_.size()); 

//         // Step 3: Perform PCA (Eigen Decomposition)
//         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance); 
//         Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors(); // Columns are principal axes 

//         // Step 4: Ensure right-handed coordinate system 
//         if(eigenvectors.determinant() < 0) 
//         {
//             eigenvectors.col(0) = -eigenvectors.col(0);
//         } 

//         // Step 5: Transform points to PCA-aligned space 
//         std::vector<Eigen::Vector3d> transformed_points; 
//         transformed_points.reserve(cluster->points_.size()); 
//         for (const auto& point : cluster->points_){ transformed_points.push_back(eigenvectors.transpose() * (point - mean)); }

//         // Step 6: Compute min/max bounds in PCA-aligned space 
//         Eigen::Vector3d min_bound = transformed_points.front(); 
//         Eigen::Vector3d max_bound = transformed_points.front();
//         for (const auto& p : transformed_points) { 
//         min_bound = min_bound.cwiseMin(p); 
//         max_bound = max_bound.cwiseMax(p); } 

//         // Step 7: Create OBB with PCA rotation 
//         auto obb = open3d::geometry::OrientedBoundingBox(); 
//         obb.center_ = mean; obb.extent_ = (
//         max_bound - min_bound).cwiseAbs(); 
//         obb.R_ = eigenvectors;  // Assign PCA rotation matrix 
//         bounding_boxes.push_back(obb);

//     }
//     } 
// return bounding_boxes; 
// }

// std::vector<open3d::geometry::OrientedBoundingBox> GeometryProcessor::computeOrientedBoundingBoxes(
//     const std::vector<PC_o3d_ptr>& clusters) 
// {
//     std::vector<open3d::geometry::OrientedBoundingBox> bounding_boxes;
//     bounding_boxes.reserve(clusters.size());

//     for (const auto& cluster : clusters) {
//         if (cluster->points_.empty()) continue;

//         // Convert Open3D point cloud to Eigen matrix
//         Eigen::MatrixXd data(cluster->points_.size(), 3);
//         for (size_t i = 0; i < cluster->points_.size(); ++i) {
//             data.row(i) = cluster->points_[i].transpose();
//         }

//         // Compute centroid
//         Eigen::Vector3d centroid = data.colwise().mean();
//         Eigen::MatrixXd centered = data.rowwise() - centroid.transpose();

//         // Compute covariance matrix
//         Eigen::Matrix3d covariance = (centered.transpose() * centered) / double(cluster->points_.size());

//         // Eigen decomposition (PCA)
//         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
//         Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors(); // Principal axes

//         // Ensure right-handed coordinate system
//         if (eigenvectors.determinant() < 0) {
//             eigenvectors.col(0) = -eigenvectors.col(0);
//         }

//         // Ensure correct alignment of X, Y, Z axes
//         Eigen::Vector3d x_axis = eigenvectors.col(0);
//         Eigen::Vector3d y_axis = eigenvectors.col(1);
//         Eigen::Vector3d z_axis = eigenvectors.col(2);

//         // Fix: Ensure Z-axis always points upward
//         Eigen::Vector3d global_up(0, 0, 1);
//         if (z_axis.dot(global_up) < 0) {
//             z_axis = -z_axis;
//             x_axis = -x_axis;
//         }

//         // Project points onto PCA space
//         std::vector<Eigen::Vector3d> transformed_points;
//         transformed_points.reserve(cluster->points_.size());
//         for (const auto& point : cluster->points_) {
//             transformed_points.push_back(eigenvectors.transpose() * (point - centroid));
//         }

//         // Compute min/max bounds in PCA-aligned space
//         Eigen::Vector3d min_bound = transformed_points.front();
//         Eigen::Vector3d max_bound = transformed_points.front();
//         for (const auto& p : transformed_points) {
//             min_bound = min_bound.cwiseMin(p);
//             max_bound = max_bound.cwiseMax(p);
//         }

//         // Get extents and reorder axes (Ensure X is the shortest, Y is the longest)
//         Eigen::Vector3d extents = (max_bound - min_bound).cwiseAbs();
//         std::array<std::pair<double, int>, 3> sorted_axes = {
//             std::make_pair(extents.x(), 0),
//             std::make_pair(extents.y(), 1),
//             std::make_pair(extents.z(), 2)
//         };

//         std::sort(sorted_axes.begin(), sorted_axes.end());

//         Eigen::Matrix3d corrected_R;
//         corrected_R.col(0) = eigenvectors.col(sorted_axes[0].second);  // X (shortest)
//         corrected_R.col(1) = eigenvectors.col(sorted_axes[1].second);  // Y (longest)
//         corrected_R.col(2) = eigenvectors.col(sorted_axes[2].second);  // Z (always up)

//         // Create final Oriented Bounding Box (OBB)
//         auto obb = open3d::geometry::OrientedBoundingBox();
//         obb.center_ = centroid;
//         obb.extent_ = Eigen::Vector3d(sorted_axes[0].first, sorted_axes[1].first, sorted_axes[2].first);
//         obb.R_ = corrected_R;

//         bounding_boxes.push_back(obb);
//     }
//     return bounding_boxes;
// }



std::vector<open3d::geometry::OrientedBoundingBox> GeometryProcessor::computeOrientedBoundingBoxes(
    const std::vector<PC_o3d_ptr>& clusters) 
{
    std::vector<open3d::geometry::OrientedBoundingBox> bounding_boxes;
    bounding_boxes.reserve(clusters.size());

    for (const auto& cluster : clusters) {
        if (cluster->points_.empty()) continue;

        // Convert Open3D point cloud to Eigen matrix
        Eigen::MatrixXd data(cluster->points_.size(), 3);
        for (size_t i = 0; i < cluster->points_.size(); ++i) {
            data.row(i) = cluster->points_[i].transpose();
        }

        // Compute centroid
        Eigen::Vector3d centroid = data.colwise().mean();
        Eigen::MatrixXd centered = data.rowwise() - centroid.transpose();

        // Compute covariance matrix
        Eigen::Matrix3d covariance = (centered.transpose() * centered) / double(cluster->points_.size());

        // PCA: eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
        Eigen::Matrix3d eigvecs = eigen_solver.eigenvectors(); // principal axes

        // Ensure right-handed coordinate system
        if (eigvecs.determinant() < 0) {
            eigvecs.col(0) *= -1;
        }

        // Project points into PCA space
        std::vector<Eigen::Vector3d> transformed_points;
        transformed_points.reserve(cluster->points_.size());
        for (const auto& point : cluster->points_) {
            transformed_points.push_back(eigvecs.transpose() * (point - centroid));
        }

        // Get min and max bounds
        Eigen::Vector3d min_bound = transformed_points.front();
        Eigen::Vector3d max_bound = transformed_points.front();
        for (const auto& p : transformed_points) {
            min_bound = min_bound.cwiseMin(p);
            max_bound = max_bound.cwiseMax(p);
        }

        Eigen::Vector3d extents = max_bound - min_bound;

        // Sort extents to match your convention: Z (shortest), X (width), Y (length)
        std::array<std::pair<double, int>, 3> dims = {{
            std::make_pair(extents[0], 0),
            std::make_pair(extents[1], 1),
            std::make_pair(extents[2], 2)
        }};
        std::sort(dims.begin(), dims.end()); // ascending

        int z_idx = dims[0].second; // shortest = thickness = Z
        int x_idx = dims[1].second; // width = X
        int y_idx = dims[2].second; // longest = length = Y

        Eigen::Matrix3d reordered_axes;
        reordered_axes.col(0) = eigvecs.col(x_idx); // X - width
        reordered_axes.col(1) = eigvecs.col(y_idx); // Y - length
        reordered_axes.col(2) = eigvecs.col(z_idx); // Z - thickness

        // Ensure Z axis points upward
        if (reordered_axes.col(2).dot(Eigen::Vector3d(0, 0, 1)) < 0) {
            reordered_axes.col(2) *= -1;
            reordered_axes.col(1) *= -1; // maintain right-handed system
        }

        // Get extents in correct order
        Eigen::Vector3d final_extents;
        final_extents.x() = extents[x_idx]; // width
        final_extents.y() = extents[y_idx]; // length
        final_extents.z() = extents[z_idx]; // thickness

        // Build bounding box
        open3d::geometry::OrientedBoundingBox obb;
        obb.center_ = centroid;
        obb.R_ = reordered_axes;
        obb.extent_ = final_extents;

        bounding_boxes.push_back(obb);
    }

    return bounding_boxes;
}

////////////////////////////////////////////////////////////////////////////
// std::vector<std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>>
// GeometryProcessor::computeOrientedBoundingBoxesWithClouds(const std::vector<PC_o3d_ptr>& clusters , bool debug)
// {
//     std::vector<std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>> result;
//     result.reserve(clusters.size());

//     for (const auto& cluster : clusters) {
//         if (cluster->points_.empty()) continue;

//         // Convert Open3D point cloud to Eigen matrix
//         Eigen::MatrixXd data(cluster->points_.size(), 3);
//         for (size_t i = 0; i < cluster->points_.size(); ++i) {
//             data.row(i) = cluster->points_[i].transpose();
//         }

//         // Compute centroid
//         Eigen::Vector3d centroid = data.colwise().mean();
//         Eigen::MatrixXd centered = data.rowwise() - centroid.transpose();

//         // Compute covariance matrix
//         Eigen::Matrix3d covariance = (centered.transpose() * centered) / double(cluster->points_.size());

//         // PCA: eigen decomposition
//         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
//         Eigen::Matrix3d eigvecs = eigen_solver.eigenvectors(); // principal axes

//         // Ensure right-handed coordinate system
//         if (eigvecs.determinant() < 0) {
//             eigvecs.col(0) *= -1;
//         }

//         // Project points into PCA space
//         std::vector<Eigen::Vector3d> transformed_points;
//         transformed_points.reserve(cluster->points_.size());
//         for (const auto& point : cluster->points_) {
//             transformed_points.push_back(eigvecs.transpose() * (point - centroid));
//         }

//         // Get min and max bounds
//         Eigen::Vector3d min_bound = transformed_points.front();
//         Eigen::Vector3d max_bound = transformed_points.front();
//         for (const auto& p : transformed_points) {
//             min_bound = min_bound.cwiseMin(p);
//             max_bound = max_bound.cwiseMax(p);
//         }

//         Eigen::Vector3d extents = max_bound - min_bound;

//         // Sort extents to match your convention: Z (shortest), X (width), Y (length)
//         std::array<std::pair<double, int>, 3> dims = {{
//             std::make_pair(extents[0], 0),
//             std::make_pair(extents[1], 1),
//             std::make_pair(extents[2], 2)
//         }};
//         std::sort(dims.begin(), dims.end()); // ascending

//         int z_idx = dims[0].second; // shortest = thickness = Z
//         int x_idx = dims[1].second; // width = X
//         int y_idx = dims[2].second; // longest = length = Y

//         Eigen::Matrix3d reordered_axes;
//         reordered_axes.col(0) = eigvecs.col(x_idx); // X - width
//         reordered_axes.col(1) = eigvecs.col(y_idx); // Y - length
//         reordered_axes.col(2) = eigvecs.col(z_idx); // Z - thickness

//         // Ensure Z axis points upward
//         if (reordered_axes.col(2).dot(Eigen::Vector3d(0, 0, 1)) < 0) {
//             reordered_axes.col(2) *= -1;
//             reordered_axes.col(1) *= -1; // maintain right-handed system
//         }

//         // Get extents in correct order
//         Eigen::Vector3d final_extents;
//         final_extents.x() = extents[x_idx]; // width
//         final_extents.y() = extents[y_idx]; // length
//         final_extents.z() = extents[z_idx]; // thickness

//         // Build bounding box
//         open3d::geometry::OrientedBoundingBox obb;
//         obb.center_ = centroid;
//         obb.R_ = reordered_axes;
//         obb.extent_ = final_extents;


//         // Save the pair (bounding box, point cloud)
//         result.emplace_back(obb, cluster);
//     }

//     return result;
// }


std::vector<std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>>
GeometryProcessor::computeOrientedBoundingBoxesWithClouds(const std::vector<PC_o3d_ptr>& clusters, bool debug)
{
    std::vector<std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>> result;
    result.reserve(clusters.size());

    for (size_t idx = 0; idx < clusters.size(); ++idx) {
        const auto& cluster = clusters[idx];
        if (cluster->points_.empty()) continue;

        Eigen::MatrixXd data(cluster->points_.size(), 3);
        for (size_t i = 0; i < cluster->points_.size(); ++i) {
            data.row(i) = cluster->points_[i].transpose();
        }

        Eigen::Vector3d centroid = data.colwise().mean();
        Eigen::MatrixXd centered = data.rowwise() - centroid.transpose();
        Eigen::Matrix3d covariance = (centered.transpose() * centered) / double(cluster->points_.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
        Eigen::Matrix3d eigvecs = eigen_solver.eigenvectors();

        if (eigvecs.determinant() < 0) {
            eigvecs.col(0) *= -1;
        }

        std::vector<Eigen::Vector3d> transformed_points;
        transformed_points.reserve(cluster->points_.size());
        for (const auto& point : cluster->points_) {
            transformed_points.push_back(eigvecs.transpose() * (point - centroid));
        }

        Eigen::Vector3d min_bound = transformed_points.front();
        Eigen::Vector3d max_bound = transformed_points.front();
        for (const auto& p : transformed_points) {
            min_bound = min_bound.cwiseMin(p);
            max_bound = max_bound.cwiseMax(p);
        }

        Eigen::Vector3d extents = max_bound - min_bound;

        std::array<std::pair<double, int>, 3> dims = {{
            {extents[0], 0},
            {extents[1], 1},
            {extents[2], 2}
        }};
        std::sort(dims.begin(), dims.end());

        int z_idx = dims[0].second;
        int x_idx = dims[1].second;
        int y_idx = dims[2].second;

        Eigen::Matrix3d reordered_axes;
        reordered_axes.col(0) = eigvecs.col(x_idx);
        reordered_axes.col(1) = eigvecs.col(y_idx);
        reordered_axes.col(2) = eigvecs.col(z_idx);

        if (reordered_axes.col(2).dot(Eigen::Vector3d(0, 0, 1)) < 0) {
            reordered_axes.col(2) *= -1;
            reordered_axes.col(1) *= -1;
        }

        Eigen::Vector3d final_extents;
        final_extents.x() = extents[x_idx];
        final_extents.y() = extents[y_idx];
        final_extents.z() = extents[z_idx];

        open3d::geometry::OrientedBoundingBox obb;
        obb.center_ = centroid;
        obb.R_ = reordered_axes;
        obb.extent_ = final_extents;
        obb.extent_.z() = 0.014;  // set thickness to a fixed value

        if (debug) {
            std::cout << "Cluster " << idx << " Bounding Box Extents (W x L x T): "
                      << final_extents.x() << " x " << final_extents.y() << " x " << final_extents.z() << std::endl;
        }

        result.emplace_back(obb, cluster);
    }

    return result;
}




////////////////////////////////////////
// computeMinimalOrientedBoundingBoxes
std::vector<open3d::geometry::OrientedBoundingBox> GeometryProcessor::computeMinimalOrientedBoundingBoxes(
    const std::vector<PC_o3d_ptr>& clusters) {

    std::vector<open3d::geometry::OrientedBoundingBox> oriented_bboxes;

    for (const auto& cluster : clusters) {
        open3d::geometry::OrientedBoundingBox bbox = cluster->GetOrientedBoundingBox();

        // Get the lengths of the bounding box axes (the diagonal vectors of the bounding box)
        Eigen::Vector3d x_axis = bbox.R_.col(0);
        Eigen::Vector3d y_axis = bbox.R_.col(1);
        Eigen::Vector3d z_axis = bbox.R_.col(2);

        // Compute the lengths of each axis
        double x_len = x_axis.norm();
        double y_len = y_axis.norm();
        double z_len = z_axis.norm();

        // Check if the shorter axis is not aligned with the X axis
        if (x_len > y_len) {
            // Swap X and Y if Y is the shorter axis
            std::swap(x_axis, y_axis);
        }
        else if (x_len > z_len) {
            // Swap X and Z if Z is the shorter axis
            std::swap(x_axis, z_axis);
        }

        // Set the rotation matrix to align the shorter axis with the X-axis
        bbox.R_ = Eigen::Matrix3d::Identity();  // Reset rotation
        bbox.R_.col(0) = x_axis.normalized();
        bbox.R_.col(1) = y_axis.normalized();
        bbox.R_.col(2) = z_axis.normalized();

        // Store the corrected oriented bounding box
        oriented_bboxes.push_back(bbox);
    }

    return oriented_bboxes;
}



///////////////////////////////////////////////////////
///////////////////////////////////////////////////
// Function to visualize bounding boxes with axis indicators
void GeometryProcessor::VisualizeBoundingBoxesAxis(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) {

    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;

    for (const auto& box : bounding_boxes) {
        // Copy bounding box
        auto bbox = std::make_shared<open3d::geometry::OrientedBoundingBox>(box);
        bbox->color_ = Eigen::Vector3d(0, 0, 1);  // Blue color for bounding box

        geometries.push_back(bbox);

        // Extract bounding box rotation matrix
        Eigen::Matrix3d rotation = bbox->R_;

        // Define axis length for visualization
        double axis_length = 0.1;

        // Create a LineSet to represent the axes
        auto line_set = std::make_shared<open3d::geometry::LineSet>();

        // Add points for the origin and axis endpoints
        Eigen::Vector3d origin = bbox->center_;
        int start_idx = line_set->points_.size();
        line_set->points_.push_back(origin);
        line_set->points_.push_back(origin + axis_length * rotation.col(0)); // X-axis
        line_set->points_.push_back(origin + axis_length * rotation.col(1)); // Y-axis
        line_set->points_.push_back(origin + axis_length * rotation.col(2)); // Z-axis

        // Define lines to represent the X, Y, and Z axes
        line_set->lines_.push_back({start_idx, start_idx + 1});
        line_set->lines_.push_back({start_idx, start_idx + 2});
        line_set->lines_.push_back({start_idx, start_idx + 3});

        // Color the axes
        line_set->colors_.push_back(Eigen::Vector3d(1, 0, 0)); // Red for X-axis
        line_set->colors_.push_back(Eigen::Vector3d(0, 1, 0)); // Green for Y-axis
        line_set->colors_.push_back(Eigen::Vector3d(0, 0, 1)); // Blue for Z-axis

        geometries.push_back(line_set);
    }

    // Convert to const pointers for DrawGeometries
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> const_geometries;
    for (const auto& geom : geometries) {
        const_geometries.push_back(std::static_pointer_cast<const open3d::geometry::Geometry>(geom));
    }

    // Visualize bounding boxes and axes
    open3d::visualization::DrawGeometries(const_geometries, "Bounding Boxes with Axes");
}




//////////////////////////////////////////////////////////////////////////
// std::vector<open3d::geometry::OrientedBoundingBox> GeometryProcessor::computeMinimalOrientedBoundingBoxes(
//     const std::vector<PC_o3d_ptr>& clusters)
// {
//     std::vector<open3d::geometry::OrientedBoundingBox> bounding_boxes;

//     // Iterate over each cluster (point cloud)
//     for (const auto& cluster : clusters)
//     {
//         if (cluster->IsEmpty())
//         {
//             continue;  // Skip empty clusters
//         }

//         // Compute the minimal oriented bounding box for the cluster
//         open3d::geometry::OrientedBoundingBox obb = cluster->GetMinimalOrientedBoundingBox();

//         // Add the computed OBB to the result list
//         bounding_boxes.push_back(obb);
//     }

//     return bounding_boxes;
// }




////////////////////////////////////////////
// 2. Visualize clusters with precomputed bounding boxes 

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




///////////////////////////////////////////
// 5. visualize bounding boxes on the original poitn cloud
void GeometryProcessor::visualizeBoundingBoxesAndOriginalPc(
        const std::shared_ptr<open3d::geometry::PointCloud>& original_pc,
        const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes)
{

// create a vector to store geometries
std::vector <std::shared_ptr <const open3d::geometry::Geometry>> geometries ;

// add the original point cloud
geometries.push_back(original_pc);


// add all bounding boxes
for (const auto& obb : bounding_boxes)
{
        auto obb_ptr = std::make_shared<open3d::geometry::OrientedBoundingBox>(obb);
        obb_ptr->color_ = Eigen::Vector3d(1.0, 0.0, 0.0); // Set color to red
        geometries.push_back(obb_ptr);
}

//open3d visualization
open3d::visualization::DrawGeometries(geometries,  "Bounding Boxes on original point cloud"); 

}



///////////////////////////////////////////
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


///////////////////////////////////////////
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




//////////////////////////////////////////////
// 6.Method to getWidthsOfBoundingBoxes
std::vector <double> GeometryProcessor::getWidthsOfBoundingBoxes(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes)
    {
            std::vector<double> widths;
    widths.reserve(bounding_boxes.size());

    for (const auto& obb : bounding_boxes) {
        // Extract the width (first component of extent_)
        double width = obb.extent_.x();
        widths.push_back(width);
    }

    return widths;
    }

/////////////////////////////////////////////
// 7. Method to getDimensionsOfBoundingBoxes
std::vector<std::array<double, 3>> GeometryProcessor::getDimensionsOfBoundingBoxes(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) {
    std::vector<std::array<double, 3>> dimensions;
    dimensions.reserve(bounding_boxes.size());

    for (const auto& obb : bounding_boxes) {
        // Get the extent components
        double x = obb.extent_.x();
        double y = obb.extent_.y();
        double z = obb.extent_.z();

        // Sort the dimensions (smallest, middle, largest)
        std::array<double, 3> sorted_extent = {x, y, z};
        std::sort(sorted_extent.begin(), sorted_extent.end());

        dimensions.push_back(sorted_extent);
    }

    return dimensions;
}

////////////////////////////////////////////







/////////////////////////

Eigen::Vector3d GeometryProcessor::projectToPlane
(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, const Eigen::Vector3d& point_on_plane) 

{
// Vector from point on the plane to the point we want to project    
Eigen::Vector3d vector_to_plane = point - point_on_plane;
// Compute the distance of the point to the plane    
double distance_to_plane = vector_to_plane.dot(normal);
// Subtract the projection of the vector from the point to get the projected point    
return point - distance_to_plane * normal;}



////////////////////
// Projection function to ensure corners are projected onto the horizontal plane
Eigen::Vector3d GeometryProcessor::projectPointOntoPlane(const Eigen::Vector3d& point,
const Eigen::Vector3d& normal,
const Eigen::Vector3d& point_on_plane)
{
// Compute the projection of the point onto the plane defined by the normal and point_on_plane    
Eigen::Vector3d v = point - point_on_plane;
double distance = v.dot(normal);// distance from the point to the plane    
return point - distance * normal; // projected point
}


////////////////
Eigen::Vector3d GeometryProcessor::projectPointOntoXYPlane
(const Eigen::Vector3d& point) 
{
// Projecting the point onto the XY plane (z=0)    
return Eigen::Vector3d(point.x(), point.y(), 0);}



///////////////////
Eigen::Vector3d GeometryProcessor::projectToXYPlane(const Eigen::Vector3d& point) {
    return Eigen::Vector3d(point.x(), point.y(), 0);// Set the Z-coordinate to 0
    }


/////////////////




///////////////////////////////
///10.  Method to extract upper rectangles of bounding boxes
//this works
// std::vector<Rectangle> GeometryProcessor::extractUpperRectangles(

//     const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) {
//     std::vector<Rectangle> upper_rectangles;
//     for (const auto& box : bounding_boxes) {
//         // Step 1: Get all 8 corners of the bounding box
//         std::vector<Eigen::Vector3d> corners = box.GetBoxPoints();
//         // Step 2: Get the normal vector of the bounding box
//         Eigen::Vector3d normal = box.R_.col(2); // Z-axis of the bounding box

//         // Step 3: Define faces of the bounding box
//         std::vector<std::array<Eigen::Vector3d, 4>> faces
//         {
//             {corners[0], corners[1], corners[2], corners[3]} ,
//             {corners[0], corners[1], corners[5], corners[4]},
//             {corners[0], corners[1], corners[5], corners[4]},
//             {corners[2], corners[3], corners[7], corners[6]},
//             {corners[0], corners[3], corners[7], corners[4]},
//             {corners[1], corners[2], corners[6], corners[5]}

//         };

        
//         // Step 4: Find the largest face that is closest to horizontal

//         double max_area = 0;
//         std::array<Eigen::Vector3d, 4> upper_face;
//         for (const auto& face : faces) {
//             Eigen::Vector3d edge1 = face[1] - face[0];
//             Eigen::Vector3d edge2 = face[2] - face[0];
//             double area = edge1.cross(edge2).norm(); // Compute area
//             Eigen::Vector3d face_normal = edge1.cross(edge2).normalized();
//             // Ensure the face is roughly horizontal and has the largest area
//             if (std::abs(face_normal.z()) > 0.9 && area > max_area) {
//                 max_area = area;
//                 upper_face = face;
//             }
//         }

//         // Step 5: Ensure a valid upper face was found

//         if (max_area == 0) {
//             std::cerr << "Warning: No valid upper face found for a bounding box!" << std::endl;
//             continue;

//         }



//         // Step 6: Sort the corners in a consistent order
//         std::array<Eigen::Vector3d, 4> sorted_corners = Rectangle::sortCornersClockwise(upper_face);

//         // Step 7: Create the rectangle and store it
//         upper_rectangles.emplace_back(sorted_corners);

//     }

//     return upper_rectangles;
// }



///////////////////////////////////////////////////////////////
//// 11. Method to visualize rectangles on the original point cloud
void GeometryProcessor::visualizeRectangles(
    const std::vector<Rectangle>& rectangles,
    const std::shared_ptr<open3d::geometry::PointCloud>& original_pc) 
{
    // Create a LineSet for visualization
    auto line_set = std::make_shared<open3d::geometry::LineSet>();

    // Add edges of all rectangles
    for (const auto& rect : rectangles) {
        auto rect_lines = rect.getEdges();
        for (const auto& edge : rect_lines) {
            line_set->points_.push_back(edge.first);
            line_set->points_.push_back(edge.second);
            line_set->lines_.emplace_back(line_set->points_.size() - 2, line_set->points_.size() - 1);
        }
    }

    // If an original point cloud is provided, visualize it together with rectangles
    if (original_pc) {
        open3d::visualization::DrawGeometries({original_pc, line_set}, "Rectangles Visualization");
    } else {
        open3d::visualization::DrawGeometries({line_set}, "Rectangles Visualization");
    }
}


/////////////////////////////////////////////////////////////////////
// 16. Method to visualize rectangle corner points
void GeometryProcessor::visualizeRectangleEdgesWithLabels(const std::vector<Rectangle>& rectangles) {

    auto line_set = std::make_shared<open3d::geometry::LineSet>();
    auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2i> lines;
    std::vector<Eigen::Vector3d> colors; // Colors for points


    std::vector<Eigen::Vector3d> color_map = {
        {0.0, 0.0, 1.0}, // Blue for P1
        {1.0, 0.0, 0.0}, // Red for P2
        {0.0, 1.0, 0.0}, // Green for P3
        {1.0, 1.0, 0.0}  // Yellow for P4
    };


    int point_index = 0;

    for (const auto& rect : rectangles) {
        auto corners = rect.getSortedCorners();
        for (size_t i = 0; i < corners.size(); ++i) {
            points.push_back(corners[i]);
            colors.push_back(color_map[i]); // Assign color based on index
        }



        // Create edges (ensuring a closed loop)

        lines.push_back(Eigen::Vector2i(point_index, point_index + 1));
        lines.push_back(Eigen::Vector2i(point_index + 1, point_index + 2));
        lines.push_back(Eigen::Vector2i(point_index + 2, point_index + 3));
        lines.push_back(Eigen::Vector2i(point_index + 3, point_index)); 

        point_index += 4;
    }



    // Assign points and colors to the point cloud

    point_cloud->points_ = points;
    point_cloud->colors_ = colors; // Apply color to each point



    // Assign points and edges to the LineSet
    line_set->points_ = points;
    line_set->lines_ = lines;


    // Show visualization
    open3d::visualization::DrawGeometries({line_set, point_cloud}, "Rectangle Corners with Color-coded Order", 800, 600);

}








////////////////////////
/// 17. Method to obtain bounding box 3D planes and visualize them
std::vector<Eigen::Matrix4d> GeometryProcessor::getPlanesFromBoundingBoxes(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes, bool debug) {

    std::vector<Eigen::Matrix4d> planes;
    Eigen::Vector3d global_up(0, 0, 1); // Define a consistent upward direction

    for (size_t i = 0; i < bounding_boxes.size(); ++i) {
        const auto& box = bounding_boxes[i];

        // Extract origin and rotation
        Eigen::Vector3d origin = box.center_;
        Eigen::Matrix3d rotation = box.R_;

        // Extract the three principal axes
        Eigen::Vector3d x_axis = rotation.col(0);  // First principal axis
        Eigen::Vector3d y_axis = rotation.col(1);  // Second principal axis
        Eigen::Vector3d normal = rotation.col(2);  // Third principal axis (Z direction)

        // Debug output before correction
         if (debug) {
        std::cout << "Bounding Box " << i << " original axes:\n";
        std::cout << "X-axis: " << x_axis.transpose() << "\n";
        std::cout << "Y-axis: " << y_axis.transpose() << "\n";
        std::cout << "Z-axis (normal): " << normal.transpose() << "\n";
        std::cout << "Origin: " << origin.transpose() << "\n";
         }

        // Step 1: Check and correct normal direction
        if (fabs(normal.z()) < fabs(normal.x()) || fabs(normal.z()) < fabs(normal.y())) {
            global_up = Eigen::Vector3d(0, 1, 0); // Use Y if Z is small
        }
        if (normal.dot(global_up) < 0) { 
            normal = -normal;
            x_axis = -x_axis;
        }

        // Step 2: Identify shorter and longer edges
        double x_length = (box.extent_(0) < box.extent_(1)) ? box.extent_(0) : box.extent_(1);
        double y_length = (box.extent_(0) > box.extent_(1)) ? box.extent_(0) : box.extent_(1);

        if (x_length > y_length) {
            std::swap(x_axis, y_axis);
        }



        // Debug output after correction
         if (debug) {
        std::cout << "Bounding Box " << i << " corrected axes:\n";
        std::cout << "X-axis: " << x_axis.transpose() << "\n";
        std::cout << "Y-axis: " << y_axis.transpose() << "\n";
        std::cout << "Z-axis (normal): " << normal.transpose() << "\n\n";
         }

        // Step 3: Construct the transformation matrix
        Eigen::Matrix4d plane = Eigen::Matrix4d::Identity();
        plane.block<3, 1>(0, 0) = x_axis;
        plane.block<3, 1>(0, 1) = y_axis;
        plane.block<3, 1>(0, 2) = normal;
        plane.block<3, 1>(0, 3) = origin;

        planes.push_back(plane);

        // Print the final transformation matrix
        std::cout << "Final Transformation Matrix for Plane " << i << ":\n" << plane << "\n\n";
    }

    return planes;
}






///////////////////////////////////////////////////////////////////
// 18.  METHOD TO VISUALIZE PLANES
void GeometryProcessor::visualizePlanesOnBoundingBoxes(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes,
    const std::vector<Eigen::Matrix4d>& planes,
    const std::shared_ptr<open3d::geometry::PointCloud>& point_cloud) {

    // Initialize Open3D Visualizer
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("Bounding Boxes and Planes", 800, 600);

    // Add Point Cloud to Visualizer
    if (point_cloud) {
        vis.AddGeometry(point_cloud);
    }


    // Visualize Bounding Boxes
    for (const auto& box : bounding_boxes) {
        // Create Open3D OrientedBoundingBox object
        auto open3d_box = std::make_shared<open3d::geometry::OrientedBoundingBox>(box);
        open3d_box->color_ = Eigen::Vector3d(0, 0, 1);  // Blue color for the bounding box

        // Add the bounding box to the visualizer
        vis.AddGeometry(open3d_box);

    }

    // Visualize Planes
    auto line_set = std::make_shared<open3d::geometry::LineSet>();
    for (const auto& plane : planes) {
        // Step 1: Extract the origin and the axes (vectors from the transformation matrix)
        Eigen::Vector3d origin = plane.block<3,1>(0,3);
        Eigen::Vector3d x_axis = plane.block<3,1>(0,0);
        Eigen::Vector3d y_axis = plane.block<3,1>(0,1);
        Eigen::Vector3d z_axis = plane.block<3,1>(0,2);

        // Step 2: Define the axis endpoints for visualization (length = 0.1 for visibility)
        Eigen::Vector3d x_end = origin + 0.1 * x_axis;
        Eigen::Vector3d y_end = origin + 0.1 * y_axis;
        Eigen::Vector3d z_end = origin + 0.1 * z_axis;

        // Step 3: Add points for the origin and the axis endpoints
        int start_idx = line_set->points_.size();
        line_set->points_.push_back(origin);
        line_set->points_.push_back(x_end);
        line_set->points_.push_back(y_end);
        line_set->points_.push_back(z_end);

        // Step 4: Add lines for the X, Y, and Z axes (red, green, blue)
        line_set->lines_.push_back({start_idx, start_idx + 1});  // X-axis (red)
        line_set->lines_.push_back({start_idx, start_idx + 2});  // Y-axis (green)
        line_set->lines_.push_back({start_idx, start_idx + 3});  // Z-axis (blue)

        // Step 5: Color the axes accordingly

        line_set->colors_.push_back(Eigen::Vector3d(1, 0, 0)); // Red for X-axis
        line_set->colors_.push_back(Eigen::Vector3d(0, 1, 0)); // Green for Y-axis
        line_set->colors_.push_back(Eigen::Vector3d(0, 0, 1)); // Blue for Z-axis
    }



    // Add line set (planes) to the visualizer
    vis.AddGeometry(line_set);

    // Step 6: Start the visualizer

    vis.Run();
    vis.DestroyVisualizerWindow();

}




//////////////////////////////////////////
// 20. Function to create n random rectangles
// std::vector<Rectangle> GeometryProcessor::createRandomRectangles(int n, double fixed_length) {
//     std::vector<Rectangle> rectangles;
//     double min_width = 0.08;    // 8 cm
//     double max_width = 0.20;    // 20 cm
//     double surface_width = 2.0; // Example surface width (adjust as needed)
//     double surface_height = 2.0;// Example surface height (adjust as needed)
//     double min_gap = 0.02;      // Minimum gap between rectangles (2 cm)

//     std::srand(static_cast<unsigned int>(std::time(0))); // Seed random generator

//     for (int i = 0; i < n; ++i) {
//         bool valid_position = false;
//         std::array<Eigen::Vector3d, 4> corners;

//         for (int attempt = 0; attempt < 100; ++attempt) { // Try up to 100 times
//             double width = min_width + (max_width - min_width) * (std::rand() / double(RAND_MAX));
//             double x = min_gap + (surface_width - width - min_gap) * (std::rand() / double(RAND_MAX));
//             double y = min_gap + (surface_height - fixed_length - min_gap) * (std::rand() / double(RAND_MAX));

//             // Define the corners
//             Eigen::Vector3d p1(x, y, 0);
//             Eigen::Vector3d p2(x + width, y, 0);
//             Eigen::Vector3d p3(x + width, y + fixed_length, 0);
//             Eigen::Vector3d p4(x, y + fixed_length, 0);

//             corners = {p1, p2, p3, p4};

//             // Check for overlap with existing rectangles
//             bool overlap = false;
//             for (const auto& rect : rectangles) {
//                 auto existing_corners = rect.getSortedCorners();
//                 double ex_min_x = existing_corners[0].x();
//                 double ex_max_x = existing_corners[1].x();
//                 double ex_min_y = existing_corners[0].y();
//                 double ex_max_y = existing_corners[2].y();

//                 double new_min_x = p1.x();
//                 double new_max_x = p2.x();
//                 double new_min_y = p1.y();
//                 double new_max_y = p3.y();

//                 if (!(new_max_x + min_gap < ex_min_x || new_min_x > ex_max_x + min_gap ||
//                       new_max_y + min_gap < ex_min_y || new_min_y > ex_max_y + min_gap)) {
//                     overlap = true;
//                     break;
//                 }
//             }

//             if (!overlap) {
//                 valid_position = true;
//                 break;
//             }
//         }

//         if (valid_position) {
//             rectangles.emplace_back(Rectangle(corners));
//         }
//     }

//     return rectangles;
// }

// std::vector<Rectangle> GeometryProcessor::createRandomRectangles(int n, double fixed_length) {
//     std::vector<Rectangle> rectangles;
    
//     double min_width = 0.08;    // 8 cm
//     double max_width = 0.20;    // 20 cm
//     double min_gap = 0.02;      // Minimum gap between rectangles (2 cm)

//     // Estimate total area needed based on average rectangle size
//     double avg_width = (min_width + max_width) / 2.0;  
//     double avg_area = avg_width * fixed_length;        
//     double density_factor = 2.5; // Allow more space
//     double total_area_needed = n * avg_area * density_factor;
//     double surface_side = std::sqrt(total_area_needed); 

//     double surface_width = surface_side;
//     double surface_height = surface_side;

//     std::cout << "[INFO] Dynamic surface size: " << surface_width << "m x " << surface_height << "m\n";

//     std::srand(static_cast<unsigned int>(std::time(0))); // Seed random generator

//     for (int i = 0; i < n; ++i) {
//         bool valid_position = false;
//         std::array<Eigen::Vector3d, 4> corners;
//         double width = 0.0;  // Store width for printing
//         double computed_length = 0.0; // Store computed length

//         for (int attempt = 0; attempt < 500; ++attempt) {  
//             width = min_width + (max_width - min_width) * (std::rand() / double(RAND_MAX));
//             double x = min_gap + (surface_width - width - min_gap) * (std::rand() / double(RAND_MAX));
//             double y = min_gap + (surface_height - fixed_length - min_gap) * (std::rand() / double(RAND_MAX));

//             Eigen::Vector3d p1(x, y, 0);
//             Eigen::Vector3d p2(x + width, y, 0);
//             Eigen::Vector3d p3(x + width, y + fixed_length, 0);
//             Eigen::Vector3d p4(x, y + fixed_length, 0);

//             corners = {p1, p2, p3, p4};

//             // Check for overlap
//             bool overlap = false;
//             for (const auto& rect : rectangles) {
//                 auto existing_corners = rect.getSortedCorners();
//                 double ex_min_x = existing_corners[0].x();
//                 double ex_max_x = existing_corners[1].x();
//                 double ex_min_y = existing_corners[0].y();
//                 double ex_max_y = existing_corners[2].y();

//                 double new_min_x = p1.x();
//                 double new_max_x = p2.x();
//                 double new_min_y = p1.y();
//                 double new_max_y = p3.y();

//                 if (!(new_max_x + min_gap < ex_min_x || new_min_x > ex_max_x + min_gap ||
//                       new_max_y + min_gap < ex_min_y || new_min_y > ex_max_y + min_gap)) {
//                     overlap = true;
//                     break;
//                 }
//             }

//             if (!overlap) {
//                 valid_position = true;
//                 break;
//             }
//         }

//         if (valid_position) {
//             Rectangle rect(corners);
//             rectangles.push_back(rect);

//             // Compute actual length after creation
//             auto created_corners = rect.getSortedCorners();
//             computed_length = (created_corners[2] - created_corners[0]).norm(); // Length along y-axis

//             std::cout << "[DEBUG] Rectangle " << i + 1 << ": "
//                       << "Width = " << width << "m, "
//                       << "Expected Length = " << fixed_length << "m, "
//                       << "Computed Length = " << computed_length << "m\n";

//             if (std::abs(computed_length - fixed_length) > 1e-6) {
//                 std::cout << "[WARNING] Length mismatch detected!\n";
//             }
//         } else {
//             std::cout << "[WARNING] Could not place rectangle " << i + 1 << " after 500 attempts.\n";
//         }
//     }

//     std::cout << "[INFO] Created " << rectangles.size() << " rectangles out of " << n << " requested.\n";
//     return rectangles;
// }

// Function to generate 'n' rectangles with random widths and fixed length 'L'
// std::vector<Rectangle> GeometryProcessor::generateRectangles(int n, double L) {
//     std::vector<Rectangle> rectangles;
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> width_dist(0.08, 0.20); // Random width: 8cm - 20cm

//     Eigen::Vector3d base_origin(0, 0, 0); // Starting point for first rectangle
//     Eigen::Vector3d x_axis(1, 0, 0); // Width direction
//     Eigen::Vector3d y_axis(0, 1, 0); // Length direction

//     for (int i = 0; i < n; ++i) {
//         double width = width_dist(gen); // Generate random width

//         // Define the four corners of the rectangle
//         Eigen::Vector3d p1 = base_origin;
//         Eigen::Vector3d p2 = p1 + width * x_axis;
//         Eigen::Vector3d p3 = p2 + L * y_axis;
//         Eigen::Vector3d p4 = p1 + L * y_axis;

//         std::array<Eigen::Vector3d, 4> corners = {p1, p2, p3, p4};

//         // Create rectangle and add to the list
//         rectangles.emplace_back(Rectangle(corners));

//         // Compute actual length after creation
//         auto created_corners = rectangles.back().getSortedCorners();
//         double computed_length = (created_corners[2] - created_corners[0]).norm(); // Length along y-axis

//         // Print the computed length
//         std::cout << "Rectangle " << i + 1 << ": Width = " << width << "m, Expected Length = " << L 
//                   << "m, Computed Length = " << computed_length << "m" << std::endl;

//         // Move base_origin for the next rectangle (stacking horizontally)
//         base_origin += width * x_axis;
//     }

//     return rectangles;
// }



// std::vector<Rectangle> GeometryProcessor::generateRectangles(int n, double L) {
//     std::vector<Rectangle> rectangles;
//     std::random_device rd;
//     std::mt19937 gen(rd());
//     std::uniform_real_distribution<double> width_dist(0.08, 0.20); // Random width: 8cm - 20cm

//     Eigen::Vector3d base_origin(0, 0, 0); // Starting point for first rectangle
//     Eigen::Vector3d x_axis(1, 0, 0); // Width direction
//     Eigen::Vector3d y_axis(0, 1, 0); // Length direction

//     for (int i = 0; i < n; ++i) {
//         double width = width_dist(gen); // Generate random width

//         // Define the four corners of the rectangle with fixed length L
//         Eigen::Vector3d p1 = base_origin;
//         Eigen::Vector3d p2 = p1 + width * x_axis;
//         Eigen::Vector3d p3 = p2 + L * y_axis; // Fixed length L
//         Eigen::Vector3d p4 = p1 + L * y_axis; // Fixed length L

//         std::array<Eigen::Vector3d, 4> corners = {p1, p2, p3, p4};

//         // Create rectangle and add to the list
//         rectangles.emplace_back(Rectangle(corners));

//         // Compute actual length after creation
//         auto created_corners = rectangles.back().getSortedCorners();
//         double computed_length = (created_corners[2] - created_corners[0]).norm(); // Length along y-axis

//         // Print the computed length
//         std::cout << "Rectangle " << i + 1 << ": Width = " << width << "m, Expected Length = " << L 
//                   << "m, Computed Length = " << computed_length << "m" << std::endl;

//         // Move base_origin for the next rectangle (stacking horizontally)
//         base_origin += width * x_axis;
//     }

//     return rectangles;
// }




/////////////////////////////////
// Function to generate a random number between min and max
    double GeometryProcessor::getRandomWidth(double min, double max ) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }

    // Function to create 'n' bounding boxes and place them next to each other
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::createBoundingBoxes(int n , double fixed_length , bool debug = false) {
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> boxes;

    double min_width = 0.08;
    double max_width = 0.2;

    // Start position for the first box
    Eigen::Vector3d start_pos(-0.5, 0.3, 0.0);
    double current_x = start_pos.x();

    for (int i = 0; i < n; ++i) {
        double width = getRandomWidth(min_width, max_width);

        // Define center of bounding box
        Eigen::Vector3d center(current_x + width / 2.0, start_pos.y() + fixed_length / 2.0, start_pos.z());

        // Set half extents with width along x, length along y, and thickness along z
        Eigen::Vector3d half_extents(width / 2.0, fixed_length / 2.0, 0.0065);

        // Create oriented bounding box
        auto box = std::make_shared<open3d::geometry::OrientedBoundingBox>(
            center, Eigen::Matrix3d::Identity(), half_extents * 2);

        // Set color
        box->color_ = Eigen::Vector3d(1, 0, 0); // Red

        // Add to list
        boxes.push_back(box);

        // Compute actual dimensions
        double computed_width = box->extent_.x();  // Width along x-axis
        double computed_length = box->extent_.y(); // Length along y-axis

        // Print bounding box dimensions
        if(debug)
        {
            std::cout << "Bounding Box " << i + 1 << ": Width = " 
            << computed_width << "m, Expected Length = " 
            << fixed_length << "m, Computed Length = " 
            << computed_length << "m" << std::endl;

            // Print the center and extents of the bounding box before and after translation/rotation
            std::cout << "Center: " << box->GetCenter().transpose() << std::endl;
            std::cout << "Half Extents: " << box->extent_.transpose() << std::endl;
        }


        // Update position for next box (move in x-direction)
        current_x += width + 0.005; // Maintain 5mm gap
    }

    return boxes;
}



//////////////////////////////////////////////////
//19. Convert Rectangles to Open3D Bounding Boxes

std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::createBoundingBoxFromRectangle(
    const std::vector<Rectangle>& rects, double height) {
    
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> bounding_boxes;

    for (const auto& rect : rects) {
        Eigen::Vector3d center = rect.getCenter();
        Eigen::Vector3d normal = rect.getNormal();
        Eigen::Matrix3d rotation;

        // Retrieve edges
        auto edges = rect.getEdges();

        // Find longest edge
        Eigen::Vector3d x_axis;
        double max_length = 0.0;

        for (const auto& edge : edges) {
            Eigen::Vector3d edge_vector = edge.second - edge.first;
            double length = edge_vector.norm();
            if (length > max_length) {
                max_length = length;
                x_axis = edge_vector.normalized();
            }
        }

        // Compute Y-axis as orthogonal to X and Normal
        Eigen::Vector3d y_axis = normal.cross(x_axis).normalized();

        // Set rotation matrix
        rotation.col(0) = x_axis;
        rotation.col(1) = y_axis;
        rotation.col(2) = normal.normalized(); // Z-axis is the normal

        // Compute extents
        double width = rect.getWidth();
        double length = rect.getLength();

        auto bbox = std::make_shared<open3d::geometry::OrientedBoundingBox>(center, rotation, Eigen::Vector3d(width, length, height));
        bounding_boxes.push_back(bbox);
    }

    return bounding_boxes;
}
////////////////////////////////////////////////









////////////////////////////////////////////////
// 21. mapBetweenFrames



////////////////////////////////////////////////
// 22.transform_bounding_box
void GeometryProcessor::transform_bounding_box(std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bb,
                            const Eigen::Vector3d& translation,
                            const Eigen::Vector3d& rotation_axis,
                            double rotation_angle,
                            const Eigen::Vector3d& scale_center,
                            double scale_factor) {


    // Apply translation
    bb->Translate(translation);

    // Apply rotation
    Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(rotation_angle, rotation_axis).toRotationMatrix();
    bb->Rotate(rotation_matrix, bb->GetCenter());

    // Apply scaling
    bb->Scale(scale_factor, scale_center);


}


///////////////////////////////////////////////
void GeometryProcessor::transform_bounding_box_to_plane(
    std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bb,
    const Eigen::Vector3d& target_plane_normal,   // Target plane normal vector
    const Eigen::Vector3d& target_plane_point,    // A point on the target plane
    const Eigen::Vector3d& scale_center,          // Scaling center
    double scale_factor)                          // Scaling factor
{
    // 1. Calculate the translation vector
    Eigen::Vector3d current_center = bb->GetCenter();
    
    // Translation to align the center with the target plane
    Eigen::Vector3d translation = target_plane_point - current_center;
    
    // Apply the translation
    bb->Translate(translation);
    
    // 2. Calculate the rotation needed to align the bounding box's normal with the target plane's normal
    Eigen::Vector3d current_normal = bb->R_.col(2);  // Z-axis of bounding box (R_ is the rotation matrix)

    // Axis of rotation (cross product between the current normal and target plane normal)
    Eigen::Vector3d rotation_axis = current_normal.cross(target_plane_normal).normalized();
    
    // Angle of rotation (dot product between the current normal and target plane normal)
    double rotation_angle = std::acos(current_normal.dot(target_plane_normal));

    // Apply the rotation if the axis is non-zero (avoids degenerate case where the vectors are aligned)
    if (rotation_axis.norm() != 0) {  
        Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(rotation_angle, rotation_axis).toRotationMatrix();
        bb->Rotate(rotation_matrix, bb->GetCenter());
    }

    // 3. Apply scaling if necessary
    bb->Scale(scale_factor, scale_center);
}

////////////////////////////////////////////////
// 23. arrangeShingleRow
// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::arrangeFirstShingleRow(
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes,
//     double gap,
//     double max_length,
//     double rotation_angle) {  // Rotation for the entire row

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
//     Eigen::Vector3d current_position(0, 0, 0);  // Start at the origin

//     double previous_half_width = 0;  // Keeps track of half the previous box's width
//     double total_length = 0;  // Keeps track of the total accumulated length

//     Eigen::Vector3d last_box_right_edge(0, 0, 0);  // Stores the rightmost edge position
//     bool min_length_reached = false;  // Flag to check if we reached at least max_length

//     for (auto& bbox : bounding_boxes) {
//         // Compute the bounding box's extent
//         Eigen::Vector3d extent = bbox->extent_;

//         // Identify the longest axis (we want Y to be the longest)
//         int longest_axis = 0;  // Assume X is longest by default
//         if (extent.y() > extent.x() && extent.y() > extent.z()) {
//             longest_axis = 1;  // Y-axis is the longest
//         } else if (extent.z() > extent.x() && extent.z() > extent.y()) {
//             longest_axis = 2;  // Z-axis is the longest
//         }

//         // Rotate the bounding box so that the longest axis is aligned with Y
//         if (longest_axis == 1) {
//             // Y is already the longest axis, no rotation needed
//         } else if (longest_axis == 2) {
//             // If Z is the longest, rotate 90 degrees around Y to align Z with Y
//             transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 0), M_PI_2, bbox->GetCenter(), true);
//         }

//         // Now the longest axis is aligned with Y, and the shorter edges are along X and Z
//         extent = bbox->extent_;  // Get the new extent after rotation
//         double current_half_width = extent.x() / 2.0;  // Half of the width after rotation

//         // Predict next position before placing
//         double new_total_length = total_length + previous_half_width + gap + current_half_width;

//         // Allow placing shingles until we reach at least max_length
//         if (min_length_reached && new_total_length > max_length) {
//             break;  // Stop only after reaching at least max_length
//         }

//         // Correct placement: shift by previous half-width + gap + current half-width
//         current_position.x() += previous_half_width + gap + current_half_width;

//         // Translate the bounding box to align it along X-axis
//         Eigen::Vector3d translation = current_position - bbox->GetCenter();
//         transform_bounding_box(bbox, translation, Eigen::Vector3d(0, 1, 0), 0, Eigen::Vector3d(0, 0, 0), true);

//         // Store the rightmost edge position (center + half width in X direction)
//         last_box_right_edge = bbox->GetCenter() + Eigen::Vector3d(current_half_width, 0, 0);

//         // Add the bounding box to the list
//         arranged_bboxes.push_back(bbox);

//         // Update tracking variables
//         previous_half_width = current_half_width;
//         total_length = new_total_length;

//         // Mark that we have reached at least max_length
//         if (total_length >= max_length) {
//             min_length_reached = true;
//         }
//     }

//     // **Convert degrees to radians** (rotation_angle is in degrees)
//     double rotation_radians = rotation_angle * M_PI / 180.0;  // Convert to radians

//     // Debug print: Rotation angle in both degrees and radians
//     std::cout << "Applying rotation: " << rotation_angle << " degrees (" 
//           << rotation_radians << " radians)" << std::endl;

//     // **Apply the correct rotation to the entire row** (rotate each box around its center)
//     Eigen::Vector3d rotation_center(0, 0, 0);  // Rotation center at the origin (can be adjusted if needed)

//     // Apply a x-degree rotation around the Y-axis to make the longer edge align with the global X-axis
//     for (auto& bbox : arranged_bboxes) {
//         // Rotate the bounding box around its center
//         transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), -rotation_radians, bbox->GetCenter(), true);
//     }

//     // Debug print: Total length of the row from (0,0,0) to rightmost edge
//     std::cout << "Total row length: " << last_box_right_edge.x() << " meters" << std::endl;

//     // Debugging bounding box rotation
//     for (auto& bbox : arranged_bboxes) {
//         Eigen::Matrix3d rotation_matrix = bbox->R_;  // Get the rotation matrix

//         // Extract rotation angles (Euler angles)
//         double angle_x = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));  // Rotation around X
//         double angle_y = std::atan2(-rotation_matrix(2, 0),
//                                     std::sqrt(rotation_matrix(2, 1) * rotation_matrix(2, 1) + 
//                                               rotation_matrix(2, 2) * rotation_matrix(2, 2)));  // Rotation around Y
//         double angle_z = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));  // Rotation around Z

//         // Convert to degrees
//         double angle_x_degrees = angle_x * 180.0 / M_PI;
//         double angle_y_degrees = angle_y * 180.0 / M_PI;
//         double angle_z_degrees = angle_z * 180.0 / M_PI;

//         // std::cout << "Bounding Box Rotation Angles:" << std::endl;
//         // std::cout << "  Around X: " << angle_x << " radians (" << angle_x_degrees << " degrees)" << std::endl;
//         // std::cout << "  Around Y: " << angle_y << " radians (" << angle_y_degrees << " degrees)" << std::endl;
//         // std::cout << "  Around Z: " << angle_z << " radians (" << angle_z_degrees << " degrees)" << std::endl;
//     }

//     // Debug: First row top face
//     Eigen::Vector3d first_row_top_face = arranged_bboxes[0]->GetCenter();
//     first_row_top_face.z() += (arranged_bboxes[0]->extent_.z() / 2.0);
//     std::cout << "[DEBUG] First row top face (final): " << first_row_top_face.transpose() << std::endl;

//     return arranged_bboxes;
// }


// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::arrangeFirstShingleRow(
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes,
//     double gap,
//     double max_length,
//     double rotation_angle) {  // Rotation for the entire row

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
//     Eigen::Vector3d current_position(0, 0, 0);  // Start at the origin

//     double previous_half_width = 0;  // Keeps track of half the previous box's width
//     double total_length = 0;  // Keeps track of the total accumulated length

//     Eigen::Vector3d last_box_right_edge(0, 0, 0);  // Stores the rightmost edge position
//     bool min_length_reached = false;  // Flag to check if we reached at least max_length


//     for (size_t i = 0; i < bounding_boxes.size(); ++i) {
//     auto& bbox = bounding_boxes[i];

//     // Compute the bounding box's extent
//     Eigen::Vector3d extent = bbox->extent_;
//     double current_half_width = extent.x() / 2.0;  // Half of the width after rotation

//     // Predict total length if this box is added
//     double new_total_length = total_length + previous_half_width + gap + current_half_width;

//     // **Check if adding this box significantly exceeds 1m**
//     if (min_length_reached && new_total_length > max_length) {
//         // Find a better alternative (smallest overshoot or closest to 1m)
//         double best_fit_length = new_total_length;
//         size_t best_fit_index = i;  // Default to current box

//         for (size_t j = i + 1; j < bounding_boxes.size(); ++j) {
//             auto& alternative_bbox = bounding_boxes[j];
//             double alt_half_width = alternative_bbox->extent_.x() / 2.0;
//             double alt_new_total_length = total_length + previous_half_width + gap + alt_half_width;

//             // Choose the box that minimizes the overshoot
//             if (alt_new_total_length >= max_length && alt_new_total_length < best_fit_length) {
//                 best_fit_length = alt_new_total_length;
//                 best_fit_index = j;
//             }
//         }

//         // If a better option was found, use it
//         if (best_fit_index != i) {
//             std::swap(bounding_boxes[i], bounding_boxes[best_fit_index]);
//             bbox = bounding_boxes[i];  // Update bbox reference
//             current_half_width = bbox->extent_.x() / 2.0;
//             new_total_length = total_length + previous_half_width + gap + current_half_width;
//         }

//         // If still exceeding, break out after ensuring at least 1m is reached
//         if (new_total_length > max_length) break;
//     }

//     // Update position
//     current_position.x() += previous_half_width + gap + current_half_width;

//     // Translate the bounding box to align along X-axis
//     Eigen::Vector3d translation = current_position - bbox->GetCenter();
//     transform_bounding_box(bbox, translation, Eigen::Vector3d(0, 1, 0), 0, Eigen::Vector3d(0, 0, 0), true);

//     // Store rightmost edge position
//     last_box_right_edge = bbox->GetCenter() + Eigen::Vector3d(current_half_width, 0, 0);

//     // Add the bounding box to the list
//     arranged_bboxes.push_back(bbox);

//     // Update tracking variables
//     previous_half_width = current_half_width;
//     total_length = new_total_length;

//     // Mark that at least 1m has been reached
//     if (total_length >= max_length) min_length_reached = true;
// }

//     // **Convert degrees to radians** (rotation_angle is in degrees)
//     double rotation_radians = rotation_angle * M_PI / 180.0;  // Convert to radians

//     // Debug print: Rotation angle in both degrees and radians
//     std::cout << "Applying rotation: " << rotation_angle << " degrees (" 
//           << rotation_radians << " radians)" << std::endl;

//     // **Apply the correct rotation to the entire row** (rotate each box around its center)
//     Eigen::Vector3d rotation_center(0, 0, 0);  // Rotation center at the origin (can be adjusted if needed)

//     // Apply a x-degree rotation around the Y-axis to make the longer edge align with the global X-axis
//     for (auto& bbox : arranged_bboxes) {
//         // Rotate the bounding box around its center
//         transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), -rotation_radians, bbox->GetCenter(), true);
//     }

//     // Debug print: Total length of the row from (0,0,0) to rightmost edge
//     std::cout << "Total first row length: " << last_box_right_edge.x() << " meters" << std::endl;

//     // Debugging bounding box rotation
//     for (auto& bbox : arranged_bboxes) {
//         Eigen::Matrix3d rotation_matrix = bbox->R_;  // Get the rotation matrix

//         // Extract rotation angles (Euler angles)
//         double angle_x = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));  // Rotation around X
//         double angle_y = std::atan2(-rotation_matrix(2, 0),
//                                     std::sqrt(rotation_matrix(2, 1) * rotation_matrix(2, 1) + 
//                                               rotation_matrix(2, 2) * rotation_matrix(2, 2)));  // Rotation around Y
//         double angle_z = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));  // Rotation around Z

//         // Convert to degrees
//         double angle_x_degrees = angle_x * 180.0 / M_PI;
//         double angle_y_degrees = angle_y * 180.0 / M_PI;
//         double angle_z_degrees = angle_z * 180.0 / M_PI;

//         // std::cout << "Bounding Box Rotation Angles:" << std::endl;
//         // std::cout << "  Around X: " << angle_x << " radians (" << angle_x_degrees << " degrees)" << std::endl;
//         // std::cout << "  Around Y: " << angle_y << " radians (" << angle_y_degrees << " degrees)" << std::endl;
//         // std::cout << "  Around Z: " << angle_z << " radians (" << angle_z_degrees << " degrees)" << std::endl;
//     }

//     // Debug: First row top face
//     Eigen::Vector3d first_row_top_face = arranged_bboxes[0]->GetCenter();
//     first_row_top_face.z() += (arranged_bboxes[0]->extent_.z() / 2.0);
//     std::cout << "[DEBUG] First row top face (final): " << first_row_top_face.transpose() << std::endl;

//     return arranged_bboxes;
// }

std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::arrangeFirstShingleRow(
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes,
    double gap,
    double max_length,
    double rotation_angle) {

    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
    Eigen::Vector3d current_position(0, 0, 0);

    double previous_half_width = 0;
    double total_length = 0;
    Eigen::Vector3d last_box_right_edge(0, 0, 0);
    const double threshold = 0.02;  // 2 cm tolerance

    for (size_t i = 0; i < bounding_boxes.size(); ++i) {
        auto& bbox = bounding_boxes[i];
        Eigen::Vector3d extent = bbox->extent_;
        double current_half_width = extent.x() / 2.0;

        double predicted_total = total_length + previous_half_width + gap + current_half_width;

        // Allow adding if we haven't yet reached max_length
        // Or if this box keeps us within the allowed tolerance
        if (predicted_total <= max_length || predicted_total <= (max_length + threshold)) {
            current_position.x() += previous_half_width + gap + current_half_width;

            Eigen::Vector3d translation = current_position - bbox->GetCenter();
            transform_bounding_box(bbox, translation, Eigen::Vector3d(0, 1, 0), 0, Eigen::Vector3d(0, 0, 0), true);

            last_box_right_edge = bbox->GetCenter() + Eigen::Vector3d(current_half_width, 0, 0);
            arranged_bboxes.push_back(bbox);

            previous_half_width = current_half_width;
            total_length = predicted_total;
        } else {
            break;  // would exceed max + threshold
        }
    }

    // Apply rotation to entire row
    double rotation_radians = rotation_angle * M_PI / 180.0;
    std::cout << "Applying rotation: " << rotation_angle << " degrees (" 
              << rotation_radians << " radians)" << std::endl;

    for (auto& bbox : arranged_bboxes) {
        transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0),
                               Eigen::Vector3d(1, 0, 0), -rotation_radians,
                               bbox->GetCenter(), true);
    }

    std::cout << "Total first row length: " << last_box_right_edge.x() << " meters" << std::endl;

    Eigen::Vector3d first_row_top_face = arranged_bboxes[0]->GetCenter();
    first_row_top_face.z() += (arranged_bboxes[0]->extent_.z() / 2.0);
    std::cout << "[DEBUG] First row top face (final): " << first_row_top_face.transpose() << std::endl;

    return arranged_bboxes;
}



/////////////////////////////////////////////////////////////////////



////////////////////////////

void GeometryProcessor::alignBoxToXYPlane(std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bbox) {
    // Extract the original rotation matrix
    Eigen::Matrix3d rotation = bbox->R_;

    // Ensure the primary axis aligns with the XY plane
    // Take the first two columns and normalize them
    Eigen::Vector3d x_axis = rotation.col(0).normalized();
    Eigen::Vector3d y_axis = rotation.col(1).normalized();

    // Project the vectors onto the XY plane by setting their Z components to zero
    x_axis.z() = 0;
    y_axis.z() = 0;

    // Normalize the projected vectors
    x_axis.normalize();
    y_axis.normalize();

    // Compute the third axis using the cross product (ensures right-handed coordinate system)
    Eigen::Vector3d z_axis = x_axis.cross(y_axis);
    z_axis.normalize();

    // Construct the new rotation matrix
    Eigen::Matrix3d new_rotation;
    new_rotation.col(0) = x_axis;
    new_rotation.col(1) = y_axis;
    new_rotation.col(2) = z_axis;  // This ensures it's still a valid rotation matrix

    // Apply the new rotation matrix
    bbox->R_ = new_rotation;

    // Optionally, ensure the center remains unchanged
    // If needed, update bbox->center_ accordingly
}

/////////////////////////////////////////////////////////////////////
//
std::shared_ptr<open3d::geometry::OrientedBoundingBox> GeometryProcessor::selectRandomCandidate(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates) {
    if (candidates.empty()) {
        return nullptr;
    }
    // Select a random index between 0 and candidates.size() - 1
    int random_index = rand() % candidates.size();
    return candidates[random_index];
}

/////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////
std::vector<double> GeometryProcessor::calculateRightEdgeDistancesFromCandidate(
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate_shingle,
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row) {

    // Right edge of the candidate shingle (second row)
    Eigen::Vector3d right_edge_candidate = candidate_shingle->GetCenter() + 
                                           Eigen::Vector3d(candidate_shingle->extent_.x() / 2.0, 0, 0);

    std::cout << "Right edge of candidate: " << right_edge_candidate.x() << std::endl;

    std::vector<double> distances;

    for (const auto& bbox : first_row) {
        // Right edge of the current bounding box in the first row
        Eigen::Vector3d right_edge_box = bbox->GetCenter() + 
                                         Eigen::Vector3d(bbox->extent_.x() / 2.0, 0, 0);

        if (right_edge_box.x() > right_edge_candidate.x()) {
            double distance = right_edge_box.x() - right_edge_candidate.x();
            distances.push_back(distance);
            std::cout << "Right edge of first row box: " << right_edge_box.x() 
                      << " | Distance: " << distance << std::endl;
        }
    }

    return distances;
}

////////////////////////////////////////////////////////////////////

// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::findNextBestShingles(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates,
//     double min_stagger,
//     double max_gap,
//     double max_length) 
// {
//     // Create a copy of the first row to avoid modifying the original
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> first_row_aligned;
//     for (const auto& bbox : first_row) {
//         auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox);
//         alignBoxToXYPlane(bbox_copy);
//         first_row_aligned.push_back(bbox_copy);
//     }

//     // Get the left edge of the first box in the first row as our starting reference
//     Eigen::Vector3d current_right_edge = first_row_aligned[0]->GetCenter() - 
//                                          Eigen::Vector3d(first_row_aligned[0]->extent_.x() / 2.0, 0, 0);
//     std::cout << "[DEBUG] Initial current right edge: " << current_right_edge.transpose() << std::endl;

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> second_row;
//     double total_width = 0.0;

//     // Find the maximum width among candidates
//     double max_candidate_width = 0.0;
//     for (const auto& candidate : candidates) {
//         max_candidate_width = std::max(max_candidate_width, candidate->extent_.x());
//     }

//     // Calculate initial distances to the right edges of the first row
//     std::vector<double> distances;
//     for (const auto& bbox : first_row_aligned) {
//         double distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//         distances.push_back(distance);
//         std::cout << "[DEBUG] Distance to right edge for current box: " << distance << std::endl;
//     }

//     // Step 1: Find the first valid shingle
//     std::shared_ptr<open3d::geometry::OrientedBoundingBox> first_shingle = nullptr;

//     for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
//         auto candidate = candidates[candidate_index];

//         std::cout << "[DEBUG] Checking candidate " << candidate_index << ", current total width: " << total_width << std::endl;

//         // Iterate through distances to find the best fit
//         for (size_t i = 0; i < distances.size(); ++i) {
//             double distance = distances[i];

//             // Remove invalid distances
//             if (distance > max_candidate_width) {
//                 distances.erase(distances.begin() + i);
//                 --i;
//                 continue;
//             }

//             // Transform candidate to match reference alignment
//             auto candidate_aligned = alignAndShiftFirstBox(first_row_aligned[0], candidate ,max_gap, max_length, 0.0 );

//             // --- Print candidate width ---
//             std::cout << "[DEBUG] Candidate width: " << candidate_aligned->extent_.x() << std::endl;


//             // Visualize the candidate
//             visualizeShingleRows(first_row_aligned, {candidate_aligned});

//             // Check if the candidate width is within the valid range
//             if (candidate->extent_.x() > (distance + min_stagger) ||  
//                 candidate->extent_.x() < (distance - min_stagger)) 

//             {
//                 first_shingle = candidate_aligned;
//                 second_row.push_back(first_shingle);
//                 std::cout << "first shingle found" << '\n';

//                 // Visualize first shingle of second row
//                 visualizeShingleRows(first_row_aligned, second_row);

//                 // Update the right edge
//                 current_right_edge = updateRightEdge(current_right_edge, first_shingle , max_gap);
//                 // print current right edge
//                 std::cout << "[DEBUG]  current right edge: " << current_right_edge.transpose() << std::endl;

        
//                 total_width += first_shingle->extent_.x();


//                 // print it

//                 // Remove the first shingle from the candidate list
//                 candidates.erase(candidates.begin() + candidate_index);
//                 --candidate_index;

//                 // Update distances based on the new right edge
//                 distances.clear();
//                 for (const auto& bbox : first_row_aligned) {
//                     double new_distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//                     distances.push_back(new_distance);


//                     // print them
//                 }

//                     // Remove invalid distances
//             distances.erase(std::remove_if(distances.begin(), distances.end(),
//             [&](double d) { return d > (max_candidate_width + 0.03) || d <  0; }), distances.end());


//                 break;
//             }
//         }

//         if (first_shingle) break;
//     }

//     if (!first_shingle) {
//         std::cerr << "[ERROR] No valid first shingle found!\n";
//         return second_row;
//     }
//     //////////
//     // **Step 2: Continue placing additional shingles**


//     // double last_right_edge_first_row = first_row_aligned.back()->GetCenter().x() + first_row_aligned.back()->extent_.x() / 2.0;
//     // std::cout << "[DEBUG] Last right edge of first row: " << last_right_edge_first_row << std::endl;


//     std::shared_ptr<open3d::geometry::OrientedBoundingBox> last_selected_shingle = first_shingle;

//     int candidate_counter = 2; // Counter to track number of shingles added to the second row
//     for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
//         auto candidate = candidates[candidate_index];

//         std::cout << "[DEBUG] Checking next candidate " << candidate_index 
//                 << ", current total width: " << total_width << std::endl;

//         std::cout << "[DEBUG] Valid distances after removal: ";
//         for (const auto& dist : distances) {
//             std::cout << dist << " ";
//         }
//         std::cout << std::endl;

//         bool is_valid_for_all_distances = true;

//         for (double distance : distances) {
//             std::cout << "[DEBUG] Checking candidate width against distance: " << distance << std::endl;

//             if (!(candidate->extent_.x() > (distance + min_stagger) ||  
//                 candidate->extent_.x() < (distance - min_stagger))) {
//                 is_valid_for_all_distances = false;
//                 break;  // If one distance fails, stop checking
//             }
//         }


//         // Step 1: Get reference shingle (last in previous row)
//         Eigen::Matrix3d last_R = last_selected_shingle->R_;  // Keep same rotation
//         Eigen::Vector3d last_center = last_selected_shingle->GetCenter();

//         // Step 2: Compute X translation for the next shingle, ensuring a gap
//         double spacing = last_selected_shingle->extent_.x() / 2.0 + max_gap + candidate->extent_.x() / 2.0;
//         Eigen::Vector3d shift_x = spacing * last_R.col(0);  // Ensure translation only along X

//         // Step 3: Apply the X translation to the new shingle
//         auto next_shingle = std::make_shared<open3d::geometry::OrientedBoundingBox>(*candidate);
//         next_shingle->R_ = last_R;  // Keep the same rotation as the previous one

//         // Ensure that the shingle is translated along the positive X direction.
//         if (shift_x.x() < 0) {
//             shift_x = -shift_x;  // Reverse if the shift goes negative along X
//         }

//         next_shingle->Translate(last_center + shift_x - next_shingle->GetCenter());
//         //
//         // auto next_shingle = alignAndShiftNextBox(last_selected_shingle, candidate, max_gap);
//         // // Extract left bottom corner (assuming the corner at min x and min y is the left bottom)
//         // Eigen::Vector3d left_bottom_corner = next_shingle->GetCenter() - 
//         // Eigen::Vector3d(next_shingle->extent_.x() / 2.0, 
//         //                 next_shingle->extent_.y() / 2.0, 
//         //                 0);


//        // visualizeShingleRows(first_row_aligned, {first_shingle, next_shingle});
//         std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> current_visualization = second_row;
//         current_visualization.push_back(next_shingle);  // Add the candidate being tested
//         visualizeShingleRows(first_row_aligned, current_visualization);



//         if (is_valid_for_all_distances) {
//             // --- Print candidate width ---
//             //std::cout << "[DEBUG] Candidate width: " << candidate->extent_.x() << std::endl;
//             std::cout << "[DEBUG] Candidate width: " << next_shingle->extent_.x() << std::endl;

//             std::cout << "[DEBUG] Candidate meets condition for ALL valid distances. Adding to second row.\n";

//             //auto next_shingle = alignAndShiftNextBox(last_selected_shingle, candidate, max_gap);
//             second_row.push_back(next_shingle);
//             std::cout << "Shingle " << candidate_counter << " found and added to the second row" << '\n'; // Show which candidate is added

//             // Increment the counter after adding the shingle
//             candidate_counter++;

//             last_selected_shingle = next_shingle;
//             current_right_edge = updateRightEdge(current_right_edge, next_shingle, max_gap);

//             std::cout << "[DEBUG]  current right edge: " << current_right_edge.transpose() << std::endl;
//             total_width += next_shingle->extent_.x();
//             std::cout << "[DEBUG] Updated total width: " << total_width << std::endl;

//             // Check if total width exceeds max_length and break if necessary
//             if (total_width > max_length) {
//                 std::cout << "[DEBUG] Total width exceeds max_length, stopping placement." << std::endl;
//                 break;  // This break is only triggered when max_length is exceeded
//             }

//             candidates.erase(candidates.begin() + candidate_index);
//             --candidate_index;

//             distances.clear();
//             for (const auto& bbox : first_row_aligned) {
//                 double new_distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//                 distances.push_back(new_distance);
//             }

//             std::cout << "[DEBUG] Distances updated after adding shingle: ";
//             for (const auto& dist : distances) {
//                 std::cout << dist << " ";
//             }
//             std::cout << std::endl;

//             // Remove invalid distances
//             distances.erase(std::remove_if(distances.begin(), distances.end(),
//                 [&](double d) { return d > (max_candidate_width + 0.03) || d < 0; }), distances.end());
//         }
//     }
//     // Final visualization for debugging
//     visualizeShingleRows(first_row_aligned, second_row);

//     std::cout << "[DEBUG] Number of shingles selected for second row: " << second_row.size() << std::endl;


//     return second_row;
// }


// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::findNextBestShingles(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates,
//     double min_stagger,
//     double max_gap,
//     double max_length,
//     bool vis_candidates) 
// {
//     // Create a copy of the first row to avoid modifying the original
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> first_row_aligned;
//     for (const auto& bbox : first_row) {
//         auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox);
//         alignBoxToXYPlane(bbox_copy);
//         first_row_aligned.push_back(bbox_copy);
//     }

//     // Get the left edge of the first box in the first row as our starting reference
//     Eigen::Vector3d current_right_edge = first_row_aligned[0]->GetCenter() - 
//                                          Eigen::Vector3d(first_row_aligned[0]->extent_.x() / 2.0, 0, 0);
//     std::cout << "[DEBUG] Initial current right edge: " << current_right_edge.transpose() << std::endl;

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> second_row;
//     double total_width = 0.0;

//     // Find the maximum width among candidates
//     double max_candidate_width = 0.0;
//     for (const auto& candidate : candidates) {
//         max_candidate_width = std::max(max_candidate_width, candidate->extent_.x());
//     }

//     // Calculate initial distances to the right edges of the first row
//     std::vector<double> distances;
//     for (const auto& bbox : first_row_aligned) {
//         double distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//         distances.push_back(distance);
//         std::cout << "[DEBUG] Distance to right edge for current box: " << distance << std::endl;
//     }

//     // Step 1: Find the first valid shingle
//     std::shared_ptr<open3d::geometry::OrientedBoundingBox> first_shingle = nullptr;

//     for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
//         auto candidate = candidates[candidate_index];

//         std::cout << "[DEBUG] Checking candidate " << candidate_index << ", current total width: " << total_width << std::endl;

//         // Iterate through distances to find the best fit
//         for (size_t i = 0; i < distances.size(); ++i) {
//             double distance = distances[i];

//             // Remove invalid distances
//             if (distance > max_candidate_width) {
//                 distances.erase(distances.begin() + i);
//                 --i;
//                 continue;
//             }

//             // Transform candidate to match reference alignment
//             auto candidate_aligned = alignAndShiftFirstBox(first_row_aligned[0], candidate ,max_gap, max_length, 0.0 );

//             // --- Print candidate width ---
//             std::cout << "[DEBUG] Candidate width: " << candidate_aligned->extent_.x() << std::endl;


//             // Visualize the candidate
//             if(vis_candidates)
//             {visualizeShingleRows(first_row_aligned, {candidate_aligned});}
            

//             // Check if the candidate width is within the valid range
//             if (candidate->extent_.x() > (distance + min_stagger) ||  
//                 candidate->extent_.x() < (distance - min_stagger)) 

//             {
//                 first_shingle = candidate_aligned;
//                 second_row.push_back(first_shingle);
//                 std::cout << "first shingle found" << '\n';

//                 // Visualize first shingle of second row
//                 visualizeShingleRows(first_row_aligned, second_row);

//                 // Update the right edge
//                 current_right_edge = updateRightEdge(current_right_edge, first_shingle , max_gap);
//                 // print current right edge
//                 std::cout << "[DEBUG]  current right edge: " << current_right_edge.transpose() << std::endl;

        
//                 total_width += first_shingle->extent_.x();


//                 // print it

//                 // Remove the first shingle from the candidate list
//                 candidates.erase(candidates.begin() + candidate_index);
//                 --candidate_index;

//                 // Update distances based on the new right edge
//                 distances.clear();
//                 for (const auto& bbox : first_row_aligned) {
//                     double new_distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//                     distances.push_back(new_distance);


//                     // print them
//                 }

//                     // Remove invalid distances
//             distances.erase(std::remove_if(distances.begin(), distances.end(),
//             [&](double d) { return d > (max_candidate_width + 0.03) || d <  0; }), distances.end());


//                 break;
//             }
//         }

//         if (first_shingle) break;
//     }

//     if (!first_shingle) {
//         std::cerr << "[ERROR] No valid first shingle found!\n";
//         return second_row;
//     }
//     //////////
//     // **Step 2: Continue placing additional shingles**
//     double last_right_edge_first_row = first_row_aligned.back()->GetCenter().x() + first_row_aligned.back()->extent_.x() / 2.0;
//     std::cout << "[DEBUG] Last right edge of first row: " << last_right_edge_first_row << std::endl;

//     std::shared_ptr<open3d::geometry::OrientedBoundingBox> last_selected_shingle = first_shingle;

//     int candidate_counter = 2;  // Counter to track number of shingles added to the second row
//     for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
//         auto candidate = candidates[candidate_index];

//         double remaining_width = last_right_edge_first_row - current_right_edge.x();
//         std::cout << "[DEBUG] Remaining width to match first row: " << remaining_width << std::endl;

//         std::cout << "[DEBUG] Checking next candidate " << candidate_index 
//                 << ", current total width: " << total_width << std::endl;
//         // --- Print candidate width ---
//         std::cout << "[DEBUG] Candidate width: " << candidate->extent_.x() << std::endl;

//         std::cout << "[DEBUG] Valid distances after removal: ";
//         for (const auto& dist : distances) {
//             std::cout << dist << " ";
//         }
//         std::cout << std::endl;



//         bool is_valid_for_all_distances = true;

//         for (double distance : distances) {
//             std::cout << "[DEBUG] Checking candidate width against distance: " << distance << std::endl;
//             if (!(candidate->extent_.x() <= (distance - min_stagger) || 
//                 candidate->extent_.x() >= (distance + min_stagger))) 
//             {
//                 is_valid_for_all_distances = false;
//                 std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() 
//                         << " is within the invalid range (" << distance - min_stagger 
//                         << " to " << distance + min_stagger << "). Rejecting." 
//                         << std::endl;
//                 break;  // Stop checking further distances if one fails
//             }
//         }


        
//     // Check if the only remaining distance matches the last right edge of the first row
//     bool is_last_shingle = (distances.size() == 1) && (std::abs(distances[0] - (last_right_edge_first_row - current_right_edge.x())) < 0.01);

//     if (is_last_shingle) {  
//         std::cout << "[DEBUG] Identified last shingle of the row. Finding best-fit candidate.\n";

//         auto best_fit = std::min_element(
//             candidates.begin(), candidates.end(),
//             [&](const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& a,
//                 const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& b) {
//                 return std::abs(a->extent_.x() - distances[0]) < std::abs(b->extent_.x() - distances[0]);
//             });

//         if (best_fit != candidates.end()) {
//             candidate = *best_fit;
//             std::cout << "[DEBUG] Best-fit last shingle width: " << candidate->extent_.x() << std::endl;
//         }
//     }



//         // Debug message for the selected candidate
//         std::cout << "[DEBUG] Candidate with width " << candidate->extent_.x() 
//                 << " selected against valid distances: ";
//         for (double distance : distances) {
//             std::cout << distance << " ";
//         }
//         std::cout << " and min_stagger: " << min_stagger << std::endl;

        

//         // // Step 1: Get reference shingle (last in previous row)
//         // Eigen::Matrix3d last_R = last_selected_shingle->R_;  // Keep same rotation
//         // Eigen::Vector3d last_center = last_selected_shingle->GetCenter();

//         // // Step 2: Compute X translation for the next shingle, ensuring a gap
//         // double spacing = last_selected_shingle->extent_.x() / 2.0 + max_gap + candidate->extent_.x() / 2.0;
//         // Eigen::Vector3d shift_x = spacing * last_R.col(0);  // Ensure translation only along X

//         // // Step 3: Apply the X translation to the new shingle
//         // auto next_shingle = std::make_shared<open3d::geometry::OrientedBoundingBox>(*candidate);
//         // next_shingle->R_ = last_R;  // Keep the same rotation as the previous one

//         // // Ensure that the shingle is translated along the positive X direction.
//         // if (shift_x.x() < 0) {
//         //     shift_x = -shift_x;  // Reverse if the shift goes negative along X
//         // }

//         // next_shingle->Translate(last_center + shift_x - next_shingle->GetCenter());

//         auto next_shingle = alignAndShiftNextBox(last_selected_shingle, candidate, max_gap);


//         std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> current_visualization = second_row;
//         current_visualization.push_back(next_shingle);
//         visualizeShingleRows(first_row_aligned, current_visualization);

//         if (is_valid_for_all_distances) {
//             std::cout << "[DEBUG] Candidate meets condition for ALL valid distances. Adding to second row.\n";

//             second_row.push_back(next_shingle);
//             std::cout << "Shingle " << candidate_counter << " found and added to the second row" << '\n';

//             candidate_counter++;
//             last_selected_shingle = next_shingle;
//             current_right_edge = updateRightEdge(current_right_edge, next_shingle, max_gap);

//             std::cout << "[DEBUG]  current right edge: " << current_right_edge.transpose() << std::endl;
//             total_width += next_shingle->extent_.x();
//             std::cout << "[DEBUG] Updated total width: " << total_width << std::endl;

//             // Check if total width exceeds max_length and break if necessary
//             if (total_width >= max_length) {
//                 std::cout << "[DEBUG] Total width exceeds max_length, stopping placement." << std::endl;
//                 break;
//             }

//             candidates.erase(std::remove(candidates.begin(), candidates.end(), candidate), candidates.end());
//             --candidate_index;

//             distances.clear();
//             for (const auto& bbox : first_row_aligned) {
//                 double new_distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//                 distances.push_back(new_distance);
//             }

//             std::cout << "[DEBUG] Distances updated after adding shingle: ";
//             for (const auto& dist : distances) {
//                 std::cout << dist << " ";
//             }
//             std::cout << std::endl;

//             distances.erase(std::remove_if(distances.begin(), distances.end(),
//                 [&](double d) { return d > (max_candidate_width + 0.03) || d < 0; }), distances.end());
//         }
//     }
//     // Final visualization for debugging
//     visualizeShingleRows(first_row_aligned, second_row);

//     std::cout << "[DEBUG] Number of shingles selected for second row: " << second_row.size() << std::endl;


//     return second_row;
// }




//////////////////////////////////
// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::findNextBestShingles(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates,
//     double min_stagger,
//     double max_gap,
//     double max_length,
//     bool vis_candidates) 
// {
//     std::cout << "[DEBUG] min_stagger: " << min_stagger * 1000.0 << " mm" << std::endl;
//     // Create a copy of the first row to avoid modifying the original
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> first_row_aligned;
//     for (const auto& bbox : first_row) {
//         auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox);
//         alignBoxToXYPlane(bbox_copy);
//         first_row_aligned.push_back(bbox_copy);
//     }

//     // Compute first row's total length
//     double first_row_length = 0.0;
//     for (const auto& bbox : first_row_aligned) {
//         first_row_length = std::max(first_row_length, bbox->GetCenter().x() + bbox->extent_.x() / 2.0);
//     }
//     first_row_length -= first_row_aligned[0]->GetCenter().x() - first_row_aligned[0]->extent_.x() / 2.0;
//     std::cout << "[DEBUG] First row length: " << first_row_length << std::endl;

//     // Compute first-row gap positions (right edge + max_gap)
//     std::vector<double> first_row_gap_positions;
//     for (const auto& bbox : first_row_aligned) {
//         double right_edge = bbox->GetCenter().x() + bbox->extent_.x() / 2.0;
//         first_row_gap_positions.push_back(right_edge + max_gap);
//     }

//     // Get the left edge of the first box as the starting reference
//     Eigen::Vector3d current_right_edge = first_row_aligned[0]->GetCenter() - 
//                                          Eigen::Vector3d(first_row_aligned[0]->extent_.x() / 2.0, 0, 0);
//     std::cout << "[DEBUG] Initial current right edge: " << current_right_edge.transpose() << std::endl;

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> second_row;
//     double total_width = 0.0;

//     // Find the maximum width among candidates
//     double max_candidate_width = 0.0;
//     for (const auto& candidate : candidates) {
//         max_candidate_width = std::max(max_candidate_width, candidate->extent_.x());
//     }

//     // Scoring function with lookahead for global optimization
//     // auto scoreCandidate = [&](const auto& candidate, const std::vector<double>& distances, 
//     //                         double remaining_width, const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& remaining_candidates) {
//     //     double score = 0.0;

//     //     // Staggering
//     //     for (double d : distances) {
//     //         double offset = std::min(std::abs(candidate->extent_.x() - (d - min_stagger)),
//     //                                 std::abs(candidate->extent_.x() - (d + min_stagger)));
//     //         if (offset >= 0.03 && offset < 0.035) {
//     //             score -= 20.0 * (0.035 - offset); // Weak penalty
//     //         } else if (offset >= 0.035) {
//     //             score += 5.0 * std::min(offset, 0.05);
//     //         }
//     //     }

//     //     // Gap non-alignment
//     //     double second_row_gap = current_right_edge.x() + candidate->extent_.x() + max_gap;
//     //     for (double first_gap : first_row_gap_positions) {
//     //         double gap_distance = std::abs(second_row_gap - first_gap);
//     //         score += std::min(gap_distance, 0.05) * 5.0;
//     //     }

//     //     // Lookahead for width
//     //     double new_total_width = total_width + candidate->extent_.x();
//     //     double new_remaining_width = first_row_length - new_total_width;
//     //     if (new_remaining_width < 0) {
//     //         score -= 30.0 * std::abs(new_remaining_width);
//     //     } else if (new_remaining_width > max_candidate_width) {
//     //         score -= 30.0 * new_remaining_width;
//     //     } else {
//     //         bool can_fill = false;
//     //         for (const auto& rem_candidate : remaining_candidates) {
//     //             if (rem_candidate != candidate && rem_candidate->extent_.x() <= new_remaining_width + 0.03) {
//     //                 can_fill = true;
//     //                 score += 1.0;
//     //             }
//     //         }
//     //         if (!can_fill) {
//     //             score -= 5.0;
//     //         }
//     //     }

//     //     // Final shingle
//     //     if (remaining_width <= max_candidate_width) {
//     //         if (new_remaining_width <= 0.03 && new_remaining_width >= -0.03) {
//     //             score += 20.0; // Weak reward
//     //         } else {
//     //             score -= 50.0 * std::abs(new_remaining_width); // Weak penalty
//     //         }
//     //     }

//     //     return score;
//     // };

// auto scoreCandidate = [&](const auto& candidate, const std::vector<double>& distances, 
//                          double remaining_width, const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& remaining_candidates, 
//                          double max_length) {
//     double score = 0.0;
//     double candidate_width = candidate->extent_.x();
//     double new_total_width = total_width + candidate_width;
//     double new_remaining_width = max_length - new_total_width; // Use max_length, not first_row_length

//     // Determine if this is the final or second-to-last shingle
//     double max_candidate_width = 0.0;
//     for (const auto& rem_candidate : remaining_candidates) {
//         if (rem_candidate != candidate) {
//             max_candidate_width = std::max(max_candidate_width, rem_candidate->extent_.x());
//         }
//     }
//     bool is_final_shingle = (remaining_width <= max_candidate_width);
//     bool is_second_to_last = (remaining_width <= 2.0 * max_candidate_width && remaining_width > max_candidate_width);
//     double min_stagger_local = is_final_shingle ? 0.025 : 0.03; // Relax stagger to 25 mm for final shingle (or 0.0 if acceptable)

//     // Staggering
//     for (double d : distances) {
//         double offset = std::min(std::abs(candidate_width - (d - min_stagger_local)),
//                                  std::abs(candidate_width - (d + min_stagger_local)));
//         if (!is_final_shingle && offset >= 0.03 && offset < 0.035) {
//             score -= 150.0 * (0.035 - offset); // Strong penalty for yellow shingles (30–35 mm)
//         } else if (offset >= 0.035 || (is_final_shingle && offset >= 0.025)) {
//             score += 25.0 * std::min(offset, 0.06); // Reward green or relaxed final shingle
//         }
//     }

//     // Gap non-alignment
//     double second_row_gap = current_right_edge.x() + candidate_width + max_gap;
//     for (double first_gap : first_row_gap_positions) {
//         double gap_distance = std::abs(second_row_gap - first_gap);
//         score += std::min(gap_distance, 0.05) * 5.0;
//     }

//     // Lookahead for second-to-last shingle
//     if (is_second_to_last) {
//         bool can_fill = false;
//         double best_fit_score = 0.0;
//         for (const auto& rem_candidate : remaining_candidates) {
//             if (rem_candidate != candidate) {
//                 double next_width = rem_candidate->extent_.x();
//                 double final_remaining = new_remaining_width - next_width;
//                 if (final_remaining >= 0 && final_remaining <= 0.05) { // Enforce 0–50 mm overhang
//                     can_fill = true;
//                     best_fit_score += 300.0 * (0.05 - final_remaining); // Reward good fit
//                     if (final_remaining <= 0.02) {
//                         best_fit_score += 100.0 * (0.02 - final_remaining); // Extra reward for 0–20 mm
//                     }
//                 } else if (final_remaining < 0) {
//                     best_fit_score -= 200.0 * std::abs(final_remaining); // Penalize overhang > 50 mm
//                 } else {
//                     best_fit_score -= 600.0 * final_remaining; // Penalize undercoverage
//                 }
//             }
//         }
//         score += best_fit_score;
//         if (!can_fill) {
//             score -= 100.0; // Penalize if no final shingle can fill
//         }
//     }

//     // Lookahead for width (non-final, non-second-to-last)
//     if (!is_final_shingle && !is_second_to_last) {
//         if (new_remaining_width < 0) {
//             score -= 150.0 * std::abs(new_remaining_width); // Penalize overhang
//         } else if (new_remaining_width > max_candidate_width) {
//             score -= 500.0 * new_remaining_width; // Penalize large undercoverage
//         } else {
//             bool can_fill = false;
//             for (const auto& rem_candidate : remaining_candidates) {
//                 if (rem_candidate != candidate && rem_candidate->extent_.x() <= new_remaining_width + 0.05) { // Allow up to 50 mm overhang
//                     can_fill = true;
//                     score += 10.0 * (1.0 - std::abs(rem_candidate->extent_.x() - new_remaining_width) / max_candidate_width);
//                 }
//             }
//             if (!can_fill) {
//                 score -= 50.0; // Penalize unfillable gaps
//             }
//         }
//     }

//     // Final shingle: Enforce full coverage, 0–50 mm overhang
//     if (is_final_shingle) {
//         if (new_remaining_width >= 0 && new_remaining_width <= 0.05) {
//             score += 400.0 * (0.05 - new_remaining_width); // Reward 0–50 mm overhang
//             if (new_remaining_width <= 0.02) {
//                 score += 100.0 * (0.02 - new_remaining_width); // Extra reward for 0–20 mm
//             }
//         } else if (new_remaining_width > 0.05) {
//             score -= 200.0 * (new_remaining_width - 0.05); // Penalize > 50 mm overhang
//         } else { // Undercoverage
//             score -= 2000.0 * std::abs(new_remaining_width); // Severe penalty for undercoverage
//         }
//     } else if (new_remaining_width <= max_candidate_width && new_remaining_width >= 0) {
//         score += 150.0 * (0.05 - std::min(new_remaining_width, 0.05)); // Encourage small gaps
//     } else if (new_remaining_width < 0) {
//         score -= 150.0 * std::abs(new_remaining_width); // Penalize overhang
//     } else {
//         score -= 600.0 * new_remaining_width; // Penalize undercoverage
//     }

//     return score;
// };



//     // Calculate initial distances to the right edges of the first row
//     std::vector<double> distances;
//     for (const auto& bbox : first_row_aligned) {
//         double distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//         distances.push_back(distance);
//         std::cout << "[DEBUG] Distance to right edge: " << distance << std::endl;
//     }

//     // Step 1: Find the first valid shingle
// std::shared_ptr<open3d::geometry::OrientedBoundingBox> first_shingle = nullptr;
//     double best_score = -std::numeric_limits<double>::max();
//     size_t best_index = 0;

//     std::cout << "[DEBUG] min_stagger: " << min_stagger * 1000.0 << " mm" << std::endl;
//     for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
//         auto candidate = candidates[candidate_index];
//         std::cout << "[DEBUG] Checking candidate " << candidate_index << ", width: " << candidate->extent_.x() << std::endl;

//         bool valid = true;
//         // Compute stagger margin
//         auto candidate_aligned = alignAndShiftFirstBox(first_row_aligned[0], candidate, max_gap, max_length, 0.0);
//         double second_row_gap = current_right_edge.x() + candidate_aligned->extent_.x() + max_gap;
//         double min_stagger_margin = std::numeric_limits<double>::max();
//         for (double first_gap : first_row_gap_positions) {
//             double stagger = std::abs(second_row_gap - first_gap);
//             min_stagger_margin = std::min(min_stagger_margin, stagger);
//         }
//         if (min_stagger_margin < min_stagger) {
//             std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() << " invalid, stagger margin " 
//                     << min_stagger_margin * 1000.0 << " mm < " << min_stagger * 1000.0 << " mm" << std::endl;
//             valid = false;
//         }

//         if (valid) {
//             double score = scoreCandidate(candidate, distances, first_row_length, candidates,max_length);
//             if (score > best_score) {
//                 best_score = score;
//                 first_shingle = candidate;
//                 best_index = candidate_index;
//             }
//         }
//     }

//     if (first_shingle) {
//         auto candidate_aligned = alignAndShiftFirstBox(first_row_aligned[0], first_shingle, max_gap, max_length, 0.0);
//         // Compute stagger margin
//         double second_row_gap = current_right_edge.x() + candidate_aligned->extent_.x() + max_gap;
//         double min_stagger_margin = std::numeric_limits<double>::max();
//         for (double first_gap : first_row_gap_positions) {
//             double stagger = std::abs(second_row_gap - first_gap);
//             min_stagger_margin = std::min(min_stagger_margin, stagger);
//         }
//         std::cout << "[DEBUG] First shingle selected, width: " << candidate_aligned->extent_.x() 
//                   << ", stagger margin: " << min_stagger_margin * 1000.0 << " mm" << std::endl;

//         if (vis_candidates) {
//             visualizeShingleRows(first_row_aligned, {candidate_aligned});
//         }
//         second_row.push_back(candidate_aligned);
//         current_right_edge = updateRightEdge(current_right_edge, candidate_aligned, max_gap);
//         total_width += candidate_aligned->extent_.x();
//         candidates.erase(candidates.begin() + best_index);
//     } else {
//         std::cerr << "[ERROR] No valid first shingle found!\n";
//         return second_row;
//     }

//     // Step 2: Continue placing additional shingles
//     double last_right_edge_first_row = first_row_length + first_row_aligned[0]->GetCenter().x() - 
//                                        first_row_aligned[0]->extent_.x() / 2.0;
//     std::cout << "[DEBUG] Last right edge of first row: " << last_right_edge_first_row << std::endl;

//     std::shared_ptr<open3d::geometry::OrientedBoundingBox> last_selected_shingle = first_shingle;
//     int candidate_counter = 2;

//     while (total_width < max_length && candidates.size() > 0) {
//         double remaining_width = last_right_edge_first_row - current_right_edge.x();
//         std::cout << "[DEBUG] Remaining width: " << remaining_width << std::endl;

//         // Update distances
//         distances.clear();
//         for (const auto& bbox : first_row_aligned) {
//             double new_distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//             distances.push_back(new_distance);
//         }
//         distances.erase(std::remove_if(distances.begin(), distances.end(),
//             [&](double d) { return d > max_candidate_width + 0.03 || d < 0; }), distances.end());

//         // Find best candidate
// std::shared_ptr<open3d::geometry::OrientedBoundingBox> best_candidate = nullptr;
//         best_score = -std::numeric_limits<double>::max();
//         best_index = 0;

//         for (size_t i = 0; i < candidates.size(); ++i) {
//             auto candidate = candidates[i];
//             bool valid = true;
//             // Compute stagger margin
//             auto next_shingle = alignAndShiftNextBox(last_selected_shingle, candidate, max_gap);
//             double second_row_gap = current_right_edge.x() + next_shingle->extent_.x() + max_gap;
//             double min_stagger_margin = std::numeric_limits<double>::max();
//             for (double first_gap : first_row_gap_positions) {
//                 double stagger = std::abs(second_row_gap - first_gap);
//                 min_stagger_margin = std::min(min_stagger_margin, stagger);
//             }
//             if (min_stagger_margin < min_stagger) {
//                 std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() << " invalid, stagger margin " 
//                         << min_stagger_margin * 1000.0 << " mm < " << min_stagger * 1000.0 << " mm" << std::endl;
//                 valid = false;
//             }

//             if (valid) {
//                 double score = scoreCandidate(candidate, distances, remaining_width, candidates, max_length);
//                 if (score > best_score) {
//                     best_score = score;
//                     best_candidate = candidate;
//                     best_index = i;
//                 }
//             }
//         }

//         if (!best_candidate) {
//             std::cout << "[DEBUG] No valid candidate found, stopping.\n";
//             break;
//         }

//         // Align and place best candidate
//         auto next_shingle = alignAndShiftNextBox(last_selected_shingle, best_candidate, max_gap);
//         // Compute stagger margin
//         double second_row_gap = current_right_edge.x() + next_shingle->extent_.x() + max_gap;
//         double min_stagger_margin = std::numeric_limits<double>::max();
//         for (double first_gap : first_row_gap_positions) {
//             double stagger = std::abs(second_row_gap - first_gap);
//             min_stagger_margin = std::min(min_stagger_margin, stagger);
//         }
//         std::cout << "[DEBUG] Shingle " << candidate_counter << " selected, width: " << next_shingle->extent_.x() 
//                   << ", stagger margin: " << min_stagger_margin * 1000.0 << " mm" << std::endl;

//         if (vis_candidates) {
//             auto current_visualization = second_row;
//             current_visualization.push_back(next_shingle);
//             visualizeShingleRows(first_row_aligned, current_visualization);
//         }

//         second_row.push_back(next_shingle);
//         candidate_counter++;
//         last_selected_shingle = next_shingle;
//         current_right_edge = updateRightEdge(current_right_edge, next_shingle, max_gap);
//         total_width += next_shingle->extent_.x();
//         candidates.erase(candidates.begin() + best_index);

//         // Check length tolerance
//         if (total_width >= first_row_length) {
//             if (total_width > first_row_length + 0.02) {
//                 std::cout << "[DEBUG] Overhang exceeds 2 cm: " << (total_width - first_row_length) << " m\n";
//             }
//             break;
//         }
//     }
// // Check for undercoverage
//     if (total_width < first_row_length) {
//         std::cout << "[WARNING] Second row undercovers by: " << (first_row_length - total_width) << " m\n";
//     }

//     // Final visualization with color coding
//     visualizeShingleRows(first_row_aligned, second_row, true);
//     std::cout << "[DEBUG] Number of shingles selected: " << second_row.size() << std::endl;
//     std::cout << "[DEBUG] Second row total width: " << total_width * 1000.0 << " mm, First row length: " 
//               << first_row_length * 1000.0 << " mm, Difference: " << (total_width - first_row_length) * 1000.0 << " mm" << std::endl;

//     return second_row;
// }

////////////////////////////////


// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::findNextBestShingles(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates,
//     double min_stagger,
//     double max_gap,
//     double max_length,
//     bool vis_candidates) 
// {
//     std::cout << "[DEBUG] min_stagger: " << min_stagger * 1000.0 << " mm, max_length: " << max_length * 1000.0 << " mm" << std::endl;

//     // Create a copy of the first row to avoid modifying the original
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> first_row_aligned;
//     for (const auto& bbox : first_row) {
//         auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox);
//         alignBoxToXYPlane(bbox_copy);
//         first_row_aligned.push_back(bbox_copy);
//     }

//     // Compute first row's gap positions (right edge + max_gap)
//     std::vector<double> first_row_gap_positions;
//     for (const auto& bbox : first_row_aligned) {
//         double right_edge = bbox->GetCenter().x() + bbox->extent_.x() / 2.0;
//         first_row_gap_positions.push_back(right_edge + max_gap);
//     }

//     // Get the left edge of the first box as the starting reference
//     Eigen::Vector3d current_right_edge = first_row_aligned[0]->GetCenter() - 
//                                          Eigen::Vector3d(first_row_aligned[0]->extent_.x() / 2.0, 0, 0);
//     std::cout << "[DEBUG] Initial current right edge: " << current_right_edge.transpose() << std::endl;

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> second_row;
//     double total_width = 0.0;

//     // Find the maximum and minimum candidate widths
//     double max_candidate_width = 0.0;
//     double min_candidate_width = std::numeric_limits<double>::max();
//     for (const auto& candidate : candidates) {
//         max_candidate_width = std::max(max_candidate_width, candidate->extent_.x());
//         min_candidate_width = std::min(min_candidate_width, candidate->extent_.x());
//     }

//     // Scoring function
//     auto scoreCandidate = [&](const auto& candidate, const std::vector<double>& distances, 
//                              double remaining_width, const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& remaining_candidates, 
//                              double max_length) {
//         double score = 0.0;
//         double candidate_width = candidate->extent_.x();
//         double new_total_width = total_width + candidate_width;
//         double new_remaining_width = max_length - new_total_width;

//         // Determine if this is the final or second-to-last shingle
//         bool is_final_shingle = (new_remaining_width <= 0.05 && new_total_width >= max_length);
//         bool is_second_to_last = (remaining_width <= 2.0 * max_candidate_width && remaining_width > max_candidate_width);

//         // Staggering
//         for (double d : distances) {
//             double offset = std::min(std::abs(candidate_width - (d - min_stagger)),
//                                      std::abs(candidate_width - (d + min_stagger)));
//             if (!is_final_shingle && offset >= min_stagger && offset < 0.035) {
//                 score -= 800.0 * (0.035 - offset); // Stronger penalty for yellow shingles
//             } else if (offset >= 0.035 || is_final_shingle) {
//                 score += 200.0 * std::min(offset, 0.06); // Stronger reward for green
//             }
//         }

//         // Gap non-alignment
//         double second_row_gap = current_right_edge.x() + candidate_width + max_gap;
//         for (double first_gap : first_row_gap_positions) {
//             double gap_distance = std::abs(second_row_gap - first_gap);
//             score += std::min(gap_distance, 0.05) * 10.0;
//         }

//         // Lookahead for second-to-last shingle
//         if (is_second_to_last && !is_final_shingle) {
//             bool can_fill = false;
//             double best_fit_score = 0.0;
//             for (const auto& rem_candidate : remaining_candidates) {
//                 if (rem_candidate != candidate) {
//                     double next_width = rem_candidate->extent_.x();
//                     double final_remaining = new_remaining_width - next_width;
//                     if (final_remaining >= 0 && final_remaining <= 0.05) {
//                         can_fill = true;
//                         best_fit_score += 2000.0 * (0.05 - final_remaining); // Increased reward
//                         if (final_remaining <= 0.02) {
//                             best_fit_score += 1000.0 * (0.02 - final_remaining);
//                         }
//                     } else if (final_remaining < 0) {
//                         best_fit_score -= 4000.0 * std::abs(final_remaining); // Stronger penalty
//                     } else {
//                         best_fit_score -= 3000.0 * final_remaining;
//                     }
//                 }
//             }
//             score += best_fit_score;
//             if (!can_fill) {
//                 score -= 1000.0; // Stronger penalty
//             }
//         }

//         // Width constraints
//         if (is_final_shingle) {
//             if (new_remaining_width >= 0 && new_remaining_width <= 0.05) {
//                 score += 2000.0 * (0.05 - new_remaining_width); // Increased reward
//                 if (new_remaining_width <= 0.02) {
//                     score += 1000.0 * (0.02 - new_remaining_width);
//                 }
//             } else if (new_remaining_width > 0.05) {
//                 score -= 4000.0 * (new_remaining_width - 0.05); // Stronger penalty
//             } else {
//                 score -= 8000.0 * std::abs(new_remaining_width); // Much stronger penalty
//             }
//         } else if (new_remaining_width <= max_candidate_width && new_remaining_width >= 0) {
//             score += 1000.0 * (0.05 - std::min(new_remaining_width, 0.05)); // Increased reward
//             bool can_fill = false;
//             for (const auto& rem_candidate : remaining_candidates) {
//                 if (rem_candidate != candidate && rem_candidate->extent_.x() <= new_remaining_width + 0.05) {
//                     can_fill = true;
//                     score += 150.0 * (1.0 - std::abs(rem_candidate->extent_.x() - new_remaining_width) / max_candidate_width);
//                 }
//             }
//             if (!can_fill) {
//                 score -= 600.0;
//             }
//         } else if (new_remaining_width < 0) {
//             score -= 4000.0 * std::abs(new_remaining_width); // Stronger penalty
//         } else {
//             score -= 3000.0 * new_remaining_width;
//         }

//         return score;
//     };

//     // Calculate initial distances to the right edges of the first row
//     std::vector<double> distances;
//     for (const auto& bbox : first_row_aligned) {
//         double distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//         distances.push_back(distance);
//         std::cout << "[DEBUG] Distance to right edge: " << distance << std::endl;
//     }

//     // Step 1: Find the first valid shingle
//     std::shared_ptr<open3d::geometry::OrientedBoundingBox> first_shingle = nullptr;
//     double best_score = -std::numeric_limits<double>::max();
//     size_t best_index = 0;

//     for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
//         auto candidate = candidates[candidate_index];
//         std::cout << "[DEBUG] Checking candidate " << candidate_index << ", width: " << candidate->extent_.x() << std::endl;

//         bool valid = true;
//         auto candidate_aligned = alignAndShiftFirstBox(first_row_aligned[0], candidate, max_gap, max_length, 0.0);
//         double second_row_gap = current_right_edge.x() + candidate_aligned->extent_.x() + max_gap;
//         double min_stagger_margin = std::numeric_limits<double>::max();
//         for (double first_gap : first_row_gap_positions) {
//             double stagger = std::abs(second_row_gap - first_gap);
//             min_stagger_margin = std::min(min_stagger_margin, stagger);
//         }
//         if (min_stagger_margin < min_stagger) {
//             std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() << " invalid, stagger margin " 
//                       << min_stagger_margin * 1000.0 << " mm < " << min_stagger * 1000.0 << " mm" << std::endl;
//             valid = false;
//         }

//         if (valid) {
//             double score = scoreCandidate(candidate, distances, max_length - total_width, candidates, max_length);
//             if (score > best_score) {
//                 best_score = score;
//                 first_shingle = candidate;
//                 best_index = candidate_index;
//             }
//         }
//     }

//     if (first_shingle) {
//         auto candidate_aligned = alignAndShiftFirstBox(first_row_aligned[0], first_shingle, max_gap, max_length, 0.0);
//         double second_row_gap = current_right_edge.x() + candidate_aligned->extent_.x() + max_gap;
//         double min_stagger_margin = std::numeric_limits<double>::max();
//         for (double first_gap : first_row_gap_positions) {
//             double stagger = std::abs(second_row_gap - first_gap);
//             min_stagger_margin = std::min(min_stagger_margin, stagger);
//         }
//         std::cout << "[DEBUG] First shingle selected, width: " << candidate_aligned->extent_.x() 
//                   << ", stagger margin: " << min_stagger_margin * 1000.0 << " mm" << std::endl;

//         if (vis_candidates) {
//             visualizeShingleRows(first_row_aligned, {candidate_aligned});
//         }
//         second_row.push_back(candidate_aligned);
//         current_right_edge = updateRightEdge(current_right_edge, candidate_aligned, max_gap);
//         total_width += candidate_aligned->extent_.x();
//         candidates.erase(candidates.begin() + best_index);
//     } else {
//         std::cerr << "[ERROR] No valid first shingle found!\n";
//         return second_row;
//     }

//     // Step 2: Continue placing additional shingles
//     std::shared_ptr<open3d::geometry::OrientedBoundingBox> last_selected_shingle = first_shingle;
//     int candidate_counter = 2;

//     while (total_width < max_length && candidates.size() > 0) {
//         double remaining_width = max_length - total_width;
//         std::cout << "[DEBUG] Remaining width: " << remaining_width << std::endl;

//         // Update distances
//         distances.clear();
//         for (const auto& bbox : first_row_aligned) {
//             double new_distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
//             distances.push_back(new_distance);
//         }
//         distances.erase(std::remove_if(distances.begin(), distances.end(),
//             [&](double d) { return d > max_candidate_width + 0.05 || d < 0; }), distances.end());

//         // Find best candidate
//         std::shared_ptr<open3d::geometry::OrientedBoundingBox> best_candidate = nullptr;
//         best_score = -std::numeric_limits<double>::max();
//         best_index = 0;
//         bool valid_candidate_found = false;

//         for (size_t i = 0; i < candidates.size(); ++i) {
//             auto candidate = candidates[i];
//             bool valid = true;
//             bool is_final_shingle = (remaining_width <= max_candidate_width && total_width + candidate->extent_.x() >= max_length);
//             double local_min_stagger = is_final_shingle ? 0.0 : min_stagger;

//             auto next_shingle = alignAndShiftNextBox(last_selected_shingle, candidate, max_gap);
//             double second_row_gap = current_right_edge.x() + next_shingle->extent_.x() + max_gap;
//             double min_stagger_margin = std::numeric_limits<double>::max();
//             for (double first_gap : first_row_gap_positions) {
//                 double stagger = std::abs(second_row_gap - first_gap);
//                 min_stagger_margin = std::min(min_stagger_margin, stagger);
//             }
//             if (min_stagger_margin < local_min_stagger) {
//                 std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() << " invalid, stagger margin " 
//                           << min_stagger_margin * 1000.0 << " mm < " << local_min_stagger * 1000.0 << " mm" << std::endl;
//                 valid = false;
//             }

//             // Additional check for final shingle to ensure overhang <= 50 mm
//             if (is_final_shingle) {
//                 double overhang = total_width + candidate->extent_.x() - max_length;
//                 if (overhang > 0.05) {
//                     valid = false; // Reject candidates causing >50 mm overhang
//                     std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() << " invalid, overhang " 
//                               << overhang * 1000.0 << " mm > 50 mm" << std::endl;
//                 }
//             }

//             if (valid) {
//                 valid_candidate_found = true;
//                 double score = scoreCandidate(candidate, distances, remaining_width, candidates, max_length);
//                 if (score > best_score) {
//                     best_score = score;
//                     best_candidate = candidate;
//                     best_index = i;
//                 }
//             }
//         }

//         if (!best_candidate) {
//             std::cout << "[DEBUG] No valid candidate found, stopping.\n";
//             break;
//         }

//         auto next_shingle = alignAndShiftNextBox(last_selected_shingle, best_candidate, max_gap);
//         double second_row_gap = current_right_edge.x() + next_shingle->extent_.x() + max_gap;
//         double min_stagger_margin = std::numeric_limits<double>::max();
//         for (double first_gap : first_row_gap_positions) {
//             double stagger = std::abs(second_row_gap - first_gap);
//             min_stagger_margin = std::min(min_stagger_margin, stagger);
//         }
//         std::cout << "[DEBUG] Shingle " << candidate_counter << " selected, width: " << next_shingle->extent_.x() 
//                   << ", stagger margin: " << min_stagger_margin * 1000.0 << " mm" << std::endl;

//         if (vis_candidates) {
//             auto current_visualization = second_row;
//             current_visualization.push_back(next_shingle);
//             visualizeShingleRows(first_row_aligned, current_visualization);
//         }

//         second_row.push_back(next_shingle);
//         candidate_counter++;
//         last_selected_shingle = next_shingle;
//         current_right_edge = updateRightEdge(current_right_edge, next_shingle, max_gap);
//         total_width += next_shingle->extent_.x();
//         candidates.erase(candidates.begin() + best_index);

//         // Check length tolerance
//         if (total_width >= max_length) {
//             if (total_width > max_length + 0.05) {
//                 std::cout << "[DEBUG] Overhang exceeds 5 cm: " << (total_width - max_length) * 1000.0 << " mm\n";
//             }
//             break;
//         }
//     }

//     // Check for undercoverage
//     if (total_width < max_length) {
//         std::cout << "[WARNING] Second row undercovers by: " << (max_length - total_width) * 1000.0 << " mm\n";
//     }

//     // Final visualization
//     visualizeShingleRows(first_row_aligned, second_row, true);
//     std::cout << "[DEBUG] Number of shingles selected: " << second_row.size() << std::endl;
//     std::cout << "[DEBUG] Second row total width: " << total_width * 1000.0 << " mm, max_length: " 
//               << max_length * 1000.0 << " mm, Difference: " << (total_width - max_length) * 1000.0 << " mm" << std::endl;

//     return second_row;
// }


/////////////////////////////////
// Helper function to format shingle debug message
    void GeometryProcessor::debugShingleRow(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row_aligned,
    const std::vector<double>& first_row_gap_positions,
    double max_gap) const {
    if (!second_row.empty()) {
        Eigen::Vector3d debug_right_edge = first_row_aligned[0]->GetCenter() - 
                                           Eigen::Vector3d(first_row_aligned[0]->extent_.x() / 2.0, 0, 0);
        for (size_t i = 0; i < second_row.size(); ++i) {
            auto& bbox = second_row[i];
            double width_mm = bbox->extent_.x() * 1000.0;
            
            // Compute stagger margin
            double second_row_gap = debug_right_edge.x() + bbox->extent_.x() + max_gap;
            double min_stagger_margin = std::numeric_limits<double>::max();
            for (double first_gap : first_row_gap_positions) {
                double stagger = std::abs(second_row_gap - first_gap);
                min_stagger_margin = std::min(min_stagger_margin, stagger);
            }
            double stagger_mm = min_stagger_margin * 1000.0;

            // Determine final/non-final and color
            bool is_final = (i == second_row.size() - 1);
            std::string final_status = is_final ? "final" : "non-final";
            std::string min_stagger_required = is_final ? "any" : "30";
            std::string color = is_final ? "green" : (min_stagger_margin >= 0.035 ? "green" : "yellow");

            // Print debug message
            std::cout << "Shingle " << (i + 1) << ": Width " << width_mm << " mm, Stagger " 
                      << stagger_mm << " mm (✓ " << final_status << ", ≥" << min_stagger_required 
                      << " mm, " << color << ")." << std::endl;

            // Update right edge for next shingle
            debug_right_edge.x() += bbox->extent_.x() + max_gap;
        }
    }
}

////////////////////////////////////////










std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::findNextBestShingles(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates,
    double min_stagger,
    double max_gap,
    double max_length,
    bool vis_candidates) 
{
    std::cout << "[DEBUG] min_stagger: " << min_stagger * 1000.0 << " mm, max_length: " << max_length * 1000.0 << " mm" << std::endl;

    // Create a copy of the first row to avoid modifying the original
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> first_row_aligned;
    for (const auto& bbox : first_row) {
        auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox);
        alignBoxToXYPlane(bbox_copy);
        first_row_aligned.push_back(bbox_copy);
    }

    // Compute first row's gap positions (right edge + max_gap)
    std::vector<double> first_row_gap_positions;
    for (const auto& bbox : first_row_aligned) {
        double right_edge = bbox->GetCenter().x() + bbox->extent_.x() / 2.0;
        first_row_gap_positions.push_back(right_edge + max_gap);
    }

    // Get the left edge of the first box as the starting reference
    Eigen::Vector3d current_right_edge = first_row_aligned[0]->GetCenter() - 
                                         Eigen::Vector3d(first_row_aligned[0]->extent_.x() / 2.0, 0, 0);
    std::cout << "[DEBUG] Initial current right edge: " << current_right_edge.transpose() << std::endl;

    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> second_row;
    double total_width = 0.0;

    // Find the maximum and minimum candidate widths
    double max_candidate_width = 0.0;
    double min_candidate_width = std::numeric_limits<double>::max();
    for (const auto& candidate : candidates) {
        max_candidate_width = std::max(max_candidate_width, candidate->extent_.x());
        min_candidate_width = std::min(min_candidate_width, candidate->extent_.x());
    }

    // Scoring function
    auto scoreCandidate = [&](const auto& candidate, const std::vector<double>& distances, 
                             double remaining_width, const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& remaining_candidates, 
                             double max_length) {
        double score = 0.0;
        double candidate_width = candidate->extent_.x();
        double new_total_width = total_width + candidate_width;
        double new_remaining_width = max_length - new_total_width;

        // Determine if this is the final or second-to-last shingle
        bool is_final_shingle = (new_remaining_width <= 0.05 && new_total_width >= max_length) || 
                                (remaining_width <= max_candidate_width);
        bool is_second_to_last = (remaining_width <= 2.0 * max_candidate_width && remaining_width > max_candidate_width);

        // Staggering
        for (double d : distances) {
            double offset = std::min(std::abs(candidate_width - (d - min_stagger)),
                                     std::abs(candidate_width - (d + min_stagger)));
            if (!is_final_shingle && offset >= min_stagger && offset < 0.035) {
                score -= 1000.0 * (0.035 - offset); // Stronger penalty for yellow shingles
            } else if (offset >= 0.035 || is_final_shingle) {
                score += 250.0 * std::min(offset, 0.06); // Stronger reward for green
            }
        }

        // Gap non-alignment
        double second_row_gap = current_right_edge.x() + candidate_width + max_gap;
        for (double first_gap : first_row_gap_positions) {
            double gap_distance = std::abs(second_row_gap - first_gap);
            score += std::min(gap_distance, 0.05) * 10.0;
        }

        // Lookahead for second-to-last shingle
        if (is_second_to_last && !is_final_shingle) {
            bool can_fill = false;
            double best_fit_score = 0.0;
            for (const auto& rem_candidate : remaining_candidates) {
                if (rem_candidate != candidate) {
                    double next_width = rem_candidate->extent_.x();
                    double final_remaining = new_remaining_width - next_width;
                    if (final_remaining >= 0 && final_remaining <= 0.05) {
                        can_fill = true;
                        best_fit_score += 2500.0 * (0.05 - final_remaining); // Increased reward
                        if (final_remaining <= 0.02) {
                            best_fit_score += 1500.0 * (0.02 - final_remaining);
                        }
                    } else if (final_remaining < 0) {
                        best_fit_score -= 5000.0 * std::abs(final_remaining); // Stronger penalty
                    } else {
                        best_fit_score -= 4000.0 * final_remaining;
                    }
                }
            }
            score += best_fit_score;
            if (!can_fill) {
                score -= 1500.0; // Stronger penalty
            }
        }

        // Width constraints
        if (is_final_shingle) {
            if (new_remaining_width >= 0 && new_remaining_width <= 0.05) {
                score += 2500.0 * (0.05 - new_remaining_width); // Increased reward
                if (new_remaining_width <= 0.02) {
                    score += 1500.0 * (0.02 - new_remaining_width);
                }
            } else if (new_remaining_width > 0.05) {
                score -= 5000.0 * (new_remaining_width - 0.05); // Stronger penalty
            } else {
                score -= 10000.0 * std::abs(new_remaining_width); // Much stronger penalty
            }
        } else if (new_remaining_width <= max_candidate_width && new_remaining_width >= 0) {
            score += 1500.0 * (0.05 - std::min(new_remaining_width, 0.05)); // Increased reward
            bool can_fill = false;
            for (const auto& rem_candidate : remaining_candidates) {
                if (rem_candidate != candidate && rem_candidate->extent_.x() <= new_remaining_width + 0.05) {
                    can_fill = true;
                    score += 200.0 * (1.0 - std::abs(rem_candidate->extent_.x() - new_remaining_width) / max_candidate_width);
                }
            }
            if (!can_fill) {
                score -= 800.0;
            }
        } else if (new_remaining_width < 0) {
            score -= 5000.0 * std::abs(new_remaining_width); // Stronger penalty
            if (new_total_width > max_length + 0.05) {
                score -= 10000.0 * (new_total_width - (max_length + 0.05)); // Additional penalty for large overhang
            }
        } else {
            score -= 4000.0 * new_remaining_width;
        }

        return score;
    };

    // Calculate initial distances to the right edges of the first row
    std::vector<double> distances;
    for (const auto& bbox : first_row_aligned) {
        double distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
        distances.push_back(distance);
        std::cout << "[DEBUG] Distance to right edge: " << distance << std::endl;
    }

    // Step 1: Find the first valid shingle
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> first_shingle = nullptr;
    double best_score = -std::numeric_limits<double>::max();
    size_t best_index = 0;

    for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
        auto candidate = candidates[candidate_index];
        std::cout << "[DEBUG] Checking candidate " << candidate_index << ", width: " << candidate->extent_.x() << std::endl;

        bool valid = true;
        auto candidate_aligned = alignAndShiftFirstBox(first_row_aligned[0], candidate, max_gap, max_length, 0.0);
        double second_row_gap = current_right_edge.x() + candidate_aligned->extent_.x() + max_gap;
        double min_stagger_margin = std::numeric_limits<double>::max();
        for (double first_gap : first_row_gap_positions) {
            double stagger = std::abs(second_row_gap - first_gap);
            min_stagger_margin = std::min(min_stagger_margin, stagger);
        }
        if (min_stagger_margin < min_stagger) {
            std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() << " invalid, stagger margin " 
                      << min_stagger_margin * 1000.0 << " mm < " << min_stagger * 1000.0 << " mm" << std::endl;
            valid = false;
        }

        if (valid) {
            double score = scoreCandidate(candidate, distances, max_length - total_width, candidates, max_length);
            if (score > best_score) {
                best_score = score;
                first_shingle = candidate;
                best_index = candidate_index;
            }
        }
    }

    if (first_shingle) {
        auto candidate_aligned = alignAndShiftFirstBox(first_row_aligned[0], first_shingle, max_gap, max_length, 0.0);
        double second_row_gap = current_right_edge.x() + candidate_aligned->extent_.x() + max_gap;
        double min_stagger_margin = std::numeric_limits<double>::max();
        for (double first_gap : first_row_gap_positions) {
            double stagger = std::abs(second_row_gap - first_gap);
            min_stagger_margin = std::min(min_stagger_margin, stagger);
        }
        std::cout << "[DEBUG] First shingle selected, width: " << candidate_aligned->extent_.x() 
                  << ", stagger margin: " << min_stagger_margin * 1000.0 << " mm" << std::endl;

        if (vis_candidates) {
            visualizeShingleRows(first_row_aligned, {candidate_aligned});
        }
        second_row.push_back(candidate_aligned);
        current_right_edge = updateRightEdge(current_right_edge, candidate_aligned, max_gap);
        total_width += candidate_aligned->extent_.x();
        candidates.erase(candidates.begin() + best_index);
    } else {
        std::cerr << "[ERROR] No valid first shingle found!\n";
        return second_row;
    }

    // Step 2: Continue placing additional shingles
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> last_selected_shingle = first_shingle;
    int candidate_counter = 2;

    while (total_width < max_length && candidates.size() > 0) {
        double remaining_width = max_length - total_width;
        std::cout << "[DEBUG] Remaining width: " << remaining_width << std::endl;

        // Update distances
        distances.clear();
        for (const auto& bbox : first_row_aligned) {
            double new_distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
            distances.push_back(new_distance);
        }
        distances.erase(std::remove_if(distances.begin(), distances.end(),
            [&](double d) { return d > max_candidate_width + 0.05 || d < 0; }), distances.end());

        // Find best candidate
        std::shared_ptr<open3d::geometry::OrientedBoundingBox> best_candidate = nullptr;
        best_score = -std::numeric_limits<double>::max();
        best_index = 0;
        bool valid_candidate_found = false;

        for (size_t i = 0; i < candidates.size(); ++i) {
            auto candidate = candidates[i];
            bool valid = true;
            bool is_final_shingle = (remaining_width <= max_candidate_width);
            double local_min_stagger = is_final_shingle ? 0.0 : min_stagger;

            auto next_shingle = alignAndShiftNextBox(last_selected_shingle, candidate, max_gap);
            double second_row_gap = current_right_edge.x() + next_shingle->extent_.x() + max_gap;
            double min_stagger_margin = std::numeric_limits<double>::max();
            for (double first_gap : first_row_gap_positions) {
                double stagger = std::abs(second_row_gap - first_gap);
                min_stagger_margin = std::min(min_stagger_margin, stagger);
            }
            if (min_stagger_margin < local_min_stagger) {
                std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() << " invalid, stagger margin " 
                          << min_stagger_margin * 1000.0 << " mm < " << local_min_stagger * 1000.0 << " mm" << std::endl;
                valid = false;
            }

            // Additional check for final shingle to ensure overhang <= 50 mm
            if (is_final_shingle) {
                double overhang = total_width + candidate->extent_.x() - max_length;
                if (overhang > 0.05 || overhang < -0.05) {
                    valid = false;
                    std::cout << "[DEBUG] Candidate width " << candidate->extent_.x() << " invalid, overhang " 
                              << overhang * 1000.0 << " mm not in [-50, 50] mm" << std::endl;
                }
            }

            if (valid) {
                valid_candidate_found = true;
                double score = scoreCandidate(candidate, distances, remaining_width, candidates, max_length);
                if (score > best_score) {
                    best_score = score;
                    best_candidate = candidate;
                    best_index = i;
                }
            }
        }

        if (!best_candidate) {
            std::cout << "[DEBUG] No valid candidate found, stopping.\n";
            break;
        }

        auto next_shingle = alignAndShiftNextBox(last_selected_shingle, best_candidate, max_gap);
        double second_row_gap = current_right_edge.x() + next_shingle->extent_.x() + max_gap;
        double min_stagger_margin = std::numeric_limits<double>::max();
        for (double first_gap : first_row_gap_positions) {
            double stagger = std::abs(second_row_gap - first_gap);
            min_stagger_margin = std::min(min_stagger_margin, stagger);
        }
        std::cout << "[DEBUG] Shingle " << candidate_counter << " selected, width: " << next_shingle->extent_.x() 
                  << ", stagger margin: " << min_stagger_margin * 1000.0 << " mm" << std::endl;

        if (vis_candidates) {
            auto current_visualization = second_row;
            current_visualization.push_back(next_shingle);
            visualizeShingleRows(first_row_aligned, current_visualization);
        }

        second_row.push_back(next_shingle);
        candidate_counter++;
        last_selected_shingle = next_shingle;
        current_right_edge = updateRightEdge(current_right_edge, next_shingle, max_gap);
        total_width += next_shingle->extent_.x();
        candidates.erase(candidates.begin() + best_index);

        // Check length tolerance
        if (total_width >= max_length) {
            if (total_width > max_length + 0.05) {
                std::cout << "[DEBUG] Overhang exceeds 5 cm: " << (total_width - max_length) * 1000.0 << " mm\n";
            }
            break;
        }
    }

    // Check for undercoverage
    if (total_width < max_length) {
        std::cout << "[WARNING] Second row undercovers by: " << (max_length - total_width) * 1000.0 << " mm\n";
    }

    // Final visualization
    visualizeShingleRows(first_row_aligned, second_row, true);
    std::cout << "[DEBUG] Number of shingles selected: " << second_row.size() << std::endl;
    std::cout << "[DEBUG] Second row total width: " << total_width * 1000.0 << " mm, max_length: " 
              << max_length * 1000.0 << " mm, Difference: " << (total_width - max_length) * 1000.0 << " mm" << std::endl;


    ///////////////
    // 
    // Debug output for all selected shingles
    debugShingleRow(second_row, first_row_aligned, first_row_gap_positions, max_gap);

    return second_row;
}


///////////////////////////////////
void GeometryProcessor::visualizeShingleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
    bool color_code) 
{
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("Shingle Rows", 1280, 720);

    // Add first row (blue)
    for (const auto& bbox : first_row) {
        auto mesh = open3d::geometry::TriangleMesh::CreateFromOrientedBoundingBox(*bbox);
        mesh->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0)); // Blue
        vis.AddGeometry(mesh);
    }

    // Compute total width and length fit
    double total_width = 0.0;
    for (const auto& bbox : second_row) {
        total_width += bbox->extent_.x();
    }
    double first_row_length = 0.0;
    for (const auto& bbox : first_row) {
        first_row_length = std::max(first_row_length, bbox->GetCenter().x() + bbox->extent_.x() / 2.0);
    }
    first_row_length -= first_row[0]->GetCenter().x() - first_row[0]->extent_.x() / 2.0;

    // Add second row with color coding
    std::vector<double> first_row_right_edges;
    for (const auto& bbox : first_row) {
        first_row_right_edges.push_back(bbox->GetCenter().x() + bbox->extent_.x() / 2.0);
    }

    for (size_t i = 0; i < second_row.size(); ++i) {
        auto bbox = second_row[i];
        auto mesh = open3d::geometry::TriangleMesh::CreateFromOrientedBoundingBox(*bbox);
        
        if (color_code) {
            // Compute minimum stagger margin
            double right_edge = bbox->GetCenter().x() + bbox->extent_.x() / 2.0;
            double min_margin = std::numeric_limits<double>::max();
            for (double first_edge : first_row_right_edges) {
                min_margin = std::min(min_margin, std::abs(right_edge - first_edge));
            }
            // Color based on margin and length fit
            bool length_ok = total_width >= first_row_length && total_width <= first_row_length + 0.03;
            if (min_margin > 0.035 && length_ok) {
                mesh->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0)); // Green: Stagger > 35 mm, good length
            } else if (min_margin >= 0.03 && length_ok) {
                mesh->PaintUniformColor(Eigen::Vector3d(1.0, 1.0, 0.0)); // Yellow: Stagger 30–35 mm, good length
            } else {
                mesh->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red: Stagger < 30 mm or bad length
            }
        } else {
            mesh->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red: No color coding
        }
        vis.AddGeometry(mesh);
    }

    vis.Run();
    vis.DestroyVisualizerWindow();
}










//////////////////////////////////////////

////////////////////////////////////////////////////
std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> 
GeometryProcessor::findNextBestShinglesForMultipleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row, 
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates, 
    int num_rows, double min_stagger, double max_gap, double max_length) 
{
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> arranged_rows;
    
    // Step 1: Make a copy of first_row to ensure it's not modified
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> first_row_copy;
    for (const auto& bbox : first_row) {
        auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox) ;
        first_row_copy.push_back(bbox_copy);
    }

    //arranged_rows.push_back(first_row_copy);
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> previous_row = first_row_copy;


    
    auto current_candidates_copy = copyBoundingBoxes(candidates);
    std::cout << "Starting row arrangement for " << num_rows << " rows.\n";
    std::cout << "[DEBUG] Initial number of candidates: " << current_candidates_copy.size() << "\n";


    for (int i = 1; i < num_rows; ++i) {
        std::cout << "Finding best shingles for row " << i + 1 << "...\n";
        std::cout << "[DEBUG] Number of candidates at the beginning of row " << i + 1 << ": " << current_candidates_copy.size() << std::endl;

        // Step 3: Call findNextBestShingles without modifying original candidates
        auto next_row = findNextBestShingles(previous_row, current_candidates_copy, min_stagger, max_gap, max_length, true);
        std::cout << "[DEBUG] Candidates AFTER arranging row " << i + 1 << ": " << current_candidates_copy.size() << "\n";


        if (next_row.empty()) {
            std::cout << "No valid shingles found for row " << i + 1 << ". Stopping arrangement.\n";
            break;
        }

        // // Step 4: Remove selected shingles from current_candidates AFTER each iteration
        // for (const auto& shingle : next_row) {
        //     current_candidates.erase(std::remove_if(current_candidates.begin(), current_candidates.end(),
        //         [&shingle](const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate) {
        //             return candidate == shingle;
        //         }), current_candidates.end());
        // }
        std::cout << "[DEBUG] Candidates AFTER REMOVING used shingles in row " << i + 1 << ": " << current_candidates_copy.size() << "\n";

        // Store the next row and update previous_row for the next iteration
        arranged_rows.push_back(next_row);
        previous_row = copyBoundingBoxes(next_row); // Work with copies instead

        std::cout << "Row " << i + 1 << " completed with " << next_row.size() << " shingles.\n";
    }

    // Step 5: Update the original candidates list
    candidates = current_candidates_copy;

    std::cout << "Shingle arrangement completed with " << arranged_rows.size() << " rows.\n";
    return arranged_rows;
}
///////////////////////////////////////



////////////////////////////////////////
std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::copyBoundingBoxes(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& input_boxes) {
    
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> copied_boxes;
    
    for (const auto& box : input_boxes) {
        // Create a new shared pointer with copied data
        auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*box);
        copied_boxes.push_back(bbox_copy);
    }

    return copied_boxes;
}


///////////////////////////////////////////
Eigen::Vector3d GeometryProcessor::updateRightEdge(
    const Eigen::Vector3d& current_right_edge,
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate ,double gap ) {
    // Update the right edge based on the candidate's position and width
    Eigen::Vector3d new_right_edge = current_right_edge;
    new_right_edge.x() += candidate->extent_.x() + gap ;  // Move the right edge by the candidate's width
    return new_right_edge;
}



///////////////////////////////////////////
std::shared_ptr<open3d::geometry::OrientedBoundingBox> GeometryProcessor::alignAndShiftFirstBox(
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& reference_box,
    std::shared_ptr<open3d::geometry::OrientedBoundingBox>& target_box,
    double gap,
    double max_length,
    double rotation_angle) 
{
    // --- Step 1: Compute the bottom left corner of the reference box.
    Eigen::Vector3d f_center = reference_box->GetCenter();
    Eigen::Vector3d f_extent = reference_box->extent_;
    Eigen::Matrix3d f_R = reference_box->R_;
    
    // Adjust to use the bottom left corner
    Eigen::Vector3d f_local_offset(-f_extent.x() / 2.0, -f_extent.y() / 2.0, -f_extent.z() / 2.0);
    Eigen::Vector3d f_global_corner = f_center + f_R * f_local_offset;

    // --- Step 2: Compute the bottom left corner of the target box.
    Eigen::Vector3d s_center = target_box->GetCenter();
    Eigen::Vector3d s_extent = target_box->extent_;
    Eigen::Matrix3d s_R = target_box->R_;
    
    // Adjust to use the bottom left corner
    Eigen::Vector3d s_local_offset(-s_extent.x() / 2.0, -s_extent.y() / 2.0, -s_extent.z() / 2.0);
    Eigen::Vector3d s_global_corner = s_center + s_R * s_local_offset;

    // --- Step 3: Translate the target box to match the reference box’s bottom left corner.
    Eigen::Vector3d trans = f_global_corner - s_global_corner;
    target_box->Translate(trans);

    // --- Step 4: Rotate the target box to match the reference box’s orientation.
    Eigen::Matrix3d R_transform = f_R * s_R.transpose();
    target_box->Rotate(R_transform, f_global_corner);

    // --- Step 5: Return the aligned and rotated box.
    return target_box;
}

/////////////////////////////////////////////////////

std::shared_ptr<open3d::geometry::OrientedBoundingBox> GeometryProcessor::alignAndShiftNextBox(
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& last_selected_shingle,
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate,
    double gap)
{
    auto aligned_candidate = std::make_shared<open3d::geometry::OrientedBoundingBox>(*candidate);

    // Step 1: Get the rotation and apply it to the candidate
    aligned_candidate->R_ = last_selected_shingle->R_; // Match orientation
    Eigen::Matrix3d R = aligned_candidate->R_;

    // Step 2: Compute bottom-right corner of last shingle
    Eigen::Vector3d last_center = last_selected_shingle->GetCenter();
    Eigen::Vector3d half_extent_last = last_selected_shingle->extent_ / 2.0;

    Eigen::Vector3d bottom_right_last = last_center
        + R.col(0) * half_extent_last.x()  // right
        - R.col(1) * half_extent_last.y(); // bottom

    // Step 3: Compute bottom-left corner of candidate
    Eigen::Vector3d cand_center = aligned_candidate->GetCenter();
    Eigen::Vector3d half_extent_cand = aligned_candidate->extent_ / 2.0;

    Eigen::Vector3d bottom_left_cand = cand_center
        - R.col(0) * half_extent_cand.x()  // left
        - R.col(1) * half_extent_cand.y(); // bottom

    // Step 4: Compute translation vector to align corners + apply gap in X direction
    Eigen::Vector3d shift = (bottom_right_last + R.col(0) * gap) - bottom_left_cand;

    aligned_candidate->Translate(shift);

    return aligned_candidate;
}



///////////////////////////////////////////////////////////////
// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
// GeometryProcessor::arrangeShingleRow(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& target_row,
//     double gap,
//     double max_length,
//     double rotation_angle,
//     double vertical_overlap) { // New parameter for vertical shift

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;

//     if (reference_row.empty() || target_row.empty()) {
//         return arranged_bboxes;
//     }

//     // Step 1: Align the first box of the target row with the first box of the reference row
//     auto first_box_target_row = target_row[0];  
//     auto first_box_reference_row = reference_row[0];

//     // Align first box of the target row to the first box of the reference row
//     auto first_box_target_row_aligned = alignAndShiftFirstBox(reference_row[0], target_row[0], gap, max_length, rotation_angle);

//     // Add the aligned first box of target row to the arranged list
//     arranged_bboxes.push_back(first_box_target_row_aligned);

//     // Step 2: Compute the X-direction (for horizontal alignment)
//     Eigen::Vector3d reference_point_first_row = reference_row[0]->GetCenter();
//     Eigen::Vector3d reference_point_second_row = reference_row[1]->GetCenter();
//     Eigen::Vector3d x_direction = (reference_point_second_row - reference_point_first_row).normalized();

//     // Step 3: Arrange the rest of the target row relative to the first box in the target row
//     double total_length = first_box_target_row_aligned->extent_.x(); // Track row length
    
//     // // // 
//     //bool min_length_reached = false;  // Flag to ensure we reach at least max_length

//     for (size_t i = 1; i < target_row.size(); ++i) {
//         auto& bbox = target_row[i];

//         // Ensure orientation matches the reference row
//         Eigen::Matrix3d rotation_fix = first_box_reference_row->R_ * bbox->R_.transpose();
//         bbox->Rotate(rotation_fix, bbox->GetCenter());

//         // Move relative to the last arranged box (ensuring positive X)
//         double spacing = arranged_bboxes.back()->extent_.x() / 2.0 + gap + bbox->extent_.x() / 2.0;
//         Eigen::Vector3d shift_x = spacing * x_direction;  

//         // Apply translation to place it next to the last arranged box
//         bbox->Translate(arranged_bboxes.back()->GetCenter() + shift_x - bbox->GetCenter());

//         // Predict next position before placing
//         double new_total_length = total_length + arranged_bboxes.back()->extent_.x() / 2.0 + gap + bbox->extent_.x() / 2.0;

//         // // // 
//         // Allow placing shingles until we reach at least max_length
//         // if (min_length_reached && new_total_length > max_length) {
//         //     break;  // Stop only after reaching at least max_length
//         // }

//         total_length = new_total_length;

//         // // // 
//         // // Mark that we have reached at least max_length
//         // if (total_length >= max_length) {
//         //     min_length_reached = true;
//         // }

//         arranged_bboxes.push_back(bbox);
//     }

//     // Step 4: Compute the Y-direction (for vertical alignment)
//     Eigen::Vector3d y_direction = reference_row[0]->R_.col(1); // Local Y-axis of the reference row

//     // Step 5: Apply vertical overlap shift using local Y direction
//     Eigen::Vector3d vertical_shift = y_direction * vertical_overlap; 

//     // Apply the vertical shift to each box in the target row
//     for (auto& bbox : arranged_bboxes) {
//         bbox->Translate(vertical_shift); // Correct vertical translation based on overlap
//     }

//     // Step 6: Stack the target row above the reference row by the thickness of the first box
//     double thickness = reference_row[0]->extent_.z(); // Thickness of the first box in the reference row
//     Eigen::Vector3d stack_shift = reference_row[0]->R_ * Eigen::Vector3d(0, 0, thickness);

//     for (auto& bbox : arranged_bboxes) {
//         bbox->Translate(stack_shift); // Stack them above reference row
//     }

//     // Print the number of shingles and total length
//     std::cout << "[INFO] Number of shingles arranged in current row: " << arranged_bboxes.size() << std::endl;
//     std::cout << "[INFO] Total current row length: " << total_length << " meters" << std::endl;

//     return arranged_bboxes;
// }


// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
// GeometryProcessor::arrangeShingleRow(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& target_row,
//     double gap,
//     double max_length,
//     double rotation_angle,
//     double vertical_overlap) {

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;

//     if (reference_row.empty() || target_row.empty()) {
//         return arranged_bboxes;
//     }

//     // Align and place the first box in the target row
//     auto first_box_aligned = alignAndShiftFirstBox(reference_row[0], target_row[0], gap, max_length, rotation_angle);
//     arranged_bboxes.push_back(first_box_aligned);
//     double total_length = first_box_aligned->extent_.x();

//     for (size_t i = 1; i < target_row.size(); ++i) {
//         auto& candidate = target_row[i];

//         // Match orientation to the reference row
//         Eigen::Matrix3d rotation_fix = reference_row[0]->R_ * candidate->R_.transpose();
//         candidate->Rotate(rotation_fix, candidate->GetCenter());

//         // Compute bottom-left corner of the last box
//         auto& last_box = arranged_bboxes.back();
//         Eigen::Vector3d last_bottom_left = last_box->GetCenter()
//             - 0.5 * last_box->extent_.x() * last_box->R_.col(0)
//             - 0.5 * last_box->extent_.y() * last_box->R_.col(1);

//         // Compute bottom-left corner of the candidate
//         Eigen::Vector3d candidate_bottom_left = candidate->GetCenter()
//             - 0.5 * candidate->extent_.x() * candidate->R_.col(0)
//             - 0.5 * candidate->extent_.y() * candidate->R_.col(1);

//         // Shift to place the candidate to the right of the last box with the gap
//         Eigen::Vector3d shift_direction = last_box->R_.col(0); // Local X direction
//         Eigen::Vector3d target_position = last_bottom_left + (last_box->extent_.x() + gap) * shift_direction;

//         Eigen::Vector3d shift = target_position - candidate_bottom_left;
//         candidate->Translate(shift);

//         // Update total length and check bounds
//         double new_total_length = total_length + gap + candidate->extent_.x();
//         if (new_total_length > max_length) {
//             break;
//         }

//         total_length = new_total_length;
//         arranged_bboxes.push_back(candidate);
//     }

//     // Vertical overlap shift (along local Y of reference box)
//     Eigen::Vector3d y_direction = reference_row[0]->R_.col(1);
//     Eigen::Vector3d vertical_shift = y_direction * vertical_overlap;
//     for (auto& box : arranged_bboxes) {
//         box->Translate(vertical_shift);
//     }

//     // Stack on top (along local Z of reference box)
//     double thickness = reference_row[0]->extent_.z();
//     Eigen::Vector3d z_shift = reference_row[0]->R_ * Eigen::Vector3d(0, 0, thickness);
//     for (auto& box : arranged_bboxes) {
//         box->Translate(z_shift);
//     }

//     std::cout << "[INFO] Number of shingles arranged in current row: " << arranged_bboxes.size() << std::endl;
//     std::cout << "[INFO] Total current row length: " << total_length << " meters" << std::endl;

//     return arranged_bboxes;
// }


std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
GeometryProcessor::arrangeShingleRow(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& target_row,
    double gap,
    double max_length,
    double rotation_angle,
    double vertical_overlap) {

    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;

    if (reference_row.empty() || target_row.empty()) {
        return arranged_bboxes;
    }

    // Step 1: Align and place the first box
    auto first_box_aligned = alignAndShiftFirstBox(reference_row[0], target_row[0], gap, max_length, rotation_angle);
    arranged_bboxes.push_back(first_box_aligned);
    //double total_length = first_box_aligned->extent_.x();

    // Step 2: Use helper for remaining boxes
    for (size_t i = 1; i < target_row.size(); ++i) {
        auto& candidate = target_row[i];

        // Align orientation to match the reference row
        Eigen::Matrix3d rotation_fix = reference_row[0]->R_ * candidate->R_.transpose();
        candidate->Rotate(rotation_fix, candidate->GetCenter());

        // Use helper to align and place the box
        auto aligned_candidate = alignAndShiftNextBox(arranged_bboxes.back(), candidate, gap);

        // // Check length constraint : no need to do this, we arrange all boxes in the target_row
        // double new_total_length = total_length + gap + aligned_candidate->extent_.x();
        // if (new_total_length > max_length) {
        //     break;
        // }

        arranged_bboxes.push_back(aligned_candidate);
        //total_length = new_total_length;
    }

    // Step 3: Apply vertical overlap shift
    Eigen::Vector3d y_direction = reference_row[0]->R_.col(1);
    Eigen::Vector3d vertical_shift = y_direction * vertical_overlap;
    for (auto& box : arranged_bboxes) {
        box->Translate(vertical_shift);
    }

    // Step 4: Stack on top
    double thickness = reference_row[0]->extent_.z();
    Eigen::Vector3d z_shift = reference_row[0]->R_ * Eigen::Vector3d(0, 0, thickness);
    for (auto& box : arranged_bboxes) {
        box->Translate(z_shift);
    }

    std::cout << "[INFO] Number of shingles arranged in current row: " << arranged_bboxes.size() << std::endl;
    //std::cout << "[INFO] Total current row length: " << total_length << " meters" << std::endl;

    return arranged_bboxes;
}


////////////////////////////////////////////////////
// std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>
// GeometryProcessor::arrangeMultipleShingleRows(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
//     std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& candidate_rows,
//     double gap,
//     double max_length,
//     double rotation_angle,
//     double vertical_overlap) 
// {
//     std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> candidate_rows_copy;
    
//     for (const auto& row : candidate_rows) {
//         candidate_rows_copy.push_back(copyBoundingBoxes(row));  // Deep copy each row's bounding boxes
//     }

//     std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> arranged_rows;
//     std::cout << "Arranging multiple rows...\n";

//     // Start with the first reference row
//     auto previous_row = reference_row;

//     // Step 1: Arrange each candidate row based on the previous row
//     for (size_t i = 0; i < candidate_rows_copy.size(); ++i) {
//         std::cout << "Arranging row " << i + 1 << "...\n";

//         auto candidate_row_copy = copyBoundingBoxes(candidate_rows_copy[i]);

//         // Step 2: Align and stack the current row on top of the previous row
//         auto arranged_row = arrangeShingleRow(previous_row, candidate_row_copy, gap, max_length, rotation_angle, 0);

//         if (arranged_row.empty()) {
//             std::cout << "No valid shingles found for row " << i + 1 << ". Skipping this row.\n";
//             continue;
//         }


//         // Step 4: Add the arranged row to the list of arranged rows
//         arranged_rows.push_back(arranged_row);

//         // Update the reference for the next row
//         previous_row = arranged_row;
//     }

//     //Step 5: Apply vertical overlap shift if needed
//     if (vertical_overlap != 0) {
//         Eigen::Vector3d y_direction = reference_row[0]->R_.col(1);

//         for (size_t i = 0; i < arranged_rows.size(); ++i) {
//             Eigen::Vector3d vertical_shift = y_direction * vertical_overlap * (i + 1);

//             for (auto& bounding_box : arranged_rows[i]) {
//                 bounding_box->Translate(vertical_shift);
//             }
//         }
//     }



//     std::cout << "Shingle arrangement for multiple rows completed.\n";
//     return arranged_rows;
// }


std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>
GeometryProcessor::arrangeMultipleShingleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& candidate_rows,
    double gap,
    double max_length,
    double rotation_angle,
    double third_fourth_overlap,  // New argument for 3rd & 4th row overlap
    double staggered_vertical_overlap)     // New argument for 5th row onward
{
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> candidate_rows_copy;
    
    for (const auto& row : candidate_rows) {
        candidate_rows_copy.push_back(copyBoundingBoxes(row));  // Deep copy each row's bounding boxes
    }

    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> arranged_rows;
    std::cout << "Arranging multiple rows...\n";

    auto previous_row = reference_row;

    for (size_t i = 0; i < candidate_rows_copy.size(); ++i) {
        std::cout << "Arranging row " << i + 1 << "...\n";

        auto candidate_row_copy = copyBoundingBoxes(candidate_rows_copy[i]);

        auto arranged_row = arrangeShingleRow(previous_row, candidate_row_copy, gap, max_length, rotation_angle, 0);

        if (arranged_row.empty()) {
            std::cout << "No valid shingles found for row " << i + 1 << ". Skipping this row.\n";
            continue;
        }

        arranged_rows.push_back(arranged_row);
        previous_row = arranged_row;
    }

    // Step 5: Apply vertical overlap shift based on row index
    if (third_fourth_overlap != 0 || staggered_vertical_overlap != 0) {
        Eigen::Vector3d y_direction = reference_row[0]->R_.col(1);

        for (size_t i = 0; i < arranged_rows.size(); ++i) {
            Eigen::Vector3d vertical_shift;

            if (i < 2) {  // 3rd and 4th rows (index 0 & 1 in arranged_rows)
                vertical_shift = y_direction * (-third_fourth_overlap);
            } else {  // From 5th row onward
                vertical_shift = y_direction * (-staggered_vertical_overlap * (i - 1));
            }

            for (auto& bbox : arranged_rows[i]) {
                bbox->Translate(vertical_shift);
            }
        }
    }

    std::cout << "Shingle arrangement for multiple rows completed.\n";
    return arranged_rows;
}

///////////////////////////////
std::vector<PC_o3d_ptr> GeometryProcessor::alignPointCloudsToArrangedBoxes(
    const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& arranged_boxes,
    const std::vector<std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>>& original_box_cloud_pairs)
{
    std::vector<PC_o3d_ptr> aligned_clouds;

    for (const auto& row : arranged_boxes) {
        for (const auto& arranged_box_ptr : row) {
            const auto& arranged_box = *arranged_box_ptr;

            // Find corresponding original box
            auto it = std::find_if(
                original_box_cloud_pairs.begin(),
                original_box_cloud_pairs.end(),
                [&arranged_box](const std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>& pair) {
                    const auto& orig_box = pair.first;
                    return (orig_box.extent_ - arranged_box.extent_).norm() < 1e-4;
                });

            if (it == original_box_cloud_pairs.end()) {
                std::cerr << "Warning: No matching box found for arranged box.\n";
                continue;
            }

            const auto& [original_box, cloud] = *it;

            // Compute transformation from original box to arranged box
            Eigen::Matrix3d R_src = original_box.R_;
            Eigen::Vector3d t_src = original_box.center_;
            Eigen::Matrix3d R_dst = arranged_box.R_;
            Eigen::Vector3d t_dst = arranged_box.center_;

            Eigen::Matrix3d R_align = R_dst * R_src.transpose();
            Eigen::Vector3d t_align = t_dst - R_align * t_src;

            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            transform.block<3, 3>(0, 0) = R_align;
            transform.block<3, 1>(0, 3) = t_align;

            // Transform the cloud
            auto aligned_cloud = std::make_shared<open3d::geometry::PointCloud>(*cloud);
            aligned_cloud->Transform(transform);

            aligned_clouds.push_back(aligned_cloud);
        }
    }

    return aligned_clouds;
}


////////////////////////////////

void GeometryProcessor::visualizeAllShingleRows(
    const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& arranged_rows)  
{
    if (arranged_rows.empty()) {
        std::cout << "No rows to visualize.\n";
        return;
    }

    std::cout << "Visualizing " << arranged_rows.size() << " rows...\n";

    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Shingle Rows Visualization", 800, 600);

    // Define a set of distinct colors (cycling through if rows > available colors)
    std::vector<Eigen::Vector3d> colors = {
        {1.0, 0.0, 0.0}, // Red
        {0.0, 1.0, 0.0}, // Green
        {0.0, 0.0, 1.0}, // Blue
        {1.0, 0.0, 1.0}, // Magenta
        {0.0, 1.0, 1.0}  // Cyan
    };

    for (size_t i = 0; i < arranged_rows.size(); ++i) {
        Eigen::Vector3d color = colors[i % colors.size()]; // Cycle through colors if needed

        for (const auto& shingle : arranged_rows[i]) {
            auto colored_box = std::make_shared<open3d::geometry::OrientedBoundingBox>(*shingle);
            colored_box->color_ = color;
            visualizer.AddGeometry(colored_box);
        }

        std::cout << "Row " << i + 1 << " visualized with color: [" 
                  << color.x() << ", " << color.y() << ", " << color.z() << "]\n";
    }

    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}


///////////////////////////////////////////////////////
// void GeometryProcessor::visualize_bounding_boxes(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes) {
//     // Create a visualizer object
//     open3d::visualization::Visualizer visualizer;
//     visualizer.CreateVisualizerWindow("Bounding Boxes Visualization");

//     // Create global coordinate axes using lines
//     auto axis_lines = std::make_shared<open3d::geometry::LineSet>();
//     axis_lines->points_ = {
//         Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),  // X-axis (Red)
//         Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 0),  // Y-axis (Green)
//         Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)   // Z-axis (Blue)
//     };
//     axis_lines->lines_ = {
//         Eigen::Vector2i(0, 1),  // X-axis
//         Eigen::Vector2i(2, 3),  // Y-axis
//         Eigen::Vector2i(4, 5)   // Z-axis
//     };
//     axis_lines->colors_ = {
//         Eigen::Vector3d(1, 0, 0),  // Red for X-axis
//         Eigen::Vector3d(0, 1, 0),  // Green for Y-axis
//         Eigen::Vector3d(0, 0, 1)   // Blue for Z-axis
//     };

//     // Add the global axes to the visualizer
//     visualizer.AddGeometry(axis_lines);



//     // Visualize bounding boxes
//     for (const auto& bbox : bounding_boxes) {
//         visualizer.AddGeometry(bbox);

//         //     // // Use built-in coordinate frame for axes
//         // auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1 , bbox->GetCenter());
//         // visualizer.AddGeometry(coordinate_frame);
//     }

//     // Start the visualizer
//     visualizer.Run();
//     visualizer.DestroyVisualizerWindow();
// }



void GeometryProcessor::visualize_bounding_boxes(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes) {

    // Create a visualizer object
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Bounding Boxes Visualization");


    // Use built-in coordinate frame for axes
    auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
    visualizer.AddGeometry(coordinate_frame);

    // Visualize bounding boxes and their axes
    for (const auto& bbox : bounding_boxes) {
        visualizer.AddGeometry(bbox);

        // Get rotation matrix and center of the bounding box
        Eigen::Matrix3d rotation = bbox->R_;  // Rotation matrix
        Eigen::Vector3d center = bbox->GetCenter();  // Center of the box

        // Define length for the axes (let's use a fixed length, e.g., 0.1m for visualization)
        double axis_length = 0.1;

        // Create lines for the 3D axes of the bounding box
        open3d::geometry::LineSet bbox_axes;
        bbox_axes.points_ = {
            center, center + rotation.col(0) * axis_length,  // X-axis (Red)
            center, center + rotation.col(1) * axis_length,  // Y-axis (Green)
            center, center + rotation.col(2) * axis_length   // Z-axis (Blue)
        };
        bbox_axes.lines_ = {
            Eigen::Vector2i(0, 1),  // X-axis
            Eigen::Vector2i(2, 3),  // Y-axis
            Eigen::Vector2i(4, 5)   // Z-axis
        };
        bbox_axes.colors_ = {
            Eigen::Vector3d(1, 0, 0),  // Red for X-axis
            Eigen::Vector3d(0, 1, 0),  // Green for Y-axis
            Eigen::Vector3d(0, 0, 1)   // Blue for Z-axis
        };

        // Add the axes of the bounding box to the visualizer
        visualizer.AddGeometry(std::make_shared<open3d::geometry::LineSet>(bbox_axes));
    }

    // Start the visualizer
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}




//////////////////////////////////////////////////
//
// void GeometryProcessor::visualizeShingleRows(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row) {

//     // Create a vector to store all geometries for visualization
//     std::vector<std::shared_ptr<const open3d::geometry::Geometry>> all_geometries;

//     // Define colors for the rows
//     Eigen::Vector3d first_row_color(0, 0, 1);  // Blue for the first row
//     Eigen::Vector3d second_row_color(1, 0, 0); // Red for the second row

//     // Convert the first row OrientedBoundingBox objects to Geometry pointers with color
//     for (const auto& bbox : first_row) {
//         auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox); // Create a copy
//         bbox_copy->color_ = first_row_color; // Set color
//         all_geometries.push_back(bbox_copy);
//     }

//     // Convert the second row OrientedBoundingBox objects to Geometry pointers with color
//     for (const auto& bbox : second_row) {
//         auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox); // Create a copy
//         bbox_copy->color_ = second_row_color; // Set color
//         all_geometries.push_back(bbox_copy);
//     }

//     // Create global coordinate axes using lines
//     auto axis_lines = std::make_shared<open3d::geometry::LineSet>();
//     axis_lines->points_ = {
//         Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),  // X-axis (Red)
//         Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 0),  // Y-axis (Green)
//         Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)   // Z-axis (Blue)
//     };
//     axis_lines->lines_ = {
//         Eigen::Vector2i(0, 1),  // X-axis
//         Eigen::Vector2i(2, 3),  // Y-axis
//         Eigen::Vector2i(4, 5)   // Z-axis
//     };
//     axis_lines->colors_ = {
//         Eigen::Vector3d(1, 0, 0),  // Red for X-axis
//         Eigen::Vector3d(0, 1, 0),  // Green for Y-axis
//         Eigen::Vector3d(0, 0, 1)   // Blue for Z-axis
//     };

//     // Add the global axes to the geometries list
//     all_geometries.push_back(axis_lines);

//     // Visualize using Open3D
//     open3d::visualization::DrawGeometries(all_geometries, "Shingles Visualization", 1600, 900);
// }


void GeometryProcessor::visualizeShingleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row) {

    // Create a vector to store all geometries for visualization
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> all_geometries;

    // Define colors for the rows
    Eigen::Vector3d first_row_color(0, 0, 1);  // Blue
    Eigen::Vector3d second_row_color(1, 0, 0); // Red

    // Add first row bounding boxes
    for (const auto& bbox : first_row) {
        auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox);
        bbox_copy->color_ = first_row_color;
        all_geometries.push_back(bbox_copy);
    }

    // Add second row bounding boxes
    for (const auto& bbox : second_row) {
        auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox);
        bbox_copy->color_ = second_row_color;
        all_geometries.push_back(bbox_copy);
    }

    // Use built-in coordinate frame for axes
    auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
    all_geometries.push_back(coordinate_frame);


    //     // Create global coordinate axes using lines
    // auto axis_lines = std::make_shared<open3d::geometry::LineSet>();
    // axis_lines->points_ = {
    //     Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),  // X-axis (Red)
    //     Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 0),  // Y-axis (Green)
    //     Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)   // Z-axis (Blue)
    // };
    // axis_lines->lines_ = {
    //     Eigen::Vector2i(0, 1),  // X-axis
    //     Eigen::Vector2i(2, 3),  // Y-axis
    //     Eigen::Vector2i(4, 5)   // Z-axis
    // };
    // axis_lines->colors_ = {
    //     Eigen::Vector3d(1, 0, 0),  // Red for X-axis
    //     Eigen::Vector3d(0, 1, 0),  // Green for Y-axis
    //     Eigen::Vector3d(0, 0, 1)   // Blue for Z-axis
    // };

    //  all_geometries.push_back(axis_lines);


    // Create an interactive visualizer
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Shingles Visualization", 1600, 900);
    for (auto& geom : all_geometries) {
        visualizer.AddGeometry(geom);
    }
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}


////////////////////////////////////////////
// Function to visualize the 3D plane of last_selected_shingle
void GeometryProcessor::visualizeShinglePlane(const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& box) {
    // Extract rotation matrix (orientation of the box)
    Eigen::Matrix3d R = box->R_;
    Eigen::Vector3d center = box->GetCenter();

    // Extract local coordinate axes
    Eigen::Vector3d x_axis = R.col(0);  // X-axis (Red)
    Eigen::Vector3d y_axis = R.col(1);  // Y-axis (Green)
    Eigen::Vector3d z_axis = R.col(2);  // Z-axis (Blue)

    double axis_length = 0.1; // Length of the visualization axes

    // Define line sets for the three local axes
    auto x_axis_line = std::make_shared<open3d::geometry::LineSet>();
    x_axis_line->points_ = {center, center + axis_length * x_axis};
    x_axis_line->lines_ = {{0, 1}};
    x_axis_line->colors_.push_back(Eigen::Vector3d(1, 0, 0)); // Red for x-axis

    auto y_axis_line = std::make_shared<open3d::geometry::LineSet>();
    y_axis_line->points_ = {center, center + axis_length * y_axis};
    y_axis_line->lines_ = {{0, 1}};
    y_axis_line->colors_.push_back(Eigen::Vector3d(0, 1, 0)); // Green for y-axis

    auto z_axis_line = std::make_shared<open3d::geometry::LineSet>();
    z_axis_line->points_ = {center, center + axis_length * z_axis};
    z_axis_line->lines_ = {{0, 1}};
    z_axis_line->colors_.push_back(Eigen::Vector3d(0, 0, 1)); // Blue for z-axis

    // Print the local axes
    std::cout << "Shingle Local Axes:\n";
    std::cout << "X-Axis: " << x_axis.transpose() << std::endl;
    std::cout << "Y-Axis: " << y_axis.transpose() << std::endl;
    std::cout << "Z-Axis: " << z_axis.transpose() << std::endl;

    // Show visualization with the box and its local axes
    open3d::visualization::DrawGeometries({box, x_axis_line, y_axis_line, z_axis_line}, "Shingle 3D Plane");
}


////////////////////////////////////////////////
void GeometryProcessor::visualizePointClouds(
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& clouds, 
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud = nullptr
) {
    // Create a Visualizer instance
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("PointClouds Visualization");

    // Add each point cloud from the vector
    for (const auto& cloud : clouds) {
        visualizer.AddGeometry(cloud);
    }

    // Optionally add a single point cloud if provided
    if (point_cloud != nullptr) {
        visualizer.AddGeometry(point_cloud);
    }

    // Run the visualizer
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}











/////////////////////////////////////////////
std::shared_ptr<open3d::geometry::TriangleMesh> GeometryProcessor::CreateMeshFromOrientedBoundingBox(
    const open3d::geometry::OrientedBoundingBox& obb, 
    const Eigen::Vector3d& color) {

    auto mesh = open3d::geometry::TriangleMesh::CreateFromOrientedBoundingBox(obb);
    if (!mesh) {
        std::cerr << "Failed to create mesh from OBB" << std::endl;
        return nullptr;
    }

    // EXPLICITLY ASSIGN COLORS TO EACH VERTEX
    mesh->vertex_colors_.clear();
    for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
        mesh->vertex_colors_.push_back(color);  // Assign per vertex
    }

    // 🔹 Ensure the color assignment is applied correctly
    if (mesh->vertex_colors_.size() != mesh->vertices_.size()) {
        std::cerr << "Error: Vertex color count mismatch!" << std::endl;
    }

    // COMPUTE NORMALS FOR BETTER VISUALIZATION
    mesh->ComputeTriangleNormals();
    mesh->ComputeVertexNormals();

    return mesh;
}




/////////////////////////////////////////////////
void GeometryProcessor::visualizeShingleMeshes(
    const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& combined_rows,
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud = nullptr) // Optional argument
{
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> all_geometries;

    // Define a color list for different rows (extend if more rows exist)
    std::vector<Eigen::Vector3d> row_colors = {
        {0, 0, 1},   // Blue for first row
        {1, 0, 0},   // Red for second row
        {0, 1, 0},   // Green for third row
        {1, 1, 0},   // Yellow for fourth row
        {1, 0, 1},   // Magenta for fifth row
        {0, 1, 1}    // Cyan for sixth row
    };

    for (size_t i = 0; i < combined_rows.size(); ++i) {
        Eigen::Vector3d color = row_colors[i % row_colors.size()]; // Cycle through colors

        for (const auto& bbox : combined_rows[i]) {
            auto mesh = CreateMeshFromOrientedBoundingBox(*bbox, color);
            all_geometries.push_back(mesh);
        }
    }

    // If a point cloud is provided, add it to the visualization
    if (point_cloud && !point_cloud->IsEmpty()) {
        point_cloud->PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5)); // Gray color for point cloud
        all_geometries.push_back(point_cloud);
    }

    // Add coordinate frame for reference
    auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
    all_geometries.push_back(coordinate_frame);

    // Visualize the meshes and point cloud
    open3d::visualization::DrawGeometries(all_geometries);
}


//////////////////////////////////////////////////////////////////////////////
// Export function: Exports each bounding box as a PLY file using triangle mesh geometry.
void GeometryProcessor::exportBoundingBoxes(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& boxes,
    const std::string& folder,
    const Eigen::Vector3d& color,  // New parameter for color
    const std::string& prefix )
{
    int index = 0;
    for (const auto& box : boxes) {
        auto mesh = CreateMeshFromOrientedBoundingBox(*box, color);

        if (!mesh) {
            std::cerr << "Failed to create mesh from bounding box" << std::endl;
            continue;
        }

        // 🔹 Ensure every vertex gets the correct color
        for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
            mesh->vertex_colors_[i] = color;
        }

        // Export the mesh
        std::string filename = folder + "/" + prefix + "box_" + std::to_string(index) + ".ply";
        if (!open3d::io::WriteTriangleMesh(filename, *mesh)) {
            std::cerr << "Failed to write bounding box file: " << filename << std::endl;
        } else {
            std::cout << "Exported bounding box to " << filename << std::endl;
        }
        index++;
    }
}


///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// class Rectangle


// Constructor
Rectangle::Rectangle(const std::array<Eigen::Vector3d , 4>& corners)
: corners_(corners)
{ 
    
    corners_ = sortCornersClockwise(corners); // Ensure corners are sorted  
    computeProperties();
}

//////////////////////////////
double Rectangle::getWidth() const 
{
    double width = (corners_[0]- corners_[1]).norm();
    return width;
}

//////////////////////////////
double Rectangle::getLength() const 
{
double length = (corners_[1]- corners_[2]).norm();
return length ;
}

//////////////////////////////
Eigen::Vector3d Rectangle::getCenter() const { return center_;}
//////////////////////////////
Eigen::Vector3d Rectangle::getNormal() const { return normal_;}
//////////////////////////////
std::array <Eigen::Vector3d , 4> Rectangle::getSortedCorners() const { return corners_;}
//////////////////////////////
std::array <std::pair <Eigen::Vector3d ,Eigen::Vector3d > , 4>  Rectangle::getEdges() const
{
    std::array <std::pair <Eigen::Vector3d ,Eigen::Vector3d > , 4> edges;
    // each edge is represented as a pair of corner points
    for (int i =0 ; i< 4 ; ++i)
    {
        edges[i] = {corners_[i] , corners_ [(i+1) % 4]};
    }
    return edges;
}
//////////////////////////////



////////////////////////////
std::array<Eigen::Vector3d, 4> Rectangle::sortCornersClockwise
(const std::array<Eigen::Vector3d, 4>& corners) 
{
// Compute centroid    
Eigen::Vector3d centroid = (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0;
// Sort based on angle from centroid    
std::array<Eigen::Vector3d, 4> sorted_corners = corners;
std::sort(sorted_corners.begin(), sorted_corners.end(),[&centroid](const Eigen::Vector3d& a, const Eigen::Vector3d& b) 

{double angleA = atan2(a.y() - centroid.y(), a.x() - centroid.x());
double angleB = atan2(b.y() - centroid.y(), b.x() - centroid.x());
return angleA < angleB; });
return sorted_corners;}

//////////////////////////////
void Rectangle::computeProperties()
{
    // calculate the center of the rectangle (mean of corners)
    center_ = (corners_[0] + corners_[1] + corners_[2] + corners_[3]) / 4.0;

    // compute normal vectors of rectangle (cross product of two edges)
    Eigen::Vector3d edge1 = corners_[1] - corners_[0];
    Eigen::Vector3d edge2 = corners_[2] - corners_[1];
    normal_ = edge1.cross(edge2). normalized();
}


//////////////////////////////
void Rectangle::visualizeEdges() const {

    // Create a LineSet to visualize edges
    auto line_set = std::make_shared<open3d::geometry::LineSet>();
    // Convert corner points into Open3D format
    std::vector<Eigen::Vector3d> points(corners_.begin(), corners_.end());
    line_set->points_ = points;

    // Define edges using index pairs
    std::vector<Eigen::Vector2i> edges = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}  // Connect corners in order
    };
    line_set->lines_ = edges;

    // Create a visualizer and display the edges
    open3d::visualization::DrawGeometries({line_set}, "Rectangle Edges");

}

/////////////////////
// Helper function to visualize an edge (right edge in this case)
void Rectangle::visualizeEdge(const Eigen::Vector3d& start, const Eigen::Vector3d& end) const {
    // Visualization logic goes here (e.g., using Open3D or other visualization tools)
    std::cout << "Visualizing edge from: " << start.transpose() << " to " << end.transpose() << std::endl;
}

/////////////////////
// Function to get and visualize the right edge of the rectangle
std::pair<Eigen::Vector3d, Eigen::Vector3d> Rectangle::getRightEdge() const {
    // Right edge is between the upper-right and lower-right corners
    Eigen::Vector3d right_edge_start = corners_[1];  // Upper-right corner
    Eigen::Vector3d right_edge_end = corners_[5];    // Lower-right corner
    
    // Visualize the right edge (you can implement this in your visualizer code)
    visualizeEdge(right_edge_start, right_edge_end);
    
    return std::make_pair(right_edge_start, right_edge_end);
}

///////////////////////////
// Function to get and visualize the top edges of a list of rectangles
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> Rectangle::getTopEdges(const std::vector<Rectangle>& rectangles) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> top_edges;
    
    for (const auto& rectangle : rectangles) {
        // Top edge is between the upper-left and upper-right corners
        Eigen::Vector3d top_edge_start = rectangle.corners_[0];  // Upper-left corner
        Eigen::Vector3d top_edge_end = rectangle.corners_[1];    // Upper-right corner
        
        // Visualize the top edge
        rectangle.visualizeEdge(top_edge_start, top_edge_end);
        
        top_edges.push_back(std::make_pair(top_edge_start, top_edge_end));
    }
    
    return top_edges;
}






///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////



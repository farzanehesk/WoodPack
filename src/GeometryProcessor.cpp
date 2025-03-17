#include "../include/GeometryProcessor.hpp"
#include <open3d/Open3D.h>
#include <iostream>
#include <Eigen/Eigenvalues> // For SelfAdjointEigenSolver
#include <cmath>
#include <Eigen/Dense>
#include <random>
#include <chrono>



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

        // Eigen decomposition (PCA)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
        Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors(); // Principal axes

        // Ensure right-handed coordinate system
        if (eigenvectors.determinant() < 0) {
            eigenvectors.col(0) = -eigenvectors.col(0);
        }

        // Ensure correct alignment of X, Y, Z axes
        Eigen::Vector3d x_axis = eigenvectors.col(0);
        Eigen::Vector3d y_axis = eigenvectors.col(1);
        Eigen::Vector3d z_axis = eigenvectors.col(2);

        // Fix: Ensure Z-axis always points upward
        Eigen::Vector3d global_up(0, 0, 1);
        if (z_axis.dot(global_up) < 0) {
            z_axis = -z_axis;
            x_axis = -x_axis;
        }

        // Project points onto PCA space
        std::vector<Eigen::Vector3d> transformed_points;
        transformed_points.reserve(cluster->points_.size());
        for (const auto& point : cluster->points_) {
            transformed_points.push_back(eigenvectors.transpose() * (point - centroid));
        }

        // Compute min/max bounds in PCA-aligned space
        Eigen::Vector3d min_bound = transformed_points.front();
        Eigen::Vector3d max_bound = transformed_points.front();
        for (const auto& p : transformed_points) {
            min_bound = min_bound.cwiseMin(p);
            max_bound = max_bound.cwiseMax(p);
        }

        // Get extents and reorder axes (Ensure X is the shortest, Y is the longest)
        Eigen::Vector3d extents = (max_bound - min_bound).cwiseAbs();
        std::array<std::pair<double, int>, 3> sorted_axes = {
            std::make_pair(extents.x(), 0),
            std::make_pair(extents.y(), 1),
            std::make_pair(extents.z(), 2)
        };

        std::sort(sorted_axes.begin(), sorted_axes.end());

        Eigen::Matrix3d corrected_R;
        corrected_R.col(0) = eigenvectors.col(sorted_axes[0].second);  // X (shortest)
        corrected_R.col(1) = eigenvectors.col(sorted_axes[1].second);  // Y (longest)
        corrected_R.col(2) = eigenvectors.col(sorted_axes[2].second);  // Z (always up)

        // Create final Oriented Bounding Box (OBB)
        auto obb = open3d::geometry::OrientedBoundingBox();
        obb.center_ = centroid;
        obb.extent_ = Eigen::Vector3d(sorted_axes[0].first, sorted_axes[1].first, sorted_axes[2].first);
        obb.R_ = corrected_R;

        bounding_boxes.push_back(obb);
    }
    return bounding_boxes;
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
std::vector<Rectangle> GeometryProcessor::createRandomRectangles(int n, double fixed_length) {
    std::vector<Rectangle> rectangles;
    double min_width = 0.08;    // 8 cm
    double max_width = 0.20;    // 20 cm
    double surface_width = 2.0; // Example surface width (adjust as needed)
    double surface_height = 2.0;// Example surface height (adjust as needed)
    double min_gap = 0.02;      // Minimum gap between rectangles (2 cm)

    std::srand(static_cast<unsigned int>(std::time(0))); // Seed random generator

    for (int i = 0; i < n; ++i) {
        bool valid_position = false;
        std::array<Eigen::Vector3d, 4> corners;

        for (int attempt = 0; attempt < 100; ++attempt) { // Try up to 100 times
            double width = min_width + (max_width - min_width) * (std::rand() / double(RAND_MAX));
            double x = min_gap + (surface_width - width - min_gap) * (std::rand() / double(RAND_MAX));
            double y = min_gap + (surface_height - fixed_length - min_gap) * (std::rand() / double(RAND_MAX));

            // Define the corners
            Eigen::Vector3d p1(x, y, 0);
            Eigen::Vector3d p2(x + width, y, 0);
            Eigen::Vector3d p3(x + width, y + fixed_length, 0);
            Eigen::Vector3d p4(x, y + fixed_length, 0);

            corners = {p1, p2, p3, p4};

            // Check for overlap with existing rectangles
            bool overlap = false;
            for (const auto& rect : rectangles) {
                auto existing_corners = rect.getSortedCorners();
                double ex_min_x = existing_corners[0].x();
                double ex_max_x = existing_corners[1].x();
                double ex_min_y = existing_corners[0].y();
                double ex_max_y = existing_corners[2].y();

                double new_min_x = p1.x();
                double new_max_x = p2.x();
                double new_min_y = p1.y();
                double new_max_y = p3.y();

                if (!(new_max_x + min_gap < ex_min_x || new_min_x > ex_max_x + min_gap ||
                      new_max_y + min_gap < ex_min_y || new_min_y > ex_max_y + min_gap)) {
                    overlap = true;
                    break;
                }
            }

            if (!overlap) {
                valid_position = true;
                break;
            }
        }

        if (valid_position) {
            rectangles.emplace_back(Rectangle(corners));
        }
    }

    return rectangles;
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


std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::arrangeFirstShingleRow(
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes,
    double gap,
    double max_length,
    double rotation_angle) {  // Rotation for the entire row

    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
    Eigen::Vector3d current_position(0, 0, 0);  // Start at the origin

    double previous_half_width = 0;  // Keeps track of half the previous box's width
    double total_length = 0;  // Keeps track of the total accumulated length

    Eigen::Vector3d last_box_right_edge(0, 0, 0);  // Stores the rightmost edge position
    bool min_length_reached = false;  // Flag to check if we reached at least max_length

    for (auto& bbox : bounding_boxes) {
        // Compute the bounding box's extent
        Eigen::Vector3d extent = bbox->extent_;


        // Identify the longest axis (we want the shorter edge to be aligned with X)
        int longest_axis = 0;  // Assume X is longest by default
        if (extent.y() > extent.x() && extent.y() > extent.z()) {
            longest_axis = 1;  // Y-axis is the longest
        } else if (extent.z() > extent.x() && extent.z() > extent.y()) {
            longest_axis = 2;  // Z-axis is the longest
        }

        // Rotate if the longest edge is not already perpendicular to X
        if (longest_axis == 1) {
            // If Y is longest, rotate 90 degrees around Z to align shorter edge with X
            transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1), M_PI_2, bbox->GetCenter(), true);
        } else if (longest_axis == 2) {
            // If Z is longest, rotate 90 degrees around Y to align shorter edge with X
            transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 0), M_PI_2, bbox->GetCenter(), true);
        }

        // Get the new extent after rotation
        extent = bbox->extent_;
        double current_half_width = extent.x() / 2.0;  // Half of the width after rotation

        // Predict next position before placing
        double new_total_length = total_length + previous_half_width + gap + current_half_width;

        // Allow placing shingles until we reach at least max_length
        if (min_length_reached && new_total_length > max_length) {
            break;  // Stop only after reaching at least max_length
        }

        // Correct placement: shift by previous half-width + gap + current half-width
        current_position.x() += previous_half_width + gap + current_half_width;

        // Translate the bounding box to align it along X-axis
        Eigen::Vector3d translation = current_position - bbox->GetCenter();
        transform_bounding_box(bbox, translation, Eigen::Vector3d(0, 1, 0), 0, Eigen::Vector3d(0, 0, 0), true);

        // Store the rightmost edge position (center + half width in X direction)
        last_box_right_edge = bbox->GetCenter() + Eigen::Vector3d(current_half_width, 0, 0);

        // Add the bounding box to the list
        arranged_bboxes.push_back(bbox);

        // Update tracking variables
        previous_half_width = current_half_width;
        total_length = new_total_length;

        // Mark that we have reached at least max_length
        if (total_length >= max_length) {
            min_length_reached = true;
        }
    }

    // **Convert degrees to radians** (rotation_angle is in degrees)
    double rotation_radians = rotation_angle * M_PI / 180.0;  // Convert to radians

    // Debug print: Rotation angle in both degrees and radians
    std::cout << "Applying rotation: " << rotation_angle << " degrees (" 
          << rotation_radians << " radians)" << std::endl;

    // **Apply the correct rotation to the entire row** (rotate each box around its center)
    Eigen::Vector3d rotation_center(0, 0, 0);  // Rotation center at the origin (can be adjusted if needed)

    // Apply a x-degree rotation around the Y-axis to make the longer edge align with the global X-axis
    for (auto& bbox : arranged_bboxes) {
        // Rotate the bounding box around its center
        transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), -rotation_radians, bbox->GetCenter(), true);

    }

    // Debug print: Total length of the row from (0,0,0) to rightmost edge
    std::cout << "Total row length: " << last_box_right_edge.x() << " meters" << std::endl;

for (auto& bbox : arranged_bboxes) {
    Eigen::Matrix3d rotation_matrix = bbox->R_;  // Get the rotation matrix

    // Extract rotation angles (Euler angles)
    double angle_x = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));  // Rotation around X
    double angle_y = std::atan2(-rotation_matrix(2, 0),
                                std::sqrt(rotation_matrix(2, 1) * rotation_matrix(2, 1) + 
                                          rotation_matrix(2, 2) * rotation_matrix(2, 2)));  // Rotation around Y
    double angle_z = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));  // Rotation around Z

    // Convert to degrees
    double angle_x_degrees = angle_x * 180.0 / M_PI;
    double angle_y_degrees = angle_y * 180.0 / M_PI;
    double angle_z_degrees = angle_z * 180.0 / M_PI;

    std::cout << "Bounding Box Rotation Angles:" << std::endl;
    std::cout << "  Around X: " << angle_x << " radians (" << angle_x_degrees << " degrees)" << std::endl;
    std::cout << "  Around Y: " << angle_y << " radians (" << angle_y_degrees << " degrees)" << std::endl;
    std::cout << "  Around Z: " << angle_z << " radians (" << angle_z_degrees << " degrees)" << std::endl;

    
}

    // debug
    Eigen::Vector3d first_row_top_face = arranged_bboxes[0]->GetCenter();
    first_row_top_face.z() += (arranged_bboxes[0]->extent_.z() / 2.0);
    std::cout << "[DEBUG] First row top face (final): " << first_row_top_face.transpose() << std::endl;

    return arranged_bboxes;
}

/////////////////////////////////////////////////////////////////////
//
// Eigen::Vector2d GeometryProcessor::projectToXYPlane(
//     const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bbox) {
//     // Get the position and rotation matrix of the bounding box
//     Eigen::Vector3d position = bbox->GetCenter();
//     Eigen::Matrix3d rotation_matrix = bbox->R_;  // Rotation matrix of the bounding box
    
//     // Project the center of the bounding box to the XY plane
//     Eigen::Vector3d projected_center = rotation_matrix * position;
//     return Eigen::Vector2d(projected_center.x(), projected_center.y());
// }



// std::shared_ptr<open3d::geometry::OrientedBoundingBox> GeometryProcessor::projectToXYPlane(
//     const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bbox) {

//     // Get the position and rotation matrix of the bounding box
//     Eigen::Vector3d position = bbox->GetCenter();
//     Eigen::Matrix3d rotation_matrix = bbox->R_;  // Rotation matrix of the bounding box

//     // Project the center of the bounding box to the XY plane by zeroing the Z-coordinate
//     Eigen::Vector3d projected_center = position;
//     projected_center[2] = 0;  // Set Z-coordinate to zero to project onto the XY plane

//     // Create a new oriented bounding box with the projected center but the same rotation
//     std::shared_ptr<open3d::geometry::OrientedBoundingBox> projected_bbox = 
//         std::make_shared<open3d::geometry::OrientedBoundingBox>(
//             projected_center, // New center (XY plane)
//             bbox->extent_,    // Same extent as the original
//             rotation_matrix   // Same rotation matrix
//         );

//     return projected_bbox;
// }



////////////////////////////
void GeometryProcessor::alignBoxToXYPlane(const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bbox) {
    // Extract the rotation matrix of the bounding box
    Eigen::Matrix3d rotation = bbox->R_;

    // Compute a matrix that aligns the box to the XY plane
    // Extract the 2x2 block and transpose it
    Eigen::Matrix2d rotation2D = rotation.block<2, 2>(0, 0).transpose();

    // Create a new 3x3 alignment matrix and set the top-left 2x2 block
    Eigen::Matrix3d alignmentMatrix = Eigen::Matrix3d::Identity();
    alignmentMatrix.block<2, 2>(0, 0) = rotation2D;

    // Apply the alignment matrix to the box's rotation matrix
    bbox->R_ = alignmentMatrix;
    
    // The center of the bounding box remains unchanged, no need to update
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




/////////////////////////////////////////////////////////////////////
// 
// double GeometryProcessor::calculateRightEdgeDistanceFromCandidate(
//     const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate_shingle,
//     const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& intersecting_shingle) {

//     // Right edge of the intersecting shingle (first row)
//     Eigen::Vector3d right_edge_intersecting = intersecting_shingle->GetCenter() + Eigen::Vector3d(intersecting_shingle->extent_.x() / 2.0, 0, 0);

//     // Right edge of the candidate shingle (second row)
//     Eigen::Vector3d right_edge_candidate = candidate_shingle->GetCenter() + Eigen::Vector3d(candidate_shingle->extent_.x() / 2.0, 0, 0);

//     // Horizontal distance between the right edge of the candidate and the intersecting box in the first row
//     double d1 = right_edge_intersecting.x() - right_edge_candidate.x();


//     return d1;
// }

/////////////////////////////////////////////////////////////////////
//
// bool GeometryProcessor::lineSegmentIntersection(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
//                             const Eigen::Vector3d& q1, const Eigen::Vector3d& q2) {
//     Eigen::Vector3d p = p2 - p1;
//     Eigen::Vector3d q = q2 - q1;
//     Eigen::Vector3d r = q1 - p1;

//     double crossProduct = p.cross(q).norm();
//     if (crossProduct == 0) return false; // Lines are parallel

//     // Use the cross-product to check for intersection (2D check, assuming it's in the XY plane)
//     double t = r.cross(q).norm() / crossProduct;
//     double u = r.cross(p).norm() / crossProduct;

//     return (t >= 0 && t <= 1) && (u >= 0 && u <= 1);
// }

// /////////////////////////////////////////////////////////////////////
// //
// std::shared_ptr<open3d::geometry::OrientedBoundingBox> GeometryProcessor::findIntersectingShingle(
//     const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate_bbox,
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row_bboxes) {
    
//     // Extract rectangles from bounding boxes in the first row and the candidate box
//     std::vector<Rectangle> first_row_rectangles = extractUpperRectangles(first_row_bboxes);
//     std::vector<Rectangle> candidate_rectangles = extractUpperRectangles({candidate_bbox});
    
//     // Get right edge of the candidate
//     auto candidate_right_edge = candidate_rectangles[0].getRightEdge();
    
//     // Iterate through the first row to find intersections
//     for (size_t i = 0; i < first_row_rectangles.size(); ++i) {
//         auto top_edges = first_row_rectangles[i].getTopEdges({first_row_rectangles[i]});
        
//         for (const auto& top_edge : top_edges) {
//             if (lineSegmentIntersection(candidate_right_edge.first, candidate_right_edge.second, top_edge.first, top_edge.second)) {
//                 // Return the intersecting bounding box from the first row
//                 return first_row_bboxes[i];
//             }
//         }
//     }
    
//     // If no intersection is found, return nullptr (no intersecting shingle)
//     return nullptr;
// }


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
void GeometryProcessor::alignCandidateToFirstBox(
    std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate,
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& first_box)
{
    if (!candidate || !first_box) {
        std::cerr << "[ERROR] Invalid bounding boxes provided!" << std::endl;
        return;
    }

    // Step 1: Get rotation matrices
    Eigen::Matrix3d first_box_rotation = first_box->R_;
    Eigen::Matrix3d candidate_rotation = candidate->R_;

    // Step 2: Compute the relative rotation needed
    Eigen::Matrix3d rotation_to_apply = first_box_rotation * candidate_rotation.inverse();

    // Step 3: Rotate the candidate
    candidate->R_ = rotation_to_apply * candidate_rotation;  // Apply the computed rotation

    // Step 4: Get the left and bottom edges of the first box
    Eigen::Vector3d first_box_center = first_box->GetCenter();
    Eigen::Vector3d first_box_extent = first_box->extent_;

    double left_edge_x = first_box_center.x() - first_box_extent.x() / 2.0;  // Left edge
    double bottom_edge_y = first_box_center.y() - first_box_extent.y() / 2.0; // Bottom edge

    // Step 5: Compute the new position of the candidate
    Eigen::Vector3d candidate_extent = candidate->extent_;
    Eigen::Vector3d candidate_center = candidate->GetCenter();

    double new_candidate_x = left_edge_x + candidate_extent.x() / 2.0;
    double new_candidate_y = bottom_edge_y + candidate_extent.y() / 2.0;

    Eigen::Vector3d translation(new_candidate_x - candidate_center.x(),
                                new_candidate_y - candidate_center.y(),
                                0.0);  // Assume translation only in XY plane

    // Step 6: Apply the translation
    candidate->Translate(translation);
}



/////////////////////////////////////////////////////////////////////

std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::findNextBestShingles(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates,
    double min_stagger,
    double max_gap,
    double max_length) 
{

    // Create a copy of the first row to avoid modifying the original
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> first_row_aligned;
    for (const auto& bbox : first_row) {
        auto bbox_copy = std::make_shared<open3d::geometry::OrientedBoundingBox>(*bbox);  // Create a copy
        alignBoxToXYPlane(bbox_copy);
        first_row_aligned.push_back(bbox_copy);
    }

    // Sort candidates based on their width
    std::sort(candidates.begin(), candidates.end(),
              [](const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& a,
                 const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& b) {
                  return a->extent_.x() < b->extent_.x();  // Sort by width
              });

    // // Initialize current right edge (starting at the origin)
    // Eigen::Vector3d current_right_edge(0, 0, 0);

    // Get the left edge of the first box in the first row as our current_right_edge
    Eigen::Vector3d current_right_edge = first_row_aligned[0]->GetCenter() - Eigen::Vector3d(first_row_aligned[0]->extent_.x() / 2.0, 0, 0);


    // Initialize second row of boxes
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> second_row;
    double total_width = 0.0;  // Track total width of the second row

    // Calculate initial distances from current right edge to the right edges of first row
    std::vector<double> distances;
    for (const auto& bbox : first_row_aligned) {
        double distance = (bbox->GetCenter() - current_right_edge).x() + bbox->extent_.x() / 2.0;
        distances.push_back(distance);
    }

    // Iterate through candidates to find valid shingles
    for (const auto& candidate : candidates) {
        bool candidate_added = false;

        // Iterate through distances to find the best fit
        for (size_t i = 0; i < distances.size(); ++i) {
            double distance = distances[i];

            // Check if candidate width is less than distance and meets min_stagger condition
            if (candidate->extent_.x() < (distance - min_stagger)) {
                // Valid candidate found, add to second row
                second_row.push_back(candidate);
                total_width += candidate->extent_.x();
                current_right_edge = updateRightEdge(current_right_edge, candidate);  // Update the right edge
                candidate_added = true;
                break;  // Exit loop after adding the candidate
            }

            // Check if candidate width is in range between two distances (with min_stagger margin)
            if (i + 1 < distances.size()) {
                double next_distance = distances[i + 1];
                if (candidate->extent_.x() > (distance + min_stagger) && candidate->extent_.x() < (next_distance - min_stagger)) {
                    // Valid candidate found, add to second row
                    second_row.push_back(candidate);
                    total_width += candidate->extent_.x();
                    current_right_edge = updateRightEdge(current_right_edge, candidate);  // Update the right edge
                    candidate_added = true;
                    break;  // Exit loop after adding the candidate
                }
            }
        }

        // Stop if no valid candidate is added in this iteration
        if (!candidate_added) {
            std::cout << "[INFO] No valid candidate for current position, moving to next." << std::endl;
        }

        // If total width exceeds max_length, stop
        if (total_width > max_length) {
            std::cout << "[INFO] Total width exceeds max length (" << total_width << " > " << max_length << "). Stopping." << std::endl;
            break;
        }
    }

    // Return the second row of shingles
    return second_row;
}


///////////////////////////////////////////
Eigen::Vector3d GeometryProcessor::updateRightEdge(
    const Eigen::Vector3d& current_right_edge,
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate) {
    // Update the right edge based on the candidate's position and width
    Eigen::Vector3d new_right_edge = current_right_edge;
    new_right_edge.x() += candidate->extent_.x();  // Move the right edge by the candidate's width
    return new_right_edge;
}
///////////////////


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
// GeometryProcessor::arrangeSecondShingleRow(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
//     double gap,              // e.g., 0.003 for 3 mm gap
//     double max_length,
//     double rotation_angle)    // in degrees
// {
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
    
//     // --- Determine horizontal starting position from first row ---
//     double start_x = first_row[0]->GetCenter().x() - (first_row[0]->extent_.x() / 2.0);
    
//     // --- Determine Y alignment (top edge of first row) ---
//     double first_row_top_y = first_row[0]->GetCenter().y() + (first_row[0]->extent_.y() / 2.0);
    
//     // --- Determine Z position for second row ---

//     double desired_z = first_row[0]->GetCenter().z() + first_row[0]->extent_.z();
//     std::cout << "[DEBUG] For Z alignment, using desired Z = " << desired_z << std::endl;
    
//     double last_right_edge = start_x;
    
//     for (size_t i = 0; i < second_row.size(); ++i) {
//         auto& bbox = second_row[i];
        
//         // Optionally, rotate the box based on its longest axis.
//         Eigen::Vector3d extent = bbox->extent_;
//         int longest_axis = (extent.y() > extent.x() && extent.y() > extent.z()) ? 1 :
//                            (extent.z() > extent.x() && extent.z() > extent.y()) ? 2 : 0;
//         if (longest_axis == 1) {
//             transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                    Eigen::Vector3d(0,0,1), M_PI_2, bbox->GetCenter(), true);
//         } else if (longest_axis == 2) {
//             transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                    Eigen::Vector3d(0,1,0), M_PI_2, bbox->GetCenter(), true);
//         }
//         extent = bbox->extent_;
//         double half_width = extent.x() / 2.0;
//         double half_thickness = extent.z() / 2.0;
        
//         // --- Determine desired center in X ---
//         double desired_center_x = (i == 0) ? (start_x + half_width)
//                                            : (last_right_edge + gap + half_width);
        
//         // --- Determine desired center in Y ---
//         // Align so that the top edge of the second row equals the top edge of the first row.
//         // For a second row box, top edge = center.y + (extent.y()/2)
//         // Set desired_center_y such that: desired_center_y + (extent.y()/2) = first_row_top_y.
//         //double desired_center_y = first_row_top_y - (extent.y() / 2.0);
//         double desired_center_y = first_row_top_y + (extent.y() / 2.0) - first_row[0]->extent_.y();



//         // --- Determine desired center in Z ---
//         // We want the second row’s bottom face (center.z - half_thickness)
//         // to align with the first row's top face.
//         // Using our forced approach, we set:
//         double desired_center_z = desired_z + half_thickness;
        
//         std::cout << "[DEBUG] Box " << i << " computed desired center: (" 
//                   << desired_center_x << ", " << desired_center_y << ", " << desired_center_z << ")" << std::endl;
        
//         Eigen::Vector3d desired_center(desired_center_x, desired_center_y, desired_center_z);
//         Eigen::Vector3d translation = desired_center - bbox->GetCenter();
        
//         std::cout << "[DEBUG] Box " << i << " translation: " << translation.transpose() << std::endl;
        
//         transform_bounding_box(bbox, translation, Eigen::Vector3d(0,1,0), 
//                                 0, Eigen::Vector3d(0,0,0), true);
        
//         std::cout << "[DEBUG] Box " << i << " new center: " << bbox->GetCenter().transpose() << std::endl;
        
//         last_right_edge = desired_center_x + half_width;
//         arranged_bboxes.push_back(bbox);
        
//         if (last_right_edge - start_x > max_length)
//             break;
//     }
    
//     // --- Optionally, rotate the entire row by rotation_angle (around each box's center) ---
//     double rotation_radians = rotation_angle * M_PI / 180.0;
//     for (auto& bbox : arranged_bboxes) {
//         transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                Eigen::Vector3d(1,0,0), -rotation_radians, bbox->GetCenter(), true);
//     }
    
//     std::cout << "[DEBUG] Total row length: " << last_right_edge - start_x << " meters" << std::endl;
//     for (size_t i = 0; i < arranged_bboxes.size(); ++i) {
//     Eigen::Vector3d center = arranged_bboxes[i]->GetCenter();
//     double half_thickness = arranged_bboxes[i]->extent_.z() / 2.0;
//     Eigen::Vector3d bottom_face = center;
//     bottom_face.z() -= half_thickness;
//     std::cout << "[DEBUG] Second row box " << i << " bottom face: " << bottom_face.transpose() << std::endl;
//     }

//     return arranged_bboxes;
// }
//////////////////////////////////////////////////////////////



// // // this one better
// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
// GeometryProcessor::arrangeSecondShingleRow(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
//     double gap,              // e.g., 0.003 for 3 mm gap
//     double max_length,
//     double rotation_angle)    // in degrees
// {
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
    
//     // --- Determine the horizontal starting position from the first row ---
//     double start_x = first_row[0]->GetCenter().x() - (first_row[0]->extent_.x() / 2.0);
    
//     // --- Calculate the top edge position of the first row in the Z-axis ---
//     double first_row_top_z = first_row[0]->GetCenter().z() + first_row[0]->extent_.z() / 2.0;

//     double first_row_top_y = first_row[0]->GetCenter().y();  // Y is consistent for both rows
//     //double first_row_top_y = first_row[0]->GetCenter().y() + (first_row[0]->extent_.y() / 2.0);
//     std::cout << "[DEBUG] First row top edge Y position: " << first_row_top_y << std::endl;

    
    
//     // --- Position the second row directly above the first row ---
//     double last_right_edge = start_x;
    
//     for (size_t i = 0; i < second_row.size(); ++i) {
//         auto& bbox = second_row[i];
        
//         // Optionally, rotate the box based on its longest axis.
//         Eigen::Vector3d extent = bbox->extent_;
//         int longest_axis = (extent.y() > extent.x() && extent.y() > extent.z()) ? 1 :
//                            (extent.z() > extent.x() && extent.z() > extent.y()) ? 2 : 0;
//         if (longest_axis == 1) {
//             transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                    Eigen::Vector3d(0,0,1), M_PI_2, bbox->GetCenter(), true);
//         } else if (longest_axis == 2) {
//             transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                    Eigen::Vector3d(0,1,0), M_PI_2, bbox->GetCenter(), true);
//         }
        
//         extent = bbox->extent_;
//         double half_width = extent.x() / 2.0;
//         double half_thickness = extent.z() / 2.0;
        
//         // --- Determine desired center in X ---
//         double desired_center_x = (i == 0) ? (start_x + half_width)
//                                            : (last_right_edge + gap + half_width);
        
//         // --- Determine desired center in Y (same as first row) ---
//         double desired_center_y = first_row_top_y;
        
//         // --- Determine desired center in Z (Align bottom of second row to top of first row) ---
//         double desired_center_z = first_row_top_z + half_thickness;
        
//         std::cout << "[DEBUG] Box " << i << " computed desired center: (" 
//                   << desired_center_x << ", " << desired_center_y << ", " << desired_center_z << ")" << std::endl;
        
//         Eigen::Vector3d desired_center(desired_center_x, desired_center_y, desired_center_z);
//         Eigen::Vector3d translation = desired_center - bbox->GetCenter();
        
//         std::cout << "[DEBUG] Box " << i << " translation: " << translation.transpose() << std::endl;
        
//         transform_bounding_box(bbox, translation, Eigen::Vector3d(0,1,0), 
//                                 0, Eigen::Vector3d(0,0,0), true);
        
//         std::cout << "[DEBUG] Box " << i << " new center: " << bbox->GetCenter().transpose() << std::endl;
        
//         last_right_edge = desired_center_x + half_width;
//         arranged_bboxes.push_back(bbox);
        
//         if (last_right_edge - start_x > max_length)
//             break;
//     }
    
//     // --- Optionally, rotate the entire row by rotation_angle (around each box's center) ---
//     double rotation_radians = rotation_angle * M_PI / 180.0;
//     for (auto& bbox : arranged_bboxes) {
//         transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                Eigen::Vector3d(1,0,0), -rotation_radians, bbox->GetCenter(), true);
//     }
    
//     std::cout << "[DEBUG] Total row length: " << last_right_edge - start_x << " meters" << std::endl;
//     for (size_t i = 0; i < arranged_bboxes.size(); ++i) {
//         Eigen::Vector3d center = arranged_bboxes[i]->GetCenter();
//         double half_thickness = arranged_bboxes[i]->extent_.z() / 2.0;
//         Eigen::Vector3d bottom_face = center;
//         bottom_face.z() -= half_thickness;
//         std::cout << "[DEBUG] Second row box " << i << " bottom face: " << bottom_face.transpose() << std::endl;
//     }

//     return arranged_bboxes;
// }

// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
// GeometryProcessor::arrangeSecondShingleRow(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
//     double gap,              // e.g., 0.003 for 3 mm gap
//     double max_length,
//     double rotation_angle)    // in degrees
// {
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
    
//     // --- Determine the horizontal starting position from the first row ---
//     double start_x = first_row[0]->GetCenter().x() - (first_row[0]->extent_.x() / 2.0);
    
//     // --- Calculate the top edge position of the first row in the Z-axis ---
//     double first_row_top_z = first_row[0]->GetCenter().z() + first_row[0]->extent_.z() / 2.0;

//     double first_row_top_y = first_row[0]->GetCenter().y() ; 


//     // --- Print plane of first box of first row (bottom-left corner) ---
//     Eigen::Vector3d first_box_first_row_center = first_row[0]->GetCenter();
//     Eigen::Vector3d first_box_first_row_half_extent = first_row[0]->extent_ / 2.0;
//     Eigen::Vector3d first_box_first_row_bottom_left = first_box_first_row_center - 
//                                                       Eigen::Vector3d(first_box_first_row_half_extent.x(), 
//                                                                       first_box_first_row_half_extent.y(), 
//                                                                       -first_box_first_row_half_extent.z());
//     Eigen::Vector3d normal_first_row = Eigen::Vector3d(0, 0, -1);  // Assuming the normal is along the Z-axis
//     Eigen::Vector3d point_first_row = first_box_first_row_bottom_left;
//     std::cout << "[DEBUG] First row first box bottom-left corner: " << first_box_first_row_bottom_left.transpose() << std::endl;
//     std::cout << "[DEBUG] First row plane equation: " << normal_first_row.transpose() << " . (P - " << point_first_row.transpose() << ") = 0" << std::endl;

//     // --- Position the second row directly above the first row ---
//     double last_right_edge = start_x;
    
//     for (size_t i = 0; i < second_row.size(); ++i) {
//         auto& bbox = second_row[i];
        
//         // Optionally, rotate the box based on its longest axis.
//         Eigen::Vector3d extent = bbox->extent_;
//         int longest_axis = (extent.y() > extent.x() && extent.y() > extent.z()) ? 1 :
//                            (extent.z() > extent.x() && extent.z() > extent.y()) ? 2 : 0;
        
//         // Apply rotation if needed
//         if (longest_axis == 1) {
//             transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                    Eigen::Vector3d(0,0,1), M_PI_2, bbox->GetCenter(), true);
//         } else if (longest_axis == 2) {
//             transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                    Eigen::Vector3d(0,1,0), M_PI_2, bbox->GetCenter(), true);
//         }
        
//         extent = bbox->extent_;
//         double half_width = extent.x() / 2.0;
//         double half_thickness = extent.z() / 2.0;
        
//         // --- Determine desired center in X ---
//         double desired_center_x = (i == 0) ? (start_x + half_width)
//                                            : (last_right_edge + gap + half_width);
        
//         // --- Ensure Y is aligned with the first row ---
//         double desired_center_y = first_row_top_y;
        
//         // --- Determine desired center in Z (Align bottom of second row to top of first row) ---
//         double desired_center_z = first_row_top_z + half_thickness;
        
//         Eigen::Vector3d desired_center(desired_center_x, desired_center_y, desired_center_z);
//         Eigen::Vector3d translation = desired_center - bbox->GetCenter();
        
//         transform_bounding_box(bbox, translation, Eigen::Vector3d(0,1,0), 
//                                0, Eigen::Vector3d(0,0,0), true);
        
//         // --- Rotate the bounding box by the specified rotation angle ---
//         double rotation_radians = rotation_angle * M_PI / 180.0;
//         transform_bounding_box(bbox, Eigen::Vector3d(0,0,0),
//                                Eigen::Vector3d(1,0,0), -rotation_radians, bbox->GetCenter(), true);
        
//         // --- Print plane of first box of second row (bottom-left corner) ---
//         Eigen::Vector3d second_box_second_row_center = bbox->GetCenter();
//         Eigen::Vector3d second_box_second_row_half_extent = bbox->extent_ / 2.0;
//         Eigen::Vector3d second_box_second_row_bottom_left = second_box_second_row_center - 
//                                                              Eigen::Vector3d(second_box_second_row_half_extent.x(), 
//                                                                              second_box_second_row_half_extent.y(), 
//                                                                              -second_box_second_row_half_extent.z());
//         Eigen::Vector3d normal_second_row = Eigen::Vector3d(0, 0, -1);  // Assuming the normal is along the Z-axis
//         Eigen::Vector3d point_second_row = second_box_second_row_bottom_left;
//         std::cout << "[DEBUG] Second row first box bottom-left corner: " << second_box_second_row_bottom_left.transpose() << std::endl;
//         std::cout << "[DEBUG] Second row plane equation: " << normal_second_row.transpose() << " . (P - " << point_second_row.transpose() << ") = 0" << std::endl;
        
//         last_right_edge = desired_center_x + half_width;
//         arranged_bboxes.push_back(bbox);
        
//         if (last_right_edge - start_x > max_length)
//             break;
//     }
    
//     return arranged_bboxes;
// }


// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
// GeometryProcessor::arrangeSecondShingleRow(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
//     double gap,              // e.g., 0.003 for 3 mm gap
//     double max_length,
//     double rotation_angle) {    // in degrees

//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
//     Eigen::Vector3d current_position(0, 0, 0);  // Start at the origin
//     double total_length = 0;  // Keeps track of the total accumulated length
//     bool min_length_reached = false;  // Flag to check if we reached at least max_length

//     // Get the first box in the first row
//     auto first_box = first_row[0];

//     // Compute the 3D plane for the first box
//     Eigen::Vector3d first_box_center = first_box->GetCenter();
//     Eigen::Vector3d first_box_normal = first_box->R_ * Eigen::Vector3d(0, 0, 1); // Assuming Z is the normal vector of the bounding box
//     Eigen::Vector3d first_box_bottom_left = first_box_center - Eigen::Vector3d(first_box->extent_.x() / 2.0, first_box->extent_.y() / 2.0, first_box->extent_.z() / 2.0);

//     // Get the first box in the second row
//     auto second_row_first_box = second_row[0];

//     // Compute the 3D plane for the first box in the second row
//     Eigen::Vector3d second_row_first_box_center = second_row_first_box->GetCenter();
//     Eigen::Vector3d second_row_first_box_normal = second_row_first_box->R_ * Eigen::Vector3d(0, 0, 1); // Z is the normal vector of the bounding box
//     Eigen::Vector3d second_row_first_box_bottom_left = second_row_first_box_center - Eigen::Vector3d(second_row_first_box->extent_.x() / 2.0, second_row_first_box->extent_.y() / 2.0, second_row_first_box->extent_.z() / 2.0);

//     // Transform the first box in the second row to the first box in the first row's plane
//     Eigen::Vector3d translation = first_box_bottom_left - second_row_first_box_bottom_left;
//     transform_bounding_box(second_row_first_box, translation, Eigen::Vector3d(1, 0, 0), 0, Eigen::Vector3d(0, 0, 0), true);

//     // Start placing the other boxes in the second row
//     Eigen::Vector3d previous_half_width = Eigen::Vector3d(second_row_first_box->extent_.x() / 2.0, 0, 0);  // Half of the width of the first box in second row
//     for (size_t i = 1; i < second_row.size(); ++i) {
//         auto& bbox = second_row[i];
        
//         // Compute extent of the bounding box
//         Eigen::Vector3d extent = bbox->extent_;

//         // Predict next position before placing
//         double new_total_length = total_length + previous_half_width.x() + gap + extent.x() / 2.0;

//         if (min_length_reached && new_total_length > max_length) {
//             break;  // Stop once we reach max_length
//         }

//         // Correct placement: shift by previous half-width + gap + current half-width
//         current_position.x() += previous_half_width.x() + gap + extent.x() / 2.0;

//         // Translate the bounding box to align it along X-axis (using first row's plane)
//         Eigen::Vector3d translation = current_position - bbox->GetCenter();
//         transform_bounding_box(bbox, translation, Eigen::Vector3d(0, 1, 0), 0, Eigen::Vector3d(0, 0, 0), true);

//         // Add the bounding box to the list
//         arranged_bboxes.push_back(bbox);

//         // Update tracking variables
//         previous_half_width = extent / 2.0;
//         total_length = new_total_length;

//         if (total_length >= max_length) {
//             min_length_reached = true;
//         }
//     }

//     // **Convert degrees to radians** (rotation_angle is in degrees)
//     double rotation_radians = rotation_angle * M_PI / 180.0;  // Convert to radians

//     // **Apply rotation to the entire row** (rotate each box around its center)
//     for (auto& bbox : arranged_bboxes) {
//         // Rotate the bounding box around its center
//         transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), -rotation_radians, bbox->GetCenter(), true);
//     }

//     return arranged_bboxes;
// }



std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
GeometryProcessor::arrangeSecondShingleRow(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
    double gap,              
    double max_length,
    double rotation_angle) {    

    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
    Eigen::Vector3d current_position(0, 0, 0);  
    double total_length = 0;  
    bool min_length_reached = false;  

    // Get the first box in the first row
    auto first_box = first_row[0];
    Eigen::Vector3d first_box_bottom_left = first_box->GetCenter() - 
        Eigen::Vector3d(first_box->extent_.x() / 2.0, first_box->extent_.y() / 2.0, first_box->extent_.z() / 2.0);

    // Process each bounding box in the second row
    for (size_t i = 0; i < second_row.size(); ++i) {
        auto& bbox = second_row[i];

        // Determine the longest axis
        Eigen::Vector3d extent = bbox->extent_;
        int longest_axis = (extent.x() >= extent.y() && extent.x() >= extent.z()) ? 0 :
                           (extent.y() >= extent.x() && extent.y() >= extent.z()) ? 1 : 2;

        // Apply rotation to align the longest side along X
        if (longest_axis == 1) {
            transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), 
                                   Eigen::Vector3d(0, 0, 1), M_PI_2, bbox->GetCenter(), true);
        } else if (longest_axis == 2) {
            transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), 
                                   Eigen::Vector3d(0, 1, 0), M_PI_2, bbox->GetCenter(), true);
        }

        // Compute the new bottom-left after alignment
        Eigen::Vector3d bbox_bottom_left = bbox->GetCenter() - 
            Eigen::Vector3d(bbox->extent_.x() / 2.0, bbox->extent_.y() / 2.0, bbox->extent_.z() / 2.0);

        // First box in second row should be aligned with first row's plane
        if (i == 0) {
            Eigen::Vector3d translation = first_box_bottom_left - bbox_bottom_left;
            transform_bounding_box(bbox, translation, Eigen::Vector3d(0, 1, 0), 0, Eigen::Vector3d(0, 0, 0), true);
            current_position = bbox->GetCenter();
        } else {
            // Predict new position
            double new_total_length = total_length + bbox->extent_.x() / 2.0 + gap + bbox->extent_.x() / 2.0;
            if (min_length_reached && new_total_length > max_length) {
                break;
            }

            // Move the bounding box to the correct position
            current_position.x() += bbox->extent_.x() / 2.0 + gap + bbox->extent_.x() / 2.0;
            Eigen::Vector3d translation = current_position - bbox->GetCenter();
            transform_bounding_box(bbox, translation, Eigen::Vector3d(0, 1, 0), 0, Eigen::Vector3d(0, 0, 0), true);

            total_length = new_total_length;
            if (total_length >= max_length) {
                min_length_reached = true;
            }
        }

        arranged_bboxes.push_back(bbox);
    }

    // Apply rotation to the entire row
    double rotation_radians = rotation_angle * M_PI / 180.0;
    for (auto& bbox : arranged_bboxes) {
        transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), 
                               Eigen::Vector3d(1, 0, 0), -rotation_radians, bbox->GetCenter(), true);
    }

    return arranged_bboxes;
}




// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
// GeometryProcessor::orientSecondRowFirstBox(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row)
// {
//     // Operate only on the first boxes of each list.
//     auto first_box = first_row[0];
//     auto second_box = second_row[0];

//     // Compute the global bottom-left corner for the first box of the first row.
//     // (Without a helper function: do it inline.)
//     Eigen::Vector3d first_center = first_box->GetCenter();
//     Eigen::Vector3d first_extent = first_box->extent_;
//     Eigen::Matrix3d first_R = first_box->R_;
//     // Local bottom-left: (-extent.x/2, -extent.y/2, -extent.z/2)
//     Eigen::Vector3d first_local_bl(-first_extent.x()/2.0, -first_extent.y()/2.0, -first_extent.z()/2.0);
//     Eigen::Vector3d first_global_bl = first_center + first_R * first_local_bl;

//     // Compute the global bottom-left corner for the first box of the second row.
//     Eigen::Vector3d second_center = second_box->GetCenter();
//     Eigen::Vector3d second_extent = second_box->extent_;
//     Eigen::Matrix3d second_R = second_box->R_;
//     Eigen::Vector3d second_local_bl(-second_extent.x()/2.0, -second_extent.y()/2.0, -second_extent.z()/2.0);
//     Eigen::Vector3d second_global_bl = second_center + second_R * second_local_bl;

//     // Compute the translation needed so that the second box's bottom-left becomes equal to the first box's bottom-left.
//     Eigen::Vector3d translation = first_global_bl - second_global_bl;
//     // Apply the translation.
//     second_box->Translate(translation);

//     // Now, after translation, recompute the second box’s bottom-left (it should now equal first_global_bl if no rotation were applied).
//     second_center = second_box->GetCenter();  // Updated center.
//     // (We don’t recompute second_R because Translate doesn't change orientation.)

//     // To align the orientation (i.e. the normal vector) we want the second box to have the same rotation as the first box.
//     // Compute the rotation needed:
//     // Since R is orthonormal, (R_second)⁻¹ = (R_second)ᵀ.
//     // Thus, the rotation to apply is:
//     Eigen::Matrix3d R_transform = first_R * second_R.transpose();
//     // We want to apply this rotation about the bottom-left corner (our pivot).
//     second_box->Rotate(R_transform, first_global_bl);

//     // Return a vector containing the reoriented first box of the second row.
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> result;
//     result.push_back(second_box);
//     return result;
// }


// std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
// GeometryProcessor::orientSecondRowFirstBox(
//     const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row)
// {
//     // Operate on the first box of each list.
//     auto first_box = first_row[0];
//     auto second_box = second_row[0];

//     // --- For the first row box, compute its global “bottom-left” corner.
//     // Here we want the opposite diagonal from what you were getting.
//     // Instead of: local_offset = (-extent.x/2, -extent.y/2, -extent.z/2),
//     // we try: local_offset = (+extent.x/2, +extent.y/2, -extent.z/2).
//     Eigen::Vector3d f_center = first_box->GetCenter();
//     Eigen::Vector3d f_extent = first_box->extent_;
//     Eigen::Matrix3d f_R = first_box->R_;
//     Eigen::Vector3d f_local_corner( f_extent.x()/2.0, f_extent.y()/2.0, -f_extent.z()/2.0 );
//     Eigen::Vector3d f_global_corner = f_center + f_R * f_local_corner;

//     // --- For the second row box, compute its global corner using the same local offset.
//     Eigen::Vector3d s_center = second_box->GetCenter();
//     Eigen::Vector3d s_extent = second_box->extent_;
//     Eigen::Matrix3d s_R = second_box->R_;
//     Eigen::Vector3d s_local_corner( s_extent.x()/2.0, s_extent.y()/2.0, -s_extent.z()/2.0 );
//     Eigen::Vector3d s_global_corner = s_center + s_R * s_local_corner;

//     // --- Compute the translation needed so that the second box's chosen corner
//     // matches the first box's chosen corner.
//     Eigen::Vector3d trans = f_global_corner - s_global_corner;
//     second_box->Translate(trans);

//     // --- Now, align the orientation (normal vector) of the second box to match the first.
//     // We compute the rotation needed as:
//     //    R_transform = first_box_R * (second_box_R)^T
//     // and then rotate the second box about the pivot at the (now aligned) corner.
//     s_center = second_box->GetCenter(); // update after translation
//     // (s_R remains unchanged by translation)
//     Eigen::Matrix3d R_transform = f_R * s_R.transpose();
//     // Rotate about the aligned corner (which should now be at f_global_corner)
//     second_box->Rotate(R_transform, f_global_corner);

//     // Return a vector containing the transformed second row first box.
//     std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> result;
//     result.push_back(second_box);
//     return result;
// }

std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>
GeometryProcessor::alignAndShiftSecondRowFirstBox(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
    double gap,              // gap (if used for later placement)
    double max_length,       // (if used for later placement)
    double rotation_angle) { // (if used for later adjustment)

    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;

    // --- Step 1. Compute the chosen corner for the first row’s first box.
    // We want the corner on the opposite diagonal from what we got before.
    // Here we define the local offset as: (+extent.x/2, +extent.y/2, -extent.z/2)
    auto first_box = first_row[0];
    Eigen::Vector3d f_center = first_box->GetCenter();
    Eigen::Vector3d f_extent = first_box->extent_;
    Eigen::Matrix3d f_R = first_box->R_;
    Eigen::Vector3d f_local_offset(f_extent.x()/2.0, f_extent.y()/2.0, -f_extent.z()/2.0);
    Eigen::Vector3d f_global_corner = f_center + f_R * f_local_offset;

    // --- Step 2. Compute the same chosen corner for the first box of the second row.
    auto second_box = second_row[0];
    Eigen::Vector3d s_center = second_box->GetCenter();
    Eigen::Vector3d s_extent = second_box->extent_;
    Eigen::Matrix3d s_R = second_box->R_;
    Eigen::Vector3d s_local_offset(s_extent.x()/2.0, s_extent.y()/2.0, -s_extent.z()/2.0);
    Eigen::Vector3d s_global_corner = s_center + s_R * s_local_offset;

    // --- Step 3. Translate the second row’s first box so its chosen corner matches the first row’s.
    Eigen::Vector3d trans = f_global_corner - s_global_corner;
    second_box->Translate(trans);

    // --- Step 4. Rotate the second row’s first box so that its orientation matches the first row’s.
    // We compute the rotation transformation as:
    //    R_transform = f_R * (s_R)^T
    Eigen::Matrix3d R_transform = f_R * s_R.transpose();
    // Rotate around the pivot (the aligned corner).
    second_box->Rotate(R_transform, f_global_corner);

    // At this point, the first box of the second row has the same global chosen corner as the first row’s first box.
    arranged_bboxes.push_back(second_box);

    // --- Step 5. Shift the now aligned second row first box along the first row’s normal
    // by an amount equal to the thickness of the first row box.
    // Here, we assume the "thickness" is given by the extent along the local z axis.
    double thickness = f_extent.z();
    // The first row’s normal is: first_R * (0,0,1)
    Eigen::Vector3d first_normal = f_R * Eigen::Vector3d(0,0,1);
    // The shift vector:
    Eigen::Vector3d shift = first_normal * thickness;
    second_box->Translate(shift);

    // (If you later want to arrange additional boxes in the second row, you would
    // compute their positions relative to this shifted box.)

    return arranged_bboxes;
}



///////////////////////////////////////////////////////
void GeometryProcessor::visualize_bounding_boxes(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes) {
    // Create a visualizer object
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Bounding Boxes Visualization");

    // Create global coordinate axes using lines
    auto axis_lines = std::make_shared<open3d::geometry::LineSet>();
    axis_lines->points_ = {
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),  // X-axis (Red)
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 1, 0),  // Y-axis (Green)
        Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)   // Z-axis (Blue)
    };
    axis_lines->lines_ = {
        Eigen::Vector2i(0, 1),  // X-axis
        Eigen::Vector2i(2, 3),  // Y-axis
        Eigen::Vector2i(4, 5)   // Z-axis
    };
    axis_lines->colors_ = {
        Eigen::Vector3d(1, 0, 0),  // Red for X-axis
        Eigen::Vector3d(0, 1, 0),  // Green for Y-axis
        Eigen::Vector3d(0, 0, 1)   // Blue for Z-axis
    };

    // Add the global axes to the visualizer
    visualizer.AddGeometry(axis_lines);

    // Visualize bounding boxes
    for (const auto& bbox : bounding_boxes) {
        visualizer.AddGeometry(bbox);
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

    // Create an interactive visualizer
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Shingles Visualization", 1600, 900);
    for (auto& geom : all_geometries) {
        visualizer.AddGeometry(geom);
    }
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
}

/////////////////////////////////////////////
namespace fs = std::filesystem;

// Helper: Convert an oriented bounding box to a triangle mesh
std::shared_ptr<open3d::geometry::TriangleMesh> GeometryProcessor::CreateMeshFromOrientedBoundingBox(
    const open3d::geometry::OrientedBoundingBox& obb) {
    // Get the 8 corner points of the bounding box.
    std::vector<Eigen::Vector3d> points = obb.GetBoxPoints();
    
    // Define triangles for the 6 faces.
    // The assumed order of points is:
    // 0: (-1,-1,-1), 1: (1,-1,-1), 2: (1,1,-1), 3: (-1,1,-1),
    // 4: (-1,-1,1),  5: (1,-1,1),  6: (1,1,1),  7: (-1,1,1)
    // after transformation by the OBB.
    std::vector<Eigen::Vector3i> triangles = {
        {0, 1, 2}, {0, 2, 3}, // bottom face
        {4, 5, 6}, {4, 6, 7}, // top face
        {0, 1, 5}, {0, 5, 4}, // front face
        {2, 3, 7}, {2, 7, 6}, // back face
        {1, 2, 6}, {1, 6, 5}, // right face
        {3, 0, 4}, {3, 4, 7}  // left face
    };

    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    mesh->vertices_ = points;
    mesh->triangles_ = triangles;
    mesh->ComputeVertexNormals();
    return mesh;
}


//////////////////////////////////////////////////////////////////////////////
// Export function: Exports each bounding box as a PLY file using triangle mesh geometry.
void GeometryProcessor::exportBoundingBoxes(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& boxes,
    const std::string& folder,
    const std::string& prefix )
{
    int index = 0;
    for (const auto& box : boxes) {
        auto mesh = CreateMeshFromOrientedBoundingBox(*box);
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



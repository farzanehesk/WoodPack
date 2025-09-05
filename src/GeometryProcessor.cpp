#include "../include/GeometryProcessor.hpp"
#include <open3d/Open3D.h>
#include <iostream>
#include <Eigen/Eigenvalues> // For SelfAdjointEigenSolver
#include <cmath>
#include <Eigen/Dense>
#include <random>
#include <chrono>
#include <iomanip>
#include <open3d/geometry/BoundingVolume.h>
#include <Eigen/Core>
#include <vector>
#include <memory>
#include <limits>
#include <string>
#include <fstream>
#include <open3d/visualization/visualizer/Visualizer.h> 



 GeometryProcessor::GeometryProcessor() {} 
 GeometryProcessor::~GeometryProcessor() {} 



///////////////////////////////////////////////////////
//

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

    // Validate input
    if (bounding_boxes.empty()) {
        std::cerr << "No bounding boxes to visualize." << '\n';
        return;
    }

    std::vector<std::shared_ptr<open3d::geometry::Geometry>> geometries;

    // Log bounding box details
    std::cout << "Visualizing " << bounding_boxes.size() << " bounding boxes." << '\n';
    for (size_t i = 0; i < bounding_boxes.size(); ++i) {
        const auto& box = bounding_boxes[i];
        std::cout << "Box " << i << ": center = " << box.center_.transpose()
                  << ", extent = " << box.extent_.transpose() << '\n';
    }

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

    // Visualize and save screenshot
    {
        std::shared_ptr<open3d::visualization::Visualizer> vis =
            std::make_shared<open3d::visualization::Visualizer>();
        vis->CreateVisualizerWindow("Bounding Boxes with Axes", 1920, 1080, 50, 50);

        // Add all geometries
        for (const auto& geom : geometries) {
            vis->AddGeometry(geom, true);
        }

        // Set options
        vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // White background
        vis->GetRenderOption().point_size_ = 2.0; // Point size (for points)
        vis->GetRenderOption().line_width_ = 5.0; // Thicker lines for visibility

        // Compute centroid and max extent of bounding box centers
        Eigen::Vector3d centroid(0.0, 0.0, 0.0);
        double max_extent = 0.0;
        for (const auto& box : bounding_boxes) {
            centroid += box.center_;
            max_extent = std::max(max_extent, box.extent_.maxCoeff());
        }
        if (!bounding_boxes.empty()) {
            centroid /= bounding_boxes.size();
        }

        // Set view control (top-down orthographic)
        auto& view_control = vis->GetViewControl();
        view_control.SetFront(Eigen::Vector3d(0.0, 0.0, -1.0)); // Look down Z-axis
        view_control.SetUp(Eigen::Vector3d(0.0, 1.0, 0.0)); // Y-axis is up
        view_control.SetLookat(centroid); // Center of bounding boxes
        // Dynamic zoom based on extent
        double zoom = 0.5 / (1.0 + max_extent);
        view_control.SetZoom(std::min(zoom, 1.0)); 

        // Ensure rendering
        vis->PollEvents();
        vis->UpdateRender();
        vis->PollEvents(); // Additional call to ensure rendering completion


        // Generate dynamic filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "output/bounding_boxes_"
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << ".png";
        std::string filename = ss.str();

        // Save screenshot
        vis->CaptureScreenImage(filename);

        // Clean up
        vis->Run();
        vis->DestroyVisualizerWindow();
    }
}
//////////////////////////////////////////////////////////////////////////




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

// Function to generate a random width with up to 2 uses per width
double GeometryProcessor::getRandomWidth(double min, double max, std::map<double, int>& width_counts, double min_variation = 0.001, int max_attempts = 100, double precision = 0.00001) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        double width = dis(gen);
        bool valid = true;

        // Round width to precision for counting (e.g., 0.1243 ≈ 0.1243)
        double rounded_width = std::round(width / precision) * precision;

        // Check if width can be used (≤ 2 uses)
        if (width_counts[rounded_width] >= 2) {
            valid = false;
        } else {
            // Check variation from existing widths
            for (const auto& [existing_width, count] : width_counts) {
                if (std::abs(width - existing_width) < min_variation && std::abs(width - existing_width) > precision) {
                    valid = false;
                    break;
                }
            }
        }

        if (valid) {
            width_counts[rounded_width]++;
            return width;
        }
    }

    // Fallback: Try incremental widths
    double new_width = min;
    while (new_width <= max) {
        double rounded_width = std::round(new_width / precision) * precision;
        bool valid = true;

        if (width_counts[rounded_width] >= 2) {
            valid = false;
        } else {
            for (const auto& [existing_width, count] : width_counts) {
                if (std::abs(new_width - existing_width) < min_variation && std::abs(new_width - existing_width) > precision) {
                    valid = false;
                    break;
                }
            }
        }

        if (valid) {
            width_counts[rounded_width]++;
            return new_width;
        }
        new_width += min_variation;
    }

    // If no valid width is found, return -1
    std::cerr << "[ERROR] Could not generate a width with up to 2 uses and minimum variation of " << min_variation * 1000.0 << " mm." << std::endl;
    return -1.0;
}

// Function to create 'n' bounding boxes with random widths (up to 2 uses per width)
std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> GeometryProcessor::createBoundingBoxes(
    int n, double fixed_length, bool debug = false) {
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> boxes;
    std::map<double, int> width_counts; // Tracks number of uses per width

    double min_width = 0.08;  // 8 cm
    double max_width = 0.20;  // 20 cm
    double min_variation = 0.001;  // 0.1 cm
    double precision = 0.00001;  // 0.001 cm for considering widths "similar"

    // Check if n is feasible
    double width_range = max_width - min_width;
    int max_unique_widths = static_cast<int>(width_range / min_variation) + 1;
    int max_possible_boxes = max_unique_widths * 2; // Up to 2 uses per width
    if (n > max_possible_boxes) {
        std::cerr << "[ERROR] Requested " << n << " boxes, but only " << max_possible_boxes
                  << " boxes are possible with " << min_variation * 1000.0 << " mm variation and 2 uses per width." << std::endl;
        return boxes;
    }

    // Start position for the first box
    Eigen::Vector3d start_pos(-0.5, 0.3, 0.0);
    double current_x = start_pos.x();

    for (int i = 0; i < n; ++i) {
        // Generate a width
        double width = getRandomWidth(min_width, max_width, width_counts, min_variation, 100, precision);
        if (width < 0) {
            std::cerr << "[ERROR] Failed to generate width for box " << i + 1 << ". Stopping." << std::endl;
            return boxes;
        }

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
        if (debug) {
            std::cout << "Bounding Box " << i + 1 << ": Width = " 
                      << computed_width << "m, Expected Length = " 
                      << fixed_length << "m, Computed Length = " 
                      << computed_length << "m" << std::endl;
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

std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
GeometryProcessor::arrangeFirstShingleRow(
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes,
    double gap,
    double max_length,
    double rotation_angle)
{
    double overhang_allowed = 0.02;
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> best_combination;
    double best_total_length = std::numeric_limits<double>::max();

    // Generate all combinations
    size_t n = bounding_boxes.size();
    for (size_t mask = 1; mask < (1 << n); ++mask) {
        std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> combination;
        double total_length = 0;
        for (size_t i = 0; i < n; ++i) {
            if (mask & (1 << i)) {
                combination.push_back(bounding_boxes[i]);
                total_length += bounding_boxes[i]->extent_.x();
            }
        }
        total_length += gap * (combination.size() - 1); // Gaps between boxes

        if (total_length >= max_length && total_length <= max_length + overhang_allowed) {
            if (total_length < best_total_length) {
                best_total_length = total_length;
                best_combination = combination;
            }
        }
    }

    if (best_combination.empty()) {
        std::cerr << "[Warning] No valid shingle combination found for target length: "
                  << max_length << "m with allowed overhang: " << overhang_allowed << "m" << std::endl;
        return {};
    }

    // Placement logic
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arranged_bboxes;
    Eigen::Vector3d current_position(0, 0, 0);
    double previous_half_width = 0;
    bool is_first = true;


    for (auto& bbox : best_combination) {
        double current_half_width = bbox->extent_.x() / 2.0;
    if (is_first) {
        current_position.x() += current_half_width;
        is_first = false;
    } else {
        current_position.x() += previous_half_width + gap + current_half_width;
    }

    Eigen::Vector3d translation = current_position - bbox->GetCenter();
    transform_bounding_box(bbox, translation, Eigen::Vector3d(0, 1, 0), 0, Eigen::Vector3d(0, 0, 0), true);

    arranged_bboxes.push_back(bbox);
    previous_half_width = current_half_width;
}

    // Apply rotation if needed
    double rotation_radians = rotation_angle * M_PI / 180.0;
    for (auto& bbox : arranged_bboxes) {
        transform_bounding_box(bbox, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),
                               -rotation_radians, bbox->GetCenter(), true);
    }

    // Compute actual length from the x-position of the first and last box
    auto first_bbox = arranged_bboxes.front();
    auto last_bbox = arranged_bboxes.back();

    double first_left = first_bbox->GetCenter().x() - first_bbox->extent_.x() / 2.0;
    double last_right = last_bbox->GetCenter().x() + last_bbox->extent_.x() / 2.0;

    double actual_total_length = last_right - first_left;

std::cout << "[Info] Target total length (ideal): " << best_total_length
          << " m, Actual arranged length: " << actual_total_length << " m" << std::endl;

    exportFirstRowData(arranged_bboxes ,actual_total_length );

    return arranged_bboxes;
}

/////////////////////////////////////////



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


///////////////////////////////////

// exportFirstRowData function
void GeometryProcessor::exportFirstRowData(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& arranged_bboxes,
    double total_length) {

    // Debug: Log received total_length
    std::cout << "[DEBUG] exportFirstRowData received total_length: " << total_length * 1000.0 << " mm" << std::endl;

    // Use a fixed filename instead of timestamp-based
    std::string filename = "output/export_shingle_data/first_row.txt";

    // Open file (overwrites if it exists)
    std::ofstream out_file(filename);
    if (!out_file.is_open()) {
        std::cerr << "[ERROR] Failed to open file: " << filename << std::endl;
        return;
    }

    // Write shingle widths in centimeters
    out_file << "widths = [";
    for (size_t i = 0; i < arranged_bboxes.size(); ++i) {
        double width_cm = arranged_bboxes[i]->extent_.x() * 100.0; // Convert meters to cm
        out_file << std::fixed << std::setprecision(0) << width_cm;
        if (i < arranged_bboxes.size() - 1) {
            out_file << ",";
        }
    }
    out_file << "]" << std::endl;

    // Write total length in meters
    out_file << "total length = [" << std::fixed << std::setprecision(6) << total_length << "]" << std::endl;

    // Verify total length by summing widths and gaps (assuming gap = 0.002 m from context)
    double computed_total_length = 0.0;
    double gap = 0.003; // 3 mm, adjust if different
    for (size_t i = 0; i < arranged_bboxes.size(); ++i) {
        computed_total_length += arranged_bboxes[i]->extent_.x();
        if (i < arranged_bboxes.size() - 1) {
            computed_total_length += gap;
        }
    }
    std::cout << "[DEBUG] exportFirstRowData computed total length: " << computed_total_length * 1000.0 << " mm" << std::endl;
    if (std::abs(computed_total_length - total_length) > 1e-6) {
        std::cerr << "[ERROR] Mismatch in exportFirstRowData: received total_length = " << total_length * 1000.0
                  << " mm, computed = " << computed_total_length * 1000.0 << " mm" << std::endl;
    }

    // Close file
    out_file.close();
    std::cout << "[INFO] Exported first row data to: " << filename << std::endl;
}


void GeometryProcessor::exportShingleData(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row,
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row_aligned,
    const std::vector<double>& first_row_gap_positions,
    double max_gap) const {
    if (second_row.empty()) {
        std::cout << "[DEBUG] No shingles to export." << std::endl;
        return;
    }

    // Generate unique filenames using a static counter
    static int file_counter = 0;
    ++file_counter;
    std::string csv_filename = "output/export_shingle_data/shingle_data_" + std::to_string(file_counter) + ".csv";
    std::string txt_filename = "output/export_shingle_data/shingle_data_" + std::to_string(file_counter) + ".txt";

    // --- CSV Export (Unchanged) ---
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        std::cerr << "[ERROR] Failed to open CSV file: " << csv_filename << std::endl;
        return;
    }

    // Write CSV header
    csv_file << "Shingle,Width_mm,Stagger_mm\n";

    // --- Text File Export ---
    std::ofstream txt_file(txt_filename);
    if (!txt_file.is_open()) {
        std::cerr << "[ERROR] Failed to open text file: " << txt_filename << std::endl;
        csv_file.close();
        return;
    }

    // Initialize right edge (same as in debugShingleRow)
    Eigen::Vector3d debug_right_edge = first_row_aligned[0]->GetCenter() -
                                       Eigen::Vector3d(first_row_aligned[0]->extent_.x() / 2.0, 0, 0);

    // Collect widths, staggers, and compute total length
    std::vector<double> widths;
    std::vector<double> staggers;
    double total_length_mm = 0.0; // Total length in millimeters

    // Process each shingle
    for (size_t i = 0; i < second_row.size(); ++i) {
        auto& bbox = second_row[i];
        double width_mm = bbox->extent_.x() * 1000.0; // Convert to millimeters
        total_length_mm += width_mm; // Add width to total length

        // Add gap to total length (except for the last shingle)
        if (i < second_row.size() - 1) {
            total_length_mm += max_gap * 1000.0; // Convert gap to millimeters
        }

        // Compute stagger margin (same as in debugShingleRow)
        double second_row_gap = debug_right_edge.x() + bbox->extent_.x() + max_gap;
        double min_stagger_margin = std::numeric_limits<double>::max();
        for (double first_gap : first_row_gap_positions) {
            double stagger = std::abs(second_row_gap - first_gap);
            min_stagger_margin = std::min(min_stagger_margin, stagger);
        }
        double stagger_mm = min_stagger_margin * 1000.0; // Convert to millimeters

        // Write to CSV
        csv_file << (i + 1) << "," << std::fixed << std::setprecision(3) << width_mm << ","
                 << stagger_mm << "\n";

        // Store for text file
        widths.push_back(width_mm);
        staggers.push_back(stagger_mm);

        // Update right edge for next shingle
        debug_right_edge.x() += bbox->extent_.x() + max_gap;
    }

    csv_file.close();
    std::cout << "[DEBUG] Exported shingle data to " << csv_filename << std::endl;

    // Write to text file in the requested format
    txt_file << "row" << file_counter << "_s=[";
    for (size_t i = 0; i < staggers.size(); ++i) {
        txt_file << std::fixed << std::setprecision(3) << staggers[i];
        if (i < staggers.size() - 1) {
            txt_file << ", ";
        }
    }
    txt_file << "] \n";

    txt_file << "row" << file_counter << "=[";
    for (size_t i = 0; i < widths.size(); ++i) {
        txt_file << std::fixed << std::setprecision(3) << widths[i];
        if (i < widths.size() - 1) {
            txt_file << ", ";
        }
    }
    txt_file << "] \n";

    // Add total length line
    txt_file << "row" << file_counter << "_length=[" << std::fixed << std::setprecision(3) << total_length_mm << "]\n";

    txt_file.close();
    std::cout << "[DEBUG] Exported shingle data to " << txt_filename << std::endl;
}



//////////////////////////////////////////
// ///////////////////////////////////////
// 
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
                score -= 1000.0 * (0.035 - offset); // Penalty for yellow shingles
            } else if (offset >= 0.035 || is_final_shingle) {
                score += 250.0 * std::min(offset, 0.06); // Reward for green shingles
                // Bonus for staggers close to 50 mm (within ±5 mm)
                if (!is_final_shingle && offset >= 0.050 && offset <= 0.070) {
                    score += 500.0; // Extra reward for ~50 mm stagger
                }
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
                        best_fit_score += 2500.0 * (0.05 - final_remaining);
                        if (final_remaining <= 0.02) {
                            best_fit_score += 1500.0 * (0.02 - final_remaining);
                        }
                    } else if (final_remaining < 0) {
                        best_fit_score -= 5000.0 * std::abs(final_remaining);
                    } else {
                        best_fit_score -= 4000.0 * final_remaining;
                    }
                }
            }
            score += best_fit_score;
            if (!can_fill) {
                score -= 1500.0;
            }
        }

        // Width constraints
        if (is_final_shingle) {
            if (new_remaining_width >= 0 && new_remaining_width <= 0.05) {
                score += 2500.0 * (0.05 - new_remaining_width);
                if (new_remaining_width <= 0.02) {
                    score += 1500.0 * (0.02 - new_remaining_width);
                }
            } else if (new_remaining_width > 0.05) {
                score -= 5000.0 * (new_remaining_width - 0.05);
            } else {
                score -= 10000.0 * std::abs(new_remaining_width);
            }
        } else if (new_remaining_width <= max_candidate_width && new_remaining_width >= 0) {
            score += 1500.0 * (0.05 - std::min(new_remaining_width, 0.05));
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
            score -= 5000.0 * std::abs(new_remaining_width);
            if (new_total_width > max_length + 0.05) {
                score -= 10000.0 * (new_total_width - (max_length + 0.05));
            }
        } else {
            score -= 4000.0 * new_remaining_width;
        }

        return score;
    };



    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////

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

    // Export shingle widths
    exportShingleData(second_row , first_row_aligned,first_row_gap_positions , max_gap );

    return second_row;
}

/////////////////////////////////////////////
/////////////////////////////////////////////
/////////////////////////////////////////////



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
        auto next_row = findNextBestShingles(previous_row, current_candidates_copy, min_stagger, max_gap, max_length, false);
        std::cout << "[DEBUG] Candidates AFTER arranging row " << i + 1 << ": " << current_candidates_copy.size() << "\n";


        if (next_row.empty()) {
            std::cout << "No valid shingles found for row " << i + 1 << ". Stopping arrangement.\n";
            break;
        }

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




//////////////////////////////////////////////////////////
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

/////////////////////////////
std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>
GeometryProcessor::arrangeLastTwoShingleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& candidate_rows,
    double gap,
    double max_length,
    double rotation_angle,
    double vertical_overlap)
{
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> candidate_rows_copy;
    
    // Deep copy only the last two rows, if available
    if (candidate_rows.size() >= 2) {
        candidate_rows_copy.push_back(copyBoundingBoxes(candidate_rows[candidate_rows.size() - 2])); // Second-to-last row
        candidate_rows_copy.push_back(copyBoundingBoxes(candidate_rows[candidate_rows.size() - 1])); // Last row
    } else if (candidate_rows.size() == 1) {
        candidate_rows_copy.push_back(copyBoundingBoxes(candidate_rows[0])); // Only one row available
    } else {
        std::cerr << "Error: Insufficient candidate rows to process (need at least one)." << std::endl;
        return {};
    }

    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> arranged_rows;
    std::cout << "Arranging last two rows...\n";

    auto previous_row = reference_row;

    for (size_t i = 0; i < candidate_rows_copy.size(); ++i) {
        std::cout << "Arranging row " << i + 1 << " (of last two)...\n";

        auto candidate_row_copy = copyBoundingBoxes(candidate_rows_copy[i]);
        auto arranged_row = arrangeShingleRow(previous_row, candidate_row_copy, gap, max_length, rotation_angle, 0);

        if (arranged_row.empty()) {
            std::cout << "No valid shingles found for row " << i + 1 << ". Skipping this row.\n";
            continue;
        }

        arranged_rows.push_back(arranged_row);
        previous_row = arranged_row;
    }

    // Apply vertical overlap shift to both rows (same shift for alignment)
    if (vertical_overlap != 0 && !arranged_rows.empty()) {
        Eigen::Vector3d y_direction = reference_row[0]->R_.col(1);
        Eigen::Vector3d vertical_shift = y_direction * (-vertical_overlap);

        for (auto& row : arranged_rows) {
            for (auto& bbox : row) {
                bbox->Translate(vertical_shift);
            }
        }
    }

    std::cout << "Shingle arrangement for last two rows completed.\n";
    return arranged_rows;
}


///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

std::pair<std::vector<PC_o3d_ptr>, std::map<std::shared_ptr<open3d::geometry::OrientedBoundingBox>, int>> GeometryProcessor::alignPointCloudsToArrangedBoxes(
    const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& arranged_boxes,
    const std::vector<std::tuple<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr, int>>& original_box_cloud_pairs)
{
    std::vector<PC_o3d_ptr> aligned_clouds;
    std::map<std::shared_ptr<open3d::geometry::OrientedBoundingBox>, int> box_to_shingle_id;

    for (const auto& row : arranged_boxes) {
        for (const auto& arranged_box_ptr : row) {
            const auto& arranged_box = *arranged_box_ptr;

            // Find corresponding original box with shingle ID
            auto it = std::find_if(
                original_box_cloud_pairs.begin(),
                original_box_cloud_pairs.end(),
                [&arranged_box](const std::tuple<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr, int>& tuple) {
                    const auto& orig_box = std::get<0>(tuple);
                    return (orig_box.extent_ - arranged_box.extent_).norm() < 1e-4;
                });

            if (it == original_box_cloud_pairs.end()) {
                std::cerr << "Warning: No matching box found for arranged box.\n";
                continue;
            }

            const auto& [original_box, cloud, shingle_id] = *it;

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
            box_to_shingle_id[arranged_box_ptr] = shingle_id;
        }
    }

    return {aligned_clouds, box_to_shingle_id};
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
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud,
    bool save_png,
    const std::string& output_path,
    const Eigen::Vector3d& front,
    const Eigen::Vector3d& lookat,
    const Eigen::Vector3d& up,
    double zoom)
{
    std::shared_ptr<open3d::visualization::Visualizer> vis =
        std::make_shared<open3d::visualization::Visualizer>();
    vis->CreateVisualizerWindow("Open3D Point Cloud Viewer", 3840, 2160, 50, 50);

    vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0); // White background
    vis->GetRenderOption().point_size_ = 2.0; // Point size
    vis->GetRenderOption().point_color_option_ = open3d::visualization::RenderOption::PointColorOption::Color;

    if (point_cloud != nullptr) {
        if (point_cloud->HasColors()) {
            std::cout << "Adding background point_cloud with " << point_cloud->colors_.size() << " colors.\n";
        } else {
            std::cerr << "Background point_cloud has no colors, assigning default gray.\n";
            point_cloud->colors_.resize(point_cloud->points_.size(), Eigen::Vector3d(0.5, 0.5, 0.5));
        }
        vis->AddGeometry(point_cloud, true);
    }

    for (const auto& cloud : clouds) {
        if (cloud->HasColors()) {
            std::cout << "Adding cloud with " << cloud->colors_.size() << " colors.\n";
        } else {
            std::cerr << "Cloud has no colors, assigning default red.\n";
            cloud->colors_.resize(cloud->points_.size(), Eigen::Vector3d(1.0, 0.0, 0.0));
        }
        vis->AddGeometry(cloud, true);
    }

    // Use the provided camera parameters
    auto& view_control = vis->GetViewControl();
    view_control.SetFront(front);
    view_control.SetLookat(lookat);
    view_control.SetUp(up);
    view_control.SetZoom(zoom);

    if (save_png) {
        vis->PollEvents();
        vis->UpdateRender();
        vis->CaptureScreenImage(output_path);
        std::cout << "Saved screenshot to " << output_path << "\n";
        vis->DestroyVisualizerWindow();
    } 
    else {
        vis->Run();
        vis->DestroyVisualizerWindow();
    }
    


}

//////////////////////////////////////////////////////
void GeometryProcessor::visualizeSingleShingle(
    std::shared_ptr<open3d::geometry::PointCloud> shingle,
    std::shared_ptr<open3d::geometry::PointCloud> background_cloud,
    bool save_png,
    const std::string& output_path)
{
    // Create a copy of the shingle to modify its color without affecting the original
    auto shingle_copy = std::make_shared<open3d::geometry::PointCloud>(*shingle);
    
    // Set the shingle color to red
    if (!shingle_copy->HasColors()) {
        shingle_copy->colors_.resize(shingle_copy->points_.size(), Eigen::Vector3d(1.0, 0.0, 0.0)); // Red
    } else {
        // If it has colors, override with red
        shingle_copy->colors_ = std::vector<Eigen::Vector3d>(shingle_copy->points_.size(), Eigen::Vector3d(1.0, 0.0, 0.0));
    }

    // Create a copy of the background cloud to modify
    auto modified_background = std::make_shared<open3d::geometry::PointCloud>(*background_cloud);
    
    // If the background has no colors, set to gray
    if (!modified_background->HasColors()) {
        modified_background->colors_.resize(modified_background->points_.size(), Eigen::Vector3d(0.5, 0.5, 0.5)); // Gray
    }

    // Get the bounding box of the shingle to exclude its points from the background
    if (!shingle_copy->points_.empty()) {
        auto shingle_bbox = shingle_copy->GetAxisAlignedBoundingBox();
        std::vector<int> indices_to_remove;
        
        // Iterate over background points to find those within the shingle's bounding box
        for (size_t i = 0; i < modified_background->points_.size(); ++i) {
            Eigen::Vector3d point = modified_background->points_[i];
            // Manual containment check
            if (point.x() >= shingle_bbox.min_bound_.x() && point.x() <= shingle_bbox.max_bound_.x() &&
                point.y() >= shingle_bbox.min_bound_.y() && point.y() <= shingle_bbox.max_bound_.y() &&
                point.z() >= shingle_bbox.min_bound_.z() && point.z() <= shingle_bbox.max_bound_.z()) {
                indices_to_remove.push_back(i);
            }
        }

        // Remove points from the background cloud
        if (!indices_to_remove.empty()) {
            std::vector<Eigen::Vector3d> new_points;
            std::vector<Eigen::Vector3d> new_colors;
            new_points.reserve(modified_background->points_.size() - indices_to_remove.size());
            if (modified_background->HasColors()) {
                new_colors.reserve(modified_background->colors_.size() - indices_to_remove.size());
            }

            size_t current_idx = 0;
            for (size_t i = 0; i < modified_background->points_.size(); ++i) {
                if (current_idx < indices_to_remove.size() && i == static_cast<size_t>(indices_to_remove[current_idx])) {
                    current_idx++;
                    continue;
                }
                new_points.push_back(modified_background->points_[i]);
                if (modified_background->HasColors()) {
                    new_colors.push_back(modified_background->colors_[i]);
                }
            }

            modified_background->points_ = new_points;
            if (modified_background->HasColors()) {
                modified_background->colors_ = new_colors;
            }
        }
    }

    // Define top-down camera view
    Eigen::Vector3d front(0.0, 0.0, -1.0); // Look straight down (along negative Z-axis)
    Eigen::Vector3d up(0.0, 1.0, 0.0);     // Y-axis as up
    double zoom = 0.5;                      // Adjust zoom as needed

    // Compute the lookat point (center of the scene)
    Eigen::Vector3d lookat(0.0, 0.0, 0.0);
    if (modified_background && !modified_background->points_.empty()) {
        auto bbox = modified_background->GetAxisAlignedBoundingBox();
        lookat = bbox.GetCenter();
    } else if (shingle_copy && !shingle_copy->points_.empty()) {
        auto bbox = shingle_copy->GetAxisAlignedBoundingBox();
        lookat = bbox.GetCenter();
    }

    // Visualize using the existing visualizePointClouds function with top-down view
    std::vector<PC_o3d_ptr> shingle_vector = {shingle_copy};
    visualizePointClouds(shingle_vector, modified_background, save_png, output_path, front, lookat, up, zoom);
}

////////////////////////////////////////////////
    void GeometryProcessor::visualizeArrangedCloudCorrespondence(
        const std::vector<PC_o3d_ptr>& all_point_clouds,
        const std::vector<PC_o3d_ptr>& arranged_clouds,
        const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& arranged_boxes,
        const std::vector<std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>>& original_box_cloud_pairs)
    {
        auto vis = std::make_shared<open3d::visualization::Visualizer>();
        vis->CreateVisualizerWindow("Point Cloud Correspondence Viewer", 1280, 720, 50, 50);
        vis->GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0);
        vis->GetRenderOption().point_size_ = 2.0;
        vis->GetRenderOption().point_color_option_ = open3d::visualization::RenderOption::PointColorOption::Color;

        size_t arranged_idx = 0;
        for (const auto& row : arranged_boxes) {
            for (const auto& arranged_box_ptr : row) {
                if (arranged_idx >= arranged_clouds.size()) {
                    std::cerr << "Warning: Mismatch between arranged_boxes and arranged_clouds." << std::endl;
                    break;
                }

                const auto& arranged_box = *arranged_box_ptr;
                const auto& aligned_cloud = arranged_clouds[arranged_idx];

                auto it = std::find_if(
                    original_box_cloud_pairs.begin(),
                    original_box_cloud_pairs.end(),
                    [&arranged_box](const std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>& pair) {
                        const auto& orig_box = pair.first;
                        return (orig_box.extent_ - arranged_box.extent_).norm() < 1e-4;
                    });

                if (it == original_box_cloud_pairs.end()) {
                    std::cerr << "Warning: No matching original box for arranged cloud " << arranged_idx << "." << std::endl;
                    arranged_idx++;
                    continue;
                }

                const auto& original_cloud = it->second;
                PC_o3d_ptr matched_original_cloud = nullptr;
                for (const auto& cloud : all_point_clouds) {
                    if (cloud == original_cloud) {
                        matched_original_cloud = cloud;
                        break;
                    }
                }

                if (!matched_original_cloud) {
                    std::cerr << "Warning: Could not find original cloud for arranged cloud " << arranged_idx << "." << std::endl;
                    arranged_idx++;
                    continue;
                }

                vis->ClearGeometries();

                auto original_cloud_copy = std::make_shared<open3d::geometry::PointCloud>(*matched_original_cloud);
                original_cloud_copy->Translate(Eigen::Vector3d(-1.0, 0.0, 0.0));
                if (!original_cloud_copy->HasColors()) {
                    original_cloud_copy->colors_.resize(original_cloud_copy->points_.size(), Eigen::Vector3d(0.0, 0.0, 1.0));
                }
                vis->AddGeometry(original_cloud_copy, true);

                auto aligned_cloud_copy = std::make_shared<open3d::geometry::PointCloud>(*aligned_cloud);
                aligned_cloud_copy->Translate(Eigen::Vector3d(1.0, 0.0, 0.0));
                if (!aligned_cloud_copy->HasColors()) {
                    aligned_cloud_copy->colors_.resize(aligned_cloud_copy->points_.size(), Eigen::Vector3d(1.0, 0.0, 0.0));
                }
                vis->AddGeometry(aligned_cloud_copy, true);

                auto& view_control = vis->GetViewControl();
                view_control.SetFront(Eigen::Vector3d(0.0, 0.0, 1.0));
                view_control.SetUp(Eigen::Vector3d(0.0, 1.0, 0.0));
                view_control.SetLookat(Eigen::Vector3d(0.0, 0.0, 0.0));
                view_control.SetZoom(0.5);

                vis->PollEvents();
                vis->UpdateRender();

                std::cout << "Visualizing pair " << arranged_idx + 1 << "/" << arranged_clouds.size()
                          << ": Original cloud (blue, left), Aligned cloud (red, right)." << std::endl;
                std::cout << "Press Enter to continue to the next pair (or Ctrl+C to exit)..." << std::endl;
                std::cin.get();

                arranged_idx++;
            }
        }

        vis->DestroyVisualizerWindow();
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
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud, // Optional argument
    bool save_png, // New parameter to control PNG export
    const std::string& output_path) // Default output path
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

    // Eigen::Vector3d scene_center(0.0, 0.0, 0.0);

    // If a point cloud is provided, add it to the visualization without changing its color
    if (point_cloud && !point_cloud->IsEmpty()) {
        all_geometries.push_back(point_cloud);
    }

    // Add coordinate frame for reference
    auto coordinate_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.1);
    all_geometries.push_back(coordinate_frame);

    if (save_png) {
        // Create a visualizer
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow("Shingle Visualization", 1600, 1200); // Set window size for high-res

        // Add geometries
        for (const auto& geom : all_geometries) {
            visualizer.AddGeometry(geom);
        }

        // Optional: Adjust view for better perspective
        auto& view_control = visualizer.GetViewControl();
        view_control.SetFront(Eigen::Vector3d(0.5, 0.4, 0.5).normalized());
        view_control.SetLookat(Eigen::Vector3d(0.7, 0.8, 0)); // Center of the scene
        view_control.SetUp(Eigen::Vector3d(0, 0, 1));     // Up direction
        view_control.SetZoom(0.5);                        // Zoom level

        // Render and capture the image
        visualizer.PollEvents();
        visualizer.UpdateRender();
        visualizer.CaptureScreenImage(output_path);

        // Close the visualizer
        visualizer.DestroyVisualizerWindow();
    } else {
        // Default visualization without saving
        open3d::visualization::DrawGeometries(all_geometries);
    }
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



//////////////////////////////////////////////

void GeometryProcessor::exportBoundingBoxesAsPolylines(
    const std::vector<open3d::geometry::OrientedBoundingBox>& boxes,
    const std::string& folder,
    const Eigen::Vector3d& color,
    const std::string& prefix,
    double scale_factor)
{


    int index = 0;
    for (const auto& box : boxes) {
        // Compute the eight vertices of the oriented bounding box
        Eigen::Vector3d center = box.center_ * scale_factor; // Scale center
        Eigen::Vector3d extent = box.extent_ * scale_factor; // Scale extent
        Eigen::Matrix3d R = box.R_; // Rotation matrix (unitless, no scaling)

        // Define the eight corners of the box in local coordinates
        std::vector<Eigen::Vector3d> local_vertices = {
            Eigen::Vector3d(-extent(0), -extent(1), -extent(2)), // 0: bottom-left-back
            Eigen::Vector3d( extent(0), -extent(1), -extent(2)), // 1: bottom-right-back
            Eigen::Vector3d( extent(0),  extent(1), -extent(2)), // 2: top-right-back
            Eigen::Vector3d(-extent(0),  extent(1), -extent(2)), // 3: top-left-back
            Eigen::Vector3d(-extent(0), -extent(1),  extent(2)), // 4: bottom-left-front
            Eigen::Vector3d( extent(0), -extent(1),  extent(2)), // 5: bottom-right-front
            Eigen::Vector3d( extent(0),  extent(1),  extent(2)), // 6: top-right-front
            Eigen::Vector3d(-extent(0),  extent(1),  extent(2))  // 7: top-left-front
        };

        // Transform vertices to world coordinates
        std::vector<Eigen::Vector3d> vertices;
        for (const auto& local_vertex : local_vertices) {
            vertices.push_back(R * local_vertex + center);
        }

        // Debug: Print vertex coordinates
        std::cout << "Box " << index << " vertices (mm):\n";
        for (size_t i = 0; i < vertices.size(); ++i) {
            std::cout << "  Vertex " << i << ": (" << vertices[i](0) << ", "
                      << vertices[i](1) << ", " << vertices[i](2) << ")\n";
        }

        // Define the 12 edges of the box
        std::vector<Eigen::Vector2i> edges = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Back face
            {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Front face
            {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Connecting edges
        };

        // Create PLY file
        std::string filename = folder + "/" + prefix + "box_" + std::to_string(index) + ".ply";
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            continue;
        }

        // Write PLY header
        file << "ply\n";
        file << "format ascii 1.0\n";
        file << "element vertex " << vertices.size() << "\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";
        file << "property uchar red\n"; // Use uchar for Rhino compatibility
        file << "property uchar green\n";
        file << "property uchar blue\n";
        file << "element edge " << edges.size() << "\n";
        file << "property int vertex1\n";
        file << "property int vertex2\n";
        file << "end_header\n";

        // Write vertices with color (scale RGB to [0, 255] for uchar)
        for (const auto& vertex : vertices) {
            file << vertex(0) << " " << vertex(1) << " " << vertex(2) << " "
                 << static_cast<int>(color(0) * 255) << " "
                 << static_cast<int>(color(1) * 255) << " "
                 << static_cast<int>(color(2) * 255) << "\n";
        }

        // Write edges
        for (const auto& edge : edges) {
            file << edge(0) << " " << edge(1) << "\n";
        }

        file.close();
        std::cout << "Exported bounding box to " << filename << std::endl;
        index++;
    }
}
///////////////////////////////////////////////
void GeometryProcessor::exportPointClouds(
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& arranged_clouds,
    const std::string& folder,
    const std::string& prefix)
{
    if (arranged_clouds.empty()) {
        std::cerr << "No point clouds to export.\n";
        return;
    }



    int index = 0;
    for (const auto& cloud : arranged_clouds) {
        if (!cloud || cloud->points_.empty()) {
            std::cerr << "Skipping empty or null point cloud at index " << index << ".\n";
            continue;
        }

        // Check extent to detect unit scale
        auto bbox = cloud->GetAxisAlignedBoundingBox();
        Eigen::Vector3d extent = bbox.max_bound_ - bbox.min_bound_;
        std::cout << "Exporting cloud " << index << " extent: " << extent.transpose() << "\n";

        // Scale to meters if in millimeters
        auto export_cloud = std::make_shared<open3d::geometry::PointCloud>(*cloud);
        if (extent.x() > 100 || extent.y() > 100 || extent.z() > 100) {
            std::cout << "Cloud " << index << " is in millimeters. Scaling to meters.\n";
            export_cloud->Scale(0.001, export_cloud->GetCenter());
        }

        // Check if the point cloud has colors
        if (!export_cloud->HasColors()) {
            std::cerr << "Point cloud at index " << index << " has no colors. Assigning default color (white).\n";
            export_cloud->colors_.resize(export_cloud->points_.size(), Eigen::Vector3d(1.0, 1.0, 1.0));
        }

        // Export the point cloud
        std::string filename = folder + "/" + prefix + "cloud_" + std::to_string(index) + ".ply";
        if (!open3d::io::WritePointCloud(filename, *export_cloud)) {
            std::cerr << "Failed to write point cloud file: " << filename << "\n";
        } else {
            std::cout << "Exported point cloud to " << filename << "\n";
        }
        index++;
    }
}

///////////////////////////////////////////////////////////////////////

// Conversion function
std::vector<std::tuple<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr, int>> GeometryProcessor::convertPairsToTuples(
    const std::vector<std::pair<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr>>& pairs) {
    std::vector<std::tuple<open3d::geometry::OrientedBoundingBox, PC_o3d_ptr, int>> tuples;
    tuples.reserve(pairs.size());

    for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& [box, cloud] = pairs[i];
        tuples.emplace_back(box, cloud, static_cast<int>(i));
    }

    return tuples;
}




//////////////////////////////////////////////////////////////

void GeometryProcessor::visualizeAndExportCorrespondingShingles(
    const std::vector<PC_o3d_ptr>& corresponding_shingles,
    const PC_o3d_ptr& full_cloud,
    const std::string& screenshot_dir,
    const std::string& ply_dir)
{
    if (corresponding_shingles.empty()) {
        std::cerr << "No corresponding shingles to visualize.\n";
        return;
    }

    std::cout << "Visualizing and exporting corresponding original shingles one by one...\n";


    for (size_t i = 0; i < corresponding_shingles.size(); ++i) {
        // Visualize the single corresponding shingle with the full cloud in the background
        std::string screenshot_path = screenshot_dir + "/shingle_" + std::to_string(i) + ".png";
        std::cout << "Visualizing shingle " << i << " (saving to " << screenshot_path << ")\n";
        visualizeSingleShingle(corresponding_shingles[i], full_cloud, true, screenshot_path);

        // Export the single corresponding shingle to a PLY file
        std::vector<PC_o3d_ptr> single_shingle = {corresponding_shingles[i]};
        exportPointClouds(single_shingle, ply_dir, "shingle_" + std::to_string(i) + "_");
    }
}





////////////////////////////////////////////////////////////////////////

void GeometryProcessor::visualizeArrangedCloudsIncrementally(
    const std::vector<PC_o3d_ptr>& arranged_clouds,
    const PC_o3d_ptr& sub_structure_pc,
    const std::string& output_dir)
{
    if (arranged_clouds.empty() || !sub_structure_pc || sub_structure_pc->IsEmpty()) {
        std::cerr << "No arranged clouds or valid substructure to visualize.\n";
        return;
    }

    // Check substructure colors
    if (sub_structure_pc->HasColors()) {
        std::cout << "sub_structure_pc has " << sub_structure_pc->colors_.size() << " colors." << std::endl;
    } else {
        std::cerr << "sub_structure_pc has no colors." << std::endl;
    }

    std::cout << "Visualizing arranged clouds incrementally with substructure...\n";


    std::vector<PC_o3d_ptr> accumulated_clouds;
    for (size_t i = 0; i < arranged_clouds.size(); ++i) {
        // Accumulate the current cloud
        accumulated_clouds.push_back(arranged_clouds[i]);

        // Compute custom look-at point
        Eigen::Vector3d custom_lookat(0.0, 0.0, 0.0);
        if (!sub_structure_pc->points_.empty()) {
            auto bbox = sub_structure_pc->GetAxisAlignedBoundingBox();
            custom_lookat = bbox.GetCenter();
        }

        // Visualize accumulated clouds with substructure
        std::string png_path = output_dir + "/sh_" + std::to_string(i) + ".png";
        std::cout << "Visualizing up to cloud " << i << " (saving to " << png_path << ")\n";
        visualizePointClouds(accumulated_clouds, sub_structure_pc, true, png_path);
    }
}

///////////////////////////////////////////////////////////////////////////
void GeometryProcessor::visualizeShingleMeshesIncrementally(
    const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& combined_rows,
    const PC_o3d_ptr& sub_structure_pc,
    const std::string& output_dir)
{
    if (combined_rows.empty() || !sub_structure_pc || sub_structure_pc->IsEmpty()) {
        std::cerr << "No combined rows or valid substructure to visualize.\n";
        return;
    }

    // Count total number of boxes
    size_t total_boxes = 0;
    for (const auto& row : combined_rows) {
        total_boxes += row.size();
    }

    if (total_boxes == 0) {
        std::cerr << "No boxes in combined rows to visualize.\n";
        return;
    }

    std::cout << "Visualizing shingle meshes incrementally by box with substructure...\n";


    // Initialize accumulated_rows
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> accumulated_rows(combined_rows.size());
    size_t box_counter = 0;

    for (size_t row_idx = 0; row_idx < combined_rows.size() && box_counter < total_boxes; ++row_idx) {
        const auto& row = combined_rows[row_idx];
        for (size_t box_idx = 0; box_idx < row.size() && box_counter < total_boxes; ++box_idx) {
            // Add current box to accumulated_rows
            accumulated_rows[row_idx].push_back(row[box_idx]);

            // Visualize accumulated rows with substructure
            std::string png_path = output_dir + "/box_" + std::to_string(box_counter) + ".png";
            std::cout << "Visualizing up to box " << box_counter << " (saving to " << png_path << ")\n";
            visualizeShingleMeshes(accumulated_rows, sub_structure_pc, true, png_path);

            box_counter++;
        }
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



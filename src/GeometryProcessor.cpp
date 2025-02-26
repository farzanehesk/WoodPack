#include "../include/GeometryProcessor.hpp"
#include <open3d/Open3D.h>
#include <iostream>
#include <Eigen/Eigenvalues> // For SelfAdjointEigenSolver
#include <cmath>
#include <Eigen/Dense>



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



//////////////////////////////////////////////
//  // 8. Method to generate a list of random rectangles
//  std::vector<Rectangle> GeometryProcessor::generateRandomRectangles(int count)
// { 
// std::vector<Rectangle> rectangles; 
// std::random_device rd;
// std::mt19937 gen(rd());
// std::uniform_real_distribution<double> width_dist(0.08, 0.2); 
// //std::uniform_real_distribution<double> height_dist(5.0, 15.0); // Example heights 

// for (int i = 0; i < count; ++i)
//  { 
// double width = width_dist(gen); 
// rectangles.push_back({width, 0.25}); 
// } 
// return rectangles; }


// //////////////////////////////////////////////
// // 9. Method to print rectangles
// void GeometryProcessor::printRectangles(const std::vector<Rectangle>& rectangles) 
// { 
// std::cout << "Rectangles in Row:\n"; 
// for (const auto& rect : rectangles) 
// { 
// std::cout << "Width: " << rect.width << " cm, " << "Length: " << rect.length << " cm, " << "X Position: " << rect.x_position << " cm\n"; } }




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
std::vector<Rectangle> GeometryProcessor::extractUpperRectangles(

    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) {

    std::vector<Rectangle> upper_rectangles;

    for (const auto& box : bounding_boxes) {

        // Step 1: Get all 8 corners of the bounding box
        std::vector<Eigen::Vector3d> corners = box.GetBoxPoints();

        // Step 2: Get the normal vector of the bounding box
        Eigen::Vector3d normal = box.R_.col(2); // Z-axis of the bounding box


        // Step 3: Define faces of the bounding box
        std::vector<std::array<Eigen::Vector3d, 4>> faces
        {
            {corners[0], corners[1], corners[2], corners[3]} ,
            {corners[0], corners[1], corners[5], corners[4]},
            {corners[0], corners[1], corners[5], corners[4]},
            {corners[2], corners[3], corners[7], corners[6]},
            {corners[0], corners[3], corners[7], corners[4]},
            {corners[1], corners[2], corners[6], corners[5]}

        };

        
        // Step 4: Find the largest face that is closest to horizontal

        double max_area = 0;

        std::array<Eigen::Vector3d, 4> upper_face;

        for (const auto& face : faces) {

            Eigen::Vector3d edge1 = face[1] - face[0];

            Eigen::Vector3d edge2 = face[2] - face[0];

            double area = edge1.cross(edge2).norm(); // Compute area

            Eigen::Vector3d face_normal = edge1.cross(edge2).normalized();

            // Ensure the face is roughly horizontal and has the largest area

            if (std::abs(face_normal.z()) > 0.9 && area > max_area) {
                max_area = area;
                upper_face = face;
            }
        }

        // Step 5: Ensure a valid upper face was found

        if (max_area == 0) {

            std::cerr << "Warning: No valid upper face found for a bounding box!" << std::endl;

            continue;

        }



        // Step 6: Sort the corners in a consistent order

        std::array<Eigen::Vector3d, 4> sorted_corners = Rectangle::sortCornersClockwise(upper_face);



        // Step 7: Create the rectangle and store it

        upper_rectangles.emplace_back(sorted_corners);

    }



    return upper_rectangles;

}



///////////////////////////////////////////////////////////////
//// 11. Method to visualize rectangles on the original point cloud
void GeometryProcessor::visualizeRectanglesAndOriginalPc(

const std::vector<Rectangle>& rectangles,
const std::shared_ptr<open3d::geometry::PointCloud>& original_pc)
{
// Create a LineSet for visualization
    auto line_set = std::make_shared<open3d::geometry::LineSet>();

    // Add edges of all rectangles

    for (const auto& rect : rectangles) {

        auto rect_lines = rect.getEdges();  // Get rectangle edges as line segments

        for (const auto& edge : rect_lines) {

            line_set->points_.push_back(edge.first);

            line_set->points_.push_back(edge.second);

            line_set->lines_.emplace_back(line_set->points_.size() - 2, line_set->points_.size() - 1);

        }

    }

    // Visualize rectangles on the original point cloud

    open3d::visualization::DrawGeometries({original_pc, line_set}, "Rectangles on Point Cloud");

}


/////////////////////////////////////////////////////////////////////
// Method to visualize rectangle corner points
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





///////////////////////////////////////////////////////////////////////
// std::vector<Eigen::Matrix4d> GeometryProcessor::getPlanesFromBoundingBoxes(

//     const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) {

    

//     std::vector<Eigen::Matrix4d> planes;



//     for (const auto& box : bounding_boxes) {

//         // Step 1: Extract the origin and rotation from the bounding box

//         Eigen::Vector3d origin = box.center_;

//         Eigen::Matrix3d rotation = box.R_;



//         // Step 2: Define the plane's transformation matrix

//         Eigen::Matrix4d plane = Eigen::Matrix4d::Identity();

//         plane.block<3, 3>(0, 0) = rotation;

//         plane.block<3, 1>(0, 3) = origin;



//         // Step 3: Ensure the plane is not flipped

//         // We will ensure the normal points upward by checking the Z-axis vector

//         Eigen::Vector3d normal = rotation.col(2);  // Extract the normal vector (Z direction)

//         if (normal.z() < 0) {

//             // If the normal is flipped, flip it back by inverting the rotation matrix

//             plane.block<3, 3>(0, 0) = -rotation;

//         }



//         planes.push_back(plane);

//     }



//     return planes;

// }





////////////////////////
std::vector<Eigen::Matrix4d> GeometryProcessor::getPlanesFromBoundingBoxes(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) {

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
        std::cout << "Bounding Box " << i << " original axes:\n";
        std::cout << "X-axis: " << x_axis.transpose() << "\n";
        std::cout << "Y-axis: " << y_axis.transpose() << "\n";
        std::cout << "Z-axis (normal): " << normal.transpose() << "\n";
        std::cout << "Origin: " << origin.transpose() << "\n";

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
        std::cout << "Bounding Box " << i << " corrected axes:\n";
        std::cout << "X-axis: " << x_axis.transpose() << "\n";
        std::cout << "Y-axis: " << y_axis.transpose() << "\n";
        std::cout << "Z-axis (normal): " << normal.transpose() << "\n\n";

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
//

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
// static std::array <Eigen::Vector3d , 4> sortCornersClockwise(const std::array<Eigen::Vector3d , 4>& corners) 
// {
// Eigen::Vector3d centroid = (corners_[0] + corners_[1] + corners_[2] + corners_[3]) / 4.0;
// std::sort(corners_.begin(), corners_.end(),[&centroid]
// (const Eigen::Vector3d& a, const Eigen::Vector3d& b) 
// {
// double angle_a = atan2(a.y() - centroid.y(), a.x() - centroid.x());
// double angle_b = atan2(b.y() - centroid.y(), b.x() - centroid.x());
// return angle_a < angle_b;// Sort counterclockwise        
// });}


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










///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////



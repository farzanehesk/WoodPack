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




//////////////////////////////////////////////
// 10. Method to Extract upper rectangles of bounding boxes
// std::vector<Rectangle> GeometryProcessor::extractUpperRectangles
// (const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) 
// {
//     std::vector<Rectangle> upper_rectangles;
// for (const auto& box : bounding_boxes) 
// {
//     // Get all 8 corner points of the bounding box        
// std::vector<Eigen::Vector3d> corners = box.GetBoxPoints();
// // Sort corners based on Z value (height)        
// std::sort(corners.begin(), corners.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) 
// { return a.z() > b.z();
// // Sort descending to get highest points first        
// });
// // Select the highest four points        
// std::array<Eigen::Vector3d, 4> upper_corners; 
// for (int i = 0; i < 4; ++i) 
// {
//     upper_corners[i] = corners[i];
// }
// // Create a Rectangle object        
// Rectangle rect(upper_corners);
// // Add to the list        
// upper_rectangles.push_back(rect);
// //rect.visualizeEdges();

// }
// return upper_rectangles;
// }




//////
// std::vector<Rectangle> GeometryProcessor::extractUpperRectangles
// (const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) 
// {

// std::vector<Rectangle> upper_rectangles;

// for (const auto& box : bounding_boxes) 
// {
// std::vector<Eigen::Vector3d> corners = box.GetBoxPoints();
// Eigen::Vector3d normal = box.R_.col(2);// Z-direction of the bounding box        
// Eigen::Vector3d center = box.center_;
// std::vector<Eigen::Vector3d> upper_corners;

// for (const auto& corner : corners) 
// {
//     if ((corner - center).dot(normal) > 0) 
//     {
//     // Select upper corners                
//     upper_corners.push_back(corner); }}

// if (upper_corners.size() != 4) 
// {std::cerr << "Error: Could not extract 4 coplanar upper corners from bounding box!" << std::endl;
// continue;}

// // Convert vector to std::array for Rectangle constructor        
// std::array<Eigen::Vector3d, 4> sorted_corners;
// std::copy(upper_corners.begin(), upper_corners.end(), sorted_corners.begin());

// // Create rectangle (sorting happens inside constructor)        
// Rectangle rect(sorted_corners);
// upper_rectangles.push_back(rect);
// // Automatically visualize edges        
// //rect.visualizeEdges()
// ;}
// return upper_rectangles;}



////////////
// std::vector<Rectangle> GeometryProcessor::extractUpperRectangles(
//     const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) 
// {
// std::vector<Rectangle> upper_rectangles;
// for (const auto& box : bounding_boxes) 
// {
// std::vector<Eigen::Vector3d> corners = box.GetBoxPoints();
// // Sort corners based on Z-coordinate (highest points first)        
// std::sort(corners.begin(), corners.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) 
// {
// return a.z() > b.z(); // Sort in descending order                  
// });
// // Take the top 4 points        
// std::vector<Eigen::Vector3d> upper_corners(corners.begin(), corners.begin() + 4);
// // Convert to std::array        
// std::array<Eigen::Vector3d, 4> upper_corners_array;
// std::copy(upper_corners.begin(), upper_corners.end(), upper_corners_array.begin());
// // Sort corners in clockwise order        
// Rectangle rect(upper_corners_array);
// rect.sortCornersClockwise();
// upper_rectangles.push_back(rect);
// rect.visualizeEdges();
// }
// return upper_rectangles;

// }


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


// std::vector<Rectangle> GeometryProcessor::extractUpperRectangles
// (const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) 

// {
// std::vector<Rectangle> upper_rectangles;
// for (const auto& box : bounding_boxes) 
// {
//     std::vector<Eigen::Vector3d> corners = box.GetBoxPoints();
// // Get the normal vector for the Z-axis (which is perpendicular to the top of the box)        
// Eigen::Vector3d normal = box.R_.col(2);
// // Z-direction of the bounding box        
// Eigen::Vector3d center = box.center_;
// // Project the corners onto the plane defined by the normal (horizontal alignment)        
// std::vector<Eigen::Vector3d> projected_corners;

// for (const auto& corner : corners) {
//     Eigen::Vector3d projected_point = projectToPlane(corner, normal, center);// Project the point onto the top plane            
//     projected_corners.push_back(projected_point);}

// // Select the upper corners (after projection)        
// std::vector<Eigen::Vector3d> upper_corners;
// for (const auto& corner : projected_corners) 
// {
//     if ((corner - center).dot(normal) > 0) 
//     {// Only the upper corners                
//     upper_corners.push_back(corner);}}

//     if (upper_corners.size() != 4) 
//     {std::cerr << "Error: Could not extract 4 coplanar upper corners from bounding box!" << std::endl;
//     continue;}
// // Convert vector to std::array        
// std::array<Eigen::Vector3d, 4> upper_corners_array;
// std::copy(upper_corners.begin(), upper_corners.end(), upper_corners_array.begin());

// // Sort the corners in a consistent clockwise order        
// std::array<Eigen::Vector3d, 4> sorted_corners = Rectangle::sortCornersClockwise(upper_corners_array);
// // Create rectangle with sorted corners        
// Rectangle rect(sorted_corners);
// upper_rectangles.push_back(rect);
// // Automatically visualize edges on the point cloud        
// rect.visualizeEdges();}
// return upper_rectangles;}

///////////////////////////////////////

// std::vector<Rectangle> GeometryProcessor::extractUpperRectangles(
//     const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes) 
    
// {
//     std::vector<Rectangle> upper_rectangles;

//     for (const auto& box : bounding_boxes) {
//         std::vector<Eigen::Vector3d> corners = box.GetBoxPoints();
//         Eigen::Vector3d normal = box.R_.col(2);// Z-direction of the bounding box        
//         Eigen::Vector3d center = box.center_;
//         std::vector<Eigen::Vector3d> upper_corners;
        
//     for (const auto& corner : corners) {
//         // Ensure that we're selecting the upper corners            
//         if ((corner - center).dot(normal) > 0) { 
//             upper_corners.push_back(corner);}}

//         if (upper_corners.size() != 4) 
//         {std::cerr << "Error: Could not extract 4 coplanar upper corners from bounding box!" << std::endl;
//             continue;}

// // Project corners onto a horizontal plane if needed (using the normal of the box)        

// for (auto& corner : upper_corners) 
// {
//     corner = projectPointOntoPlane(corner, normal, center); // project onto plane        
//     }
//     // Convert vector to std::array        
//     std::array<Eigen::Vector3d, 4> upper_corners_array; 
//     std::copy(upper_corners.begin(), upper_corners.end(), upper_corners_array.begin());
//     // Sort the corners in a consistent clockwise order        
//     std::array<Eigen::Vector3d, 4> sorted_corners = Rectangle::sortCornersClockwise(upper_corners_array);

//     // Create rectangle with sorted corners        
//     Rectangle rect(sorted_corners);
//     upper_rectangles.push_back(rect);
//     // Automatically visualize edges        
//     rect.visualizeEdges();
//     }
//     return upper_rectangles;}




///////////////////////////////
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


////////////////////////////////

// 














///////////////////////////////////////////////////////////////

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



#ifndef GEOMETRYPROCESSOR_H
#define GEOMETRYPROCESSOR_H


#include <open3d/Open3D.h>
#include <vector>
#include "custom_types.hpp"
#include <Eigen/Dense>
#include <functional> // For std::hash

// Forward declaration of the PointCloud type
using PC_o3d_ptr = std::shared_ptr<open3d::geometry::PointCloud>;








class Rectangle 
{
public: 
// Constructor 
Rectangle(const std::array<Eigen::Vector3d , 4>& corners); 

// Getters for properties 
double getWidth() const; 
double getLength() const; 
Eigen::Vector3d getCenter() const; 
Eigen::Vector3d getNormal() const;
std::array <Eigen::Vector3d , 4> getSortedCorners() const ;
//std::vector<Eigen::Vector3d> getEdges() const; 
std::array <std::pair <Eigen::Vector3d ,Eigen::Vector3d > , 4>  getEdges() const; 

// utility function
//void sortCornersClockwise();
static std::array <Eigen::Vector3d , 4> sortCornersClockwise(const std::array<Eigen::Vector3d , 4>& corners);
void visualizeEdges() const ;

private: 

std::array <Eigen::Vector3d , 4> corners_; 
Eigen::Vector3d center_; 
Eigen::Vector3d normal_; 
//std::vector<Eigen::Vector3d> edges_; 

//helper function to compute center and normal
void computeProperties(); 
};







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


    // 7. Method to getDimensionsOfBoundingBoxes
    std::vector<std::array<double, 3>> getDimensionsOfBoundingBoxes(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes);


    // // 8. Method to generate a list of random rectangles
    // std::vector <Rectangle> generateRandomRectangles (int count);


    // // 9. Method to print rectangles
    // void printRectangles(const std::vector<Rectangle>& rectangles);


    // 10. Method to Extract upper rectangles of bounding boxes
    std::vector <Rectangle> extractUpperRectangles(const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes);

    // 11. Method to visualize rectangles on the original point cloud
    void visualizeRectanglesAndOriginalPc(
    const std::vector<Rectangle>& rectangles,
    const std::shared_ptr<open3d::geometry::PointCloud>& original_pc);


    // 
    Eigen::Vector3d projectToPlane (
        const Eigen::Vector3d& point, const Eigen::Vector3d& normal, 
        const Eigen::Vector3d& point_on_plane);


    //
    Eigen::Vector3d projectPointOntoPlane(const Eigen::Vector3d& point,
                                            const Eigen::Vector3d& normal,
                                            const Eigen::Vector3d& point_on_plane);


    ///
    Eigen::Vector3d projectPointOntoXYPlane (const Eigen::Vector3d& point);

    ///
    Eigen::Vector3d projectToXYPlane(const Eigen::Vector3d& point);


    /// Method to visualize rectangle corner points
    void visualizeRectangleEdgesWithLabels(const std::vector<Rectangle>& rectangles);










};








#endif //GEOMETRYPROCESSOR_H
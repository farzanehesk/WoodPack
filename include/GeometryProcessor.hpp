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

    // function parameters
    // double gap_;       // 3mm gap
    // double max_length_;  // Ensure row is at least 1m long
    // double rotation_angle_; 

    // Constructor and Destructor
    GeometryProcessor();
    ~GeometryProcessor();

    // 1. Method to compute oriented bounding boxes for a vector of point clouds
    std::vector<open3d::geometry::OrientedBoundingBox> computeOrientedBoundingBoxes(
        const std::vector<PC_o3d_ptr>& clusters);
    

    //
    std::vector<open3d::geometry::OrientedBoundingBox> computeMinimalOrientedBoundingBoxes(
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



    // 10. Method to Extract upper rectangles of bounding boxes
    std::vector <Rectangle> extractUpperRectangles(const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes);

    // 11. Method to visualize rectangles; if original_pc is provided, visualize it along with rectangle edges.
    void visualizeRectangles(
        const std::vector<Rectangle>& rectangles,
        const std::shared_ptr<open3d::geometry::PointCloud>& original_pc = nullptr);


    // 12
    Eigen::Vector3d projectToPlane (
        const Eigen::Vector3d& point, const Eigen::Vector3d& normal, 
        const Eigen::Vector3d& point_on_plane);


    //13
    Eigen::Vector3d projectPointOntoPlane(  const Eigen::Vector3d& point,
                                            const Eigen::Vector3d& normal,
                                            const Eigen::Vector3d& point_on_plane);


    ///14
    Eigen::Vector3d projectPointOntoXYPlane (const Eigen::Vector3d& point);

    ///15
    Eigen::Vector3d projectToXYPlane(const Eigen::Vector3d& point);


    /// 16. Method to visualize rectangle corner points
    void visualizeRectangleEdgesWithLabels(const std::vector<Rectangle>& rectangles);


    /// 17. Method to obtain bounding box 3D planes and visualize them
    std::vector<Eigen::Matrix4d> getPlanesFromBoundingBoxes(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes , bool debug);


    ////18.  METHOD TO VISUALIZE PLANES
    void visualizePlanesOnBoundingBoxes(
        const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes,
        const std::vector<Eigen::Matrix4d>& planes,
        const std::shared_ptr<open3d::geometry::PointCloud>& point_cloud);
    


    // 19. Convert Rectangles to Open3D Bounding Boxes
std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> createBoundingBoxFromRectangle(
    const std::vector<Rectangle>& rects, double height);



    // 20.  /// Creates n random rectangles placed randomly on a horizontal surface.
            /// Each rectangle has a fixed length of 0.15m and a random width between 0.08m and 0.20m.
            /// Returns a vector of Rectangle objects.
    std::vector<Rectangle> createRandomRectangles(int n);

    //

    //
    // 22.transform_bounding_box
    void transform_bounding_box(std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bb,
                                const Eigen::Vector3d& translation,
                                const Eigen::Vector3d& rotation_axis,
                                double rotation_angle,
                                const Eigen::Vector3d& scale_center,
                                double scale_factor);

    // 
    // 23. arrangeShingleRow
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> arrangeFirstShingleRow(
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes,
    double gap , double max_row_length , double rotation_angle);

    //
    //24
    void visualize_bounding_boxes(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes);

    // 
    void VisualizeBoundingBoxesAxis(
    const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes);



};








#endif //GEOMETRYPROCESSOR_H
#ifndef GEOMETRYPROCESSOR_H
#define GEOMETRYPROCESSOR_H


#include <open3d/Open3D.h>
#include <vector>
#include "custom_types.hpp"
#include <Eigen/Dense>
#include <functional> // For std::hash

// Forward declaration of the PointCloud type
using PC_o3d_ptr = std::shared_ptr<open3d::geometry::PointCloud>;




// class Edge
// {




// };



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
std::pair<Eigen::Vector3d, Eigen::Vector3d> getRightEdge() const ;
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getTopEdges(const std::vector<Rectangle>& rectangles) ;

// utility function
//void sortCornersClockwise();
static std::array <Eigen::Vector3d , 4> sortCornersClockwise(const std::array<Eigen::Vector3d , 4>& corners);
void visualizeEdges() const ;
void visualizeEdge(const Eigen::Vector3d& start, const Eigen::Vector3d& end) const ;



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
    // std::vector <Rectangle> extractUpperRectangles(
    //     // const std::vector<open3d::geometry::OrientedBoundingBox>& bounding_boxes
    //      const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& bounding_boxes);

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
    std::vector<Rectangle> createRandomRectangles(int n, double fixed_length);

    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> createBoundingBoxes(int n , double fixed_length , bool debug) ;

    //


    // Function to generate a random number between min and max
    double getRandomWidth(double min, double max) ;
    

    //
    // 22.transform_bounding_box
    void transform_bounding_box(std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bb,
                                const Eigen::Vector3d& translation,
                                const Eigen::Vector3d& rotation_axis,
                                double rotation_angle,
                                const Eigen::Vector3d& scale_center,
                                double scale_factor);


    //
    // 23
    void transform_bounding_box_to_plane(
    std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bb,
    const Eigen::Vector3d& target_plane_normal,   // Target plane normal vector
    const Eigen::Vector3d& target_plane_point,    // A point on the target plane
    const Eigen::Vector3d& scale_center,          // Scaling center
    double scale_factor);                          // Scaling factor                            

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




    //
    //void alignBoxToXYPlane(const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bbox) ;

    //
    void alignBoxToXYPlane(std::shared_ptr<open3d::geometry::OrientedBoundingBox>& bbox);

    
    ///
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> selectRandomCandidate(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates) ;



    //


    //
    double calculateTotalWidth(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row);

    //
    std::vector<double> calculateRightEdgeDistancesFromCandidate(
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate_shingle,
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row);

    //
    // bool lineSegmentIntersection(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
    //                               const Eigen::Vector3d& q1, const Eigen::Vector3d& q2) ;


    // //
    // std::shared_ptr<open3d::geometry::OrientedBoundingBox> findIntersectingShingle(
    // const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate_bbox,
    // const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row_bboxes) ;


    ///
    void alignCandidateToFirstBox(
    std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate,
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& first_box);

    ///
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> findNextBestShingles(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates,
    double min_stagger,
    double max_gap,
    double max_length);

    //
    Eigen::Vector3d updateRightEdge(
        const Eigen::Vector3d& current_right_edge,
        const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate , double gap);



    //
    void visualizeShingleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& second_row) ;

    //
    void exportBoundingBoxes(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& boxes,
    const std::string& folder,
    const Eigen::Vector3d& color,  // New parameter for color
    const std::string& prefix = "");

    //
    std::shared_ptr<open3d::geometry::TriangleMesh> CreateMeshFromOrientedBoundingBox(
    const open3d::geometry::OrientedBoundingBox& obb, 
    const Eigen::Vector3d& color);


    ///
    // void visualizeShingleMeshes(
    // const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& combined_rows);

    void visualizeShingleMeshes(
    const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& combined_rows,
    std::shared_ptr<open3d::geometry::PointCloud> point_cloud );


    //
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> 
    arrangeShingleRow(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& target_row,
    double gap,
    double max_length,
    double rotation_angle,
    double vertical_overlap);


    // 
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> alignAndShiftFirstBox(
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& reference_box,
    std::shared_ptr<open3d::geometry::OrientedBoundingBox>& target_box,
    double gap,
    double max_length,
    double rotation_angle) ;


    //
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> alignAndShiftNextBox(
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& last_selected_shingle,
    //const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate,
    const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& candidate,
    double gap);


    //
std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> 
    findNextBestShinglesForMultipleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row, 
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates, 
    int num_rows, double min_stagger, double max_gap, double max_length)    ;


    //
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> 
    arrangeShingleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& first_row,
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& candidates,
    double gap,
    double max_length,
    double rotation_angle) ;


    //
    
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>> 
    arrangeMultipleShingleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& candidate_rows,
    double gap,
    double max_length,
    double rotation_angle,
    double third_fourth_overlap,  // New argument for 3rd & 4th row overlap
    double staggered_vertical_overlap);


    //
    void visualizeAllShingleRows(
    const std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& arranged_rows) ;


    //
std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>
    arrangeMultipleShingleRows(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& reference_row,
    std::vector<std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>>& candidate_rows,  // Non-const reference
    double gap,
    double max_length,
    double rotation_angle,
    double vertical_overlap)  ;
    

    //
    void visualizeShinglePlane(const std::shared_ptr<open3d::geometry::OrientedBoundingBox>& box);


    //
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> copyBoundingBoxes(
    const std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>& input_boxes);




};








#endif //GEOMETRYPROCESSOR_H
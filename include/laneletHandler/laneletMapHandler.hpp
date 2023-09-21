#ifndef LANELET_MAP_HANDLER_HPP_
#define LANELET_MAP_HANDLER_HPP_

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "lane_keep_system/GetLaneletScenario.h"
#include "linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <memory>

class LaneletHandler
{
public:
    LaneletHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nhp);
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_p;
    ros::ServiceServer lanelet_service_;
    ros::Publisher pub_road_lines;

    visualization_msgs::MarkerArray markerArray;

    lanelet::ConstLanelets roadLanelets;
    TrajectoryPoints pathPoints;
    std::string gps_topic;
    std::string lanelet_frame;
    std::string ego_frame;
    Pose2D laneletFramePose;
    bool visualize_path;
    int polyline_count;
    int lastStartPointIdx;
    float nearestNeighborThreshold; // in meters

    PolynomialSubfunctions polynomialSubfunctions;
    CoordinateTransforms coordinateTransforms;

    // Load parameters, load lanelet file, plan path
    bool init();
    
    // Convert Points2D to geometry_msgs::Point
    geometry_msgs::Point convertPoint_CPP2ROS(const Points2D pt);
    // Convert geometry_msgs::Point to Points2D
    Points2D convertPoint_ROS2CPP(const geometry_msgs::Point geoPt);
    // Calculate distance between two given points
    float distanceBetweenPoints(const Points2D a, const Points2D b);
    // Get yaw from GPS data
    float getYawFromPose(const geometry_msgs::PoseStamped& gps_pose);
    // Get nearest point idx to GPS position from path
    int getGPSNNPointIdx(const Points2D& gps_pos);
    // Get point on polynom at the given x value
    Points2D getPointOnPoly(const float x, const lane_keep_system::Polynomial& coeffs);
    // Get color as ROS object
    std_msgs::ColorRGBA getColorObj(const float r, const float g, const float b, const float a);
    // Initialize markers
    void initMarker(visualization_msgs::Marker &m, const std::string frame_id, const std::string ns, const int32_t type, const std_msgs::ColorRGBA color, const float scale);
    // Create scenario
    bool createScenario(
        const geometry_msgs::PoseStamped& gpsPose, 
        const std::vector<float>& nodePtDistances,
        TrajectoryPoints& scenarioFullEGO,
        std::vector<int>& nodePtIndexes);
    // Slice scenario into segments at nodepoints
    Segments sliceScenario(
        const TrajectoryPoints& scenarioFullEGO, 
        const std::vector<int>& nodePtIndexes);
    // Fit polynomials on segments
    std::vector<lane_keep_system::Polynomial> fitPolynomials(const Segments& segments);
    // Get numerical derivative of TrajectoryPoints
    TrajectoryPoints numericalDerivative(const TrajectoryPoints& points);
    // Get moving average of the y values of TrajectoryPoints
    TrajectoryPoints movingAverage(const TrajectoryPoints& points, int windowSize);

    // ROS service callback for calculating polynomial coefficients for the path ahead of the car
    bool LaneletScenarioServiceCallback(
        lane_keep_system::GetLaneletScenario::Request&  req, 
        lane_keep_system::GetLaneletScenario::Response& res);
};

#endif // LANELET_MAP_HANDLER_HPP_
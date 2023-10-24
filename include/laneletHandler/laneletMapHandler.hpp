#ifndef LANELET_MAP_HANDLER_HPP_
#define LANELET_MAP_HANDLER_HPP_

#include <ros/ros.h>
#include <lanelet2_core/LaneletMap.h>

#include "lane_keep_system/GetLaneletScenario.h"
#include "lane_keep_system/Derivatives.h"

#include "linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.hpp"
#include "utilities/rosUtilities.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
    ros::Publisher pub_lanelet_lines;
    ros::Publisher pub_derivatives;

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
    ROSUtilities rosUtilities;

    // Load parameters, load lanelet file, plan path
    bool init();
    
    // Calculate distance between two given points
    float distanceBetweenPoints(const Points2D a, const Points2D b);
    // Get nearest point idx to GPS position from path
    int getGPSNNPointIdx(const Points2D& gps_pos);
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
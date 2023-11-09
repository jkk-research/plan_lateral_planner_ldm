#ifndef LANELET_MAP_HANDLER_HPP_
#define LANELET_MAP_HANDLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <lanelet2_core/LaneletMap.h>

#include "lane_keep_system/srv/get_lanelet_scenario.hpp"
#include "lane_keep_system/msg/derivatives.hpp"

#include "linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.hpp"
#include "utilities/rosUtilities.hpp"

#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <memory>

class LaneletHandler : public rclcpp::Node
{
public:
    LaneletHandler();
private:
    rclcpp::Service<lane_keep_system::srv::GetLaneletScenario>::SharedPtr lanelet_service_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    pub_road_lines;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr    pub_lanelet_lines;
    rclcpp::Publisher<lane_keep_system::msg::Derivatives>::SharedPtr      pub_derivatives;

    visualization_msgs::msg::MarkerArray markerArray;

    lanelet::ConstLanelets roadLanelets;
    TrajectoryPoints       pathPoints;
    Pose2D                 laneletFramePose;
    
    std::string gps_topic;
    std::string lanelet_frame;
    std::string ego_frame;
    bool        visualize_path;
    int         polyline_count;
    int         lastStartPointIdx;
    float       nearestNeighborThreshold; // in meters

    PolynomialSubfunctions polynomialSubfunctions;
    CoordinateTransforms   coordinateTransforms;
    ROSUtilities           rosUtilities;

    // Load parameters, load lanelet file, plan path
    bool init();
    
    // Calculate distance between two given points
    float distanceBetweenPoints(const Points2D a, const Points2D b);
    // Get nearest point idx to GPS position from path
    int getGPSNNPointIdx(const Points2D& gps_pos);
    // Create scenario
    bool createScenario(
        const geometry_msgs::msg::PoseStamped& gpsPose, 
        const std::vector<float>&              nodePtDistances,
        TrajectoryPoints&                      scenarioFullEGO,
        std::vector<int>&                      nodePtIndexes);
    // Slice scenario into segments at nodepoints
    Segments sliceScenario(
        const TrajectoryPoints& scenarioFullEGO, 
        const std::vector<int>& nodePtIndexes);
    // Fit polynomials on segments
    std::vector<lane_keep_system::msg::Polynomial> fitPolynomials(const Segments& segments);
    // Get numerical derivative of TrajectoryPoints
    TrajectoryPoints numericalDerivative(const TrajectoryPoints& points);
    // Get moving average of the y values of TrajectoryPoints
    TrajectoryPoints movingAverage(const TrajectoryPoints& points, int windowSize);

    // ROS service callback for calculating polynomial coefficients for the path ahead of the car
    bool LaneletScenarioServiceCallback(
        const std::shared_ptr<lane_keep_system::srv::GetLaneletScenario::Request>  req,
        const std::shared_ptr<lane_keep_system::srv::GetLaneletScenario::Response> res);
};

#endif // LANELET_MAP_HANDLER_HPP_
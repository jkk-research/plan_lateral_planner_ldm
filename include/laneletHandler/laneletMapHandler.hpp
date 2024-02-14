#ifndef LANELET_MAP_HANDLER_HPP_
#define LANELET_MAP_HANDLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <lanelet2_core/LaneletMap.h>

#include "lane_keep_system/msg/derivatives.hpp"
#include "lane_keep_system/msg/scenario.hpp"

#include "linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.hpp"
#include "utilities/rosUtilities.hpp"

#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <lane_keep_system/msg/point_list.hpp>

#include <vector>
#include <memory>

class LaneletHandler : public rclcpp::Node
{
public:
    LaneletHandler();
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   sub_gps_;
    rclcpp::Publisher<lane_keep_system::msg::Scenario>::SharedPtr      pub_scenario_;
    rclcpp::Publisher<lane_keep_system::msg::Derivatives>::SharedPtr   pub_derivatives_;
    rclcpp::Publisher<lane_keep_system::msg::PointList>::SharedPtr     pub_centerline_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_road_lines_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_lanelet_lines_;
    rclcpp::TimerBase::SharedPtr timer_;

    TrajectoryPoints       pathPoints;
    lanelet::ConstLanelets roadLanelets;

    geometry_msgs::msg::TransformStamped gpsTransform;
    geometry_msgs::msg::PoseStamped      currentGPSMsg;
    visualization_msgs::msg::MarkerArray markerArray;
    
    const uint8_t POLYLINE_COUNT = 3;

    std::string gps_topic;
    std::string lanelet_frame;
    std::string ego_frame;
    std::string gps_frame;
    bool        visualize_path;
    int         lastStartPointIdx;
    float       nearestNeighborThreshold; // in meters
    float       gps_yaw_offset;
    float       nodePtDistances[3];
    float       start_id, end_id;

    PolynomialSubfunctions polynomialSubfunctions;
    CoordinateTransforms   coordinateTransforms;
    ROSUtilities           rosUtilities;

    // Load parameters, load lanelet file, plan path
    bool init();
    
    // ROS callback for gps data
    void gpsCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& gps_msg_);
    // Calculate distance between two given points
    float distanceBetweenPoints(const Points2D a, const Points2D b);
    // Get nearest point idx to GPS position from path
    int getGPSNNPointIdx(const Points2D& gps_pos);
    // Create scenario
    bool createScenario(
        const geometry_msgs::msg::PoseStamped& gpsPose, 
        const float                            nodePtDistances[3],
        TrajectoryPoints&                      scenarioFullEGO,
        std::vector<uint16_t>&                 nodePtIndexes);
    // Slice scenario into segments at nodepoints
    Segments sliceScenario(
        const TrajectoryPoints&      scenarioFullEGO, 
        const std::vector<uint16_t>& nodePtIndexes);
    // Fit polynomials on segments
    std::vector<lane_keep_system::msg::Polynomial> fitPolynomials(const Segments& segments);
    // Get numerical derivative of TrajectoryPoints
    TrajectoryPoints numericalDerivative(const TrajectoryPoints& points);
    // Get moving average of the y values of TrajectoryPoints
    TrajectoryPoints movingAverage(const TrajectoryPoints& points, int windowSize);
    
    // Publish scenario
    void publishScenario();
};

#endif // LANELET_MAP_HANDLER_HPP_
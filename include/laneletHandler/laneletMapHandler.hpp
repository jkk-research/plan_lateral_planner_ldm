#ifndef LANELET_MAP_HANDLER_HPP_
#define LANELET_MAP_HANDLER_HPP_

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>

#include <visualization_msgs/MarkerArray.h>

#include "lane_keep_system/GetLaneletScenario.h"
#include "linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"

#include <tf2_ros/transform_listener.h>

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
    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;

    visualization_msgs::MarkerArray markerArray;
    lanelet::ConstLanelets roadLanelets;
    TrajectoryPoints pathPoints;
    std::string gps_topic;
    std::string lanelet_frame;
    std::string ego_frame;
    int polyline_count;
    bool visualize_path;

    PolynomialSubfunctions polynomialSubfunctions;

    // Load parameters, load lanelet file, plan path
    bool init();
    // Calculate distance between two given points
    double distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b);
    // Get point on polynom at the given x value
    Points2D getPointOnPoly(float x, PolynomialCoeffs coeffs);
    // Get color as ROS object
    std_msgs::ColorRGBA getColorObj(float r, float g, float b, float a);
    // Initialize markers
    void initMarker(visualization_msgs::Marker &m, std::string frame_id, std::string ns, int32_t type, std_msgs::ColorRGBA color);

    // ROS service callback for calculating polynomial coefficients for the path ahead of the car
    bool LaneletScenarioServiceCallback(lane_keep_system::GetLaneletScenario::Request &req, lane_keep_system::GetLaneletScenario::Response &res);
};

#endif // LANELET_MAP_HANDLER_HPP_
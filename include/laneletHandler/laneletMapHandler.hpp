#ifndef LANELET_MAP_HANDLER_HPP_
#define LANELET_MAP_HANDLER_HPP_

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>

#include <visualization_msgs/MarkerArray.h>

#include "lane_keep_system/GetLaneletScenario.h"
#include "linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"

#include <vector>

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
    lanelet::ConstLanelets road_lanelets;
    Rb::Vmc::TrajectoryPoints path_pts;
    std::string lanelet_frame;
    std::string ego_frame;
    double scenario_length;
    int polyline_count;
    bool visualize_path;

    Rb::Vmc::PolynomialSubfunctions polynomialSubfunctions;

    // Load parameters, load lanelet file, plan path
    bool init();
    // Calculate distance between two given points
    double DistanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b);

    // ROS service callback for calculating polynomial coefficients for the path ahead of the car
    bool LaneletScenarioServiceCallback(lane_keep_system::GetLaneletScenario::Request &req, lane_keep_system::GetLaneletScenario::Response &res);
};

#endif // LANELET_MAP_HANDLER_HPP_
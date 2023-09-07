#ifndef TRAJECTORY_CONTROLLER_HPP_
#define TRAJECTORY_CONTROLLER_HPP_

#include <ros/ros.h>

#include "lane_keep_system/GetLaneletScenario.h"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModel/emg_linearDriverModel.hpp"

#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class TrajectoryPlanner
{
public:
    TrajectoryPlanner(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_p_);

    bool runTrajectory();
private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_p;
    ros::ServiceClient client;
    ros::Publisher pub_visualization;
    ros::Subscriber sub_gps;
    geometry_msgs::PoseStamped currentGPSMsg;
    visualization_msgs::MarkerArray markerArray;

    LDMParamIn params;
    LinearDriverModel ldm;
    bool visualize_trajectory;

    ScenarioPolynomials GetScenario();
    // ROS callback for gps topic
    void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps_msg);
};

#endif // TRAJECTORY_CONTROLLER_HPP_
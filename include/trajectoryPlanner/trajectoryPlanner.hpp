#ifndef TRAJECTORY_CONTROLLER_HPP_
#define TRAJECTORY_CONTROLLER_HPP_

#include <ros/ros.h>

#include "lane_keep_system/GetLaneletScenario.h"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModel/emg_linearDriverModel.hpp"
#include "utilities/rosUtilities.hpp"

#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleStatus.h>

class TrajectoryPlanner
{
public:
    TrajectoryPlanner(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_p_);

    bool runTrajectory();
private:
    ros::NodeHandle    nh;
    ros::NodeHandle    nh_p;
    ros::ServiceClient client;
    ros::Subscriber    sub_gps;
    ros::Publisher     pub_visualization;
    ros::Publisher     pub_waypoints;
    ros::Publisher     pub_currentPose;

    geometry_msgs::PoseStamped      currentGPSMsg;
    visualization_msgs::MarkerArray markerArray;
    autoware_msgs::Lane             waypoints;

    LDMParamIn        params;
    LinearDriverModel ldm;
    std::string       lanelet_frame;
    float             gps_yaw_offset;
    bool              visualize_trajectory;
    bool              start_on_corridor;
    float             targetSpeed;

    ROSUtilities rosUtilities;

    // ROS callback for gps data
    void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps_msg);
    // Get current scenario by calling the lanelet handler service
    ScenarioPolynomials getScenario();
    // Visualize output
    void visualizeOutput(const TrajectoryOutput& trajectoryOutput);
    // Publish the output
    void publishOutput(const PolynomialCoeffsThreeSegments& coeffs, const Pose2D& egoPose);
};

#endif // TRAJECTORY_CONTROLLER_HPP_
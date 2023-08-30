#ifndef TRAJECTORY_CONTROLLER_HPP_
#define TRAJECTORY_CONTROLLER_HPP_

#include <ros/ros.h>

#include "lane_keep_system/GetLaneletScenario.h"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModel/emg_linearDriverModel.hpp"

#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TrajectoryPlanner
{
public:
    TrajectoryPlanner(const ros::NodeHandle &nh);
private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    LinearDriverModel ldm;

    ScenarioPolynomials GetScenario();
};

#endif // TRAJECTORY_CONTROLLER_HPP_
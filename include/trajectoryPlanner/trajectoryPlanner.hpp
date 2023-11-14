#ifndef TRAJECTORY_CONTROLLER_HPP_
#define TRAJECTORY_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lane_keep_system/srv/get_lanelet_scenario.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModel/emg_linearDriverModel.hpp"
#include "utilities/rosUtilities.hpp"

#include <lane_keep_system/msg/scenario.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <memory>

class TrajectoryPlanner : public rclcpp::Node
{
public:
    TrajectoryPlanner();

    // Initialize class
    bool init();
private:
    rclcpp::Subscription<lane_keep_system::msg::Scenario>::SharedPtr           sub_scenario_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr         pub_visualization_;
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                      pub_odometry_;
    rclcpp::TimerBase::SharedPtr timer_;

    visualization_msgs::msg::MarkerArray         markerArray;
    autoware_auto_planning_msgs::msg::Trajectory trajectory;
    lane_keep_system::msg::Scenario              currentScenario;

    LDMParamIn        params;
    LinearDriverModel ldm;
    std::string       lanelet_frame;
    std::string       gps_topic;
    float             gps_yaw_offset;
    bool              visualize_trajectory;
    bool              start_on_corridor;
    float             targetSpeed;

    ROSUtilities rosUtilities;

    // Run the planning cycle
    bool runTrajectory();

    // Scenario callback
    void scenarioCallback(const std::shared_ptr<const lane_keep_system::msg::Scenario>& msg_);
    // Get the scenario
    ScenarioPolynomials getScenario();
    // Visualize output
    void visualizeOutput(const TrajectoryOutput& trajectoryOutput);
    // Publish the output
    void publishOutput(const PolynomialCoeffsThreeSegments& coeffs);
};

#endif // TRAJECTORY_CONTROLLER_HPP_
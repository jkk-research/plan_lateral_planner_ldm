#ifndef TRAJECTORY_CONTROLLER_HPP_
#define TRAJECTORY_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModel/emg_linearDriverModel.hpp"
#include "linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.hpp"
#include "utilities/rosUtilities.hpp"

#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <lane_keep_system/msg/scenario.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <lane_keep_system/msg/point_list.hpp>
#include <lane_keep_system/msg/trajectory.hpp>

#include <vector>
#include <memory>

class TrajectoryPlanner : public rclcpp::Node
{
public:
    TrajectoryPlanner();

    // Initialize class
    bool init();
private:
    rclcpp::Subscription<lane_keep_system::msg::Scenario>::SharedPtr                 sub_scenario_;
    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_;
    rclcpp::Subscription<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr               sub_accel;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr            pub_visualization_;
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr    pub_trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                         pub_odometry_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr    pub_gear_;
    rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr  pub_accel_;
    rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr pub_operation_mode_state_;
    rclcpp::Publisher<lane_keep_system::msg::Trajectory>::SharedPtr               pub_debug_trajectory_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped gpsTransform;
    visualization_msgs::msg::MarkerArray         markerArray;
    autoware_auto_planning_msgs::msg::Trajectory trajectory;
    lane_keep_system::msg::Scenario              currentScenario;

    CoordinateTransforms   coordinateTransforms;

    LDMParamIn        params;
    LinearDriverModel ldm;
    std::string       lanelet_frame;
    float             targetSpeed;
    float             currentSpeed;
    float             currentVelocity;
    float             currentAcceleration;
    bool              visualize_trajectory;
    bool              start_on_corridor;
    bool              global_path;

    ROSUtilities rosUtilities;

    // Run the planning cycle
    bool runTrajectory();

    // Scenario callback
    void scenarioCallback(const std::shared_ptr<const lane_keep_system::msg::Scenario>& msg_);
    // Velocity callback
    void velocity_callback(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::VelocityReport>& msg_);
    // acceleration callback
    void accel_callback(const std::shared_ptr<const pacmod3_msgs::msg::SystemRptFloat>& msg_);
    
    // Get the scenario
    ScenarioPolynomials getScenario();
    // Visualize output
    void visualizeOutput(const TrajectoryOutput& trajectoryOutput);
    // Publish the output
    void publishOutput(const TrajectoryOutput& coeffs, const Pose2D& egoPose);
};

#endif // TRAJECTORY_CONTROLLER_HPP_
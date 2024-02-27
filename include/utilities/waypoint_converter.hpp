#ifndef WAYPOINT_CONVERTER_HPP
#define WAYPOINT_CONVERTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

class WaypointConverter : public rclcpp::Node
{
public:
    WaypointConverter();

private:
    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_waypoints_;

    void trajectoryCallback(const std::shared_ptr<const autoware_auto_planning_msgs::msg::Trajectory>& msg_);
};


#endif // WAYPOINT_CONVERTER_HPP
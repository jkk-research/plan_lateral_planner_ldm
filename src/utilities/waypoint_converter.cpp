#include "utilities/waypoint_converter.hpp"

WaypointConverter::WaypointConverter() : Node("waypoint_converter")
{
    sub_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        "/planning/scenario_planning/trajectory", 10, std::bind(&WaypointConverter::trajectoryCallback, this, std::placeholders::_1));
    pub_waypoints_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypointarray", 10);
}

void WaypointConverter::trajectoryCallback(const std::shared_ptr<const autoware_auto_planning_msgs::msg::Trajectory>& msg_)
{
    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header = msg_->header;
    for (auto point : msg_->points)
    {
        geometry_msgs::msg::Pose pose;
        pose.position = point.pose.position;
        pose.orientation = point.pose.orientation;
        waypoints.poses.push_back(pose);
    }
    pub_waypoints_->publish(waypoints);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointConverter>());
    rclcpp::shutdown();
    return 0;
}
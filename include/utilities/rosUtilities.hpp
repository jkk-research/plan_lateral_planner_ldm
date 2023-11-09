#ifndef ROS_UTILITIES_HPP
#define ROS_UTILITIES_HPP

#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>



class ROSUtilities
{
public:
    // Get ROS color object from RBGA parameters
    std_msgs::msg::ColorRGBA getColorObj(float r, float g, float b, float a);
    // Initialize marker object
    void initMarker(
        visualization_msgs::msg::Marker &m, 
        const std::string               frame_id, 
        const std::string               ns, 
        const int32_t                   type, 
        const std_msgs::msg::ColorRGBA  color, 
        const float                     scale=0.4f);
    // Get geometry_msgs point on polynomial
    geometry_msgs::msg::Point getROSPointOnPoly(float x, const PolynomialCoeffs& coeffs);
    // Convert Points2D to geometry_msgs::Point
    geometry_msgs::msg::Point convertPoint_CPP2ROS(const Points2D pt);
    // Convert geometry_msgs::Point to Points2D
    Points2D convertPoint_ROS2CPP(const geometry_msgs::msg::Point geoPt);
    // Get yaw rotation from gps pose
    float getYawFromPose(const geometry_msgs::msg::PoseStamped& gps_pose);
    // Get Pose2D object from gps pose
    Pose2D getEgoPose(const geometry_msgs::msg::PoseStamped& gps_pose);
};

#endif // ROS_UTILITIES_HPP
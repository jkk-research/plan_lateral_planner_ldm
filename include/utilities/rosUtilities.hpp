#ifndef ROS_TRANSFORMS_HPP
#define ROS_TRANSFORMS_HPP

#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>



class ROSUtilities
{
public:
    // Get ROS color object from RBGA parameters
    std_msgs::ColorRGBA getColorObj(float r, float g, float b, float a);
    // Initialize marker object
    void initMarker(
        visualization_msgs::Marker &m, 
        const std::string          frame_id, 
        const std::string          ns, 
        const int32_t              type, 
        const std_msgs::ColorRGBA  color, 
        const float                scale=0.4f);
    // Get geometry_msgs point on polynomial
    geometry_msgs::Point getROSPointOnPoly(float x, const PolynomialCoeffs& coeffs);
    // Convert Points2D to geometry_msgs::Point
    geometry_msgs::Point convertPoint_CPP2ROS(const Points2D pt);
    // Convert geometry_msgs::Point to Points2D
    Points2D convertPoint_ROS2CPP(const geometry_msgs::Point geoPt);
    // Get yaw rotation from gps pose
    float getYawFromPose(const geometry_msgs::PoseStamped& gps_pose);
    // Get Pose2D object from gps pose
    Pose2D getEgoPose(const geometry_msgs::PoseStamped& gps_pose);
};

#endif // ROS_TRANSFORMS_HPP
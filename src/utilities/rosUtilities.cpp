#include "utilities/rosUtilities.hpp"


void ROSUtilities::initMarker(
    visualization_msgs::msg::Marker &m, 
    const std::string               frame_id, 
    const std::string               ns, 
    const int32_t                   type, 
    const std_msgs::msg::ColorRGBA  color, 
    const float                     scale)
{
    m.header.frame_id = frame_id;
    m.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    m.lifetime = rclcpp::Duration(0, 0);
    m.ns = ns;
    m.type = type;
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.color.r = color.r;
    m.color.g = color.g;
    m.color.b = color.b;
    m.color.a = color.a;
}

std_msgs::msg::ColorRGBA ROSUtilities::getColorObj(float r, float g, float b, float a)
{
	std_msgs::msg::ColorRGBA c;
    c.r = r;
	c.g = g;
	c.b = b;
	c.a = a;
    return c;
}

geometry_msgs::msg::Point ROSUtilities::convertPoint_CPP2ROS(const Points2D pt)
{
    geometry_msgs::msg::Point geoPt;
    geoPt.x = pt.x;
    geoPt.y = pt.y;
    return geoPt;
}

Points2D ROSUtilities::convertPoint_ROS2CPP(const geometry_msgs::msg::Point geoPt)
{
    Points2D pt;
    pt.x = geoPt.x;
    pt.y = geoPt.y;
    return pt;
}

float ROSUtilities::getYawFromPose(const geometry_msgs::msg::PoseStamped& gps_pose)
{
    tf2::Quaternion q;
    tf2::fromMsg(gps_pose.pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

Pose2D ROSUtilities::getEgoPose(const geometry_msgs::msg::PoseStamped& gps_pose)
{
    Pose2D egoPose;

    egoPose.Pose2DCoordinates.x = gps_pose.pose.position.x;
    egoPose.Pose2DCoordinates.y = gps_pose.pose.position.y;
    egoPose.Pose2DTheta = getYawFromPose(gps_pose);

    return egoPose;
}

geometry_msgs::msg::Point ROSUtilities::getROSPointOnPoly(float x, const PolynomialCoeffs& coeffs)
{
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = coeffs.c0 + coeffs.c1 * x + coeffs.c2 * pow(x, 2) + coeffs.c3 * pow(x, 3);
    return pt;
}

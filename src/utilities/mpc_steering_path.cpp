#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <autoware_msgs/ControlCommandStamped.h>

double wheelbase = 2.7;
double steering_angle; 
ros::Publisher marker_pub;
std::string ego_frame;

void ctrlCallback(const autoware_msgs::ControlCommandStamped &ctrl_msg)
{
    steering_angle = ctrl_msg.cmd.steering_angle;
}

void loop()
{
    visualization_msgs::Marker steer_marker;
    steer_marker.header.frame_id = ego_frame;
    steer_marker.header.stamp = ros::Time::now();
    steer_marker.ns = "steering_path";
    steer_marker.id = 0;
    steer_marker.type = steer_marker.LINE_STRIP;
    steer_marker.action = visualization_msgs::Marker::ADD;
    steer_marker.pose.position.x = 0;
    steer_marker.pose.position.y = 0;
    steer_marker.pose.position.z = 0;
    steer_marker.pose.orientation.x = 0.0;
    steer_marker.pose.orientation.y = 0.0;
    steer_marker.pose.orientation.z = 0.0;
    steer_marker.pose.orientation.w = 1.0;
    steer_marker.scale.x = 0.6;
    steer_marker.color.r = 0.94f; steer_marker.color.g = 0.83f; steer_marker.color.b = 0.07f;
    steer_marker.color.a = 0.8;
    steer_marker.lifetime = ros::Duration();
    double marker_pos_x = 0.0, marker_pos_y = 0.0, theta = 0.0;
    for (int i = 0; i < 100; i++)
    {
        marker_pos_x += 0.01 * 10 * cos(theta);
        marker_pos_y += 0.01 * 10 * sin(theta);
        theta += 0.01 * 10 / wheelbase * tan(steering_angle);
        geometry_msgs::Point p;
        p.x = marker_pos_x;
        p.y = marker_pos_y;
        p.z = 1;
        steer_marker.points.push_back(p);
    }
    marker_pub.publish(steer_marker);
    steer_marker.points.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_steering_path");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
    nh_p.getParam("lanelet_frame", ego_frame);
    ros::Subscriber sub_steer = nh.subscribe("/ctrl_raw", 1, ctrlCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("mpc_steering_path", 1);
    ros::Rate rate(20); // ROS Rate at 20Hz
    while (ros::ok()) {
        loop();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
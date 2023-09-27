#include "trajectoryPlanner/trajectoryPlanner.hpp"


TrajectoryPlanner::TrajectoryPlanner(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_p_) : nh(nh_), nh_p(nh_p_)
{
    // set parameters
    std::vector<float> driverParams(21);
    std::vector<float> nodePtDistances(3);
    nh.getParam(  "trajectory_planner/P",                  driverParams);
    nh.getParam(  "trajectory_planner/replan_cycle",       params.replanCycle);
    nh.getParam(  "trajectory_planner/nodePointDistances", nodePtDistances);
    nh.getParam(  "trajectory_planner/target_speed",       targetSpeed);
    nh_p.getParam("lanelet_frame",                         lanelet_frame);
    nh_p.getParam("gps_yaw_offset",                        gps_yaw_offset);
    nh_p.getParam("visualize",                             visualize_trajectory);

    for (uint8_t i = 0; i < 21; i++)
        params.P[i] = driverParams[i];

    for (uint8_t i = 0; i < 3; i++)
        params.P_nodePointDistances[i] = nodePtDistances[i];

    vehicleStatus.header.frame_id = "base_link";
    vehicleStatus.drivemode = 0;

    // subscribers
    ROS_INFO("Waiting for GPS data on /gps/duro/current_pose");
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/gps/duro/current_pose");
    
    sub_gps =           nh.subscribe("/gps/duro/current_pose", 1, &TrajectoryPlanner::gpsCallback, this);
    sub_vehicleSpeed =  nh.subscribe("/vehicle_speed_kmph",    1, &TrajectoryPlanner::vehicleSpeedCallback, this);
    sub_wheelAngle =    nh.subscribe("/wheel_angle_deg",       1, &TrajectoryPlanner::wheelAngleCallback, this);

    // publishers
    pub_visualization = nh.advertise<visualization_msgs::MarkerArray>("ldm_path", 1, true);
    pub_vehicleStatus = nh.advertise<autoware_msgs::VehicleStatus>("vehicle_status", 1, false);
    pub_waypoints =     nh.advertise<autoware_msgs::Lane>("mpc_waypoints", 1, false);
    pub_currentPose =   nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1, false);

    // connect to lanelet map service
    ros::service::waitForService("/get_lanelet_scenario");
    client = nh.serviceClient<lane_keep_system::GetLaneletScenario>("/get_lanelet_scenario", true);
}

void initMarker(visualization_msgs::Marker &m, std::string frame_id, std::string ns, int32_t type, std_msgs::ColorRGBA color, float scale=0.4)
{
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.lifetime = ros::Duration(0);
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

std_msgs::ColorRGBA getColorObj(float r, float g, float b, float a)
{
	std_msgs::ColorRGBA c;
    c.r = r;
	c.g = g;
	c.b = b;
	c.a = a;
    return c;
}

Pose2D TrajectoryPlanner::getEgoPose()
{
    Pose2D egoPose;
    geometry_msgs::PoseStamped gps_pose = currentGPSMsg;

    egoPose.Pose2DCoordinates.x = gps_pose.pose.position.x;
    egoPose.Pose2DCoordinates.y = gps_pose.pose.position.y;

    tf2::Quaternion q(
        gps_pose.pose.orientation.x,
        gps_pose.pose.orientation.y,
        gps_pose.pose.orientation.z,
        gps_pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    egoPose.Pose2DTheta = yaw;

    return egoPose;
}

ScenarioPolynomials TrajectoryPlanner::getScenario()
{
    ScenarioPolynomials sp;

    lane_keep_system::GetLaneletScenario srv;
    srv.request.nodePointDistances.push_back(params.P_nodePointDistances[0]);
    srv.request.nodePointDistances.push_back(params.P_nodePointDistances[1]);
    srv.request.nodePointDistances.push_back(params.P_nodePointDistances[2]);
    srv.request.gps = currentGPSMsg;
    
    client.call(srv);
    
    sp.coeffs.reserve(srv.response.coefficients.size());
    sp.kappaNominal.reserve(srv.response.kappa.size());

    for (auto coeffs: srv.response.coefficients)
    {
        PolynomialCoeffs polyCoeffs;
        polyCoeffs.c0 = coeffs.c0;
        polyCoeffs.c1 = coeffs.c1;
        polyCoeffs.c2 = coeffs.c2;
        polyCoeffs.c3 = coeffs.c3;
        sp.coeffs.push_back(polyCoeffs);
    }
    for (auto k: srv.response.kappa)
    {
        sp.kappaNominal.push_back(k);
    }

    return sp;
}

geometry_msgs::Point TrajectoryPlanner::getROSPointOnPoly(float x, const PolynomialCoeffs& coeffs)
{
    geometry_msgs::Point pt;
    pt.x = x;
    pt.y = coeffs.c0 + coeffs.c1 * x + coeffs.c2 * pow(x, 2) + coeffs.c3 * pow(x, 3);
    return pt;
}

void TrajectoryPlanner::visualizeOutput(const TrajectoryOutput& trajectoryOutput)
{
    markerArray.markers.clear();

    PolynomialCoeffsThreeSegments pcts = trajectoryOutput.segmentCoeffs;
    int colors[4]{1,0,0,0};
    for (int i = 0; i < 3; i++)
    {
        visualization_msgs::Marker mark;
        initMarker(mark, "map_zala_0", "segment "+std::to_string(i), visualization_msgs::Marker::LINE_STRIP, getColorObj(colors[0], colors[1], colors[2], 1));
        colors[i] = 0;
        colors[i+1] = 1;
        
        for (float j = pcts.sectionBorderStart[i]; j <= pcts.sectionBorderEnd[i]; j++)
        {
            mark.points.push_back(getROSPointOnPoly(j, pcts.segmentCoeffs[i]));
        }

        mark.points.push_back(getROSPointOnPoly(pcts.sectionBorderEnd[i], pcts.segmentCoeffs[i]));
        markerArray.markers.push_back(mark);
    }
    
    visualization_msgs::Marker mark;
    initMarker(mark, "map_zala_0", "nodePts", visualization_msgs::Marker::POINTS, getColorObj(0, 1, 1, 1), 0.8);
    for (int i = 0; i < 4; i++)
    {
        geometry_msgs::Point p;
        p.x = trajectoryOutput.nodePts.nodePointsCoordinates[i].x;
        p.y = trajectoryOutput.nodePts.nodePointsCoordinates[i].y;
        p.z = 0.5;
        mark.points.push_back(p);
    }
    markerArray.markers.push_back(mark);
    
    visualization_msgs::Marker ego_mark;
    initMarker(ego_mark, "map_zala_0", "ego", visualization_msgs::Marker::POINTS, getColorObj(1, 0, 1, 1), 1);
    ego_mark.scale.x = 4;
    ego_mark.scale.y = 1.6;
    geometry_msgs::Point ego_point;
    ego_point.x = 0;
    ego_point.y = 0;
    ego_point.z = 1.5;
    ego_mark.points.push_back(ego_point);
    markerArray.markers.push_back(ego_mark);

    pub_visualization.publish(markerArray);
}

void TrajectoryPlanner::publishOutput(const PolynomialCoeffsThreeSegments& segmentCoeffs, const Pose2D& egoPose)
{
    // vehicle_status
    vehicleStatus.header.stamp = ros::Time::now();
    vehicleStatus.speed = vehicleSpeed;
    vehicleStatus.angle = wheelAngle;

    pub_vehicleStatus.publish(vehicleStatus);

    // base_waypoints
    autoware_msgs::Lane lane;
    lane.header.stamp = ros::Time::now();
    lane.header.frame_id = lanelet_frame;

    int coeffsIdx = 0;
    float x = segmentCoeffs.sectionBorderStart[0];
    for (; x < params.P_nodePointDistances[2]; x++)
    {
        PolynomialCoeffs coeffs = segmentCoeffs.segmentCoeffs[coeffsIdx];

        autoware_msgs::Waypoint wp;
        // position
        wp.pose.pose.position = getROSPointOnPoly(x, coeffs);
        
        // orientation
        float yaw = atan(coeffs.c1 + 2 * coeffs.c2 * x + 3 * coeffs.c3 * pow(x,2));
        tf2::Quaternion quat_tf;
        quat_tf.setEuler(yaw, 0, 0);
        quat_tf.normalize();
        wp.pose.pose.orientation = tf2::toMsg(quat_tf);
        
        // speed
        wp.twist.twist.linear.x = targetSpeed;

        lane.waypoints.push_back(wp);

        if (coeffsIdx > segmentCoeffs.sectionBorderEnd[coeffsIdx] && coeffsIdx < 2)
            coeffsIdx++;
    }

    pub_waypoints.publish(lane);
    
    // current_pose
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = lanelet_frame;

    pub_currentPose.publish(current_pose);
}


bool TrajectoryPlanner::runTrajectory()
{
    ScenarioPolynomials sp = getScenario();
    
    Pose2D egoPose = getEgoPose();

    TrajectoryOutput trajectoryOutput = ldm.runCoeffsLite(
        sp,
        egoPose,
        params
    );

    publishOutput(trajectoryOutput.segmentCoeffs, egoPose);

    // visualization
    if (visualize_trajectory)
        visualizeOutput(trajectoryOutput);

    return true;
}

void TrajectoryPlanner::gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps_msg)
{
    tf2::Quaternion q;
    tf2::fromMsg(gps_msg->pose.orientation, q);
    
    tf2::Quaternion q_rotation;
    q_rotation.setRPY(0, 0, gps_yaw_offset);

    tf2::Quaternion q_rotated = q_rotation * q;
    q_rotated.normalize();

    currentGPSMsg = *gps_msg;
    currentGPSMsg.pose.orientation = tf2::toMsg(q_rotated);
    
}

void TrajectoryPlanner::vehicleSpeedCallback(const std_msgs::Float32::ConstPtr& speed_msg)
{
    vehicleSpeed = speed_msg->data;
}

void TrajectoryPlanner::wheelAngleCallback(const std_msgs::Float32::ConstPtr& angle_msg)
{
    wheelAngle = angle_msg->data;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_planner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    TrajectoryPlanner trajectoryPlanner(nh, nh_p);
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    int target_rate;
    nh.getParam("trajectory_planner/target_rate", target_rate);

    ros::Rate rate(target_rate);

    while (ros::ok)
    {
        ros::spinOnce();
        trajectoryPlanner.runTrajectory();
        rate.sleep();
    }

    return 0;
}
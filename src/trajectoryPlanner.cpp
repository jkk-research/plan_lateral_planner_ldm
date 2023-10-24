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
    nh.getParam(  "trajectory_planner/start_on_corridor",  start_on_corridor);
    nh_p.getParam("lanelet_frame",                         lanelet_frame);
    nh_p.getParam("gps_yaw_offset",                        gps_yaw_offset);
    nh_p.getParam("visualize",                             visualize_trajectory);

    for (uint8_t i = 0; i < 21; i++)
        params.P[i] = driverParams[i];

    for (uint8_t i = 0; i < 3; i++)
        params.P_nodePointDistances[i] = nodePtDistances[i];

    // subscribers
    ROS_INFO("Waiting for GPS data on /gps/duro/current_pose");
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/gps/duro/current_pose");
    
    sub_gps =           nh.subscribe("/gps/duro/current_pose", 1, &TrajectoryPlanner::gpsCallback, this);

    // publishers
    pub_visualization = nh.advertise<visualization_msgs::MarkerArray>("ldm_path", 1, true);
    pub_waypoints =     nh.advertise<autoware_msgs::Lane>("mpc_waypoints", 1, false);
    pub_currentPose =   nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1, false);

    // connect to lanelet map service
    ros::service::waitForService("/get_lanelet_scenario");
    client = nh.serviceClient<lane_keep_system::GetLaneletScenario>("/get_lanelet_scenario", true);

    if (start_on_corridor)
    {
        ros::spinOnce();
        ScenarioPolynomials sp = getScenario();
        ldm.initCoeffs(sp);
    }

    ROS_INFO("Trajectory planner initialized");
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

void TrajectoryPlanner::visualizeOutput(const TrajectoryOutput& trajectoryOutput)
{
    markerArray.markers.clear();

    PolynomialCoeffsThreeSegments pcts = trajectoryOutput.segmentCoeffs;
    int colors[4]{1,0,0,0};
    for (int i = 0; i < 3; i++)
    {
        visualization_msgs::Marker mark;
        rosUtilities.initMarker(mark, "map_zala_0", "segment "+std::to_string(i), visualization_msgs::Marker::LINE_STRIP, rosUtilities.getColorObj(colors[0], colors[1], colors[2], 1));
        colors[i] = 0;
        colors[i+1] = 1;
        
        for (float j = pcts.sectionBorderStart[i]; j <= pcts.sectionBorderEnd[i]; j++)
        {
            mark.points.push_back(rosUtilities.getROSPointOnPoly(j, pcts.segmentCoeffs[i]));
        }

        mark.points.push_back(rosUtilities.getROSPointOnPoly(pcts.sectionBorderEnd[i], pcts.segmentCoeffs[i]));
        markerArray.markers.push_back(mark);
    }
    
    visualization_msgs::Marker mark;
    rosUtilities.initMarker(mark, "map_zala_0", "nodePts", visualization_msgs::Marker::POINTS, rosUtilities.getColorObj(0, 1, 1, 1), 0.8);
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
    rosUtilities.initMarker(ego_mark, "map_zala_0", "ego", visualization_msgs::Marker::POINTS, rosUtilities.getColorObj(1, 0, 1, 1), 1);
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
        wp.pose.pose.position = rosUtilities.getROSPointOnPoly(x, coeffs);
        
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
    
    Pose2D egoPose = rosUtilities.getEgoPose(currentGPSMsg);

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
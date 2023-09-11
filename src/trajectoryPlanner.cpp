#include "trajectoryController/trajectoryPlanner.hpp"

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

void TrajectoryPlanner::gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps_msg)
{
    currentGPSMsg = *gps_msg;
}

TrajectoryPlanner::TrajectoryPlanner(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_p_) : nh(nh_), nh_p(nh_p_)
{
    // set parameters
    std::vector<float> driverParams(21);
    nh.getParam("trajectory_planner/P", driverParams);
    nh.getParam("trajectory_planner/replan_cycle", params.replanCycle);
    nh_p.getParam("visualize", visualize_trajectory);

    for (uint8_t i = 0; i < 21; i++)
    {
        params.P[i] = driverParams[i];
    }
    
    // subscribe to gps
    ROS_INFO("Waiting for GPS data on /gps/duro/current_pose");
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/gps/duro/current_pose");
    sub_gps = nh.subscribe("/gps/duro/current_pose", 1, &TrajectoryPlanner::gpsCallback, this);

    // connect to lanelet map service
    ros::service::waitForService("/get_lanelet_scenario");
    client = nh.serviceClient<lane_keep_system::GetLaneletScenario>("/get_lanelet_scenario", true);
    
    pub_visualization = nh.advertise<visualization_msgs::MarkerArray>("ldm_path", 1, true);
}

bool TrajectoryPlanner::runTrajectory()
{
    ScenarioPolynomials sp = GetScenario();
    
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

    TrajectoryOutput to = ldm.runCoeffsLite(
        sp,
        egoPose,
        params
    );
    PolynomialCoeffsThreeSegments pcts = to.pcts;

    //ROS_INFO("KappaNominal: %f ; %f ; %f", sp.kappaNominal[0], sp.kappaNominal[1], sp.kappaNominal[2]);

    if (visualize_trajectory)
    {
        markerArray.markers.clear();
        int colors[4]{1,0,0,0};
        for (int i = 0; i < 3; i++)
        {
            // ROS_INFO("%i. coeff: %f - %f - %f - %f", i+1, pcts.segmentCoeffs[i].c0, pcts.segmentCoeffs[i].c1, pcts.segmentCoeffs[i].c2, pcts.segmentCoeffs[i].c3);

            visualization_msgs::Marker mark;
            initMarker(mark, "map_zala_0", "segment "+std::to_string(i), visualization_msgs::Marker::LINE_STRIP, getColorObj(colors[0], colors[1], colors[2], 1));
            colors[i] = 0;
            colors[i+1] = 1;
            for (int j = pcts.sectionBorderStart[i]; j <= pcts.sectionBorderEnd[i]; j++)
            {
                geometry_msgs::Point p;
                p.x = j;
                p.y = pcts.segmentCoeffs[i].c0 + pcts.segmentCoeffs[i].c1 * j + pcts.segmentCoeffs[i].c2 * j * j + pcts.segmentCoeffs[i].c3 * j * j * j;
                mark.points.push_back(p);
            }
            markerArray.markers.push_back(mark);
        }
        ROS_INFO("-----");
        visualization_msgs::Marker mark;
        initMarker(mark, "map_zala_0", "nodePts", visualization_msgs::Marker::POINTS, getColorObj(0, 1, 1, 1), 1);
        for (int i = 0; i < 4; i++)
        {
            geometry_msgs::Point p;
            p.x = to.np.nodePointsCoordinates[i].x;
            p.y = to.np.nodePointsCoordinates[i].y;
            p.z = 1;
            mark.points.push_back(p);
            // ROS_INFO("%i. nodePt: %f - %f", i, p.x, p.y);
        }
        markerArray.markers.push_back(mark);
        
        visualization_msgs::Marker ego_mark;
        initMarker(ego_mark, "map_zala_0", "ego", visualization_msgs::Marker::POINTS, getColorObj(1, 0, 1, 1), 1);
        ego_mark.scale.x = 4;
        ego_mark.scale.y = 1.6;
        geometry_msgs::Point ego_point;
        ego_point.x = 0;
        ego_point.y = 0;
        ego_mark.points.push_back(ego_point);
        markerArray.markers.push_back(ego_mark);
        
        ROS_INFO("=====");

        pub_visualization.publish(markerArray);
    }

    return true;
}

ScenarioPolynomials TrajectoryPlanner::GetScenario()
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

    for (auto c: srv.response.coefficients)
    {
        sp.coeffs.push_back(c);
    }
    for (auto k: srv.response.kappa)
    {
        sp.kappaNominal.push_back(k);
    }

    return sp;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_planner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    TrajectoryPlanner trajectoryPlanner(nh, nh_p);
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    ros::Rate rate(50);

    while (ros::ok)
    {
        ros::Time t0 = ros::Time::now();
        trajectoryPlanner.runTrajectory();
        ros::Time t1 = ros::Time::now();
        ROS_INFO("Time: %f", (t1-t0).toSec());

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
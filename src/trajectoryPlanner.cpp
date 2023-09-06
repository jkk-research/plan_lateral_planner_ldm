#include "trajectoryController/trajectoryPlanner.hpp"

void initMarker(visualization_msgs::Marker &m, std::string frame_id, std::string ns, int32_t type, std_msgs::ColorRGBA color)
{
    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.lifetime = ros::Duration(0);
    m.ns = ns;
    m.type = type;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
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

TrajectoryPlanner::TrajectoryPlanner(const ros::NodeHandle &nh_) : nh(nh_)
{
    // subscribe to gps
    sub_gps = nh.subscribe("/gps/duro/current_pose", 1, &TrajectoryPlanner::gpsCallback, this);
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/gps/duro/current_pose");

    client = nh.serviceClient<lane_keep_system::GetLaneletScenario>("/get_lanelet_scenario", true);
    client.waitForExistence();

    pub_visualization = nh.advertise<visualization_msgs::MarkerArray>("ldm_path", 1, true);

    // TODO: init params
}

bool TrajectoryPlanner::runTrajectory()
{
    ScenarioPolynomials sp = GetScenario();
    
    Pose2D egoPose;
    geometry_msgs::PoseStamped gps_pose = currentGPSMsg;
    egoPose.Pose2DCoordinates.x = gps_pose.pose.position.x;
    egoPose.Pose2DCoordinates.y = gps_pose.pose.position.y;
    egoPose.Pose2DCoordinates.z = gps_pose.pose.position.z;

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

    PolynomialCoeffsThreeSegments pcts = ldm.runCoeffsLite(
        sp,
        egoPose,
        params
    );

    markerArray.markers.clear();
    int colors[4]{1,0,0,0};
    for (int i = 0; i < 3; i++)
    {
        ROS_INFO("%f - %f - %f - %f", pcts.segmentCoeffs[i].c0, pcts.segmentCoeffs[i].c1, pcts.segmentCoeffs[i].c2, pcts.segmentCoeffs[i].c3);

        visualization_msgs::Marker mark;
        initMarker(mark, "map_zala_0", "segment "+std::to_string(i), visualization_msgs::Marker::LINE_STRIP, getColorObj(colors[0], colors[1], colors[2], 1));
        colors[i] = 0;
        colors[i+1] = 1;
        for (int j = pcts.sectionBorderStart[i]; j <= pcts.sectionBorderEnd[i]; j++)
        {
            Points2D p;
            p.x = j;
            p.y = pcts.segmentCoeffs[i].c0 + pcts.segmentCoeffs[i].c1 * j + pcts.segmentCoeffs[i].c2 * j * j + pcts.segmentCoeffs[i].c3 * j * j * j;
            mark.points.push_back(p);
        }
        markerArray.markers.push_back(mark);
    }
    
    ROS_INFO("=====");

    pub_visualization.publish(markerArray);

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

    TrajectoryPlanner trajectoryPlanner(nh);
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    ros::Rate rate(20);

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
#include "trajectoryController/trajectoryPlanner.hpp"

TrajectoryPlanner::TrajectoryPlanner(const ros::NodeHandle &nh_) : nh(nh_)
{
    client = nh.serviceClient<lane_keep_system::GetLaneletScenario>("/get_lanelet_scenario", true);

    while (true)
    {
        ScenarioPolynomials sp = GetScenario();

        Pose2D egoPose;
        geometry_msgs::PoseStamped gps_msg = *(ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/gps/duro/current_pose"));
        egoPose.Pose2DCoordinates.x = gps_msg.pose.position.x;
        egoPose.Pose2DCoordinates.y = gps_msg.pose.position.y;
        egoPose.Pose2DCoordinates.z = gps_msg.pose.position.z;

        tf2::Quaternion q(
            gps_msg.pose.orientation.x,
            gps_msg.pose.orientation.y,
            gps_msg.pose.orientation.z,
            gps_msg.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        ROS_INFO("%f", yaw);

        egoPose.Pose2DTheta = yaw;
        
        LDMParamIn params;
        
        for (auto x: sp.coeffs){
            ROS_INFO("%f - %f - %f - %f", x.c0, x.c1, x.c2, x.c3);
        }

        PolynomialCoeffsThreeSegments pcts = ldm.runCoeffsLite(
            sp,
            egoPose,
            params
        );

        ROS_INFO("%f - %f - %f - %f", pcts.segmentCoeffs[0].c0, pcts.segmentCoeffs[0].c1, pcts.segmentCoeffs[0].c2, pcts.segmentCoeffs[0].c3);
    }
}

ScenarioPolynomials TrajectoryPlanner::GetScenario()
{
    ScenarioPolynomials sp;

    lane_keep_system::GetLaneletScenario srv;
    srv.request.nodePointDistances.push_back(10);
    srv.request.nodePointDistances.push_back(30);
    srv.request.nodePointDistances.push_back(80);
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

    TrajectoryPlanner lh(nh);

    ros::spin();

    return 0;
}
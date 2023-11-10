#include "trajectoryPlanner/trajectoryPlanner.hpp"

TrajectoryPlanner::TrajectoryPlanner() : Node("trajectory_planner")
{
}

bool TrajectoryPlanner::init()
{
    RCLCPP_INFO(this->get_logger(), " --- Trajectory planner initializing... --- ");

    // set parameters
    std::vector<double> driverParams;
    std::vector<double> nodePtDistances;
    float target_rate;
    
    this->declare_parameter<std::vector<double>>("P",                  std::vector<double>{});
    this->declare_parameter<std::vector<double>>("nodePointDistances", std::vector<double>{});
    this->declare_parameter<int>                ("replan_cycle",       1);
    this->declare_parameter<float>              ("target_rate",        10.0f);
    this->declare_parameter<float>              ("target_speed",       50.0f);
    this->declare_parameter<bool>               ("start_on_corridor",  true);
    this->declare_parameter<std::string>        ("lanelet_frame",      "");
    this->declare_parameter<std::string>        ("gps_topic",          "");
    this->declare_parameter<float>              ("gps_yaw_offset",     0.0f);
    this->declare_parameter<bool>               ("visualize",          true);

    this->get_parameter<std::vector<double>>("P",                  driverParams);
    this->get_parameter<std::vector<double>>("nodePointDistances", nodePtDistances);
    this->get_parameter<int>                ("replan_cycle",       params.replanCycle);
    this->get_parameter<float>              ("target_rate",        target_rate);
    this->get_parameter<float>              ("target_speed",       targetSpeed);
    this->get_parameter<bool>               ("start_on_corridor",  start_on_corridor);
    this->get_parameter<std::string>        ("lanelet_frame",      lanelet_frame);
    this->get_parameter<std::string>        ("gps_topic",          gps_topic);
    this->get_parameter<float>              ("gps_yaw_offset",     gps_yaw_offset);
    this->get_parameter<bool>               ("visualize",          visualize_trajectory);

    for (uint8_t i = 0; i < 21; i++)
        params.P[i] = driverParams[i];

    for (uint8_t i = 0; i < 3; i++)
        params.P_nodePointDistances[i] = nodePtDistances[i];

    // subscribers
    sub_gps_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(gps_topic, 1, std::bind(&TrajectoryPlanner::gpsCallback, this, std::placeholders::_1));
    
    // publishers
    pub_visualization_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ldm_path", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_trajectory_ =    this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory", 1);

    // connect to lanelet map service
    lanelet_service_client_ = this->create_client<lane_keep_system::srv::GetLaneletScenario>("/get_lanelet_scenario");
    lanelet_service_client_->wait_for_service();

    if (start_on_corridor)
    {
        rclcpp::spin_some(this->shared_from_this());
        ScenarioPolynomials sp = getScenario();
        ldm.initCoeffs(sp);
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000/target_rate)),
        std::bind(&TrajectoryPlanner::runTrajectory, this)
    );

    RCLCPP_INFO(this->get_logger(), " --- Trajectory planner initialized --- ");
    return true;
}

ScenarioPolynomials TrajectoryPlanner::getScenario()
{
    RCLCPP_INFO(this->get_logger(), "Getting scenario");
    ScenarioPolynomials sp;

    std::shared_ptr<lane_keep_system::srv::GetLaneletScenario_Request> request_ = 
        std::make_shared<lane_keep_system::srv::GetLaneletScenario_Request>();
    
    request_->node_point_distances.push_back(params.P_nodePointDistances[0]);
    request_->node_point_distances.push_back(params.P_nodePointDistances[1]);
    request_->node_point_distances.push_back(params.P_nodePointDistances[2]);
    request_->gps = currentGPSMsg;
    
    std::shared_ptr<lane_keep_system::srv::GetLaneletScenario_Response> response_ = 
        lanelet_service_client_->async_send_request(request_).get();
    
    sp.coeffs.reserve(response_->coefficients.size());
    sp.kappaNominal.reserve(response_->kappa.size());

    for (auto coeffs: response_->coefficients)
    {
        PolynomialCoeffs polyCoeffs;
        polyCoeffs.c0 = coeffs.c0;
        polyCoeffs.c1 = coeffs.c1;
        polyCoeffs.c2 = coeffs.c2;
        polyCoeffs.c3 = coeffs.c3;
        sp.coeffs.push_back(polyCoeffs);
    }
    for (auto k: response_->kappa)
    {
        sp.kappaNominal.push_back(k);
    }

    RCLCPP_INFO(this->get_logger(), "Scenario received");

    return sp;
}

void TrajectoryPlanner::visualizeOutput(const TrajectoryOutput& trajectoryOutput)
{
    markerArray.markers.clear();

    PolynomialCoeffsThreeSegments pcts = trajectoryOutput.segmentCoeffs;
    int colors[4]{1,0,0,0};
    for (int i = 0; i < 3; i++)
    {
        visualization_msgs::msg::Marker mark;
        rosUtilities.initMarker(mark, "map_zala_0", "segment "+std::to_string(i), visualization_msgs::msg::Marker::LINE_STRIP, rosUtilities.getColorObj(colors[0], colors[1], colors[2], 1));
        colors[i] = 0;
        colors[i+1] = 1;
        
        for (float j = pcts.sectionBorderStart[i]; j <= pcts.sectionBorderEnd[i]; j++)
        {
            mark.points.push_back(rosUtilities.getROSPointOnPoly(j, pcts.segmentCoeffs[i]));
        }

        mark.points.push_back(rosUtilities.getROSPointOnPoly(pcts.sectionBorderEnd[i], pcts.segmentCoeffs[i]));
        markerArray.markers.push_back(mark);
    }
    
    visualization_msgs::msg::Marker mark;
    rosUtilities.initMarker(mark, "map_zala_0", "nodePts", visualization_msgs::msg::Marker::POINTS, rosUtilities.getColorObj(0, 1, 1, 1), 0.8);
    for (int i = 0; i < 4; i++)
    {
        geometry_msgs::msg::Point p;
        p.x = trajectoryOutput.nodePts.nodePointsCoordinates[i].x;
        p.y = trajectoryOutput.nodePts.nodePointsCoordinates[i].y;
        p.z = 0.5;
        mark.points.push_back(p);
    }
    markerArray.markers.push_back(mark);
    
    visualization_msgs::msg::Marker ego_mark;
    rosUtilities.initMarker(ego_mark, "map_zala_0", "ego", visualization_msgs::msg::Marker::POINTS, rosUtilities.getColorObj(1, 0, 1, 1), 1);
    ego_mark.scale.x = 4;
    ego_mark.scale.y = 1.6;
    geometry_msgs::msg::Point ego_point;
    ego_point.x = 0;
    ego_point.y = 0;
    ego_point.z = 1.5;
    ego_mark.points.push_back(ego_point);
    markerArray.markers.push_back(ego_mark);

    pub_visualization_->publish(markerArray);
}

void TrajectoryPlanner::publishOutput(const PolynomialCoeffsThreeSegments& segmentCoeffs)
{
    // trajectory
    autoware_auto_planning_msgs::msg::Trajectory trajectory;
    trajectory.header.stamp = rclcpp::Clock().now();
    trajectory.header.frame_id = lanelet_frame;

    int coeffsIdx = 0;
    float x = segmentCoeffs.sectionBorderStart[0];
    for (; x < params.P_nodePointDistances[2]; x++)
    {
        PolynomialCoeffs coeffs = segmentCoeffs.segmentCoeffs[coeffsIdx];

        autoware_auto_planning_msgs::msg::TrajectoryPoint tp;
        // position
        tp.pose.position = rosUtilities.getROSPointOnPoly(x, coeffs);
     
        // orientation
        float yaw = atan(coeffs.c1 + 2 * coeffs.c2 * x + 3 * coeffs.c3 * pow(x,2));
        tf2::Quaternion quat_tf;
        quat_tf.setEuler(yaw, 0, 0);
        quat_tf.normalize();
        tp.pose.orientation = tf2::toMsg(quat_tf);

        // speed
        tp.longitudinal_velocity_mps = targetSpeed / 3.6;
        tp.lateral_velocity_mps =      0;

        // acceleration
        tp.acceleration_mps2 = 0;

        // time_from_start
        // heading_rate_rps
        // front_wheel_angle_rad
        // rear_wheel_angle_rad

        trajectory.points.push_back(tp);

        if (coeffsIdx > segmentCoeffs.sectionBorderEnd[coeffsIdx] && coeffsIdx < 2)
            coeffsIdx++;
    }

    pub_trajectory_->publish(trajectory);
    
    // odometry

    nav_msgs::msg::Odometry odometry;

    odometry.header.stamp = rclcpp::Clock().now();
    odometry.header.frame_id = lanelet_frame;
    odometry.child_frame_id =  lanelet_frame;

    odometry.pose.pose.orientation.w = 1;
}


bool TrajectoryPlanner::runTrajectory()
{
    RCLCPP_INFO(this->get_logger(), "Trajectory planner running");
    ScenarioPolynomials sp = getScenario();
    RCLCPP_INFO(this->get_logger(), "Scenario received");
    
    Pose2D egoPose = rosUtilities.getEgoPose(currentGPSMsg);

    TrajectoryOutput trajectoryOutput = ldm.runCoeffsLite(
        sp,
        egoPose,
        params
    );

    publishOutput(trajectoryOutput.segmentCoeffs);

    // visualization
    if (visualize_trajectory)
        visualizeOutput(trajectoryOutput);

    return true;
}

void TrajectoryPlanner::gpsCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& gps_msg_)
{
    tf2::Quaternion q;
    tf2::fromMsg(gps_msg_->pose.orientation, q);
    
    tf2::Quaternion q_rotation;
    q_rotation.setRPY(0, 0, gps_yaw_offset);

    tf2::Quaternion q_rotated = q_rotation * q;
    q_rotated.normalize();

    currentGPSMsg = *gps_msg_;
    currentGPSMsg.pose.orientation = tf2::toMsg(q_rotated);
    
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<TrajectoryPlanner> trajectoryPlannerNode_ = std::make_shared<TrajectoryPlanner>();

    if (!trajectoryPlannerNode_->init())
        return 1;

    rclcpp::spin(trajectoryPlannerNode_);

    return 0;
}
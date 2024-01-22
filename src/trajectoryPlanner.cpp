#include "trajectoryPlanner/trajectoryPlanner.hpp"

#include <rclcpp/wait_for_message.hpp>


TrajectoryPlanner::TrajectoryPlanner() : Node("trajectory_planner")
{
}

bool TrajectoryPlanner::init()
{
    RCLCPP_INFO(this->get_logger(), "Trajectory planner initializing...");

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

    RCLCPP_INFO(this->get_logger(), "Waiting for scenario message...");
    rclcpp::wait_for_message<lane_keep_system::msg::Scenario>(currentScenario, this->shared_from_this(), "scenario");
    
    if (start_on_corridor)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing corridor start...");

        ScenarioPolynomials sp = getScenario();
        ldm.initCoeffs(sp);
    }

    // subscribers
    sub_scenario_ = this->create_subscription<lane_keep_system::msg::Scenario>("scenario", 1, 
        std::bind(&TrajectoryPlanner::scenarioCallback, this, std::placeholders::_1));
    sub_velocity_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", 1, std::bind(&TrajectoryPlanner::velocity_callback, this, std::placeholders::_1));
    sub_accel = this->create_subscription<pacmod3_msgs::msg::SystemRptFloat>(
        "/pacmod/accel_rpt", 1, std::bind(&TrajectoryPlanner::accel_callback, this, std::placeholders::_1));

    // publishers
    pub_visualization_        = this->create_publisher<visualization_msgs::msg::MarkerArray>("ldm_path", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_trajectory_           = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("/planning/scenario_planning/trajectory", 1);
    pub_odometry_             = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 1);
    pub_gear_                 = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 1);
    pub_accel_                = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("/localization/acceleration", 1);
    pub_operation_mode_state_ = this->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>("/system/operation_mode/state", 1);
    pub_debug_trajectory_    = this->create_publisher<lane_keep_system::msg::Trajectory>("debug/planned_trajectory", 1);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000/target_rate)),
        std::bind(&TrajectoryPlanner::runTrajectory, this)
    );

    RCLCPP_INFO(this->get_logger(), " --- Trajectory planner initialized --- ");
    return true;
}

void TrajectoryPlanner::visualizeOutput(const TrajectoryOutput& trajectoryOutput)
{
    markerArray.markers.clear();

    PolynomialCoeffsThreeSegments pcts = trajectoryOutput.segmentCoeffs;
    int colors[4]{1,0,0,0};
    for (int i = 0; i < 3; i++)
    {
        visualization_msgs::msg::Marker mark;
        rosUtilities.initMarker(mark, lanelet_frame, "segment "+std::to_string(i), visualization_msgs::msg::Marker::LINE_STRIP, rosUtilities.getColorObj(colors[0], colors[1], colors[2], 1));
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
    rosUtilities.initMarker(mark, lanelet_frame, "nodePts", visualization_msgs::msg::Marker::POINTS, rosUtilities.getColorObj(0, 1, 1, 1), 0.8);
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
    rosUtilities.initMarker(ego_mark, lanelet_frame, "ego", visualization_msgs::msg::Marker::POINTS, rosUtilities.getColorObj(1, 0, 1, 1), 1);
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

void TrajectoryPlanner::publishOutput(const TrajectoryOutput& trajectoryOutput)
{
    PolynomialCoeffsThreeSegments segmentCoeffs = trajectoryOutput.segmentCoeffs;

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

        // tp.lateral_velocity_mps;
        // tp.acceleration_mps2;
        // time_from_start
        // heading_rate_rps
        // front_wheel_angle_rad
        // rear_wheel_angle_rad

        trajectory.points.push_back(tp);

        if (coeffsIdx > segmentCoeffs.sectionBorderEnd[coeffsIdx] && coeffsIdx < 2)
            coeffsIdx++;
    }

    pub_trajectory_->publish(trajectory);

    // debug topic
    lane_keep_system::msg::Trajectory debug_trajectory_msg;
    debug_trajectory_msg.stamp = rclcpp::Clock().now();

    for (int i = 0; i < 3; i++)
    {
        PolynomialCoeffs coeffs = trajectoryOutput.segmentCoeffs.segmentCoeffs[i];
        lane_keep_system::msg::Polynomial poly;
        poly.c0 = coeffs.c0;
        poly.c1 = coeffs.c1;
        poly.c2 = coeffs.c2;
        poly.c3 = coeffs.c3;

        debug_trajectory_msg.coeffs.push_back(poly);
    }

    for (int i = 0; i < 3; i++)
    {
        geometry_msgs::msg::Point p;
        p.x = trajectoryOutput.nodePts.nodePointsCoordinates[coeffsIdx].x;
        p.y = trajectoryOutput.nodePts.nodePointsCoordinates[coeffsIdx].y;

        debug_trajectory_msg.node_point_coords.push_back(p);
    }

    debug_trajectory_msg.node_point_thetas.push_back(trajectoryOutput.nodePts.nodePointsTheta[coeffsIdx]);
    pub_debug_trajectory_->publish(debug_trajectory_msg);

    // odometry
    nav_msgs::msg::Odometry odometry;

    odometry.header.stamp = rclcpp::Clock().now();
    odometry.header.frame_id = lanelet_frame;
    odometry.child_frame_id  = lanelet_frame;
    odometry.pose.pose.orientation.w = 1;
    odometry.twist.twist.linear.x = currentVelocity;
    pub_odometry_->publish(odometry);

    // acceleration
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header.stamp = rclcpp::Clock().now();
    accel.accel.accel.linear.x = currentAcceleration;
    pub_accel_->publish(accel);

    // gear
    autoware_auto_vehicle_msgs::msg::GearCommand gear;
    gear.stamp = rclcpp::Clock().now();
    gear.command = 2;
    pub_gear_->publish(gear);

    // operation mode state
    autoware_adapi_v1_msgs::msg::OperationModeState operation_mode_state;
    operation_mode_state.stamp = rclcpp::Clock().now();
    operation_mode_state.mode = operation_mode_state.AUTONOMOUS;
    operation_mode_state.is_autoware_control_enabled = true;
    pub_operation_mode_state_->publish(operation_mode_state);
}

ScenarioPolynomials TrajectoryPlanner::getScenario()
{
    ScenarioPolynomials sp;
    for (auto coeffs: currentScenario.coefficients)
    {
        PolynomialCoeffs polyCoeffs;
        polyCoeffs.c0 = coeffs.c0;
        polyCoeffs.c1 = coeffs.c1;
        polyCoeffs.c2 = coeffs.c2;
        polyCoeffs.c3 = coeffs.c3;
        sp.coeffs.push_back(polyCoeffs);
    }
    for (auto k: currentScenario.kappa)
    {
        sp.kappaNominal.push_back(k);
    }
    return sp;
}

bool TrajectoryPlanner::runTrajectory()
{
    // get scenario
    ScenarioPolynomials sp = getScenario();

    // define ego pose from scenario
    Pose2D egoPose = rosUtilities.getEgoPose(currentScenario.gps);

    // run LDM
    TrajectoryOutput trajectoryOutput = ldm.runCoeffsLite(
        sp,
        egoPose,
        params
    );
    
    // publish output to MPC
    publishOutput(trajectoryOutput);

    // visualization
    if (visualize_trajectory)
        visualizeOutput(trajectoryOutput);

    return true;
}

void TrajectoryPlanner::scenarioCallback(const std::shared_ptr<const lane_keep_system::msg::Scenario>& msg_)
{
    currentScenario = *msg_;
}

void TrajectoryPlanner::velocity_callback(const std::shared_ptr<const autoware_auto_vehicle_msgs::msg::VelocityReport>& msg_)
{
    currentVelocity = msg_->longitudinal_velocity;
}

void TrajectoryPlanner::accel_callback(const std::shared_ptr<const pacmod3_msgs::msg::SystemRptFloat>& msg_)
{
    currentAcceleration = msg_->output;
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
#include "laneletHandler/laneletMapHandler.hpp"

#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/filesystem.hpp>

#include "laneletHandler/laneletMapHandler.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"

LaneletHandler::LaneletHandler() : Node("lanelet_handler")
{
    if (!init()){
        RCLCPP_ERROR(this->get_logger(), " !!! Lanelet handler initialization failed !!! ");
        rclcpp::shutdown();
    }
}


bool LaneletHandler::init()
{
    RCLCPP_INFO(this->get_logger(), "LaneletHandler initializing...");

    std::string lanelet2_path;
    float target_rate;
    std::vector<double> node_point_distances;
    
    this->declare_parameter<std::string>        ("lanelet2_path",      "");
    this->declare_parameter<std::string>        ("lanelet_frame",      "");
    this->declare_parameter<std::string>        ("ego_frame",          "");
    this->declare_parameter<std::string>        ("gps_frame",          "");
    this->declare_parameter<std::string>        ("gps_topic",          "");
    this->declare_parameter<bool>               ("visualize",          false);
    this->declare_parameter<float>              ("target_rate",        10.0f);
    this->declare_parameter<float>              ("gps_yaw_offset",     0.0f);
    this->declare_parameter<std::vector<double>>("nodePointDistances", std::vector<double>{});
    this->declare_parameter<std::string>        ("mw_ew",              "east");

    // get parameters
    std::string mw_ew;
    this->get_parameter<std::string>        ("lanelet2_path",      lanelet2_path);
    this->get_parameter<std::string>        ("lanelet_frame",      lanelet_frame);
    this->get_parameter<std::string>        ("ego_frame",          ego_frame);
    this->get_parameter<std::string>        ("gps_frame",          gps_frame);
    this->get_parameter<std::string>        ("gps_topic",          gps_topic);
    this->get_parameter<bool>               ("visualize",          visualize_path);
    this->get_parameter<float>              ("target_rate",        target_rate);
    this->get_parameter<float>              ("gps_yaw_offset",     gps_yaw_offset);
    this->get_parameter<std::vector<double>>("nodePointDistances", node_point_distances);
    this->get_parameter<std::string>        ("mw_ew",              mw_ew);

    // TODO: better way to handle planning
    if (mw_ew == "east")
    {
        start_id = 27757;
        end_id = 27749;
    }
    else
    {
        start_id = 27763;
        end_id = 27759;
    }

    for (uint8_t i = 0; i < 3; i++)
        nodePtDistances[i] = node_point_distances[i];

    // get gps to lanelet transform
    tf2_ros::Buffer tfBuffer(this->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);
    gpsTransform = tfBuffer.lookupTransform(ego_frame, gps_frame, rclcpp::Time(0), rclcpp::Duration(3, 0));
    
    // get lanelet file path
    std::string lanelet2_file_path;
    boost::filesystem::path path(lanelet2_path);
    lanelet2_file_path = path.generic_string();

    if (lanelet2_file_path == "")
    {
        RCLCPP_ERROR(this->get_logger(), "File name is not specified or wrong. [%s]", lanelet2_file_path.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Loading file \"%s\"", lanelet2_file_path.c_str());

    // load lanelet file
    lanelet::ErrorMessages errors{};
    lanelet::projection::MGRSProjector projector{};
    const lanelet::LaneletMapPtr map = lanelet::load(lanelet2_file_path, projector, &errors);

    // overwrite local_x, local_y
    for (lanelet::Point3d point : map->pointLayer) {
      if (point.hasAttribute("local_x")) {
        point.x() = point.attribute("local_x").asDouble().value() - 1.0;
      }
      if (point.hasAttribute("local_y")) {
        point.y() = point.attribute("local_y").asDouble().value();
      }
    }

    // realign lanelet borders using updated points
    for (lanelet::Lanelet lanelet : map->laneletLayer) {
      auto left = lanelet.leftBound();
      auto right = lanelet.rightBound();
      std::tie(left, right) = lanelet::geometry::align(left, right);
      lanelet.setLeftBound(left);
      lanelet.setRightBound(right);
    }

    lanelet::utils::overwriteLaneletsCenterline(map, 1, false);

    for (const auto& error : errors)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), error);
    }
    if (!errors.empty())
    {
        return false;
    }

    // isolate road lanes
    roadLanelets = lanelet::utils::query::laneletLayer(map);
    
    // Initialize path planning
    lanelet::traffic_rules::TrafficRulesPtr trafficRules{lanelet::traffic_rules::TrafficRulesFactory::instance().create(lanelet::Locations::Germany, lanelet::Participants::Vehicle)};
    lanelet::routing::RoutingGraphPtr graph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);

    // path planning
    // MW
    lanelet::Optional<lanelet::routing::LaneletPath> trajectory_path = graph->shortestPath(map->laneletLayer.get(start_id), map->laneletLayer.get(end_id));

    // TODO: remove this
    // UNI
    // lanelet::ConstLanelets pathLanelets;
    // pathLanelets.push_back(map->laneletLayer.get(10649));

    // lanelet::Optional<lanelet::routing::LaneletPath> trajectory_path = graph->shortestPathVia(map->laneletLayer.get(13235), pathLanelets, map->laneletLayer.get(13235));


    // init subscribers
    sub_gps_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(gps_topic, 1, std::bind(&LaneletHandler::gpsCallback, this, std::placeholders::_1));

    // wait for gps message
    while (rclcpp::ok() && !currentGPSMsg.header.stamp.sec)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // init publishers
    pub_scenario_      = this->create_publisher<lane_keep_system::msg::Scenario>     ("scenario",          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_road_lines_    = this->create_publisher<visualization_msgs::msg::MarkerArray>("paths",             rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_lanelet_lines_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lanelet_lanes",     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_derivatives_   = this->create_publisher<lane_keep_system::msg::Derivatives>  ("debug/derivatives", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_centerline_    = this->create_publisher<lane_keep_system::msg::PointList>    ("debug/centerline",  rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // collect points from planned path
    pathPoints.clear();
    
    visualization_msgs::msg::Marker laneletCenterMarker;
    visualization_msgs::msg::Marker laneletLeftMarker;
    visualization_msgs::msg::Marker laneletRightMarker;

    rosUtilities.initMarker(laneletCenterMarker, lanelet_frame, "lanelet_center", visualization_msgs::msg::Marker::LINE_STRIP, rosUtilities.getColorObj(1, 0.5, 0, 1), 0.2);
    rosUtilities.initMarker(laneletLeftMarker,   lanelet_frame, "lanelet_left",   visualization_msgs::msg::Marker::LINE_STRIP, rosUtilities.getColorObj(1, 0.5, 0, 1), 0.2);
    rosUtilities.initMarker(laneletRightMarker,  lanelet_frame, "lanelet_right",  visualization_msgs::msg::Marker::LINE_STRIP, rosUtilities.getColorObj(1, 0.5, 0, 1), 0.2);

    lane_keep_system::msg::PointList centerline_msg;
    centerline_msg.stamp = rclcpp::Clock().now();

    for (auto lane = trajectory_path.get().begin(); lane != trajectory_path.get().end(); lane++)
    {
        for (auto pt = lane->centerline().begin(); pt != lane->centerline().end(); pt++)
        {
            Points2D p;
            p.x = pt->x();
            p.y = pt->y();
            pathPoints.push_back(p);

            geometry_msgs::msg::Point geoPt;
            geoPt.x = p.x;
            geoPt.y = p.y;
            centerline_msg.points.push_back(geoPt);
            laneletCenterMarker.points.push_back(geoPt);
        }
        for (auto pt = lane->leftBound().begin(); pt != lane->leftBound().end(); pt++)
        {
            geometry_msgs::msg::Point geoPt;
            geoPt.x = pt->x();
            geoPt.y = pt->y();
            laneletLeftMarker.points.push_back(geoPt);
        }
        for (auto pt = lane->rightBound().begin(); pt != lane->rightBound().end(); pt++)
        {
            geometry_msgs::msg::Point geoPt;
            geoPt.x = pt->x();
            geoPt.y = pt->y();
            laneletRightMarker.points.push_back(geoPt);
        }
    }
    markerArray.markers.push_back(laneletCenterMarker);
    markerArray.markers.push_back(laneletLeftMarker);
    markerArray.markers.push_back(laneletRightMarker);

    pub_lanelet_lines_->publish(markerArray);
    // debug topic
    pub_centerline_->publish(centerline_msg);

    // save last point index for faster nearest neighbor search
    lastStartPointIdx = 0;
    nearestNeighborThreshold = 2;

    // init timer for scenario publishing
    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000/target_rate)), std::bind(&LaneletHandler::publishScenario, this));

    RCLCPP_INFO(this->get_logger(), " --- LaneletHandler initialized --- ");
    return true;
}

void LaneletHandler::gpsCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& gps_msg_)
{
    geometry_msgs::msg::PoseStamped pose;
    pose = *gps_msg_;

    if (gps_yaw_offset != 0.0f)
    {
        tf2::Quaternion q;
        tf2::fromMsg(gps_msg_->pose.orientation, q);
        
        tf2::Quaternion q_rotation;
        q_rotation.setRPY(0, 0, gps_yaw_offset);

        tf2::Quaternion q_rotated = q_rotation * q;
        q_rotated.normalize();
        pose.pose.orientation = tf2::toMsg(q_rotated);
    }

    tf2::doTransform(pose, pose, gpsTransform);
    
    currentGPSMsg = pose;
}


float LaneletHandler::distanceBetweenPoints(const Points2D a, const Points2D b)
{
    return sqrt(pow(b.x-a.x, 2)+pow(b.y-a.y, 2));
}

int LaneletHandler::getGPSNNPointIdx(const Points2D& gps_pos)
{
    int startPointIdx = lastStartPointIdx;
    bool nnTrheshold_reached = false;
    double min_dist = distanceBetweenPoints(gps_pos, pathPoints[startPointIdx]);
    for (uint32_t i = startPointIdx+1; i < pathPoints.size(); i++)
    {
        double current_dist = distanceBetweenPoints(gps_pos, pathPoints[i]);

        if (current_dist < min_dist)
        {
            min_dist = current_dist;
            startPointIdx = i;
        }

        if (current_dist < nearestNeighborThreshold)
            nnTrheshold_reached = true;
        else if (nnTrheshold_reached)
            break;
    }
    lastStartPointIdx = startPointIdx;
    return startPointIdx;
}

bool LaneletHandler::createScenario(
    const geometry_msgs::msg::PoseStamped& gpsPose, 
    const float                            nodePtDistances[3],
    TrajectoryPoints&                      scenarioFullEGO,
    std::vector<uint16_t>&                 nodePtIndexes)
{
    Points2D gps_position;
    gps_position.x = gpsPose.pose.position.x;
    gps_position.y = gpsPose.pose.position.y;

    // find nearest point to gps postition on path
    int gpsNNPointIdx = getGPSNNPointIdx(gps_position);

    double   current_length = 0;
    uint32_t pathPointIdx = gpsNNPointIdx + 1;
    uint16_t nodePtIdx = 0;
    uint16_t scenarioPtCounter = 1;
    bool     isLengthReached = false;

    Points2D prev_pt;
    Points2D transformedPoint;
    Pose2D egoPose;
    egoPose.Pose2DCoordinates.x = gps_position.x;
    egoPose.Pose2DCoordinates.y = gps_position.y;
    egoPose.Pose2DTheta         = rosUtilities.getYawFromPose(gpsPose);

    coordinateTransforms.transform2D(pathPoints[gpsNNPointIdx], egoPose, prev_pt);
    scenarioFullEGO.push_back(prev_pt);

    while (pathPointIdx < pathPoints.size() && !isLengthReached)
    {
        coordinateTransforms.transform2D(pathPoints[pathPointIdx], egoPose, transformedPoint);

        if (prev_pt.x == transformedPoint.x)
        {
            // consecutive point duplicate on map
            pathPointIdx++;
            continue;
        }
        current_length += abs(transformedPoint.x - prev_pt.x);

        // node point locations in scenario
        if (nodePtIdx < POLYLINE_COUNT)
        {
            if (current_length > nodePtDistances[nodePtIdx])
            {
                // first point BEFORE the length criteria
                nodePtIndexes.push_back(scenarioPtCounter - 1);
                nodePtIdx++;
            }
            scenarioFullEGO.push_back(transformedPoint);
        }
        else
        {
            isLengthReached = true;
        }

        prev_pt = transformedPoint;
        pathPointIdx++;
        scenarioPtCounter++;
    }

    // not enough points to plan (path ended)
    if (current_length < nodePtDistances[POLYLINE_COUNT-1])
    {
        return false;
    }
    return true;
}

Segments LaneletHandler::sliceScenario(
    const TrajectoryPoints&      scenarioFullEGO, 
    const std::vector<uint16_t>& nodePtIndexes)
{
    Segments segments;
    segments.reserve(POLYLINE_COUNT + 1);

    uint16_t prevNodePtIdx = 0;
    uint16_t segmentIdx = 0;
    for (uint16_t nodePtIdx: nodePtIndexes)
    {
        TrajectoryPoints tp;
        tp.reserve(nodePtIdx - prevNodePtIdx + 1);

        for (uint16_t i = prevNodePtIdx; i <= nodePtIdx; i++)
        {
            tp.push_back(scenarioFullEGO[i]);
        }
        prevNodePtIdx = nodePtIdx;
        segmentIdx++;

        segments.push_back(tp);
    }

    return segments;
}

std::vector<lane_keep_system::msg::Polynomial> LaneletHandler::fitPolynomials(const Segments& segments)
{
    std::vector<lane_keep_system::msg::Polynomial> scenarioPolynomials;
    for (uint8_t i = 0; i < POLYLINE_COUNT; i++)
    {
        PolynomialCoeffs pc = polynomialSubfunctions.fitThirdOrderPolynomial(segments[i]);
        lane_keep_system::msg::Polynomial poly;
        poly.c0 = pc.c0;
        poly.c1 = pc.c1;
        poly.c2 = pc.c2;
        poly.c3 = pc.c3;
        scenarioPolynomials.push_back(poly);
    }
    return scenarioPolynomials;
}

TrajectoryPoints LaneletHandler::numericalDerivative(const TrajectoryPoints& points)
{
    TrajectoryPoints derivative;
    derivative.reserve(points.size());
    
    TrajectoryPoints::const_iterator currentPt_it = points.begin();

    // forward differencing first point
    Points2D currentPoint;
    currentPoint.x = currentPt_it->x;
    currentPoint.y = (
        ((currentPt_it + 1)->y - currentPt_it->y) /
        ((currentPt_it + 1)->x - currentPt_it->x));
    derivative.push_back(currentPoint);
    
    currentPt_it++;
    for (; currentPt_it != points.end() - 1; currentPt_it++)
    {
        // central differencing middle points
        currentPoint.x = currentPt_it->x;
        currentPoint.y = (
            ((currentPt_it + 1)->y - (currentPt_it - 1)->y) /
            ((currentPt_it + 1)->x - (currentPt_it - 1)->x));
        derivative.push_back(currentPoint);
    }
    
    // backward differencing last point
    currentPoint.x = currentPt_it->x;
    currentPoint.y = (
        (currentPt_it->y - (currentPt_it - 1)->y) /
        (currentPt_it->x - (currentPt_it - 1)->x));
    derivative.push_back(currentPoint);

    return derivative;
}

TrajectoryPoints LaneletHandler::movingAverage(const TrajectoryPoints& points, int windowSize)
{
    uint16_t maxCheckSize = (windowSize-1) / 2;
    TrajectoryPoints maResult;

    for (uint16_t i = 0; i < points.size(); i++)
    {
        int neighbourCheckSize = std::min<uint16_t>({i, maxCheckSize, (uint16_t)(points.size()-i-1)});

        Points2D outPt;
        outPt.x = points[i].x;

        float sum = points[i].y;
        for (int j = 1; j <= neighbourCheckSize; j++)
        {
            sum += points[i-j].y;
            sum += points[i+j].y;
        }
        outPt.y = sum / (neighbourCheckSize * 2 + 1);

        maResult.push_back(outPt);
    }

    return maResult;
}

void LaneletHandler::publishScenario()
{
    geometry_msgs::msg::PoseStamped gps = currentGPSMsg;
    lane_keep_system::msg::Scenario scenario_msg;
    scenario_msg.stamp = rclcpp::Clock().now();
    scenario_msg.gps = gps;

    // get scenario
    TrajectoryPoints scenarioFullEGO;
    std::vector<uint16_t> nodePtIndexes;
    bool validScenario = createScenario(gps, nodePtDistances, scenarioFullEGO, nodePtIndexes);
    
    if (!validScenario)
    {
        // invalid scenario, return all 0
        for (uint8_t i = 0; i < POLYLINE_COUNT; i++)
        {
            lane_keep_system::msg::Polynomial out_coeffs;
            scenario_msg.coefficients.push_back(out_coeffs);
            scenario_msg.kappa.push_back(0);
        }
        scenario_msg.valid_scenario = false;
        pub_scenario_->publish(scenario_msg);
        return;
    }
    
    // slice trajectory at nodepoints
    Segments segments = sliceScenario(scenarioFullEGO, nodePtIndexes);

    // fit polynomes
    std::vector<lane_keep_system::msg::Polynomial> scenarioPolynomials = fitPolynomials(segments);
    scenario_msg.coefficients = scenarioPolynomials;

    // calculate 2nd degree numerical derivatives for kappa
    TrajectoryPoints derivative_1 =    numericalDerivative(scenarioFullEGO);
    TrajectoryPoints derivative_1_ma = movingAverage(derivative_1, 5);

    TrajectoryPoints derivative_2 =    numericalDerivative(derivative_1_ma);
    TrajectoryPoints derivative_2_ma = movingAverage(derivative_2, 5);
    
    // publish derivatives for debugging
    lane_keep_system::msg::Derivatives derivatives_msg;
    derivatives_msg.stamp = rclcpp::Clock().now();

    for (uint16_t i = 0; i < scenarioFullEGO.size(); i++)
    {
        derivatives_msg.x.push_back(scenarioFullEGO[i].x);
        derivatives_msg.der1.push_back(derivative_1[i].y);
        derivatives_msg.der1_ma.push_back(derivative_1_ma[i].y);
        derivatives_msg.der2.push_back(derivative_2[i].y);
        derivatives_msg.der2_ma.push_back(derivative_2_ma[i].y);
    }

    // kappa averages between nodepoints
    uint16_t prevNodePtIdx = 0;
    for (uint16_t nodePtIdx: nodePtIndexes)
    {
        float kappa = 0;
        for (uint16_t i = prevNodePtIdx; i <= nodePtIdx; i++)
        {
            kappa += derivative_2[i].y;
        }
        kappa /= nodePtIdx - prevNodePtIdx + 1;
        scenario_msg.kappa.push_back(kappa);

        prevNodePtIdx = nodePtIdx;
    }

    // publish scenario
    scenario_msg.valid_scenario = true;
    pub_scenario_->publish(scenario_msg);
    pub_derivatives_->publish(derivatives_msg);
    
    // visualistion
    if (visualize_path)
    {
        markerArray.markers.clear();

        // create and initialize markers
        visualization_msgs::msg::Marker plannedPathMarker;
        visualization_msgs::msg::Marker scenarioPathCenterMarker;
        visualization_msgs::msg::Marker scenarioPathLeftMarker;
        visualization_msgs::msg::Marker scenarioPathRightMarker;
        
        rosUtilities.initMarker(plannedPathMarker,        lanelet_frame, "planned_path",             visualization_msgs::msg::Marker::LINE_STRIP, rosUtilities.getColorObj(0, 1, 0, 0.2));
        rosUtilities.initMarker(scenarioPathCenterMarker, lanelet_frame, "scenario_path_center",     visualization_msgs::msg::Marker::POINTS,     rosUtilities.getColorObj(1, 0.5, 0, 1));
        rosUtilities.initMarker(scenarioPathLeftMarker,   lanelet_frame, "scenario_path_edge_left",  visualization_msgs::msg::Marker::LINE_STRIP, rosUtilities.getColorObj(1, 0.5, 0, 1), 0.2);
        rosUtilities.initMarker(scenarioPathRightMarker,  lanelet_frame, "scenario_path_edge_right", visualization_msgs::msg::Marker::LINE_STRIP, rosUtilities.getColorObj(1, 0.5, 0, 1), 0.2);
        
        // planned path
        for (Points2D pt: pathPoints)
        {
            plannedPathMarker.points.push_back(rosUtilities.convertPoint_CPP2ROS(pt));
        }
        
        // transformed scenario + road edge approximation
        for (uint16_t i = 0; i < scenarioFullEGO.size(); i++)
        {
            Points2D p;
            p.x = scenarioFullEGO[i].x;
            p.y = scenarioFullEGO[i].y;

            scenarioPathCenterMarker.points.push_back(rosUtilities.convertPoint_CPP2ROS(p));
            p.y += 1.9;
            scenarioPathLeftMarker.points.push_back(rosUtilities.convertPoint_CPP2ROS(p));
            p.y -= 3.8;
            scenarioPathRightMarker.points.push_back(rosUtilities.convertPoint_CPP2ROS(p));
        }
        
        markerArray.markers.push_back(plannedPathMarker);
        markerArray.markers.push_back(scenarioPathCenterMarker);
        markerArray.markers.push_back(scenarioPathLeftMarker);
        markerArray.markers.push_back(scenarioPathRightMarker);

        pub_road_lines_->publish(markerArray);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LaneletHandler>());

    return 0;
}

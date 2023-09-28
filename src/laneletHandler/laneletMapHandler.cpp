#include "laneletHandler/laneletMapHandler.hpp"

#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/filesystem.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "laneletHandler/laneletMapHandler.hpp"
#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"

LaneletHandler::LaneletHandler(const ros::NodeHandle &nh, const ros::NodeHandle &nh_p) : nh(nh), nh_p(nh_p)
{
    if (!init()){
        ros::shutdown();
    }
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
    for (int i = startPointIdx+1; i < pathPoints.size(); i++)
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
    const geometry_msgs::PoseStamped& gpsPose, 
    const std::vector<float>&         nodePtDistances,
    TrajectoryPoints&                 scenarioFullEGO,
    std::vector<int>&                 nodePtIndexes)
{
    // transform gps coordinates from global frame to lanelet frame
    Points2D gps_position;
    gps_position.x = gpsPose.pose.position.x - laneletFramePose.Pose2DCoordinates.x;
    gps_position.y = gpsPose.pose.position.y - laneletFramePose.Pose2DCoordinates.y;

    // find nearest point to gps postition on path
    int gpsNNPointIdx = getGPSNNPointIdx(gps_position);

    double current_length = 0;
    int    pathPointIdx = gpsNNPointIdx + 1;
    int    nodePtIdx = 0;
    int    scenarioPtCounter = 1;
    bool   isLengthReached = false;

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
        if (nodePtIdx < nodePtDistances.size())
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
    if (current_length < nodePtDistances.back())
    {
        return false;
    }
    return true;
}

Segments LaneletHandler::sliceScenario(
    const TrajectoryPoints& scenarioFullEGO, 
    const std::vector<int>& nodePtIndexes)
{
    Segments segments;
    segments.reserve(polyline_count);

    int prevNodePtIdx = 0;
    int segmentIdx = 0;
    for (int nodePtIdx: nodePtIndexes)
    {
        TrajectoryPoints tp;
        tp.reserve(nodePtIdx - prevNodePtIdx + 1);

        for (int i = prevNodePtIdx; i <= nodePtIdx; i++)
        {
            tp.push_back(scenarioFullEGO[i]);
        }
        prevNodePtIdx = nodePtIdx;
        segmentIdx++;

        segments.push_back(tp);
    }

    return segments;
}

std::vector<lane_keep_system::Polynomial> LaneletHandler::fitPolynomials(const Segments& segments)
{
    std::vector<lane_keep_system::Polynomial> scenarioPolynomials;
    for (uint8_t i = 0; i < polyline_count; i++)
    {
        PolynomialCoeffs pc = polynomialSubfunctions.fitThirdOrderPolynomial(segments[i]);
        lane_keep_system::Polynomial poly;
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
    int maxCheckSize = (windowSize-1) / 2;
    TrajectoryPoints maResult;

    for (int i = 0; i < points.size(); i++)
    {
        int neighbourCheckSize = std::min({i, maxCheckSize, (int)points.size()-i-1});

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


bool LaneletHandler::init()
{
    std::string lanelet2_path;
    
    // get parameters
    nh_p.param<std::string>("lanelet2_path", lanelet2_path, "");
    nh_p.param<std::string>("lanelet_frame", lanelet_frame, "");
    nh_p.param<std::string>("ego_frame", ego_frame, "");
    nh_p.param<bool>("visualize", visualize_path, false);

    // get gps to lanelet transform
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped lanelet_2_map_transform = tfBuffer.lookupTransform(lanelet_frame, "map", ros::Time(0), ros::Duration(3));
    laneletFramePose.Pose2DCoordinates.x = -lanelet_2_map_transform.transform.translation.x;
    laneletFramePose.Pose2DCoordinates.y = -lanelet_2_map_transform.transform.translation.y;
    laneletFramePose.Pose2DTheta = 0;

    // get lanelet file path
    std::string lanelet2_file_path;
    boost::filesystem::path path(lanelet2_path);
    lanelet2_file_path = path.generic_string();

    if (lanelet2_file_path == "")
    {
        ROS_ERROR("File name is not specified or wrong. [%s]", lanelet2_file_path.c_str());
        return false;
    }

    ROS_INFO("Loading file \"%s\"", lanelet2_file_path.c_str());

    // load lanelet file
    lanelet::ErrorMessages errors;
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_file_path, lanelet::projection::UtmProjector(lanelet::Origin({46.894188, 16.834861, 0})), &errors);
    
    for (const auto& error : errors)
    {
        ROS_ERROR_STREAM(error);
    }
    if (!errors.empty())
    {
        return false;
    }

    // remake centerlines for better quality
    lanelet::utils::overwriteLaneletsCenterline(map, false);

    // isolate road lanes
    roadLanelets = lanelet::utils::query::laneletLayer(map);
    
    // Initialize path planning
    lanelet::traffic_rules::TrafficRulesPtr trafficRules{lanelet::traffic_rules::TrafficRulesFactory::instance().create(lanelet::Locations::Germany, lanelet::Participants::Vehicle)};
    lanelet::routing::RoutingGraphPtr graph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);

    // path planning
    lanelet::Optional<lanelet::routing::LaneletPath> trajectory_path = graph->shortestPath(map->laneletLayer.get(-27757), map->laneletLayer.get(-27749));

    // collect points from planned path
    pathPoints.clear();
    for (auto lane = trajectory_path.get().begin(); lane != trajectory_path.get().end(); lane++)
    {
        for (auto pt = lane->centerline().begin(); pt != lane->centerline().end(); pt++)
        {
            Points2D p;
            p.x = pt->x();
            p.y = pt->y();
            pathPoints.push_back(p);
        }
    }
    
    // save last point index for faster nearest neighbor search
    lastStartPointIdx = 0;
    nearestNeighborThreshold = 2;

    // TEMP
    pub_road_lines =  nh.advertise<visualization_msgs::MarkerArray>("paths", 1, true);
    pub_derivatives = nh.advertise<lane_keep_system::Derivatives>("derivatives", 1, true);

    // init lanelet scenario service
    lanelet_service_ = nh.advertiseService("/get_lanelet_scenario", &LaneletHandler::LaneletScenarioServiceCallback, this);
    
    ROS_INFO("LaneletHandler initialized.");
    return true;
}

bool LaneletHandler::LaneletScenarioServiceCallback(
    lane_keep_system::GetLaneletScenario::Request &req, 
    lane_keep_system::GetLaneletScenario::Response &res)
{
    polyline_count = req.nodePointDistances.size();
    
    // get scenario
    TrajectoryPoints scenarioFullEGO;
    std::vector<int> nodePtIndexes;
    bool validScenario = createScenario(req.gps, req.nodePointDistances, scenarioFullEGO, nodePtIndexes);
    
    if (!validScenario)
    {
        // invalid scenario, return all 0
        for (uint8_t i = 0; i < polyline_count; i++)
        {
            lane_keep_system::Polynomial out_coeffs;
            res.coefficients.push_back(out_coeffs);
            res.kappa.push_back(0);
        }
        return false;
    }
    
    // slice trajectory at nodepoints
    Segments segments = sliceScenario(scenarioFullEGO, nodePtIndexes);

    // fit polynomes
    std::vector<lane_keep_system::Polynomial> scenarioPolynomials = fitPolynomials(segments);
    res.coefficients = scenarioPolynomials;

    // calculate 2nd degree numerical derivatives for kappa
    TrajectoryPoints derivative_1 =    numericalDerivative(scenarioFullEGO);
    TrajectoryPoints derivative_1_ma = movingAverage(derivative_1, 5);

    TrajectoryPoints derivative_2 =    numericalDerivative(derivative_1_ma);
    TrajectoryPoints derivative_2_ma = movingAverage(derivative_2, 5);
    
    // publish derivatives for debugging
    lane_keep_system::Derivatives plt;

    for (int i = 0; i < scenarioFullEGO.size(); i++)
    {
        plt.x.push_back(scenarioFullEGO[i].x);
        plt.der1.push_back(derivative_1[i].y);
        plt.der1_ma.push_back(derivative_1_ma[i].y);
        plt.der2.push_back(derivative_2[i].y);
        plt.der2_ma.push_back(derivative_2_ma[i].y);
    }
    pub_derivatives.publish(plt);

    // kappa averages between nodepoints
    int prevNodePtIdx = 0;
    for (int nodePtIdx: nodePtIndexes)
    {
        float kappa = 0;
        for (int i = prevNodePtIdx; i <= nodePtIdx; i++)
        {
            kappa += derivative_2[i].y;
        }
        kappa /= nodePtIdx - prevNodePtIdx + 1;
        res.kappa.push_back(kappa);

        prevNodePtIdx = nodePtIdx;
    }
    
    // visualization
    if (visualize_path)
    {
        markerArray.markers.clear();

        // create and initialize markers
        visualization_msgs::Marker plannedPathMarker;
        visualization_msgs::Marker scenarioPathCenterMarker;
        visualization_msgs::Marker scenarioPathLeftMarker;
        visualization_msgs::Marker scenarioPathRightMarker;
        
        rosUtilities.initMarker(plannedPathMarker,        lanelet_frame, "planned_path",                visualization_msgs::Marker::LINE_STRIP, rosUtilities.getColorObj(0, 1, 0, 0.2));
        rosUtilities.initMarker(scenarioPathCenterMarker, lanelet_frame, "scenario_path_center",        visualization_msgs::Marker::POINTS,     rosUtilities.getColorObj(1, 0.5, 0, 1));
        rosUtilities.initMarker(scenarioPathLeftMarker,   lanelet_frame, "scenario_path_edge_left",     visualization_msgs::Marker::LINE_STRIP, rosUtilities.getColorObj(1, 0.5, 0, 1), 0.2);
        rosUtilities.initMarker(scenarioPathRightMarker,  lanelet_frame, "scenario_path_edge_right",    visualization_msgs::Marker::LINE_STRIP, rosUtilities.getColorObj(1, 0.5, 0, 1), 0.2);
        
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

        pub_road_lines.publish(markerArray);
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lanelet_handler");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    LaneletHandler lh(nh, nh_p);

    ros::spin();

    return 0;
}

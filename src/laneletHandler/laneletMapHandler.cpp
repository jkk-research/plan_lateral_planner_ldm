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

bool LaneletHandler::init()
{
    std::string lanelet2_path;
    
    // get parameters
    nh_p.param<std::string>("lanelet2_path", lanelet2_path, "");
    nh_p.param<std::string>("lanelet_frame", lanelet_frame, "");
    nh_p.param<std::string>("ego_frame", ego_frame, "");
    nh_p.param<bool>("visualize_path", visualize_path, false);

    // init tf listener
    tfListener = std::make_unique<tf2_ros::TransformListener>(tfBuffer);
    // init gps transform
    ros::Duration(1,0).sleep();
    lanelet_2_map_transform = tfBuffer.lookupTransform(lanelet_frame, "map", ros::Time(0));

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
    // TODO: make from and to inputs (tf / file / parameter)
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
    
    // visualize in rviz if parameter set
    if (visualize_path)
    {
        pub_road_lines = nh.advertise<visualization_msgs::MarkerArray>("paths", 1, true);

        visualization_msgs::Marker laneletCenterlineMarker;

        initMarker(laneletCenterlineMarker, lanelet_frame, "lanelet_center_line", visualization_msgs::Marker::POINTS, getColorObj(0,0,1,1));

        for (int i = 0; i < pathPoints.size(); i++)
        {
            laneletCenterlineMarker.points.push_back(pathPoints[i]);
        }

        markerArray.markers.push_back(laneletCenterlineMarker);
        
        pub_road_lines.publish(markerArray);
    }
    
    // save last point index for faster nearest neighbor search
    lastStartPointIdx = 0;
    nearestNeighborThreshold = 2;

    // init lanelet scenario service
    lanelet_service_ = nh.advertiseService("/get_lanelet_scenario", &LaneletHandler::LaneletScenarioServiceCallback, this);
    
    ROS_INFO("ok");
    return true;
}

double LaneletHandler::distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b)
{
    return sqrt(pow(b.x-a.x, 2)+pow(b.y-a.y, 2));
}

std_msgs::ColorRGBA LaneletHandler::getColorObj(float r, float g, float b, float a)
{
	std_msgs::ColorRGBA c;
    c.r = r;
	c.g = g;
	c.b = b;
	c.a = a;
    return c;
}

void LaneletHandler::initMarker(visualization_msgs::Marker &m, std::string frame_id, std::string ns, int32_t type, std_msgs::ColorRGBA color)
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

Points2D LaneletHandler::getPointOnPoly(float x, PolynomialCoeffs coeffs)
{
    Points2D pt;
    pt.x = x;
    pt.y = coeffs.c0 + coeffs.c1 * x + coeffs.c2 * pow(x, 2) + coeffs.c3 * pow(x, 3);
    return pt;
}

bool LaneletHandler::LaneletScenarioServiceCallback(
    lane_keep_system::GetLaneletScenario::Request &req, 
    lane_keep_system::GetLaneletScenario::Response &res)
{
    int polyline_count = req.nodePointDistances.size();

    geometry_msgs::PoseStamped gps_pose;

    // transform gps coordinates from global frame to lanelet frame
    tf2::doTransform<geometry_msgs::PoseStamped>(req.gps, gps_pose, lanelet_2_map_transform);
    
    // find nearest point to gps postition on path
    int start_point = lastStartPointIdx;
    bool nnTrheshold_reached = false;
    double min_dist = distanceBetweenPoints(gps_pose.pose.position, pathPoints[start_point]);
    for (int i = start_point+1; i < pathPoints.size(); i++)
    {
        double current_dist = distanceBetweenPoints(gps_pose.pose.position, pathPoints[i]);

        if (current_dist < min_dist)
        {
            min_dist = current_dist;
            start_point = i;
        }

        if (current_dist < nearestNeighborThreshold)
            nnTrheshold_reached = true;
        else if (nnTrheshold_reached)
            break;
    }
    lastStartPointIdx = start_point;

    // get centerline pts until scenario_length is reached
    TrajectoryPoints scenarioFull;

    double current_length = 0;
    int prev_pt = start_point;
    int j = start_point + 1;
    std::vector<float> scenarioPointDistances; // distances compared to previous point

    std::vector<int> nodePtIndexes; // node point locations in scenario
    int nodePtIndex = 0;
    
    scenarioFull.push_back(pathPoints[start_point]);
    scenarioPointDistances.push_back(0);
    int scenarioPtCounter = 1;
    bool isLengthReached = false;
    while (j < pathPoints.size() && !isLengthReached)
    {
        if (pathPoints[prev_pt].x == pathPoints[j].x)
        {
            // point duplicate on map
            j++;
            continue;
        }
        
        float currentDistance = distanceBetweenPoints(pathPoints[prev_pt], pathPoints[j]);
        current_length += currentDistance;

        // node point locations in scenario
        if (nodePtIndex < req.nodePointDistances.size())
        {
            if (current_length > req.nodePointDistances[nodePtIndex])
            {
                // first point BEFORE the length criteria
                nodePtIndexes.push_back(scenarioPtCounter - 1);
                nodePtIndex++;
            }

            scenarioPointDistances.push_back(currentDistance);
            scenarioFull.push_back(pathPoints[j]);
        }
        else
        {
            isLengthReached = true;
        }

        prev_pt = j;
        j++;
        scenarioPtCounter++;
    }
    
    // not enough points to plan (path ended)
    if (current_length < req.nodePointDistances.back())
    {
        for (uint8_t i = 0; i < polyline_count; i++)
        {
            PolynomialCoeffs out_coeffs;
            res.coefficients.push_back(out_coeffs);
        }

        return true;
    }

    // transform scenario pts to ego
    geometry_msgs::TransformStamped transformStamped;
    TrajectoryPoints scenarioFull_transformed;
    scenarioFull_transformed.reserve(scenarioFull.size());

    tf2::Quaternion q(
        gps_pose.pose.orientation.x,
        gps_pose.pose.orientation.y,
        gps_pose.pose.orientation.z,
        gps_pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    for (Points2D pt: scenarioFull)
    {
        Points2D transformedPoint;

        Pose2D egopose;
        egopose.Pose2DCoordinates.x = gps_pose.pose.position.x;
        egopose.Pose2DCoordinates.y = gps_pose.pose.position.y;
        egopose.Pose2DTheta = yaw;

        coordinateTransforms.transform2D(pt, egopose, transformedPoint);

        scenarioFull_transformed.push_back(transformedPoint);
    }

    // slice trajectory at nodePoints
    TrajectoryPoints segments[polyline_count];
    PolynomialCoeffs scenarioPolynomes[polyline_count];

    int nodePtIdxStart = 0;
    int segmentIdx = 0;
    for (int nodePtIdx: nodePtIndexes)
    {
        segments[segmentIdx].reserve(nodePtIdx - nodePtIdxStart + 1);

        // add first point without adding length
        segments[segmentIdx].push_back(scenarioFull_transformed[nodePtIdxStart]);

        current_length = 0;
        for (int i = nodePtIdxStart + 1; i <= nodePtIdx; i++)
        {
            current_length += scenarioPointDistances[i];
            segments[segmentIdx].push_back(scenarioFull_transformed[i]);
        }
        scenarioPolynomes[segmentIdx].length = current_length;
        nodePtIdxStart = nodePtIdx;
        segmentIdx++;
    }

    // fit polynomes
    for (uint8_t i = 0; i < polyline_count; i++)
    {
        float polyLength = scenarioPolynomes[i].length;
        scenarioPolynomes[i] = polynomialSubfunctions.fitThirdOrderPolynomial(segments[i]);
        scenarioPolynomes[i].length = polyLength;
        
        res.coefficients.push_back(scenarioPolynomes[i]);
    }

    // calculate kappa
    // scenario first derivative
    std::vector<float> derivative_1;
    derivative_1.reserve(scenarioFull_transformed.size());
    
    TrajectoryPoints::iterator currentPt_it = scenarioFull_transformed.begin();

    // forward differencing first point
    derivative_1.push_back(
        ((currentPt_it + 1)->y - currentPt_it->y) / 
        ((currentPt_it + 1)->x - currentPt_it->x));
    
    currentPt_it++;
    for (; currentPt_it != scenarioFull_transformed.end() - 1; currentPt_it++)
    {
        // central differencing middle points
        derivative_1.push_back(
            ((currentPt_it + 1)->y - (currentPt_it - 1)->y) / 
            ((currentPt_it + 1)->x - (currentPt_it - 1)->x));
    }
    
    // backward differencing last point
    derivative_1.push_back(
        (currentPt_it->y - (currentPt_it - 1)->y) / 
        (currentPt_it->x - (currentPt_it - 1)->x));
    

    // scenario second derivative
    std::vector<float> derivative_2;
    derivative_2.reserve(scenarioFull_transformed.size());
    
    TrajectoryPoints::iterator currentXPt_it = scenarioFull_transformed.begin();
    std::vector<float>::iterator currentY_it = derivative_1.begin();

    // forward differencing first point
    derivative_2.push_back(
        (*(currentY_it + 1) - *currentY_it) / 
        ((currentXPt_it + 1)->x - currentXPt_it->x));
    
    currentXPt_it++;
    for (; currentXPt_it != scenarioFull_transformed.end() - 1; currentXPt_it++)
    {
        // central differencing middle points
        derivative_2.push_back(
            (*(currentY_it + 1) - *(currentY_it - 1)) / 
            ((currentXPt_it + 1)->x - (currentXPt_it - 1)->x));
    }
    
    // backward differencing last point
    derivative_2.push_back(
        (*currentY_it - *(currentY_it - 1)) / 
        (currentXPt_it->x - (currentXPt_it - 1)->x));
    
    // kappa averages
    //TODO: TEST
    nodePtIdxStart = 0;
    for (int nodePtIdx: nodePtIndexes)
    {
        float kappa = 0;
        for (int i = nodePtIdxStart; i <= nodePtIdx; i++)
        {
            kappa += derivative_2[i];
        }
        kappa /= nodePtIdx - nodePtIdxStart + 1;
        nodePtIdxStart = nodePtIdx;

        res.kappa.push_back(kappa);
    }
    
    // visualization
    if (visualize_path)
    {
        markerArray.markers.clear();

        // visualize original and transformed paths
        visualization_msgs::Marker plannedPathMarker;
        visualization_msgs::Marker scenarioPathMarker;
        visualization_msgs::Marker scenarioTransformedMarker;
        visualization_msgs::Marker scenarioSegmentMarker[polyline_count];
        visualization_msgs::Marker polyMarker[polyline_count];

        initMarker(plannedPathMarker, lanelet_frame, "planned_path", visualization_msgs::Marker::LINE_STRIP, getColorObj(0, 1, 0, 0.2));
        initMarker(scenarioPathMarker, lanelet_frame, "scenario_path", visualization_msgs::Marker::LINE_STRIP, getColorObj(1, 0.5, 0, 1));
        initMarker(scenarioTransformedMarker, lanelet_frame, "scenario_transformed", visualization_msgs::Marker::LINE_STRIP, getColorObj(1, 0, 1, 1));

        for (uint8_t i = 0; i < polyline_count; i++)
        {
            initMarker(scenarioSegmentMarker[i], lanelet_frame, ("scenario_segment_"+std::to_string(i+1)), visualization_msgs::Marker::POINTS, getColorObj(i%2, (i/2)%2, 1-i%2, 1));
            initMarker(polyMarker[i], lanelet_frame, ("poly_"+std::to_string(i+1)), visualization_msgs::Marker::LINE_STRIP, getColorObj(i%2, (i/2)%2, 1-i%2, 1));
        }
        
        for (Points2D pt: pathPoints)
        {
            plannedPathMarker.points.push_back(pt);
        }

        for (uint16_t i = 0; i < scenarioFull.size(); i++)
        {
            scenarioPathMarker.points.push_back(scenarioFull[i]);
            
            scenarioTransformedMarker.points.push_back(scenarioFull_transformed[i]);
        }

        for (uint8_t i = 0; i < polyline_count; i++)
        {
            for (Points2D pt: segments[i])
            {
                scenarioSegmentMarker[i].points.push_back(pt);
                
                geometry_msgs::Point p = getPointOnPoly(pt.x, scenarioPolynomes[i]);
                p.z = 0.5;
                polyMarker[i].points.push_back(p);
            }

            markerArray.markers.push_back(scenarioSegmentMarker[i]);
            markerArray.markers.push_back(polyMarker[i]);
        }

        markerArray.markers.push_back(plannedPathMarker);
        markerArray.markers.push_back(scenarioPathMarker);
        markerArray.markers.push_back(scenarioTransformedMarker);

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

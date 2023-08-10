#include "laneletHandler/laneletMapHandler.hpp"

#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/filesystem.hpp>

#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "lane_keep_system/PolynomialCoeffs.h"

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
    nh_p.param<double>("scenario_length", scenario_length, 150);
    nh_p.param<int>("polyline_count", polyline_count, 2);
    nh_p.param<bool>("visualize_path", visualize_path, false);

    // init tf listener
    tfListener = std::make_unique<tf2_ros::TransformListener>(tfBuffer);

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
    lanelet::projection::MGRSProjector projector;
    lanelet::LaneletMapPtr map = lanelet::load(lanelet2_file_path, projector, &errors);
    
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
    roadLanelets = lanelet::utils::query::roadLanelets(lanelet::utils::query::laneletLayer(map));
    
    // Initialize path planning
    lanelet::traffic_rules::TrafficRulesPtr trafficRules{lanelet::traffic_rules::TrafficRulesFactory::instance().create(lanelet::Locations::Germany, lanelet::Participants::Vehicle)};
    lanelet::routing::RoutingGraphPtr graph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);

    // path planning
    // TODO: make from and to inputs (tf / file / parameter)
    lanelet::Optional<lanelet::routing::LaneletPath> trajectory_path = graph->shortestPath(map->laneletLayer.get(12560), map->laneletLayer.get(12607));

    // collect points from planned path
    pathPoints.clear();
    for (auto lane = trajectory_path.get().begin(); lane != trajectory_path.get().end(); lane++)
    {
        for (auto pt = lane->centerline().begin(); pt != lane->centerline().end(); pt++)
        {
            Rb::Vmc::Points2D p;
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
    
    // init lanelet scenario service
    lanelet_service_ = nh.advertiseService("/get_lanelet_scenario", &LaneletHandler::LaneletScenarioServiceCallback, this);

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

bool LaneletHandler::LaneletScenarioServiceCallback(
    lane_keep_system::GetLaneletScenario::Request &req, 
    lane_keep_system::GetLaneletScenario::Response &res)
{
    // get GPS data directly from topic
    geometry_msgs::PoseStamped gps_msg = *(ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/gps/duro/current_pose"));

    // find nearest point to gps postition on path
    int start_point = 0;
    double min_dist = distanceBetweenPoints(gps_msg.pose.position, pathPoints[start_point]);
    for (int i = start_point+1; i < pathPoints.size(); i++)
    {
        double current_dist = distanceBetweenPoints(gps_msg.pose.position, pathPoints[i]);

        if (current_dist < min_dist)
        {
            min_dist = current_dist;
            start_point = i;
        }
    }

    // get centerline pts until scenario_lenght is reached
    Rb::Vmc::TrajectoryPoints scenarioFull;

    double current_length = 0;
    int prev_pt = start_point;
    int j = start_point + 1;
    while (j < pathPoints.size() && current_length < scenario_length)
    {
        current_length += distanceBetweenPoints(pathPoints[prev_pt], pathPoints[j]);

        scenarioFull.push_back(pathPoints[j]);

        prev_pt = j;
        j++;
    }
    
    // not enough points to plan (path ended)
    if (current_length < scenario_length)
    {
        for (uint8_t i = 0; i < polyline_count; i++)
        {
            lane_keep_system::PolynomialCoeffs out_coeffs;
            res.coefficients.push_back(out_coeffs);
        }

        return true;
    }
    
    // transform scenario pts to ego
    geometry_msgs::TransformStamped transformStamped;
    Rb::Vmc::TrajectoryPoints scenarioFull_transformed;
    try
    {
        geometry_msgs::TransformStamped lanelet2ego_transform = tfBuffer.lookupTransform(ego_frame, lanelet_frame, ros::Time(0));
        for (Rb::Vmc::Points2D pt: scenarioFull)
        {
            geometry_msgs::Point p;
            tf2::doTransform<geometry_msgs::Point>(pt, p, lanelet2ego_transform);
            scenarioFull_transformed.push_back(p);
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        return false;
    }
    
    // slice trajectory into equal segments
    Rb::Vmc::TrajectoryPoints segments[polyline_count];
    lane_keep_system::PolynomialCoeffs scenarioPolynomes[polyline_count];
    uint16_t prev_pt_idx = 0;
    int segmentSeparationSize = floor(scenarioFull_transformed.size() / (float)polyline_count);
    
    for (uint8_t i = 0; i < polyline_count - 1; i++)
    {
        for (uint8_t j = i*segmentSeparationSize; j <= (i+1)*segmentSeparationSize; j++)
        {
            Rb::Vmc::Points2D pt = scenarioFull_transformed[j];
            scenarioPolynomes[i].length += distanceBetweenPoints(scenarioFull_transformed[prev_pt_idx], pt);
            
            segments[i].push_back(pt);

            prev_pt_idx = j;
        }
    }
    for (uint8_t j = (polyline_count - 1)*segmentSeparationSize; j < scenarioFull_transformed.size(); j++)
    {
        Rb::Vmc::Points2D pt = scenarioFull_transformed[j];
        scenarioPolynomes[polyline_count - 1].length += distanceBetweenPoints(scenarioFull_transformed[prev_pt_idx], pt);
            
        segments[polyline_count - 1].push_back(pt);

        prev_pt_idx = j;
    }
    
    // fit polynomes
    for (uint8_t i = 0; i < polyline_count; i++)
    {
        Rb::Vmc::PolynomialCoeffs out_coeffs = polynomialSubfunctions.fitThirdOrderPolynomial(segments[i]);
        scenarioPolynomes[i].c0 = out_coeffs.c0;
        scenarioPolynomes[i].c1 = out_coeffs.c1;
        scenarioPolynomes[i].c2 = out_coeffs.c2;
        scenarioPolynomes[i].c3 = out_coeffs.c3;

        res.coefficients.push_back(scenarioPolynomes[i]);
    }
    

    if (visualize_path)
    {
        // visualize original and transformed paths
        visualization_msgs::Marker plannedPathMarker;
        visualization_msgs::Marker scenarioPathMarker;
        visualization_msgs::Marker scenarioTransformedMarker;
        visualization_msgs::Marker scenarioSegmentMarker[polyline_count];
        visualization_msgs::Marker polyMarker[polyline_count];

        initMarker(plannedPathMarker, lanelet_frame, "planned_path", visualization_msgs::Marker::LINE_STRIP, getColorObj(1, 0, 1, 0.5));
        initMarker(scenarioPathMarker, lanelet_frame, "scenario_path", visualization_msgs::Marker::LINE_STRIP, getColorObj(0, 1, 0, 1));
        initMarker(scenarioTransformedMarker, lanelet_frame, "scenario_transformed", visualization_msgs::Marker::LINE_STRIP, getColorObj(1, 0, 0, 1));

        for (uint8_t i = 0; i < polyline_count; i++)
        {
            initMarker(scenarioSegmentMarker[i], lanelet_frame, ("scenario_segment_"+std::to_string(i+1)), visualization_msgs::Marker::POINTS, getColorObj(0, 1, 1-i%2, 1));
            initMarker(polyMarker[i], lanelet_frame, ("poly_"+std::to_string(i+1)), visualization_msgs::Marker::LINE_STRIP, getColorObj(0, 1, i%2, 1));
        }
        
        for (Rb::Vmc::Points2D pt: pathPoints)
        {
            plannedPathMarker.points.push_back(pt);
        }

        for (uint8_t i = 0; i < scenarioFull.size(); i++)
        {
            scenarioPathMarker.points.push_back(scenarioFull[i]);
            
            scenarioTransformedMarker.points.push_back(scenarioFull_transformed[i]);
        }

        for (uint8_t i = 0; i < polyline_count; i++)
        {
            for (Rb::Vmc::Points2D pt: segments[i])
            {
                scenarioSegmentMarker[i].points.push_back(pt);
                
                geometry_msgs::Point p;

                p.x = pt.x;
                p.y = scenarioPolynomes[i].c0 + scenarioPolynomes[i].c1 * p.x + scenarioPolynomes[i].c2 * pow(p.x, 2) + scenarioPolynomes[i].c3 * pow(p.x, 3);
                p.z = 0.5;
                polyMarker[i].points.push_back(p);
            }

            markerArray.markers.push_back(scenarioSegmentMarker[i]);
            markerArray.markers.push_back(polyMarker[i]);
        }

        markerArray.markers.push_back(plannedPathMarker);
        markerArray.markers.push_back(scenarioPathMarker);
        markerArray.markers.push_back(scenarioTransformedMarker);
    }

    pub_road_lines.publish(markerArray);

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

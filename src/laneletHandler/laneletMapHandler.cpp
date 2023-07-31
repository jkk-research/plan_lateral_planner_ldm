#include "laneletHandler/laneletMapHandler.hpp"

#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/filesystem.hpp>

#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "lane_keep_system/PolynomCoeffs.h"

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
    road_lanelets = lanelet::utils::query::roadLanelets(lanelet::utils::query::laneletLayer(map));
    
    // init lanelet scenario service
    lanelet_service_ = nh.advertiseService("/get_lanelet_scenario", &LaneletHandler::LaneletScenarioServiceCallback, this);
    
    // Initialize path planning
    lanelet::traffic_rules::TrafficRulesPtr trafficRules{lanelet::traffic_rules::TrafficRulesFactory::instance().create(lanelet::Locations::Germany, lanelet::Participants::Vehicle)};
    lanelet::routing::RoutingGraphPtr graph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);

    // path planning
    // TODO: make from and to inputs (tf / file / parameter)
    lanelet::Optional<lanelet::routing::LaneletPath> trajectory_path = graph->shortestPath(map->laneletLayer.get(12560), map->laneletLayer.get(12607));

    // collect points from planned path
    path_pts.clear();
    for (auto lane = trajectory_path.get().begin(); lane != trajectory_path.get().end(); lane++)
    {
        for (auto pt = lane->centerline().begin(); pt != lane->centerline().end(); pt++)
        {
            geometry_msgs::Point p;
            p.x = pt->x();
            p.y = pt->y();
            path_pts.push_back(p);
        }
    }
    
    // visualize in rviz if parameter set
    if (visualize_path)
    {
        pub_road_lines = nh.advertise<visualization_msgs::MarkerArray>("lanelet_road_center", 1, true);

        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker centerLineMarker;

        centerLineMarker.header.frame_id = lanelet_frame;
        centerLineMarker.header.stamp = ros::Time::now();
        centerLineMarker.lifetime = ros::Duration(0);
        centerLineMarker.ns = "lanelet_center_lane";
        centerLineMarker.type = visualization_msgs::Marker::LINE_STRIP;
        centerLineMarker.scale.x = 0.3;
        centerLineMarker.scale.y = 0.3;
        centerLineMarker.scale.z = 0.3;
        centerLineMarker.color.r = 0;
        centerLineMarker.color.g = 0;
        centerLineMarker.color.b = 1;
        centerLineMarker.color.a = 0.5;

        for (auto pt: path_pts)
        {
            geometry_msgs::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = 0;
            centerLineMarker.points.push_back(p);
        }

        markerArray.markers.push_back(centerLineMarker);
        
        pub_road_lines.publish(markerArray);
    }

    return true;
}

double LaneletHandler::DistanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b)
{
    return sqrt(pow(b.x-a.x, 2)+pow(b.y-a.y, 2));
}

bool LaneletHandler::LaneletScenarioServiceCallback(
    lane_keep_system::GetLaneletScenario::Request &req, 
    lane_keep_system::GetLaneletScenario::Response &res)
{
    // get GPS data directly from topic
    geometry_msgs::PoseStamped gps_msg = *(ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/gps/duro/current_pose"));

    // find nearest point to gps postition on path
    Rb::Vmc::TrajectoryPoints::iterator start_point_it = path_pts.begin();
    double min_dist = DistanceBetweenPoints(gps_msg.pose.position, *start_point_it);
    for (auto pt_it = path_pts.begin() + 1; pt_it != path_pts.end(); pt_it++)
    {
        double current_dist = DistanceBetweenPoints(gps_msg.pose.position, *pt_it);

        if (current_dist < min_dist)
        {
            min_dist = current_dist;
            start_point_it = pt_it;
        }
    }

    // get centerline pts until scenario_lenght is reached
    Rb::Vmc::TrajectoryPoints scenarioFull;
    scenarioFull.push_back(*start_point_it);

    double current_length = 0;
    Rb::Vmc::TrajectoryPoints::iterator prev_pt_it = start_point_it;
    Rb::Vmc::TrajectoryPoints::iterator pt_it = start_point_it + 1;
    while (pt_it != path_pts.end() && current_length < scenario_length)
    {
        current_length += DistanceBetweenPoints(*prev_pt_it, *pt_it);

        scenarioFull.push_back(*pt_it);

        prev_pt_it = pt_it;
        pt_it++;
    }
    
    // transform scenario pts to ego
    // TODO: make listener global
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Duration d(1, 0);
    d.sleep();

    geometry_msgs::TransformStamped transformStamped;
    Rb::Vmc::TrajectoryPoints scenarioFull_transformed;
    try
    {
        geometry_msgs::TransformStamped lanelet2ego_transform = tfBuffer.lookupTransform(ego_frame, lanelet_frame, ros::Time(0));
        for (auto pt_it = scenarioFull.begin(); pt_it != scenarioFull.end(); pt_it++)
        {
            geometry_msgs::Point p;
            tf2::doTransform<geometry_msgs::Point>(*pt_it, p, lanelet2ego_transform);
            scenarioFull_transformed.push_back(p);
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    // slice trajectory into equal segments
    Rb::Vmc::TrajectorySegment segments[polyline_count];
    Rb::Vmc::PolynomialCoeffs scenarioPolynomes[polyline_count];
    uint8_t polyline_counter = 0;
    Rb::Vmc::Points2D prev_pt;

    scenarioPolynomes[0].offset = scenarioFull_transformed[0];
    for (uint8_t i = 0; i < scenarioFull_transformed.size(); i++)
    {
        if (i < round(scenarioFull_transformed.size() / polyline_count) * (polyline_counter + 1))
        {
            Rb::Vmc::Points2D p;
            p.x = scenarioFull_transformed[i].x;
            p.y = scenarioFull_transformed[i].y;
            scenarioPolynomes[polyline_counter].length += DistanceBetweenPoints(prev_pt, p);
            
            prev_pt = p;

            p.x -= scenarioPolynomes[polyline_counter].offset.x;
            p.y -= scenarioPolynomes[polyline_counter].offset.y;
            segments[polyline_counter].points.push_back(p);
        }
        else if (polyline_counter < polyline_count)
        {
            polyline_counter++;
            
            Rb::Vmc::Points2D p;
            p.x = scenarioFull_transformed[i].x;
            p.y = scenarioFull_transformed[i].y;
            scenarioPolynomes[polyline_counter - 1].length += DistanceBetweenPoints(prev_pt, p);
            prev_pt = p;

            p.x -= scenarioPolynomes[polyline_counter - 1].offset.x;
            p.y -= scenarioPolynomes[polyline_counter - 1].offset.y;
            segments[polyline_counter - 1].points.push_back(p);

            // set offset as the first point of the segment
            scenarioPolynomes[polyline_counter].offset = scenarioFull_transformed[i];

            p.x = scenarioFull_transformed[i].x - scenarioPolynomes[polyline_counter].offset.x;
            p.y = scenarioFull_transformed[i].y - scenarioPolynomes[polyline_counter].offset.y;

            segments[polyline_counter].points.push_back(p);
        }
        else
        {
            Rb::Vmc::Points2D p;
            p.x = scenarioFull_transformed[i].x;
            p.y = scenarioFull_transformed[i].y;
            scenarioPolynomes[polyline_counter - 1].length += DistanceBetweenPoints(prev_pt, p);

            p.x -= scenarioPolynomes[polyline_counter].offset.x;
            p.y -= scenarioPolynomes[polyline_counter].offset.y;
            segments[polyline_count - 1].points.push_back(p);

            prev_pt = p;
        }
    }
    
    // fit polynomes
    for (uint8_t i = 0; i < polyline_count; i++)
    {
        Rb::Vmc::PolynomialCoeffs out_coeffs = polynomialSubfunctions.fitThirdOrderPolynomial(segments[i].points);
        scenarioPolynomes[i].c0 = out_coeffs.c0;
        scenarioPolynomes[i].c1 = out_coeffs.c1;
        scenarioPolynomes[i].c2 = out_coeffs.c2;
        scenarioPolynomes[i].c3 = out_coeffs.c3;

        lane_keep_system::PolynomCoeffs pc;
        pc.c0 = out_coeffs.c0;
        pc.c1 = out_coeffs.c1;
        pc.c2 = out_coeffs.c2;
        pc.c3 = out_coeffs.c3;
        pc.length = scenarioPolynomes[i].length;
        pc.offset = scenarioPolynomes[i].offset;

        res.coefficients.push_back(pc);
    }
    
    if (visualize_path)
    {
        // visualize original and transformed paths
        visualization_msgs::Marker plannedPathMarker;
        visualization_msgs::Marker scenarioPathMarker;
        visualization_msgs::Marker scenarioTransformedMarker;
        visualization_msgs::Marker polyMarker[polyline_count];

        plannedPathMarker.header.frame_id = lanelet_frame;
        plannedPathMarker.header.stamp = ros::Time::now();
        plannedPathMarker.lifetime = ros::Duration(0);
        plannedPathMarker.ns = "planned_path";
        plannedPathMarker.type = visualization_msgs::Marker::LINE_STRIP;
        plannedPathMarker.scale.x = 0.3;
        plannedPathMarker.scale.y = 0.3;
        plannedPathMarker.scale.z = 0.3;
        plannedPathMarker.color.r = 1;
        plannedPathMarker.color.g = 0;
        plannedPathMarker.color.b = 1;
        plannedPathMarker.color.a = 0.5;

        scenarioPathMarker = plannedPathMarker;
        scenarioPathMarker.ns = "scenario_path";
        scenarioPathMarker.color.r = 0;
        scenarioPathMarker.color.g = 1;
        scenarioPathMarker.color.b = 0;
        scenarioPathMarker.color.a = 1;

        scenarioTransformedMarker = plannedPathMarker;
        scenarioTransformedMarker.ns = "scenario_transformed";
        scenarioTransformedMarker.color.r = 1;
        scenarioTransformedMarker.color.g = 0;
        scenarioTransformedMarker.color.b = 0;
        scenarioTransformedMarker.color.a = 1;

        for (uint8_t i = 0; i < polyline_count; i++)
        {
            polyMarker[i] = plannedPathMarker;
            polyMarker[i].ns = "scenario_segment_" + std::to_string(i+1);
            polyMarker[i].color.r = 0;
            polyMarker[i].color.g = 1;
            polyMarker[i].color.b = i%2;
            polyMarker[i].color.a = 1;
        }
        
        for (auto pt: path_pts)
        {
            geometry_msgs::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = -1;
            plannedPathMarker.points.push_back(p);
        }

        for (uint8_t i = 0; i < scenarioFull.size(); i++)
        {
            scenarioPathMarker.points.push_back(scenarioFull[i]);
            
            scenarioTransformedMarker.points.push_back(scenarioFull_transformed[i]);
        }

        for (uint8_t i = 0; i < polyline_count; i++)
        {
            for (uint8_t j = 0; j < segments[i].points.size(); j++)
            {
                geometry_msgs::Point p;
                p.x = scenarioPolynomes[i].offset.x + segments[i].points[j].x;
                p.y = scenarioPolynomes[i].offset.y + scenarioPolynomes[i].c0 + scenarioPolynomes[i].c1 * segments[i].points[j].x + scenarioPolynomes[i].c2 * pow(segments[i].points[j].x, 2) + scenarioPolynomes[i].c3 * pow(segments[i].points[j].x, 3);
                p.z = 0.5;
                polyMarker[i].points.push_back(p);
            }

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
    ros::init(argc, argv, "lanelet_handler_handler");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    LaneletHandler lh(nh, nh_p);

    ros::spin();

    return 0;
}

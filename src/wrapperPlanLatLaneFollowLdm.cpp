#include "../include/wrapperPlanLatLaneFollowLdm.hpp"


crp::apl::PlanLatLaneFollowHandler::PlanLatLaneFollowHandler() : WrapperBase("plan_lat_lane_follow")
{
}


void crp::apl::PlanLatLaneFollowHandler::plan(const PlannerInput & input, PlannerOutput & output)
{
    // run path planner including LDm
    TrajectoryOutput trajectoryOutput = m_linearDriverModel.runCoeffsLite(
        m_scenarioPolynomials,
        m_egoPose,
        m_ldmParams
    );
}

// method calculates the subsegments of the incoming trajectory based on a LDMParamIn type parameters struct
void crp::apl::PlanLatLaneFollowHandler::calculateNodePoints(
    const PlannerInput & input)
{

}

void crp::apl::PlanLatLaneFollowHandler::fitPolynomialOnWaypoints(
    const PlannerInput & input)
{
    std::vector<float> x;
    std::vector<float> y;
    for (int i=0; i<input.path.pathPoints.size(); i++)
    {
        x.push_back(input.path.pathPoints.at(i).pose.position.x);
        y.push_back(input.path.pathPoints.at(i).pose.position.y);
    }
    std::vector<float> coefficients = m_polynomialRegressor.fitThirdOrderPolynomial(x,y);
    
}

void crp::apl::PlanLatLaneFollowHandler::generateEgoPose(
    const PlannerInput & input)
{
    m_egoPose.Pose2DCoordinates.x = input.egoPose.position.x;
    m_egoPose.Pose2DCoordinates.y = input.egoPose.position.y;
    m_egoPose.Pose2DTheta = input.egoPose.orientation;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<crp::apl::PlanLatLaneFollowHandler>());
    rclcpp::shutdown();
    return 0;
}

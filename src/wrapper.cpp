#include "../include/wrapper.hpp"


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


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<crp::apl::PlanLatLaneFollowHandler>());
    rclcpp::shutdown();
    return 0;
}

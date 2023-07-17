///=============================================================================
///  C O P Y R I G H T
///-----------------------------------------------------------------------------
/// @copyright (c) 2022 by Robert Bosch GmbH. All rights reserved.
///
///  The reproduction, distribution and utilization of this file as
///  well as the communication of its contents to others without express
///  authorization is prohibited. Offenders will be held liable for the
///  payment of damages. All rights reserved in the event of the grant
///  of a patent, utility model or design.
///  @file
///=============================================================================

#ifndef DC_EMG_LINEARDRIVERMODEL_TRAJECTORYPLANNER_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_TRAJECTORYPLANNER_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

#include "../emg_linearDriverModel_interfaces.hpp"
#include "../linearDriverModelPlanner/emg_linearDriverModel_segmentPlanner.hpp"
#include "../linearDriverModelPlanner/emg_linearDriverModel_trajectoryEvaluation.hpp"


namespace Dc
{
namespace Emg
{

class TrajectoryPlanner
{
public:
   // main method
   void calc(const NodePoints&, const EvalPoints&, SegmentParams&, TrajectoryPoints&);
   void calc(const NodePoints&, const EvalPoints&, PolynomialCoeffs&, SegmentParams&, TrajectoryPoints&);

private:
   SegmentPlanner       segmentPlanner{};
   TrajectoryEvaluation trajectoryEvaluation{};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_TRAJECTORYPLANNER_HPP_INCLUDED

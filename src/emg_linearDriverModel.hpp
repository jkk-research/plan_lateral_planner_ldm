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

#ifndef DC_EMG_LINEARDRIVERMODEL_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/container/vfc_fixedvector.hpp"
#include "vfc/core/vfc_functionattributes.hpp"
#include "vfc/container/vfc_carray.hpp"

#include "emg_linearDriverModel_interfaces.hpp"
#include "emg_linearDriverModel_controlLogic.hpp"
#include "LinearDriverModelDriverModel/emg_linearDriverModel_driverModel.hpp"
#include "LinearDriverModelPlanner/emg_linearDriverModel_segmentPlanner.hpp"
#include "LinearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.hpp"

// >>>IntecrioMeasurement<<<
#if !defined _MSC_VER && !defined(__GNUC__)
extern "C"
{
#endif
#include "vmc/runnables/ES910Wrapper/linearDriverModel.h"
#if !defined _MSC_VER && !defined(__GNUC__)
}
#endif

namespace Rb
{
namespace Vmc
{

class LinearDriverModel  // cover class of DriverTrajectoryPlanner
{
public:
   PolynomialCoeffsTwoSegments runCoeffsLite(
      const CorridorInfoCoefficients&, const Pose2D&, const LDMParamIn&);

   // Control Logic
   ControlLogic controlLogic{};

   // Driver Model
   DriverModel driverModel{};

   // internal input and output structs
   NodePoints nodePoints{};
   NodePoints nodePointsEgoFrame{};
   Pose2D     egoPoseGlobalPlan{};
   // Evaluation Points calculation for trajectory planner
   // Preprocess

private:
   // Transforms
   CoordinateTransforms coordinateTransforms{};

   // Curve fitting - segment planner
   SegmentPlanner    segmentPlanner{};
   SegmentParams        segmentParams{};
   SegmentParams        segmentParamsEgoFrame{};

   // Results of the curve fitting and return of coefficients towards TRC
   PolynomialCoeffsThreeSegments trajectoryCoeffsThreeSegments{};
   PolynomialCoeffsTwoSegments coefficientsToController{};

   // internal variables
   bool                     firstCycle{true};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_HPP_INCLUDED

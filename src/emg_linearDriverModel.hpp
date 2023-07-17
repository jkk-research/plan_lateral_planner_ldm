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
#include "linearDriverModelDriverModel/emg_linearDriverModel_driverModel.hpp"
#include "linearDriverModelPlanner/emg_linearDriverModel_trajectoryPlanner.hpp"
#include "linearDriverModelUtilities/emg_linearDriverModel_coordinateTransforms.hpp"


namespace Dc
{
namespace Emg
{

class LinearDriverModel  // cover class of DriverTrajectoryPlanner
{
public:
   void                 init();  // might be deleted
   TrajectoryPoints     run(const CorridorInfo&, const Pose2D&, const LDMParamIn&);
   TrajectoryPolynomial runPoly(const CorridorInfo&, const Pose2D&, const LDMParamIn&);
   // cover function for simulink
   TrajectoryPolynomial runPoly(
      vfc::CSI::si_metre_f32_t     corrX[300],
      vfc::CSI::si_metre_f32_t     corrY[300],
      vfc::CSI::si_radian_f32_t    corrC1[300],
      vfc::CSI::si_per_metre_f32_t corrC2[300],
      vfc::uint16_t                corrLength,
      vfc::CSI::si_metre_f32_t     egoPoseX,
      vfc::CSI::si_metre_f32_t     egoPoseY,
      vfc::CSI::si_radian_f32_t    egoPoseTheta,
      vfc::CSI::si_metre_f32_t     LAD,
      vfc::uint8_t                 replanCycle,
      vfc::float32_t               P[21],
      vfc::float32_t               P_nodePointDistances[3]);
   PolynomialCoeffs runCoeffs(const CorridorInfo&, const Pose2D&, const LDMParamIn&);
   // cover function for simulink
   PolynomialCoeffs runCoeffs(
      vfc::CSI::si_metre_f32_t     corrX[300],
      vfc::CSI::si_metre_f32_t     corrY[300],
      vfc::CSI::si_radian_f32_t    corrC1[300],
      vfc::CSI::si_per_metre_f32_t corrC2[300],
      vfc::uint16_t                corrLength,
      vfc::CSI::si_metre_f32_t     egoPoseX,
      vfc::CSI::si_metre_f32_t     egoPoseY,
      vfc::CSI::si_radian_f32_t    egoPoseTheta,
      vfc::CSI::si_metre_f32_t     LAD,
      vfc::uint8_t                 replanCycle,
      vfc::float32_t               P[21],
      vfc::float32_t               P_nodePointDistances[3]);
   PolynomialCoeffsThreeSegments runCoeffsLite(
      const CorridorInfoCoefficients&, const Pose2D&, const LDMParamIn&);
   // cover function for simulink S-function
   vfc::CSI::si_metre_f32_t* run(
      vfc::CSI::si_metre_f32_t     corrX[300],
      vfc::CSI::si_metre_f32_t     corrY[300],
      vfc::CSI::si_radian_f32_t    corrC1[300],
      vfc::CSI::si_per_metre_f32_t corrC2[300],
      vfc::uint16_t                corrLength,
      vfc::CSI::si_metre_f32_t     egoPoseX,
      vfc::CSI::si_metre_f32_t     egoPoseY,
      vfc::CSI::si_radian_f32_t    egoPoseTheta,
      vfc::CSI::si_metre_f32_t     LAD,
      vfc::uint8_t                 replanCycle,
      vfc::float32_t               P[21],
      vfc::float32_t               P_nodePointDistances[3]);
   CorridorInfo corridor{};
   LDMParamIn   params{};
   Pose2D       egoPoseGlob{};
   // Control Logic
   ControlLogic controlLogic{};
   // Driver Model
   DriverModel driverModel{};
   // internal input and output structs
   EvalPoints evalPoints{};
   NodePoints nodePoints{};
   NodePoints nodePointsGlobalFrame{};
   NodePoints nodePointsEgoFrame{};
   Pose2D     plannerFramePoseGlobalFrame{};
   Pose2D     previousPlannerFrame{};
   Pose2D     egoPoseGlobalPlan{};
   // Evaluation Points calculation for trajectory planner
   void evalPointsPreprocess(vfc::uint16_t[4], const CorridorInfo&, EvalPoints&);
   // Preprocess
   void getPlannerFrame(
      const Pose2D&,
      const LDMParamIn&,
      const NodePoints&,
      const SegmentParams&,
      const TrajectoryPoints&,
      const TrajectoryPoints&);

private:
   // Transforms
   CoordinateTransforms   coordinateTransforms{};
   PolynomialSubfunctions polynomialSubfunctions{};
   // TrajectoryPlanner
   TrajectoryPlanner trajectoryPlanner{};
   SegmentPlanner    segmentPlanner{};
   // internal input and output structs
   CorridorInfo                  corridorPlannerFrame{};
   CorridorInfo                  corridorGlobalFrame{};
   CorridorInfoCoefficients      corridorCoefficientsGlobal{};
   TrajectoryPoints              trajectoryPlannerFrame{};
   TrajectoryPoints              previousTrajectoryPlannerFrame{};
   TrajectoryPoints              trajectoryGlobalFrame{};
   TrajectoryPoints              trajectoryEgoFrame{};
   TrajectoryPoints              trajectoryEmpty{};
   Pose2D                        tempPlannerFrame{};
   SegmentParams                 segmentParams{};
   SegmentParams                 segmentParamsEgoFrame{};
   PolynomialCoeffs              polynomialCoeffs{};
   PolynomialCoeffs              trajectoryCoeffs{};
   PolynomialCoeffs              trajectoryCoeffsEmpty{};
   PolynomialCoeffsThreeSegments trajectoryCoeffsThreeSegments{};
   PolynomialCoeffsThreeSegments trajectoryCoeffsThreeSegmentsEmpty{};
   TrajectoryPolynomial          trajectoryPolynomial{};
   TrajectoryPolynomial          trajectoryPolynomialEmpty{};
   // internal variables
   vfc::float32_t           diffTrajX{0.0f};
   vfc::float32_t           diffTrajY{0.0f};
   vfc::float32_t           distance{0.0f};
   vfc::float32_t           arcLength{0.0f};
   vfc::int16_t             segmentId{0};
   bool                     firstCycle{true};
   bool                     triggerPlanner{false};
   vfc::CSI::si_metre_f32_t trajEmpty[601]{static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_HPP_INCLUDED

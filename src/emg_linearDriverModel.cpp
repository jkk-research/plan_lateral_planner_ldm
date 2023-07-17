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

#include "emg_linearDriverModel.hpp"

namespace Dc
{
namespace Emg
{

void LinearDriverModel::init()
{
}

vfc::CSI::si_metre_f32_t* LinearDriverModel::run(
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
   vfc::float32_t               P_nodePointDistances[3])
{
   // parameter init
   for (uint16_t i{0}; i < 300; i++)
   {
      corridor.corridorCurvature[i]      = corrC2[i];
      corridor.corridorOrientation[i]    = corrC1[i];
      corridor.corridorPoints[i].PointsX = corrX[i];
      corridor.corridorPoints[i].PointsY = corrY[i];
      corridor.corridorLength            = corrLength;
   }

   for (uint8_t i{0}; i < 21; i++)
   {
      params.P[i] = P[i];
   }
   params.replanCycle       = replanCycle;
   params.lookAheadDistance = LAD;
   for (uint8_t i{0}; i < 3; i++)
   {
      params.P_nodePointDistances[i] = P_nodePointDistances[i];
   }

   egoPoseGlob.Pose2DCoordinates.PointsX = egoPoseX;
   egoPoseGlob.Pose2DCoordinates.PointsY = egoPoseY;
   egoPoseGlob.Pose2DTheta               = egoPoseTheta;

   // here starts the main function
   // transforming CorridorInfo from Ego to Global frame
   memset(corridorGlobalFrame.corridorPoints, 0.0f, sizeof(corridorGlobalFrame.corridorPoints));
   memset(corridorGlobalFrame.corridorOrientation, 0.0f, sizeof(corridorGlobalFrame.corridorOrientation));
   memset(corridorGlobalFrame.corridorCurvature, 0.0f, sizeof(corridorGlobalFrame.corridorCurvature));

   coordinateTransforms.reverseTransform2D(
      corridor.corridorPoints, egoPoseGlob, corridor.corridorLength, corridorGlobalFrame.corridorPoints);
   coordinateTransforms.angleTransform2D(corridor, egoPoseGlob, corridor.corridorLength, corridorGlobalFrame);
   corridorGlobalFrame.corridorLength = corridor.corridorLength;

   // calculating trigger for Planner activation
   bool triggerPlanner =
      controlLogic.calcValidity(corridor, corridorGlobalFrame, egoPoseGlob, params, firstCycle);

   if (controlLogic.validity && triggerPlanner)
   {
      firstCycle = false;
   }

   if (controlLogic.validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         // calculate the actual Planner frame
         getPlannerFrame(
            egoPoseGlob,
            params,
            nodePoints,
            segmentParams,
            trajectoryEgoFrame,
            previousTrajectoryPlannerFrame);
         // storing the value for the next replan cycle to calculate the next planner frame

         previousPlannerFrame = plannerFramePoseGlobalFrame;

         // transforming CorridorInfo from Global to Planner frame
         memset(corridorPlannerFrame.corridorPoints, 0.0f, sizeof(corridorPlannerFrame.corridorPoints));
         memset(
            corridorPlannerFrame.corridorOrientation, 0.0f, sizeof(corridorPlannerFrame.corridorOrientation));
         memset(corridorPlannerFrame.corridorCurvature, 0.0f, sizeof(corridorPlannerFrame.corridorCurvature));

         coordinateTransforms.transform2D(
            corridorGlobalFrame.corridorPoints,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame.corridorPoints);

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         coordinateTransforms.angleTransform2D(
            corridorGlobalFrame,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame);
         corridorPlannerFrame.corridorLength = corridorGlobalFrame.corridorLength;

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         // calculating the Node Points
         driverModel.calc(corridorPlannerFrame, params, nodePoints);

         // calculating the Evaluation Points for the trajectory planner
         evalPointsPreprocess(driverModel.indices, corridorPlannerFrame, evalPoints);

         // calculating the clothoid parameters for each segments, then evaluating them
         memset(
            trajectoryPlannerFrame.trajectoryPoints, 0.0f, sizeof(trajectoryPlannerFrame.trajectoryPoints));
         trajectoryPlannerFrame.trajectoryLength = 0U;

         trajectoryPlanner.calc(nodePoints, evalPoints, segmentParams, trajectoryPlannerFrame);

         memset(
            previousTrajectoryPlannerFrame.trajectoryPoints,
            0.0f,
            sizeof(previousTrajectoryPlannerFrame.trajectoryPoints));
         previousTrajectoryPlannerFrame.trajectoryLength = 0U;
         for (uint16_t i{0}; i < 300; i++)
         {
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsX =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsX;
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsY =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsY;
         }
         previousTrajectoryPlannerFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      }

      // transforming trajectory from Planner to Global frame
      memset(trajectoryGlobalFrame.trajectoryPoints, 0.0f, sizeof(trajectoryGlobalFrame.trajectoryPoints));

      coordinateTransforms.reverseTransform2D(
         trajectoryPlannerFrame.trajectoryPoints,
         plannerFramePoseGlobalFrame,
         trajectoryPlannerFrame.trajectoryLength,
         trajectoryGlobalFrame.trajectoryPoints);
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      // transforming trajectory from Global to Ego frame
      memset(trajectoryEgoFrame.trajectoryPoints, 0.0f, sizeof(trajectoryEgoFrame.trajectoryPoints));

      coordinateTransforms.transform2D(
         trajectoryGlobalFrame.trajectoryPoints,
         egoPoseGlob,
         trajectoryGlobalFrame.trajectoryLength,
         trajectoryEgoFrame.trajectoryPoints);
      trajectoryEgoFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;

      vfc::CSI::si_metre_f32_t trajEgoFrame[601]{static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};
      for (uint16_t i{0}; i < 300; i++)
      {
         trajEgoFrame[i]       = trajectoryEgoFrame.trajectoryPoints[i].PointsX;
         trajEgoFrame[i + 300] = trajectoryEgoFrame.trajectoryPoints[i].PointsY;
      }
      trajEgoFrame[600] = static_cast<vfc::CSI::si_metre_f32_t>(trajectoryEgoFrame.trajectoryLength);
      return trajEgoFrame;
   }
   // If the corridor is not valid return trajectory points of zeros
   else
   {
      return trajEmpty;
   }
}


TrajectoryPoints LinearDriverModel::run(
   const CorridorInfo& corridorEgoFrame, const Pose2D& egoPoseGlobal, const LDMParamIn& parameters)
{
   // transforming CorridorInfo from Ego to Global frame
   memset(corridorGlobalFrame.corridorPoints, 0.0f, sizeof(corridorGlobalFrame.corridorPoints));
   memset(corridorGlobalFrame.corridorOrientation, 0.0f, sizeof(corridorGlobalFrame.corridorOrientation));
   memset(corridorGlobalFrame.corridorCurvature, 0.0f, sizeof(corridorGlobalFrame.corridorCurvature));

   coordinateTransforms.reverseTransform2D(
      corridorEgoFrame.corridorPoints,
      egoPoseGlobal,
      corridorEgoFrame.corridorLength,
      corridorGlobalFrame.corridorPoints);
   coordinateTransforms.angleTransform2D(
      corridorEgoFrame, egoPoseGlobal, corridorEgoFrame.corridorLength, corridorGlobalFrame);
   corridorGlobalFrame.corridorLength = corridorEgoFrame.corridorLength;

   // calculating trigger for Planner activation
   bool triggerPlanner =
      controlLogic.calcValidity(corridorEgoFrame, corridorGlobalFrame, egoPoseGlobal, parameters, firstCycle);

   if (controlLogic.validity && triggerPlanner)
   {
      firstCycle = false;
   }

   if (controlLogic.validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         // calculate the actual Planner frame
         getPlannerFrame(
            egoPoseGlobal,
            parameters,
            nodePoints,
            segmentParams,
            trajectoryEgoFrame,
            previousTrajectoryPlannerFrame);
         // storing the value for the next replan cycle to calculate the next planner frame
         previousPlannerFrame = plannerFramePoseGlobalFrame;

         // transforming CorridorInfo from Global to Planner frame
         memset(corridorPlannerFrame.corridorPoints, 0.0f, sizeof(corridorPlannerFrame.corridorPoints));
         memset(
            corridorPlannerFrame.corridorOrientation, 0.0f, sizeof(corridorPlannerFrame.corridorOrientation));
         memset(corridorPlannerFrame.corridorCurvature, 0.0f, sizeof(corridorPlannerFrame.corridorCurvature));

         coordinateTransforms.transform2D(
            corridorGlobalFrame.corridorPoints,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame.corridorPoints);

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         coordinateTransforms.angleTransform2D(
            corridorGlobalFrame,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame);
         corridorPlannerFrame.corridorLength = corridorGlobalFrame.corridorLength;

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         // calculating the Node Points
         driverModel.calc(corridorPlannerFrame, parameters, nodePoints);

         // calculating the Evaluation Points for the trajectory planner
         evalPointsPreprocess(driverModel.indices, corridorPlannerFrame, evalPoints);

         // calculating the clothoid parameters for each segments, then evaluating them
         memset(
            trajectoryPlannerFrame.trajectoryPoints, 0.0f, sizeof(trajectoryPlannerFrame.trajectoryPoints));
         trajectoryPlannerFrame.trajectoryLength = 0U;

         trajectoryPlanner.calc(nodePoints, evalPoints, segmentParams, trajectoryPlannerFrame);

         memset(
            previousTrajectoryPlannerFrame.trajectoryPoints,
            0.0f,
            sizeof(previousTrajectoryPlannerFrame.trajectoryPoints));
         previousTrajectoryPlannerFrame.trajectoryLength = 0U;
         for (uint16_t i{0}; i < 300; i++)
         {
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsX =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsX;
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsY =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsY;
         }
         previousTrajectoryPlannerFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      }

      // transforming trajectory from Planner to Global frame
      memset(trajectoryGlobalFrame.trajectoryPoints, 0.0f, sizeof(trajectoryGlobalFrame.trajectoryPoints));

      coordinateTransforms.reverseTransform2D(
         trajectoryPlannerFrame.trajectoryPoints,
         plannerFramePoseGlobalFrame,
         trajectoryPlannerFrame.trajectoryLength,
         trajectoryGlobalFrame.trajectoryPoints);
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      // transforming trajectory from Global to Ego frame
      memset(trajectoryEgoFrame.trajectoryPoints, 0.0f, sizeof(trajectoryEgoFrame.trajectoryPoints));

      coordinateTransforms.transform2D(
         trajectoryGlobalFrame.trajectoryPoints,
         egoPoseGlobal,
         trajectoryGlobalFrame.trajectoryLength,
         trajectoryEgoFrame.trajectoryPoints);
      trajectoryEgoFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;

      return trajectoryEgoFrame;
   }
   // If the corridor is not valid return trajectory points of zeros
   else
   {
      return trajectoryEmpty;
   }
}

TrajectoryPolynomial LinearDriverModel::runPoly(
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
   vfc::float32_t               P_nodePointDistances[3])
{
   // parameter init
   for (uint16_t i{0}; i < 300; i++)
   {
      corridor.corridorCurvature[i]      = corrC2[i];
      corridor.corridorOrientation[i]    = corrC1[i];
      corridor.corridorPoints[i].PointsX = corrX[i];
      corridor.corridorPoints[i].PointsY = corrY[i];
      corridor.corridorLength            = corrLength;
   }

   for (uint8_t i{0}; i < 21; i++)
   {
      params.P[i] = P[i];
   }
   params.replanCycle       = replanCycle;
   params.lookAheadDistance = LAD;
   for (uint8_t i{0}; i < 3; i++)
   {
      params.P_nodePointDistances[i] = P_nodePointDistances[i];
   }

   egoPoseGlob.Pose2DCoordinates.PointsX = egoPoseX;
   egoPoseGlob.Pose2DCoordinates.PointsY = egoPoseY;
   egoPoseGlob.Pose2DTheta               = egoPoseTheta;

   // transforming CorridorInfo from Ego to Global frame
   memset(corridorGlobalFrame.corridorPoints, 0.0f, sizeof(corridorGlobalFrame.corridorPoints));
   memset(corridorGlobalFrame.corridorOrientation, 0.0f, sizeof(corridorGlobalFrame.corridorOrientation));
   memset(corridorGlobalFrame.corridorCurvature, 0.0f, sizeof(corridorGlobalFrame.corridorCurvature));

   coordinateTransforms.reverseTransform2D(
      corridor.corridorPoints, egoPoseGlob, corridor.corridorLength, corridorGlobalFrame.corridorPoints);
   coordinateTransforms.angleTransform2D(corridor, egoPoseGlob, corridor.corridorLength, corridorGlobalFrame);
   corridorGlobalFrame.corridorLength = corridor.corridorLength;

   // calculating trigger for Planner activation
   bool triggerPlanner =
      controlLogic.calcValidity(corridor, corridorGlobalFrame, egoPoseGlob, params, firstCycle);

   if (controlLogic.validity && triggerPlanner)
   {
      firstCycle = false;
   }

   if (controlLogic.validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         // calculate the actual Planner frame
         getPlannerFrame(
            egoPoseGlob,
            params,
            nodePoints,
            segmentParams,
            trajectoryEgoFrame,
            previousTrajectoryPlannerFrame);
         // storing the value for the next replan cycle to calculate the next planner frame
         previousPlannerFrame = plannerFramePoseGlobalFrame;

         // transforming CorridorInfo from Global to Planner frame
         memset(corridorPlannerFrame.corridorPoints, 0.0f, sizeof(corridorPlannerFrame.corridorPoints));
         memset(
            corridorPlannerFrame.corridorOrientation, 0.0f, sizeof(corridorPlannerFrame.corridorOrientation));
         memset(corridorPlannerFrame.corridorCurvature, 0.0f, sizeof(corridorPlannerFrame.corridorCurvature));

         coordinateTransforms.transform2D(
            corridorGlobalFrame.corridorPoints,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame.corridorPoints);

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         coordinateTransforms.angleTransform2D(
            corridorGlobalFrame,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame);
         corridorPlannerFrame.corridorLength = corridorGlobalFrame.corridorLength;

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         // calculating the Node Points
         driverModel.calc(corridorPlannerFrame, params, nodePoints);

         // calculating the Evaluation Points for the trajectory planner
         evalPointsPreprocess(driverModel.indices, corridorPlannerFrame, evalPoints);

         // calculating the 3rd degree polynomial coefficients for each segments, then evaluating them
         memset(
            trajectoryPlannerFrame.trajectoryPoints, 0.0f, sizeof(trajectoryPlannerFrame.trajectoryPoints));
         trajectoryPlannerFrame.trajectoryLength = 0U;

         trajectoryPlanner
            .calc(nodePoints, evalPoints, polynomialCoeffs, segmentParams, trajectoryPlannerFrame);

         memset(
            previousTrajectoryPlannerFrame.trajectoryPoints,
            0.0f,
            sizeof(previousTrajectoryPlannerFrame.trajectoryPoints));
         previousTrajectoryPlannerFrame.trajectoryLength = 0U;
         for (uint16_t i{0}; i < 300; i++)
         {
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsX =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsX;
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsY =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsY;
         }
         previousTrajectoryPlannerFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      }

      // transforming trajectory from Planner to Global frame
      memset(trajectoryGlobalFrame.trajectoryPoints, 0.0f, sizeof(trajectoryGlobalFrame.trajectoryPoints));

      coordinateTransforms.reverseTransform2D(
         trajectoryPlannerFrame.trajectoryPoints,
         plannerFramePoseGlobalFrame,
         trajectoryPlannerFrame.trajectoryLength,
         trajectoryGlobalFrame.trajectoryPoints);
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      // transforming trajectory from Global to Ego frame
      memset(trajectoryEgoFrame.trajectoryPoints, 0.0f, sizeof(trajectoryEgoFrame.trajectoryPoints));

      coordinateTransforms.transform2D(
         trajectoryGlobalFrame.trajectoryPoints,
         egoPoseGlob,
         trajectoryGlobalFrame.trajectoryLength,
         trajectoryEgoFrame.trajectoryPoints);
      trajectoryEgoFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;

      memset(
         nodePointsGlobalFrame.nodePointsCoordinates,
         0.0f,
         sizeof(nodePointsGlobalFrame.nodePointsCoordinates));
      memset(
         nodePointsEgoFrame.nodePointsCoordinates, 0.0f, sizeof(nodePointsEgoFrame.nodePointsCoordinates));
      coordinateTransforms.reverseTransform2D(
         nodePoints.nodePointsCoordinates,
         plannerFramePoseGlobalFrame,
         4,
         nodePointsGlobalFrame.nodePointsCoordinates);
      coordinateTransforms.transform2D(
         nodePointsGlobalFrame.nodePointsCoordinates,
         egoPoseGlob,
         4,
         nodePointsEgoFrame.nodePointsCoordinates);
      segmentPlanner.buildPolynomial(nodePointsEgoFrame, polynomialCoeffs, segmentParamsEgoFrame);

      trajectoryPolynomial.trajectory       = trajectoryEgoFrame;
      trajectoryPolynomial.polynomialCoeffs = polynomialCoeffs;

      return trajectoryPolynomial;
   }
   // If the corridor is not valid return trajectory points of zeros
   else
   {
      return trajectoryPolynomialEmpty;
   }
}

TrajectoryPolynomial LinearDriverModel::runPoly(
   const CorridorInfo& corridorEgoFrame, const Pose2D& egoPoseGlobal, const LDMParamIn& parameters)
{
   // transforming CorridorInfo from Ego to Global frame
   memset(corridorGlobalFrame.corridorPoints, 0.0f, sizeof(corridorGlobalFrame.corridorPoints));
   memset(corridorGlobalFrame.corridorOrientation, 0.0f, sizeof(corridorGlobalFrame.corridorOrientation));
   memset(corridorGlobalFrame.corridorCurvature, 0.0f, sizeof(corridorGlobalFrame.corridorCurvature));

   coordinateTransforms.reverseTransform2D(
      corridorEgoFrame.corridorPoints,
      egoPoseGlobal,
      corridorEgoFrame.corridorLength,
      corridorGlobalFrame.corridorPoints);
   coordinateTransforms.angleTransform2D(
      corridorEgoFrame, egoPoseGlobal, corridorEgoFrame.corridorLength, corridorGlobalFrame);
   corridorGlobalFrame.corridorLength = corridorEgoFrame.corridorLength;

   // calculating trigger for Planner activation
   bool triggerPlanner =
      controlLogic.calcValidity(corridorEgoFrame, corridorGlobalFrame, egoPoseGlobal, parameters, firstCycle);

   if (controlLogic.validity && triggerPlanner)
   {
      firstCycle = false;
   }

   if (controlLogic.validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         // calculate the actual Planner frame
         getPlannerFrame(
            egoPoseGlobal,
            parameters,
            nodePoints,
            segmentParams,
            trajectoryEgoFrame,
            previousTrajectoryPlannerFrame);
         // storing the value for the next replan cycle to calculate the next planner frame
         previousPlannerFrame = plannerFramePoseGlobalFrame;

         // transforming CorridorInfo from Global to Planner frame
         memset(corridorPlannerFrame.corridorPoints, 0.0f, sizeof(corridorPlannerFrame.corridorPoints));
         memset(
            corridorPlannerFrame.corridorOrientation, 0.0f, sizeof(corridorPlannerFrame.corridorOrientation));
         memset(corridorPlannerFrame.corridorCurvature, 0.0f, sizeof(corridorPlannerFrame.corridorCurvature));

         coordinateTransforms.transform2D(
            corridorGlobalFrame.corridorPoints,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame.corridorPoints);

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         coordinateTransforms.angleTransform2D(
            corridorGlobalFrame,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame);
         corridorPlannerFrame.corridorLength = corridorGlobalFrame.corridorLength;

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         // calculating the Node Points
         driverModel.calc(corridorPlannerFrame, parameters, nodePoints);

         // calculating the Evaluation Points for the trajectory planner
         evalPointsPreprocess(driverModel.indices, corridorPlannerFrame, evalPoints);

         // calculating the 3rd degree polynomial coefficients for each segments, then evaluating them
         memset(
            trajectoryPlannerFrame.trajectoryPoints, 0.0f, sizeof(trajectoryPlannerFrame.trajectoryPoints));
         trajectoryPlannerFrame.trajectoryLength = 0U;

         trajectoryPlanner
            .calc(nodePoints, evalPoints, polynomialCoeffs, segmentParams, trajectoryPlannerFrame);

         memset(
            previousTrajectoryPlannerFrame.trajectoryPoints,
            0.0f,
            sizeof(previousTrajectoryPlannerFrame.trajectoryPoints));
         previousTrajectoryPlannerFrame.trajectoryLength = 0U;
         for (uint16_t i{0}; i < 300; i++)
         {
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsX =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsX;
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsY =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsY;
         }
         previousTrajectoryPlannerFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      }

      // transforming trajectory from Planner to Global frame
      memset(trajectoryGlobalFrame.trajectoryPoints, 0.0f, sizeof(trajectoryGlobalFrame.trajectoryPoints));

      coordinateTransforms.reverseTransform2D(
         trajectoryPlannerFrame.trajectoryPoints,
         plannerFramePoseGlobalFrame,
         trajectoryPlannerFrame.trajectoryLength,
         trajectoryGlobalFrame.trajectoryPoints);
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      // transforming trajectory from Global to Ego frame
      memset(trajectoryEgoFrame.trajectoryPoints, 0.0f, sizeof(trajectoryEgoFrame.trajectoryPoints));

      coordinateTransforms.transform2D(
         trajectoryGlobalFrame.trajectoryPoints,
         egoPoseGlobal,
         trajectoryGlobalFrame.trajectoryLength,
         trajectoryEgoFrame.trajectoryPoints);
      trajectoryEgoFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;

      memset(
         nodePointsGlobalFrame.nodePointsCoordinates,
         0.0f,
         sizeof(nodePointsGlobalFrame.nodePointsCoordinates));
      memset(
         nodePointsEgoFrame.nodePointsCoordinates, 0.0f, sizeof(nodePointsEgoFrame.nodePointsCoordinates));
      coordinateTransforms.reverseTransform2D(
         nodePoints.nodePointsCoordinates,
         plannerFramePoseGlobalFrame,
         4,
         nodePointsGlobalFrame.nodePointsCoordinates);
      coordinateTransforms.transform2D(
         nodePointsGlobalFrame.nodePointsCoordinates,
         egoPoseGlobal,
         4,
         nodePointsEgoFrame.nodePointsCoordinates);
      segmentPlanner.buildPolynomial(nodePointsEgoFrame, polynomialCoeffs, segmentParamsEgoFrame);

      trajectoryPolynomial.trajectory       = trajectoryEgoFrame;
      trajectoryPolynomial.polynomialCoeffs = polynomialCoeffs;

      return trajectoryPolynomial;
   }
   // If the corridor is not valid return trajectory points of zeros
   else
   {
      return trajectoryPolynomialEmpty;
   }
}

PolynomialCoeffs LinearDriverModel::runCoeffs(
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
   vfc::float32_t               P_nodePointDistances[3])
{
   // parameter init
   for (uint16_t i{0}; i < 300; i++)
   {
      corridor.corridorCurvature[i]      = corrC2[i];
      corridor.corridorOrientation[i]    = corrC1[i];
      corridor.corridorPoints[i].PointsX = corrX[i];
      corridor.corridorPoints[i].PointsY = corrY[i];
      corridor.corridorLength            = corrLength;
   }

   for (uint8_t i{0}; i < 21; i++)
   {
      params.P[i] = P[i];
   }
   params.replanCycle       = replanCycle;
   params.lookAheadDistance = LAD;
   for (uint8_t i{0}; i < 3; i++)
   {
      params.P_nodePointDistances[i] = P_nodePointDistances[i];
   }

   egoPoseGlob.Pose2DCoordinates.PointsX = egoPoseX;
   egoPoseGlob.Pose2DCoordinates.PointsY = egoPoseY;
   egoPoseGlob.Pose2DTheta               = egoPoseTheta;

   // transforming CorridorInfo from Ego to Global frame
   memset(corridorGlobalFrame.corridorPoints, 0.0f, sizeof(corridorGlobalFrame.corridorPoints));
   memset(corridorGlobalFrame.corridorOrientation, 0.0f, sizeof(corridorGlobalFrame.corridorOrientation));
   memset(corridorGlobalFrame.corridorCurvature, 0.0f, sizeof(corridorGlobalFrame.corridorCurvature));

   coordinateTransforms.reverseTransform2D(
      corridor.corridorPoints, egoPoseGlob, corridor.corridorLength, corridorGlobalFrame.corridorPoints);
   coordinateTransforms.angleTransform2D(corridor, egoPoseGlob, corridor.corridorLength, corridorGlobalFrame);
   corridorGlobalFrame.corridorLength = corridor.corridorLength;

   // calculating trigger for Planner activation
   triggerPlanner = controlLogic.calcValidity(corridor, corridorGlobalFrame, egoPoseGlob, params, firstCycle);

   if (controlLogic.validity && triggerPlanner)
   {
      firstCycle = false;
   }

   if (controlLogic.validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         // calculate the actual Planner frame
         getPlannerFrame(
            egoPoseGlob,
            params,
            nodePoints,
            segmentParams,
            trajectoryEgoFrame,
            previousTrajectoryPlannerFrame);
         // storing the value for the next replan cycle to calculate the next planner frame
         previousPlannerFrame = plannerFramePoseGlobalFrame;

         // transforming CorridorInfo from Global to Planner frame
         memset(corridorPlannerFrame.corridorPoints, 0.0f, sizeof(corridorPlannerFrame.corridorPoints));
         memset(
            corridorPlannerFrame.corridorOrientation, 0.0f, sizeof(corridorPlannerFrame.corridorOrientation));
         memset(corridorPlannerFrame.corridorCurvature, 0.0f, sizeof(corridorPlannerFrame.corridorCurvature));

         coordinateTransforms.transform2D(
            corridorGlobalFrame.corridorPoints,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame.corridorPoints);

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         coordinateTransforms.angleTransform2D(
            corridorGlobalFrame,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame);
         corridorPlannerFrame.corridorLength = corridorGlobalFrame.corridorLength;

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         // calculating the Node Points
         driverModel.calc(corridorPlannerFrame, params, nodePoints);

         // calculating the Evaluation Points for the trajectory planner
         evalPointsPreprocess(driverModel.indices, corridorPlannerFrame, evalPoints);

         // calculating the 3rd degree polynomial coefficients for each segments, then evaluating them
         memset(
            trajectoryPlannerFrame.trajectoryPoints, 0.0f, sizeof(trajectoryPlannerFrame.trajectoryPoints));
         trajectoryPlannerFrame.trajectoryLength = 0U;

         trajectoryPlanner.calc(nodePoints, evalPoints, segmentParams, trajectoryPlannerFrame);

         memset(
            previousTrajectoryPlannerFrame.trajectoryPoints,
            0.0f,
            sizeof(previousTrajectoryPlannerFrame.trajectoryPoints));
         previousTrajectoryPlannerFrame.trajectoryLength = 0U;
         for (uint16_t i{0}; i < 300; i++)
         {
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsX =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsX;
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsY =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsY;
         }
         previousTrajectoryPlannerFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      }

      // transforming trajectory from Planner to Global frame
      memset(trajectoryGlobalFrame.trajectoryPoints, 0.0f, sizeof(trajectoryGlobalFrame.trajectoryPoints));

      coordinateTransforms.reverseTransform2D(
         trajectoryPlannerFrame.trajectoryPoints,
         plannerFramePoseGlobalFrame,
         trajectoryPlannerFrame.trajectoryLength,
         trajectoryGlobalFrame.trajectoryPoints);
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      // transforming trajectory from Global to Ego frame
      memset(trajectoryEgoFrame.trajectoryPoints, 0.0f, sizeof(trajectoryEgoFrame.trajectoryPoints));

      coordinateTransforms.transform2D(
         trajectoryGlobalFrame.trajectoryPoints,
         egoPoseGlob,
         trajectoryGlobalFrame.trajectoryLength,
         trajectoryEgoFrame.trajectoryPoints);
      trajectoryEgoFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;

      if (trajectoryEgoFrame.trajectoryLength > 0)
      {
         trajectoryCoeffs = polynomialSubfunctions.fitThirdOrderPolynomial(trajectoryEgoFrame);
         // needs to return a polynomial like this: y = c0 + c1 * x + 2*c2 * x^2 + 6*c3 * x^3
         trajectoryCoeffs.c2 = trajectoryCoeffs.c2 * 2;
         trajectoryCoeffs.c3 = trajectoryCoeffs.c3 * 6;
         // TRC input
         trajectoryCoeffs.length =
            trajectoryEgoFrame.trajectoryPoints[trajectoryEgoFrame.trajectoryLength - 1U].PointsX;
         trajectoryCoeffs.breakPoint = trajectoryCoeffs.length;
      }
      else
      {
         trajectoryCoeffs.c0                 = 0;
         trajectoryCoeffs.c1                 = 0;
         trajectoryCoeffs.c2                 = 0;
         trajectoryCoeffs.c3                 = 0;
         trajectoryCoeffs.length.value()     = 0.0f;
         trajectoryCoeffs.breakPoint.value() = 0.0f;
      }

      return trajectoryCoeffs;
   }
   // If the corridor is not valid return trajectory points of zeros
   else
   {
      return trajectoryCoeffsEmpty;
   }
}

PolynomialCoeffs LinearDriverModel::runCoeffs(
   const CorridorInfo& corridorEgoFrame, const Pose2D& egoPoseGlobal, const LDMParamIn& parameters)
{
   // transforming CorridorInfo from Ego to Global frame
   memset(corridorGlobalFrame.corridorPoints, 0.0f, sizeof(corridorGlobalFrame.corridorPoints));
   memset(corridorGlobalFrame.corridorOrientation, 0.0f, sizeof(corridorGlobalFrame.corridorOrientation));
   memset(corridorGlobalFrame.corridorCurvature, 0.0f, sizeof(corridorGlobalFrame.corridorCurvature));

   coordinateTransforms.reverseTransform2D(
      corridorEgoFrame.corridorPoints,
      egoPoseGlobal,
      corridorEgoFrame.corridorLength,
      corridorGlobalFrame.corridorPoints);
   coordinateTransforms.angleTransform2D(
      corridorEgoFrame, egoPoseGlobal, corridorEgoFrame.corridorLength, corridorGlobalFrame);
   corridorGlobalFrame.corridorLength = corridorEgoFrame.corridorLength;

   // calculating trigger for Planner activation
   bool triggerPlanner =
      controlLogic.calcValidity(corridorEgoFrame, corridorGlobalFrame, egoPoseGlobal, parameters, firstCycle);

   if (controlLogic.validity && triggerPlanner)
   {
      firstCycle = false;
   }

   if (controlLogic.validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         // calculate the actual Planner frame
         getPlannerFrame(
            egoPoseGlobal,
            parameters,
            nodePoints,
            segmentParams,
            trajectoryEgoFrame,
            previousTrajectoryPlannerFrame);
         // storing the value for the next replan cycle to calculate the next planner frame
         previousPlannerFrame = plannerFramePoseGlobalFrame;

         // transforming CorridorInfo from Global to Planner frame
         memset(corridorPlannerFrame.corridorPoints, 0.0f, sizeof(corridorPlannerFrame.corridorPoints));
         memset(
            corridorPlannerFrame.corridorOrientation, 0.0f, sizeof(corridorPlannerFrame.corridorOrientation));
         memset(corridorPlannerFrame.corridorCurvature, 0.0f, sizeof(corridorPlannerFrame.corridorCurvature));

         coordinateTransforms.transform2D(
            corridorGlobalFrame.corridorPoints,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame.corridorPoints);

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         coordinateTransforms.angleTransform2D(
            corridorGlobalFrame,
            plannerFramePoseGlobalFrame,
            corridorGlobalFrame.corridorLength,
            corridorPlannerFrame);
         corridorPlannerFrame.corridorLength = corridorGlobalFrame.corridorLength;

         plannerFramePoseGlobalFrame.Pose2DTheta.value() =
            -1 * plannerFramePoseGlobalFrame.Pose2DTheta.value();

         // calculating the Node Points
         driverModel.calc(corridorPlannerFrame, parameters, nodePoints);

         // calculating the Evaluation Points for the trajectory planner
         evalPointsPreprocess(driverModel.indices, corridorPlannerFrame, evalPoints);

         // calculating the 3rd degree polynomial coefficients for each segments, then evaluating them
         memset(
            trajectoryPlannerFrame.trajectoryPoints, 0.0f, sizeof(trajectoryPlannerFrame.trajectoryPoints));
         trajectoryPlannerFrame.trajectoryLength = 0U;

         trajectoryPlanner.calc(nodePoints, evalPoints, segmentParams, trajectoryPlannerFrame);

         memset(
            previousTrajectoryPlannerFrame.trajectoryPoints,
            0.0f,
            sizeof(previousTrajectoryPlannerFrame.trajectoryPoints));
         previousTrajectoryPlannerFrame.trajectoryLength = 0U;
         for (uint16_t i{0}; i < 300; i++)
         {
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsX =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsX;
            previousTrajectoryPlannerFrame.trajectoryPoints[i].PointsY =
               trajectoryPlannerFrame.trajectoryPoints[i].PointsY;
         }
         previousTrajectoryPlannerFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      }

      // transforming trajectory from Planner to Global frame
      memset(trajectoryGlobalFrame.trajectoryPoints, 0.0f, sizeof(trajectoryGlobalFrame.trajectoryPoints));

      coordinateTransforms.reverseTransform2D(
         trajectoryPlannerFrame.trajectoryPoints,
         plannerFramePoseGlobalFrame,
         trajectoryPlannerFrame.trajectoryLength,
         trajectoryGlobalFrame.trajectoryPoints);
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      trajectoryGlobalFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;
      // transforming trajectory from Global to Ego frame
      memset(trajectoryEgoFrame.trajectoryPoints, 0.0f, sizeof(trajectoryEgoFrame.trajectoryPoints));

      coordinateTransforms.transform2D(
         trajectoryGlobalFrame.trajectoryPoints,
         egoPoseGlobal,
         trajectoryGlobalFrame.trajectoryLength,
         trajectoryEgoFrame.trajectoryPoints);
      trajectoryEgoFrame.trajectoryLength = trajectoryPlannerFrame.trajectoryLength;

      if (trajectoryEgoFrame.trajectoryLength > 0)
      {
         trajectoryCoeffs = polynomialSubfunctions.fitThirdOrderPolynomial(trajectoryEgoFrame);
         // needs to return a polynomial like this: y = c0 + c1 * x + 2*c2 * x^2 + 6*c3 * x^3
         trajectoryCoeffs.c2 = trajectoryCoeffs.c2 * 2;
         trajectoryCoeffs.c3 = trajectoryCoeffs.c3 * 6;
         // TRC input
         trajectoryCoeffs.length =
            trajectoryEgoFrame.trajectoryPoints[trajectoryEgoFrame.trajectoryLength].PointsX;
         trajectoryCoeffs.breakPoint = trajectoryCoeffs.length;
      }
      else
      {
         trajectoryCoeffs.c0                 = 0;
         trajectoryCoeffs.c1                 = 0;
         trajectoryCoeffs.c2                 = 0;
         trajectoryCoeffs.c3                 = 0;
         trajectoryCoeffs.length.value()     = 0;
         trajectoryCoeffs.breakPoint.value() = 0;
      }

      return trajectoryCoeffs;
   }
   // If the corridor is not valid return trajectory points of zeros
   else
   {
      return trajectoryCoeffsEmpty;
   }
}

PolynomialCoeffsThreeSegments LinearDriverModel::runCoeffsLite(
   const CorridorInfoCoefficients& corridorCoefficients,
   const Pose2D&                   egoPoseGlobal,
   const LDMParamIn&               parameters)
{
   // calculating trigger for Planner activation
   bool triggerPlanner =
      controlLogic.calcValidity(corridorCoefficients, egoPoseGlobal, parameters, firstCycle);

   if (controlLogic.validity && triggerPlanner)
   {
      firstCycle = false;
   }

   if (controlLogic.validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         // store the current ego pose till the next planning
         egoPoseGlobalPlan = egoPoseGlobal;

         // calculating the Node Points
         driverModel.driverModelPlannerLite(
            corridorCoefficients, trajectoryCoeffsThreeSegments, parameters, nodePoints);

         // calculating the polynomial coefficients for each segments

         segmentPlanner.buildThreeSegmentPolynomial(nodePoints, trajectoryCoeffsThreeSegments, segmentParams);
         for (uint8_t i{0U}; i < 3; i++)
         {
            trajectoryCoeffsThreeSegments.sectionBorderStart[i] =
               nodePoints.nodePointsCoordinates[i].PointsX.value();
            trajectoryCoeffsThreeSegments.sectionBorderEnd[i] =
               nodePoints.nodePointsCoordinates[i + 1U].PointsX.value();
         }
      }

      else
      {
         memset(
            nodePointsEgoFrame.nodePointsCoordinates, 0.0f, sizeof(nodePointsEgoFrame.nodePointsCoordinates));

         coordinateTransforms
            .transformNodePoints(nodePoints, egoPoseGlobalPlan, egoPoseGlobal, nodePointsEgoFrame);
         segmentPlanner.buildThreeSegmentPolynomial(
            nodePointsEgoFrame, trajectoryCoeffsThreeSegments, segmentParamsEgoFrame);
         for (uint8_t i{0U}; i < 3; i++)
         {
            trajectoryCoeffsThreeSegments.sectionBorderStart[i] =
               nodePointsEgoFrame.nodePointsCoordinates[i].PointsX.value();
            trajectoryCoeffsThreeSegments.sectionBorderEnd[i] =
               nodePointsEgoFrame.nodePointsCoordinates[i + 1U].PointsX.value();
         }
      }

      if (trajectoryCoeffsThreeSegments.sectionBorderEnd[0U] > 0)
      {
         // needs to return a polynomial like this: y = c0 + c1 * x + 2*c2 * x^2 + 6*c3 * x^3
         for (uint8_t i{0U}; i < 3; i++)
         {
            trajectoryCoeffsThreeSegments.segmentCoeffs[i].c2 *= 2.0f;
            trajectoryCoeffsThreeSegments.segmentCoeffs[i].c3 *= 6.0f;
         }
         // TRC input
         trajectoryCoeffsThreeSegments.segmentCoeffs[0U].breakPoint.value() =
            trajectoryCoeffsThreeSegments.sectionBorderEnd[0U];
         trajectoryCoeffsThreeSegments.segmentCoeffs[0U].breakPoint.value() =
            trajectoryCoeffsThreeSegments.sectionBorderEnd[1U];
      }
      else
      {
         for (uint8_t i{0U}; i < 3; i++)
         {
            trajectoryCoeffsThreeSegments.segmentCoeffs[i].c0                 = 0;
            trajectoryCoeffsThreeSegments.segmentCoeffs[i].c1                 = 0;
            trajectoryCoeffsThreeSegments.segmentCoeffs[i].c2                 = 0;
            trajectoryCoeffsThreeSegments.segmentCoeffs[i].c3                 = 0;
            trajectoryCoeffsThreeSegments.segmentCoeffs[i].length.value()     = 0;
            trajectoryCoeffsThreeSegments.segmentCoeffs[i].breakPoint.value() = 0;
            trajectoryCoeffsThreeSegments.sectionBorderStart[i]               = 0;
            trajectoryCoeffsThreeSegments.sectionBorderEnd[i]                 = 0;
         }
      }

      return trajectoryCoeffsThreeSegments;
   }
   // If the corridor is not valid return trajectory points of zeros
   else
   {
      return trajectoryCoeffsThreeSegmentsEmpty;
   }
}

void LinearDriverModel::getPlannerFrame(
   const Pose2D&           egoPoseGlobal,
   const LDMParamIn&       parameters,
   const NodePoints&       nodePoints,
   const SegmentParams&    segmentParams,
   const TrajectoryPoints& previousTrajectoryPointsEgoFrame,
   const TrajectoryPoints& previousTrajectoryPointsPlannerFrame)
{
   if (controlLogic.initialized == false)
   {
      plannerFramePoseGlobalFrame.Pose2DCoordinates.PointsX = egoPoseGlobal.Pose2DCoordinates.PointsX;
      plannerFramePoseGlobalFrame.Pose2DCoordinates.PointsY = egoPoseGlobal.Pose2DCoordinates.PointsY;
      plannerFramePoseGlobalFrame.Pose2DTheta               = egoPoseGlobal.Pose2DTheta;
   }
   else
   {
      vfc::uint16_t index = 0;
      for (uint16_t i{0}; i < 300; i++)
      {
         if (previousTrajectoryPointsEgoFrame.trajectoryPoints[i].PointsX.value() >= 0)
         {
            index = i;
            break;
         }
      }
      tempPlannerFrame.Pose2DCoordinates.PointsX =
         previousTrajectoryPointsPlannerFrame.trajectoryPoints[index].PointsX;
      tempPlannerFrame.Pose2DCoordinates.PointsY =
         previousTrajectoryPointsPlannerFrame.trajectoryPoints[index].PointsY;
      tempPlannerFrame.Pose2DTheta.value() = 0.0f;

      for (int8_t i{0}; i <= 3U; i++)
      {
         if (
            nodePoints.nodePointsCoordinates[i].PointsX.value()
            > tempPlannerFrame.Pose2DCoordinates.PointsX.value())
         {
            segmentId = i - 1L;
            break;
         }
      }
      if (segmentId == -1L)
      {
         segmentId = 0L;
      }

      vfc::float32_t c0Selected     = segmentParams.k[segmentId].value();
      vfc::float32_t c1Selected     = segmentParams.dk[segmentId].value();
      vfc::float32_t theta0Selected = nodePoints.nodePointsTheta[segmentId].value();

      for (uint16_t i{driverModel.indices[segmentId]}; i < (driverModel.indices[segmentId] + index); i++)
      {
         diffTrajX = previousTrajectoryPointsPlannerFrame.trajectoryPoints[i + 1].PointsX.value()
                     - previousTrajectoryPointsPlannerFrame.trajectoryPoints[i].PointsX.value();
         diffTrajY = previousTrajectoryPointsPlannerFrame.trajectoryPoints[i + 1].PointsY.value()
                     - previousTrajectoryPointsPlannerFrame.trajectoryPoints[i].PointsY.value();
         distance = vfc::sqrt(vfc::sqr(diffTrajX) + vfc::sqr(diffTrajY));
         arcLength += distance;
      }
      arcLength *= 0.7109f;  // this is a magic number from Krisztian

      tempPlannerFrame.Pose2DTheta.value() = theta0Selected + (c0Selected * arcLength)
                                             + ((vfc::divide(c1Selected, 2.0f)) * arcLength * arcLength);

      coordinateTransforms.reverseTransform2D(
         tempPlannerFrame.Pose2DCoordinates,
         previousPlannerFrame,
         plannerFramePoseGlobalFrame.Pose2DCoordinates);

      coordinateTransforms
         .angleTransform2D(tempPlannerFrame, previousPlannerFrame, plannerFramePoseGlobalFrame);
   }
}

void LinearDriverModel::evalPointsPreprocess(
   vfc::uint16_t indices[4], const CorridorInfo& corridorPlanner, EvalPoints& evalPoints)
{
   // clearing the previous points
   for (uint16_t i{0U}; i < evalPoints.evalPointsLengths[0U]; i++)
   {
      evalPoints.evalPointsSegment1[i].value() = 0.0f;
   }
   for (uint16_t i{0U}; i < evalPoints.evalPointsLengths[1U]; i++)
   {
      evalPoints.evalPointsSegment2[i].value() = 0.0f;
   }
   for (uint16_t i{0U}; i < evalPoints.evalPointsLengths[2U]; i++)
   {
      evalPoints.evalPointsSegment3[i].value() = 0.0f;
   }

   evalPoints.evalPointsLengths[0] = indices[1];
   evalPoints.evalPointsLengths[1] = indices[2] - indices[1];
   if (indices[3] > 0)
   {
      evalPoints.evalPointsLengths[2] = indices[3] - indices[2] + 1;
   }
   else
   {
      evalPoints.evalPointsLengths[2] = 0;
   }

   for (uint16_t i{0U}; i < evalPoints.evalPointsLengths[0U]; i++)
   {
      evalPoints.evalPointsSegment1[i] = corridorPlanner.corridorPoints[i].PointsX;
   }
   for (uint16_t i{0U}; i < evalPoints.evalPointsLengths[1U]; i++)
   {
      evalPoints.evalPointsSegment2[i] =
         corridorPlanner.corridorPoints[i + evalPoints.evalPointsLengths[0U]].PointsX;
   }
   for (uint16_t i{0U}; i < evalPoints.evalPointsLengths[2U]; i++)
   {
      evalPoints.evalPointsSegment3[i] =
         corridorPlanner
            .corridorPoints[i + evalPoints.evalPointsLengths[0U] + evalPoints.evalPointsLengths[1U]]
            .PointsX;
   }
}

}  // namespace Emg
}  // namespace Dc

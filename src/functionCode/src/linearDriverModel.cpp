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

#include "../inc/linearDriverModel/linearDriverModel.hpp"

void LinearDriverModel::initCoeffs(const ScenarioPolynomials& corridorCoefficients)
{
   if (!firstCycle)
      return;

   for (uint8_t i{0U}; i < 3; i++)
   {
      trajectoryCoeffsThreeSegments.segmentCoeffs[i] = corridorCoefficients.coeffs[i];
   }
}

TrajectoryOutput LinearDriverModel::runCoeffsLite(
   const ScenarioPolynomials&   corridorCoefficients,
   const Pose2D&                egoPoseGlobal,
   const LDMParamIn&            parameters)
{
   // calculating trigger for Planner activation
   bool triggerPlanner =
      controlLogic.calcValidity(corridorCoefficients, egoPoseGlobal, parameters, firstCycle);

   TrajectoryOutput trajectoryOutput;
   if (controlLogic.validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         firstCycle = false;
         // store the current ego pose till the next planning
         egoPoseGlobalPlan = egoPoseGlobal;

         // calculating the Node Points
         driverModel.driverModelPlannerLite(corridorCoefficients, trajectoryCoeffsThreeSegments, parameters, nodePoints);

         // calculating the polynomial coefficients for each segments

         segmentPlanner.buildThreeSegmentPolynomial(nodePoints, trajectoryCoeffsThreeSegments, segmentParams);
         for (uint8_t i{0U}; i < 3; i++)
         {
            trajectoryCoeffsThreeSegments.sectionBorderStart[i] =
               nodePoints.nodePointsCoordinates[i].x;
            trajectoryCoeffsThreeSegments.sectionBorderEnd[i] =
               nodePoints.nodePointsCoordinates[i + 1U].x;
         }

         trajectoryOutput.nodePts = nodePoints;
      }
      else
      {
         memset(nodePointsEgoFrame.nodePointsCoordinates, 0.0f, sizeof(nodePointsEgoFrame.nodePointsCoordinates));

         coordinateTransforms.transformNodePoints(nodePoints, egoPoseGlobalPlan, egoPoseGlobal, nodePointsEgoFrame);
         segmentPlanner.buildThreeSegmentPolynomial(nodePointsEgoFrame, trajectoryCoeffsThreeSegments, segmentParamsEgoFrame);
         for (uint8_t i{0U}; i < 3; i++)
         {
            trajectoryCoeffsThreeSegments.sectionBorderStart[i] =
               nodePointsEgoFrame.nodePointsCoordinates[i].x;
            trajectoryCoeffsThreeSegments.sectionBorderEnd[i] =
               nodePointsEgoFrame.nodePointsCoordinates[i + 1U].x;
         }

         trajectoryOutput.nodePts = nodePointsEgoFrame;
      }
      
      trajectoryOutput.segmentCoeffs = trajectoryCoeffsThreeSegments;
   }
   // invalid corridor returns all 0
   return trajectoryOutput;
}
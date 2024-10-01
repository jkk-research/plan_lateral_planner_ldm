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
   if (!m_firstCycle)
      return;

   for (uint8_t i{0U}; i < 3; i++)
   {
      m_trajectoryCoeffsThreeSegments.segmentCoeffs[i] = corridorCoefficients.coeffs[i];
   }
}

TrajectoryOutput LinearDriverModel::runCoeffsLite(
   const ScenarioPolynomials&   corridorCoefficients,
   const Pose2D&                egoPoseGlobal,
   const LDMParamIn&            parameters)
{
   // calculating trigger for Planner activation
   bool triggerPlanner =
      m_controlLogic.calcValidity(corridorCoefficients, egoPoseGlobal, parameters, m_firstCycle);

   TrajectoryOutput trajectoryOutput;
   if (m_controlLogic.m_validity)
   {
      // The planner activates after a replan cycle ends, or if the planner is not initialized.
      // Until the replan cycle does not end, the calculated values are kept constant,
      // and the trajectory is being transformed with respect to the vehicle's current position.
      if (triggerPlanner)
      {
         m_firstCycle = false;
         // store the current ego pose till the next planning
         m_egoPoseGlobalPlan = egoPoseGlobal;

         // calculating the Node Points
         m_driverModel.driverModelPlannerLite(corridorCoefficients, m_trajectoryCoeffsThreeSegments, parameters, m_nodePoints);

         // calculating the polynomial coefficients for each segments

         m_segmentPlanner.buildThreeSegmentPolynomial(m_nodePoints, m_trajectoryCoeffsThreeSegments, m_segmentParams);
         for (uint8_t i{0U}; i < 3; i++)
         {
            m_trajectoryCoeffsThreeSegments.sectionBorderStart[i] =
               m_nodePoints.nodePointsCoordinates[i].x;
            m_trajectoryCoeffsThreeSegments.sectionBorderEnd[i] =
               m_nodePoints.nodePointsCoordinates[i + 1U].x;
         }

         trajectoryOutput.nodePts = m_nodePoints;
      }
      else
      {
         memset(m_nodePointsEgoFrame.nodePointsCoordinates, 0.0f, sizeof(m_nodePointsEgoFrame.nodePointsCoordinates));

         m_coordinateTransforms.transformNodePoints(m_nodePoints, m_egoPoseGlobalPlan, egoPoseGlobal, m_nodePointsEgoFrame);
         m_segmentPlanner.buildThreeSegmentPolynomial(m_nodePointsEgoFrame, m_trajectoryCoeffsThreeSegments, m_segmentParamsEgoFrame);
         for (uint8_t i{0U}; i < 3; i++)
         {
            m_trajectoryCoeffsThreeSegments.sectionBorderStart[i] =
               m_nodePointsEgoFrame.nodePointsCoordinates[i].x;
            m_trajectoryCoeffsThreeSegments.sectionBorderEnd[i] =
               m_nodePointsEgoFrame.nodePointsCoordinates[i + 1U].x;
         }

         trajectoryOutput.nodePts = m_nodePointsEgoFrame;
      }
      
      trajectoryOutput.segmentCoeffs = m_trajectoryCoeffsThreeSegments;
   }
   // invalid corridor returns all 0
   return trajectoryOutput;
}

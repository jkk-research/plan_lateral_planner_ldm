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

#include "linearDriverModel/emg_linearDriverModel.hpp"

// Parameters
float P_breakPointMinimumDistance = 0.0f; 


PolynomialCoeffsTwoSegments LinearDriverModel::runCoeffsLite(
   const ScenarioCoeffs&   corridorCoefficients,
   const Pose2D&           egoPoseGlobal,
   const LDMParamIn&       parameters)
{
   // calculating trigger for Planner activation
   bool triggerPlanner =
      controlLogic.calcValidity(corridorCoefficients, egoPoseGlobal, parameters, firstCycle);

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

      // Finding the valid trajectory segment and writing it out to the return variable
      // needs to return a polynomial like this: y = c0 + c1 * x + 2*c2 * x^2 + 6*c3 * x^3
      // initializing before search
      for (uint8_t i{0U}; i < 2; i++){
         coefficientsToController.segmentCoeffs[i].breakPoint.value() = 0.0f;
         coefficientsToController.segmentCoeffs[i].c0 = 0.0f;
         coefficientsToController.segmentCoeffs[i].c1 = 0.0f;
         coefficientsToController.segmentCoeffs[i].c2 = 0.0f;
         coefficientsToController.segmentCoeffs[i].c3 = 0.0f;
      }      

      for (uint8_t i{0U}; i < 3; i++)
      {
         if (trajectoryCoeffsThreeSegments.sectionBorderStart[i] <= P_breakPointMinimumDistance &&
         trajectoryCoeffsThreeSegments.sectionBorderEnd[i] > P_breakPointMinimumDistance){
            // this is the valid segment, put its coefficients to the output
            if (i<2){
               coefficientsToController.segmentCoeffs[0].breakPoint.value() = trajectoryCoeffsThreeSegments.sectionBorderEnd[i];
               coefficientsToController.segmentCoeffs[0].c0 = trajectoryCoeffsThreeSegments.segmentCoeffs[i].c0;
               coefficientsToController.segmentCoeffs[0].c1 = trajectoryCoeffsThreeSegments.segmentCoeffs[i].c1;
               coefficientsToController.segmentCoeffs[0].c2 = trajectoryCoeffsThreeSegments.segmentCoeffs[i].c2*2.0f;
               coefficientsToController.segmentCoeffs[0].c3 = trajectoryCoeffsThreeSegments.segmentCoeffs[i].c3*6.0f;
               // second section
               coefficientsToController.segmentCoeffs[1].breakPoint.value() = trajectoryCoeffsThreeSegments.sectionBorderEnd[i+1U];
               coefficientsToController.segmentCoeffs[1].c0 = trajectoryCoeffsThreeSegments.segmentCoeffs[i+1U].c0;
               coefficientsToController.segmentCoeffs[1].c1 = trajectoryCoeffsThreeSegments.segmentCoeffs[i+1U].c1;
               coefficientsToController.segmentCoeffs[1].c2 = trajectoryCoeffsThreeSegments.segmentCoeffs[i+1U].c2*2.0f;
               coefficientsToController.segmentCoeffs[1].c3 = trajectoryCoeffsThreeSegments.segmentCoeffs[i+1U].c3*6.0f;
            }
            else{
               for (uint8_t j{0U}; j < 2; j++){
                  coefficientsToController.segmentCoeffs[j].breakPoint.value() = trajectoryCoeffsThreeSegments.sectionBorderEnd[i];
                  coefficientsToController.segmentCoeffs[j].c0 = trajectoryCoeffsThreeSegments.segmentCoeffs[i].c0;
                  coefficientsToController.segmentCoeffs[j].c1 = trajectoryCoeffsThreeSegments.segmentCoeffs[i].c1;
                  coefficientsToController.segmentCoeffs[j].c2 = trajectoryCoeffsThreeSegments.segmentCoeffs[i].c2*2.0f;
                  coefficientsToController.segmentCoeffs[j].c3 = trajectoryCoeffsThreeSegments.segmentCoeffs[i].c3*6.0f;
               }
            }            
            break;
         }
      }  
   }
   // If the corridor is not valid return trajectory points of zeros
   else
   {
      for (uint8_t i{0U}; i < 2; i++){
         coefficientsToController.segmentCoeffs[i].breakPoint.value() = 0.0f;
         coefficientsToController.segmentCoeffs[i].c0 = 0.0f;
         coefficientsToController.segmentCoeffs[i].c1 = 0.0f;
         coefficientsToController.segmentCoeffs[i].c2 = 0.0f;
         coefficientsToController.segmentCoeffs[i].c3 = 0.0f;
      }        
   }

   return coefficientsToController;
}

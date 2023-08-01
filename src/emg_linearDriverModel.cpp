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

// >>> measurement variables <<<
// linearDriverModel
float ldm_egoPoseX;
float ldm_egoPoseY;
float ldm_egoPoseTheta;
int paramReplanCycle;
float paramP11;
float paramP12;
float paramP13;
float paramP21;
float paramP22;
float paramP23;
float paramP31;
float paramP32;
float paramP33;
float paramNodePointDist1;
float paramNodePointDist2;
float paramNodePointDist3;
// E-LDM lite planner frame
float egoPoseGlobalPlanX;
float egoPoseGlobalPlanY;
float egoPoseGlobalPlanTheta;
// linearDriverModelDriverModel
float nodePoint0X;
float nodePoint0Y;
float nodePoint0Theta;
float nodePoint1X;
float nodePoint1Y;
float nodePoint1Theta;
float nodePoint2X;
float nodePoint2Y;
float nodePoint2Theta;
float nodePoint3X;
float nodePoint3Y;
float nodePoint3Theta;
float kappaNominal1;
float kappaNominal2;
float kappaNominal3;
float x1;
float x2;
float x3;
float X_nominal1;
float X_nominal2;
float X_nominal3;
float Y_nominal1;
float Y_nominal2;
float Y_nominal3;
float theta_nominal1;
float theta_nominal2;
float theta_nominal3;
// linearDriverModelDriverModelLocal
float nodePointLocal0X;
float nodePointLocal0Y;
float nodePointLocal0Theta;
float nodePointLocal1X;
float nodePointLocal1Y;
float nodePointLocal1Theta;
float nodePointLocal2X;
float nodePointLocal2Y;
float nodePointLocal2Theta;
float nodePointLocal3X;
float nodePointLocal3Y;
float nodePointLocal3Theta;
// linearDriverModelSegmentPlanner
float segmentParamsInitPose1X;
float segmentParamsInitPose1Y;
float segmentParamsInitPose1Theta;
float segmentParamsInitPose2X;
float segmentParamsInitPose2Y;
float segmentParamsInitPose2Theta;
float segmentParamsInitPose3X;
float segmentParamsInitPose3Y;
float segmentParamsInitPose3Theta;
float validCoefficient_c0;
float validCoefficient_c1;
float validCoefficient_c2;
float validCoefficient_c3;

// linearDriverModelPolynomialCoefficientsThreeSegments
float c01_ldm;
float c11_ldm;
float c21_ldm;
float c31_ldm;
float breakPoint1_ldm;
float length1_ldm;
float sectionBorderStart1;
float sectionBorderEnd1;
float c02_ldm;
float c12_ldm;
float c22_ldm;
float c32_ldm;
float breakPoint2_ldm;
float length2_ldm;
float sectionBorderStart2;
float sectionBorderEnd2;
float c03_ldm;
float c13_ldm;
float c23_ldm;
float c33_ldm;
float breakPoint3_ldm;
float length3_ldm;
float sectionBorderStart3;
float sectionBorderEnd3;
float controlValidity;
float triggerPlanner_measured;

// corridor coefficients
float c0_refline;
float c1_refline;
float c2_refline;
float c3_refline;

// Parameters
float P_breakPointMinimumDistance = 0.0f; 

namespace Rb
{
namespace Vmc
{

PolynomialCoeffsTwoSegments LinearDriverModel::runCoeffsLite(
   const CorridorInfoCoefficients& corridorCoefficients,
   const Pose2D&                   egoPoseGlobal,
   const LDMParamIn&               parameters)
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

   // measurement

      // Inputs
      c0_refline = corridorCoefficients.c0;
      c1_refline = corridorCoefficients.c1;
      c2_refline = corridorCoefficients.c2;
      c3_refline = corridorCoefficients.c3;
      ldm_egoPoseX = egoPoseGlobal.Pose2DCoordinates.PointsX.value();
      ldm_egoPoseY = egoPoseGlobal.Pose2DCoordinates.PointsY.value();
      ldm_egoPoseTheta = egoPoseGlobal.Pose2DTheta.value();

      // Parameters
      paramReplanCycle = parameters.replanCycle;
      paramP11 = parameters.P[0];
      paramP12 = parameters.P[1];
      paramP13 = parameters.P[2];
      paramP21 = parameters.P[7];
      paramP22 = parameters.P[8];
      paramP23 = parameters.P[9];
      paramP31 = parameters.P[14];
      paramP32 = parameters.P[15];
      paramP33 = parameters.P[16];
      paramNodePointDist1 = parameters.P_nodePointDistances[0];
      paramNodePointDist2 = parameters.P_nodePointDistances[1];
      paramNodePointDist3 = parameters.P_nodePointDistances[2];

      // Internal variables
      egoPoseGlobalPlanX = egoPoseGlobalPlan.Pose2DCoordinates.PointsX.value();
      egoPoseGlobalPlanY = egoPoseGlobalPlan.Pose2DCoordinates.PointsY.value();
      egoPoseGlobalPlanTheta = egoPoseGlobal.Pose2DTheta.value();
      nodePoint0X = nodePoints.nodePointsCoordinates[0].PointsX.value();
      nodePoint0Y = nodePoints.nodePointsCoordinates[0].PointsY.value();
      nodePoint0Theta = nodePoints.nodePointsTheta[0].value();
      nodePoint1X = nodePoints.nodePointsCoordinates[1].PointsX.value();
      nodePoint1Y = nodePoints.nodePointsCoordinates[1].PointsY.value();
      nodePoint1Theta = nodePoints.nodePointsTheta[1].value();
      nodePoint2X = nodePoints.nodePointsCoordinates[2].PointsX.value();
      nodePoint2Y = nodePoints.nodePointsCoordinates[2].PointsY.value();
      nodePoint2Theta = nodePoints.nodePointsTheta[2].value();
      nodePoint3X = nodePoints.nodePointsCoordinates[3].PointsX.value();
      nodePoint3Y = nodePoints.nodePointsCoordinates[3].PointsY.value();
      nodePoint3Theta = nodePointsEgoFrame.nodePointsTheta[3].value();
      nodePointLocal0X = nodePointsEgoFrame.nodePointsCoordinates[0].PointsX.value();
      nodePointLocal0Y = nodePointsEgoFrame.nodePointsCoordinates[0].PointsY.value();
      nodePointLocal0Theta = nodePointsEgoFrame.nodePointsTheta[0].value();
      nodePointLocal1X = nodePointsEgoFrame.nodePointsCoordinates[1].PointsX.value();
      nodePointLocal1Y = nodePointsEgoFrame.nodePointsCoordinates[1].PointsY.value();
      nodePointLocal1Theta = nodePointsEgoFrame.nodePointsTheta[1].value();
      nodePointLocal2X = nodePointsEgoFrame.nodePointsCoordinates[2].PointsX.value();
      nodePointLocal2Y = nodePointsEgoFrame.nodePointsCoordinates[2].PointsY.value();
      nodePointLocal2Theta = nodePointsEgoFrame.nodePointsTheta[2].value();
      nodePointLocal3X = nodePointsEgoFrame.nodePointsCoordinates[3].PointsX.value();
      nodePointLocal3Y = nodePointsEgoFrame.nodePointsCoordinates[3].PointsY.value();
      nodePointLocal3Theta = nodePointsEgoFrame.nodePointsTheta[3].value();
      validCoefficient_c0 = driverModel.validCoefficients.c0;
      validCoefficient_c1 = driverModel.validCoefficients.c1;
      validCoefficient_c2 = driverModel.validCoefficients.c2;
      validCoefficient_c3 = driverModel.validCoefficients.c3;

      if (controlLogic.validity){
         controlValidity = 1.0f;
      }
      else{
         controlValidity = 0.0f;
      }
      if(triggerPlanner){
          triggerPlanner_measured = 1.0f;
      }
      else{
         triggerPlanner_measured = 0.0f;
      }

      // Nominal values of node points
      X_nominal1 = driverModel.XNominal[0];
      X_nominal2 = driverModel.XNominal[1];
      X_nominal3 = driverModel.XNominal[2];
      Y_nominal1 = driverModel.YNominal[0];
      Y_nominal2 = driverModel.YNominal[1];
      Y_nominal3 = driverModel.YNominal[2];
      theta_nominal1 = driverModel.thetaNominal[0];
      theta_nominal2 = driverModel.thetaNominal[1];
      theta_nominal3 = driverModel.thetaNominal[2];
      kappaNominal1 = driverModel.kappaNominal[0];
      kappaNominal2 = driverModel.kappaNominal[1];
      kappaNominal3 = driverModel.kappaNominal[2];
      x1 = driverModel.x[0];
      x2 = driverModel.x[1];
      x3 = driverModel.x[2];
      segmentParamsInitPose1X = segmentParams.initPose[0].PointsX.value();
      segmentParamsInitPose1Y = segmentParams.initPose[0].PointsY.value();
      segmentParamsInitPose1Theta = segmentParams.initPoseTheta[0].value();
      segmentParamsInitPose2X = segmentParams.initPose[1].PointsX.value();
      segmentParamsInitPose2Y = segmentParams.initPose[1].PointsY.value();
      segmentParamsInitPose2Theta = segmentParams.initPoseTheta[1].value();
      segmentParamsInitPose3X = segmentParams.initPose[2].PointsX.value();
      segmentParamsInitPose3Y = segmentParams.initPose[2].PointsY.value();
      segmentParamsInitPose3Theta = segmentParams.initPoseTheta[2].value();
      c01_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[0].c0;
      c11_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[0].c1;
      c21_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[0].c2;
      c31_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[0].c3;
      breakPoint1_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[0].breakPoint.value();
      length1_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[0].length.value();
      sectionBorderStart1 = trajectoryCoeffsThreeSegments.sectionBorderStart[0];
      sectionBorderEnd1 = trajectoryCoeffsThreeSegments.sectionBorderEnd[0];
      c02_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[1].c0;
      c12_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[1].c1;
      c22_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[1].c2;
      c32_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[1].c3;
      breakPoint2_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[1].breakPoint.value();
      length2_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[1].length.value();
      sectionBorderStart2 = trajectoryCoeffsThreeSegments.sectionBorderStart[1];
      sectionBorderEnd2 = trajectoryCoeffsThreeSegments.sectionBorderStart[2];
      c03_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[2].c0;
      c13_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[2].c1;
      c23_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[2].c2;
      c33_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[2].c3;
      breakPoint3_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[2].breakPoint.value();
      length3_ldm = trajectoryCoeffsThreeSegments.segmentCoeffs[2].length.value();
      sectionBorderStart3 = trajectoryCoeffsThreeSegments.sectionBorderStart[2];
      sectionBorderEnd3 = trajectoryCoeffsThreeSegments.sectionBorderEnd[2];

      return coefficientsToController;
}


}  // namespace Emg
}  // namespace Dc

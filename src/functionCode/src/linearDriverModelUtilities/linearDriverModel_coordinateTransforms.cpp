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

#include "../../inc/linearDriverModelUtilities/linearDriverModel_coordinateTransforms.hpp"


void CoordinateTransforms::transform2D(
   const Points2D& inPoints, const Pose2D& egoPoseGlobal, Points2D& outPoints)
{
   float R[2]{
      cos(egoPoseGlobal.Pose2DTheta),
      sin(egoPoseGlobal.Pose2DTheta)};
   float translatedX{0.0};
   float translatedY{0.0};

   translatedX = inPoints.x - egoPoseGlobal.Pose2DCoordinates.x;
   translatedY = inPoints.y - egoPoseGlobal.Pose2DCoordinates.y;

   outPoints.x = R[0] * translatedX + R[1] * translatedY;
   outPoints.y = -R[1] * translatedX + R[0] * translatedY;
}

void CoordinateTransforms::transform2D(
   const TrajectoryPoints  inPoints,
   const Pose2D&           egoPoseGlobal,
   TrajectoryPoints        outPoints)
{
   float R[2]{
      cos(egoPoseGlobal.Pose2DTheta),
      sin(egoPoseGlobal.Pose2DTheta)};
   
   std::vector<float> translatedX;
   translatedX.reserve(inPoints.size());
   std::vector<float> translatedY;
   translatedY.reserve(inPoints.size());
   outPoints.reserve(inPoints.size());

   for (uint16_t i{0}; i < inPoints.size(); i++)
   {
      translatedX[i] = inPoints[i].x - egoPoseGlobal.Pose2DCoordinates.x;
      translatedY[i] = inPoints[i].y - egoPoseGlobal.Pose2DCoordinates.y;

      outPoints[i].x = R[0] * translatedX[i] + R[1] * translatedY[i];
      outPoints[i].y = -R[1] * translatedX[i] + R[0] * translatedY[i];
   }
}

void CoordinateTransforms::reverseTransform2D(
   const Points2D& inPoints, const Pose2D& egoPoseGlobal, Points2D& outPoints)
{
   float R[2]{
      cos(egoPoseGlobal.Pose2DTheta),
      sin(egoPoseGlobal.Pose2DTheta)};
   float rotatedX{0.0};
   float rotatedY{0.0};

   rotatedX = R[0] * inPoints.x - R[1] * inPoints.y;
   rotatedY = R[1] * inPoints.x + R[0] * inPoints.y;

   outPoints.x = rotatedX + egoPoseGlobal.Pose2DCoordinates.x;
   outPoints.y = rotatedY + egoPoseGlobal.Pose2DCoordinates.y;
}

void CoordinateTransforms::reverseTransform2D(
   const TrajectoryPoints  inPoints,
   const Pose2D&           egoPoseGlobal,
   TrajectoryPoints        outPoints)
{
   float R[2]{
      cos(egoPoseGlobal.Pose2DTheta),
      sin(egoPoseGlobal.Pose2DTheta)};
   
   std::vector<float> rotatedX;
   rotatedX.reserve(inPoints.size());
   std::vector<float> rotatedY;
   rotatedY.reserve(inPoints.size());
   outPoints.reserve(inPoints.size());

   for (uint16_t i{0}; i < inPoints.size(); i++)
   {
      rotatedX[i] = R[0] * inPoints[i].x - R[1] * inPoints[i].y;
      rotatedY[i] = R[1] * inPoints[i].x + R[0] * inPoints[i].y;

      outPoints[i].x = rotatedX[i] + egoPoseGlobal.Pose2DCoordinates.x;
      outPoints[i].y = rotatedY[i] + egoPoseGlobal.Pose2DCoordinates.y;
   }
}

void CoordinateTransforms::transformNodePoints(
   const NodePoints& nodePoints,
   const Pose2D&     egoPoseGlobalPlan,
   const Pose2D&     egoPoseGlobal,
   NodePoints&       nodePointsLocal)
{
   float distance = sqrt(
      pow(egoPoseGlobalPlan.Pose2DCoordinates.x - egoPoseGlobal.Pose2DCoordinates.x, 2)
      + pow(egoPoseGlobalPlan.Pose2DCoordinates.y - egoPoseGlobal.Pose2DCoordinates.y, 2));
   float dtheta = egoPoseGlobal.Pose2DTheta - egoPoseGlobalPlan.Pose2DTheta;
   float alfa   = atan2(
      egoPoseGlobal.Pose2DCoordinates.y
         - egoPoseGlobalPlan.Pose2DCoordinates.y,
      egoPoseGlobal.Pose2DCoordinates.x
         - egoPoseGlobalPlan.Pose2DCoordinates.x);
   Pose2D displacementPose{};
   displacementPose.Pose2DCoordinates.x =
      distance * cos(alfa - egoPoseGlobalPlan.Pose2DTheta);
   displacementPose.Pose2DCoordinates.y =
      distance * sin(alfa - egoPoseGlobalPlan.Pose2DTheta);
   displacementPose.Pose2DTheta = dtheta;

   for (uint8_t i{0}; i < 4; i++)
   {
      transform2D(
         nodePoints.nodePointsCoordinates[i], displacementPose, nodePointsLocal.nodePointsCoordinates[i]);
      nodePointsLocal.nodePointsTheta[i] = nodePoints.nodePointsTheta[i] - dtheta;
   }
}

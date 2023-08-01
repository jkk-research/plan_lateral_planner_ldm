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

#include "../emg_linearDriverModel.hpp"

namespace Rb
{
namespace Vmc
{

void CoordinateTransforms::transform2D(
   const Points2D& inPoints, const Pose2D& egoPoseGlobal, Points2D& outPoints)
{
   vfc::float32_t R[2]{vfc::cos(static_cast<vfc::CRadian32>(egoPoseGlobal.Pose2DTheta.value())),
                       vfc::sin(static_cast<vfc::CRadian32>(egoPoseGlobal.Pose2DTheta.value()))};
   vfc::float32_t translatedX{0.0};
   vfc::float32_t translatedY{0.0};

   translatedX = inPoints.PointsX.value() - egoPoseGlobal.Pose2DCoordinates.PointsX.value();
   translatedY = inPoints.PointsY.value() - egoPoseGlobal.Pose2DCoordinates.PointsY.value();

   outPoints.PointsX.value() = R[0] * translatedX + R[1] * translatedY;
   outPoints.PointsY.value() = -R[1] * translatedX + R[0] * translatedY;
}
void CoordinateTransforms::transform2D(
   const Points2D      inPoints[300],
   const Pose2D&       egoPoseGlobal,
   const vfc::uint16_t length,
   Points2D            outPoints[300])
{
   vfc::float32_t R[2]{vfc::cos(static_cast<vfc::CRadian32>(egoPoseGlobal.Pose2DTheta.value())),
                       vfc::sin(static_cast<vfc::CRadian32>(egoPoseGlobal.Pose2DTheta.value()))};
   vfc::float32_t translatedX[300]{0.0};
   vfc::float32_t translatedY[300]{0.0};

   for (uint16_t i{0}; i < length; i++)
   {
      translatedX[i] = inPoints[i].PointsX.value() - egoPoseGlobal.Pose2DCoordinates.PointsX.value();
      translatedY[i] = inPoints[i].PointsY.value() - egoPoseGlobal.Pose2DCoordinates.PointsY.value();

      outPoints[i].PointsX.value() = R[0] * translatedX[i] + R[1] * translatedY[i];
      outPoints[i].PointsY.value() = -R[1] * translatedX[i] + R[0] * translatedY[i];
   }
}

void CoordinateTransforms::reverseTransform2D(
   const Points2D& inPoints, const Pose2D& egoPoseGlobal, Points2D& outPoints)
{
   vfc::float32_t           R[2]{vfc::cos(static_cast<vfc::CRadian32>(egoPoseGlobal.Pose2DTheta.value())),
                       vfc::sin(static_cast<vfc::CRadian32>(egoPoseGlobal.Pose2DTheta.value()))};
   vfc::CSI::si_metre_f32_t rotatedX{0.0};
   vfc::CSI::si_metre_f32_t rotatedY{0.0};

   rotatedX.value() = R[0] * inPoints.PointsX.value() - R[1] * inPoints.PointsY.value();
   rotatedY.value() = R[1] * inPoints.PointsX.value() + R[0] * inPoints.PointsY.value();

   outPoints.PointsX = rotatedX + egoPoseGlobal.Pose2DCoordinates.PointsX;
   outPoints.PointsY = rotatedY + egoPoseGlobal.Pose2DCoordinates.PointsY;
}

void CoordinateTransforms::reverseTransform2D(
   const Points2D      inPoints[300],
   const Pose2D&       egoPoseGlobal,
   const vfc::uint16_t length,
   Points2D            outPoints[300])
{
   vfc::float32_t           R[2]{vfc::cos(static_cast<vfc::CRadian32>(egoPoseGlobal.Pose2DTheta.value())),
                       vfc::sin(static_cast<vfc::CRadian32>(egoPoseGlobal.Pose2DTheta.value()))};
   vfc::CSI::si_metre_f32_t rotatedX[300]{static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};
   vfc::CSI::si_metre_f32_t rotatedY[300]{static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};

   for (uint16_t i{0}; i < length; i++)
   {
      rotatedX[i].value() = R[0] * inPoints[i].PointsX.value() - R[1] * inPoints[i].PointsY.value();
      rotatedY[i].value() = R[1] * inPoints[i].PointsX.value() + R[0] * inPoints[i].PointsY.value();

      outPoints[i].PointsX = rotatedX[i] + egoPoseGlobal.Pose2DCoordinates.PointsX;
      outPoints[i].PointsY = rotatedY[i] + egoPoseGlobal.Pose2DCoordinates.PointsY;
   }
}

void CoordinateTransforms::angleTransform2D(
   const Pose2D& inPose, const Pose2D& egoPoseGlobal, Pose2D& outPose)
{
   outPose.Pose2DTheta = inPose.Pose2DTheta + egoPoseGlobal.Pose2DTheta;
}
void CoordinateTransforms::angleTransform2D(
   const CorridorInfo& corridorIn,
   const Pose2D&       egoPoseGlobal,
   const vfc::uint16_t length,
   CorridorInfo&       corridorOut)
{
   for (uint16_t i{0}; i < length; i++)
   {
      corridorOut.corridorOrientation[i] = corridorIn.corridorOrientation[i] + egoPoseGlobal.Pose2DTheta;
   }
   memcpy(corridorOut.corridorCurvature, corridorIn.corridorCurvature, sizeof(corridorIn.corridorCurvature));
}

void CoordinateTransforms::transformNodePoints(
   const NodePoints& nodePoints,
   const Pose2D&     egoPoseGlobalPlan,
   const Pose2D&     egoPoseGlobal,
   NodePoints&       nodePointsLocal)
{
   vfc::CSI::si_metre_f32_t distance = vfc::sqrt(
      vfc::sqr(egoPoseGlobalPlan.Pose2DCoordinates.PointsX - egoPoseGlobal.Pose2DCoordinates.PointsX)
      + vfc::sqr(egoPoseGlobalPlan.Pose2DCoordinates.PointsY - egoPoseGlobal.Pose2DCoordinates.PointsY));
   vfc::CSI::si_radian_f32_t dtheta = egoPoseGlobal.Pose2DTheta - egoPoseGlobalPlan.Pose2DTheta;
   vfc::float32_t            alfa   = vfc::atan2<vfc::TRadian, vfc::float32_t>(
                            egoPoseGlobal.Pose2DCoordinates.PointsY.value()
                               - egoPoseGlobalPlan.Pose2DCoordinates.PointsY.value(),
                            egoPoseGlobal.Pose2DCoordinates.PointsX.value()
                               - egoPoseGlobalPlan.Pose2DCoordinates.PointsX.value())
                            .value();
   Pose2D displacementPose{};
   displacementPose.Pose2DCoordinates.PointsX =
      distance * vfc::cos(static_cast<vfc::CRadian32>(alfa - egoPoseGlobalPlan.Pose2DTheta.value()));
   displacementPose.Pose2DCoordinates.PointsY =
      distance * vfc::sin(static_cast<vfc::CRadian32>(alfa - egoPoseGlobalPlan.Pose2DTheta.value()));
   displacementPose.Pose2DTheta = dtheta;

   for (uint8_t i{0}; i < 4; i++)
   {
      transform2D(
         nodePoints.nodePointsCoordinates[i], displacementPose, nodePointsLocal.nodePointsCoordinates[i]);
      nodePointsLocal.nodePointsTheta[i] = nodePoints.nodePointsTheta[i] - dtheta;
   }
}


}  // namespace Emg
}  // namespace Dc

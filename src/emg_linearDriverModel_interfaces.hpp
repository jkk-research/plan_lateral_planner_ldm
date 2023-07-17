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

#ifndef DC_EMG_LINEARDRIVERMODEL_INTERFACES_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_INTERFACES_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_siunits_convenienttypes.hpp"
#include "vfc/container/vfc_fixedvector.hpp"
#include "vfc/container/vfc_carray.hpp"


namespace Dc
{
namespace Emg
{

struct Points2D
{
   vfc::CSI::si_metre_f32_t PointsX{0.0f};
   vfc::CSI::si_metre_f32_t PointsY{0.0f};
};

struct CorridorInfo
{
   Points2D                     corridorPoints[300];
   vfc::CSI::si_radian_f32_t    corridorOrientation[300]{static_cast<vfc::CSI::si_radian_f32_t>(0.0)};
   vfc::CSI::si_per_metre_f32_t corridorCurvature[300]{static_cast<vfc::CSI::si_per_metre_f32_t>(0.0)};
   vfc::uint16_t                corridorLength{0U};
};

struct CorridorInfoCoefficients
{
   vfc::float32_t c0{0.0f};
   vfc::float32_t c1{0.0f};
   vfc::float32_t c2{0.0f};
   vfc::float32_t c3{0.0f};
};

struct Pose2D
{
   Points2D                  Pose2DCoordinates;
   vfc::CSI::si_radian_f32_t Pose2DTheta{0.0f};
};

struct TrajectoryPoints
{
   Points2D      trajectoryPoints[300];
   vfc::uint16_t trajectoryLength{0U};
};

struct NodePoints
{
   Points2D                  nodePointsCoordinates[4];
   vfc::CSI::si_radian_f32_t nodePointsTheta[4]{static_cast<vfc::CSI::si_radian_f32_t>(0.0f)};
};

struct EvalPoints
{
   vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U> evalPointsSegment1{
      static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};
   vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U> evalPointsSegment2{
      static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};
   vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U> evalPointsSegment3{
      static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};
   vfc::uint16_t evalPointsLengths[3]{0U, 0U, 0U};
};

struct SegmentParams
{
   Points2D                            initPose[3];
   vfc::CSI::si_radian_f32_t           initPoseTheta[3]{static_cast<vfc::CSI::si_radian_f32_t>(0.0f)};
   vfc::CSI::si_per_metre_f32_t        k[3]{static_cast<vfc::CSI::si_per_metre_f32_t>(0.0f)};
   vfc::CSI::si_per_square_metre_f32_t dk[3]{static_cast<vfc::CSI::si_per_square_metre_f32_t>(0.0f)};
   vfc::CSI::si_metre_f32_t            L[3]{static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};
};

struct PolynomialCoeffs
{
   vfc::float32_t           c0{0.0f};
   vfc::float32_t           c1{0.0f};
   vfc::float32_t           c2{0.0f};
   vfc::float32_t           c3{0.0f};
   vfc::CSI::si_metre_f32_t length{0.0f};
   vfc::CSI::si_metre_f32_t breakPoint{0.0f};
};

struct PolynomialCoeffsThreeSegments
{
   PolynomialCoeffs segmentCoeffs[3];
   vfc::float32_t   sectionBorderStart[3];
   vfc::float32_t   sectionBorderEnd[3];
};

struct TrajectoryPolynomial
{
   PolynomialCoeffs polynomialCoeffs;
   TrajectoryPoints trajectory;
};

struct LDMParamIn
{
   vfc::CSI::si_metre_f32_t lookAheadDistance{250.0f};
   vfc::uint16_t            replanCycle{25U};
   vfc::float32_t           P[21]{0.0f};
   vfc::float32_t           P_nodePointDistances[3]{0.0f, 0.1160f, 0.3840f};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_INTERFACES_HPP_INCLUDED

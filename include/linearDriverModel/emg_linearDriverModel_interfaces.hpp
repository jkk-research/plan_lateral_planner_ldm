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

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "lane_keep_system/PolynomialCoeffs.h"

#include <string>
#include <vector>


typedef geometry_msgs::Point Points2D;

typedef std::vector<Points2D> TrajectoryPoints;

typedef lane_keep_system::PolynomialCoeffs PolynomialCoeffs;

// was CorridorInfoCoefficients
struct ScenarioPolynomials
{
   std::vector<PolynomialCoeffs> coeffs;
   std::vector<float>            kappaNominal;
};


// ? instead use scenarioCoeffs
// struct CorridorInfoCoefficients
// {
//    float c0{0.0f};
//    float c1{0.0f};
//    float c2{0.0f};
//    float c3{0.0f};
// };

// struct CorridorInfo
// {
//    TrajectoryPoints     corridorPoints;
//    std::vector<float>   corridorOrientation;
//    std::vector<float>   corridorCurvature;
//    uint16_t             corridorLength{0U};
// };

struct Pose2D
{
   Points2D Pose2DCoordinates;
   float    Pose2DTheta{0.0f};
};

struct NodePoints
{
   Points2D nodePointsCoordinates[4];
   float    nodePointsTheta[4]{0.0f};
};

// struct SegmentParams
// {
//    Points2D                            initPose[3];
//    vfc::CSI::si_radian_f32_t           initPoseTheta[3]{static_cast<vfc::CSI::si_radian_f32_t>(0.0f)};
//    vfc::CSI::si_per_metre_f32_t        k[3]{static_cast<vfc::CSI::si_per_metre_f32_t>(0.0f)};
//    vfc::CSI::si_per_square_metre_f32_t dk[3]{static_cast<vfc::CSI::si_per_square_metre_f32_t>(0.0f)};
//    vfc::CSI::si_metre_f32_t            L[3]{static_cast<vfc::CSI::si_metre_f32_t>(0.0f)};
// };

// struct PolynomialCoeffsTwoSegments
// {
//    PolynomialCoeffs segmentCoeffs[2];
// };

struct PolynomialCoeffsThreeSegments
{
   PolynomialCoeffs  segmentCoeffs[3];
   float             sectionBorderStart[3];
   float             sectionBorderEnd[3];
};

// struct TrajectoryPolynomial
// {
//    PolynomialCoeffs polynomialCoeffs;
//    TrajectoryPoints trajectory;
// };

struct LDMParamIn
{
   // float lookAheadDistance{250.0f};
   int   replanCycle{25U};
   float P[21]{0.0f};
   float P_nodePointDistances[3]{6.0f, 24.0f, 80.0f};
};

#endif  // DC_EMG_LINEARDRIVERMODEL_INTERFACES_HPP_INCLUDED

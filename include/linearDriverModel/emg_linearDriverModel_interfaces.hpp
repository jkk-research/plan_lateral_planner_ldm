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

#ifndef LINEARDRIVERMODEL_INTERFACES_HPP_INCLUDED
#define LINEARDRIVERMODEL_INTERFACES_HPP_INCLUDED

#include "lane_keep_system/PolynomialCoeffs.h"

#include <string>
#include <vector>


struct Points2D
{
   float x;
   float y;
};

typedef std::vector<Points2D> TrajectoryPoints;

typedef lane_keep_system::PolynomialCoeffs PolynomialCoeffs;

struct ScenarioPolynomials
{
   std::vector<PolynomialCoeffs> coeffs;
   std::vector<float>            kappaNominal;
};

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

struct SegmentParams
{
   Points2D initPose[3];
   float    initPoseTheta[3]{0.0f};
   float    k[3]{0.0f};
   float    dk[3]{0.0f};
   float    L[3]{0.0f};
};

struct PolynomialCoeffsThreeSegments
{
   PolynomialCoeffs  segmentCoeffs[3];
   float             sectionBorderStart[3];
   float             sectionBorderEnd[3];
};

struct LDMParamIn
{
   // float lookAheadDistance{250.0f};
   int   replanCycle{25U};
   float P[21]{0.0f};
   float P_nodePointDistances[3]{6.0f, 24.0f, 80.0f};
};

#endif  // LINEARDRIVERMODEL_INTERFACES_HPP_INCLUDED

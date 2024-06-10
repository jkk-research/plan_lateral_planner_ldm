﻿///=============================================================================
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

#ifndef LINEARDRIVERMODEL_HPP_INCLUDED
#define LINEARDRIVERMODEL_HPP_INCLUDED

#include "linearDriverModel_interfaces.hpp"
#include "linearDriverModel_controlLogic.hpp"
#include "../linearDriverModelDriverModel/linearDriverModel_driverModel.hpp"
#include "../linearDriverModelPlanner/linearDriverModel_segmentPlanner.hpp"
#include "../linearDriverModelUtilities/linearDriverModel_coordinateTransforms.hpp"

#include <cstring>

class LinearDriverModel  // cover class of DriverTrajectoryPlanner
{
public:
   TrajectoryOutput runCoeffsLite(
      const ScenarioPolynomials& corridorCoefficients, const Pose2D& egoPoseGlobal, const LDMParamIn& parameters);

   void initCoeffs(const ScenarioPolynomials& corridorCoefficients);

   // Control Logic
   ControlLogic controlLogic{};
   
   // Driver Model
   DriverModel driverModel{};

   // internal input and output structs
   NodePoints nodePoints{};
   NodePoints nodePointsEgoFrame{};
   Pose2D     egoPoseGlobalPlan{};
   // Evaluation Points calculation for trajectory planner
   // Preprocess

private:
   // Transforms
   CoordinateTransforms coordinateTransforms{};

   // Curve fitting - segment planner
   SegmentPlanner segmentPlanner{};
   SegmentParams  segmentParams{};
   SegmentParams  segmentParamsEgoFrame{};

   // Results of the curve fitting and return of coefficients towards TRC
   PolynomialCoeffsThreeSegments trajectoryCoeffsThreeSegments{};

   // internal variables
   bool                     firstCycle{true};
};

#endif  // LINEARDRIVERMODEL_HPP_INCLUDED
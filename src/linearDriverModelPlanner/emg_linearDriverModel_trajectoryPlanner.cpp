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

namespace Dc
{
namespace Emg
{

void TrajectoryPlanner::calc(
   const NodePoints& nodePoints,
   const EvalPoints& evalPoints,
   SegmentParams&    segmentParams,
   TrajectoryPoints& trajectoryPoints)
{
   for (uint8_t startIndex{0U}; startIndex <= 2; startIndex++)
   {
      segmentPlanner.buildClothoid(nodePoints, segmentParams, startIndex);
      trajectoryEvaluation.calc(segmentParams, evalPoints, trajectoryPoints, startIndex);
   }
}

void TrajectoryPlanner::calc(
   const NodePoints& nodePoints,
   const EvalPoints& evalPoints,
   PolynomialCoeffs& polynomialCoeffs,
   SegmentParams&    segmentParams,
   TrajectoryPoints& trajectoryPoints)
{
   segmentPlanner.buildPolynomial(nodePoints, polynomialCoeffs, segmentParams);
   trajectoryEvaluation.calc(polynomialCoeffs, evalPoints, trajectoryPoints);
}

}  // namespace Emg
}  // namespace Dc

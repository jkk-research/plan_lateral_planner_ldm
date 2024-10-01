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

#ifndef LINEARDRIVERMODEL_SEGMENTPLANNER_HPP_INCLUDED
#define LINEARDRIVERMODEL_SEGMENTPLANNER_HPP_INCLUDED

#include "../linearDriverModel/linearDriverModel_interfaces.hpp"
#include "../linearDriverModelUtilities/linearDriverModel_polynomialSubfunctions.hpp"


class SegmentPlanner
{
public:
   // main method
   void buildThreeSegmentPolynomial(
      const NodePoints&              nodePoints,
      PolynomialCoeffsThreeSegments& polynomialCoeffs,
      SegmentParams&                 segmentParams);
      
private:
   // helper class
   PolynomialSubfunctions m_polynomialSubfunctions{};
   // helper variable
   float m_gaussMatrix[4][5]{{0.0f}};
};

#endif  // LINEARDRIVERMODEL_SEGMENTPLANNER_HPP_INCLUDED

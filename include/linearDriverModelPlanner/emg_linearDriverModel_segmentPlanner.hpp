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

#ifndef DC_EMG_LINEARDRIVERMODEL_SEGMENTPLANNER_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_SEGMENTPLANNER_HPP_INCLUDED

#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"
#include "linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"


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
   PolynomialSubfunctions polynomialSubfunctions{};
   // helper variable
   float gaussMatrix[4][5]{{0.0f}};
};

#endif  // DC_EMG_LINEARDRIVERMODEL_SEGMENTPLANNER_HPP_INCLUDED

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

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_math.hpp"
#include "vfc/core/vfc_algorithm.hpp"

#include "../emg_linearDriverModel_interfaces.hpp"
#include "../LinearDriverModelUtilities/emg_linearDriverModel_clothoidSubfunctions.hpp"
#include "../LinearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"

namespace Rb
{
namespace Vmc
{

class SegmentPlanner
{
public:
   // main method
   void buildClothoid(const NodePoints&, SegmentParams&, vfc::uint8_t);
   void buildPolynomial(const NodePoints&, PolynomialCoeffs&, SegmentParams&);
   void buildThreeSegmentPolynomial(
      const NodePoints&              nodePoints,
      PolynomialCoeffsThreeSegments& polynomialCoeffs,
      SegmentParams&                 segmentParams);
      
private:
   // helper class
   ClothoidSubfunctions   clothoidSubfunctions{};
   PolynomialSubfunctions polynomialSubfunctions{};
   // helper variable
   vfc::float32_t gaussMatrix[4][5]{{0.0f}};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_SEGMENTPLANNER_HPP_INCLUDED

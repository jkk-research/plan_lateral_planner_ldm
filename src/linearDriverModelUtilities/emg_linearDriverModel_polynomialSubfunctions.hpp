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

#ifndef DC_EMG_LINEARDRIVERMODEL_POLYNOMIALSUBFUNCTIONS_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_POLYNOMIALSUBFUNCTIONS_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

#include "../emg_linearDriverModel_interfaces.hpp"


namespace Dc
{
namespace Emg
{

class PolynomialSubfunctions
{
public:
   // main function for gauss elimination
   void gaussElimination(vfc::float32_t (&gaussMatrix)[4][5], PolynomialCoeffs&);
   // main function for 3rd order polynomial fitting
   PolynomialCoeffs fitThirdOrderPolynomial(const TrajectoryPoints&);

private:
   // helper methods for gauss elimination
   vfc::int8_t forwardElimination(vfc::float32_t (&gaussMatrix)[4][5]);
   void        backSubstitute(vfc::float32_t (&gaussMatrix)[4][5]);
   // helper variable for gauss elimination
   vfc::float32_t gaussResult[4]{0.0f};
   // helper methods for polynomial fitting
   void           calculateBvector(const TrajectoryPoints&);
   void           calculateMmatrix(const TrajectoryPoints&);
   vfc::float32_t calculateDeterminant(vfc::float32_t Mx[4][4]);
   vfc::float32_t calculateSubDeterminant(vfc::float32_t Mx[4][4], vfc::uint8_t);
   void           calculateModifiedMMatrix(vfc::uint8_t);
   // helper variable for polynomial fitting
   PolynomialCoeffs polyCoeffs{};
   vfc::float32_t   a[4];
   vfc::float32_t   v[9];
   vfc::float32_t   M_[4][4]{0.0f};
   vfc::float32_t   M[4][4]{0.0f};
   vfc::float32_t   detM;
   vfc::float32_t   b[4]{0.0f};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_POLYNOMIALSUBFUNCTIONS_HPP_INCLUDED

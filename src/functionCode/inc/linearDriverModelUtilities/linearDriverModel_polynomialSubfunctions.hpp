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

#ifndef LINEARDRIVERMODEL_POLYNOMIALSUBFUNCTIONS_HPP_INCLUDED
#define LINEARDRIVERMODEL_POLYNOMIALSUBFUNCTIONS_HPP_INCLUDED

#include "../linearDriverModel/linearDriverModel_interfaces.hpp"


class PolynomialSubfunctions
{
public:
   // main function for gauss elimination
   void gaussElimination(float (&gaussMatrix)[4][5], PolynomialCoeffs&);
   // main function for 3rd order polynomial fitting
   PolynomialCoeffs fitThirdOrderPolynomial(const TrajectoryPoints&);

private:
   // helper methods for gauss elimination
   uint8_t forwardElimination(float (&gaussMatrix)[4][5]);
   void backSubstitute(float (&gaussMatrix)[4][5]);
   // helper variable for gauss elimination
   float gaussResult[4]{0.0f};
   // helper methods for polynomial fitting
   void calculateBvector(const TrajectoryPoints&);
   void calculateMmatrix(const TrajectoryPoints&);
   double calculateDeterminant(double Mx[4][4]);
   double calculateSubDeterminant(double Mx[4][4], uint8_t);
   void calculateModifiedMMatrix(uint8_t);
   // helper variable for polynomial fitting
   PolynomialCoeffs polyCoeffs{};
   float  a[4];
   double v[9];
   double M_[4][4]{{0.0f}};
   double M[4][4]{{0.0f}};
   double detM;
   double b[4]{0.0f};
};

#endif  // LINEARDRIVERMODEL_POLYNOMIALSUBFUNCTIONS_HPP_INCLUDED

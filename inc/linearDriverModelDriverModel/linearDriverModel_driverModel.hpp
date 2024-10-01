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

#ifndef LINEARDRIVERMODEL_DRIVERMODEL_HPP_INCLUDED
#define LINEARDRIVERMODEL_DRIVERMODEL_HPP_INCLUDED

#include "../linearDriverModel/linearDriverModel_interfaces.hpp"


class DriverModel
{
public:
   // arguments
   float    m_U[7]{0.0f};
   float    m_x[3]{0.0f};
   float    m_nominal[12]{0.0f};
   uint16_t m_indices[4]{0U};
   
   float    m_XNominal[3]{0.0f};
   float    m_YNominal[3]{0.0f};
   float    m_thetaNominal[3]{0.0f};
   float    m_kappaNominal[3]{0.0f};
   PolynomialCoeffs m_validCoefficients{};

   void driverModelPlannerLite(
      const ScenarioPolynomials&,
      const PolynomialCoeffsThreeSegments&,
      const LDMParamIn&,
      NodePoints&);

private:
   // helper methods
   void offsetCalcExtendedLDM(const LDMParamIn&, const std::vector<float>);
};


#endif  // LINEARDRIVERMODEL_DRIVERMODEL_HPP_INCLUDED

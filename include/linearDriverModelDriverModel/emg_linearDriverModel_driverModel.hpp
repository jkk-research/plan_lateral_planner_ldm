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

#ifndef DC_EMG_LINEARDRIVERMODEL_DRIVERMODEL_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_DRIVERMODEL_HPP_INCLUDED

#include "linearDriverModel/emg_linearDriverModel_interfaces.hpp"


class DriverModel
{
public:
   // main method
   void calc(const CorridorInfo&, const LDMParamIn&, NodePoints&);
   // arguments
   float    U[7]{0.0f};
   float    x[3]{0.0f};
   float    nominal[12]{0.0f};
   uint16_t indices[4]{0U};
   
   float    XNominal[3]{0.0f};
   float    YNominal[3]{0.0f};
   float    thetaNominal[3]{0.0f};
   float    kappaNominal[3]{0.0f};
   PolynomialCoeffs validCoefficients{};

   void driverModelPlannerLite(
      const ScenarioPolynomials&,
      const PolynomialCoeffsThreeSegments&,
      const LDMParamIn&,
      NodePoints&);

private:
   // helper methods
   void offsetCalcExtendedLDM(const LDMParamIn&, const std::vector<float>);
};


#endif  // DC_EMG_LINEARDRIVERMODEL_DRIVERMODEL_HPP_INCLUDED

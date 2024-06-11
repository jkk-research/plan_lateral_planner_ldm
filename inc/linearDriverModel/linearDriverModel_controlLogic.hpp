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

#ifndef LINEARDRIVERMODEL_CONTROLLOGIC_HPP_INCLUDED
#define LINEARDRIVERMODEL_CONTROLLOGIC_HPP_INCLUDED

#include "../linearDriverModel/linearDriverModel_interfaces.hpp"


class ControlLogic
{
public:
   // main method
   bool calcValidity(const ScenarioPolynomials&, const Pose2D&, const LDMParamIn&, const bool);
   // helper methods
   void corridorValidityCalc(const ScenarioPolynomials&, const Pose2D&);
   void initalizedCalc(const bool, const bool);
   void replanCalc(const LDMParamIn&);

   // arguments
   float   egoPoseDatasX[4]{0.0f};
   float   egoPoseDatasY[4]{0.0f};
   float   egoPoseDatasTheta[4]{0.0f};
   bool    initialized{false};
   bool    validity{false};
   bool    replan{false};
   uint8_t replanCount{0U};
};

#endif  // LINEARDRIVERMODEL_CONTROLLOGIC_HPP_INCLUDED

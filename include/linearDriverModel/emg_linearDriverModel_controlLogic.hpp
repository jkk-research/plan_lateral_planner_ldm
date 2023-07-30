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

#ifndef DC_EMG_LINEARDRIVERMODEL_CONTROLLOGIC_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_CONTROLLOGIC_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

#include "emg_linearDriverModel_interfaces.hpp"

namespace Rb
{
namespace Vmc
{

class ControlLogic
{
public:
   // main method
   bool calcValidity(const CorridorInfo&, const CorridorInfo&, const Pose2D&, const LDMParamIn&, const bool);
   bool calcValidity(const CorridorInfoCoefficients&, const Pose2D&, const LDMParamIn&, const bool);
   // helper methods
   void corridorValidityCalc(const CorridorInfo&, const CorridorInfo&, const Pose2D&);
   void corridorValidityCalc(const CorridorInfoCoefficients&, const Pose2D&);
   void initalizedCalc(const CorridorInfo&, const Pose2D&, const bool, const bool);
   void initalizedCalc(const bool, const bool);
   void replanCalc(const LDMParamIn&);

   // arguments
   vfc::float32_t egoPoseDatasX[4]{0.0f};
   vfc::float32_t egoPoseDatasY[4]{0.0f};
   vfc::float32_t egoPoseDatasTheta[4]{0.0f};
   bool           initialized{false};
   bool           validity{false};
   bool           replan{false};
   vfc::uint8_t   replanCount{0U};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_CONTROLLOGIC_HPP_INCLUDED

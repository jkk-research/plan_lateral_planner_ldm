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

#ifndef DC_EMG_LINEARDRIVERMODEL_CLOTHOIDSUBFUNCTIONS_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_CLOTHOIDSUBFUNCTIONS_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_math.hpp"
#include "vfc/core/vfc_algorithm.hpp"

#include "../emg_linearDriverModel_interfaces.hpp"


namespace Rb
{
namespace Vmc
{

class ClothoidSubfunctions
{
public:
   vfc::float32_t normalizeAngle(vfc::float32_t);
   vfc::float32_t guessA(vfc::float32_t, vfc::float32_t);
   vfc::float32_t findA(vfc::float32_t, vfc::float32_t, vfc::float32_t, vfc::float32_t);
   void           generalizedFresnelCS(
      vfc::uint8_t,
      vfc::float32_t,
      vfc::float32_t,
      vfc::float32_t,
      vfc::float32_t (&X)[3],
      vfc::float32_t (&Y)[3]);
   // arguments
   vfc::float32_t h{0};
   vfc::float32_t g{0};
   vfc::float32_t X[3]{0};
   vfc::float32_t Y[3]{0};

private:
   // helper methods
   void           fresnelCS(vfc::float32_t, vfc::float32_t&, vfc::float32_t&);
   void           fresnelCSk(vfc::uint8_t, vfc::float32_t, vfc::float32_t (&C)[3], vfc::float32_t (&S)[3]);
   void           evalXYaLarge(
       vfc::uint8_t,
       vfc::float32_t,
       vfc::float32_t,
       vfc::float32_t (&X)[3],
       vfc::float32_t (&Y)[3]);
   vfc::float32_t rLommel(vfc::float32_t, vfc::float32_t, vfc::float32_t);
   void evalXYaZero(vfc::uint8_t, vfc::float32_t, vfc::float32_t (&X0)[17], vfc::float32_t (&Y0)[17]);
   void evalXYaSmall(
      vfc::uint8_t,
      vfc::float32_t,
      vfc::float32_t,
      vfc::uint8_t,
      vfc::float32_t (&X)[3],
      vfc::float32_t (&Y)[3]);
   // arguments
   vfc::float32_t intC[3]{0};
   vfc::float32_t intS[3]{0};
   vfc::float32_t Cl[3]{0};
   vfc::float32_t Sl[3]{0};
   vfc::float32_t Cz[3]{0};
   vfc::float32_t Sz[3]{0};
   vfc::float32_t X0[17]{0};
   vfc::float32_t Y0[17]{0};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_CLOTHOIDSUBFUNCTIONS_HPP_INCLUDED

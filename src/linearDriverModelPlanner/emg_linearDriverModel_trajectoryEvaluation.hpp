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

#ifndef DC_EMG_LINEARDRIVERMODEL_TRAJECTORYEVALUATION_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_TRAJECTORYEVALUATION_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

#include "../emg_linearDriverModel_interfaces.hpp"
#include "../linearDriverModelPlanner/emg_linearDriverModel_segmentPlanner.hpp"


namespace Dc
{
namespace Emg
{

class TrajectoryEvaluation
{
public:
   // main method
   void calc(const SegmentParams&, const EvalPoints&, TrajectoryPoints&, vfc::uint8_t);
   void calc(const PolynomialCoeffs&, const EvalPoints&, TrajectoryPoints&);

private:
   // helper method for interp1D
   void interpolation1D(
      const vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U>&,
      const vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U>&,
      const vfc::uint16_t,
      const vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U>&,
      vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U>&,
      const vfc::uint16_t);
   // arguments
   vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U> X{static_cast<vfc::CSI::si_metre_f32_t>(0.0F)};
   vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U> Y{static_cast<vfc::CSI::si_metre_f32_t>(0.0F)};
   vfc::float32_t                               C[3]{0};
   vfc::float32_t                               S[3]{0};
   ClothoidSubfunctions                         clothoidSubfunctions{};
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_TRAJECTORYEVALUATION_HPP_INCLUDED

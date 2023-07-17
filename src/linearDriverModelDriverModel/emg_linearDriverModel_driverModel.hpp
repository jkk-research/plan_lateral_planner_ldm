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

#include "vfc/core/vfc_types.hpp"

#include "../emg_linearDriverModel_interfaces.hpp"


namespace Dc
{
namespace Emg
{

class DriverModel
{
public:
   // main method
   void calc(const CorridorInfo&, const LDMParamIn&, NodePoints&);
   // arguments
   vfc::float32_t U[7]{0.0f};
   vfc::float32_t x[3]{0.0f};
   vfc::float32_t nominal[12]{0.0f};
   vfc::uint16_t  indices[4]{0U};
   void           driverModelPlanner(const CorridorInfo&, const LDMParamIn&, NodePoints&);
   void           driverModelPlannerLite(
                const CorridorInfoCoefficients&,
                const PolynomialCoeffsThreeSegments& trajectoryCoeffsThreeSegments,
                const LDMParamIn&,
                NodePoints&);

private:
   // helper methods
   void nodePointModel(const CorridorInfo&, const LDMParamIn&);
   void offsetCalcLDM(const LDMParamIn&, const vfc::float32_t*, const vfc::float32_t*);
   void offsetCalcExtendedLDM(const LDMParamIn&, const vfc::float32_t*);
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_DRIVERMODEL_HPP_INCLUDED

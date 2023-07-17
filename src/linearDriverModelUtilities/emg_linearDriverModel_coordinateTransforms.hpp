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

#ifndef DC_EMG_LINEARDRIVERMODEL_COORDINATETRANSFORMS_HPP_INCLUDED
#define DC_EMG_LINEARDRIVERMODEL_COORDINATETRANSFORMS_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

#include "../emg_linearDriverModel_interfaces.hpp"


namespace Dc
{
namespace Emg
{

class CoordinateTransforms
{
public:
   void transform2D(const Points2D&, const Pose2D&, Points2D&);
   void transform2D(const Points2D[300], const Pose2D&, const vfc::uint16_t, Points2D[300]);
   void reverseTransform2D(const Points2D&, const Pose2D&, Points2D&);
   void reverseTransform2D(const Points2D[300], const Pose2D&, const vfc::uint16_t, Points2D[300]);
   void angleTransform2D(const Pose2D&, const Pose2D&, Pose2D&);
   void angleTransform2D(const CorridorInfo&, const Pose2D&, const vfc::uint16_t, CorridorInfo&);
   void transformNodePoints(const NodePoints&, const Pose2D&, const Pose2D&, NodePoints&);
};

}  // namespace Emg
}  // namespace Dc

#endif  // DC_EMG_LINEARDRIVERMODEL_COORDINATETRANSFORM_HPP_INCLUDED

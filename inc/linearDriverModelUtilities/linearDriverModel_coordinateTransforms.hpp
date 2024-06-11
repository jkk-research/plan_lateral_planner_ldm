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

#ifndef LINEARDRIVERMODEL_COORDINATETRANSFORMS_HPP_INCLUDED
#define LINEARDRIVERMODEL_COORDINATETRANSFORMS_HPP_INCLUDED

#include "../linearDriverModel/linearDriverModel_interfaces.hpp"


class CoordinateTransforms
{
public:
   void transform2D(const Points2D&, const Pose2D&, Points2D&);
   void transform2D(const TrajectoryPoints, const Pose2D&, TrajectoryPoints);
   void reverseTransform2D(const Points2D&, const Pose2D&, Points2D&);
   void reverseTransform2D(const TrajectoryPoints, const Pose2D&, TrajectoryPoints);
   void transformNodePoints(const NodePoints&, const Pose2D&, const Pose2D&, NodePoints&);
};

#endif  // LINEARDRIVERMODEL_COORDINATETRANSFORM_HPP_INCLUDED

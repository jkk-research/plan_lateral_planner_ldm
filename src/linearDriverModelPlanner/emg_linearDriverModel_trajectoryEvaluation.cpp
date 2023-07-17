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

#include "../emg_linearDriverModel.hpp"

namespace Dc
{
namespace Emg
{

void TrajectoryEvaluation::calc(
   const SegmentParams& segmentParams,
   const EvalPoints&    evalPoints,
   TrajectoryPoints&    trajectoryPoints,
   vfc::uint8_t         startIndex)
{
   vfc::float32_t x0     = segmentParams.initPose[startIndex].PointsX.value();
   vfc::float32_t y0     = segmentParams.initPose[startIndex].PointsY.value();
   vfc::float32_t theta0 = segmentParams.initPoseTheta[startIndex].value();

   if (
      (startIndex == 1
       && (segmentParams.initPose[1].PointsX.value() == 0.0f
           && segmentParams.initPose[1].PointsY.value() == 0.0f
           && segmentParams.initPoseTheta[1].value() == 0.0f))
      || (startIndex == 2
          && (segmentParams.initPose[2].PointsX.value() == 0.0f
              && segmentParams.initPose[2].PointsY.value() == 0.0f
              && segmentParams.initPoseTheta[2].value() == 0.0f))
      || evalPoints.evalPointsLengths[startIndex] == 0)
   {
      memset(trajectoryPoints.trajectoryPoints, 0.0f, sizeof(trajectoryPoints.trajectoryPoints));
      trajectoryPoints.trajectoryLength = 0;
   }
   else
   {
      // reseting trajectory
      if (startIndex == 0)
      {
         memset(trajectoryPoints.trajectoryPoints, 0.0f, sizeof(trajectoryPoints.trajectoryPoints));
         trajectoryPoints.trajectoryLength = 0;
      }

      vfc::float32_t kappa  = segmentParams.k[startIndex].value();
      vfc::float32_t dkappa = segmentParams.dk[startIndex].value();
      vfc::float32_t L      = segmentParams.L[startIndex].value();

      vfc::uint16_t npts = evalPoints.evalPointsLengths[startIndex];
      trajectoryPoints.trajectoryLength += npts;
      vfc::float32_t t = 0.0f;
      vfc::float32_t step_size{0.0f};

      if (startIndex == 0)
      {
         step_size = vfc::divide(L, static_cast<vfc::float32_t>(npts));
         npts += 1;
      }
      else if (startIndex == 1)
      {
         step_size = vfc::divide(L, static_cast<vfc::float32_t>(npts - 1));
      }
      else if (startIndex == 2)
      {
         step_size = vfc::divide(L, static_cast<vfc::float32_t>(npts - 2));
         npts -= 1;
      }
      for (uint32_t i{0}; i < npts; i++)
      {
         clothoidSubfunctions.generalizedFresnelCS(1, dkappa * vfc::sqr(t), kappa * t, theta0, C, S);
         X[i].value() = x0 + t * C[0U];
         Y[i].value() = y0 + t * S[0U];
         t += step_size;
      }

      vfc::uint16_t startInd = 0U;
      vfc::uint16_t endInd   = 0U;

      if (startIndex == 0U)
      {
         startInd = 0;
         endInd   = evalPoints.evalPointsLengths[0U];
      }
      else if (startIndex == 1U)
      {
         startInd = evalPoints.evalPointsLengths[0U];
         endInd   = evalPoints.evalPointsLengths[0U] + npts;
      }
      else if (startIndex == 2U)
      {
         startInd = evalPoints.evalPointsLengths[0U] + evalPoints.evalPointsLengths[1U];
         endInd   = evalPoints.evalPointsLengths[0U] + evalPoints.evalPointsLengths[1U] + npts + 1;
      }

      vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U> Ys{static_cast<vfc::CSI::si_metre_f32_t>(0.0F)};
      vfc::uint16_t                                sizeofXY  = npts;
      vfc::uint16_t                                sizeofXYs = 0;
      if (startIndex == 0U && evalPoints.evalPointsLengths[0] > 0)
      {
         sizeofXYs = evalPoints.evalPointsLengths[0];
         interpolation1D(X, Y, sizeofXY, evalPoints.evalPointsSegment1, Ys, sizeofXYs);
      }
      else if (startIndex == 1U && evalPoints.evalPointsLengths[1] > 0)
      {
         sizeofXYs = evalPoints.evalPointsLengths[1];
         interpolation1D(X, Y, sizeofXY, evalPoints.evalPointsSegment2, Ys, sizeofXYs);
      }
      else if (startIndex == 2U && evalPoints.evalPointsLengths[2] > 0)
      {
         sizeofXYs = evalPoints.evalPointsLengths[2];
         interpolation1D(X, Y, sizeofXY, evalPoints.evalPointsSegment3, Ys, sizeofXYs);
      }

      for (uint16_t i = startInd; i < endInd; i++)
      {
         // selecting X trajectory points
         if (startIndex == 0U)
         {
            trajectoryPoints.trajectoryPoints[i].PointsX = evalPoints.evalPointsSegment1[i];
         }
         else if (startIndex == 1U)
         {
            trajectoryPoints.trajectoryPoints[i].PointsX = evalPoints.evalPointsSegment2[(i - startInd)];
         }
         else if (startIndex == 2U)
         {
            trajectoryPoints.trajectoryPoints[i].PointsX = evalPoints.evalPointsSegment3[(i - startInd)];
         }
         // selecting Y trajectory points
         trajectoryPoints.trajectoryPoints[i].PointsY.value() = Ys[i - startInd].value();
      }
   }
}

void TrajectoryEvaluation::calc(
   const PolynomialCoeffs& polynomialCoeffs, const EvalPoints& evalPoints, TrajectoryPoints& trajectoryPoints)
{
   if (
      polynomialCoeffs.c0 == 0 && polynomialCoeffs.c1 == 0 && polynomialCoeffs.c2 == 0
      && polynomialCoeffs.c3 == 0)
   {
      memset(trajectoryPoints.trajectoryPoints, 0.0f, sizeof(trajectoryPoints.trajectoryPoints));
      trajectoryPoints.trajectoryLength = 0;
   }
   else
   {
      memset(trajectoryPoints.trajectoryPoints, 0.0f, sizeof(trajectoryPoints.trajectoryPoints));
      trajectoryPoints.trajectoryLength = 0;

      vfc::uint16_t npts =
         evalPoints.evalPointsLengths[0] + evalPoints.evalPointsLengths[1] + evalPoints.evalPointsLengths[2];
      trajectoryPoints.trajectoryLength = npts;

      for (uint16_t i{0U}; i < npts; i++)
      {
         if (i < evalPoints.evalPointsLengths[0U])
         {
            trajectoryPoints.trajectoryPoints[i].PointsX.value() = evalPoints.evalPointsSegment1[i].value();
            trajectoryPoints.trajectoryPoints[i].PointsY.value() =
               polynomialCoeffs.c0
               + polynomialCoeffs.c1 * vfc::pow(evalPoints.evalPointsSegment1[i].value(), 1)
               + polynomialCoeffs.c2 * vfc::pow(evalPoints.evalPointsSegment1[i].value(), 2)
               + polynomialCoeffs.c3 * vfc::pow(evalPoints.evalPointsSegment1[i].value(), 3);
         }
         else if (i < evalPoints.evalPointsLengths[0U] + evalPoints.evalPointsLengths[1U])
         {
            trajectoryPoints.trajectoryPoints[i].PointsX.value() =
               evalPoints.evalPointsSegment2[i - evalPoints.evalPointsLengths[0U]].value();
            trajectoryPoints.trajectoryPoints[i].PointsY.value() =
               polynomialCoeffs.c0
               + polynomialCoeffs.c1
                    * vfc::pow(evalPoints.evalPointsSegment2[i - evalPoints.evalPointsLengths[0U]].value(), 1)
               + polynomialCoeffs.c2
                    * vfc::pow(evalPoints.evalPointsSegment2[i - evalPoints.evalPointsLengths[0U]].value(), 2)
               + polynomialCoeffs.c3
                    * vfc::
                         pow(evalPoints.evalPointsSegment2[i - evalPoints.evalPointsLengths[0U]].value(), 3);
         }
         else if (
            i < evalPoints.evalPointsLengths[0U] + evalPoints.evalPointsLengths[1U]
                   + evalPoints.evalPointsLengths[2U])
         {
            trajectoryPoints.trajectoryPoints[i].PointsX.value() =
               evalPoints
                  .evalPointsSegment3[i - evalPoints.evalPointsLengths[0U] - evalPoints.evalPointsLengths[1U]]
                  .value();
            trajectoryPoints.trajectoryPoints[i].PointsY.value() =
               polynomialCoeffs.c0
               + polynomialCoeffs.c1
                    * vfc::pow(
                         evalPoints
                            .evalPointsSegment3
                               [i - evalPoints.evalPointsLengths[0U] - evalPoints.evalPointsLengths[1U]]
                            .value(),
                         1)
               + polynomialCoeffs.c2
                    * vfc::pow(
                         evalPoints
                            .evalPointsSegment3
                               [i - evalPoints.evalPointsLengths[0U] - evalPoints.evalPointsLengths[1U]]
                            .value(),
                         2)
               + polynomialCoeffs.c3
                    * vfc::pow(
                         evalPoints
                            .evalPointsSegment3
                               [i - evalPoints.evalPointsLengths[0U] - evalPoints.evalPointsLengths[1U]]
                            .value(),
                         3);
         }
      }
   }
}

void TrajectoryEvaluation::interpolation1D(
   const vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U>& X,
   const vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U>& Y,
   const vfc::uint16_t                                 sizeOfXY,
   const vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U>& Xs,
   vfc::TCArray<vfc::CSI::si_metre_f32_t, 300U>&       Ys,
   const vfc::uint16_t                                 sizeOfXYs)
{
   vfc::float32_t x0{0.0f};
   vfc::float32_t y0{0.0f};
   vfc::float32_t x1{0.0f};
   vfc::float32_t y1{0.0f};

   for (uint16_t i{0}; i < sizeOfXYs; i++)
   {
      if (Xs[i] < X[0])
      {
         Ys[i] = Y[0];
      }
      else if (Xs[i] > X[sizeOfXY - 1])
      {
         Ys[i] = Y[sizeOfXY - 1];
      }
      else
      {
         for (uint16_t j{1}; j < sizeOfXY; j++)
         {
            if (Xs[i] < X[j])
            {
               x0 = X[j - 1].value();
               y0 = Y[j - 1].value();
               x1 = X[j].value();
               y1 = Y[j].value();
               break;
            }
         }
         if (vfc::abs(x1 - x0) < vfc::float32_t{1e-3F})
         {
            Ys[i].value() = y0;
         }
         else
         {
            Ys[i].value() = y0 + (y1 - y0) / (x1 - x0) * (Xs[i].value() - x0);
         }
      }
   }
}

}  // namespace Emg
}  // namespace Dc

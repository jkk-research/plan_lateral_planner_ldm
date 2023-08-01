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


namespace Rb
{
namespace Vmc
{

void SegmentPlanner::buildClothoid(
   const NodePoints& nodePoints, SegmentParams& segmentParams, vfc::uint8_t startIndex)
{
   vfc::float32_t dx = nodePoints.nodePointsCoordinates[startIndex + 1U].PointsX.value()
                       - nodePoints.nodePointsCoordinates[startIndex].PointsX.value();
   vfc::float32_t dy = nodePoints.nodePointsCoordinates[startIndex + 1U].PointsY.value()
                       - nodePoints.nodePointsCoordinates[startIndex].PointsY.value();
   vfc::float32_t r   = vfc::sqrt(vfc::sqr(dx) + vfc::sqr(dy));
   vfc::float32_t phi = vfc::atan2<vfc::TRadian, vfc::float32_t>(dy, dx).value();

   vfc::float32_t phi0 =
      clothoidSubfunctions.normalizeAngle(nodePoints.nodePointsTheta[startIndex].value() - phi);
   vfc::float32_t phi1 =
      clothoidSubfunctions.normalizeAngle(nodePoints.nodePointsTheta[startIndex + 1U].value() - phi);
   vfc::float32_t delta = phi1 - phi0;

   vfc::float32_t Aguess = clothoidSubfunctions.guessA(phi0, phi1);
   vfc::float32_t A      = clothoidSubfunctions.findA(Aguess, delta, phi0, 1e-12f);

   clothoidSubfunctions
      .generalizedFresnelCS(1U, 2U * A, delta - A, phi0, clothoidSubfunctions.X, clothoidSubfunctions.Y);
   clothoidSubfunctions.h = clothoidSubfunctions.X[0U];
   clothoidSubfunctions.g = clothoidSubfunctions.Y[0U];

   if (clothoidSubfunctions.h == 0)
   {
      clothoidSubfunctions.h = 0.0000002f;
   }

   segmentParams.L[startIndex].value()  = vfc::divide(r, clothoidSubfunctions.h);
   if (segmentParams.L[startIndex].value() == 0.0f)
   {
      segmentParams.k[startIndex].value()  = 0.0f;
      segmentParams.dk[startIndex].value() = 0.0f;
   }
   else
   {
      segmentParams.k[startIndex].value() = vfc::divide(delta - A, segmentParams.L[startIndex].value());
      segmentParams.dk[startIndex].value() =
         vfc::divide(2 * A, vfc::sqr(segmentParams.L[startIndex].value()));
   }
   segmentParams.initPose[startIndex].PointsX = nodePoints.nodePointsCoordinates[startIndex].PointsX;
   segmentParams.initPose[startIndex].PointsY = nodePoints.nodePointsCoordinates[startIndex].PointsY;
   segmentParams.initPoseTheta[startIndex]    = nodePoints.nodePointsTheta[startIndex];
}

void SegmentPlanner::buildPolynomial(
   const NodePoints& nodePoints, PolynomialCoeffs& polynomialCoeffs, SegmentParams& segmentParams)
{
   // calculating coefficients for segmentParams
   for (uint8_t k{0U}; k < 3U; k++)
   {
      for (uint8_t i{0U}; i < 4U; i++)
      {
         for (uint8_t j{0U}; j < 5U; j++)
         {
            if (j == 0 && (i % 2 == 0))
            {
               gaussMatrix[i][j] = 1.0f;
            }
            else if (j == 0 && (i % 2 == 1))
            {
               gaussMatrix[i][j] = 0.0f;
            }
            if (i == 0U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = vfc::
                  pow(nodePoints.nodePointsCoordinates[k].PointsX.value(), static_cast<vfc::float32_t>(j));
            }
            else if (i == 0U && j == 4U)
            {
               gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[k].PointsY.value();
            }
            else if (i == 1U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = j
                                   * vfc::pow(
                                        nodePoints.nodePointsCoordinates[k].PointsX.value(),
                                        static_cast<vfc::float32_t>(j - 1));
            }
            else if (i == 1U && j == 4U)
            {
               gaussMatrix[i][j] =
                  vfc::tan(static_cast<vfc::CRadian32>(nodePoints.nodePointsTheta[k].value()));
            }
            else if (i == 2U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = vfc::pow(
                  nodePoints.nodePointsCoordinates[k + 1].PointsX.value(), static_cast<vfc::float32_t>(j));
            }
            else if (i == 2U && j == 4U)
            {
               gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[k + 1].PointsY.value();
            }
            else if (i == 3U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = j
                                   * vfc::pow(
                                        nodePoints.nodePointsCoordinates[k + 1].PointsX.value(),
                                        static_cast<vfc::float32_t>(j - 1));
            }
            else if (i == 3U && j == 4U)
            {
               gaussMatrix[i][j] =
                  vfc::tan(static_cast<vfc::CRadian32>(nodePoints.nodePointsTheta[k + 1].value()));
            }
         }
      }
      polynomialSubfunctions.gaussElimination(gaussMatrix, polynomialCoeffs);

      segmentParams.initPose[k].PointsX = nodePoints.nodePointsCoordinates[k].PointsX;
      segmentParams.initPose[k].PointsY = nodePoints.nodePointsCoordinates[k].PointsY;
      segmentParams.initPoseTheta[k]    = nodePoints.nodePointsTheta[k];
      segmentParams.k[k].value()        = polynomialCoeffs.c2 * 2.0f;
      segmentParams.dk[k].value()       = polynomialCoeffs.c3 * 6.0f;
      segmentParams.L[k].value()        = 0.0f;
   }

   // calculating coefficients for trajectory evaluation
   for (uint8_t i{0U}; i < 4U; i++)
   {
      for (uint8_t j{0U}; j < 5U; j++)
      {
         if (j == 0)
         {
            gaussMatrix[i][j] = 1.0f;
         }
         if (i == 0U && (0U < j && j < 4U))
         {
            gaussMatrix[i][j] =
               vfc::pow(nodePoints.nodePointsCoordinates[0].PointsX.value(), static_cast<vfc::float32_t>(j));
         }
         else if (i == 0U && j == 4U)
         {
            gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[0].PointsY.value();
         }
         else if (i == 1U && (0U < j && j < 4U))
         {
            gaussMatrix[i][j] =
               vfc::pow(nodePoints.nodePointsCoordinates[1].PointsX.value(), static_cast<vfc::float32_t>(j));
         }
         else if (i == 1U && j == 4U)
         {
            gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[1].PointsY.value();
         }
         else if (i == 2U && (0U < j && j < 4U))
         {
            gaussMatrix[i][j] =
               vfc::pow(nodePoints.nodePointsCoordinates[2].PointsX.value(), static_cast<vfc::float32_t>(j));
         }
         else if (i == 2U && j == 4U)
         {
            gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[2].PointsY.value();
         }
         else if (i == 3U && (0U < j && j < 4U))
         {
            gaussMatrix[i][j] =
               vfc::pow(nodePoints.nodePointsCoordinates[3].PointsX.value(), static_cast<vfc::float32_t>(j));
         }
         else if (i == 3U && j == 4U)
         {
            gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[3].PointsY.value();
         }
      }
   }
   polynomialSubfunctions.gaussElimination(gaussMatrix, polynomialCoeffs);
}

void SegmentPlanner::buildThreeSegmentPolynomial(
   const NodePoints&              nodePoints,
   PolynomialCoeffsThreeSegments& polynomialCoeffsThreeSegments,
   SegmentParams&                 segmentParams)
{
   // calculating coefficients for segmentParams
   for (uint8_t k{0U}; k < 3U; k++)
   {
      for (uint8_t i{0U}; i < 4U; i++)
      {
         for (uint8_t j{0U}; j < 5U; j++)
         {
            if (j == 0 && (i % 2 == 0))
            {
               gaussMatrix[i][j] = 1.0f;
            }
            else if (j == 0 && (i % 2 == 1))
            {
               gaussMatrix[i][j] = 0.0f;
            }
            if (i == 0U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = vfc::
                  pow(nodePoints.nodePointsCoordinates[k].PointsX.value(), static_cast<vfc::float32_t>(j));
            }
            else if (i == 0U && j == 4U)
            {
               gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[k].PointsY.value();
            }
            else if (i == 1U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = j
                                   * vfc::pow(
                                        nodePoints.nodePointsCoordinates[k].PointsX.value(),
                                        static_cast<vfc::float32_t>(j - 1));
            }
            else if (i == 1U && j == 4U)
            {
               gaussMatrix[i][j] =
                  vfc::tan(static_cast<vfc::CRadian32>(nodePoints.nodePointsTheta[k].value()));
            }
            else if (i == 2U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = vfc::pow(
                  nodePoints.nodePointsCoordinates[k + 1].PointsX.value(), static_cast<vfc::float32_t>(j));
            }
            else if (i == 2U && j == 4U)
            {
               gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[k + 1].PointsY.value();
            }
            else if (i == 3U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = j
                                   * vfc::pow(
                                        nodePoints.nodePointsCoordinates[k + 1].PointsX.value(),
                                        static_cast<vfc::float32_t>(j - 1));
            }
            else if (i == 3U && j == 4U)
            {
               gaussMatrix[i][j] =
                  vfc::tan(static_cast<vfc::CRadian32>(nodePoints.nodePointsTheta[k + 1].value()));
            }
         }
      }
      polynomialSubfunctions.gaussElimination(gaussMatrix, polynomialCoeffsThreeSegments.segmentCoeffs[k]);

      segmentParams.initPose[k].PointsX = nodePoints.nodePointsCoordinates[k].PointsX;
      segmentParams.initPose[k].PointsY = nodePoints.nodePointsCoordinates[k].PointsY;
      segmentParams.initPoseTheta[k]    = nodePoints.nodePointsTheta[k];
      segmentParams.k[k].value()        = polynomialCoeffsThreeSegments.segmentCoeffs[k].c2 * 2.0f;
      segmentParams.dk[k].value()       = polynomialCoeffsThreeSegments.segmentCoeffs[k].c3 * 6.0f;
      segmentParams.L[k].value()        = 0.0f;
   }
}

}  // namespace Emg
}  // namespace Dc

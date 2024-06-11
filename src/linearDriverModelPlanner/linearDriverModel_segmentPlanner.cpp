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

#include "../../inc/linearDriverModelPlanner/linearDriverModel_segmentPlanner.hpp"


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
               gaussMatrix[i][j] = 
                  pow(nodePoints.nodePointsCoordinates[k].x, (float)j);
            }
            else if (i == 0U && j == 4U)
            {
               gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[k].y;
            }
            else if (i == 1U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = j * pow(
                  nodePoints.nodePointsCoordinates[k].x,
                  (float)(j - 1));
            }
            else if (i == 1U && j == 4U)
            {
               gaussMatrix[i][j] =
                  tan(nodePoints.nodePointsTheta[k]);
            }
            else if (i == 2U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = pow(
                  nodePoints.nodePointsCoordinates[k + 1].x, (float)j);
            }
            else if (i == 2U && j == 4U)
            {
               gaussMatrix[i][j] = nodePoints.nodePointsCoordinates[k + 1].y;
            }
            else if (i == 3U && (0U < j && j < 4U))
            {
               gaussMatrix[i][j] = j
                                   * pow(
                                        nodePoints.nodePointsCoordinates[k + 1].x,
                                        (float)(j - 1));
            }
            else if (i == 3U && j == 4U)
            {
               gaussMatrix[i][j] =
                  tan(nodePoints.nodePointsTheta[k + 1]);
            }
         }
      }
      polynomialSubfunctions.gaussElimination(gaussMatrix, polynomialCoeffsThreeSegments.segmentCoeffs[k]);

      segmentParams.initPose[k].x    = nodePoints.nodePointsCoordinates[k].x;
      segmentParams.initPose[k].y    = nodePoints.nodePointsCoordinates[k].y;
      segmentParams.initPoseTheta[k] = nodePoints.nodePointsTheta[k];
      segmentParams.k[k]             = polynomialCoeffsThreeSegments.segmentCoeffs[k].c2 * 2.0f;
      segmentParams.dk[k]            = polynomialCoeffsThreeSegments.segmentCoeffs[k].c3 * 6.0f;
      segmentParams.L[k]             = 0.0f;
   }
}

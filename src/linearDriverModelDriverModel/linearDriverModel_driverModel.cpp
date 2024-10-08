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

#include "../../inc/linearDriverModel/linearDriverModel.hpp"

void DriverModel::driverModelPlannerLite(
   const ScenarioPolynomials&             corridorPolynomials,
   const PolynomialCoeffsThreeSegments&   trajectoryCoeffsThreeSegments,
   const LDMParamIn&                      parameters,
   NodePoints&                            nodePoints)
{
   // producing nominal values
   for (uint8_t i{0U}; i < 3; i++)
   {
      m_XNominal[i] = parameters.P_nodePointDistances[i];
      m_YNominal[i] = corridorPolynomials.coeffs[i].c0 + corridorPolynomials.coeffs[i].c1 * parameters.P_nodePointDistances[i]
                  + corridorPolynomials.coeffs[i].c2 * pow(parameters.P_nodePointDistances[i], 2.0f)
                  + corridorPolynomials.coeffs[i].c3 * pow(parameters.P_nodePointDistances[i], 3.0f);
      m_thetaNominal[i] = atan(
                           corridorPolynomials.coeffs[i].c1 + 2 * corridorPolynomials.coeffs[i].c2 * parameters.P_nodePointDistances[i]
                           + 3 * corridorPolynomials.coeffs[i].c3 * pow(parameters.P_nodePointDistances[i], 2.0f));
   }
   
   // calculating the offsets
   offsetCalcExtendedLDM(parameters, corridorPolynomials.kappaNominal);

   // intializing with further most segment due to extrapolation
   if (trajectoryCoeffsThreeSegments.sectionBorderEnd[2] < 0.0f)
   {
      m_validCoefficients = trajectoryCoeffsThreeSegments.segmentCoeffs[2];
   }
   else
   {
      m_validCoefficients = trajectoryCoeffsThreeSegments.segmentCoeffs[0];
      for (uint8_t i{0U}; i < 3; i++)
      {
         if (
            trajectoryCoeffsThreeSegments.sectionBorderStart[i] <= 0.0f
            && trajectoryCoeffsThreeSegments.sectionBorderEnd[i] > 0.0f)
         {
            m_validCoefficients = trajectoryCoeffsThreeSegments.segmentCoeffs[i];
            break;
         }
      }
   }

   nodePoints.nodePointsCoordinates[0U].x = 0.0f;
   nodePoints.nodePointsCoordinates[0U].y = m_validCoefficients.c0;
   nodePoints.nodePointsTheta[0U] = atan(m_validCoefficients.c1);
   for (uint8_t i{0U}; i < 3; i++)
   {
      nodePoints.nodePointsCoordinates[i + 1U].x =
         m_XNominal[i] - sin(m_thetaNominal[i]) * m_x[i];
      nodePoints.nodePointsCoordinates[i + 1U].y =
         m_YNominal[i] + cos(m_thetaNominal[i]) * m_x[i];
      nodePoints.nodePointsTheta[i + 1U] = m_thetaNominal[i];
   }
}

void DriverModel::offsetCalcExtendedLDM(const LDMParamIn& parameters, const std::vector<float> kappaNominal)
{
   for (uint8_t i{0U}; i <= 6U; i++)
   {
      if (i < 3U)
      {
         m_U[i] = std::max(kappaNominal[i], 0.0f);
      }
      else if (i < 6U)
      {
         m_U[i] = std::min(kappaNominal[i - 3], 0.0f);
      }
      else
      {
         m_U[i] = 1.0f;
      }
   }

   float U_lim[7]{1e-3f, 1e-3f, 1e-3f, 1e-3f, 1e-3f, 1e-3f, 1.0f};

   for (uint8_t i{0U}; i <= 6U; i++)
   {
      m_U[i] = m_U[i] / U_lim[i];
   }

   memset(m_x, 0.0f, sizeof(m_x));

   for (uint8_t i{0U}; i <= 20U; i++)
   {
      if (i <= 6U)
      {
         m_x[0U] += m_U[i] * parameters.P[i];
      }
      else if (i <= 13U)
      {
         m_x[1] += m_U[i - 7] * parameters.P[i];
      }
      else
      {
         m_x[2] += m_U[i - 14] * parameters.P[i];
      }
   }

   // limiting offset value: x
   for (uint8_t i{0U}; i < 3U; i++)
   {
      m_x[i] = std::min(std::max(-1.25f, m_x[i]), 1.25f);
   }
}

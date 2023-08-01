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

void DriverModel::driverModelPlannerLite(
   const CorridorInfoCoefficients&      corridorCoefficients,
   const PolynomialCoeffsThreeSegments& trajectoryCoeffsThreeSegments,
   const LDMParamIn&                    parameters,
   NodePoints&                          nodePoints)
{

   // producing nominal values
   vfc::float32_t scaledNodePointDist[3]{0.0f};
   for (uint8_t i{0U}; i < 3; i++)
   {
      if (i == 0)
      {
         scaledNodePointDist[i] = vfc::max(10.0f, parameters.P_nodePointDistances[i] * 250.0f);
      }
      else
      {
         scaledNodePointDist[i] =
            vfc::max(10.0f, parameters.P_nodePointDistances[i] * 250.0f + scaledNodePointDist[i - 1U]);
      }
      XNominal[i] = scaledNodePointDist[i];
      YNominal[i] = corridorCoefficients.c0 + corridorCoefficients.c1 * scaledNodePointDist[i]
                    + corridorCoefficients.c2 * vfc::pow(scaledNodePointDist[i], 2.0f)
                    + corridorCoefficients.c3 * vfc::pow(scaledNodePointDist[i], 3.0f);
      thetaNominal[i] = vfc::atan<vfc::TRadian, vfc::float32_t>(
                           corridorCoefficients.c1 + 2 * corridorCoefficients.c2 * scaledNodePointDist[i]
                           + 3 * corridorCoefficients.c3 * vfc::pow(scaledNodePointDist[i], 2.0f))
                           .value();
   }

   for (uint8_t i{0U}; i < 3; i++)
   {
      if (i == 0)
      {
         kappaNominal[i] =
            2 * corridorCoefficients.c2 + 3 * corridorCoefficients.c3 * scaledNodePointDist[i];
      }
      else
      {
         kappaNominal[i] =
            2 * corridorCoefficients.c2
            + 3 * corridorCoefficients.c3
                 * (scaledNodePointDist[i] + scaledNodePointDist[i - 1U]);
      }
   }

   // calculating the offsets
   offsetCalcExtendedLDM(parameters, kappaNominal);

   // intializing with further most segment due to extrapolation
   validCoefficients = trajectoryCoeffsThreeSegments.segmentCoeffs[2];
   
   for (uint8_t i{0U}; i < 3; i++)
   {
      if (
         trajectoryCoeffsThreeSegments.sectionBorderStart[i] <= 0.0f
         && trajectoryCoeffsThreeSegments.sectionBorderEnd[i] > 0.0f)
      {
         validCoefficients = trajectoryCoeffsThreeSegments.segmentCoeffs[i];
         break;
      }
   }

   nodePoints.nodePointsCoordinates[0U].PointsX.value() = 0.0f;
   nodePoints.nodePointsCoordinates[0U].PointsY.value() = validCoefficients.c0;
   nodePoints.nodePointsTheta[0U].value() =
      vfc::atan<vfc::TRadian, vfc::float32_t>(validCoefficients.c1).value();
   for (uint8_t i{0U}; i < 3; i++)
   {
      nodePoints.nodePointsCoordinates[i + 1U].PointsX.value() =
         XNominal[i] - vfc::sin(static_cast<vfc::CRadian32>(thetaNominal[i])) * x[i];
      nodePoints.nodePointsCoordinates[i + 1U].PointsY.value() =
         YNominal[i] + vfc::cos(static_cast<vfc::CRadian32>(thetaNominal[i])) * x[i];
      nodePoints.nodePointsTheta[i + 1U].value() = thetaNominal[i];
   }
}

void DriverModel::offsetCalcLDM(
   const LDMParamIn& parameters, const vfc::float32_t* kappaNominal, const vfc::float32_t* dkappaNominal)
{
   for (uint8_t i{0U}; i <= 6U; i++)
   {
      if (i < 3U)
      {
         U[i] = kappaNominal[i];
      }
      else
      {
         U[i] = dkappaNominal[i - 3U];
      }
   }

   vfc::float32_t U_lim[7]{1e-3f, 1e-3f, 1e-3f, 1e-5f, 1e-5f, 1e-5f, 1e-5f};

   for (uint8_t i{0U}; i <= 6U; i++)
   {
      U[i] = vfc::divide(U[i], U_lim[i]);
   }

   memset(x, 0.0f, sizeof(x));

   for (uint8_t i{0U}; i <= 20U; i++)
   {
      if (i <= 6U)
      {
         x[0U] += U[i] * parameters.P[i];
      }
      else if (i <= 13U)
      {
         x[1] += U[i - 7] * parameters.P[i];
      }
      else
      {
         x[2] += U[i - 14] * parameters.P[i];
      }
   }

   // limiting offset value: x
   for (uint8_t i{0U}; i < 3U; i++)
   {
      x[i] = vfc::min(vfc::max(-1.25f, x[i]), 1.25f);
   }
}

void DriverModel::offsetCalcExtendedLDM(const LDMParamIn& parameters, const vfc::float32_t* kappaNominal)
{
   for (uint8_t i{0U}; i <= 6U; i++)
   {
      if (i < 3U)
      {
         U[i] = vfc::max(kappaNominal[i], 0.0f);
      }
      else if (i < 6U)
      {
         U[i] = vfc::min(kappaNominal[i - 3], 0.0f);
      }
      else
      {
         U[i] = 1.0f;
      }
   }

   vfc::float32_t U_lim[7]{1e-3f, 1e-3f, 1e-3f, 1e-3f, 1e-3f, 1e-3f, 1.0f};

   for (uint8_t i{0U}; i <= 6U; i++)
   {
      U[i] = vfc::divide(U[i], U_lim[i]);
   }

   memset(x, 0.0f, sizeof(x));

   for (uint8_t i{0U}; i <= 20U; i++)
   {
      if (i <= 6U)
      {
         x[0U] += U[i] * parameters.P[i];
      }
      else if (i <= 13U)
      {
         x[1] += U[i - 7] * parameters.P[i];
      }
      else
      {
         x[2] += U[i - 14] * parameters.P[i];
      }
   }

   // limiting offset value: x
   for (uint8_t i{0U}; i < 3U; i++)
   {
      x[i] = vfc::min(vfc::max(-1.25f, x[i]), 1.25f);
   }
}

}  // namespace Emg
}  // namespace Dc

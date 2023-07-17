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

void DriverModel::calc(
   const CorridorInfo& corridorPlannerFrame, const LDMParamIn& parameters, NodePoints& nodePoints)
{
   nodePointModel(corridorPlannerFrame, parameters);
   driverModelPlanner(corridorPlannerFrame, parameters, nodePoints);
}

void DriverModel::nodePointModel(const CorridorInfo& corridorPlanner, const LDMParamIn& parameters)
{
   vfc::float32_t L[4]{0.0f};
   vfc::float32_t lengths[300]{0.0f};
   vfc::float32_t minStepDist = 10.0f;
   vfc::uint16_t  n           = corridorPlanner.corridorLength - 1U;

   if (n >= 0)
   {
      for (uint16_t i{0}; i <= n; i++)
      {
         lengths[i] = vfc::sqrt(
            vfc::sqr(corridorPlanner.corridorPoints[i].PointsX.value())
            + vfc::sqr(corridorPlanner.corridorPoints[i].PointsY.value()));
      }
      indices[0] = 0U;
      if (lengths[n] > (3 * minStepDist))
      {
         L[1] = vfc::min(
            lengths[n] - 3 * minStepDist, vfc::max(minStepDist, parameters.P_nodePointDistances[0] * 250.0f));
         L[2] = vfc::min(
            lengths[n] - 2 * minStepDist,
            vfc::max(L[1] + minStepDist, L[1] + parameters.P_nodePointDistances[1] * 250.0f));
         L[3] = vfc::min(
            lengths[n] - minStepDist,
            vfc::max(L[2] + minStepDist, L[2] + parameters.P_nodePointDistances[2] * 250.0f));

         vfc::uint16_t index1 = 0U;
         vfc::uint16_t index2 = 0U;
         vfc::uint16_t index3 = 0U;

         while (lengths[index1] <= L[1] && index1 < 300U)
         {
            index1++;
         }
         indices[1] = index1;

         while (lengths[index2] <= L[2] && index2 < 300U)
         {
            index2++;
         }
         indices[2] = index2;

         while (lengths[index3] <= L[3] && index3 < 300U)
         {
            index3++;
         }
         indices[3] = vfc::min(n, index3);
      }
      else
      {
         indices[1] = 0U;
         indices[2] = 0U;
         indices[3] = 0U;
      }
   }
   else
   {
      indices[1] = 0U;
      indices[2] = 0U;
      indices[3] = 0U;
   }
}

void DriverModel::driverModelPlanner(
   const CorridorInfo& corridorPlanner, const LDMParamIn& parameters, NodePoints& nodePoints)
{
   vfc::float32_t XNominal[4]{0.0f};
   vfc::float32_t YNominal[4]{0.0f};
   vfc::float32_t thetaNominal[4]{0.0f};
   vfc::float32_t kappaNominal[3]{0.0f};
   vfc::float32_t dkappaNominal[4]{0.0f};
   vfc::uint16_t  n = corridorPlanner.corridorLength;

   bool invalidIndices{false};

   for (uint8_t i{1}; i < 4; i++)
   {
      if (indices[i] <= indices[i - 1])
      {
         invalidIndices = true;
         break;
      }
   }

   if (invalidIndices)
   {
      for (uint8_t i{0}; i <= 3; i++)
      {
         nodePoints.nodePointsCoordinates[i].PointsY.value() = 0.0f;
         nodePoints.nodePointsCoordinates[i].PointsX.value() = 0.0f;
         nodePoints.nodePointsTheta[i].value()               = 0.0f;
      }
   }
   else
   {
      for (uint8_t i{0}; i <= 3; i++)
      {
         XNominal[i]     = corridorPlanner.corridorPoints[indices[i]].PointsX.value();
         YNominal[i]     = corridorPlanner.corridorPoints[indices[i]].PointsY.value();
         thetaNominal[i] = corridorPlanner.corridorOrientation[indices[i]].value();
      }

      vfc::uint16_t  numberOfElements = 0U;
      vfc::float32_t sum              = 0.0f;
      vfc::float32_t mean_kappa       = 0.0f;
      for (uint8_t i{0}; i <= 2; i++)
      {
         numberOfElements = indices[i + 1] - indices[i] + 1;
         sum              = 0.0f;

         for (uint16_t j{0}; j < numberOfElements; j++)
         {
            sum += corridorPlanner.corridorCurvature[indices[i] + j].value();
         }
         mean_kappa      = vfc::divide(sum, static_cast<vfc::float32_t>(numberOfElements));
         kappaNominal[i] = mean_kappa;
      }

      for (uint8_t i{0U}; i <= 3U; i++)
      {
         if (i == 0)
         {
            dkappaNominal[i] = vfc::divide(
               corridorPlanner.corridorCurvature[indices[i] + 1].value()
                  - corridorPlanner.corridorCurvature[indices[i]].value(),
               corridorPlanner.corridorPoints[indices[i] + 1].PointsX.value()
                  - corridorPlanner.corridorPoints[indices[i]].PointsX.value());
         }
         else
         {
            dkappaNominal[i] = vfc::divide(
               corridorPlanner.corridorCurvature[indices[i]].value()
                  - corridorPlanner.corridorCurvature[indices[i] - 1].value(),
               corridorPlanner.corridorPoints[indices[i]].PointsX.value()
                  - corridorPlanner.corridorPoints[indices[i] - 1].PointsX.value());
         }
      }

      for (uint8_t i{0U}; i <= 3U; i++)
      {
         nominal[i]     = XNominal[i];
         nominal[i + 4] = YNominal[i];
         nominal[i + 8] = thetaNominal[i];
      }

      // offsetCalcLDM(parameters, kappaNominal, dkappaNominal);
      offsetCalcExtendedLDM(parameters, kappaNominal);

      for (uint8_t i{0}; i <= 3U; i++)
      {
         if (i == 0U)
         {
            nodePoints.nodePointsCoordinates[i].PointsY.value() = 0.0f;
            nodePoints.nodePointsCoordinates[i].PointsX.value() = 0.0f;
            nodePoints.nodePointsTheta[i].value()               = 0.0f;
         }
         else
         {
            nodePoints.nodePointsCoordinates[i].PointsY.value() = YNominal[i] + x[i - 1];
            nodePoints.nodePointsCoordinates[i].PointsX.value() = XNominal[i];
            nodePoints.nodePointsTheta[i].value()               = thetaNominal[i];
         }
      }
   }
}

void DriverModel::driverModelPlannerLite(
   const CorridorInfoCoefficients&      corridorCoefficients,
   const PolynomialCoeffsThreeSegments& trajectoryCoeffsThreeSegments,
   const LDMParamIn&                    parameters,
   NodePoints&                          nodePoints)
{
   vfc::float32_t   XNominal[3]{0.0f};
   vfc::float32_t   YNominal[3]{0.0f};
   vfc::float32_t   thetaNominal[3]{0.0f};
   vfc::float32_t   kappaNominal[3]{0.0f};
   PolynomialCoeffs validCoefficients{};

   for (uint8_t i{0U}; i < 3; i++)
   {
      if (i == 0)
      {
         kappaNominal[i] =
            2 * corridorCoefficients.c2 + 3 * corridorCoefficients.c3 * parameters.P_nodePointDistances[i];
      }
      else
      {
         kappaNominal[i] =
            2 * corridorCoefficients.c2
            + 3 * corridorCoefficients.c3
                 * (parameters.P_nodePointDistances[i] + parameters.P_nodePointDistances[i - 1U]);
      }
   }
   // calculating the offsets
   offsetCalcExtendedLDM(parameters, kappaNominal);

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

   if (trajectoryCoeffsThreeSegments.sectionBorderEnd[3] != 0.0f)
   {
      validCoefficients = trajectoryCoeffsThreeSegments.segmentCoeffs[1];
      for (uint8_t i{0U}; i < 2; i++)
      {
         if (
            trajectoryCoeffsThreeSegments.sectionBorderStart[i] <= 0.0f
            && trajectoryCoeffsThreeSegments.sectionBorderEnd[i] > 0.0f)
         {
            validCoefficients = trajectoryCoeffsThreeSegments.segmentCoeffs[i];
         }
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

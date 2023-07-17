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

#include "emg_linearDriverModel.hpp"

namespace Dc
{
namespace Emg
{

void ControlLogic::corridorValidityCalc(
   const CorridorInfo& corridorEgo, const CorridorInfo& corridorGlobal, const Pose2D& egoPoseGlobal)
{
   vfc::float32_t distances[300]{0.0f};
   vfc::float32_t distanceFirstPoint{0.0f};
   vfc::float32_t diffPoints{0.0f};
   vfc::float32_t diffCorridorEgo{0.0f};
   vfc::float32_t movMeanX{0.0f};
   vfc::float32_t movMeanY{0.0f};
   vfc::float32_t movMeanTheta{0.0f};
   vfc::float32_t previousMovMeanX{0.0f};
   vfc::float32_t previousMovMeanY{0.0f};
   vfc::float32_t previousMovMeanTheta{0.0f};
   bool           invalidPoint{false};
   bool           monotonIncreasing{true};
   bool           invalidEgoPose{false};
   validity = true;

   // checking GPS data validity
   // uploading the gps data array for further moving average calculation
   // (later consider using vfc buffer type)
   if (egoPoseDatasX[3] == 0.0f && egoPoseDatasY[3] == 0.0f && egoPoseDatasTheta[3] == 0.0f)
   {
      for (uint8_t i{0}; i < 4; i++)
      {
         if (egoPoseDatasX[i] == 0.0f)
         {
            egoPoseDatasX[i]     = egoPoseGlobal.Pose2DCoordinates.PointsX.value();
            egoPoseDatasY[i]     = egoPoseGlobal.Pose2DCoordinates.PointsY.value();
            egoPoseDatasTheta[i] = egoPoseGlobal.Pose2DTheta.value();
         }
      }
   }
   // if the array is uploaded, perform the calculations
   else
   {
      previousMovMeanX     = 0.0f;
      previousMovMeanY     = 0.0f;
      previousMovMeanTheta = 0.0f;
      for (uint8_t i{0}; i < 4; i++)
      {
         previousMovMeanX += egoPoseDatasX[i];
         previousMovMeanY += egoPoseDatasY[i];
         previousMovMeanTheta += egoPoseDatasTheta[i];

         if (i != 3)
         {
            egoPoseDatasX[i]     = egoPoseDatasX[i + 1];
            egoPoseDatasY[i]     = egoPoseDatasY[i + 1];
            egoPoseDatasTheta[i] = egoPoseDatasTheta[i + 1];
         }
         else
         {
            egoPoseDatasX[i]     = egoPoseGlobal.Pose2DCoordinates.PointsX.value();
            egoPoseDatasY[i]     = egoPoseGlobal.Pose2DCoordinates.PointsY.value();
            egoPoseDatasTheta[i] = egoPoseGlobal.Pose2DTheta.value();
         }
      }
      previousMovMeanX     = vfc::divide(previousMovMeanX, 4.0f);
      previousMovMeanY     = vfc::divide(previousMovMeanY, 4.0f);
      previousMovMeanTheta = vfc::divide(previousMovMeanTheta, 4.0f);
      for (uint8_t i{0}; i < 4; i++)
      {
         movMeanX += egoPoseDatasX[i];
         movMeanY += egoPoseDatasY[i];
         movMeanTheta += egoPoseDatasTheta[i];
      }
      movMeanX     = vfc::divide(movMeanX, 4.0f);
      movMeanY     = vfc::divide(movMeanY, 4.0f);
      movMeanTheta = vfc::divide(movMeanTheta, 4.0f);

      if (
         vfc::abs(movMeanX - previousMovMeanX) > 4 || vfc::abs(movMeanY - previousMovMeanY) > 4
         || vfc::abs(movMeanTheta - previousMovMeanTheta) > 0.3)
      {
         validity = false;
         return;
      }
   }

   // checking corridor validity
   distanceFirstPoint = vfc::sqrt(
      vfc::sqr(
         corridorGlobal.corridorPoints[0].PointsX.value() - egoPoseGlobal.Pose2DCoordinates.PointsX.value())
      + vfc::sqr(
           corridorGlobal.corridorPoints[0].PointsY.value()
           - egoPoseGlobal.Pose2DCoordinates.PointsY.value()));
   if (distanceFirstPoint > 5.0f || corridorEgo.corridorLength < 2U)
   {
      validity = false;
      return;
   }

   for (uint16_t i{0U}; i < corridorEgo.corridorLength; i++)
   {
      distances[i] = vfc::sqrt(
         vfc::sqr(
            corridorGlobal.corridorPoints[i].PointsX.value()
            - egoPoseGlobal.Pose2DCoordinates.PointsX.value())
         + vfc::sqr(
              corridorGlobal.corridorPoints[i].PointsY.value()
              - egoPoseGlobal.Pose2DCoordinates.PointsY.value()));
      if (i > 0U)
      {
         diffPoints = vfc::abs(distances[i] - distances[i - 1]);
         diffCorridorEgo =
            corridorEgo.corridorPoints[i].PointsX.value() - corridorEgo.corridorPoints[i - 1].PointsX.value();
         if (diffPoints > 6.0f)
         {
            invalidPoint = true;
            break;
         }
         else if (diffCorridorEgo <= 0.0f)
         {
            monotonIncreasing = false;
            break;
         }
      }
   }

   if (invalidPoint == false && monotonIncreasing == true)
   {
      validity = true;
   }
}

void ControlLogic::corridorValidityCalc(
   const CorridorInfoCoefficients& corridorCoefficients, const Pose2D& egoPoseGlobal)
{
   vfc::float32_t movMeanX{0.0f};
   vfc::float32_t movMeanY{0.0f};
   vfc::float32_t movMeanTheta{0.0f};
   vfc::float32_t previousMovMeanX{0.0f};
   vfc::float32_t previousMovMeanY{0.0f};
   vfc::float32_t previousMovMeanTheta{0.0f};
   bool           invalidEgoPose{false};
   validity = true;

   // checking GPS data validity
   // uploading the gps data array for further moving average calculation
   // (later consider using vfc buffer type)
   if (egoPoseDatasX[3] == 0.0f && egoPoseDatasY[3] == 0.0f && egoPoseDatasTheta[3] == 0.0f)
   {
      for (uint8_t i{0}; i < 4; i++)
      {
         if (egoPoseDatasX[i] == 0.0f)
         {
            egoPoseDatasX[i]     = egoPoseGlobal.Pose2DCoordinates.PointsX.value();
            egoPoseDatasY[i]     = egoPoseGlobal.Pose2DCoordinates.PointsY.value();
            egoPoseDatasTheta[i] = egoPoseGlobal.Pose2DTheta.value();
         }
      }
   }
   // if the array is uploaded, perform the calculations
   else
   {
      previousMovMeanX     = 0.0f;
      previousMovMeanY     = 0.0f;
      previousMovMeanTheta = 0.0f;
      for (uint8_t i{0}; i < 4; i++)
      {
         previousMovMeanX += egoPoseDatasX[i];
         previousMovMeanY += egoPoseDatasY[i];
         previousMovMeanTheta += egoPoseDatasTheta[i];

         if (i != 3)
         {
            egoPoseDatasX[i]     = egoPoseDatasX[i + 1];
            egoPoseDatasY[i]     = egoPoseDatasY[i + 1];
            egoPoseDatasTheta[i] = egoPoseDatasTheta[i + 1];
         }
         else
         {
            egoPoseDatasX[i]     = egoPoseGlobal.Pose2DCoordinates.PointsX.value();
            egoPoseDatasY[i]     = egoPoseGlobal.Pose2DCoordinates.PointsY.value();
            egoPoseDatasTheta[i] = egoPoseGlobal.Pose2DTheta.value();
         }
      }
      previousMovMeanX     = vfc::divide(previousMovMeanX, 4.0f);
      previousMovMeanY     = vfc::divide(previousMovMeanY, 4.0f);
      previousMovMeanTheta = vfc::divide(previousMovMeanTheta, 4.0f);
      for (uint8_t i{0}; i < 4; i++)
      {
         movMeanX += egoPoseDatasX[i];
         movMeanY += egoPoseDatasY[i];
         movMeanTheta += egoPoseDatasTheta[i];
      }
      movMeanX     = vfc::divide(movMeanX, 4.0f);
      movMeanY     = vfc::divide(movMeanY, 4.0f);
      movMeanTheta = vfc::divide(movMeanTheta, 4.0f);

      if (
         vfc::abs(movMeanX - previousMovMeanX) > 4 || vfc::abs(movMeanY - previousMovMeanY) > 4
         || vfc::abs(movMeanTheta - previousMovMeanTheta) > 0.3)
      {
         validity = false;
         return;
      }
   }

   // checking corridor validity
   if (
      vfc::abs(corridorCoefficients.c0) < 2.2f && vfc::abs(corridorCoefficients.c1) < 1.0f
      && vfc::abs(corridorCoefficients.c2) < 0.01f && vfc::abs(corridorCoefficients.c3) < 0.01f)
   {
      validity = true;
   }
}

void ControlLogic::initalizedCalc(
   const CorridorInfo& corridorGlobal,
   const Pose2D&       egoPoseGlobal,
   const bool          validity,
   const bool          firstCycle)
{
   vfc::CSI::si_metre_f32_t distance{0.0f};
   vfc::CSI::si_metre_f32_t distanceMax{0.0f};

   for (uint16_t i{0U}; i < corridorGlobal.corridorLength; i++)
   {
      distance = vfc::sqrt(
         vfc::sqr(corridorGlobal.corridorPoints[i].PointsX - egoPoseGlobal.Pose2DCoordinates.PointsX)
         + vfc::sqr(corridorGlobal.corridorPoints[i].PointsY - egoPoseGlobal.Pose2DCoordinates.PointsY));
      if (distance > distanceMax)
      {
         distanceMax = distance;
      }
   }

   if (!firstCycle && distanceMax.value() > 50.0f && validity == true)
   {
      initialized = true;
   }
   else
   {
      initialized = false;
   }
}

void ControlLogic::initalizedCalc(const bool validity, const bool firstCycle)
{
   if (!firstCycle && validity == true)
   {
      initialized = true;
   }
   else
   {
      initialized = false;
   }
}

void ControlLogic::replanCalc(const LDMParamIn& parameters)
{
   replanCount += 1;
   if (replanCount < parameters.replanCycle)
   {
      replan = false;
   }
   else
   {
      replan      = true;
      replanCount = 0U;
   }
}

bool ControlLogic::calcValidity(
   const CorridorInfo& corridorEgo,
   const CorridorInfo& corridorGlobal,
   const Pose2D&       egoPoseGlobal,
   const LDMParamIn&   parameters,
   const bool          firstCycle)
{
   corridorValidityCalc(corridorEgo, corridorGlobal, egoPoseGlobal);
   initalizedCalc(corridorGlobal, egoPoseGlobal, validity, firstCycle);
   replanCalc(parameters);

   return ((replan || !initialized) && validity);
}

bool ControlLogic::calcValidity(
   const CorridorInfoCoefficients& corridorCoefficients,
   const Pose2D&                   egoPoseGlobal,
   const LDMParamIn&               parameters,
   const bool                      firstCycle)
{
   corridorValidityCalc(corridorCoefficients, egoPoseGlobal);
   initalizedCalc(validity, firstCycle);
   replanCalc(parameters);

   return ((replan || !initialized) && validity);
}


}  // namespace Emg
}  // namespace Dc

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

#include "linearDriverModel/emg_linearDriverModel.hpp"


void ControlLogic::corridorValidityCalc(
   const ScenarioPolynomials& corridorCoefficients, const Pose2D& egoPoseGlobal)
{
   float movMeanX{0.0f};
   float movMeanY{0.0f};
   float movMeanTheta{0.0f};
   float previousMovMeanX{0.0f};
   float previousMovMeanY{0.0f};
   float previousMovMeanTheta{0.0f};
   bool  invalidEgoPose{false};
   validity = true;

   // checking GPS data validity
   // uploading the gps data array for further moving average calculation
   if (egoPoseDatasX[3] == 0.0f && egoPoseDatasY[3] == 0.0f && egoPoseDatasTheta[3] == 0.0f)
   {
      for (uint8_t i{0}; i < 4; i++)
      {
         if (egoPoseDatasX[i] == 0.0f)
         {
            egoPoseDatasX[i]     = egoPoseGlobal.Pose2DCoordinates.x;
            egoPoseDatasY[i]     = egoPoseGlobal.Pose2DCoordinates.y;
            egoPoseDatasTheta[i] = egoPoseGlobal.Pose2DTheta;
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
         previousMovMeanX     += egoPoseDatasX[i];
         previousMovMeanY     += egoPoseDatasY[i];
         previousMovMeanTheta += egoPoseDatasTheta[i];

         if (i != 3)
         {
            egoPoseDatasX[i]     = egoPoseDatasX[i + 1];
            egoPoseDatasY[i]     = egoPoseDatasY[i + 1];
            egoPoseDatasTheta[i] = egoPoseDatasTheta[i + 1];
         }
         else
         {
            egoPoseDatasX[i]     = egoPoseGlobal.Pose2DCoordinates.x;
            egoPoseDatasY[i]     = egoPoseGlobal.Pose2DCoordinates.y;
            egoPoseDatasTheta[i] = egoPoseGlobal.Pose2DTheta;
         }
      }
      previousMovMeanX     = previousMovMeanX     / 4.0f;
      previousMovMeanY     = previousMovMeanY     / 4.0f;
      previousMovMeanTheta = previousMovMeanTheta / 4.0f;
      for (uint8_t i{0}; i < 4; i++)
      {
         movMeanX += egoPoseDatasX[i];
         movMeanY += egoPoseDatasY[i];
         movMeanTheta += egoPoseDatasTheta[i];
      }
      movMeanX     = movMeanX     / 4.0f;
      movMeanY     = movMeanY     / 4.0f;
      movMeanTheta = movMeanTheta / 4.0f;

      if (
         abs(movMeanX - previousMovMeanX) > 4 || abs(movMeanY - previousMovMeanY) > 4
         || abs(movMeanTheta - previousMovMeanTheta) > 0.3)
      {
         validity = false;
         return;
      }
   }

   // TODO: validity check for every polynomial
   // checking corridor validity
   // if (
   //    abs(corridorCoefficients.c0) < 2.2f && abs(corridorCoefficients.c1) < 1.0f
   //    && abs(corridorCoefficients.c2) < 0.01f && abs(corridorCoefficients.c3) < 0.01f)
   // {
   //    validity = true;
   // }
   validity = true;
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
   const ScenarioPolynomials& corridorCoefficients,
   const Pose2D&                   egoPoseGlobal,
   const LDMParamIn&               parameters,
   const bool                      firstCycle)
{
   corridorValidityCalc(corridorCoefficients, egoPoseGlobal);
   initalizedCalc(validity, firstCycle);
   replanCalc(parameters);

   return ((replan || !initialized) && validity);
}

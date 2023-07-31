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

#include "linearDriverModelUtilities/emg_linearDriverModel_polynomialSubfunctions.hpp"

namespace Rb
{
namespace Vmc
{
void PolynomialSubfunctions::gaussElimination(
   float (&gaussMatrix)[4][5], PolynomialCoeffs& polynomialCoeffs)
{
   int singular = forwardElimination(gaussMatrix);

   if (singular != -1)
   {
      polynomialCoeffs.c0 = 0;
      polynomialCoeffs.c1 = 0;
      polynomialCoeffs.c2 = 0;
      polynomialCoeffs.c3 = 0;
      return;
   }

   backSubstitute(gaussMatrix);

   polynomialCoeffs.c0 = gaussResult[0];
   polynomialCoeffs.c1 = gaussResult[1];
   polynomialCoeffs.c2 = gaussResult[2];
   polynomialCoeffs.c3 = gaussResult[3];
}

PolynomialCoeffs PolynomialSubfunctions::fitThirdOrderPolynomial(const TrajectoryPoints& trajectory)
{
   calculateMmatrix(trajectory);
   detM = calculateDeterminant(M);
   calculateBvector(trajectory);

   for (uint8_t i = 0; i < 4; i++)
   {
      calculateModifiedMMatrix(i);
      a[i] = calculateDeterminant(M_) / detM;
   }

   polyCoeffs.c0 = a[0];
   polyCoeffs.c1 = a[1];
   polyCoeffs.c2 = a[2];
   polyCoeffs.c3 = a[3];

   return polyCoeffs;
}

uint8_t PolynomialSubfunctions::forwardElimination(float (&gaussMatrix)[4][5])
{
   for (uint8_t k{0U}; k < 4U; k++)
   {
      uint8_t i_max = k;
      float v_max = gaussMatrix[i_max][k];

      for (uint8_t i{k + 1U}; i < 4U; i++)
      {
         if (abs(gaussMatrix[i][k]) > v_max)
         {
            v_max = gaussMatrix[i][k];
            i_max = i;
         }
      }

      if (!gaussMatrix[k][k])
      {
         return k;
      }

      if (i_max != k)
      {
         for (uint8_t i{0U}; i <= 4U; i++)
         {
            float temp   = gaussMatrix[k][i];
            gaussMatrix[k][i]     = gaussMatrix[i_max][i];
            gaussMatrix[i_max][i] = temp;
         }
      }

      for (uint8_t i{k + 1U}; i < 4U; i++)
      {
         float f = gaussMatrix[i][k] - gaussMatrix[k][k];
         for (uint8_t j{k + 1U}; j <= 4U; j++)
         {
            gaussMatrix[i][j] -= gaussMatrix[k][j] * f;
         }
         gaussMatrix[i][k] = 0.0f;
      }
   }
   return -1;
}

void PolynomialSubfunctions::backSubstitute(float (&gaussMatrix)[4][5])
{
   for (int8_t i{3}; i >= 0; i--)
   {
      gaussResult[i] = gaussMatrix[i][4U];
      for (uint8_t j{i + 1U}; j < 4U; j++)
      {
         gaussResult[i] -= gaussMatrix[i][j] * gaussResult[j];
      }
      gaussResult[i] = gaussResult[i] - gaussMatrix[i][i];
   }
}

void PolynomialSubfunctions::calculateBvector(const TrajectoryPoints& trajectory)
{
   float sum = 0.0f;
   for (uint8_t i = 0; i < 4; i++)
   {
      sum = 0;
      for (uint16_t j = 0; j < trajectory.size(); j++)
      {
         sum = sum + pow(trajectory[j].x, i) * trajectory[j].y;
      }
      b[i] = sum;
   }
}

void PolynomialSubfunctions::calculateMmatrix(const TrajectoryPoints& trajectory)
{
   float sum = 0.0f;
   for (uint8_t i = 0; i < 9; i++)
   {
      sum = 0;
      for (uint16_t j = 0; j < trajectory.size(); j++)
      {
         sum = sum + pow(trajectory[j].x, i);
      }
      v[i] = sum;
   }
   for (uint8_t i = 0; i < 4; i++)
   {
      for (uint8_t j = 0; j < 4; j++)
      {
         M[i][j] = v[i + j];
      }
   }
}

float PolynomialSubfunctions::calculateDeterminant(float Mx[4][4])
{
   float D = 0;
   for (uint8_t i = 0; i < 4; i++)
   {
      float dM = calculateSubDeterminant(Mx, i);
      if (i == 0 || i == 2)
      {
         D = D + Mx[0][i] * dM;
      }
      else
      {
         D = D - Mx[0][i] * dM;
      }
   }
   return D;
}

float PolynomialSubfunctions::calculateSubDeterminant(float Mx[4][4], uint8_t i)
{
   float m[3][3];
   uint8_t columns[3];
   switch (i)
   {
      case 0:
         columns[0] = 1;
         columns[1] = 2;
         columns[2] = 3;
         break;
      case 1:
         columns[0] = 0;
         columns[1] = 2;
         columns[2] = 3;
         break;
      case 2:
         columns[0] = 0;
         columns[1] = 1;
         columns[2] = 3;
         break;
      case 3:
         columns[0] = 0;
         columns[1] = 1;
         columns[2] = 2;
         break;
   }
   for (uint8_t j = 0; j < 3; j++)
   {
      for (uint8_t k = 0; k < 3; k++)
      {
         m[j][k] = Mx[j + 1][columns[k]];
      }
   }

   return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
          - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
          + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

void PolynomialSubfunctions::calculateModifiedMMatrix(uint8_t k)
{
   for (uint8_t i = 0; i < 4; i++)
   {
      for (uint8_t j = 0; j < 4; j++)
      {
         if (j == k)
         {
            M_[i][j] = b[i];
         }
         else
         {
            M_[i][j] = M[i][j];
         }
      }
   }
}

}  // namespace Emg
}  // namespace Dc

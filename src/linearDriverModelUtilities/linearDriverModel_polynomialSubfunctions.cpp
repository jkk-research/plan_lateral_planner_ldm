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

#include "../../inc/linearDriverModelUtilities/linearDriverModel_polynomialSubfunctions.hpp"


void PolynomialSubfunctions::gaussElimination(
   float (&gaussMatrix)[4][5], PolynomialCoeffs& polynomialCoeffs)
{
   int8_t singular = forwardElimination(gaussMatrix);

   if (singular != -1)
   {
      polynomialCoeffs.c0 = 0;
      polynomialCoeffs.c1 = 0;
      polynomialCoeffs.c2 = 0;
      polynomialCoeffs.c3 = 0;
      return;
   }

   backSubstitute(gaussMatrix);

   polynomialCoeffs.c0 = m_gaussResult[0];
   polynomialCoeffs.c1 = m_gaussResult[1];
   polynomialCoeffs.c2 = m_gaussResult[2];
   polynomialCoeffs.c3 = m_gaussResult[3];
}

PolynomialCoeffs PolynomialSubfunctions::fitThirdOrderPolynomial(const TrajectoryPoints& trajectory)
{
   calculateMmatrix(trajectory);
   m_detM = calculateDeterminant(m_M);
   calculateBvector(trajectory);

   for (uint8_t i = 0; i < 4; i++)
   {
      calculateModifiedMMatrix(i);
      m_a[i] = calculateDeterminant(m_M_) / m_detM;
   }

   m_polyCoeffs.c0 = m_a[0];
   m_polyCoeffs.c1 = m_a[1];
   m_polyCoeffs.c2 = m_a[2];
   m_polyCoeffs.c3 = m_a[3];

   return m_polyCoeffs;
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
         float f = gaussMatrix[i][k] / gaussMatrix[k][k];
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
      m_gaussResult[i] = gaussMatrix[i][4U];
      for (uint8_t j{i + 1U}; j < 4U; j++)
      {
         m_gaussResult[i] -= gaussMatrix[i][j] * m_gaussResult[j];
      }
      m_gaussResult[i] = m_gaussResult[i] / gaussMatrix[i][i];
   }
}

void PolynomialSubfunctions::calculateBvector(const TrajectoryPoints& trajectory)
{
   double sum = 0.0f;
   for (uint8_t i = 0; i < 4; i++)
   {
      sum = 0;
      for (uint16_t j = 0; j < trajectory.size(); j++)
      {
         sum = sum + pow(trajectory[j].x, i) * trajectory[j].y;
      }
      m_b[i] = sum;
   }
}

void PolynomialSubfunctions::calculateMmatrix(const TrajectoryPoints& trajectory)
{
   double sum = 0.0f;
   for (uint8_t i = 0; i < 7; i++)
   {
      sum = 0;
      for (uint16_t j = 0; j < trajectory.size(); j++)
      {
         sum = sum + pow(trajectory[j].x, i);
      }
      m_v[i] = sum;
   }
   for (uint8_t i = 0; i < 4; i++)
   {
      for (uint8_t j = 0; j < 4; j++)
      {
         m_M[i][j] = m_v[i + j];
      }
   }
}

double PolynomialSubfunctions::calculateDeterminant(double Mx[4][4])
{
   double D = 0;
   for (uint8_t i = 0; i < 4; i++)
   {
      double dM = calculateSubDeterminant(Mx, i);
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

double PolynomialSubfunctions::calculateSubDeterminant(double Mx[4][4], uint8_t i)
{
   double m[3][3];
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
            m_M_[i][j] = m_b[i];
         }
         else
         {
            m_M_[i][j] = m_M[i][j];
         }
      }
   }
}

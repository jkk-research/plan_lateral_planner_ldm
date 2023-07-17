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

vfc::float32_t ClothoidSubfunctions::normalizeAngle(vfc::float32_t phi_in)
{
   vfc::float32_t phi = phi_in;
   while (phi > vfc::G_PI)
   {
      phi -= vfc::G_2PI;
   }
   while (phi < -vfc::G_PI)
   {
      phi += vfc::G_2PI;
   }
   return phi;
}

vfc::float32_t ClothoidSubfunctions::guessA(vfc::float32_t phi0, vfc::float32_t phi1)
{
   const vfc::float32_t CF[]{2.989696028701907f,
                             0.716228953608281f,
                             -0.458969738821509f,
                             -0.502821153340377f,
                             0.261062141752652f,
                             -0.045854475238709f};
   vfc::float32_t       X  = phi0 / vfc::G_PI;
   vfc::float32_t       Y  = phi1 / vfc::G_PI;
   vfc::float32_t       xy = X * Y;
   vfc::float32_t       A =
      (phi0 + phi1)
      * (CF[0U] + xy * (CF[1U] + xy * CF[2U]) + (CF[3U] + xy * CF[4U]) * (vfc::sqr(X) + vfc::sqr(Y))
         + CF[5U] * (vfc::pow(X, 4U) + vfc::pow(Y, 4U)));
   return A;
}

void ClothoidSubfunctions::fresnelCS(vfc::float32_t y, vfc::float32_t& C, vfc::float32_t& S)
{
   const vfc::float32_t eps = 2.2204e-16f;
   const vfc::float32_t fn[]{0.49999988085884732562f,
                             1.3511177791210715095f,
                             1.3175407836168659241f,
                             1.1861149300293854992f,
                             0.7709627298888346769f,
                             0.4173874338787963957f,
                             0.19044202705272903923f,
                             0.06655998896627697537f,
                             0.022789258616785717418f,
                             0.0040116689358507943804f,
                             0.0012192036851249883877f};

   const vfc::float32_t fd[]{1.0f,
                             2.7022305772400260215f,
                             4.2059268151438492767f,
                             4.5221882840107715516f,
                             3.7240352281630359588f,
                             2.4589286254678152943f,
                             1.3125491629443702962f,
                             0.5997685720120932908f,
                             0.20907680750378849485f,
                             0.07159621634657901433f,
                             0.012602969513793714191f,
                             0.0038302423512931250065f};

   const vfc::float32_t gn[]{0.50000014392706344801f,
                             0.032346434925349128728f,
                             0.17619325157863254363f,
                             0.038606273170706486252f,
                             0.023693692309257725361f,
                             0.007092018516845033662f,
                             0.0012492123212412087428f,
                             0.00044023040894778468486f,
                             -8.80266827476172521e-6f,
                             -1.4033554916580018648e-8f,
                             2.3509221782155474353e-10f};

   const vfc::float32_t gd[]{1.0f,
                             2.0646987497019598937f,
                             2.9109311766948031235f,
                             2.6561936751333032911f,
                             2.0195563983177268073f,
                             1.1167891129189363902f,
                             0.57267874755973172715f,
                             0.19408481169593070798f,
                             0.07634808341431248904f,
                             0.011573247407207865977f,
                             0.0044099273693067311209f,
                             -0.00009070958410429993314f};

   C = 0.0f;
   S = 0.0f;

   vfc::float32_t x = vfc::abs(y);
   if (x < 1.0f)
   {
      vfc::float32_t t       = -vfc::pow(vfc::G_PI_2 * vfc::sqr(x), 2U);
      vfc::float32_t twofn   = 0.0f;
      vfc::float32_t fact    = 1.0f;
      vfc::float32_t denterm = 1.0f;
      vfc::float32_t numterm = 1.0f;
      vfc::float32_t sum     = 1.0f;
      vfc::float32_t ratio   = 10.0f;

      while (ratio > eps)
      {
         twofn += 2.0f;
         fact *= twofn * (twofn - 1.0f);
         denterm += 4.0f;
         numterm *= t;
         vfc::float32_t term = vfc::divide(numterm, fact * denterm);
         sum += term;
         ratio = vfc::abs(vfc::divide(term, sum));
      }
      C = x * sum;

      twofn   = 1.0f;
      fact    = 1.0f;
      denterm = 3.0f;
      numterm = 1.0f;
      sum     = vfc::G_1_3;
      ratio   = 10.0f;

      while (ratio > eps)
      {
         twofn += 2.0f;
         fact *= twofn * (twofn - 1.0f);
         denterm += 4.0f;
         numterm *= t;
         vfc::float32_t term = vfc::divide(numterm, fact * denterm);
         sum += term;
         ratio = vfc::abs(vfc::divide(term, sum));
      }
      S = vfc::G_PI_2 * sum * vfc::pow(x, 3U);
   }
   else if (x < 6.0f)
   {
      vfc::float32_t sumn = 0.0f;
      vfc::float32_t sumd = fd[11U];
      for (int8_t i{11}; i >= 0L; --i)
      {
         sumn = fn[i] + x * sumn;
         sumd = fd[i] + x * sumd;
      }
      vfc::float32_t f = vfc::divide(sumn, sumd);

      sumn = 0.0f;
      sumd = gd[11U];
      for (int8_t i{11}; i >= 0L; --i)
      {
         sumn = gn[i] + x * sumn;
         sumd = gd[i] + x * sumd;
      }
      vfc::float32_t g    = vfc::divide(sumn, sumd);
      vfc::float32_t U    = vfc::G_PI_2 * vfc::sqr(x);
      vfc::float32_t sinU = vfc::sin(static_cast<vfc::CRadian32>(U));
      vfc::float32_t cosU = vfc::cos(static_cast<vfc::CRadian32>(U));

      C = 0.5f + f * sinU - g * cosU;
      S = 0.5f - f * cosU - g * sinU;
   }
   else
   {
      vfc::float32_t t = -vfc::pow((vfc::G_PI * vfc::sqr(x)), -2.0f);

      vfc::float32_t numterm = -1.0f;
      vfc::float32_t term    = 1.0f;
      vfc::float32_t sum     = 1.0f;
      vfc::float32_t oldterm = 1.0f;
      vfc::float32_t ratio   = 10.0f;
      vfc::float32_t eps10   = 0.1f * eps;

      while (ratio > eps10)
      {
         numterm += 4.0f;
         term *= numterm * (numterm - 2.0f) * t;
         sum += term;
         vfc::float32_t absterm = vfc::abs(term);
         ratio                  = vfc::abs(vfc::divide(term, sum));
         if (oldterm < absterm)
         {
            ratio = eps10;
         }
         oldterm = absterm;
      }
      vfc::float32_t f = vfc::divide(sum, vfc::G_PI * x);
      numterm          = -1.0f;
      term             = 1.0f;
      sum              = 1.0f;
      oldterm          = 1.0f;
      ratio            = 10.0f;
      eps10            = 0.1f * eps;

      while (ratio > eps10)
      {
         numterm += 4.0f;
         term *= numterm * (numterm + 2.0f) * t;
         sum += term;
         vfc::float32_t absterm = vfc::abs(term);
         ratio                  = vfc::abs(vfc::divide(term, sum));
         if (oldterm < absterm)
         {
            ratio = eps10;
         }
         oldterm = absterm;
      }
      vfc::float32_t g    = vfc::divide(sum, vfc::pow(vfc::G_PI * x, 2U * x));
      vfc::float32_t U    = vfc::G_PI_2 * vfc::sqr(x);
      vfc::float32_t sinU = vfc::sin(static_cast<vfc::CRadian32>(U));
      vfc::float32_t cosU = vfc::cos(static_cast<vfc::CRadian32>(U));

      C = 0.5f + f * sinU - g * cosU;
      S = 0.5f - f * cosU - g * sinU;
   }
   if (y < 0U)
   {
      C = -C;
      S = -S;
   }
}

void ClothoidSubfunctions::fresnelCSk(
   vfc::uint8_t nk, vfc::float32_t t, vfc::float32_t (&C)[3], vfc::float32_t (&S)[3])
{
   fresnelCS(t, C[0U], S[0U]);
   if (nk > 1U)
   {
      vfc::float32_t tt = vfc::G_PI_2 * vfc::sqr(t);
      vfc::float32_t ss = vfc::sin(static_cast<vfc::CRadian32>(tt));
      vfc::float32_t cc = vfc::cos(static_cast<vfc::CRadian32>(tt));
      C[1U]             = vfc::divide(ss, vfc::G_PI);
      S[1U]             = vfc::divide(1L - cc, vfc::G_PI);
      if (nk > 2U)
      {
         C[2U] = vfc::divide(t * ss - S[0U], vfc::G_PI);
         S[2U] = vfc::divide(C[0U] - t * cc, vfc::G_PI);
      }
   }
}

void ClothoidSubfunctions::evalXYaLarge(
   vfc::uint8_t nk, vfc::float32_t a, vfc::float32_t b, vfc::float32_t (&X)[3], vfc::float32_t (&Y)[3])
{
   vfc::int8_t    s   = vfc::signum(a);
   vfc::float32_t z   = vfc::sqrt(vfc::divide(vfc::abs(a), vfc::G_PI));
   vfc::float32_t ell = vfc::divide(s * b, vfc::sqrt(vfc::abs(a) * vfc::G_PI));
   vfc::float32_t g   = vfc::divide(-0.5f * s * vfc::sqr(b), vfc::abs(a));
   fresnelCSk(nk, ell, Cl, Sl);
   fresnelCSk(nk, ell + z, Cz, Sz);
   vfc::float32_t dC[3U]{Cz[0U] - Cl[0U], Cz[1U] - Cl[1U], Cz[2U] - Cl[2U]};
   vfc::float32_t dS[3U]{Sz[0U] - Sl[0U], Sz[1U] - Sl[1U], Sz[2U] - Sl[2U]};
   vfc::float32_t cg = vfc::divide(vfc::cos(static_cast<vfc::CRadian32>(g)), z);
   vfc::float32_t sg = vfc::divide(vfc::sin(static_cast<vfc::CRadian32>(g)), z);
   X[0U]             = cg * dC[0U] - s * sg * dS[0U];
   Y[0U]             = sg * dC[0U] + s * cg * dS[0U];
   if (nk > 1U)
   {
      cg                = vfc::divide(cg, z);
      sg                = vfc::divide(sg, z);
      vfc::float32_t DC = dC[1U] - ell * dC[0U];
      vfc::float32_t DS = dS[1U] - ell * dS[0U];
      X[1U]             = cg * DC - s * sg * DS;
      Y[1U]             = sg * DC + s * cg * DS;
      if (nk > 2U)
      {
         cg    = vfc::divide(cg, z);
         sg    = vfc::divide(sg, z);
         DC    = dC[2U] + ell * (ell * dC[0U] - 2U * dC[1U]);
         DS    = dS[2U] + ell * (ell * dS[0U] - 2U * dS[1U]);
         X[2U] = cg * DC - s * sg * DS;
         Y[2U] = sg * DC + s * cg * DS;
      }
   }
}

vfc::float32_t ClothoidSubfunctions::rLommel(vfc::float32_t mu, vfc::float32_t nu, vfc::float32_t b)
{
   vfc::float32_t tmp = vfc::divide(1.0f, (mu + nu + 1.0f) * (mu - nu + 1.0f));
   vfc::float32_t res = tmp;
   for (uint8_t i{1}; i <= 100U; i++)
   {
      tmp *= vfc::divide(-b, 2.0f * static_cast<vfc::float32_t>(i) + mu - nu + 1.0f)
             * vfc::divide(b, 2.0f * static_cast<vfc::float32_t>(i) + mu + nu + 1.0f);
      res += tmp;
      if (vfc::abs(tmp) < vfc::abs(res) * 1e-50l)
      {
         break;
      }
   }
   return res;
}

void ClothoidSubfunctions::evalXYaZero(
   vfc::uint8_t nk, vfc::float32_t b, vfc::float32_t (&X0)[17], vfc::float32_t (&Y0)[17])
{
   vfc::float32_t sb = vfc::sin(static_cast<vfc::CRadian32>(b));
   vfc::float32_t cb = vfc::cos(static_cast<vfc::CRadian32>(b));
   vfc::float32_t b2 = vfc::sqr(b);
   if (vfc::abs(b) < 1e-3f)
   {
      X0[0U] = 1L - (b2 / 6U) * (1L - (vfc::divide(b2, 20.0f) * (1L - vfc::divide(b2, 42.0f))));
      Y0[0U] = vfc::divide(b, 2.0f) * (1L - vfc::divide(b2, 12.0f)) * (1L - vfc::divide(b2, 30.0f));
   }
   else
   {
      X0[0U] = vfc::divide(sb, b);
      Y0[0U] = vfc::divide(1L - cb, b);
   }
   vfc::float32_t m = vfc::min(vfc::max(1.0f, vfc::floor(2.0f * b)), static_cast<vfc::float32_t>(nk));
   for (uint8_t i{0}; i <= m - 2U; i++)
   {
      X0[i + 1U] = vfc::divide(sb - i * Y[i], b);
      Y0[i + 1U] = vfc::divide(i * X[i] - cb, b);
   }
   if (m < nk)
   {
      vfc::float32_t A   = b * sb;
      vfc::float32_t D   = sb - b * cb;
      vfc::float32_t B   = b * D;
      vfc::float32_t C   = -b2 * sb;
      vfc::float32_t rLa = rLommel(vfc::divide(m + 1.0f, 2.0f), vfc::divide(3.0f, 2.0f), b);
      vfc::float32_t rLd = rLommel(vfc::divide(m + 1.0f, 2.0f), vfc::divide(1.0f, 2.0f), b);
      for (uint8_t i = m - 1U; i <= nk - 2U; i++)
      {
         vfc::float32_t rLb = rLommel(vfc::divide(i + 3.0f, 2.0f), vfc::divide(1.0f, 2.0f), b);
         vfc::float32_t rLc = rLommel(vfc::divide(i + 3.0f, 2.0f), vfc::divide(3.0f, 2.0f), b);
         X0[i + 1U]         = vfc::divide(i * A * rLa + B * rLb + cb, 2.0f + i);
         Y0[i + 1U]         = vfc::divide(C * rLc + sb, (3.0f + i) + D * rLd);
         rLa                = rLc;
         rLd                = rLb;
      }
   }
}

void ClothoidSubfunctions::evalXYaSmall(
   vfc::uint8_t   nk,
   vfc::float32_t a,
   vfc::float32_t b,
   vfc::uint8_t   p,
   vfc::float32_t (&X)[3],
   vfc::float32_t (&Y)[3])
{
   evalXYaZero(nk + 4U * p + 2U, b, X0, Y0);
   vfc::float32_t tmpX[4]{0};
   vfc::float32_t tmpY[4]{0};

   for (uint8_t i{0}; i <= nk - 1U; i++)
   {
      tmpX[0U]          = X0[i] - vfc::divide(a, 2.0f) * Y0[i + 2U];
      tmpY[0U]          = Y0[i] + vfc::divide(a, 2.0f) * X0[i + 2U];
      vfc::float32_t t  = 1.0f;
      vfc::float32_t aa = -vfc::sqr(vfc::divide(a, 2.0f));
      for (uint8_t j{0}; j <= 2U; j++)
      {
         vfc::uint8_t   ii  = (4U * (j + 1U) + (i + 1U)) - 1U;
         vfc::float32_t div = 2U * (j + 1.0f) * (2U * (j + 1U) - 1U);
         t *= vfc::divide(aa, div);
         vfc::float32_t bf = vfc::divide(a, 4.0f * (j + 1.0f) + 2.0f);
         tmpX[1U + j]      = t * (X0[ii] - bf * Y0[ii + 2U]);
         tmpY[1U + j]      = t * (Y0[ii] + bf * X0[ii + 2U]);
      }
      X[i] = tmpX[0U] + tmpX[1U] + tmpX[2U] + tmpX[3U];
      Y[i] = tmpY[0U] + tmpY[1U] + tmpY[2U] + tmpY[3U];
   }
}

void ClothoidSubfunctions::generalizedFresnelCS(
   vfc::uint8_t   nk,
   vfc::float32_t a,
   vfc::float32_t b,
   vfc::float32_t c,
   vfc::float32_t (&X)[3],
   vfc::float32_t (&Y)[3])
{
   vfc::float32_t epsi = 1e-2f;
   if (vfc::abs(a) < epsi)
   {
      evalXYaSmall(nk, a, b, 3U, X, Y);
   }
   else
   {
      evalXYaLarge(nk, a, b, X, Y);
   }
   vfc::float32_t cc = vfc::cos(static_cast<vfc::CRadian32>(c));
   vfc::float32_t ss = vfc::sin(static_cast<vfc::CRadian32>(c));

   for (uint8_t i{0}; i <= nk - 1U; i++)
   {
      vfc::float32_t xx = X[i];
      vfc::float32_t yy = Y[i];

      X[i] = xx * cc - yy * ss;
      Y[i] = xx * ss + yy * cc;
   }
}

vfc::float32_t ClothoidSubfunctions::findA(
   vfc::float32_t Aguess, vfc::float32_t delta, vfc::float32_t phi0, vfc::float32_t tol)
{
   vfc::float32_t A = Aguess;
   for (uint8_t i{1}; i <= 100U; i++)
   {
      generalizedFresnelCS(3U, 2L * A, delta - A, phi0, intC, intS);
      vfc::float32_t f  = intS[0U];
      vfc::float32_t df = intC[2U] - intC[1U];
      A -= vfc::divide(f, df);
      if (vfc::abs(f) < tol)
      {
         break;
      }
   }
   return A;
}

}  // namespace Emg
}  // namespace Dc

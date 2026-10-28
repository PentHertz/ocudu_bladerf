// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Passes of the Highway DFT, for the Highway target that the including source file selects.
///
/// See dft_processor_hwy_params.h for the algorithm. Each source file dft_processor_hwy_<target>.cpp defines
/// HWY_TARGET, includes this header, and returns the entry points of get_target_kernels() from its own function.
///
/// \note Disclaimer: this file was generated with Claude Code, the agentic coding tool from Anthropic, using the
/// Claude Opus 5.5 model (claude-opus-5-5).

#pragma once
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wextra-semi"

#include "dft_processor_hwy_params.h"
#include "hwy/highway.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <type_traits>

HWY_BEFORE_NAMESPACE();
namespace ocudu {
namespace dft_hwy {
namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

// The DFT needs vectors of at least four single precision lanes, with a size known at compile time, so that they can be
// array elements. The scalar target has one lane, and the SVE and RVV vectors are sizeless.
#if (HWY_TARGET == HWY_SCALAR) || HWY_HAVE_SCALABLE
#error "The Highway DFT needs fixed size vectors of four or more lanes."
#endif

/// Multiplies two complex vectors, with the same operations on every target.
template <class V>
HWY_INLINE void complex_mul(V& re, V& im, V w_re, V w_im)
{
  V out_re = hn::MulSub(re, w_re, hn::Mul(im, w_im));
  V out_im = hn::MulAdd(re, w_im, hn::Mul(im, w_re));
  re       = out_re;
  im       = out_im;
}

/// Computes a radix-4 butterfly in place: X1 = d02 + j * Sign * d13 and X3 = d02 - j * Sign * d13.
template <int Sign, class V>
HWY_INLINE void radix4(V* re, V* im)
{
  V a_re = hn::Add(re[0], re[2]), a_im = hn::Add(im[0], im[2]);
  V b_re = hn::Sub(re[0], re[2]), b_im = hn::Sub(im[0], im[2]);
  V c_re = hn::Add(re[1], re[3]), c_im = hn::Add(im[1], im[3]);
  V d_re = hn::Sub(re[1], re[3]), d_im = hn::Sub(im[1], im[3]);
  re[0] = hn::Add(a_re, c_re);
  im[0] = hn::Add(a_im, c_im);
  re[2] = hn::Sub(a_re, c_re);
  im[2] = hn::Sub(a_im, c_im);
  if constexpr (Sign > 0) {
    re[1] = hn::Sub(b_re, d_im);
    im[1] = hn::Add(b_im, d_re);
    re[3] = hn::Add(b_re, d_im);
    im[3] = hn::Sub(b_im, d_re);
  } else {
    re[1] = hn::Add(b_re, d_im);
    im[1] = hn::Sub(b_im, d_re);
    re[3] = hn::Sub(b_re, d_im);
    im[3] = hn::Add(b_im, d_re);
  }
}

/// Multiplies by W_8^q = e^(j * Sign * 2 * pi * q / 8), for q = 1, 2 or 3.
template <int Sign, unsigned Q, class D>
HWY_INLINE void mul_w8(D d, hn::Vec<D>& re, hn::Vec<D>& im)
{
  using V                        = hn::Vec<D>;
  static constexpr float sqrt1_2 = 0.70710678118654752440F;
  if constexpr (Q == 2) {
    // Multiplication by j * Sign.
    V out_re = (Sign > 0) ? hn::Neg(im) : im;
    V out_im = (Sign > 0) ? re : hn::Neg(re);
    re       = out_re;
    im       = out_im;
  } else if constexpr (Q == 1) {
    // (a + j b) (1 + j Sign) / sqrt(2).
    V out_re = (Sign > 0) ? hn::Sub(re, im) : hn::Add(re, im);
    V out_im = (Sign > 0) ? hn::Add(im, re) : hn::Sub(im, re);
    re       = hn::Mul(out_re, hn::Set(d, sqrt1_2));
    im       = hn::Mul(out_im, hn::Set(d, sqrt1_2));
  } else {
    // (a + j b) (-1 + j Sign) / sqrt(2).
    V out_re = (Sign > 0) ? hn::Neg(hn::Add(re, im)) : hn::Sub(im, re);
    V out_im = (Sign > 0) ? hn::Sub(re, im) : hn::Neg(hn::Add(re, im));
    re       = hn::Mul(out_re, hn::Set(d, sqrt1_2));
    im       = hn::Mul(out_im, hn::Set(d, sqrt1_2));
  }
}

/// Computes a radix-2 butterfly in place.
template <class V>
HWY_INLINE void radix2(V* re, V* im)
{
  V a_re = hn::Add(re[0], re[1]), a_im = hn::Add(im[0], im[1]);
  V b_re = hn::Sub(re[0], re[1]), b_im = hn::Sub(im[0], im[1]);
  re[0] = a_re;
  im[0] = a_im;
  re[1] = b_re;
  im[1] = b_im;
}

/// Computes a radix-3 butterfly in place: X1 = x0 - (x1 + x2) / 2 + j * Sign * sqrt(3) / 2 * (x1 - x2), and X2 the
/// same with -j.
template <int Sign, class D>
HWY_INLINE void radix3(D d, hn::Vec<D>* re, hn::Vec<D>* im)
{
  using V                      = hn::Vec<D>;
  static constexpr float sin60 = Sign * 0.86602540378443864676F;
  const V                half  = hn::Set(d, 0.5F);
  V                      s_re = hn::Add(re[1], re[2]), s_im = hn::Add(im[1], im[2]);
  V                      d_re = hn::Mul(hn::Sub(re[1], re[2]), hn::Set(d, sin60));
  V                      d_im = hn::Mul(hn::Sub(im[1], im[2]), hn::Set(d, sin60));
  V                      m_re = hn::NegMulAdd(half, s_re, re[0]);
  V                      m_im = hn::NegMulAdd(half, s_im, im[0]);
  re[0]                       = hn::Add(re[0], s_re);
  im[0]                       = hn::Add(im[0], s_im);
  re[1]                       = hn::Sub(m_re, d_im);
  im[1]                       = hn::Add(m_im, d_re);
  re[2]                       = hn::Add(m_re, d_im);
  im[2]                       = hn::Sub(m_im, d_re);
}

/// \brief Computes a radix-5 butterfly in place.
///
/// With the sums s1 = x1 + x4 and s2 = x2 + x3, and the differences d1 = x1 - x4 and d2 = x2 - x3, the outputs are
/// X1, X4 = x0 + c1 * s1 + c2 * s2 +/- j * (s1' * d1 + s2' * d2) and X2, X3 = x0 + c2 * s1 + c1 * s2 +/- j * (s2' * d1
/// - s1' * d2), where c and s' are the cosines and the signed sines of 2 * pi / 5 and 4 * pi / 5.
template <int Sign, class D>
HWY_INLINE void radix5(D d, hn::Vec<D>* re, hn::Vec<D>* im)
{
  using V                    = hn::Vec<D>;
  static constexpr float c1  = 0.30901699437494742410F;
  static constexpr float c2  = -0.80901699437494742410F;
  static constexpr float s1  = Sign * 0.95105651629515357212F;
  static constexpr float s2  = Sign * 0.58778525229247312917F;
  const V                vc1 = hn::Set(d, c1), vc2 = hn::Set(d, c2), vs1 = hn::Set(d, s1), vs2 = hn::Set(d, s2);

  V sum1_re = hn::Add(re[1], re[4]), sum1_im = hn::Add(im[1], im[4]);
  V sum2_re = hn::Add(re[2], re[3]), sum2_im = hn::Add(im[2], im[3]);
  V dif1_re = hn::Sub(re[1], re[4]), dif1_im = hn::Sub(im[1], im[4]);
  V dif2_re = hn::Sub(re[2], re[3]), dif2_im = hn::Sub(im[2], im[3]);

  V a1_re = hn::MulAdd(vc2, sum2_re, hn::MulAdd(vc1, sum1_re, re[0]));
  V a1_im = hn::MulAdd(vc2, sum2_im, hn::MulAdd(vc1, sum1_im, im[0]));
  V a2_re = hn::MulAdd(vc1, sum2_re, hn::MulAdd(vc2, sum1_re, re[0]));
  V a2_im = hn::MulAdd(vc1, sum2_im, hn::MulAdd(vc2, sum1_im, im[0]));
  V b1_re = hn::MulAdd(vs2, dif2_re, hn::Mul(vs1, dif1_re));
  V b1_im = hn::MulAdd(vs2, dif2_im, hn::Mul(vs1, dif1_im));
  V b2_re = hn::NegMulAdd(vs1, dif2_re, hn::Mul(vs2, dif1_re));
  V b2_im = hn::NegMulAdd(vs1, dif2_im, hn::Mul(vs2, dif1_im));

  re[0] = hn::Add(re[0], hn::Add(sum1_re, sum2_re));
  im[0] = hn::Add(im[0], hn::Add(sum1_im, sum2_im));
  re[1] = hn::Sub(a1_re, b1_im);
  im[1] = hn::Add(a1_im, b1_re);
  re[4] = hn::Add(a1_re, b1_im);
  im[4] = hn::Sub(a1_im, b1_re);
  re[2] = hn::Sub(a2_re, b2_im);
  im[2] = hn::Add(a2_im, b2_re);
  re[3] = hn::Add(a2_re, b2_im);
  im[3] = hn::Sub(a2_im, b2_re);
}

template <int Sign, unsigned R, class D>
HWY_INLINE void small_dft(D d, hn::Vec<D>* re, hn::Vec<D>* im);

/// \brief Computes a DFT of size R = A * B in registers, in place and in natural order.
///
/// It computes B DFTs of size A on the inputs b + B * q, multiplies the outputs c by W_R^(b * c), and computes A DFTs
/// of size B across b. The output c + A * q is the output q of the DFT c.
template <int Sign, unsigned A, unsigned B, class D>
HWY_INLINE void small_dft_composite(D d, hn::Vec<D>* re, hn::Vec<D>* im)
{
  using V                     = hn::Vec<D>;
  static constexpr unsigned R = A * B;
  // Real and imaginary parts of W_R^i.
  static constexpr auto twiddles = [] {
    std::array<float, 2 * R> table = {};
    HWY_UNROLL(32)
    for (unsigned i = 0; i != R; ++i) {
      table[2 * i]     = static_cast<float>(constexpr_cos_2pi(i, R));
      table[2 * i + 1] = static_cast<float>(Sign * constexpr_sin_2pi(i, R));
    }
    return table;
  }();

  V y_re[B][A];
  V y_im[B][A];
  HWY_UNROLL(32)
  for (unsigned b = 0; b != B; ++b) {
    HWY_UNROLL(32)
    for (unsigned q = 0; q != A; ++q) {
      y_re[b][q] = re[b + B * q];
      y_im[b][q] = im[b + B * q];
    }
    small_dft<Sign, A>(d, y_re[b], y_im[b]);
  }
  HWY_UNROLL(32)
  for (unsigned b = 1; b != B; ++b) {
    HWY_UNROLL(32)
    for (unsigned c = 1; c != A; ++c) {
      // The loops are unrolled, so the branches on the twiddle index resolve at compile time.
      unsigned i    = (b * c) % R;
      V&       x_re = y_re[b][c];
      V&       x_im = y_im[b][c];
      if (2 * i == R) {
        // Multiplication by -1.
        x_re = hn::Neg(x_re);
        x_im = hn::Neg(x_im);
      } else if ((4 * i == R) || (4 * i == 3 * R)) {
        // Multiplication by j * Sign or -j * Sign.
        bool plus_j = (4 * i == R) == (Sign > 0);
        V    out_re = plus_j ? hn::Neg(x_im) : x_im;
        V    out_im = plus_j ? x_re : hn::Neg(x_re);
        x_re        = out_re;
        x_im        = out_im;
      } else {
        complex_mul(x_re, x_im, hn::Set(d, twiddles[2 * i]), hn::Set(d, twiddles[2 * i + 1]));
      }
    }
  }
  HWY_UNROLL(32)
  for (unsigned c = 0; c != A; ++c) {
    V z_re[B];
    V z_im[B];
    HWY_UNROLL(32)
    for (unsigned b = 0; b != B; ++b) {
      z_re[b] = y_re[b][c];
      z_im[b] = y_im[b][c];
    }
    small_dft<Sign, B>(d, z_re, z_im);
    HWY_UNROLL(32)
    for (unsigned q = 0; q != B; ++q) {
      re[c + A * q] = z_re[q];
      im[c + A * q] = z_im[q];
    }
  }
}

/// Computes a DFT of size R in registers, in place and in natural order, for the radices of is_kernel_radix().
template <int Sign, unsigned R, class D>
HWY_INLINE void small_dft(D d, hn::Vec<D>* re, hn::Vec<D>* im)
{
  using V = hn::Vec<D>;
  if constexpr (R == 2) {
    radix2(re, im);
  } else if constexpr (R == 3) {
    radix3<Sign>(d, re, im);
  } else if constexpr (R == 4) {
    radix4<Sign>(re, im);
  } else if constexpr (R == 5) {
    radix5<Sign>(d, re, im);
  } else if constexpr (R == 6) {
    small_dft_composite<Sign, 3, 2>(d, re, im);
  } else if constexpr (R == 9) {
    small_dft_composite<Sign, 3, 3>(d, re, im);
  } else if constexpr (R == 10) {
    small_dft_composite<Sign, 5, 2>(d, re, im);
  } else if constexpr (R == 12) {
    small_dft_composite<Sign, 3, 4>(d, re, im);
  } else if constexpr (R == 15) {
    small_dft_composite<Sign, 5, 3>(d, re, im);
  } else if constexpr (R == 18) {
    small_dft_composite<Sign, 6, 3>(d, re, im);
  } else if constexpr (R == 24) {
    small_dft_composite<Sign, 8, 3>(d, re, im);
  } else if constexpr (R == 30) {
    small_dft_composite<Sign, 10, 3>(d, re, im);
  } else if constexpr (R == 32) {
    small_dft_composite<Sign, 8, 4>(d, re, im);
  } else if constexpr (R == 48) {
    small_dft_composite<Sign, 16, 3>(d, re, im);

  } else if constexpr (R == 8) {
    // Radix-4 DFTs of the even and the odd inputs, and radix-2 butterflies with W_8^q.
    V e_re[4] = {re[0], re[2], re[4], re[6]};
    V e_im[4] = {im[0], im[2], im[4], im[6]};
    V o_re[4] = {re[1], re[3], re[5], re[7]};
    V o_im[4] = {im[1], im[3], im[5], im[7]};
    radix4<Sign>(e_re, e_im);
    radix4<Sign>(o_re, o_im);
    mul_w8<Sign, 1>(d, o_re[1], o_im[1]);
    mul_w8<Sign, 2>(d, o_re[2], o_im[2]);
    mul_w8<Sign, 3>(d, o_re[3], o_im[3]);
    HWY_UNROLL(32)
    for (unsigned q = 0; q != 4; ++q) {
      re[q]     = hn::Add(e_re[q], o_re[q]);
      im[q]     = hn::Add(e_im[q], o_im[q]);
      re[q + 4] = hn::Sub(e_re[q], o_re[q]);
      im[q + 4] = hn::Sub(e_im[q], o_im[q]);
    }
  } else {
    static_assert(R == 16, "Unsupported radix.");
    // Four radix-4 DFTs on the inputs b, b + 4, b + 8 and b + 12, twiddle factors W_16^(b * c), and four radix-4 DFTs
    // across b. The output c + 4 * q is the output q of the radix-4 DFT c.
    static constexpr float c1 = 0.92387953251128675613F;
    static constexpr float s1 = 0.38268343236508977173F;
    V                      y_re[4][4];
    V                      y_im[4][4];
    HWY_UNROLL(32)
    for (unsigned b = 0; b != 4; ++b) {
      HWY_UNROLL(32)
      for (unsigned q = 0; q != 4; ++q) {
        y_re[b][q] = re[b + 4 * q];
        y_im[b][q] = im[b + 4 * q];
      }
      radix4<Sign>(y_re[b], y_im[b]);
    }
    // W_16^1, W_16^2 = W_8^1, W_16^3, W_16^4 = W_8^2, W_16^6 = W_8^3 and W_16^9 = -W_16^1.
    const V w1_re = hn::Set(d, c1), w1_im = hn::Set(d, Sign * s1);
    const V w3_re = hn::Set(d, s1), w3_im = hn::Set(d, Sign * c1);
    complex_mul(y_re[1][1], y_im[1][1], w1_re, w1_im);
    mul_w8<Sign, 1>(d, y_re[1][2], y_im[1][2]);
    complex_mul(y_re[1][3], y_im[1][3], w3_re, w3_im);
    mul_w8<Sign, 1>(d, y_re[2][1], y_im[2][1]);
    mul_w8<Sign, 2>(d, y_re[2][2], y_im[2][2]);
    mul_w8<Sign, 3>(d, y_re[2][3], y_im[2][3]);
    complex_mul(y_re[3][1], y_im[3][1], w3_re, w3_im);
    mul_w8<Sign, 3>(d, y_re[3][2], y_im[3][2]);
    complex_mul(y_re[3][3], y_im[3][3], hn::Neg(w1_re), hn::Neg(w1_im));
    HWY_UNROLL(32)
    for (unsigned c = 0; c != 4; ++c) {
      V z_re[4] = {y_re[0][c], y_re[1][c], y_re[2][c], y_re[3][c]};
      V z_im[4] = {y_im[0][c], y_im[1][c], y_im[2][c], y_im[3][c]};
      radix4<Sign>(z_re, z_im);
      HWY_UNROLL(32)
      for (unsigned q = 0; q != 4; ++q) {
        re[c + 4 * q] = z_re[q];
        im[c + 4 * q] = z_im[q];
      }
    }
  }
}

/// \brief Calls func(k1) for the first outputs k1 of the blocks of L consecutive outputs of N, as compile-time
/// constants.
///
/// The last block overlaps the previous one if N is not a multiple of L. The constants keep the arrays of registers
/// indexed by k1 in registers.
template <unsigned N, unsigned L, unsigned K = 0, class Func>
HWY_INLINE void for_each_block(const Func& func)
{
  if constexpr (K < N) {
    func(std::integral_constant<unsigned, std::min(K, N - L)>());
    for_each_block<N, L, K + L>(func);
  }
}

/// \brief Computes the stage I of a vertical DFT of size M and the following stages, in place.
///
/// The radix, the size and the offsets of the stage are compile-time constants. The last stage passes its outputs to
/// the sink, as sink(e, re, im), instead of storing them.
template <int Sign, unsigned M, unsigned I, class D, class Sink>
HWY_INLINE void vertical_stages(D d, const vertical_dft& dft, float* HWY_RESTRICT scratch, const Sink& sink)
{
  static constexpr radix_plan plan = get_radix_plan(M);
  if constexpr (I < plan.nof_stages) {
    using V                         = hn::Vec<D>;
    static constexpr unsigned L     = hn::MaxLanes(D());
    static constexpr unsigned two_L = 2 * L;
    static constexpr unsigned R     = plan.radix[I];
    static constexpr unsigned n     = [] {
      unsigned size = 1;
      for (unsigned i = 0; i <= I; ++i) {
        size *= plan.radix[i];
      }
      return size;
    }();
    static constexpr unsigned m = n / R;

    const float* HWY_RESTRICT tw = dft.stage_twiddles[I - 1];
    for (unsigned block = 0; block != M; block += n) {
      float* HWY_RESTRICT x = scratch + block * two_L;
      for (unsigned k = 0; k != m; ++k) {
        float* HWY_RESTRICT       xk = x + k * two_L;
        const float* HWY_RESTRICT wk = tw + 2 * (R - 1) * k;
        V                         re[R];
        V                         im[R];
        re[0] = hn::Load(d, xk);
        im[0] = hn::Load(d, xk + L);
        HWY_UNROLL(32)
        for (unsigned r = 1; r != R; ++r) {
          re[r] = hn::Load(d, xk + r * m * two_L);
          im[r] = hn::Load(d, xk + r * m * two_L + L);
          complex_mul(re[r], im[r], hn::Set(d, wk[2 * (r - 1)]), hn::Set(d, wk[2 * (r - 1) + 1]));
        }
        small_dft<Sign, R>(d, re, im);
        HWY_UNROLL(32)
        for (unsigned q = 0; q != R; ++q) {
          if constexpr (I + 1 == plan.nof_stages) {
            sink(block + k + q * m, re[q], im[q]);
          } else {
            hn::Store(re[q], d, xk + q * m * two_L);
            hn::Store(im[q], d, xk + q * m * two_L + L);
          }
        }
      }
    }
    vertical_stages<Sign, M, I + 1>(d, dft, scratch, sink);
  }
}

/// \brief Computes vertical DFTs of size M, one per register lane, in a scratch buffer.
///
/// The point e of the DFTs has its real parts at scratch[2 * e * L] and its imaginary parts at scratch[(2 * e + 1) *
/// L], where L is the number of lanes. The radices of the stages, from the kernels, are given by get_radix_plan(). The
/// size is a template parameter, so the offsets are constants.
///
/// \param[in] load Loads the input t of all the lanes, as load(t, re, im).
/// \param[in] sink Takes the output e of all the lanes, as sink(e, re, im).
template <int Sign, unsigned M, class D, class Load, class Sink>
HWY_INLINE void
vertical_dft_fixed(D d, const vertical_dft& dft, float* HWY_RESTRICT scratch, const Load& load, const Sink& sink)
{
  using V                           = hn::Vec<D>;
  static constexpr unsigned   L     = hn::MaxLanes(D());
  static constexpr unsigned   two_L = 2 * L;
  static constexpr radix_plan plan  = get_radix_plan(M);

  // Kernels from the strided input, written in the output order of the decimation in time.
  static constexpr unsigned R      = plan.radix[0];
  static constexpr unsigned stride = M / R;
  for (unsigned o = 0; o != stride; ++o) {
    V re[R];
    V im[R];
    HWY_UNROLL(32)
    for (unsigned q = 0; q != R; ++q) {
      load(o + q * stride, re[q], im[q]);
    }
    small_dft<Sign, R>(d, re, im);

    float* HWY_RESTRICT x = scratch + dft.kernel_pos[o] * two_L;
    HWY_UNROLL(32)
    for (unsigned q = 0; q != R; ++q) {
      if constexpr (plan.nof_stages == 1) {
        sink(dft.kernel_pos[o] + q, re[q], im[q]);
      } else {
        hn::Store(re[q], d, x + q * two_L);
        hn::Store(im[q], d, x + q * two_L + L);
      }
    }
  }

  // Stages, in place, with compile-time radices and sizes.
  vertical_stages<Sign, M, 1>(d, dft, scratch, sink);
}

/// Computes the kernels of a vertical DFT of any size, with a compile-time radix. The last stage passes its outputs to
/// the sink.
template <int Sign, unsigned R, bool Last, class D, class Load, class Sink>
HWY_INLINE void
generic_kernels(D d, const vertical_dft& dft, float* HWY_RESTRICT scratch, const Load& load, const Sink& sink)
{
  using V                                 = hn::Vec<D>;
  static constexpr unsigned    L          = hn::MaxLanes(D());
  static constexpr unsigned    two_L      = 2 * L;
  const unsigned               stride     = dft.size / R;
  const uint32_t* HWY_RESTRICT kernel_pos = dft.kernel_pos;
  for (unsigned o = 0; o != stride; ++o) {
    V re[R];
    V im[R];
    HWY_UNROLL(32)
    for (unsigned q = 0; q != R; ++q) {
      load(o + q * stride, re[q], im[q]);
    }
    small_dft<Sign, R>(d, re, im);

    unsigned pos = kernel_pos[o];
    HWY_UNROLL(32)
    for (unsigned q = 0; q != R; ++q) {
      if constexpr (Last) {
        sink(pos + q, re[q], im[q]);
      } else {
        hn::Store(re[q], d, scratch + (pos + q) * two_L);
        hn::Store(im[q], d, scratch + (pos + q) * two_L + L);
      }
    }
  }
}

/// Computes a stage of size n of a vertical DFT of size M and any radix R, in place. The last stage passes its outputs
/// to the sink.
template <int Sign, unsigned R, bool Last, class D, class Sink>
HWY_INLINE void
generic_stage(D d, unsigned M, unsigned n, const float* HWY_RESTRICT tw, float* HWY_RESTRICT scratch, const Sink& sink)
{
  using V                         = hn::Vec<D>;
  static constexpr unsigned L     = hn::MaxLanes(D());
  static constexpr unsigned two_L = 2 * L;
  const unsigned            m     = n / R;
  for (unsigned block = 0; block != M; block += n) {
    float* HWY_RESTRICT x = scratch + block * two_L;
    for (unsigned k = 0; k != m; ++k) {
      float* HWY_RESTRICT       xk = x + k * two_L;
      const float* HWY_RESTRICT wk = tw + 2 * (R - 1) * k;
      V                         re[R];
      V                         im[R];
      re[0] = hn::Load(d, xk);
      im[0] = hn::Load(d, xk + L);
      HWY_UNROLL(32)
      for (unsigned r = 1; r != R; ++r) {
        re[r] = hn::Load(d, xk + r * m * two_L);
        im[r] = hn::Load(d, xk + r * m * two_L + L);
        complex_mul(re[r], im[r], hn::Set(d, wk[2 * (r - 1)]), hn::Set(d, wk[2 * (r - 1) + 1]));
      }
      small_dft<Sign, R>(d, re, im);
      HWY_UNROLL(32)
      for (unsigned q = 0; q != R; ++q) {
        if constexpr (Last) {
          sink(block + k + q * m, re[q], im[q]);
        } else {
          hn::Store(re[q], d, xk + q * m * two_L);
          hn::Store(im[q], d, xk + q * m * two_L + L);
        }
      }
    }
  }
}

/// Computes the kernels of a vertical DFT of any size, dispatching the radix to a compile-time constant.
template <int Sign, bool Last, class D, class Load, class Sink>
HWY_INLINE void
generic_kernels_run(D d, const vertical_dft& dft, float* HWY_RESTRICT scratch, const Load& load, const Sink& sink)
{
  switch (dft.plan.radix[0]) {
    case 2:
      return generic_kernels<Sign, 2, Last>(d, dft, scratch, load, sink);
    case 3:
      return generic_kernels<Sign, 3, Last>(d, dft, scratch, load, sink);
    case 4:
      return generic_kernels<Sign, 4, Last>(d, dft, scratch, load, sink);
    case 5:
      return generic_kernels<Sign, 5, Last>(d, dft, scratch, load, sink);
    case 6:
      return generic_kernels<Sign, 6, Last>(d, dft, scratch, load, sink);
    case 8:
      return generic_kernels<Sign, 8, Last>(d, dft, scratch, load, sink);
    case 9:
      return generic_kernels<Sign, 9, Last>(d, dft, scratch, load, sink);
    case 10:
      return generic_kernels<Sign, 10, Last>(d, dft, scratch, load, sink);
    case 12:
      return generic_kernels<Sign, 12, Last>(d, dft, scratch, load, sink);
    case 15:
      return generic_kernels<Sign, 15, Last>(d, dft, scratch, load, sink);
    default:
      return generic_kernels<Sign, 16, Last>(d, dft, scratch, load, sink);
  }
}

/// Computes a stage of a vertical DFT of any size, dispatching the radix to a compile-time constant.
template <int Sign, bool Last, class D, class Sink>
HWY_INLINE void generic_stage_run(D                         d,
                                  unsigned                  R,
                                  unsigned                  M,
                                  unsigned                  n,
                                  const float* HWY_RESTRICT tw,
                                  float* HWY_RESTRICT       scratch,
                                  const Sink&               sink)
{
  switch (R) {
    case 2:
      return generic_stage<Sign, 2, Last>(d, M, n, tw, scratch, sink);
    case 3:
      return generic_stage<Sign, 3, Last>(d, M, n, tw, scratch, sink);
    case 4:
      return generic_stage<Sign, 4, Last>(d, M, n, tw, scratch, sink);
    case 5:
      return generic_stage<Sign, 5, Last>(d, M, n, tw, scratch, sink);
    case 8:
      return generic_stage<Sign, 8, Last>(d, M, n, tw, scratch, sink);
    default:
      return generic_stage<Sign, 16, Last>(d, M, n, tw, scratch, sink);
  }
}

/// \brief Computes vertical DFTs of any size that factors into 2, 3 and 5, with the radices of the plan.
///
/// The layout of the scratch buffer, the load and the sink are those of vertical_dft_fixed().
template <int Sign, class D, class Load, class Sink>
HWY_INLINE void
vertical_dft_generic(D d, const vertical_dft& dft, float* HWY_RESTRICT scratch, const Load& load, const Sink& sink)
{
  const unsigned nof_stages = dft.plan.nof_stages;
  if (nof_stages == 1) {
    generic_kernels_run<Sign, true>(d, dft, scratch, load, sink);
    return;
  }
  generic_kernels_run<Sign, false>(d, dft, scratch, load, sink);

  unsigned n = dft.plan.radix[0];
  for (unsigned i_stage = 1; i_stage != nof_stages; ++i_stage) {
    unsigned                  R  = dft.plan.radix[i_stage];
    const float* HWY_RESTRICT tw = dft.stage_twiddles[i_stage - 1];
    n *= R;
    if (i_stage + 1 == nof_stages) {
      generic_stage_run<Sign, true>(d, R, dft.size, n, tw, scratch, sink);
    } else {
      generic_stage_run<Sign, false>(d, R, dft.size, n, tw, scratch, sink);
    }
  }
}

/// Computes vertical DFTs, dispatching the powers of two to a compile-time size.
template <int Sign, class D, class Load, class Sink>
HWY_INLINE void
vertical_dft_run(D d, const vertical_dft& dft, float* HWY_RESTRICT scratch, const Load& load, const Sink& sink)
{
  if (!dft.fixed) {
    return vertical_dft_generic<Sign>(d, dft, scratch, load, sink);
  }
  switch (dft.size) {
    case 8:
      return vertical_dft_fixed<Sign, 8>(d, dft, scratch, load, sink);
    case 16:
      return vertical_dft_fixed<Sign, 16>(d, dft, scratch, load, sink);
    case 32:
      return vertical_dft_fixed<Sign, 32>(d, dft, scratch, load, sink);
    case 64:
      return vertical_dft_fixed<Sign, 64>(d, dft, scratch, load, sink);
    case 128:
      return vertical_dft_fixed<Sign, 128>(d, dft, scratch, load, sink);
    case 256:
      return vertical_dft_fixed<Sign, 256>(d, dft, scratch, load, sink);
    case 512:
      return vertical_dft_fixed<Sign, 512>(d, dft, scratch, load, sink);
    case 1024:
      return vertical_dft_fixed<Sign, 1024>(d, dft, scratch, load, sink);
    default:
      return vertical_dft_generic<Sign>(d, dft, scratch, load, sink);
  }
}

/// Transposes an 8 by 8 matrix of single precision values.
template <class D>
HWY_INLINE void transpose_8x8(D d, hn::Vec<D>* v)
{
  hn::Repartition<double, D> dd;
  using V = hn::Vec<D>;
  V    t0 = hn::InterleaveLower(d, v[0], v[1]);
  V    t1 = hn::InterleaveUpper(d, v[0], v[1]);
  V    t2 = hn::InterleaveLower(d, v[2], v[3]);
  V    t3 = hn::InterleaveUpper(d, v[2], v[3]);
  V    t4 = hn::InterleaveLower(d, v[4], v[5]);
  V    t5 = hn::InterleaveUpper(d, v[4], v[5]);
  V    t6 = hn::InterleaveLower(d, v[6], v[7]);
  V    t7 = hn::InterleaveUpper(d, v[6], v[7]);
  auto lo = [&](V a, V b) { return hn::BitCast(d, hn::InterleaveLower(dd, hn::BitCast(dd, a), hn::BitCast(dd, b))); };
  auto hi = [&](V a, V b) { return hn::BitCast(d, hn::InterleaveUpper(dd, hn::BitCast(dd, a), hn::BitCast(dd, b))); };
  V    s0 = lo(t0, t2);
  V    s1 = hi(t0, t2);
  V    s2 = lo(t1, t3);
  V    s3 = hi(t1, t3);
  V    s4 = lo(t4, t6);
  V    s5 = hi(t4, t6);
  V    s6 = lo(t5, t7);
  V    s7 = hi(t5, t7);
  v[0]    = hn::ConcatLowerLower(d, s4, s0);
  v[1]    = hn::ConcatLowerLower(d, s5, s1);
  v[2]    = hn::ConcatLowerLower(d, s6, s2);
  v[3]    = hn::ConcatLowerLower(d, s7, s3);
  v[4]    = hn::ConcatUpperUpper(d, s4, s0);
  v[5]    = hn::ConcatUpperUpper(d, s5, s1);
  v[6]    = hn::ConcatUpperUpper(d, s6, s2);
  v[7]    = hn::ConcatUpperUpper(d, s7, s3);
}

/// Transposes a 4 by 4 matrix of single precision values.
template <class D>
HWY_INLINE void transpose_4x4(D d, hn::Vec<D>* v)
{
  hn::Repartition<double, D> dd;
  using V = hn::Vec<D>;
  V t0    = hn::InterleaveLower(d, v[0], v[1]);
  V t1    = hn::InterleaveUpper(d, v[0], v[1]);
  V t2    = hn::InterleaveLower(d, v[2], v[3]);
  V t3    = hn::InterleaveUpper(d, v[2], v[3]);
  v[0]    = hn::BitCast(d, hn::InterleaveLower(dd, hn::BitCast(dd, t0), hn::BitCast(dd, t2)));
  v[1]    = hn::BitCast(d, hn::InterleaveUpper(dd, hn::BitCast(dd, t0), hn::BitCast(dd, t2)));
  v[2]    = hn::BitCast(d, hn::InterleaveLower(dd, hn::BitCast(dd, t1), hn::BitCast(dd, t3)));
  v[3]    = hn::BitCast(d, hn::InterleaveUpper(dd, hn::BitCast(dd, t1), hn::BitCast(dd, t3)));
}

/// Transposes a square matrix with as many rows as lanes, four or eight.
template <class D>
HWY_INLINE void transpose(D d, hn::Vec<D>* v)
{
  if constexpr (hn::MaxLanes(D()) == 8) {
    transpose_8x8(d, v);
  } else {
    static_assert(hn::MaxLanes(D()) == 4, "Unsupported number of lanes.");
    transpose_4x4(d, v);
  }
}

/// \brief Computes the vertical DFTs of pass 1 into the scratch buffer.
///
/// The lane l of the input t is the input sample l + stride * t.
template <int Sign, class D>
HWY_NOINLINE void vertical_dft_input(D                        d,
                                     const vertical_dft&      dft_,
                                     float* HWY_RESTRICT      scratch,
                                     const cf_t* HWY_RESTRICT in,
                                     size_t                   stride)
{
  using V                     = hn::Vec<D>;
  static constexpr unsigned L = hn::MaxLanes(D());
  // The parameters are copied to a local variable because the stores of single precision values may alias them.
  const vertical_dft dft = dft_;
  vertical_dft_run<Sign>(
      d,
      dft,
      scratch,
      [&](unsigned t, V& re, V& im) {
        hn::LoadInterleaved2(d, reinterpret_cast<const float*>(in + t * stride), re, im);
      },
      [&](unsigned e, V re, V im) {
        hn::Store(re, d, scratch + 2 * e * L);
        hn::Store(im, d, scratch + (2 * e + 1) * L);
      });
}

/// \brief Loads the vertical DFT outputs k1 to k1 + L - 1 of pass 1 from the scratch buffer and multiplies them by the
/// twiddle factors.
///
/// The twiddle factors of the output k are at tw_re[k * L] and tw_im[k * L], for the L lanes.
template <class D>
HWY_INLINE void load_twiddled(D                         d,
                              const float* HWY_RESTRICT scratch,
                              const float* HWY_RESTRICT tw_re,
                              const float* HWY_RESTRICT tw_im,
                              unsigned                  k1,
                              hn::Vec<D>*               re,
                              hn::Vec<D>*               im)
{
  static constexpr unsigned L = hn::MaxLanes(D());
  HWY_UNROLL(32)
  for (unsigned i = 0; i != L; ++i) {
    unsigned k = k1 + i;
    re[i]      = hn::Load(d, scratch + 2 * k * L);
    im[i]      = hn::Load(d, scratch + (2 * k + 1) * L);
    complex_mul(re[i], im[i], hn::Load(d, tw_re + k * L), hn::Load(d, tw_im + k * L));
  }
}

/// \brief Pass 1: DFTs of size N1, twiddle factors and transpose into the work buffer.
///
/// The tag D has eight lanes on x86 and four lanes on Arm, or four lanes on any target, that is, as many DFTs at a
/// time. The x86 targets with 512-bit vectors also use 256-bit vectors, because the scratch buffer of sixteen DFTs of
/// size 256 fills the first level cache.
template <int Sign, class D>
HWY_NOINLINE void pass1(D d, const pass_params& p, const cf_t* HWY_RESTRICT in)
{
  using V                     = hn::Vec<D>;
  static constexpr unsigned L = hn::MaxLanes(D());
  // The parameters are copied to local variables because the stores of single precision values may alias them.
  const unsigned            N1          = p.N1;
  const unsigned            N2          = p.N2;
  const size_t              work_stride = p.work_stride;
  float* HWY_RESTRICT       scratch     = p.scratch;
  const float* HWY_RESTRICT twiddle_re  = p.twiddle_re;
  const float* HWY_RESTRICT twiddle_im  = p.twiddle_im;
  float* HWY_RESTRICT       work_re0    = p.work_re;
  float* HWY_RESTRICT       work_im0    = p.work_im;
  const unsigned            nof_groups  = (N2 + L - 1) / L;

  for (unsigned i_group = 0; i_group != nof_groups; ++i_group) {
    // DFTs of the lanes g to g + L - 1. The last group overlaps the previous one if N2 is not a multiple of L.
    unsigned g = std::min(i_group * L, N2 - L);
    vertical_dft_input<Sign>(d, p.dft1, scratch, in + g, N2);

    // Twiddle factors W_N^(n2 * k) of the lanes, for consecutive k.
    const float* HWY_RESTRICT tw_re   = twiddle_re + i_group * L * N1;
    const float* HWY_RESTRICT tw_im   = twiddle_im + i_group * L * N1;
    float* HWY_RESTRICT       work_re = work_re0 + g * work_stride;
    float* HWY_RESTRICT       work_im = work_im0 + g * work_stride;

    for (unsigned k1_block = 0; k1_block < N1; k1_block += L) {
      unsigned k1 = std::min(k1_block, N1 - L);
      V        re[L];
      V        im[L];
      load_twiddled(d, scratch, tw_re, tw_im, k1, re, im);

      // After the transposes, each register holds L consecutive k1 of one n2.
      transpose(d, re);
      transpose(d, im);
      HWY_UNROLL(32)
      for (unsigned j = 0; j != L; ++j) {
        hn::StoreU(re[j], d, work_re + j * work_stride + k1);
        hn::StoreU(im[j], d, work_im + j * work_stride + k1);
      }
    }
  }
}

#if HWY_ARCH_X86 && (HWY_TARGET != HWY_EMU128)
/// \brief Rounds to the nearest integer, with ties to even.
///
/// Unlike hn::NearestInt, it does not correct the overflow: the values out of range give the most negative integer.
HWY_INLINE hn::Vec128<int32_t> round_unchecked(hn::Vec128<float> v)
{
  return hn::Vec128<int32_t>{_mm_cvtps_epi32(v.raw)};
}

#if HWY_TARGET <= HWY_AVX2
/// \copydoc round_unchecked(hn::Vec128<float>)
HWY_INLINE hn::Vec256<int32_t> round_unchecked(hn::Vec256<float> v)
{
  return hn::Vec256<int32_t>{_mm256_cvtps_epi32(v.raw)};
}
#endif // HWY_TARGET <= HWY_AVX2

#if HWY_TARGET <= HWY_AVX3
/// \copydoc round_unchecked(hn::Vec128<float>)
HWY_INLINE hn::Vec512<int32_t> round_unchecked(hn::Vec512<float> v)
{
  return hn::Vec512<int32_t>{_mm512_cvtps_epi32(v.raw)};
}
#endif // HWY_TARGET <= HWY_AVX3
#else  // HWY_ARCH_X86 && (HWY_TARGET != HWY_EMU128)
/// \brief Rounds to the nearest integer, with ties to even.
///
/// The values out of range saturate.
template <class V>
HWY_INLINE auto round_unchecked(V v)
{
  return hn::NearestInt(v);
}
#endif // HWY_ARCH_X86 && (HWY_TARGET != HWY_EMU128)

/// Writes consecutive single precision output samples.
template <class D>
HWY_INLINE void write_output(D d, hn::Vec<D> re, hn::Vec<D> im, cf_t* HWY_RESTRICT out)
{
  hn::StoreInterleaved2(re, im, d, reinterpret_cast<float*>(out));
}

/// \brief Writes consecutive output samples rounded to 16-bit integers.
///
/// It rounds once and saturates. The rounding of the values above the range gives the most negative integer, so only
/// the upper bound needs clamping. The values below the range saturate in the pack.
template <class D>
HWY_INLINE void write_output(D d, hn::Vec<D> re, hn::Vec<D> im, ci16_t* HWY_RESTRICT out)
{
  hn::Repartition<int16_t, D> d16;
  hn::Repartition<uint8_t, D> du8;
  const auto                  upper = hn::Set(d, 32767.0F);
  // Interleaves, within each 128-bit block, the four real parts and the four imaginary parts of the saturating pack.
  alignas(16) static constexpr uint8_t interleave_bytes[16] = {0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15};
  const auto                           interleave           = hn::LoadDup128(du8, interleave_bytes);

  auto r      = round_unchecked(hn::Min(re, upper));
  auto i      = round_unchecked(hn::Min(im, upper));
  auto packed = hn::BitCast(du8, hn::ReorderDemote2To(d16, r, i));
  hn::StoreU(hn::BitCast(d16, hn::TableLookupBytes(packed, interleave)), d16, reinterpret_cast<int16_t*>(out));
}

/// \brief Pass 2: DFTs of size N2 over the rows of the work buffer and output, in single precision or 16-bit integers.
///
/// The tag D has up to the widest vector of the target. The last group of lanes overlaps the previous one if N1 is not
/// a multiple of the lanes.
template <int Sign, class D, typename T>
HWY_NOINLINE void pass2(D d, const pass_params& p, T* HWY_RESTRICT out)
{
  using V                     = hn::Vec<D>;
  static constexpr unsigned L = hn::MaxLanes(D());
  // The parameters are copied to local variables because the stores of single precision values may alias them.
  const unsigned            N1          = p.N1;
  const size_t              work_stride = p.work_stride;
  const float* HWY_RESTRICT work_re     = p.work_re;
  const float* HWY_RESTRICT work_im     = p.work_im;
  const vertical_dft        dft         = p.dft2;
  float* HWY_RESTRICT       scratch     = p.scratch;

  for (unsigned k1_block = 0; k1_block < N1; k1_block += L) {
    unsigned k1 = std::min(k1_block, N1 - L);
    vertical_dft_run<Sign>(
        d,
        dft,
        scratch,
        [&](unsigned n2, V& re, V& im) {
          re = hn::LoadU(d, work_re + n2 * work_stride + k1);
          im = hn::LoadU(d, work_im + n2 * work_stride + k1);
        },
        [&](unsigned k2, V re, V im) { write_output(d, re, im, out + k1 + N1 * k2); });
  }
}

/// Computes the DFT in two passes, with L1 lanes in pass 1 and L2 lanes in pass 2.
template <int Sign, unsigned L1, unsigned L2, typename T>
void run_two_pass(const pass_params& p, T* HWY_RESTRICT out, const cf_t* HWY_RESTRICT in)
{
  pass1<Sign>(hn::CappedTag<float, L1>(), p, in);
  pass2<Sign>(hn::CappedTag<float, L2>(), p, out);
}

/// \brief Computes the fused DFT, in a single pass.
///
/// Pass 1 computes the N2 DFTs of size N1 in a single group of lanes, the lanes from N2 compute values that are not
/// used. After the twiddle factors and the transposes of L consecutive k1, the DFTs of size N2 run in registers.
template <int Sign, unsigned N2, class D, typename T>
HWY_NOINLINE void run_fused(const pass_params& p, T* HWY_RESTRICT out, const cf_t* HWY_RESTRICT in)
{
  D d;
  using V                     = hn::Vec<D>;
  static constexpr unsigned L = hn::MaxLanes(D());
  static_assert(N2 <= L, "The fused DFT needs the DFTs of pass 1 in a single group of lanes.");
  const unsigned            N1         = p.N1;
  float* HWY_RESTRICT       scratch    = p.scratch;
  const float* HWY_RESTRICT twiddle_re = p.twiddle_re;
  const float* HWY_RESTRICT twiddle_im = p.twiddle_im;

  vertical_dft_input<Sign>(d, p.dft1, scratch, in, N2);

  for (unsigned k1_block = 0; k1_block < N1; k1_block += L) {
    unsigned k1 = std::min(k1_block, N1 - L);
    V        re[L];
    V        im[L];
    load_twiddled(d, scratch, twiddle_re, twiddle_im, k1, re, im);
    transpose(d, re);
    transpose(d, im);
    small_dft<Sign, N2>(d, re, im);
    HWY_UNROLL(32)
    for (unsigned k2 = 0; k2 != N2; ++k2) {
      write_output(d, re[k2], im[k2], out + k1 + N1 * k2);
    }
  }
}

/// \brief Computes the fused DFT of a compile-time size N1 that is a kernel radix, in registers.
///
/// The arithmetic is the same as run_fused(), without the scratch buffer.
template <int Sign, unsigned N1, unsigned N2, class D, typename T>
HWY_NOINLINE void run_fused_registers(const pass_params& p, T* HWY_RESTRICT out, const cf_t* HWY_RESTRICT in)
{
  D d;
  using V                     = hn::Vec<D>;
  static constexpr unsigned L = hn::MaxLanes(D());
  static_assert((N2 <= L) && (N1 >= L), "Invalid fused DFT sizes.");
  const float* HWY_RESTRICT twiddle_re = p.twiddle_re;
  const float* HWY_RESTRICT twiddle_im = p.twiddle_im;

  V x_re[N1];
  V x_im[N1];
  HWY_UNROLL(32)
  for (unsigned t = 0; t != N1; ++t) {
    hn::LoadInterleaved2(d, reinterpret_cast<const float*>(in + t * N2), x_re[t], x_im[t]);
  }
  small_dft<Sign, N1>(d, x_re, x_im);

  for_each_block<N1, L>([&](auto k1_constant) {
    static constexpr unsigned k1 = decltype(k1_constant)::value;
    V                         re[L];
    V                         im[L];
    HWY_UNROLL(32)
    for (unsigned i = 0; i != L; ++i) {
      re[i] = x_re[k1 + i];
      im[i] = x_im[k1 + i];
      complex_mul(re[i], im[i], hn::Load(d, twiddle_re + (k1 + i) * L), hn::Load(d, twiddle_im + (k1 + i) * L));
    }
    transpose(d, re);
    transpose(d, im);
    small_dft<Sign, N2>(d, re, im);
    HWY_UNROLL(32)
    for (unsigned k2 = 0; k2 != N2; ++k2) {
      write_output(d, re[k2], im[k2], out + k1 + N1 * k2);
    }
  });
}

/// Selects the fused DFT in registers of the sizes N1 and N2, with L lanes.
template <int Sign, unsigned L, unsigned N1, typename T>
run_function<T> select_fused_registers(unsigned N2)
{
  using D = hn::CappedTag<float, L>;
  // Only the sizes that the plan selection reaches are compiled, the others use run_fused().
  if constexpr (N1 >= hn::MaxLanes(D())) {
    if constexpr (hn::MaxLanes(D()) >= 8) {
      if (N2 == 8) {
        return &run_fused_registers<Sign, N1, 8, D, T>;
      }
      if constexpr ((N1 == 8) || (N1 == 9) || (N1 == 10) || (N1 == 15) || (N1 == 18) || (N1 == 30)) {
        if (N2 == 6) {
          return &run_fused_registers<Sign, N1, 6, D, T>;
        }
      }
    } else if constexpr (N1 <= 9) {
      switch (N2) {
        case 3:
          return &run_fused_registers<Sign, N1, 3, D, T>;
        case 4:
          return &run_fused_registers<Sign, N1, 4, D, T>;
        default:
          break;
      }
    }
  }
  return nullptr;
}

/// Selects the fused DFT of the sizes N1 and N2, with L lanes.
template <int Sign, unsigned L, typename T>
run_function<T> select_fused(unsigned N1, unsigned N2)
{
  using D                   = hn::CappedTag<float, L>;
  run_function<T> registers = nullptr;
  switch (N1) {
    case 4:
      registers = select_fused_registers<Sign, L, 4, T>(N2);
      break;
    case 5:
      registers = select_fused_registers<Sign, L, 5, T>(N2);
      break;
    case 6:
      registers = select_fused_registers<Sign, L, 6, T>(N2);
      break;
    case 8:
      registers = select_fused_registers<Sign, L, 8, T>(N2);
      break;
    case 9:
      registers = select_fused_registers<Sign, L, 9, T>(N2);
      break;
    case 10:
      registers = select_fused_registers<Sign, L, 10, T>(N2);
      break;
    case 12:
      registers = select_fused_registers<Sign, L, 12, T>(N2);
      break;
    case 15:
      registers = select_fused_registers<Sign, L, 15, T>(N2);
      break;
    case 16:
      registers = select_fused_registers<Sign, L, 16, T>(N2);
      break;
    case 18:
      registers = select_fused_registers<Sign, L, 18, T>(N2);
      break;
    case 24:
      registers = select_fused_registers<Sign, L, 24, T>(N2);
      break;
    case 30:
      registers = select_fused_registers<Sign, L, 30, T>(N2);
      break;
    case 32:
      registers = select_fused_registers<Sign, L, 32, T>(N2);
      break;
    case 48:
      registers = select_fused_registers<Sign, L, 48, T>(N2);
      break;

    default:
      break;
  }
  if (registers != nullptr) {
    return registers;
  }
  switch (N2) {
    case 2:
      return &run_fused<Sign, 2, D, T>;
    case 3:
      return &run_fused<Sign, 3, D, T>;
    case 4:
      return &run_fused<Sign, 4, D, T>;
    default:
      break;
  }
  if constexpr (hn::MaxLanes(D()) >= 8) {
    switch (N2) {
      case 5:
        return &run_fused<Sign, 5, D, T>;
      case 6:
        return &run_fused<Sign, 6, D, T>;
      case 8:
        return &run_fused<Sign, 8, D, T>;
      default:
        break;
    }
  }
  return nullptr;
}

/// Selects the DFT function of the parameters, for a direction sign.
template <int Sign, typename T>
run_function<T> select_run_sign(const pass_params& p)
{
  // The fused DFT does not have a work buffer.
  if (p.work_re == nullptr) {
    return (p.L1 == 4) ? select_fused<Sign, 4, T>(p.N1, p.N2) : select_fused<Sign, 8, T>(p.N1, p.N2);
  }
  // Pass 2 uses the widest vector of the target.
  static constexpr unsigned max_lanes = hn::MaxLanes(hn::ScalableTag<float>());
  if constexpr (max_lanes < 8) {
    return &run_two_pass<Sign, 4, 4, T>;
  } else {
    return (p.L1 == 4) ? &run_two_pass<Sign, 4, max_lanes, T> : &run_two_pass<Sign, 8, max_lanes, T>;
  }
}

/// Selects the DFT function of the parameters.
template <typename T>
run_function<T> select_run(const pass_params& p)
{
  return (p.sign > 0) ? select_run_sign<1, T>(p) : select_run_sign<-1, T>(p);
}

/// Gets the entry points of the passes of the target.
target_kernels get_target_kernels()
{
  return {HWY_TARGET,
          &select_run<cf_t>,
          &select_run<ci16_t>,
          static_cast<unsigned>(hn::MaxLanes(hn::ScalableTag<float>()))};
}

} // namespace HWY_NAMESPACE
} // namespace dft_hwy
} // namespace ocudu
HWY_AFTER_NAMESPACE();

#pragma GCC diagnostic pop

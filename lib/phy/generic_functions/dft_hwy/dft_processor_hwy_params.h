// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Parameters shared by the target-specific passes of the Highway DFT.
///
/// The DFT of size N = N1 * N2 runs in two passes, in single precision floating point:
/// - Pass 1 computes the N2 DFTs of size N1 on the input samples n2 + N2 * n1, L1 at a time with one DFT per register
///   lane. It multiplies the outputs by the twiddle factors W_N^(n2 * k1) and writes them, transposed, to a work
///   buffer as rows of N1 values, one row per n2.
/// - Pass 2 computes the N1 DFTs of size N2 over the rows, for L2 consecutive k1 in the register lanes, and writes the
///   output samples k1 + N1 * k2, in single precision or rounded once to 16-bit integers.
///
/// When N2 or N1 is not a multiple of the lanes, the last group of lanes overlaps the previous one, and computes some
/// DFTs twice with the same result.
///
/// The fused DFT is the case N2 <= L1: pass 1 has a single group of lanes, and pass 2 computes its DFTs of size N2 in
/// registers, right after the transposes, without the work buffer.
///
/// The DFTs of both passes, named vertical DFTs, are decimation-in-time DFTs of sizes that factor into 2, 3 and 5.
///
/// \note Disclaimer: this file was generated with Claude Code, the agentic coding tool from Anthropic, using the
/// Claude Opus 5.5 model (claude-opus-5-5).

#pragma once

#include "ocudu/adt/complex.h"
#include <cstdint>

namespace ocudu {
namespace dft_hwy {

/// Largest number of stages of a vertical DFT, including the kernels.
constexpr unsigned max_nof_stages = 16;

/// Number of complex values that the input buffer has after the DFT size, which pass 1 may read but not use.
constexpr unsigned input_padding = 16;

/// Radices of the stages of a vertical DFT, from the kernels to the top stage.
struct radix_plan {
  /// Number of stages, including the kernels.
  unsigned nof_stages;
  /// Radix of each stage.
  unsigned radix[max_nof_stages];
};

/// \brief Gets the radices of the stages of a vertical DFT of size M, a power of two from 8 to 1024.
///
/// The stages compute their butterflies in registers, so the plans prefer large radices to read and write the data
/// fewer times.
constexpr radix_plan get_radix_plan(unsigned M)
{
  switch (M) {
    case 8:
      return {1, {8}};
    case 16:
      return {1, {16}};
    case 32:
      return {2, {8, 4}};
    case 64:
      return {2, {8, 8}};
    case 128:
      return {2, {8, 16}};
    case 256:
      return {3, {8, 8, 4}};
    case 512:
      return {3, {8, 8, 8}};
    default:
      return {3, {16, 16, 4}};
  }
}

/// Returns true if the vertical DFTs of size M use the stages of get_radix_plan(), with compile-time sizes.
constexpr bool is_fixed_vertical_size(unsigned M)
{
  return (M >= 8) && (M <= 1024) && ((M & (M - 1)) == 0);
}

/// Returns true if a DFT of size R is computed in registers, as a kernel of a vertical DFT.
constexpr bool is_kernel_radix(unsigned R)
{
  switch (R) {
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 8:
    case 9:
    case 10:
    case 12:
    case 15:
    case 16:
      return true;
    default:
      return false;
  }
}

/// \brief Gets the radices of the stages of a vertical DFT of any size M that factors into 2, 3 and 5.
///
/// The powers of two from 8 to 1024 use get_radix_plan(). The other sizes up to 16 are a single kernel. Larger sizes
/// have radix-16, radix-8 or radix-4 stages for the power of two, the first of them as the kernel, then radix-5 and
/// radix-3 stages. A lone factor of two joins an odd factor in a radix-6 or radix-10 kernel.
/// \return The plan, or a plan without stages if M does not factor into 2, 3 and 5.
constexpr radix_plan make_radix_plan(unsigned M)
{
  if (is_fixed_vertical_size(M)) {
    return get_radix_plan(M);
  }
  radix_plan plan = {};
  if (M < 2) {
    return plan;
  }
  if ((M <= 16) && is_kernel_radix(M)) {
    plan.nof_stages = 1;
    plan.radix[0]   = M;
    return plan;
  }

  unsigned log2 = 0;
  unsigned pow3 = 0;
  unsigned pow5 = 0;
  unsigned rest = M;
  for (; rest % 2 == 0; rest /= 2) {
    ++log2;
  }
  for (; rest % 3 == 0; rest /= 3) {
    ++pow3;
  }
  for (; rest % 5 == 0; rest /= 5) {
    ++pow5;
  }
  if (rest != 1) {
    return plan;
  }

  unsigned nof_stages = 0;
  auto     push       = [&plan, &nof_stages](unsigned radix) {
    if (nof_stages < max_nof_stages) {
      plan.radix[nof_stages] = radix;
    }
    ++nof_stages;
  };

  // Splits the power of two in radices 16, and a remainder of 4 or 8, or 8 and 4 instead of 32.
  unsigned pow2_radices[max_nof_stages] = {};
  unsigned nof_pow2_radices             = 0;
  if (log2 >= 2) {
    unsigned remainder = log2 % 4;
    unsigned nof_16    = log2 / 4;
    if (remainder == 1) {
      // Replaces a radix 16 and the remaining 2 by 8 and 4.
      --nof_16;
      pow2_radices[nof_pow2_radices++] = 8;
      pow2_radices[nof_pow2_radices++] = 4;
    } else if (remainder != 0) {
      pow2_radices[nof_pow2_radices++] = 1U << remainder;
    }
    for (unsigned i = 0; i != nof_16; ++i) {
      pow2_radices[nof_pow2_radices++] = 16;
    }
  }

  // Kernel: the first power of two, if any. A lone factor of two joins an odd factor.
  if (log2 == 1) {
    if (pow5 > 0) {
      push(10);
      --pow5;
    } else {
      push(6);
      --pow3;
    }
  } else if (nof_pow2_radices == 0) {
    if ((pow5 > 0) && (pow3 > 0)) {
      push(15);
      --pow5;
      --pow3;
    } else if (pow5 > 0) {
      push(5);
      --pow5;
    } else {
      push((pow3 >= 2) ? 9 : 3);
      pow3 -= (pow3 >= 2) ? 2 : 1;
    }
  }
  for (unsigned i = 0; i != nof_pow2_radices; ++i) {
    push(pow2_radices[i]);
  }
  for (; pow5 > 0; --pow5) {
    push(5);
  }
  for (; pow3 > 0; --pow3) {
    push(3);
  }

  if (nof_stages > max_nof_stages) {
    return {};
  }
  plan.nof_stages = nof_stages;
  return plan;
}

/// \brief Tables of a decimation-in-time DFT, for vertical butterflies.
///
/// Each butterfly k of a stage of radix R and size n has the real and imaginary parts of W_n^(r * k) for the inputs
/// r = 1 to R - 1.
struct vertical_dft {
  /// DFT size.
  unsigned size;
  /// Position of the kernels in the output order, indexed by their input offset.
  const uint32_t* kernel_pos;
  /// Number of stages after the kernels.
  unsigned nof_stages;
  /// Twiddle factors of each stage.
  const float* const* stage_twiddles;
  /// Radices of the stages.
  radix_plan plan;
  /// True if the stages are those of get_radix_plan(), with compile-time sizes.
  bool fixed;
};

/// Parameters of the two passes.
struct pass_params {
  /// DFT direction sign.
  int sign;
  /// Size of the DFTs of pass 1.
  unsigned N1;
  /// Size of the DFTs of pass 2, which is also the number of DFTs of pass 1.
  unsigned N2;
  /// Number of lanes of pass 1.
  unsigned L1;
  /// Number of lanes of pass 2.
  unsigned L2;
  /// DFTs of pass 1.
  vertical_dft dft1;
  /// DFTs of pass 2.
  vertical_dft dft2;
  /// \brief Real parts of the twiddle factors of pass 1, W_N^(n2 * k1).
  ///
  /// For the group of lanes g, the factor of lane l is at g * L1 * N1 + k1 * L1 + l, so pass 1 reads the table
  /// sequentially.
  const float* twiddle_re;
  /// Imaginary parts of the twiddle factors of pass 1.
  const float* twiddle_im;
  /// Distance between the rows of the work buffer, in values.
  unsigned work_stride;
  /// Real parts of the work buffer, N2 rows of N1 values.
  float* work_re;
  /// Imaginary parts of the work buffer, N2 rows of N1 values.
  float* work_im;
  /// Scratch memory for the vertical DFTs, aligned to 64 bytes.
  float* scratch;
};

/// DFT function of a Highway target, with the output type T.
template <typename T>
using run_function = void (*)(const pass_params&, T*, const cf_t*);

/// Entry points of the passes of a Highway target.
struct target_kernels {
  /// Highway target, or zero if the source file does not compile any.
  int64_t target;
  /// Selects the DFT function with a single precision output.
  run_function<cf_t> (*select_cf)(const pass_params& p);
  /// Selects the DFT function with a 16-bit integer output.
  run_function<ci16_t> (*select_ci16)(const pass_params& p);
  /// Maximum number of single precision lanes of pass 2.
  unsigned max_lanes;
};

/// Gets the passes for AVX2, from dft_processor_hwy_avx2.cpp.
target_kernels get_target_kernels_avx2();

/// Gets the passes for AVX-512, from dft_processor_hwy_avx512.cpp.
target_kernels get_target_kernels_avx512();

/// \brief Gets the passes for the native target, the target of the project compiler options.
///
/// They are the passes of the static target compiled with the project compiler options, from
/// dft_processor_hwy_native.cpp, or the passes of the source file that compiles the same Highway target. On Arm, the
/// static targets without fixed size vectors of four or more lanes use the NEON passes. On x86, they have no passes.
target_kernels get_target_kernels_native();

/// Gets the passes for NEON, from dft_processor_hwy_neon.cpp.
target_kernels get_target_kernels_neon();

/// Computes the cosine of 2 * pi * m / n at compile time.
constexpr double constexpr_cos_2pi(unsigned m, unsigned n)
{
  constexpr double pi = 3.14159265358979323846;
  double           x  = 2.0 * pi * static_cast<double>(m % n) / static_cast<double>(n);
  if (x > pi) {
    x -= 2.0 * pi;
  }
  double term = 1.0;
  double sum  = 1.0;
  for (int i = 1; i != 40; ++i) {
    term *= -x * x / static_cast<double>((2 * i - 1) * (2 * i));
    sum += term;
  }
  return sum;
}

/// Computes the sine of 2 * pi * m / n at compile time.
constexpr double constexpr_sin_2pi(unsigned m, unsigned n)
{
  return constexpr_cos_2pi(4 * m + 3 * n, 4 * n);
}

} // namespace dft_hwy
} // namespace ocudu

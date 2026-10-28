// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Passes of the Highway DFT for the native target, the static target of the project compiler options.
///
/// The file compiles the passes of the static target with the project compiler options. It returns the passes of
/// another source file instead if:
/// - the other source file compiles the same Highway target, since two definitions of the same functions with
///   different compiler options break the one definition rule; or
/// - the static target does not support the DFT, which needs fixed size vectors of four or more lanes. On Arm, the
///   passes are then the NEON ones, which every 64-bit Arm processor supports.
///
/// \note Disclaimer: this file was generated with Claude Code, the agentic coding tool from Anthropic, using the
/// Claude Opus 5.5 model (claude-opus-5-5).

// Highway selects the SSE4 and later static targets only with AES and PCLMUL, which the passes do not use.
#define HWY_DISABLE_PCLMUL_AES

#include "dft_processor_hwy_params.h"
#include "hwy/highway.h"

// The file compiles the passes unless the target has its own source file (AVX2, AVX3 and NEON without AES), or does
// not support the DFT (the scalar target has one lane, and the scalable vectors cannot be array elements).
#if (HWY_TARGET != HWY_AVX2) && (HWY_TARGET != HWY_AVX3) && (HWY_TARGET != HWY_NEON_WITHOUT_AES) &&                    \
    (HWY_TARGET != HWY_SCALAR) && !HWY_HAVE_SCALABLE
#define OCUDU_DFT_HWY_NATIVE_PASSES
#include "dft_processor_hwy_kernels.h"
#endif

ocudu::dft_hwy::target_kernels ocudu::dft_hwy::get_target_kernels_native()
{
#if defined(OCUDU_DFT_HWY_NATIVE_PASSES)
  return HWY_NAMESPACE::get_target_kernels();
#elif HWY_ARCH_X86 && (HWY_TARGET == HWY_AVX3)
  return get_target_kernels_avx512();
#elif HWY_ARCH_X86 && (HWY_TARGET == HWY_AVX2)
  return get_target_kernels_avx2();
#elif HWY_ARCH_ARM_A64
  return get_target_kernels_neon();
#else
  return {};
#endif
}

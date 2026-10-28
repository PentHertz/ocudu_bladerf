// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Passes of the Highway DFT for AVX2.
///
/// The file selects the Highway target HWY_AVX2 in place of the static target of the compiler options. The compiler
/// options have the x86-64 baseline, so the code that the linker keeps from this file runs on any x86 processor.
///
/// \note Disclaimer: this file was generated with Claude Code, the agentic coding tool from Anthropic, using the
/// Claude Opus 5.5 model (claude-opus-5-5).

#include "hwy/detect_targets.h"

#undef HWY_TARGET
#define HWY_TARGET HWY_AVX2

#include "dft_processor_hwy_kernels.h"

ocudu::dft_hwy::target_kernels ocudu::dft_hwy::get_target_kernels_avx2()
{
  return N_AVX2::get_target_kernels();
}

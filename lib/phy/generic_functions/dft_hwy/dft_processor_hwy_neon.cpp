// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Passes of the Highway DFT for NEON.
///
/// The file selects the Highway target HWY_NEON_WITHOUT_AES in place of the static target of the compiler options. The
/// DFT does not use the AES instructions, so the target without them runs on every Armv8-A processor.
///
/// \note Disclaimer: this file was generated with Claude Code, the agentic coding tool from Anthropic, using the
/// Claude Opus 5.5 model (claude-opus-5-5).

#include "hwy/detect_targets.h"

#undef HWY_TARGET
#define HWY_TARGET HWY_NEON_WITHOUT_AES

#include "dft_processor_hwy_kernels.h"

ocudu::dft_hwy::target_kernels ocudu::dft_hwy::get_target_kernels_neon()
{
  return N_NEON_WITHOUT_AES::get_target_kernels();
}

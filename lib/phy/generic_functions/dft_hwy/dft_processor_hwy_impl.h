// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief DFT processor based on Google Highway: architecture selection and creation.
///
/// \note Disclaimer: this file was generated with Claude Code, the agentic coding tool from Anthropic, using the
/// Claude Opus 5.5 model (claude-opus-5-5).

#pragma once

#include "ocudu/phy/generic_functions/dft_processor.h"
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ocudu {

namespace dft_hwy {

/// \brief Plan of a Highway DFT of size N = N1 * N2.
///
/// See dft_processor_hwy_params.h for the algorithm.
struct dft_processor_hwy_plan {
  /// True for the fused DFT, false for the two passes.
  bool fused;
  /// Size of the DFTs of pass 1.
  unsigned N1;
  /// Size of the DFTs of pass 2.
  unsigned N2;
  /// Number of lanes of pass 1, four or eight.
  unsigned L1;
};

/// \brief Gets all the plans of a DFT size.
/// \param[in] size      DFT size.
/// \param[in] max_lanes Maximum number of single precision lanes of the target.
/// \return The plans, or none if the DFT size is not supported.
std::vector<dft_processor_hwy_plan> get_dft_processor_hwy_plans(unsigned size, unsigned max_lanes);

/// \brief Selects the plan of a DFT size.
///
/// The plan is the same for all the targets with at least eight lanes, so that they give the same results.
/// \param[in] size      DFT size.
/// \param[in] max_lanes Maximum number of single precision lanes of the target.
/// \return The plan, or none if the DFT size is not supported.
std::optional<dft_processor_hwy_plan> select_dft_processor_hwy_plan(unsigned size, unsigned max_lanes);

} // namespace dft_hwy

/// \brief Resolves the Highway target of an architecture name.
///
/// On x86, avx2 and avx512 select AVX2 and AVX3. On Arm, neon selects NEON. The name native selects the target of the
/// project compiler options, or the best one that it supports, and auto selects the best target that the CPU supports.
///
/// \param[in] arch Architecture name.
/// \return The Highway target, or zero if the name is unknown, or the target is not compiled or not supported by the
/// CPU.
int64_t resolve_dft_processor_hwy_target(const std::string& arch);

/// \brief Gets the maximum number of single precision lanes of a Highway target.
/// \return The number of lanes, or zero if the target is not available.
unsigned get_dft_processor_hwy_max_lanes(int64_t target);

/// \brief Creates a DFT processor based on Google Highway, for a Highway target.
///
/// It computes in single precision floating point. The 16-bit integer output is rounded once.
///
/// \param[in] config DFT configuration.
/// \param[in] target Highway target, as given by resolve_dft_processor_hwy_target().
/// \return A DFT processor if the DFT size factors into 2, 3 and 5, and the target is available, otherwise nullptr.
std::unique_ptr<dft_processor> create_dft_processor_hwy(const dft_processor::configuration& config, int64_t target);

/// \brief Creates a DFT processor based on Google Highway, for a Highway target and a plan.
/// \param[in] config DFT configuration.
/// \param[in] target Highway target, as given by resolve_dft_processor_hwy_target().
/// \param[in] plan   One of the plans of get_dft_processor_hwy_plans() for the DFT size and the target lanes.
/// \return A DFT processor if the target is available, otherwise nullptr.
std::unique_ptr<dft_processor> create_dft_processor_hwy(const dft_processor::configuration&    config,
                                                        int64_t                                target,
                                                        const dft_hwy::dft_processor_hwy_plan& plan);

/// \brief Gets the architecture names that the CPU supports, separated by commas.
///
/// The automatic selection, auto, is not listed.
std::string list_dft_processor_hwy_architectures();

} // namespace ocudu

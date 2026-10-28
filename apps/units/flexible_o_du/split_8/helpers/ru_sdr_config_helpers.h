// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ru_sdr_config_validator.h"
#include "ocudu/ran/band_helper.h"
#include "ocudu/ran/bs_channel_bandwidth.h"
#include "ocudu/ran/phy_time_unit.h"
#include "ocudu/ran/resource_block.h"
#include "ocudu/ran/subcarrier_spacing.h"
#include <array>
#include <cmath>
#include <optional>

namespace ocudu {

/// Common SDR sampling rates in MHz, in ascending order.
inline constexpr std::array<double, 10> ru_sdr_standard_srates_MHz =
    {7.68, 11.52, 15.36, 23.04, 30.72, 46.08, 61.44, 92.16, 122.88, 184.32};

/// Shortest NR time unit that must be an integer number of samples.
inline constexpr phy_time_unit ru_sdr_srate_reference_time = phy_time_unit::from_units_of_kappa(16);

/// Converts a sampling rate in MHz to an integer number of hertz.
inline int64_t convert_srate_MHz_to_Hz(double srate_MHz)
{
  return std::llround(srate_MHz * 1e6);
}

/// Returns true if the reference time is an integer number of samples at the given sampling rate.
inline bool is_srate_sample_accurate(double srate_MHz)
{
  return ru_sdr_srate_reference_time.is_sample_accurate(convert_srate_MHz_to_Hz(srate_MHz));
}

/// Returns true if the PRACH cyclic prefix is an integer number of samples at the given sampling rate.
inline bool is_srate_prach_sample_accurate(double srate_MHz, const ru_sdr_cell_validation_config& cell)
{
  return cell.preamble_info.cp_length.is_sample_accurate(convert_srate_MHz_to_Hz(srate_MHz));
}

/// Returns true if the sampling rate is not lower than the cell channel bandwidth.
inline bool does_srate_cover_bandwidth(double srate_MHz, const ru_sdr_cell_validation_config& cell)
{
  return srate_MHz >= bs_channel_bandwidth_to_MHz(cell.channel_bw_mhz);
}

/// Returns true if the DFT size is an integer larger than the number of occupied subcarriers.
inline bool does_srate_fit_dft_size(double srate_MHz, const ru_sdr_cell_validation_config& cell, frequency_range fr)
{
  int64_t srate_Hz = convert_srate_MHz_to_Hz(srate_MHz);
  int64_t scs_Hz   = scs_to_khz(cell.common_scs) * 1000;
  if (srate_Hz % scs_Hz != 0) {
    return false;
  }

  unsigned nof_subcarriers =
      NOF_SUBCARRIERS_PER_RB * band_helper::get_n_rbs_from_bw(cell.channel_bw_mhz, cell.common_scs, fr);
  return srate_Hz / scs_Hz > nof_subcarriers;
}

/// Returns the lowest standard sampling rate in MHz that is valid for the given cell, if any.
inline std::optional<double> derive_ru_sdr_srate_MHz(const ru_sdr_cell_validation_config& cell, frequency_range fr)
{
  for (double srate_MHz : ru_sdr_standard_srates_MHz) {
    if (does_srate_cover_bandwidth(srate_MHz, cell) && does_srate_fit_dft_size(srate_MHz, cell, fr) &&
        is_srate_sample_accurate(srate_MHz) && is_srate_prach_sample_accurate(srate_MHz, cell)) {
      return srate_MHz;
    }
  }

  return std::nullopt;
}

} // namespace ocudu

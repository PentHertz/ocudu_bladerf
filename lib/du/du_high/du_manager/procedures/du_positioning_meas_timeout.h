// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ran/srs/srs_configuration.h"
#include "ocudu/ran/subcarrier_spacing.h"
#include <algorithm>
#include <chrono>

namespace ocudu::odu {

/// \brief Number of SRS periods that the MAC waits for a measurement.
///
/// A second period covers a missed SRS occasion, or an occasion that an earlier measurement of the same C-RNTI uses.
inline constexpr unsigned NOF_SRS_PERIODS_PER_POSITIONING_MEAS = 2;

/// \brief [Implementation-defined] Time that the MAC waits for a measurement on top of the SRS periods.
///
/// It covers the time for the scheduler to start the measurement on an SRS occasion, and the delay of the PHY report.
inline constexpr std::chrono::milliseconds POSITIONING_MEAS_TIMEOUT_MARGIN{20};

/// \brief Time that the gNB-DU keeps to answer within the Response Time.
///
/// The gNB-DU returns the result within the time that the gNB-CU indicates, as per TS 38.473 section 8.13.2.2. The
/// measurement therefore stops before that time, so that the answer still arrives inside it.
inline constexpr std::chrono::milliseconds POSITIONING_MEAS_RESPONSE_MARGIN{10};

/// \brief Returns the time that the MAC waits when the gNB-CU indicates a Response Time.
///
/// The value keeps a margin for the answer. It stays above zero, so that a very short Response Time still starts a
/// measurement and ends in a failure instead of an immediate one.
inline std::chrono::milliseconds get_response_time_deadline(std::chrono::milliseconds response_time)
{
  if (response_time <= POSITIONING_MEAS_RESPONSE_MARGIN) {
    return std::chrono::milliseconds{1};
  }
  return response_time - POSITIONING_MEAS_RESPONSE_MARGIN;
}

/// \brief Returns the time that the MAC waits for the SRS measurements of the given SRS configuration.
///
/// The measurement completes on an SRS occasion, so the time follows the SRS period. A value below one period makes
/// the measurement fail before the UE transmits.
inline std::chrono::milliseconds get_positioning_meas_timeout(const srs_config& srs_cfg, subcarrier_spacing scs)
{
  unsigned max_period_slots = 0;
  for (const auto& srs_res : srs_cfg.srs_res_list) {
    if (srs_res.periodicity_and_offset.has_value()) {
      max_period_slots = std::max(max_period_slots, static_cast<unsigned>(srs_res.periodicity_and_offset->period));
    }
  }

  // Slots per millisecond, as per TS 38.211 section 4.3.1.
  const unsigned slots_per_ms = get_nof_slots_per_subframe(scs);
  const unsigned periods_ms   = (max_period_slots * NOF_SRS_PERIODS_PER_POSITIONING_MEAS) / slots_per_ms;

  return std::chrono::milliseconds{periods_ms} + POSITIONING_MEAS_TIMEOUT_MARGIN;
}

} // namespace ocudu::odu

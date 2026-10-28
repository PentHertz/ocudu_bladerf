// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ran/du_types.h"
#include "ocudu/ran/harq_id.h"
#include "ocudu/ran/phy_time_unit.h"
#include "ocudu/ran/rnti.h"
#include "ocudu/ran/slot_point.h"
#include "ocudu/support/units.h"
#include <optional>

namespace ocudu {

/// Outcome of a UL CRC for a UE PUSCH.
enum class ul_crc_status { ok, ko, dtx };

/// UL CRC processed by the scheduler for a UE PUSCH.
struct ul_crc_event {
  du_ue_index_t                ue_index;
  rnti_t                       rnti;
  du_cell_index_t              cell_index;
  slot_point                   sl_rx;
  harq_id_t                    h_id;
  ul_crc_status                crc;
  units::bytes                 tbs;
  std::optional<float>         ul_sinr_db;
  std::optional<float>         ul_rsrp_dbfs;
  std::optional<phy_time_unit> time_advance_offset;
};

} // namespace ocudu

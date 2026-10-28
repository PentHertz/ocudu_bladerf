// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/cu_cp/cu_cp_configuration.h"

namespace ocudu::config_helpers {

inline bool is_valid_configuration(const ocucp::mobility_configuration& config, ocudulog::basic_logger& logger)
{
  if (!is_valid_configuration(config.meas_mgr_config, logger)) {
    fmt::print("Invalid meas manager configuration.\n");
    return false;
  }

  return true;
}

/// Returns true if the given CU-CP configuration is valid, otherwise false.
inline bool is_valid_configuration(const ocucp::cu_cp_configuration& config, ocudulog::basic_logger& logger)
{
  // Notifiers aren't checked here.
  if (config.node.ran_node_name.empty()) {
    fmt::print("RAN node name is empty\n");
    return false;
  }

  if (!is_valid_configuration(config.mobility, logger)) {
    fmt::print("Invalid mobility configuration.\n");
    return false;
  }

  return true;
}

inline std::vector<plmn_identity>
get_supported_plmns(const std::vector<ocucp::cu_cp_configuration::amf_configuration>& ngaps)
{
  std::vector<plmn_identity> plmns;

  for (const auto& ngap : ngaps) {
    for (const auto& ta : ngap.supported_tas) {
      for (const auto& plmn_item : ta.plmn_list) {
        plmns.push_back(plmn_item.plmn_id);
      }
    }
  }

  return plmns;
}

} // namespace ocudu::config_helpers

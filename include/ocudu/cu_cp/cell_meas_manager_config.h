// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ocudulog/logger.h"
#include "ocudu/ran/meas_types.h"
#include "ocudu/ran/nr_band.h"
#include "ocudu/ran/nr_cell_identity.h"
#include "ocudu/ran/pci.h"
#include "ocudu/ran/plmn_identity.h"
#include "ocudu/ran/tac.h"
#include "ocudu/support/enum_utils.h"

namespace ocudu::ocucp {

/// \brief Essential parameters required to configure serving cell measurements in the UE.
/// Note that some optional values need to be provided by the DU upon F1Setup.
struct serving_cell_meas_config {
  /// The NR cell identifier.
  nr_cell_identity nci;
  /// gNodeB identifier bit length.
  unsigned gnb_id_bit_length;
  /// PLMN identity.
  plmn_identity plmn = plmn_identity::test_value();
  /// Optional configuration parameters that are not set in the initial config and must be provided in a config update
  /// after DU attach.
  /// Tracking area code.
  std::optional<tac_t> tac;
  /// Physical cell identifier.
  std::optional<pci_t> pci;
  /// NR band.
  std::optional<nr_band> band;
  /// SSB measurement and timing config.
  std::optional<rrc_ssb_mtc> ssb_mtc;
  /// SSB ARFCN.
  std::optional<arfcn_t> ssb_arfcn;
  /// SSB subcarrier spacing.
  std::optional<subcarrier_spacing> ssb_scs;
};

/// Neighbor cell MEAS configuration parameters.
struct neighbor_cell_meas_config {
  /// The NR cell identifier.
  nr_cell_identity nci;
  /// The configured report configs.
  std::vector<report_cfg_id_t> report_cfg_ids;
  /// NTN-NeighbourCellInfo-r18.
  std::optional<rrc_ntn_neighbour_cell_info> ntn_neighbour_info;
  /// NTN-PolarizationDL/UL-r17.
  std::optional<ntn_polarization_t> ntn_polarization;
};

/// \brief Essential parameters required to configure serving and neighbor cell measurements in the UE.
/// Note that some optional values need to be provided by the DU upon F1Setup.
struct cell_meas_config {
  /// Serving cell measurement configuration.
  serving_cell_meas_config serving_cell_cfg;
  /// The periodic report configuration.
  std::optional<report_cfg_id_t> periodic_report_cfg_id;
  /// List of neighbor cells.
  std::vector<neighbor_cell_meas_config> ncells;
};

/// \brief Verifies required parameters are set. Returns true if config is valid, false otherwise.
[[nodiscard]] bool is_complete(const serving_cell_meas_config& cfg);

/// Cell manager configuration.
struct cell_meas_manager_config {
  /// Measurement related configs for all known cells.
  std::map<nr_cell_identity, cell_meas_config> cells;
  /// RRC report configuration for all known identifiers.
  std::map<report_cfg_id_t, rrc_report_cfg_nr> report_config_ids;
};

/// \brief Validates configuration but doesn't verify if all provided cells have complete configuration (yet). Returns
/// true if config is valid, false otherwise.
/// \param[in] ssb_freq_to_meas_object Measurement objects already configured, indexed by SSB frequency.
/// \param[in] cfg                     Configuration to validate.
/// \param[in] logger                  Logger used to report the cause of an invalid configuration.
[[nodiscard]] bool
is_valid_configuration(const cell_meas_manager_config&                             cfg,
                       ocudulog::basic_logger&                                     logger,
                       const std::unordered_map<ssb_frequency_t, rrc_meas_obj_nr>& ssb_freq_to_meas_object = {});

/// \brief Same as config validation but additionally verifies that the measurement related parameters are present for
/// all cells.
/// \param[in] cfg    Configuration to validate.
/// \param[in] logger Logger used to report the cause of an invalid or incomplete configuration.
[[nodiscard]] bool is_complete(const cell_meas_manager_config& cfg, ocudulog::basic_logger& logger);

} // namespace ocudu::ocucp

namespace fmt {

// Cell meas config formatter
template <>
struct formatter<ocudu::ocucp::cell_meas_config> {
  template <typename ParseContext>
  auto parse(ParseContext& ctx)
  {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const ocudu::ocucp::cell_meas_config& cfg, FormatContext& ctx) const
  {
    std::string ncell_str = "[ ";
    for (const auto& ncell : cfg.ncells) {
      ncell_str += fmt::format("{:#x} ", ncell.nci);
    }
    ncell_str = ncell_str + "]";

    return format_to(ctx.out(),
                     "nci={:#x} complete={} gnb_id={} pci={} band={} ssb_arfcn={} ssb_scs={} ncells={}",
                     cfg.serving_cell_cfg.nci,
                     is_complete(cfg.serving_cell_cfg) ? "yes" : "no",
                     cfg.serving_cell_cfg.nci.gnb_id(cfg.serving_cell_cfg.gnb_id_bit_length).id,
                     cfg.serving_cell_cfg.pci.has_value() ? to_string(*cfg.serving_cell_cfg.pci) : "?",
                     cfg.serving_cell_cfg.band.has_value() ? to_string(to_underlying(*cfg.serving_cell_cfg.band)) : "?",
                     cfg.serving_cell_cfg.ssb_arfcn.has_value() ? to_string(*cfg.serving_cell_cfg.ssb_arfcn) : "?",
                     cfg.serving_cell_cfg.ssb_scs.has_value() ? to_string(*cfg.serving_cell_cfg.ssb_scs) : "?",
                     ncell_str);
  }
};

} // namespace fmt

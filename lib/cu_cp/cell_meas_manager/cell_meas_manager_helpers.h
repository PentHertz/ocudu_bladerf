// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "measurement_context.h"
#include "ocudu/cu_cp/cell_meas_manager_config.h"
#include "ocudu/rrc/rrc_ue.h"

namespace ocudu::ocucp {

/// \brief Log configured cells.
/// \param[in] cells  Measurement related configuration of all known cells.
/// \param[in] logger Logger used to print the cells.
void log_cells(const std::map<nr_cell_identity, cell_meas_config>& cells, ocudulog::basic_logger& logger);

/// \brief Log measurement objects.
void log_meas_objects(const std::unordered_map<ssb_frequency_t, rrc_meas_obj_nr>& meas_objects,
                      ocudulog::basic_logger&                                     logger);

/// \brief Add old measurement configuration to the remove list of the new configuration.
/// \param[in,out] new_cfg The new configuration.
/// \param[in]     old_cfg The old configuration.
void add_old_meas_config_to_rem_list(rrc_meas_cfg& new_cfg, const rrc_meas_cfg& old_cfg);

/// \brief Drop the removal list entries that the new configuration adds back anyway.
/// Ids already known to the UE are modified in place (TS 38.331 Sections 5.5.2.5, 5.5.2.7 and 5.5.2.3), and removing a
/// measurement object also drops the measurement ids using it (Section 5.5.2.4).
/// \param[in,out] new_cfg The new configuration, with its removal lists already filled from \c old_cfg.
/// \param[in]     old_cfg The configuration currently applied by the UE.
void prune_redundant_rem_list_entries(rrc_meas_cfg& new_cfg, const rrc_meas_cfg& old_cfg);

/// \brief Generate measurement objects for the given cell configuration.
/// \param[in] serving_nci The cell id.
/// \param[in] cfg         The cell configuration.
/// \param[in] logger      Logger used to report neighbor cells without configuration.
/// \returns A vector of SSB frequencies that correlate to measurement objects.
[[nodiscard]] std::vector<ssb_frequency_t> generate_measurement_object_list(nr_cell_identity                serving_nci,
                                                                            const cell_meas_manager_config& cfg,
                                                                            ocudulog::basic_logger&         logger);

/// \brief Generate unique SSB frequency list for CHO (conditional) measurement config.
/// Returns the set of SSB frequencies of neighbor cells whose PCI is in \p candidate_pcis (all neighbors when \p
/// candidate_pcis is empty) and whose configuration is complete.
/// \param[in] serving_nci    The serving cell NCI.
/// \param[in] candidate_pcis Optional PCI filter; empty means all neighbors are included.
/// \param[in] cells          Measurement related configuration of all known cells.
/// \param[in] logger         Logger used to report neighbor cells without configuration.
/// \returns Vector of unique SSB frequencies.
[[nodiscard]] std::vector<ssb_frequency_t>
generate_cho_measurement_object_list(nr_cell_identity                                    serving_nci,
                                     span<const pci_t>                                   candidate_pcis,
                                     const std::map<nr_cell_identity, cell_meas_config>& cells,
                                     ocudulog::basic_logger&                             logger);

/// \brief Check whether the given report config is a conditional trigger (i.e. used for CHO).
/// \param[in] report_cfg_id     The report configuration id.
/// \param[in] report_config_ids The configured report configurations.
/// \returns True if the report config exists and holds a \c rrc_cond_trigger_cfg, false otherwise.
[[nodiscard]] bool is_cond_trigger_report_config(report_cfg_id_t                                     report_cfg_id,
                                                 const std::map<report_cfg_id_t, rrc_report_cfg_nr>& report_config_ids);

/// \brief Generate report configuration for the given cell configuration.
/// \param[in,out] meas_cfg        The measurement configuration the report configuration and measurement id are added
/// to.
/// \param[in,out] ue_meas_context The UE measurement context. \param[in]     nci             The cell id.
/// \param[in]     report_cfg_id   The report configuration id.
/// \param[in]     cfg             The cell configuration.
/// \param[in]     logger          Logger used to report an unknown report configuration id.
void generate_report_config(rrc_meas_cfg&                   meas_cfg,
                            cell_meas_manager_ue_context&   ue_meas_context,
                            nr_cell_identity                nci,
                            report_cfg_id_t                 report_cfg_id,
                            const cell_meas_manager_config& cfg,
                            ocudulog::basic_logger&         logger);

/// \brief Collect rrc_cond_trigger_cfg report configs from \p report_config_ids into \p meas_cfg, filtered by UE
/// capabilities.
/// Appends matching entries to meas_cfg.report_cfg_to_add_mod_list.
/// The number of collected triggers is capped at 1 when the UE does not support condHO-TwoTriggerEvents-r16, and at 2
/// otherwise. Trigger configs requiring Rel-17 capabilities (event-A4, location, time) are skipped if the UE does not
/// advertise the corresponding capability.
/// \returns Vector of report_cfg_id_t for the collected conditional trigger configs, or empty if the UE does
///          not support CHO at all.
[[nodiscard]] std::vector<report_cfg_id_t>
collect_cond_trigger_report_configs(rrc_meas_cfg&                                       meas_cfg,
                                    const rrc_ue_capability_handler&                    ue_caps,
                                    const std::map<report_cfg_id_t, rrc_report_cfg_nr>& report_config_ids,
                                    ocudulog::basic_logger&                             logger);

/// \brief Build measurement IDs linking MOs to conditional trigger report configs for CHO.
/// For each (mo_id, report_cfg_id) pair: allocates a meas_id, appends rrc_meas_id_to_add_mod, populates
/// meas_id_to_meas_context for every NCI that maps to that MO, and fills meas_cfg.nci_to_meas_ids.
/// \returns false if any meas_id allocation fails, true otherwise.
[[nodiscard]] bool generate_cho_meas_ids(rrc_meas_cfg&                                       meas_cfg,
                                         cell_meas_manager_ue_context&                       ue_meas_context,
                                         span<const report_cfg_id_t>                         cond_trigger_ids,
                                         const std::map<nr_cell_identity, cell_meas_config>& cells);

/// \brief Generates the measurement object for the given configuration.
[[nodiscard]] rrc_meas_obj_nr generate_measurement_object(const serving_cell_meas_config& cfg);

/// \brief Check whether the given measurement objects are the same.
[[nodiscard]] bool is_duplicate(const rrc_meas_obj_nr& obj_1, const rrc_meas_obj_nr& obj_2);

} // namespace ocudu::ocucp

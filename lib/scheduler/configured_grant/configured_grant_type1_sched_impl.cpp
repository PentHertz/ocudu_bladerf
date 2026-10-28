// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "configured_grant_type1_sched_impl.h"
#include "../support/mcs_tbs_calculator.h"
#include "../support/sch_pdu_builder.h"
#include "cg_sched_helpers.h"

using namespace ocudu;

static vrb_interval compute_cg_vrbs(const cg_configuration::rrc_configured_ul_grant& ul_grant)
{
  // Compute VRBs: CG PUSCH uses non-interleaved VRB-to-PRB mapping, so VRBs = PRBs.
  const auto& cg_freq_alloc = std::get<ra_frequency_type1_configuration>(ul_grant.freq_domain_res);
  return vrb_interval::start_and_len(cg_freq_alloc.start_vrb, cg_freq_alloc.length_vrb);
}

configured_grant_type1_sched_impl::configured_grant_type1_sched_impl(const cell_configuration& cell_cfg_,
                                                                     const cg_builder_params&  cell_cg_cfg,
                                                                     uci_allocator&            uci_alloc_,
                                                                     ue_repository&            ues_) :
  configured_grant_scheduler_base(cell_cfg_, cell_cg_cfg, uci_alloc_, ues_)
{
}

bool configured_grant_type1_sched_impl::has_cg_grant_cfg(const ue_cell_configuration& ue_cfg) const
{
  const auto* ul_ded = ue_cfg.init_bwp().ul.ded();
  return ul_ded != nullptr and ul_ded->cg_cfg.has_value() and ul_ded->cg_cfg->rrc_configured_ul_grant_cfg.has_value();
}

void configured_grant_type1_sched_impl::add_ue_to_wheel(const ue_cell_configuration& ue_cfg)
{
  auto* u = ues.find_by_rnti(ue_cfg.crnti);
  if (u == nullptr) {
    logger.error("rnti={}: UE not found in the CG scheduler UEs repo", ue_cfg.crnti);
    return;
  }

  auto* ue_cc = u->find_cell(cell_cfg.cell_index);
  if (ue_cc == nullptr) {
    logger.error("rnti={}: UE cell not found in the CG scheduler cell", ue_cfg.crnti);
    return;
  }

  if (not has_cg_grant_cfg(ue_cfg)) {
    return;
  }

  const cg_configuration& cg_cfg   = ue_cfg.init_bwp().ul.ded()->cg_cfg.value();
  const auto&             ul_grant = cg_cfg.rrc_configured_ul_grant_cfg.value();

  // Fill the slot wheel at every slot where a CG PUSCH opportunity occurs.
  const auto wheel_res = periodic_pusch_slot_wheel.add_ue(ue_cfg.crnti, ul_grant.time_domain_offset);
  if (wheel_res.nof_dup != 0) {
    logger.error("rnti={}: CG grant already present in slot wheel at {} offsets", ue_cfg.crnti, wheel_res.nof_dup);
  }
  if (wheel_res.nof_full != 0) {
    logger.error("rnti={}: CG grant dropped at {} slot wheel offsets with no room left; the RRM admitted a UE the "
                 "scheduler cannot hold",
                 ue_cfg.crnti,
                 wheel_res.nof_full);
  }

  // Register the UE in the list of recently updated UEs, so that its CG resources get pre-reserved over the whole
  // resource grid at the next run_slot().
  updated_ues.push_back(ue_cfg.crnti);

  const pusch_config_params pusch_params = build_cg_pusch_cfg_params(ue_cfg, ul_grant.time_domain_allocation);
  const auto                cg_vrbs      = compute_cg_vrbs(ul_grant);
  // In CG type1, res ID is not applicable.
  constexpr unsigned non_applicable_res_id = 0U;
  // Save the PUSCH parameters that will be used for the periodic allocations.
  ue_cc->get_conf_grant_state_manager().update_state(
      static_cast<unsigned>(cg_periodicity),
      compute_ul_tbs_unsafe(pusch_params, sch_mcs_index{ul_grant.mcs}, cg_vrbs.length()),
      cg_vrbs,
      ul_grant.mcs,
      ul_grant.time_domain_allocation,
      ul_grant.time_domain_offset,
      non_applicable_res_id);
}

void configured_grant_type1_sched_impl::rem_ue(const ue_cell_configuration& ue_cfg)
{
  if (not has_cg_grant_cfg(ue_cfg)) {
    return;
  }
  const cg_configuration& cg_cfg   = ue_cfg.init_bwp().ul.ded()->cg_cfg.value();
  const auto&             ul_grant = cg_cfg.rrc_configured_ul_grant_cfg.value();

  // Remove the UE from the periodic wheel and the updated-UE list, even though the UE is gone from the repository: this
  // is to avoid to keep allocating CG PUSCH for a released RNTI (stale entries).
  const unsigned nof_missing = periodic_pusch_slot_wheel.rem_ue(ue_cfg.crnti, ul_grant.time_domain_offset);
  if (nof_missing != 0) {
    logger.error(
        "rnti={}: CG grant not found in slot wheel at {} offsets during UE removal", ue_cfg.crnti, nof_missing);
  }
  rem_ue_from_updated_ues(ue_cfg.crnti);

  // Resetting the CG state needs the UE cell context, which the steps above deliberately do not.
  auto* u = ues.find_by_rnti(ue_cfg.crnti);
  if (u == nullptr) {
    logger.error("rnti={}: UE not found in the CG scheduler UEs repo during UE removal", ue_cfg.crnti);
    return;
  }
  auto* ue_cc = u->find_cell(cell_cfg.cell_index);
  if (ue_cc == nullptr) {
    logger.error("rnti={}: UE cell not found in the CG scheduler cell during UE removal", ue_cfg.crnti);
    return;
  }
  ue_cc->get_conf_grant_state_manager().reset_state();
}

void configured_grant_type1_sched_impl::add_reconf_ue(const ue_cell_configuration& new_ue_cfg,
                                                      const ue_cell_configuration* old_ue_cfg)
{
  if (old_ue_cfg == nullptr) {
    add_ue_to_wheel(new_ue_cfg);
    return;
  }

  const auto* new_ul_ded = new_ue_cfg.init_bwp().ul.ded();
  const auto* old_ul_ded = old_ue_cfg->init_bwp().ul.ded();

  if (new_ul_ded != nullptr and old_ul_ded != nullptr and new_ul_ded->cg_cfg.has_value() and
      old_ul_ded->cg_cfg.has_value() and new_ul_ded->cg_cfg.value() == old_ul_ded->cg_cfg.value()) {
    // CG configuration unchanged — nothing to do.
    return;
  }

  rem_ue(*old_ue_cfg);
  add_ue_to_wheel(new_ue_cfg);
}

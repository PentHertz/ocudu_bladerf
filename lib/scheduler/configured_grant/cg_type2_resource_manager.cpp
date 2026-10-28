// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "cg_type2_resource_manager.h"
#include "ocudu/adt/format.h"
#include "ocudu/ran/resource_allocation/resource_allocation_frequency.h"
#include "ocudu/scheduler/config/ran_cell_config_helper.h"
#include "ocudu/scheduler/rrm/configured_grant_type1_rrm.h"
#include "ocudu/scheduler/scheduler_configurator.h"
#include "ocudu/scheduler/support/rb_helper.h"

using namespace ocudu;

cg_type2_resource_manager::cg_type2_resource_manager(const cell_configuration& cell_cfg_) :
  cell_cfg(cell_cfg_), logger(ocudulog::fetch_basic_logger("SCHED"))
{
  // Compute the CRB block of each CG resource. Each resource takes the next block of CRBs, so that two UEs activated
  // on the same slot offset, which are necessarily handed two different resource ids, never end up transmitting on the
  // same RBs.
  const std::vector<crb_interval> cg_res_crbs = config_helpers::compute_cg_type2_freq_resources(cell_cfg.params);
  cg_resource_list.reserve(cg_res_crbs.size());
  for (unsigned n = 0, sz = cg_res_crbs.size(); n != sz; ++n) {
    cg_resource_list.push_back({n, cg_res_crbs[n]});
  }

  // Resize the free resource wheel according to the CG period in slot.
  ocudu_assert(cell_cfg.params.init_bwp.cg_cfg.has_value(), "Configured Grant must be configured");
  resource_wheel.resize(static_cast<unsigned>(cell_cfg.params.init_bwp.cg_cfg.value().periodicity));

  // Add the resources to the wheel, at the slot offsets that are usable for CG.
  reset();
}

void cg_type2_resource_manager::reset()
{
  for (std::vector<cg_res_occupancy>& res_at_offset : resource_wheel) {
    res_at_offset.clear();
  }

  nof_free_resources = 0;
  for (unsigned offset : config_helpers::compute_cg_usable_slot_offsets(cell_cfg.params)) {
    std::vector<cg_res_occupancy>& res_at_offset = resource_wheel[offset];
    res_at_offset.reserve(cg_resource_list.size());
    for (const auto& cg_res : cg_resource_list) {
      res_at_offset.push_back({cg_res.id, std::nullopt});
      ++nof_free_resources;
    }
  }
}

std::optional<cg_type2_resource> cg_type2_resource_manager::request_resource(unsigned cg_offset, rnti_t rnti)
{
  std::vector<cg_res_occupancy>& res_at_offset = resource_wheel[cg_offset];

  // Hand out the highest free id first.
  // TODO: optimize the way resources are allocated.
  auto free_it = res_at_offset.rend();
  for (auto it = res_at_offset.rbegin(); it != res_at_offset.rend(); ++it) {
    if (it->owner == rnti) {
      // Handing out a second resource would leak the first one, as the UE only ever returns one.
      logger.error("rnti={}: CG resource requested at CG time offset={}, but the UE already holds id={} there",
                   rnti,
                   cg_offset,
                   it->id);
      return cg_resource_list[it->id];
    }
    if (free_it == res_at_offset.rend() and not it->owner.has_value()) {
      free_it = it;
    }
  }
  if (free_it == res_at_offset.rend()) {
    return std::nullopt;
  }

  free_it->owner.emplace(rnti);

  ocudu_assert(nof_free_resources != 0, "Unexpected nof of free CG type2 resources");
  --nof_free_resources;
  // Resources are stored in order of ID.
  return cg_resource_list[free_it->id];
}

unsigned cg_type2_resource_manager::get_nof_free_resources() const
{
  return nof_free_resources;
}

void cg_type2_resource_manager::deallocate_resource(unsigned cg_offset, cg_type2_resource::res_id res_id, rnti_t rnti)
{
  std::vector<cg_res_occupancy>& res_at_offset = resource_wheel[cg_offset];

  const auto it = std::find_if(
      res_at_offset.begin(), res_at_offset.end(), [res_id](const cg_res_occupancy& res) { return res.id == res_id; });

  if (it == res_at_offset.end()) {
    logger.error(
        "rnti={}: CG resource id={} to deallocate does not exist at CG time offset={}", rnti, res_id, cg_offset);
    return;
  }
  if (not it->owner.has_value()) {
    logger.error(
        "rnti={}: CG resource id={} to deallocate is already free at CG time offset={}", rnti, res_id, cg_offset);
    return;
  }
  // NOTE: putting the CRBs of a resource back in the pool while the UE that holds it keeps transmitting on them would
  // hand the same RBs to a second UE in the same slot; this would be worse than leaking the resource, hence return.
  if (it->owner.value() != rnti) {
    logger.error("rnti={}: CG resource id={} to deallocate at CG time offset={} is held by rnti={}",
                 rnti,
                 res_id,
                 cg_offset,
                 it->owner.value());
    return;
  }

  it->owner.reset();
  ++nof_free_resources;
}

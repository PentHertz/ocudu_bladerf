// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/scheduler/result/vrb_alloc.h"
#include "ocudu/support/enum_utils.h"

using namespace ocudu;

/// Computes the BWP VRBs covered by an RBG.
static vrb_interval get_rbg_vrbs(crb_interval bwp_rbs, nominal_rbg_size P, unsigned first_rbg_size, unsigned rbg_idx)
{
  const unsigned vrb_start = (rbg_idx == 0) ? 0 : (rbg_idx - 1) * to_underlying(P) + first_rbg_size;
  return {vrb_start, vrb_start + get_rbg_size(bwp_rbs, P, rbg_idx)};
}

vrb_bitmap ocudu::convert_rbgs_to_vrbs(const rbg_bitmap& rbgs, crb_interval bwp_rbs, nominal_rbg_size P)
{
  const unsigned first_rbg_size = get_rbg_size(bwp_rbs, P, 0);

  vrb_bitmap vrbs(bwp_rbs.length());
  for (unsigned idx = 0, nof_rbgs = rbgs.size(); idx != nof_rbgs; ++idx) {
    if (rbgs.test(idx)) {
      vrbs |= get_rbg_vrbs(bwp_rbs, P, first_rbg_size, idx);
    }
  }
  return vrbs;
}

rbg_bitmap ocudu::convert_vrbs_to_rbgs(const vrb_bitmap& vrbs, crb_interval bwp_rbs, nominal_rbg_size P)
{
  const unsigned first_rbg_size = get_rbg_size(bwp_rbs, P, 0);

  rbg_bitmap rbgs(get_nof_rbgs(bwp_rbs, P));
  for (unsigned idx = 0, nof_rbgs = rbgs.size(); idx != nof_rbgs; ++idx) {
    const vrb_interval rbg_vrbs = get_rbg_vrbs(bwp_rbs, P, first_rbg_size, idx);
    if (vrbs.any(rbg_vrbs.start(), rbg_vrbs.stop())) {
      rbgs.set(idx);
    }
  }
  return rbgs;
}

rbg_bitmap ocudu::find_free_rbgs_with_nof_vrbs(const rbg_bitmap& used_rbgs,
                                               crb_interval      bwp_rbs,
                                               nominal_rbg_size  P,
                                               unsigned          nof_vrbs)
{
  const unsigned P_rbs    = to_underlying(P);
  const unsigned last_rbg = used_rbgs.size() - 1;

  // Only the first and last RBGs may be smaller than P, so the RBGs in between are interchangeable. A greedy selection
  // could pick a short edge RBG that makes the exact size unreachable.
  unsigned nof_free_mid_rbgs = 0;
  for (unsigned rbg_idx = 1; rbg_idx < last_rbg; ++rbg_idx) {
    nof_free_mid_rbgs += used_rbgs.test(rbg_idx) ? 0 : 1;
  }

  rbg_bitmap selected_rbgs(used_rbgs.size());
  for (unsigned edge_mask = 0; edge_mask != 4; ++edge_mask) {
    const bool use_first = (edge_mask & 1U) != 0;
    const bool use_last  = (edge_mask & 2U) != 0;
    if ((use_first and used_rbgs.test(0)) or (use_last and (last_rbg == 0 or used_rbgs.test(last_rbg)))) {
      continue;
    }
    const unsigned edge_vrbs =
        (use_first ? get_rbg_size(bwp_rbs, P, 0) : 0) + (use_last ? get_rbg_size(bwp_rbs, P, last_rbg) : 0);
    if (edge_vrbs > nof_vrbs or (nof_vrbs - edge_vrbs) % P_rbs != 0 or
        (nof_vrbs - edge_vrbs) / P_rbs > nof_free_mid_rbgs) {
      continue;
    }

    if (use_first) {
      selected_rbgs.set(0);
    }
    if (use_last) {
      selected_rbgs.set(last_rbg);
    }
    for (unsigned rbg_idx = 1, nof_mid_rbgs = (nof_vrbs - edge_vrbs) / P_rbs; nof_mid_rbgs != 0; ++rbg_idx) {
      if (not used_rbgs.test(rbg_idx)) {
        selected_rbgs.set(rbg_idx);
        --nof_mid_rbgs;
      }
    }
    return selected_rbgs;
  }
  return selected_rbgs;
}

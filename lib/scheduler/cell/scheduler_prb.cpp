// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "scheduler_prb.h"
#include "ocudu/scheduler/result/resource_block_group.h"
#include "ocudu/support/enum_utils.h"
#include <cmath>

using namespace ocudu;

bwp_rb_bitmap::bwp_rb_bitmap(crb_interval bwp_crbs, rbg_size rbg_sz) :
  prbs_(bwp_crbs.length()),
  rbgs_(get_nof_rbgs(bwp_crbs, rbg_sz)),
  P_(get_nominal_rbg_size(bwp_crbs.length(), rbg_sz)),
  Pnofbits(std::log2(to_underlying(P_))),
  first_rbg_size(get_rbg_size(bwp_crbs, P_, 0))
{
}

uint32_t bwp_rb_bitmap::prb_to_rbg_idx(uint32_t prb_idx) const
{
  return ((prb_idx + to_underlying(P_) - first_rbg_size) >> Pnofbits);
}

void bwp_rb_bitmap::add_prbs_to_rbgs(const prb_bitmap& grant)
{
  int idx = 0;
  do {
    idx = grant.find_lowest(idx, grant.size(), true);
    if (idx < 0) {
      return;
    }
    uint32_t rbg_idx = prb_to_rbg_idx(idx);
    rbgs_.set(rbg_idx, true);
    idx++;
  } while (idx != (int)grant.size());
}

void bwp_rb_bitmap::add_prbs_to_rbgs(const prb_interval& grant)
{
  uint32_t rbg_start = prb_to_rbg_idx(grant.start());
  uint32_t rbg_stop  = std::min(prb_to_rbg_idx(grant.stop() - 1) + 1u, (uint32_t)rbgs_.size());
  rbgs_.fill(rbg_start, rbg_stop);
}

void bwp_rb_bitmap::add_rbgs_to_prbs(const rbg_bitmap& grant)
{
  int idx = 0;
  do {
    idx = grant.find_lowest(idx, grant.size(), true);
    if (idx < 0) {
      return;
    }
    uint32_t prb_idx = (idx == 0) ? 0 : (idx - 1) * to_underlying(P_) + first_rbg_size;
    uint32_t prb_end = std::min(prb_idx + ((idx == 0) ? first_rbg_size : to_underlying(P_)), (uint32_t)prbs_.size());
    prbs_.fill(prb_idx, prb_end);
    idx++;
  } while (idx != (int)grant.size());
}

// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/scheduler/result/sched_result_grid.h"
#include <vector>

namespace ocudu {

/// Broadcast allocations of a slot, held in plain vectors.
struct dl_broadcast_allocation_copy {
  std::vector<ssb_information> ssb_info;
  std::vector<sib_information> sibs;
};

/// \brief Downlink result of a slot, held in plain vectors.
///
/// Mirrors \ref dl_sched_result, and unlike it is copyable whichever pool its PDUs came from.
struct dl_sched_result_copy {
  unsigned                          nof_dl_symbols = 0;
  std::vector<pdcch_dl_information> dl_pdcchs;
  std::vector<pdcch_ul_information> ul_pdcchs;
  dl_broadcast_allocation_copy      bc;
  std::vector<rar_information>      rar_grants;
  std::vector<dl_paging_allocation> paging_grants;
  std::vector<dl_msg_alloc>         ue_grants;
  std::vector<csi_rs_info>          csi_rs;
  std::vector<prs_info>             prs;
};

/// Uplink result of a slot, held in plain vectors.
struct ul_sched_result_copy {
  unsigned                         nof_ul_symbols = 0;
  std::vector<ul_sched_info>       puschs;
  std::vector<prach_occasion_info> prachs;
  std::vector<pucch_info>          pucchs;
  std::vector<srs_info>            srss;
};

/// Result of a slot, held in plain vectors.
struct sched_result_copy {
  bool                  success = false;
  failed_alloc_attempts failed_attempts;
  dl_sched_result_copy  dl;
  ul_sched_result_copy  ul;
};

namespace detail {

/// Copies the PDUs a view yields into a vector.
template <typename View, typename T>
void copy_pdu_list(const View& src, std::vector<T>& dst)
{
  dst.assign(src.begin(), src.end());
}

/// Copies the PDUs of a vector into the list a builder fills.
template <typename T, typename Builder>
void fill_pdu_list(const std::vector<T>& src, Builder dst)
{
  for (const T& pdu : src) {
    [[maybe_unused]] const T* added = dst.emplace_back(pdu);
    ocudu_assert(added != nullptr, "Slot result list is too small to hold the copy");
  }
}

} // namespace detail

/// Copies the downlink result a reader points at.
inline void copy_result(const sched_slot_dl_result_reader& src, dl_sched_result_copy& dst)
{
  dst.nof_dl_symbols = src.nof_dl_symbols();
  detail::copy_pdu_list(src.dl_pdcchs(), dst.dl_pdcchs);
  detail::copy_pdu_list(src.ul_pdcchs(), dst.ul_pdcchs);
  detail::copy_pdu_list(src.ssb_info(), dst.bc.ssb_info);
  detail::copy_pdu_list(src.sibs(), dst.bc.sibs);
  detail::copy_pdu_list(src.rar_grants(), dst.rar_grants);
  detail::copy_pdu_list(src.paging_grants(), dst.paging_grants);
  detail::copy_pdu_list(src.ue_grants(), dst.ue_grants);
  detail::copy_pdu_list(src.csi_rs(), dst.csi_rs);
  detail::copy_pdu_list(src.prs(), dst.prs);
}

/// Copies the uplink result a reader points at.
inline void copy_result(const sched_slot_ul_result_reader& src, ul_sched_result_copy& dst)
{
  dst.nof_ul_symbols = src.nof_ul_symbols();
  detail::copy_pdu_list(src.puschs(), dst.puschs);
  detail::copy_pdu_list(src.prachs(), dst.prachs);
  detail::copy_pdu_list(src.srss(), dst.srss);
  const span<const pucch_info> pucchs = src.pucchs().unsorted();
  dst.pucchs.assign(pucchs.begin(), pucchs.end());
}

/// Fills the downlink result of a slot with a copy held in plain vectors.
inline void copy_result(const dl_sched_result_copy& src, sched_slot_dl_result_builder dst)
{
  detail::fill_pdu_list(src.dl_pdcchs, dst.dl_pdcchs());
  detail::fill_pdu_list(src.ul_pdcchs, dst.ul_pdcchs());
  detail::fill_pdu_list(src.bc.ssb_info, dst.ssb_info());
  detail::fill_pdu_list(src.bc.sibs, dst.sibs());
  detail::fill_pdu_list(src.rar_grants, dst.rar_grants());
  detail::fill_pdu_list(src.paging_grants, dst.paging_grants());
  detail::fill_pdu_list(src.ue_grants, dst.ue_grants());
  detail::fill_pdu_list(src.csi_rs, dst.csi_rs());
  detail::fill_pdu_list(src.prs, dst.prs());
}

/// Fills the uplink result of a slot with a copy held in plain vectors.
inline void copy_result(const ul_sched_result_copy& src, sched_slot_ul_result_builder dst)
{
  detail::fill_pdu_list(src.puschs, dst.puschs());
  detail::fill_pdu_list(src.prachs, dst.prachs());
  detail::fill_pdu_list(src.srss, dst.srss());
  for (const pucch_info& pucch : src.pucchs) {
    dst.pucchs().emplace(pucch);
  }
}

/// Copies the result a reader points at.
inline void copy_result(const sched_slot_result_reader& src, sched_result_copy& dst)
{
  dst.success         = src.success();
  dst.failed_attempts = src.failed_attempts();
  copy_result(src.dl(), dst.dl);
  copy_result(src.ul(), dst.ul);
}

} // namespace ocudu

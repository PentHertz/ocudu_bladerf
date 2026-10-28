// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/ran/rnti.h"
#include "ocudu/scheduler/result/sched_result_grid.h"

namespace ocudu {
struct csi_report_configuration;

template <typename DlResult>
inline const pdcch_dl_information* find_ue_dl_pdcch(rnti_t rnti, const DlResult& res)
{
  const auto it = std::find_if(res.dl_pdcchs.begin(), res.dl_pdcchs.end(), [rnti](const pdcch_dl_information& pdcch) {
    return pdcch.ctx.rnti == rnti;
  });
  return it != res.dl_pdcchs.end() ? &*it : nullptr;
}

template <typename DlResult>
inline const pdcch_ul_information* find_ue_ul_pdcch(rnti_t rnti, const DlResult& res)
{
  const auto it = std::find_if(res.ul_pdcchs.begin(), res.ul_pdcchs.end(), [rnti](const pdcch_ul_information& pdcch) {
    return pdcch.ctx.rnti == rnti;
  });
  return it != res.ul_pdcchs.end() ? &*it : nullptr;
}

inline const pdcch_dl_information* find_ue_dl_pdcch(rnti_t rnti, sched_slot_dl_result_reader dl_res)
{
  const auto pdcchs = dl_res.dl_pdcchs();
  const auto it     = std::find_if(
      pdcchs.begin(), pdcchs.end(), [rnti](const pdcch_dl_information& pdcch) { return pdcch.ctx.rnti == rnti; });
  return it != pdcchs.end() ? &*it : nullptr;
}

inline const pdcch_ul_information* find_ue_ul_pdcch(rnti_t rnti, sched_slot_dl_result_reader dl_res)
{
  const auto pdcchs = dl_res.ul_pdcchs();
  const auto it     = std::find_if(
      pdcchs.begin(), pdcchs.end(), [rnti](const pdcch_ul_information& pdcch) { return pdcch.ctx.rnti == rnti; });
  return it != pdcchs.end() ? &*it : nullptr;
}

template <typename List>
inline const rar_information* find_rar_pdsch(rnti_t ra_rnti, const List& rars)
{
  const auto it = std::find_if(
      rars.begin(), rars.end(), [ra_rnti](const rar_information& grant) { return grant.pdsch_cfg.rnti == ra_rnti; });
  return it != rars.end() ? &*it : nullptr;
}

template <typename List>
inline const dl_msg_alloc* find_ue_pdsch(rnti_t rnti, const List& dlgrants)
{
  const auto it = std::find_if(
      dlgrants.begin(), dlgrants.end(), [rnti](const dl_msg_alloc& pdsch) { return pdsch.pdsch_cfg.rnti == rnti; });
  return it != dlgrants.end() ? &*it : nullptr;
}

template <typename List>
inline const dl_msg_alloc* find_ue_pdsch_with_lcid(rnti_t rnti, lcid_dl_sch_t lcid, const List& dlgrants)
{
  const auto it = std::find_if(dlgrants.begin(), dlgrants.end(), [rnti, lcid](const dl_msg_alloc& pdsch) {
    return pdsch.pdsch_cfg.rnti == rnti and std::any_of(pdsch.tb_list[0].lc_chs_to_sched.begin(),
                                                        pdsch.tb_list[0].lc_chs_to_sched.end(),
                                                        [lcid](const dl_msg_lc_info& t) { return t.lcid == lcid; });
  });
  return it != dlgrants.end() ? &*it : nullptr;
}

template <typename List>
inline const dl_msg_alloc* find_ue_pdsch_with_lcid(rnti_t rnti, lcid_t lcid, const List& dlgrants)
{
  return find_ue_pdsch_with_lcid(rnti, lcid_dl_sch_t{lcid}, dlgrants);
}

template <typename List>
inline const ul_sched_info* find_ue_pusch(rnti_t rnti, const List& ulgrants)
{
  const auto it = std::find_if(
      ulgrants.begin(), ulgrants.end(), [rnti](const auto& pusch) { return pusch.pusch_cfg.rnti == rnti; });
  return it != ulgrants.end() ? &*it : nullptr;
}

inline const ul_sched_info* find_ue_pusch(rnti_t rnti, const ul_sched_result& res)
{
  return find_ue_pusch(rnti, res.puschs);
}

inline const pucch_info* find_ue_pucch(rnti_t rnti, const span<const pucch_info> pucchs)
{
  const auto* it =
      std::find_if(pucchs.begin(), pucchs.end(), [rnti](const pucch_info& pucch) { return pucch.crnti == rnti; });
  return it != pucchs.end() ? &*it : nullptr;
}

inline const pucch_info* find_ue_pucch_with_harq_ack(rnti_t rnti, span<const pucch_info> pucchs)
{
  const pucch_info* pucch = find_ue_pucch(rnti, pucchs);
  if (pucch != nullptr and pucch->uci_bits.harq_ack_nof_bits > 0) {
    return pucch;
  }
  return nullptr;
}

inline const pucch_info* find_ue_pucch_with_sr(rnti_t rnti, span<const pucch_info> pucchs)
{
  const pucch_info* pucch = find_ue_pucch(rnti, pucchs);
  if (pucch != nullptr and pucch->uci_bits.sr_bits != sr_nof_bits::no_sr) {
    return pucch;
  }
  return nullptr;
}

inline const pucch_info* find_ue_pucch_with_csi(rnti_t rnti, span<const pucch_info> pucchs)
{
  const pucch_info* pucch = find_ue_pucch(rnti, pucchs);
  if (pucch != nullptr and pucch->uci_bits.csi_part1_nof_bits > 0) {
    return pucch;
  }
  return nullptr;
}

/// Find UCI (both PUCCH and PUSCH) containing CSI for the given RNTI.
template <typename UlResult>
const csi_report_configuration* find_ue_uci_with_csi(rnti_t rnti, const UlResult& ul_res)
{
  if (ul_res.nof_ul_symbols == 0) {
    return nullptr;
  }
  const span<const pucch_info> pucchs{ul_res.pucchs.data(), ul_res.pucchs.size()};
  const pucch_info*            pucch = find_ue_pucch_with_csi(rnti, pucchs);
  if (pucch != nullptr) {
    return &pucch->csi_rep_cfg.value();
  }
  for (const ul_sched_info& ul : ul_res.puschs) {
    if (ul.pusch_cfg.rnti == rnti and ul.uci.has_value() and ul.uci.value().csi.has_value()) {
      return &ul.uci.value().csi.value().csi_rep_cfg;
    }
  }
  return nullptr;
}

/// Find UCI (both PUCCH and PUSCH) containing CSI for the given RNTI.
inline const csi_report_configuration* find_ue_uci_with_csi(rnti_t rnti, sched_slot_ul_result_reader ul_res)
{
  if (ul_res.nof_ul_symbols() == 0) {
    return nullptr;
  }
  const pucch_info* pucch = find_ue_pucch_with_csi(rnti, ul_res.pucchs().unsorted());
  if (pucch != nullptr) {
    return &pucch->csi_rep_cfg.value();
  }
  const auto puschs = ul_res.puschs();
  for (const ul_sched_info& ul : puschs) {
    if (ul.pusch_cfg.rnti == rnti and ul.uci.has_value() and ul.uci.value().csi.has_value()) {
      return &ul.uci.value().csi.value().csi_rep_cfg;
    }
  }
  return nullptr;
}

} // namespace ocudu

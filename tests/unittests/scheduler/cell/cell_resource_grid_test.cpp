// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "lib/scheduler/cell/resource_grid.h"
#include "tests/ocudu_test_requirements.h"
#include "tests/test_doubles/scheduler/scheduler_config_helper.h"
#include "tests/unittests/scheduler/test_utils/config_generators.h"
#include "ocudu/scheduler/config/scheduler_expert_config_factory.h"
#include "ocudu/scheduler/resource_grid_util.h"
#include <gtest/gtest.h>

using namespace ocudu;

std::vector<scs_specific_carrier> test_carriers()
{
  std::vector<scs_specific_carrier> ret;
  // 15 kHz.
  ret.emplace_back();
  ret.back().scs               = subcarrier_spacing::kHz15;
  ret.back().offset_to_carrier = 0;
  ret.back().carrier_bandwidth = 52;
  // 120 kHz.
  ret.emplace_back();
  ret.back().scs               = subcarrier_spacing::kHz120;
  ret.back().offset_to_carrier = 0;
  ret.back().carrier_bandwidth = 275;
  return ret;
}

TEST(carrier_subslot_resource_grid_test, test_all)
{
  std::vector<scs_specific_carrier> carrier_cfgs = test_carriers();

  // Wideband Carrier, 15kHz case.
  {
    carrier_subslot_resource_grid carrier_grid(carrier_cfgs[0]);
    ASSERT_EQ(fmt::underlying(subcarrier_spacing::kHz15), fmt::underlying(carrier_grid.scs()));
    ASSERT_EQ(52, carrier_grid.nof_rbs());
    ASSERT_EQ(0, carrier_grid.offset());
    crb_interval lims{0, 52};
    ASSERT_TRUE(carrier_grid.rb_dims() == lims);
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{0, 14}, lims));
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{2, 5}, crb_interval{2, 5}));
    ASSERT_TRUE(not carrier_grid.all_set(ofdm_symbol_range{0, 14}, lims));

    carrier_grid.fill({2, 5}, crb_interval{2, 4});
    ASSERT_TRUE(carrier_grid.collides(ofdm_symbol_range{2, 3}, crb_interval{1, 3}));
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{2, 3}, crb_interval{1, 2}));
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{0, 1}, crb_interval{1, 3}));
    ASSERT_TRUE(carrier_grid.all_set(ofdm_symbol_range{2, 5}, crb_interval{2, 4}));
    ASSERT_TRUE(carrier_grid.all_set(ofdm_symbol_range{2, 4}, crb_interval{2, 3}));
    ASSERT_TRUE(carrier_grid.all_set(ofdm_symbol_range{3, 5}, crb_interval{3, 4}));
    ASSERT_TRUE(not carrier_grid.all_set(ofdm_symbol_range{1, 5}, crb_interval{1, 4}));
    ASSERT_TRUE(not carrier_grid.all_set(ofdm_symbol_range{2, 6}, crb_interval{2, 5}));

    carrier_grid.fill(ofdm_symbol_range{0, 14}, lims);
    ASSERT_TRUE(carrier_grid.all_set(ofdm_symbol_range{0, 14}, lims));
    ASSERT_TRUE(carrier_grid.all_set(ofdm_symbol_range{0, 13}, {0, 51}));
    ASSERT_TRUE(carrier_grid.all_set(ofdm_symbol_range{1, 14}, {1, 52}));

    carrier_grid.clear();
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{2, 3}, crb_interval{1, 3}));
  }

  // Narrowband Carrier, 15kHz case.
  {
    scs_specific_carrier carrier_cfg = carrier_cfgs[0];
    carrier_cfg.offset_to_carrier    = 10;
    carrier_cfg.carrier_bandwidth    = 20;
    carrier_subslot_resource_grid carrier_grid(carrier_cfg);
    ASSERT_EQ(fmt::underlying(subcarrier_spacing::kHz15), fmt::underlying(carrier_grid.scs()));
    ASSERT_EQ(20, carrier_grid.nof_rbs());
    ASSERT_EQ(10, carrier_grid.offset());
    crb_interval lims{10, 30};
    ASSERT_TRUE(carrier_grid.rb_dims() == lims);
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{0, 14}, lims));
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{2, 5}, crb_interval{12, 15}));

    carrier_grid.fill({2, 5}, crb_interval{12, 14});
    ASSERT_TRUE(carrier_grid.collides(ofdm_symbol_range{2, 3}, crb_interval{11, 13}));
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{2, 3}, crb_interval{11, 12}));
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{0, 1}, crb_interval{11, 13}));

    carrier_grid.clear();
    ASSERT_TRUE(not carrier_grid.collides(ofdm_symbol_range{2, 3}, crb_interval{11, 13}));
  }
}

TEST(cell_resource_grid_test, test_all)
{
  std::vector<scs_specific_carrier> carrier_cfgs = test_carriers();

  // Wide BWP, 15 kHz case.
  {
    cell_slot_resource_grid cell_grid{carrier_cfgs};
    bwp_configuration       bwp_cfg{};
    bwp_cfg.scs  = ocudu::subcarrier_spacing::kHz15;
    bwp_cfg.crbs = {0, 52};

    ASSERT_TRUE(not cell_grid.collides(bwp_sch_grant_info(bwp_cfg, {0, 14}, {0, 52})));

    bwp_sch_grant_info grant{bwp_cfg, {2, 14}, {5, 10}};
    cell_grid.fill(grant);
    ASSERT_TRUE(cell_grid.collides(subcarrier_spacing::kHz15, {0, 14}, crb_interval{0, 52}));
    ASSERT_TRUE(not cell_grid.collides(subcarrier_spacing::kHz15, {0, 14}, crb_interval{0, 5}));
    ASSERT_TRUE(cell_grid.collides(subcarrier_spacing::kHz15, {2, 3}, crb_interval{0, 6}));
    ASSERT_TRUE(not cell_grid.collides(subcarrier_spacing::kHz15, {0, 2}, crb_interval{0, 6}));

    cell_grid.clear();
    ASSERT_TRUE(not cell_grid.collides(subcarrier_spacing::kHz15, {2, 3}, crb_interval{0, 6}));
  }

  // Narrow BWP, 15 kHz case.
  {
    cell_slot_resource_grid cell_grid{carrier_cfgs};
    bwp_configuration       bwp_cfg{};
    bwp_cfg.scs  = ocudu::subcarrier_spacing::kHz15;
    bwp_cfg.crbs = {10, 30};

    ASSERT_TRUE(not cell_grid.collides(subcarrier_spacing::kHz15, {0, 14}, crb_interval{0, 52}));

    bwp_sch_grant_info grant{bwp_cfg, {2, 14}, {5, 10}};
    cell_grid.fill(grant);
    ASSERT_TRUE(cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {0, 14}, {0, 20}}));
    ASSERT_TRUE(not cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {0, 14}, {0, 5}}));
    ASSERT_TRUE(cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {2, 3}, {0, 6}}));
    ASSERT_TRUE(not cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {0, 2}, {0, 6}}));

    cell_grid.clear();
    ASSERT_TRUE(not cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {2, 3}, {0, 6}}));
  }

  // Wide BWP, 120 kHz case.
  {
    cell_slot_resource_grid cell_grid{carrier_cfgs};
    bwp_configuration       bwp_cfg{};
    bwp_cfg.scs  = ocudu::subcarrier_spacing::kHz120;
    bwp_cfg.crbs = {10, 275};

    ASSERT_TRUE(not cell_grid.collides(subcarrier_spacing::kHz120, {0, 14}, crb_interval{0, 265}));

    bwp_sch_grant_info grant{bwp_cfg, {2, 14}, {5, 200}};
    cell_grid.fill(grant);
    ASSERT_TRUE(cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {0, 14}, {10, 30}}));
    ASSERT_TRUE(not cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {0, 14}, {0, 5}}));
    ASSERT_TRUE(cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {2, 3}, {0, 30}}));
    ASSERT_TRUE(not cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {0, 2}, {0, 30}}));

    cell_grid.clear();
    ASSERT_TRUE(not cell_grid.collides(bwp_sch_grant_info{bwp_cfg, {2, 3}, {0, 30}}));
  }
}

/// Test allocation of resources in a cell resource grid
TEST(pusch_resource_allocation_test, test_all)
{
  scheduler_expert_config                 sched_cfg = config_helpers::make_default_scheduler_expert_config();
  test_helpers::test_sched_config_manager cfg_mng{sched_cfg};
  const cell_configuration&               cell_cfg =
      *cfg_mng.add_cell(sched_config_helper::make_default_sched_cell_configuration_request());
  cell_resource_allocator res_grid_alloc{cell_cfg};
  bwp_configuration       bwp_cfg{};
  bwp_cfg.crbs = {0, 52};
  bwp_cfg.scs  = cell_cfg.params.dl_cfg_common.freq_info_dl.scs_carrier_list[0].scs;

  slot_point sl_tx{0, 0};

  // Action 1: New slot
  res_grid_alloc.slot_indication(sl_tx);

  // Test: resource allocator operator[] returns a slot_allocator pointing at the correct slot
  ASSERT_EQ(sl_tx, res_grid_alloc.slot_tx());
  ASSERT_EQ(sl_tx, res_grid_alloc[0].slot);
  ASSERT_EQ(sl_tx + 1, res_grid_alloc[1].slot);

  // Test: No allocations made yet
  ASSERT_TRUE(res_grid_alloc[0].ul_res_grid.used_crbs(bwp_cfg, {0, 14}).none());
  ASSERT_TRUE(res_grid_alloc[0].result_reader().ul().puschs().empty());

  // Action 2: Allocate PUSCH grant in current slot_tx
  bwp_sch_grant_info ul_grant{bwp_cfg, {0, 14}, {1, 5}};
  res_grid_alloc[0].ul_res_grid.fill(ul_grant);
  ASSERT_NE(res_grid_alloc[0].result_builder().ul().puschs().emplace_back(), nullptr);

  // Test: Allocated PUSCH was registered in the cell resource grid for slot_tx
  ASSERT_TRUE(res_grid_alloc[0].ul_res_grid.used_crbs(bwp_cfg, {0, 14}).any());
  ASSERT_EQ(res_grid_alloc[0].ul_res_grid.used_crbs(bwp_cfg, {0, 14}).count(), ul_grant.prbs.length());
  ASSERT_EQ(1, res_grid_alloc[0].result_reader().ul().puschs().size());

  // Action 3: Allocate PUSCH grant in slot_tx + 1
  bwp_sch_grant_info ul_grant2{bwp_cfg, {0, 14}, {4, 20}};
  res_grid_alloc[1].ul_res_grid.fill(ul_grant2);
  ASSERT_NE(res_grid_alloc[1].result_builder().ul().puschs().emplace_back(), nullptr);

  // Test: Allocated PUSCH was registered in the cell resource grid for slot_tx + 1
  ASSERT_TRUE(res_grid_alloc[1].ul_res_grid.used_crbs(bwp_cfg, {0, 14}).any());
  ASSERT_EQ(res_grid_alloc[1].ul_res_grid.used_crbs(bwp_cfg, {0, 14}).count(), ul_grant2.prbs.length());
  ASSERT_EQ(1, res_grid_alloc[1].result_reader().ul().puschs().size());

  // Action 4: New slot
  res_grid_alloc.slot_indication(++sl_tx);

  // Test: Current slot_tx allocations match the ones done with "slot_tx + 1" in the previous slot
  ASSERT_EQ(res_grid_alloc[0].ul_res_grid.used_crbs(bwp_cfg, {0, 14}).count(), ul_grant2.prbs.length());
}

/// In an NTN cell, the UL grant of a DCI lands up to Koffset slots later, so the grid must keep the allocations of that
/// many more slots ahead without the ring wrapping onto them.
TEST(cell_resource_grid_test, ntn_grid_keeps_allocations_up_to_the_koffset_delayed_ul_slot)
{
  OCUDU_TEST_REQUIREMENTS("DU-NTN-TIM-1");

  // Cell-specific Koffset, in milliseconds, of LEO and GEO cells.
  for (unsigned koffset_ms : {16U, 240U, 480U}) {
    scheduler_expert_config                  sched_cfg = config_helpers::make_default_scheduler_expert_config();
    test_helpers::test_sched_config_manager  cfg_mng{sched_cfg};
    sched_cell_configuration_request_message cell_req =
        sched_config_helper::make_default_sched_cell_configuration_request();
    cell_req.ran.ntn_params.emplace();
    cell_req.ran.ntn_params->ntn_cfg.cell_specific_koffset = std::chrono::milliseconds{koffset_ms};
    const cell_configuration& cell_cfg                     = *cfg_mng.add_cell(cell_req);
    cell_resource_allocator   res_grid{cell_cfg};

    ASSERT_GT(cell_cfg.ntn_cs_koffset, 0U);
    const unsigned max_ul_delay = get_max_slot_ul_alloc_delay(cell_cfg.ntn_cs_koffset);
    // The ring must reach the furthest UL slot: operator[] accepts delays up to and including max_ul_delay.
    ASSERT_GE(res_grid.ring_size(), max_ul_delay) << fmt::format("Koffset={}ms", koffset_ms);

    // An allocation in the furthest UL slot stays in place until that slot is reached.
    slot_point sl_tx{0, 0};
    res_grid.slot_indication(sl_tx);
    const slot_point alloc_slot = sl_tx + max_ul_delay;
    ASSERT_NE(res_grid[alloc_slot].result_builder().ul().puschs().emplace_back(), nullptr);
    for (unsigned i = 0; i != max_ul_delay; ++i) {
      res_grid.slot_indication(++sl_tx);
      ASSERT_EQ(res_grid[alloc_slot].result_reader().ul().puschs().size(), 1U)
          << fmt::format("Koffset={}ms: allocation lost at slot {}", koffset_ms, sl_tx);
    }
    ASSERT_EQ(res_grid[0].slot, alloc_slot);
  }
}

/// Fills every pooled PDU list of a slot up to the capacity the slot gives it.
static void fill_pooled_lists(cell_slot_resource_allocator& slot_res, slot_point sl_tx, unsigned koffset_ms)
{
  const auto fail = [sl_tx, koffset_ms](const char* list) {
    return fmt::format("Koffset={}ms: pool of {} ran out at slot {}", koffset_ms, list, sl_tx);
  };

  auto dl = slot_res.result_builder().dl();
  for (unsigned i = 0; i != MAX_DL_PDCCH_PDUS_PER_SLOT; ++i) {
    ASSERT_NE(dl.dl_pdcchs().emplace_back(), nullptr) << fail("dl_pdcchs");
  }
  for (unsigned i = 0; i != MAX_UL_PDCCH_PDUS_PER_SLOT; ++i) {
    ASSERT_NE(dl.ul_pdcchs().emplace_back(), nullptr) << fail("ul_pdcchs");
  }
  for (unsigned i = 0; i != MAX_RAR_PDUS_PER_SLOT; ++i) {
    ASSERT_NE(dl.rar_grants().emplace_back(), nullptr) << fail("rar_grants");
  }
  for (unsigned i = 0; i != MAX_UE_PDUS_PER_SLOT; ++i) {
    ASSERT_NE(dl.ue_grants().emplace_back(), nullptr) << fail("ue_grants");
  }
  for (unsigned i = 0; i != MAX_CSI_RS_PDUS_PER_SLOT; ++i) {
    ASSERT_NE(dl.csi_rs().emplace_back(), nullptr) << fail("csi_rs");
  }
  auto ul = slot_res.result_builder().ul();
  for (unsigned i = 0; i != MAX_PUSCH_PDUS_PER_SLOT; ++i) {
    ASSERT_NE(ul.puschs().emplace_back(), nullptr) << fail("puschs");
  }
}

TEST(cell_resource_grid_test, pools_hold_every_slot_the_grid_can_reach_filled_to_capacity)
{
  OCUDU_TEST_REQUIREMENTS("DU-NTN-TIM-1");

  // Non-NTN, then the cell-specific Koffset, in milliseconds, of LEO and GEO cells.
  for (unsigned koffset_ms : {0U, 16U, 240U}) {
    scheduler_expert_config                  sched_cfg = config_helpers::make_default_scheduler_expert_config();
    test_helpers::test_sched_config_manager  cfg_mng{sched_cfg};
    sched_cell_configuration_request_message cell_req =
        sched_config_helper::make_default_sched_cell_configuration_request();
    if (koffset_ms != 0) {
      cell_req.ran.ntn_params.emplace();
      cell_req.ran.ntn_params->ntn_cfg.cell_specific_koffset = std::chrono::milliseconds{koffset_ms};
    }
    const cell_configuration& cell_cfg = *cfg_mng.add_cell(cell_req);
    cell_resource_allocator   res_grid{cell_cfg};

    const unsigned max_ul_delay = get_max_slot_ul_alloc_delay(cell_cfg.ntn_cs_koffset);

    slot_point sl_tx{0, 0};
    res_grid.slot_indication(sl_tx);

    // Every slot the grid can reach right now holds a full slot worth of PDUs.
    for (unsigned d = 0; d <= max_ul_delay; ++d) {
      ASSERT_NO_FATAL_FAILURE(fill_pooled_lists(res_grid[d], sl_tx + d, koffset_ms));
    }

    // Advancing past the history window keeps the furthest slot reachable and full, so the pools must cover the
    // history and the allocation delay at the same time.
    for (unsigned i = 0; i != cell_resource_allocator::RING_MAX_HISTORY_SIZE + 1; ++i) {
      res_grid.slot_indication(++sl_tx);
      ASSERT_NO_FATAL_FAILURE(fill_pooled_lists(res_grid[max_ul_delay], sl_tx + max_ul_delay, koffset_ms));
    }
  }
}

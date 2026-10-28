// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Unit tests for configured_grant_type2_rrm. Verifies that the UE admission cap matches the size of the Type-2
/// CG resource pool held by the scheduler, that UEs are refused once the cap is reached, and that removing a UE frees a
/// slot for a new one.

#include "lib/scheduler/configured_grant/cg_type2_resource_manager.h"
#include "tests/test_doubles/scheduler/cell_config_builder_profiles.h"
#include "tests/unittests/scheduler/test_utils/config_generators.h"
#include "ocudu/du/du_cell_config_helpers.h"
#include "ocudu/scheduler/config/cg_builder_params.h"
#include "ocudu/scheduler/config/ran_cell_config_helper.h"
#include "ocudu/scheduler/config/serving_cell_config_factory.h"
#include "ocudu/scheduler/rrm/configured_grant_type2_rrm.h"
#include <gtest/gtest.h>

using namespace ocudu;

namespace {

// ---- Test parameters ----

struct cg_type2_test_params {
  // If set, it's TDD and the value is the number of full-UL slots in the TDD pattern; otherwise FDD.
  std::optional<unsigned> nof_ul_slots;
  // Number of CRBs the cell reserves for CG. It bounds the number of CG resources that fit side by side in frequency.
  unsigned max_nof_cell_cg_rbs;
};

std::ostream& operator<<(std::ostream& out, const cg_type2_test_params& p)
{
  if (p.nof_ul_slots.has_value()) {
    out << "TDD_ul_slots_" << p.nof_ul_slots.value();
  } else {
    out << "FDD";
  }
  return out << "_cg_rbs_" << p.max_nof_cell_cg_rbs;
}

// ---- Test fixture ----

class configured_grant_type2_rrm_test : public ::testing::TestWithParam<cg_type2_test_params>
{
protected:
  configured_grant_type2_rrm_test() : cfg_mng(make_builder_params()), cell_cfg(*cfg_mng.add_cell(make_cell_req()))
  {
    cg_res_mng.add_cell(to_du_cell_index(0), cell_cfg.params);
  }

  static cell_config_builder_params make_builder_params()
  {
    const bool                 is_tdd = GetParam().nof_ul_slots.has_value();
    cell_config_builder_params params =
        cell_config_builder_profiles::create(is_tdd ? duplex_mode::TDD : duplex_mode::FDD);
    if (is_tdd) {
      auto&              tdd_cfg    = params.tdd_ul_dl_cfg_common.emplace();
      const unsigned     nof_ul_sl  = GetParam().nof_ul_slots.value();
      constexpr unsigned nof_ul_sym = 0U;
      const unsigned     nof_dl_sl  = 10U - nof_ul_sl - 1U;
      constexpr unsigned nof_dl_sym = 10U;
      tdd_cfg.pattern1              = {nof_dl_sl + 1U + nof_ul_sl, nof_dl_sl, nof_dl_sym, nof_ul_sl, nof_ul_sym};
      tdd_cfg.ref_scs               = subcarrier_spacing::kHz30;
    }
    return params;
  }

  sched_cell_configuration_request_message make_cell_req() const
  {
    auto req                = cfg_mng.get_default_cell_config_request();
    req.ran.init_bwp.cg_cfg = cg_builder_params{.periodicity         = cg_configuration::periodicity_t::sl40,
                                                .type                = cg_builder_params::cg_type::type2,
                                                .max_nof_cell_cg_rbs = GetParam().max_nof_cell_cg_rbs};
    return req;
  }

  // Creates a ue_cell_config for a new UE PCell on the given cell, with the CG config reset so that alloc_resources
  // fills it fresh.
  ue_cell_config make_ue_cell_cfg(du_cell_index_t cell_index = to_du_cell_index(0)) const
  {
    ue_cell_config ue_cell_cfg           = config_helpers::make_default_ue_cell_config(cell_cfg.params);
    ue_cell_cfg.serv_cell_cfg.cell_index = cell_index;
    ue_cell_cfg.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.reset();
    return ue_cell_cfg;
  }

  // Creates a ue_cell_config for a new UE PCell and calls build_ue_cg_config.
  // Returns the ue_cell_config if the allocation succeeded, nullopt otherwise.
  std::optional<ue_cell_config> add_ue(du_ue_index_t ue_idx)
  {
    ue_cell_config ue_cell_cfg = make_ue_cell_cfg();

    if (not cg_res_mng.build_ue_cg_config(ue_cell_cfg)) {
      return std::nullopt;
    }

    ues.emplace(ue_idx, ue_cell_cfg);
    return ue_cell_cfg;
  }

  void rem_ue(du_ue_index_t ue_idx)
  {
    ocudu_assert(ues.contains(ue_idx), "UE not present");
    cg_res_mng.reset_ue_cg_config(ues[ue_idx]);
    ues.erase(ue_idx);
  }

  // Adds UEs to cell 0 until the resource manager refuses one, and returns how many were admitted. The UEs are kept
  // in \c ues, so that they can be removed later on.
  unsigned fill_cell()
  {
    unsigned nof_allocated = 0;
    for (unsigned i = 0; i != MAX_NOF_DU_UES; ++i) {
      if (not add_ue(to_du_ue_index(i)).has_value()) {
        break;
      }
      ++nof_allocated;
    }
    return nof_allocated;
  }

  // Number of UEs the cell can host, taken from the size of the resource pool the scheduler builds for the same cell.
  unsigned cell_capacity() const { return cg_type2_resource_manager(cell_cfg).get_nof_free_resources(); }

  // Same, for an arbitrary cell and without keeping the UEs around.
  unsigned admit_until_refused(du_cell_index_t cell_index)
  {
    unsigned nof_allocated = 0;
    for (unsigned i = 0; i != MAX_NOF_DU_UES; ++i) {
      ue_cell_config ue_cell_cfg = make_ue_cell_cfg(cell_index);
      if (not cg_res_mng.build_ue_cg_config(ue_cell_cfg) or
          not ue_cell_cfg.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.has_value()) {
        break;
      }
      ++nof_allocated;
    }
    return nof_allocated;
  }

  test_helpers::test_sched_config_manager       cfg_mng;
  const cell_configuration&                     cell_cfg;
  configured_grant_type2_rrm                    cg_res_mng;
  slotted_array<ue_cell_config, MAX_NOF_DU_UES> ues;
};

// ---- Tests ----

/// The number of UEs the resource manager admits must match the number of resources the scheduler's
/// cg_type2_resource_manager builds for the same cell: a UE admitted beyond that would never get a CG activated.
TEST_P(configured_grant_type2_rrm_test, ue_capacity_matches_the_scheduler_resource_pool)
{
  const cg_type2_resource_manager sched_res_mng(cell_cfg);
  ASSERT_GT(sched_res_mng.get_nof_free_resources(), 0U) << "no CG Type 2 resource was built for this cell";

  EXPECT_EQ(fill_cell(), sched_res_mng.get_nof_free_resources());
}

/// Every UE admitted up to the cap gets a fully populated Type-2 CG configuration.
TEST_P(configured_grant_type2_rrm_test, admitted_ues_get_a_type2_cg_config)
{
  auto ue = add_ue(to_du_ue_index(0));
  ASSERT_TRUE(ue.has_value()) << "CG allocation failed for a single UE";

  ASSERT_TRUE(ue->serv_cell_cfg.ul_config.has_value());
  ASSERT_TRUE(ue->serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.has_value());
  const auto& cg_cfg = ue->serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.value();
  EXPECT_EQ(cg_cfg.periodicity, cg_configuration::periodicity_t::sl40);
  // Type 2 CG: the grant is given by DCI at activation, not by RRC.
  EXPECT_FALSE(cg_cfg.rrc_configured_ul_grant_cfg.has_value());
}

/// Once the cap is reached, the next UE is refused and its config is left untouched.
TEST_P(configured_grant_type2_rrm_test, ue_is_refused_once_the_cell_is_full)
{
  const unsigned nof_allocated = fill_cell();
  ASSERT_GT(nof_allocated, 0U);

  ue_cell_config extra_ue = make_ue_cell_cfg();

  EXPECT_FALSE(cg_res_mng.build_ue_cg_config(extra_ue)) << "UE admitted beyond the cell CG capacity";
  EXPECT_FALSE(extra_ue.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.has_value())
      << "a refused UE must be left without a CG configuration";
}

/// Allocating twice for the same UE must consume one slot, not two. An overcount would silently cost the cell one UE
/// of capacity for good, so the second call has to be a no-op rather than a second allocation.
TEST_P(configured_grant_type2_rrm_test, allocating_twice_for_the_same_ue_consumes_one_slot)
{
  ue_cell_config ue_cell_cfg = make_ue_cell_cfg();
  ASSERT_TRUE(cg_res_mng.build_ue_cg_config(ue_cell_cfg));
  const cg_configuration cg_cfg_after_first_alloc = ue_cell_cfg.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.value();

  EXPECT_TRUE(cg_res_mng.build_ue_cg_config(ue_cell_cfg)) << "re-allocating for an already configured UE must succeed";
  EXPECT_EQ(ue_cell_cfg.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.value(), cg_cfg_after_first_alloc)
      << "the CG configuration of the UE must be left untouched";

  // The cell must have room for its remaining capacity, i.e. the second call consumed nothing.
  EXPECT_EQ(admit_until_refused(to_du_cell_index(0)), cell_capacity() - 1);
}

/// Removing a UE must give its slot back, and only one slot.
TEST_P(configured_grant_type2_rrm_test, removing_a_ue_frees_exactly_one_slot)
{
  const unsigned nof_allocated = fill_cell();
  ASSERT_GT(nof_allocated, 0U);
  ASSERT_FALSE(add_ue(to_du_ue_index(nof_allocated)).has_value());

  rem_ue(to_du_ue_index(0));

  EXPECT_TRUE(add_ue(to_du_ue_index(nof_allocated)).has_value())
      << "expected the freed CG resource to be handed to a new UE";
  EXPECT_FALSE(add_ue(to_du_ue_index(nof_allocated + 1)).has_value()) << "removing one UE must free one slot only";
}

/// Deallocating a UE that holds no CG configuration must not give a slot back.
TEST_P(configured_grant_type2_rrm_test, dealloc_of_a_ue_without_cg_is_a_no_op)
{
  const unsigned nof_allocated = fill_cell();
  ASSERT_GT(nof_allocated, 0U);

  ue_cell_config ue_without_cg = make_ue_cell_cfg();
  cg_res_mng.reset_ue_cg_config(ue_without_cg);

  EXPECT_FALSE(add_ue(to_du_ue_index(nof_allocated)).has_value())
      << "deallocating a UE without CG must not free a slot";
}

/// Cells that were never registered in the resource manager are left alone, whatever the UE count of the other cells.
TEST_P(configured_grant_type2_rrm_test, unregistered_cell_is_skipped)
{
  ue_cell_config ue_cell_cfg = make_ue_cell_cfg(to_du_cell_index(1));

  EXPECT_TRUE(cg_res_mng.build_ue_cg_config(ue_cell_cfg));
  EXPECT_FALSE(ue_cell_cfg.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.has_value());
}

/// Deallocating a UE of a cell the resource manager does not know about must leave the registered cells untouched.
TEST_P(configured_grant_type2_rrm_test, dealloc_on_an_unregistered_cell_is_a_no_op)
{
  const unsigned nof_allocated = fill_cell();
  ASSERT_GT(nof_allocated, 0U);

  // A UE that does hold a CG config, so that the deallocation is not short-circuited by the missing config, but whose
  // cell was never registered.
  ue_cell_config ue_on_other_cell = make_ue_cell_cfg(to_du_cell_index(1));
  ue_on_other_cell.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.emplace(
      config_helpers::make_default_cell_cg_config(cell_cfg.params));
  cg_res_mng.reset_ue_cg_config(ue_on_other_cell);

  EXPECT_FALSE(add_ue(to_du_ue_index(nof_allocated)).has_value())
      << "the UE count of cell 0 was decremented by a deallocation on another cell";
}

/// A removed cell is skipped like one that was never registered, and re-adding it rebuilds a context with a clean UE
/// count, so the whole capacity of the cell is available again.
TEST_P(configured_grant_type2_rrm_test, removed_cell_is_skipped_and_re_adding_it_resets_the_ue_count)
{
  const unsigned nof_allocated = fill_cell();
  ASSERT_GT(nof_allocated, 0U);
  ASSERT_FALSE(add_ue(to_du_ue_index(nof_allocated)).has_value()) << "the cell should be full";

  cg_res_mng.rem_cell(to_du_cell_index(0));

  // The cell is no longer registered, so the allocation is skipped: it reports success without setting a CG config.
  ue_cell_config ue_on_removed_cell = make_ue_cell_cfg();
  EXPECT_TRUE(cg_res_mng.build_ue_cg_config(ue_on_removed_cell));
  EXPECT_FALSE(ue_on_removed_cell.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.has_value());

  // The UEs of the previous incarnation of the cell no longer count against it.
  cg_res_mng.add_cell(to_du_cell_index(0), cell_cfg.params);
  EXPECT_EQ(admit_until_refused(to_du_cell_index(0)), nof_allocated);
}

/// Each registered cell keeps its own UE count: exhausting one must not eat into the capacity of another.
TEST_P(configured_grant_type2_rrm_test, cells_keep_independent_ue_counts)
{
  // Register a second cell with the same configuration, hence the same capacity.
  cg_res_mng.add_cell(to_du_cell_index(1), cell_cfg.params);

  const unsigned nof_allocated = fill_cell();
  ASSERT_GT(nof_allocated, 0U);
  ASSERT_FALSE(add_ue(to_du_ue_index(nof_allocated)).has_value()) << "cell 0 should be full";

  EXPECT_EQ(admit_until_refused(to_du_cell_index(1)), nof_allocated)
      << "filling cell 0 consumed the capacity of cell 1";
}

// ---- Parameterization ----

INSTANTIATE_TEST_SUITE_P(configured_grant_type2_rrm,
                         configured_grant_type2_rrm_test,
                         ::testing::Values(
                             // FDD, one CG resource per slot offset.
                             cg_type2_test_params{.max_nof_cell_cg_rbs = 10},
                             // FDD, several CG resources side by side in frequency.
                             cg_type2_test_params{.max_nof_cell_cg_rbs = 40},
                             // TDD DL-heavy: 7 DL + 1 partial (10S DL, 0S UL) + 2 UL.
                             cg_type2_test_params{.nof_ul_slots = 2, .max_nof_cell_cg_rbs = 40},
                             // TDD UL-heavy: 3 DL + 1 partial (10S DL, 0S UL) + 6 UL.
                             cg_type2_test_params{.nof_ul_slots = 6, .max_nof_cell_cg_rbs = 40}),
                         [](const ::testing::TestParamInfo<cg_type2_test_params>& param_info) {
                           std::ostringstream os;
                           os << param_info.param;
                           return os.str();
                         });

// ---- Non-parameterized tests ----

/// A cell can have CG configured and still have no resource to hand out, when no slot offset survives the TDD and
/// PRACH patterns. Its capacity is zero, so the very first UE must be refused.
TEST(configured_grant_type2_rrm_capacity_test, cell_without_usable_cg_offset_admits_no_ue)
{
  // TDD 7 DL + 1 partial + 2 UL (10-slot period) against a 16-slot CG period: gcd(16, 10) = 2, so each offset recurs
  // on 5 different TDD slot indices and none of them can be a full-UL slot at every occurrence.
  cell_config_builder_params cell_params = cell_config_builder_profiles::create(duplex_mode::TDD);
  auto&                      tdd_cfg     = cell_params.tdd_ul_dl_cfg_common.emplace();
  tdd_cfg.pattern1                       = {10, 7, 10, 2, 0};
  tdd_cfg.ref_scs                        = subcarrier_spacing::kHz30;

  odu::du_cell_config du_cfg = config_helpers::make_default_du_cell_config(cell_params);
  du_cfg.ran.init_bwp.cg_cfg = cg_builder_params{.periodicity = cg_configuration::periodicity_t::sl16,
                                                 .type        = cg_builder_params::cg_type::type2};

  configured_grant_type2_rrm res_mng;
  res_mng.add_cell(to_du_cell_index(0), du_cfg.ran);

  ue_cell_config ue_cell_cfg = config_helpers::make_default_ue_cell_config(du_cfg.ran);
  ue_cell_cfg.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.reset();

  EXPECT_FALSE(res_mng.build_ue_cg_config(ue_cell_cfg)) << "a UE was admitted to a cell with no usable CG slot offset";
  EXPECT_FALSE(ue_cell_cfg.serv_cell_cfg.ul_config->init_ul_bwp.cg_cfg.has_value());
}

} // namespace

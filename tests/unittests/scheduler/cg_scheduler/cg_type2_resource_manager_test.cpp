// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Unit tests for cg_type2_resource_manager. Verifies that the CG Type 2 resources it hands out are mutually
/// orthogonal in frequency, and that they are returned to the pool on deallocation.

#include "lib/scheduler/configured_grant/cg_type2_resource_manager.h"
#include "tests/unittests/scheduler/test_utils/config_generators.h"
#include "ocudu/scheduler/config/cg_builder_params.h"
#include <gtest/gtest.h>

using namespace ocudu;

namespace {

class cg_type2_resource_manager_test : public ::testing::Test
{
protected:
  cg_type2_resource_manager_test() :
    cfg_mng(builder_params), cell_cfg(*cfg_mng.add_cell(make_cell_req())), res_mng(cell_cfg)
  {
  }

  sched_cell_configuration_request_message make_cell_req() const
  {
    auto req = cfg_mng.get_default_cell_config_request();
    // max_nof_cell_cg_rbs defaults to 10, which yields a single CG resource per slot offset and could not expose an
    // overlap between two of them. Widen the CG band so that several resources fit side by side.
    req.ran.init_bwp.cg_cfg = cg_builder_params{.periodicity         = cg_configuration::periodicity_t::sl40,
                                                .type                = cg_builder_params::cg_type::type2,
                                                .max_nof_cell_cg_rbs = 40};
    return req;
  }

  // Returns a new C-RNTI on each call. A UE holds at most one resource per slot offset, so the tests that drain the
  // pool must request each resource for a different UE.
  rnti_t next_rnti() { return to_rnti(0x4700 + nof_drain_ues++); }

  static constexpr rnti_t ue1 = to_rnti(0x4601);
  static constexpr rnti_t ue2 = to_rnti(0x4602);

  unsigned nof_drain_ues = 0;

  cell_config_builder_params              builder_params;
  test_helpers::test_sched_config_manager cfg_mng;
  const cell_configuration&               cell_cfg;
  cg_type2_resource_manager               res_mng;
};

/// Drains every CG resource available at one slot offset and checks that no two of them share a CRB. Two UEs
/// activated on the same slot offset are handed two different resource ids, so overlapping CRBs would put their CG
/// PUSCHes on top of each other.
TEST_F(cg_type2_resource_manager_test, resources_of_the_same_slot_offset_do_not_overlap_in_frequency)
{
  // Any offset carrying resources will do; find the first one.
  std::optional<cg_type2_resource> first;
  unsigned                         cg_offset = 0;
  for (; cg_offset != 40; ++cg_offset) {
    first = res_mng.request_resource(cg_offset, ue1);
    if (first.has_value()) {
      break;
    }
  }
  ASSERT_TRUE(first.has_value()) << "no CG Type 2 resource was available at any slot offset";

  std::vector<cg_type2_resource> allocated{first.value()};
  while (auto res = res_mng.request_resource(cg_offset, next_rnti())) {
    allocated.push_back(res.value());
  }
  ASSERT_GT(allocated.size(), 1U) << "only one CG resource per slot offset; test cannot detect an overlap";

  // Laying the resources side by side must not push the last one out of the UL BWP.
  const crb_interval ul_bwp_crbs = cell_cfg.params.ul_cfg_common.init_ul_bwp.generic_params.crbs;
  for (const cg_type2_resource& res : allocated) {
    EXPECT_TRUE(ul_bwp_crbs.contains(res.crbs))
        << "CG resource id=" << res.id << " with crbs=" << fmt::format("{}", res.crbs) << " falls outside the UL BWP "
        << fmt::format("{}", ul_bwp_crbs);
  }

  for (unsigned a = 0; a != allocated.size(); ++a) {
    for (unsigned b = a + 1; b != allocated.size(); ++b) {
      EXPECT_NE(allocated[a].id, allocated[b].id) << "the same resource id was handed out twice";
      EXPECT_FALSE(allocated[a].crbs.overlaps(allocated[b].crbs))
          << "CG resources id=" << allocated[a].id << " and id=" << allocated[b].id
          << " overlap in frequency: " << fmt::format("{}", allocated[a].crbs) << " vs "
          << fmt::format("{}", allocated[b].crbs);
    }
  }
}

/// The CRBs of a resource must not depend on the slot offset it was requested at: the id indexes one fixed CRB block.
TEST_F(cg_type2_resource_manager_test, resource_crbs_are_stable_across_slot_offsets)
{
  std::map<cg_type2_resource::res_id, crb_interval> crbs_of_id;
  for (unsigned cg_offset = 0; cg_offset != 40; ++cg_offset) {
    while (auto res = res_mng.request_resource(cg_offset, next_rnti())) {
      const auto it = crbs_of_id.find(res->id);
      if (it == crbs_of_id.end()) {
        crbs_of_id.emplace(res->id, res->crbs);
        continue;
      }
      EXPECT_EQ(it->second, res->crbs) << "resource id=" << res->id << " changed CRBs across slot offsets";
    }
  }
  EXPECT_FALSE(crbs_of_id.empty()) << "no CG Type 2 resources were available at any slot offset";
}

/// Cell deactivation drops the UEs holding a CG resource straight from the UE repository, without going through
/// rem_ue(), so reset() is the only thing that reclaims their resources. Without it the pool drains a little on every
/// deactivation cycle until no UE can be activated any more.
TEST_F(cg_type2_resource_manager_test, reset_returns_every_outstanding_resource_to_the_pool)
{
  const unsigned nof_free_at_start = res_mng.get_nof_free_resources();
  ASSERT_GT(nof_free_at_start, 0U) << "no CG Type 2 resources were available at any slot offset";

  // Drain the pool completely, as a cell holding activated CG UEs would.
  unsigned nof_allocated = 0;
  for (unsigned cg_offset = 0; cg_offset != 40; ++cg_offset) {
    while (res_mng.request_resource(cg_offset, next_rnti()).has_value()) {
      ++nof_allocated;
    }
  }
  ASSERT_EQ(nof_free_at_start, nof_allocated);
  ASSERT_EQ(0U, res_mng.get_nof_free_resources());

  res_mng.reset();

  ASSERT_EQ(nof_free_at_start, res_mng.get_nof_free_resources());

  // The reclaimed resources must be handed out again, at the same slot offsets and in the same number.
  unsigned nof_reallocated = 0;
  for (unsigned cg_offset = 0; cg_offset != 40; ++cg_offset) {
    while (res_mng.request_resource(cg_offset, next_rnti()).has_value()) {
      ++nof_reallocated;
    }
  }
  ASSERT_EQ(nof_allocated, nof_reallocated);
}

/// reset() on a pool that was only partially drained must not duplicate the resources that were still free.
TEST_F(cg_type2_resource_manager_test, reset_does_not_duplicate_the_resources_left_in_the_pool)
{
  const unsigned nof_free_at_start = res_mng.get_nof_free_resources();

  unsigned cg_offset = 0;
  for (; cg_offset != 40; ++cg_offset) {
    if (res_mng.request_resource(cg_offset, ue1).has_value()) {
      break;
    }
  }
  ASSERT_NE(40U, cg_offset) << "no CG Type 2 resource was available at any slot offset";
  ASSERT_EQ(nof_free_at_start - 1, res_mng.get_nof_free_resources());

  res_mng.reset();

  ASSERT_EQ(nof_free_at_start, res_mng.get_nof_free_resources());
}

/// A resource must not be returned by a UE that does not hold it: doing so would put its CRBs back in the pool while
/// the owner keeps transmitting on them, so a third UE would be handed the same RBs in the same slot.
TEST_F(cg_type2_resource_manager_test, resource_held_by_another_ue_is_not_deallocated)
{
  std::optional<cg_type2_resource> res;
  unsigned                         cg_offset = 0;
  for (; cg_offset != 40; ++cg_offset) {
    res = res_mng.request_resource(cg_offset, ue1);
    if (res.has_value()) {
      break;
    }
  }
  ASSERT_TRUE(res.has_value()) << "no CG Type 2 resource was available at any slot offset";

  const unsigned nof_free_while_held = res_mng.get_nof_free_resources();

  // ue2 does not hold this resource, so the deallocation must be rejected and the pool left untouched.
  res_mng.deallocate_resource(cg_offset, res->id, ue2);
  EXPECT_EQ(nof_free_while_held, res_mng.get_nof_free_resources());

  // The owner can still return it.
  res_mng.deallocate_resource(cg_offset, res->id, ue1);
  EXPECT_EQ(nof_free_while_held + 1, res_mng.get_nof_free_resources());
}

/// Returning a resource that is already free must not inflate the pool beyond its real size.
TEST_F(cg_type2_resource_manager_test, deallocating_a_free_resource_does_not_grow_the_pool)
{
  const unsigned nof_free_at_start = res_mng.get_nof_free_resources();

  std::optional<cg_type2_resource> res;
  unsigned                         cg_offset = 0;
  for (; cg_offset != 40; ++cg_offset) {
    res = res_mng.request_resource(cg_offset, ue1);
    if (res.has_value()) {
      break;
    }
  }
  ASSERT_TRUE(res.has_value()) << "no CG Type 2 resource was available at any slot offset";

  res_mng.deallocate_resource(cg_offset, res->id, ue1);
  ASSERT_EQ(nof_free_at_start, res_mng.get_nof_free_resources());

  // Second deallocation of the same resource: rejected, so the pool stays at its full size.
  res_mng.deallocate_resource(cg_offset, res->id, ue1);
  EXPECT_EQ(nof_free_at_start, res_mng.get_nof_free_resources());
}

/// A UE whose activation starts twice requests a resource twice at the same slot offset. It must get back the
/// resource it already holds: a second one would leak, as the UE only ever returns one.
TEST_F(cg_type2_resource_manager_test, repeated_request_by_the_same_ue_returns_the_resource_it_holds)
{
  std::optional<cg_type2_resource> res;
  unsigned                         cg_offset = 0;
  for (; cg_offset != 40; ++cg_offset) {
    res = res_mng.request_resource(cg_offset, ue1);
    if (res.has_value()) {
      break;
    }
  }
  ASSERT_TRUE(res.has_value()) << "no CG Type 2 resource was available at any slot offset";

  const unsigned nof_free_while_held = res_mng.get_nof_free_resources();

  const std::optional<cg_type2_resource> res2 = res_mng.request_resource(cg_offset, ue1);
  ASSERT_TRUE(res2.has_value());
  EXPECT_EQ(res->id, res2->id);
  EXPECT_EQ(res->crbs, res2->crbs);
  EXPECT_EQ(nof_free_while_held, res_mng.get_nof_free_resources());

  // A single deallocation returns the pool to its full size.
  res_mng.deallocate_resource(cg_offset, res->id, ue1);
  EXPECT_EQ(nof_free_while_held + 1, res_mng.get_nof_free_resources());
}

/// A deallocated resource must become available again at the same slot offset.
TEST_F(cg_type2_resource_manager_test, deallocated_resource_returns_to_the_pool)
{
  std::optional<cg_type2_resource> res;
  unsigned                         cg_offset = 0;
  for (; cg_offset != 40; ++cg_offset) {
    res = res_mng.request_resource(cg_offset, ue1);
    if (res.has_value()) {
      break;
    }
  }
  ASSERT_TRUE(res.has_value());

  const unsigned nof_free_after_alloc = res_mng.get_nof_free_resources();
  res_mng.deallocate_resource(cg_offset, res->id, ue1);
  EXPECT_EQ(res_mng.get_nof_free_resources(), nof_free_after_alloc + 1);
}

} // namespace

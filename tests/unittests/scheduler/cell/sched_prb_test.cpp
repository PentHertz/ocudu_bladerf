// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "lib/scheduler/cell/scheduler_prb.h"
#include "ocudu/scheduler/support/rb_helper.h"
#include "ocudu/support/test_utils.h"
#include <gtest/gtest.h>

using namespace ocudu;

TEST(prb_grant_test, test_all)
{
  // TEST: default ctor
  vrb_alloc grant;
  ASSERT_TRUE(grant.is_type1());
  ASSERT_TRUE(grant.type1().length() == 0);

  // TEST: assignment of VRB bitmap
  vrb_bitmap vrbs(52);
  vrbs.set(1);
  grant = vrbs;
  ASSERT_TRUE(grant.is_type0() and grant.nof_vrbs() == 1);

  // TEST: assignment of PRBs
  vrb_interval vrb_interv(2, 5);
  grant = vrb_interv;
  ASSERT_TRUE(grant.is_type1() and grant.nof_vrbs() == 3);

  // TEST: non-default ctor
  vrb_alloc grant2(vrb_interv), grant3(vrbs);
  ASSERT_TRUE(grant2.is_type1() and grant2.type1().length() == 3);
  ASSERT_TRUE(grant3.is_type0() and grant3.type0().count() == 1);

  // TEST: copy ctor
  vrb_alloc grant4(grant2), grant5(grant3);
  ASSERT_TRUE(grant4.is_type1() and grant4.type1().length() == 3);
  ASSERT_TRUE(grant5.is_type0() and grant5.type0().count() == 1);

  // TEST: copy assignment
  grant = grant3;
  ASSERT_TRUE(grant.is_type0() and grant.type0().count() == 1);
  grant = grant2;
  ASSERT_TRUE(grant.is_type1() and grant.type1().length() == 3);

  // TEST: formatting
  ASSERT_EQ("[2..5)", fmt::format("{}", grant2));
  ASSERT_EQ(fmt::format("{:i}", vrbs), fmt::format("{}", grant3));
}

TEST(convert_rbgs_to_vrbs_test, when_bwp_start_is_not_aligned_to_rbg_size_then_first_and_last_rbgs_are_shorter)
{
  // BWP of 100 PRBs starting at CRB 3, with P=8. The first RBG has 5 PRBs and the last RBG has 7 PRBs.
  const crb_interval     bwp_crbs{3, 103};
  const nominal_rbg_size P = get_nominal_rbg_size(bwp_crbs.length(), rbg_size::config1);
  ASSERT_EQ(P, nominal_rbg_size::P8);
  rbg_bitmap rbgs(get_nof_rbgs(bwp_crbs, P));
  ASSERT_EQ(rbgs.size(), 13);
  rbgs.set(0);
  rbgs.set(1);
  rbgs.set(12);

  vrb_bitmap expected(bwp_crbs.length());
  expected.fill(0, 13);
  expected.fill(93, 100);
  ASSERT_EQ(convert_rbgs_to_vrbs(rbgs, bwp_crbs, P), expected);
}

TEST(convert_vrbs_to_rbgs_test, when_any_vrb_of_an_rbg_is_set_then_the_rbg_is_set)
{
  // BWP of 100 PRBs starting at CRB 3, with P=8. RBG 0 spans VRBs [0, 5) and RBG 1 spans VRBs [5, 13).
  const crb_interval     bwp_crbs{3, 103};
  const nominal_rbg_size P = get_nominal_rbg_size(bwp_crbs.length(), rbg_size::config1);
  vrb_bitmap             vrbs(bwp_crbs.length());
  vrbs.set(4);
  vrbs.set(99);

  rbg_bitmap expected(get_nof_rbgs(bwp_crbs, P));
  expected.set(0);
  expected.set(12);
  ASSERT_EQ(convert_vrbs_to_rbgs(vrbs, bwp_crbs, P), expected);
  ASSERT_EQ(convert_vrbs_to_rbgs(convert_rbgs_to_vrbs(expected, bwp_crbs, P), bwp_crbs, P), expected);
}

TEST(find_free_rbgs_with_nof_vrbs_test, when_short_first_rbg_is_free_then_full_rbgs_are_still_selected)
{
  // BWP of 50 PRBs starting at CRB 2, with P=4. RBG sizes are 2, 4, ..., 4.
  const crb_interval     bwp_crbs{2, 52};
  const nominal_rbg_size P = nominal_rbg_size::P4;
  rbg_bitmap             used_rbgs(get_nof_rbgs(bwp_crbs, P));
  used_rbgs.fill(2, used_rbgs.size());

  // A greedy selection takes the 2-VRB first RBG and then cannot reach 4 VRBs.
  rbg_bitmap expected(used_rbgs.size());
  expected.set(1);
  ASSERT_EQ(find_free_rbgs_with_nof_vrbs(used_rbgs, bwp_crbs, P, 4), expected);
}

TEST(find_free_rbgs_with_nof_vrbs_test, when_short_edge_rbgs_are_needed_then_they_are_selected)
{
  // BWP of 49 PRBs starting at CRB 2, with P=4. RBG sizes are 2, 4, ..., 4, 3.
  const crb_interval     bwp_crbs{2, 51};
  const nominal_rbg_size P = nominal_rbg_size::P4;
  const rbg_bitmap       used_rbgs(get_nof_rbgs(bwp_crbs, P));

  rbg_bitmap expected(used_rbgs.size());
  expected.set(0);
  expected.set(1);
  expected.set(used_rbgs.size() - 1);
  const rbg_bitmap rbgs = find_free_rbgs_with_nof_vrbs(used_rbgs, bwp_crbs, P, 9);
  ASSERT_EQ(rbgs, expected);
  ASSERT_EQ(convert_rbgs_to_vrbs(rbgs, bwp_crbs, P).count(), 9);
}

TEST(find_free_rbgs_with_nof_vrbs_test, when_no_free_rbgs_add_up_to_nof_vrbs_then_no_rbg_is_selected)
{
  // BWP of 50 PRBs starting at CRB 2, with P=4. RBG sizes are 2, 4, ..., 4.
  const crb_interval     bwp_crbs{2, 52};
  const nominal_rbg_size P = nominal_rbg_size::P4;
  rbg_bitmap             used_rbgs(get_nof_rbgs(bwp_crbs, P));
  used_rbgs.set(0);

  ASSERT_TRUE(find_free_rbgs_with_nof_vrbs(used_rbgs, bwp_crbs, P, 6).none());
  ASSERT_TRUE(find_free_rbgs_with_nof_vrbs(used_rbgs, bwp_crbs, P, 3).none());
}

TEST(bwp_rb_bitmap, when_bwp_start_is_not_aligned_to_rbg_size_then_rbgs_map_to_shorter_edge_rbgs)
{
  bwp_rb_bitmap rb_bitmap(crb_interval{3, 103}, rbg_size::config1);
  ASSERT_EQ(rb_bitmap.P(), 8);
  ASSERT_EQ(rb_bitmap.nof_rbgs(), 13);

  rbg_bitmap rbgs(rb_bitmap.nof_rbgs());
  rbgs.set(0);
  rbgs.set(12);
  rb_bitmap |= rbgs;

  prb_bitmap expected(rb_bitmap.nof_prbs());
  expected.fill(0, 5);
  expected.fill(93, 100);
  ASSERT_EQ(rb_bitmap.prbs(), expected);
  ASSERT_EQ(rb_bitmap.rbgs(), rbgs);
}

TEST(bwp_rb_bitmap, test_all)
{
  bwp_rb_bitmap rb_bitmap(crb_interval{0, 275}, rbg_size::config1);

  ASSERT_TRUE(rb_bitmap.P() == 16);
  ASSERT_TRUE(rb_bitmap.rbgs().none());
  ASSERT_TRUE(rb_bitmap.prbs().none());
  ASSERT_TRUE(rb_bitmap.prbs().size() == 275 and rb_bitmap.nof_prbs() == 275);
  ASSERT_TRUE(rb_bitmap.rbgs().size() == 18 and rb_bitmap.nof_rbgs() == 18);

  rb_bitmap.add(prb_interval{0, 1});
  ASSERT_TRUE(rb_bitmap.prbs().count() == 1 and rb_bitmap.prbs().test(0));
  ASSERT_TRUE(rb_bitmap.rbgs().count() == 1 and rb_bitmap.rbgs().test(0));
  rb_bitmap.add(prb_interval{2, 4});
  ASSERT_TRUE(rb_bitmap.prbs().count() == 3 and rb_bitmap.prbs().test(2) and not rb_bitmap.prbs().test(1));
  ASSERT_TRUE(rb_bitmap.rbgs().count() == 1 and rb_bitmap.rbgs().test(0));

  prb_bitmap prbs(rb_bitmap.nof_prbs());
  prbs.set(1);
  prbs.set(2);
  prbs.set(15);
  rb_bitmap.add(prbs);
  ASSERT_TRUE(rb_bitmap.prbs().count() == 5 and rb_bitmap.prbs().test(1) and rb_bitmap.prbs().test(15));
  ASSERT_TRUE(rb_bitmap.rbgs().count() == 1 and rb_bitmap.rbgs().test(0));
  prbs.set(16);
  rb_bitmap |= prbs;
  ASSERT_TRUE(rb_bitmap.prbs().count() == 6 and rb_bitmap.prbs().test(16));
  ASSERT_TRUE(rb_bitmap.rbgs().count() == 2 and rb_bitmap.rbgs().test(1));

  rbg_bitmap rbgs(rb_bitmap.nof_rbgs());
  rbgs.set(3);
  rbgs.set(17);
  rb_bitmap |= rbgs;
  ASSERT_TRUE(rb_bitmap.prbs().count() == (6 + 16 + 3) and rb_bitmap.prbs().test(rb_bitmap.nof_prbs() - 1));
  ASSERT_TRUE(rb_bitmap.rbgs().count() == 4 and rb_bitmap.rbgs().test(3) and rb_bitmap.rbgs().test(17));
  rbgs.set(0);
  rb_bitmap |= rbgs;
  ASSERT_TRUE(rb_bitmap.prbs().count() == (16 + 1 + 16 + 3) and rb_bitmap.prbs().test(rb_bitmap.nof_prbs() - 1));
  ASSERT_TRUE(rb_bitmap.rbgs().count() == 4 and rb_bitmap.rbgs().test(3) and rb_bitmap.rbgs().test(17));

  // TEST: collides operator
  ASSERT_TRUE(rb_bitmap.collides(convert_rbgs_to_vrbs(rbgs, crb_interval{0, 275}, nominal_rbg_size::P16)));
  ASSERT_TRUE(rb_bitmap.collides(vrb_interval{0, 2}));
}

TEST(bwp_rb_bitmap, search)
{
  bwp_rb_bitmap rb_bitmap(crb_interval{0, 275}, rbg_size::config1);

  prb_interval prbs = rb_helper::find_empty_interval_of_length(rb_bitmap.prbs(), 5);
  ASSERT_TRUE(prbs == prb_interval(0, 5));
  prbs = rb_helper::find_empty_interval_of_length(rb_bitmap.prbs(), rb_bitmap.prbs().size());
  ASSERT_TRUE(prbs == prb_interval(0, rb_bitmap.prbs().size()));

  rb_bitmap |= prb_interval{1, 5};
  prbs = rb_helper::find_empty_interval_of_length(rb_bitmap.prbs(), rb_bitmap.prbs().size());
  ASSERT_TRUE(prbs == prb_interval(5, rb_bitmap.prbs().size()));

  rb_bitmap |= prb_interval{16, 32};
  prbs = rb_helper::find_empty_interval_of_length(rb_bitmap.prbs(), rb_bitmap.prbs().size());
  ASSERT_TRUE(prbs == prb_interval(32, rb_bitmap.prbs().size()));

  rb_bitmap |= prb_interval{270, 275};
  prbs = rb_helper::find_empty_interval_of_length(rb_bitmap.prbs(), rb_bitmap.prbs().size());
  ASSERT_TRUE(prbs == prb_interval(32, 270));
  prbs = rb_helper::find_empty_interval_of_length(rb_bitmap.prbs(), 1);
  ASSERT_TRUE(prbs == prb_interval(0, 1));
  prbs = rb_helper::find_empty_interval_of_length(rb_bitmap.prbs(), 5);
  ASSERT_TRUE(prbs == prb_interval(5, 10));
}

TEST(get_nof_rbgs_test, when_bwp_does_not_start_at_rbg_boundary_then_nof_rbgs_can_exceed_bwp_size_over_p)
{
  // BWP of 144 PRBs starting at CRB 7, with P=8.
  const crb_interval bwp_crbs{7, 151};
  ASSERT_EQ(get_nominal_rbg_size(bwp_crbs.length(), rbg_size::config1), nominal_rbg_size::P8);
  ASSERT_EQ(get_nof_rbgs(bwp_crbs, rbg_size::config1), MAX_NOF_RBGS);
}

TEST(prb_to_crb_test, when_prb_bitmap_is_converted_then_crbs_are_offset_by_bwp_start)
{
  const crb_interval bwp_crbs{70, 140};
  prb_bitmap         prbs(bwp_crbs.length());
  prbs.set(0);
  prbs.fill(10, 20);
  prbs.set(bwp_crbs.length() - 1);

  crb_bitmap expected(bwp_crbs.stop());
  expected.set(70);
  expected.fill(80, 90);
  expected.set(139);
  ASSERT_EQ(prb_to_crb(bwp_crbs, prbs), expected);
}

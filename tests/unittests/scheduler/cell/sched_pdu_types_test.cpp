// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/scheduler/result/detail/sched_pdu_types.h"
#include <gtest/gtest.h>

using namespace ocudu;
using namespace ocudu::sched_pdu_detail;

namespace {

/// Number of PDUs a list of the test holds.
constexpr size_t test_list_capacity = 4;
/// Number of lists the pool of the test serves.
constexpr size_t test_nof_lists = 2;

/// PDU of the test, carrying a value that identifies it.
struct test_pdu {
  unsigned value = 0;
};

using test_list = pooled_pdu_list<test_pdu>;

/// Pool set holding the single pool of the test.
class test_pool_set
{
public:
  explicit test_pool_set(size_t nof_pdus) : pdu_pool(nof_pdus) {}

  template <typename T>
  free_list_object_pool<T>& get()
  {
    return pdu_pool;
  }

private:
  free_list_object_pool<test_pdu> pdu_pool;
};

class pooled_list_test : public ::testing::Test
{
protected:
  pooled_list_test() : pools(test_list_capacity * test_nof_lists)
  {
    first_list.reserve(test_list_capacity);
    second_list.reserve(test_list_capacity);
  }

  /// Appends PDUs numbered from zero to a list, asserting that each one fits.
  void fill(pooled_list_builder<test_pdu> builder, unsigned nof_pdus)
  {
    for (unsigned i = 0; i != nof_pdus; ++i) {
      test_pdu* pdu = builder.emplace_back();
      ASSERT_NE(pdu, nullptr);
      pdu->value = i;
    }
  }

  test_pool_set pools;
  test_list     first_list;
  test_list     second_list;

  pooled_list_builder<test_pdu> first{first_list, pools};
  pooled_list_builder<test_pdu> second{second_list, pools};
};

} // namespace

// A pooled list resolves to the pooled builder and view, an inline one to the inline builder and a plain span.
static_assert(std::is_same_v<get_list_builder<test_list>::type, pooled_list_builder<test_pdu>>,
              "Pooled list resolved to the wrong builder");
static_assert(std::is_same_v<get_list_view<test_list>::type, pooled_list_view<test_pdu>>,
              "Pooled list resolved to the wrong view");
static_assert(std::is_same_v<get_list_builder<static_vector<test_pdu, test_list_capacity>>::type,
                             static_list_builder<test_pdu, test_list_capacity>>,
              "Inline list resolved to the wrong builder");
static_assert(std::is_same_v<get_list_view<static_vector<test_pdu, test_list_capacity>>::type, span<const test_pdu>>,
              "Inline list resolved to the wrong view");

TEST_F(pooled_list_test, when_list_is_created_then_it_is_empty)
{
  ASSERT_TRUE(first.empty());
  ASSERT_EQ(first.size(), 0);
  ASSERT_FALSE(first.full());
}

TEST_F(pooled_list_test, when_pdu_is_emplaced_then_the_list_holds_it)
{
  test_pdu* pdu = first.emplace_back();
  ASSERT_NE(pdu, nullptr);
  pdu->value = 7;

  ASSERT_EQ(first.size(), 1);
  ASSERT_EQ(first[0].value, 7);
  ASSERT_EQ(first.back().value, 7);
  ASSERT_EQ(first.view()[0].value, 7);
}

TEST_F(pooled_list_test, builder_iteration_visits_every_pdu_in_order)
{
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));

  unsigned count = 0;
  for (test_pdu& pdu : first) {
    ASSERT_EQ(pdu.value, count);
    ++count;
  }
  ASSERT_EQ(count, test_list_capacity);
}

TEST_F(pooled_list_test, view_iteration_visits_every_pdu_in_order)
{
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));

  unsigned count = 0;
  for (const test_pdu& pdu : first.view()) {
    ASSERT_EQ(pdu.value, count);
    ++count;
  }
  ASSERT_EQ(count, test_list_capacity);
}

TEST_F(pooled_list_test, view_last_returns_the_tail_of_the_list)
{
  static constexpr size_t tail_size = 2;
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));

  pooled_list_view<test_pdu> tail = first.view().last(tail_size);
  ASSERT_EQ(tail.size(), tail_size);
  ASSERT_EQ(tail.front().value, test_list_capacity - tail_size);
  ASSERT_EQ(tail.back().value, test_list_capacity - 1);
}

TEST_F(pooled_list_test, default_constructed_view_is_empty)
{
  pooled_list_view<test_pdu> view;
  ASSERT_TRUE(view.empty());
  ASSERT_EQ(view.size(), 0);
  ASSERT_EQ(view.begin(), view.end());
}

TEST_F(pooled_list_test, when_list_is_full_then_emplace_back_returns_nullptr)
{
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));

  ASSERT_TRUE(first.full());
  ASSERT_EQ(first.emplace_back(), nullptr);
  ASSERT_EQ(first.size(), test_list_capacity);
}

TEST_F(pooled_list_test, pdus_of_different_lists_do_not_alias)
{
  test_pdu* from_first = first.emplace_back();
  ASSERT_NE(from_first, nullptr);
  from_first->value = 1;

  test_pdu* from_second = second.emplace_back();
  ASSERT_NE(from_second, nullptr);
  from_second->value = 2;

  ASSERT_NE(from_first, from_second);
  ASSERT_EQ(from_first->value, 1);
}

TEST_F(pooled_list_test, when_pool_is_exhausted_then_emplace_back_returns_nullptr)
{
  // The pool serves both lists, so filling every list exhausts it.
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));
  ASSERT_NO_FATAL_FAILURE(fill(second, test_list_capacity));

  test_list third_list;
  third_list.reserve(test_list_capacity);
  pooled_list_builder<test_pdu> third{third_list, pools};
  ASSERT_TRUE(third.empty());
  ASSERT_TRUE(third.full());
  ASSERT_EQ(third.emplace_back(), nullptr);
}

TEST_F(pooled_list_test, when_pdu_is_popped_then_it_returns_to_the_pool)
{
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));
  ASSERT_NO_FATAL_FAILURE(fill(second, test_list_capacity));
  ASSERT_TRUE(first.full());

  first.pop_back();
  ASSERT_EQ(first.size(), test_list_capacity - 1);
  ASSERT_FALSE(first.full());
  ASSERT_NE(first.emplace_back(), nullptr);
}

TEST_F(pooled_list_test, when_list_is_cleared_then_its_pdus_return_to_the_pool)
{
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));
  ASSERT_NO_FATAL_FAILURE(fill(second, test_list_capacity));

  // A third list has room of its own, so only the exhausted pool keeps it from allocating.
  test_list third_list;
  third_list.reserve(test_list_capacity);
  pooled_list_builder<test_pdu> third{third_list, pools};
  ASSERT_TRUE(third.full());

  first_list.clear();

  ASSERT_TRUE(first.empty());
  ASSERT_FALSE(third.full());
  ASSERT_NE(third.emplace_back(), nullptr);
}

TEST_F(pooled_list_test, when_list_is_cleared_then_its_capacity_is_preserved)
{
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));
  first_list.clear();

  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));
  ASSERT_TRUE(first.full());
  ASSERT_EQ(first.size(), test_list_capacity);
}

TEST_F(pooled_list_test, space_left_counts_the_room_of_the_list)
{
  ASSERT_EQ(first.capacity(), test_list_capacity);
  ASSERT_EQ(first.space_left(), test_list_capacity);

  ASSERT_NE(first.emplace_back(), nullptr);
  ASSERT_EQ(first.capacity(), test_list_capacity);
  ASSERT_EQ(first.space_left(), test_list_capacity - 1);
}

TEST_F(pooled_list_test, space_left_is_limited_by_the_pdus_left_in_the_pool)
{
  // The pool serves both lists, so filling one leaves the other only what the pool has left.
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));

  test_list third_list;
  third_list.reserve(test_list_capacity);
  pooled_list_builder<test_pdu> third{third_list, pools};
  ASSERT_EQ(third.capacity(), test_list_capacity);
  ASSERT_EQ(third.space_left(), test_list_capacity);

  ASSERT_NO_FATAL_FAILURE(fill(second, test_list_capacity - 1));
  ASSERT_EQ(third.space_left(), 1);
  ASSERT_FALSE(third.full());

  ASSERT_NE(second.emplace_back(), nullptr);
  ASSERT_EQ(third.space_left(), 0);
  ASSERT_TRUE(third.full());
}

TEST_F(pooled_list_test, list_yields_the_pdus_and_not_their_handles)
{
  ASSERT_NO_FATAL_FAILURE(fill(first, test_list_capacity));

  // The list is reached the same way as one that holds its PDUs inline.
  ASSERT_EQ(first_list.size(), test_list_capacity);
  ASSERT_FALSE(first_list.empty());
  ASSERT_EQ(first_list.capacity(), test_list_capacity);
  ASSERT_EQ(first_list[0].value, 0);
  ASSERT_EQ(first_list.front().value, 0);
  ASSERT_EQ(first_list.back().value, test_list_capacity - 1);

  unsigned count = 0;
  for (const test_pdu& pdu : first_list) {
    ASSERT_EQ(pdu.value, count);
    ++count;
  }
  ASSERT_EQ(count, test_list_capacity);
}

TEST_F(pooled_list_test, const_list_yields_the_pdus)
{
  ASSERT_NO_FATAL_FAILURE(fill(first, 2));

  const test_list& const_list = first_list;
  ASSERT_EQ(const_list[1].value, 1);
  ASSERT_EQ(const_list.back().value, 1);
  ASSERT_EQ(std::distance(const_list.begin(), const_list.end()), 2);
}

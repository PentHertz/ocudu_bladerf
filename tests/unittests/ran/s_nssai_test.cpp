// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/ran/s_nssai.h"
#include <gtest/gtest.h>

using namespace ocudu;

TEST(s_nssai_test, when_sst_values_differ_then_sst_is_not_equal)
{
  ASSERT_TRUE(slice_service_type{1} != slice_service_type{2});
  ASSERT_FALSE(slice_service_type{1} != slice_service_type{1});
}

TEST(s_nssai_test, when_sd_values_differ_then_sd_is_not_equal)
{
  ASSERT_TRUE(slice_differentiator::create(1).value() != slice_differentiator::create(2).value());
  ASSERT_FALSE(slice_differentiator::create(1).value() != slice_differentiator::create(1).value());
}

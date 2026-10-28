// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "lib/du/du_high/du_manager/ran_resource_management/du_pdsch_resource_manager.h"
#include "tests/test_doubles/scheduler/cell_config_builder_profiles.h"
#include "ocudu/du/du_cell_config_helpers.h"
#include "ocudu/scheduler/config/serving_cell_config_factory.h"
#include <gtest/gtest.h>

using namespace ocudu;
using namespace odu;

class du_pdsch_res_alloc_resource_manager_test : public ::testing::TestWithParam<pdsch_config::resource_allocation>
{
protected:
  static du_cell_config make_cell_config(const cell_config_builder_params& params)
  {
    du_cell_config cell               = config_helpers::make_default_du_cell_config(params);
    cell.ran.init_bwp.pdsch.res_alloc = GetParam();
    cell.ran.init_bwp.pdsch.rbg_sz    = rbg_size::config2;
    return cell;
  }

  cell_group_config make_ue() const
  {
    cell_group_config cfg;
    cfg.cells.emplace(SERVING_PCELL_IDX, config_helpers::make_default_ue_cell_config(cell_cfg_list.front().ran));
    return cfg;
  }

  static ue_capability_summary make_caps(bool dyn_switch_supported)
  {
    ue_capability_summary caps;
    caps.pdsch_dyn_switch_ra_type0_1_supported = dyn_switch_supported;
    return caps;
  }

  static const pdsch_config& get_pdsch_cfg(const cell_group_config& ue)
  {
    return ue.cells.at(SERVING_PCELL_IDX).serv_cell_cfg.init_dl_bwp.pdsch_cfg.value();
  }

  cell_config_builder_params  params{cell_config_builder_profiles::create(duplex_mode::TDD)};
  std::vector<du_cell_config> cell_cfg_list{make_cell_config(params)};
  du_test_mode_config         test_cfg{};
  du_pdsch_resource_manager   pdsch_mng{cell_cfg_list, test_cfg};
};

TEST_P(du_pdsch_res_alloc_resource_manager_test, when_ue_capabilities_are_unknown_then_type1_is_used)
{
  cell_group_config ue = make_ue();
  pdsch_mng.alloc_resources(ue);

  ASSERT_EQ(get_pdsch_cfg(ue).res_alloc, pdsch_config::resource_allocation::resource_allocation_type_1);
}

TEST_P(du_pdsch_res_alloc_resource_manager_test, when_ue_supports_dynamic_switch_then_configured_type_is_used)
{
  cell_group_config ue = make_ue();
  pdsch_mng.alloc_resources(ue);
  pdsch_mng.update_resources(ue, make_caps(true));

  ASSERT_EQ(get_pdsch_cfg(ue).res_alloc, GetParam());
  ASSERT_EQ(get_pdsch_cfg(ue).rbg_sz, rbg_size::config2);
}

TEST_P(du_pdsch_res_alloc_resource_manager_test, when_ue_does_not_support_dynamic_switch_then_it_is_not_used)
{
  cell_group_config ue = make_ue();
  pdsch_mng.alloc_resources(ue);
  pdsch_mng.update_resources(ue, make_caps(false));

  // Resource allocation type 0 is mandatory for UEs, while the dynamic switch is a UE capability.
  const pdsch_config::resource_allocation expected = GetParam() == pdsch_config::resource_allocation::dynamic_switch
                                                         ? pdsch_config::resource_allocation::resource_allocation_type_1
                                                         : GetParam();
  ASSERT_EQ(get_pdsch_cfg(ue).res_alloc, expected);
}

INSTANTIATE_TEST_SUITE_P(du_pdsch_res_alloc_resource_manager_test,
                         du_pdsch_res_alloc_resource_manager_test,
                         ::testing::Values(pdsch_config::resource_allocation::resource_allocation_type_0,
                                           pdsch_config::resource_allocation::resource_allocation_type_1,
                                           pdsch_config::resource_allocation::dynamic_switch));

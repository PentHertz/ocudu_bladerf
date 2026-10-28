// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "lib/du/du_high/du_manager/ran_resource_management/du_ran_resource_manager_impl.h"
#include "ocudu/adt/format.h"
#include "ocudu/du/du_cell_config_helpers.h"
#include "ocudu/du/du_high/du_qos_config_helpers.h"
#include <gtest/gtest.h>

using namespace ocudu;
using namespace odu;

namespace {

enum class cell_type { terrestrial, ntn_without_ue_koffset, ntn_with_ue_koffset };

du_cell_config create_du_cell_config(const cell_config_builder_params& params, cell_type type)
{
  du_cell_config cell = config_helpers::make_default_du_cell_config(params);
  if (type == cell_type::terrestrial) {
    return cell;
  }

  ntn_cell_params ntn;
  ntn.ntn_cfg.cell_specific_koffset = std::chrono::milliseconds{16};
  if (type == cell_type::ntn_with_ue_koffset) {
    ntn.ue_koffset_cfg = ue_koffset_config{std::chrono::milliseconds{5}};
  }
  cell.ran.ntn_params = ntn;
  return cell;
}

/// Builds UE capabilities for the given band.
ue_capability_summary make_koffset_caps(nr_band band, bool ue_specific_k_offset_supported)
{
  ue_capability_summary caps;

  ue_capability_summary::supported_band band_caps;
  band_caps.ue_specific_k_offset_supported = ue_specific_k_offset_supported;
  caps.bands.emplace(band, band_caps);

  return caps;
}

class du_ue_koffset_config_tester : public ::testing::TestWithParam<cell_type>
{
protected:
  du_ue_koffset_config_tester() :
    cell_cfg_list({create_du_cell_config(params, GetParam())}),
    qos_cfg_list(config_helpers::make_default_du_qos_config_list(/* warn_on_drop */ true, 1000)),
    res_mng(cell_cfg_list,
            scheduler_expert_config{.ue = {.max_pucchs_per_slot = 31}},
            srb_cfg_list,
            qos_cfg_list,
            dummy_test_mode_cfg)
  {
    auto result = res_mng.create_ue_resource_configurator(ue_idx, to_du_cell_index(0), true);
    report_fatal_error_if_not(result.has_value(), "Failed to create UE resources");
    ue_res.emplace(std::move(result.value()));
  }

  /// Applies a UE configuration with the given capabilities and returns whether the UE accepts a Differential Koffset.
  bool update_ue_caps(const ue_capability_summary& caps)
  {
    f1ap_ue_context_update_request req;
    req.ue_index = ue_idx;
    req.srbs_to_setup.push_back(srb_id_t::srb1);

    const du_ue_resource_update_response resp = ue_res->update(to_du_cell_index(0), req, nullptr, &caps);
    report_fatal_error_if_not(not resp.failed(), "UE configuration failed");

    return ue_res->value().ue_specific_koffset_supported;
  }

  nr_band cell_band() const { return cell_cfg_list[0].ran.dl_carrier.band; }

  static constexpr du_ue_index_t ue_idx = to_du_ue_index(0);

  cell_config_builder_params                  params{};
  du_test_mode_config                         dummy_test_mode_cfg{};
  std::vector<du_cell_config>                 cell_cfg_list;
  std::map<srb_id_t, du_srb_config>           srb_cfg_list;
  std::map<five_qi_t, du_qos_config>          qos_cfg_list;
  du_ran_resource_manager_impl                res_mng;
  std::optional<ue_ran_resource_configurator> ue_res;
};

} // namespace

TEST_P(du_ue_koffset_config_tester, ue_koffset_is_enabled_only_by_an_ntn_cell_that_derives_one)
{
  // The UE supports the feature, so only the cell configuration decides.
  const bool supported = update_ue_caps(make_koffset_caps(cell_band(), true));

  ASSERT_EQ(supported, GetParam() == cell_type::ntn_with_ue_koffset);
}

TEST_P(du_ue_koffset_config_tester, ue_koffset_is_not_enabled_without_the_ue_capability)
{
  // The UE does not declare ue-specific-K-Offset-r17.
  ASSERT_FALSE(update_ue_caps(make_koffset_caps(cell_band(), false)));
}

TEST_P(du_ue_koffset_config_tester, ue_koffset_is_not_enabled_when_the_capability_omits_the_serving_band)
{
  // The capability is declared per band.
  ASSERT_FALSE(update_ue_caps(ue_capability_summary{}));
}

INSTANTIATE_TEST_SUITE_P(du_ue_koffset_config,
                         du_ue_koffset_config_tester,
                         ::testing::Values(cell_type::terrestrial,
                                           cell_type::ntn_without_ue_koffset,
                                           cell_type::ntn_with_ue_koffset));

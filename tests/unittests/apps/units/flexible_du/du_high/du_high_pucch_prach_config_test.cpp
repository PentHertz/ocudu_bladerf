// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "apps/units/flexible_o_du/o_du_high/du_high/du_high_config_cli11_schema.h"
#include "apps/units/flexible_o_du/o_du_high/du_high/du_high_config_translators.h"
#include "apps/units/flexible_o_du/o_du_high/du_high/du_high_config_validator.h"
#include "ocudu/du/du_cell_config_validation.h"
#include "ocudu/support/config_parsers.h"
#include <gtest/gtest.h>
#include <sstream>

using namespace ocudu;

namespace {

/// Carrier of a single cell.
struct carrier_params {
  unsigned band;
  unsigned dl_arfcn;
  unsigned scs_khz;
  unsigned bw_mhz;
};

void PrintTo(const carrier_params& params, std::ostream* os)
{
  *os << fmt::format("band=n{} scs={}kHz bw={}MHz", params.band, params.scs_khz, params.bw_mhz);
}

/// Parses a YAML configuration through the CLI11 schema, as the application does, and auto-derives its parameters.
du_high_unit_config parse_config(const std::string& yaml)
{
  CLI::App app{"du_high_pucch_prach_config_test"};
  app.config_formatter(create_yaml_config_parser());
  app.allow_config_extras(CLI::config_extras_mode::error);

  du_high_parsed_config parsed_cfg;
  configure_cli11_with_du_high_config_schema(app, parsed_cfg);

  std::istringstream input(yaml);
  app.parse_from_stream(input);
  autoderive_du_high_parameters_after_parsing(parsed_cfg.config);

  return parsed_cfg.config;
}

class du_high_pucch_prach_config_test : public ::testing::TestWithParam<carrier_params>
{};

} // namespace

/// With the default PUCCH and PRACH parameters, the PRACH and its guardbands fit in between the PUCCH resources.
TEST_P(du_high_pucch_prach_config_test, default_pucch_and_prach_config_is_valid)
{
  const carrier_params&     params = GetParam();
  const du_high_unit_config cfg    = parse_config(fmt::format(R"(
cell_cfg:
  dl_arfcn: {}
  band: {}
  channel_bandwidth_MHz: {}
  common_scs: {}
  pci: 1
)",
                                                           params.dl_arfcn,
                                                           params.band,
                                                           params.bw_mhz,
                                                           params.scs_khz));
  ASSERT_TRUE(validate_du_high_config(cfg));

  const std::vector<odu::du_cell_config> cells = generate_du_cell_config(cfg);
  ASSERT_EQ(cells.size(), 1);
  const validator_result du_check = is_du_cell_config_valid(cells.front());
  EXPECT_TRUE(du_check.has_value()) << (du_check.has_value() ? std::string{} : du_check.error());
}

INSTANTIATE_TEST_SUITE_P(narrow_and_wide_bandwidths,
                         du_high_pucch_prach_config_test,
                         ::testing::Values(carrier_params{78, 650000, 30, 10},
                                           carrier_params{78, 650000, 30, 15},
                                           carrier_params{78, 650000, 30, 20},
                                           carrier_params{78, 650000, 30, 100},
                                           carrier_params{41, 520002, 15, 5},
                                           carrier_params{41, 520002, 15, 10},
                                           carrier_params{3, 368500, 15, 5},
                                           carrier_params{3, 368500, 15, 20}));

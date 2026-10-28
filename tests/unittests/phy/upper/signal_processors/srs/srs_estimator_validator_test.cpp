// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "../../../support/resource_grid_test_doubles.h"
#include "ocudu/adt/format.h"
#include "ocudu/phy/upper/signal_processors/srs/formatters.h"
#include "ocudu/phy/upper/signal_processors/srs/srs_estimator_configuration.h"
#include "ocudu/phy/upper/signal_processors/srs/srs_estimator_factory.h"
#include "fmt/ostream.h"
#include <gtest/gtest.h>

using namespace ocudu;

namespace {

// Valid SRS configuration used as a base for the test case.
const srs_estimator_configuration base_config = {
    .slot     = {0, 130, 8, 0},
    .resource = {.nof_antenna_ports   = srs_resource_configuration::one_two_four_enum(2),
                 .nof_symbols         = srs_nof_symbols(1),
                 .start_symbol        = 12,
                 .configuration_index = 17,
                 .sequence_id         = 647,
                 .bandwidth_index     = 2,
                 .comb_size           = tx_comb_size(2),
                 .comb_offset         = 1,
                 .cyclic_shift        = 1,
                 .freq_position       = 66,
                 .freq_shift          = 1,
                 .freq_hopping        = 3,
                 .hopping             = srs_group_or_sequence_hopping::neither,
                 .periodicity         = std::nullopt},
    .ports    = {0}};

struct test_case_t {
  std::function<srs_estimator_configuration()> get_config;
  std::string                                  expr;
};

std::ostream& operator<<(std::ostream& os, const test_case_t& test_case)
{
  fmt::print(os, "{}", test_case.get_config());
  return os;
}

const std::vector<test_case_t> srs_estimator_validator_test_data = {
    {[] {
       srs_estimator_configuration config = base_config;
       config.resource.comb_size          = tx_comb_size(2);
       config.resource.comb_offset        = 2;
       return config;
     },
     R"(Invalid SRS resource configuration\.)"},
    {[] {
       srs_estimator_configuration config = base_config;
       config.resource.bandwidth_index    = 1;
       config.resource.freq_hopping       = 0;
       return config;
     },
     R"(The SRS estimator does not support frequency hopping\.)"},
    {[] {
       srs_estimator_configuration config = base_config;
       config.resource.hopping            = srs_group_or_sequence_hopping::group_hopping;
       return config;
     },
     R"(The SRS estimator does not support group or sequence hopping\.)"},
    {[] {
       srs_estimator_configuration config = base_config;
       config.ports.clear();
       return config;
     },
     R"(Empty list of Rx ports for the SRS estimator\.)"},
    {[] {
       srs_estimator_configuration config = base_config;
       config.resource.bandwidth_index    = 0;
       return config;
     },
     R"(SRS resource exceeds maximum bandwidth\.)"},
};

class srsEstimatorValidatorFixture : public ::testing::TestWithParam<test_case_t>
{
protected:
  void SetUp() override
  {
    std::shared_ptr<low_papr_sequence_generator_factory> sequence_generator_factory =
        create_low_papr_sequence_generator_sw_factory();
    ASSERT_NE(sequence_generator_factory, nullptr);

    std::shared_ptr<dft_processor_factory> dft_factory = create_dft_processor_factory_fftw_slow();
    if (!dft_factory) {
      dft_factory = create_dft_processor_factory_generic();
    }
    report_fatal_error_if_not(dft_factory, "Invalid DFT factory.");

    std::shared_ptr<time_alignment_estimator_factory> ta_est_factory =
        create_time_alignment_estimator_dft_factory(dft_factory);
    report_fatal_error_if_not(ta_est_factory, "Invalid TA estimator factory.");

    std::shared_ptr<srs_estimator_factory> srs_est_factory =
        create_srs_estimator_generic_factory(sequence_generator_factory, ta_est_factory, nullptr, 25);
    ASSERT_NE(srs_est_factory, nullptr);

    estimator = srs_est_factory->create();
    ASSERT_NE(estimator, nullptr);

    validator = srs_est_factory->create_validator();
    ASSERT_NE(validator, nullptr);
  }

  std::unique_ptr<srs_estimator>                         estimator = nullptr;
  std::unique_ptr<srs_estimator_configuration_validator> validator = nullptr;
};

TEST_P(srsEstimatorValidatorFixture, srsEstimatorValidatorDeathTest)
{
  // Use thread safe death test.
  ::testing::GTEST_FLAG(death_test_style) = "threadsafe";

  ASSERT_NE(estimator, nullptr);
  ASSERT_NE(validator, nullptr);

  const test_case_t& param = GetParam();

  srs_estimator_configuration config = param.get_config();

  // Make sure the configuration is invalid.
  ASSERT_FALSE(validator->is_valid(config));

  // Prepare resource grid and resource grid mapper spies.
  resource_grid_reader_spy grid(0, 0, 0);

  // Estimate the SRS channel.
#ifdef ASSERTS_ENABLED
  ASSERT_DEATH({ estimator->estimate(grid, config); }, param.expr);
#endif // ASSERTS_ENABLED
}

// Creates test suite that combines all possible parameters.
INSTANTIATE_TEST_SUITE_P(srsEstimatorValidatorDeathTest,
                         srsEstimatorValidatorFixture,
                         ::testing::ValuesIn(srs_estimator_validator_test_data));

} // namespace

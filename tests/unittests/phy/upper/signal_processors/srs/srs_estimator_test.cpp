// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/phy/generic_functions/generic_functions_factories.h"
#include "ocudu/phy/support/resource_grid.h"
#include "ocudu/phy/support/resource_grid_writer.h"
#include "ocudu/phy/support/support_factories.h"
#include "ocudu/phy/support/time_alignment_estimator/time_alignment_estimator_factories.h"
#include "ocudu/phy/upper/sequence_generators/sequence_generator_factories.h"
#include "ocudu/phy/upper/signal_processors/srs/doa_estimator_configuration.h"
#include "ocudu/phy/upper/signal_processors/srs/doa_estimator_factory.h"
#include "ocudu/phy/upper/signal_processors/srs/srs_estimator.h"
#include "ocudu/phy/upper/signal_processors/srs/srs_estimator_configuration.h"
#include "ocudu/phy/upper/signal_processors/srs/srs_estimator_factory.h"
#include "ocudu/phy/upper/signal_processors/srs/srs_estimator_result.h"
#include "ocudu/ran/cyclic_prefix.h"
#include "ocudu/ran/resource_block.h"
#include "ocudu/ran/srs/srs_information.h"
#include "ocudu/ran/srs/srs_resource_configuration.h"
#include "ocudu/support/math/complex_normal_random.h"
#include "ocudu/support/math/math_utils.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using namespace ocudu;

namespace {

constexpr unsigned           nof_antennas = 4;
constexpr unsigned           max_nof_prb  = 52;
constexpr subcarrier_spacing scs          = subcarrier_spacing::kHz30;

/// Grid ports, in antenna order, through which the tests receive the SRS.
using port_list = std::array<unsigned, nof_antennas>;

/// Requests that differ only in the grid ports: in antenna order, out of order, and beyond the number of requested
/// ports.
const std::array<port_list, 3> port_lists = {{{0, 1, 2, 3}, {3, 1, 0, 2}, {4, 5, 6, 7}}};

/// \brief Parameters of the received SRS.
///
/// The SRS arrives as a plane wave on a uniform linear array with half-wavelength spacing, with the same amplitude on
/// every antenna, delayed by a common propagation delay and corrupted by additive white Gaussian noise.
struct channel_parameters {
  /// Broadside angle of arrival in degrees.
  float angle_degrees = 20.0F;
  /// Amplitude of the channel coefficient of every antenna.
  float amplitude = 1.0F;
  /// Delay of the received signal in seconds.
  double delay_s = 0.0;
  /// Variance of the noise added to every received resource element, zero for no noise.
  float noise_variance = 0.0F;
};

/// Creates an SRS estimator with direction-of-arrival estimation for a uniform linear array.
std::unique_ptr<srs_estimator> create_estimator()
{
  std::shared_ptr<dft_processor_factory> dft_factory = create_dft_processor_factory_generic();
  doa_estimator_configuration            doa_config  = {
                  .nof_antennas = nof_antennas, .antenna_distance_over_wavelength = 0.5F, .cross_polarized = false};
  std::shared_ptr<srs_estimator_factory> factory =
      create_srs_estimator_generic_factory(create_low_papr_sequence_generator_sw_factory(),
                                           create_time_alignment_estimator_dft_factory(dft_factory),
                                           create_doa_estimator_factory(doa_config),
                                           max_nof_prb);
  return factory->create();
}

/// Creates the configuration of a one-port SRS received by the given grid ports.
srs_estimator_configuration make_config(const port_list& rx_ports)
{
  srs_resource_configuration resource;
  resource.nof_antenna_ports   = srs_resource_configuration::one_two_four_enum::one;
  resource.nof_symbols         = srs_nof_symbols::n2;
  resource.start_symbol        = 12;
  resource.configuration_index = 6;
  resource.sequence_id         = 5;
  resource.bandwidth_index     = 0;
  resource.comb_size           = tx_comb_size::n2;
  resource.comb_offset         = 0;
  resource.cyclic_shift        = 0;
  resource.freq_position       = 0;
  resource.freq_shift          = 0;
  resource.freq_hopping        = 0;
  resource.hopping             = srs_group_or_sequence_hopping::neither;
  resource.periodicity         = srs_resource_configuration::periodicity_and_offset{0, 0};

  srs_estimator_configuration config;
  config.slot     = slot_point(to_numerology_value(scs), 0);
  config.resource = resource;
  config.ports.assign(rx_ports.begin(), rx_ports.end());
  return config;
}

/// Returns the channel coefficient of an antenna of the array, without the delay.
cf_t antenna_coefficient(const channel_parameters& channel, unsigned i_antenna)
{
  // A wave arriving from a positive broadside angle reaches antenna i later than antenna 0.
  float phase_step = M_PI * std::sin(channel.angle_degrees * static_cast<float>(M_PI) / 180.0F);
  return std::polar(channel.amplitude, -phase_step * static_cast<float>(i_antenna));
}

/// \brief Writes the received SRS in the resource grid.
///
/// The \f$i\f$-th requested port is the \f$i\f$-th antenna of the array, whichever grid port it is. The delay rotates
/// subcarrier \f$k\f$ by \f$e^{-j 2 \pi k \Delta f \tau}\f$.
void write_received_srs(resource_grid&                     grid,
                        const srs_estimator_configuration& config,
                        const channel_parameters&          channel,
                        std::mt19937&                      rgen)
{
  srs_information   info = get_srs_information(config.resource, 0);
  std::vector<cf_t> sequence(info.sequence_length);
  create_low_papr_sequence_generator_sw_factory()->create()->generate(
      sequence, info.sequence_group, info.sequence_number, info.n_cs, info.n_cs_max);

  // Unit variance complex Gaussian noise, scaled to the noise variance of the channel.
  complex_normal_distribution<cf_t> noise;
  float                             noise_std = std::sqrt(channel.noise_variance);
  double                            scs_Hz    = scs_to_khz(scs) * 1000.0;

  std::vector<cbf16_t> received(info.sequence_length);
  grid.set_all_zero();
  for (unsigned i_antenna = 0; i_antenna != nof_antennas; ++i_antenna) {
    cf_t coefficient = antenna_coefficient(channel, i_antenna);
    for (unsigned i_symbol     = config.resource.start_symbol.value(),
                  i_symbol_end = i_symbol + static_cast<unsigned>(config.resource.nof_symbols);
         i_symbol != i_symbol_end;
         ++i_symbol) {
      for (unsigned i_re = 0; i_re != info.sequence_length; ++i_re) {
        unsigned k     = info.mapping_initial_subcarrier + i_re * info.comb_size;
        cf_t     delay = std::polar(1.0F, static_cast<float>(-TWOPI * k * scs_Hz * channel.delay_s));
        received[i_re] = to_cbf16(coefficient * delay * sequence[i_re] + noise_std * noise(rgen));
      }
      grid.get_writer().put(
          config.ports[i_antenna], i_symbol, info.mapping_initial_subcarrier, info.comb_size, received);
    }
  }
}

/// Estimates the SRS received on the given grid ports.
srs_estimator_result estimate(const port_list& rx_ports, const channel_parameters& channel)
{
  // The same noise realization for every request, so that the results can be compared exactly.
  std::mt19937 rgen(1234);

  srs_estimator_configuration    config = make_config(rx_ports);
  std::unique_ptr<resource_grid> grid   = create_resource_grid_factory()->create(
      2 * nof_antennas, get_nsymb_per_slot(cyclic_prefix::NORMAL), max_nof_prb * NOF_SUBCARRIERS_PER_RB);
  write_received_srs(*grid, config, channel, rgen);

  return create_estimator()->estimate(grid->get_reader(), config);
}

} // namespace

// The direction of arrival depends on which antenna receives each sample, that is, on the position of the port in the
// request, not on the grid port that holds it. Requests that differ only in the grid ports must estimate the same
// direction.
TEST(srs_estimator_test, direction_follows_the_order_of_the_requested_ports)
{
  channel_parameters channel;

  srs_estimator_result reference = estimate(port_lists.front(), channel);
  ASSERT_TRUE(reference.doa_result.has_value());
  ASSERT_FALSE(reference.doa_result->doa_components.empty());
  EXPECT_NEAR(reference.doa_result->doa_components.front().broadside_angle_degrees, channel.angle_degrees, 1.0F);

  for (const port_list& rx_ports : span<const port_list>(port_lists).last(port_lists.size() - 1)) {
    srs_estimator_result result = estimate(rx_ports, channel);
    ASSERT_TRUE(result.doa_result.has_value());
    ASSERT_EQ(result.doa_result->doa_components.size(), reference.doa_result->doa_components.size());
    for (unsigned i_component = 0; i_component != result.doa_result->doa_components.size(); ++i_component) {
      EXPECT_FLOAT_EQ(result.doa_result->doa_components[i_component].broadside_angle_degrees,
                      reference.doa_result->doa_components[i_component].broadside_angle_degrees);
      EXPECT_FLOAT_EQ(result.doa_result->doa_components[i_component].spectrum_strength,
                      reference.doa_result->doa_components[i_component].spectrum_strength);
    }
  }
}

// The power, noise and channel measurements, for a noisy signal of known power received through each of the port lists.
TEST(srs_estimator_test, measurements_match_the_received_signal)
{
  for (float noise_variance : {0.0025F, 0.025F}) {
    channel_parameters channel;
    channel.amplitude      = 0.5F;
    channel.noise_variance = noise_variance;

    float signal_power = channel.amplitude * channel.amplitude;
    float snr          = signal_power / channel.noise_variance;

    for (const port_list& rx_ports : port_lists) {
      SCOPED_TRACE(fmt::format("noise_variance={} ports={}", noise_variance, span<const unsigned>(rx_ports)));
      srs_estimator_result result = estimate(rx_ports, channel);

      // Powers. The noise variance is estimated from the residual of the channel estimate, the RSRP is the power of
      // the channel coefficients and the EPRE is the power of the received resource elements.
      ASSERT_TRUE(result.noise_variance.has_value());
      EXPECT_NEAR(
          convert_power_to_dB(result.noise_variance.value()), convert_power_to_dB(channel.noise_variance), 0.5F);
      ASSERT_TRUE(result.rsrp_dB.has_value());
      EXPECT_NEAR(result.rsrp_dB.value(), convert_power_to_dB(signal_power), 0.3F);
      ASSERT_TRUE(result.epre_dB.has_value());
      EXPECT_NEAR(result.epre_dB.value(), convert_power_to_dB(signal_power + channel.noise_variance), 0.3F);

      // Channel matrix: one row per requested port and one column per transmit port, normalized by the noise standard
      // deviation, so that the power of each coefficient is the SNR. Row i holds the coefficient of antenna i, so the
      // phase differences between rows are those of the array steering vector.
      ASSERT_EQ(result.channel_matrix.get_nof_rx_ports(), nof_antennas);
      ASSERT_EQ(result.channel_matrix.get_nof_tx_ports(), 1);
      cf_t coefficient_0 = result.channel_matrix.get_coefficient(0, 0);
      for (unsigned i_antenna = 0; i_antenna != nof_antennas; ++i_antenna) {
        cf_t coefficient = result.channel_matrix.get_coefficient(i_antenna, 0);
        EXPECT_NEAR(convert_power_to_dB(std::norm(coefficient)), convert_power_to_dB(snr), 0.5F)
            << "antenna " << i_antenna;
        cf_t steering = antenna_coefficient(channel, i_antenna) / antenna_coefficient(channel, 0);
        EXPECT_NEAR(std::arg(coefficient / coefficient_0 / steering), 0.0F, 0.1F) << "antenna " << i_antenna;
      }

      // No delay.
      EXPECT_NEAR(result.time_alignment.time_alignment, 0.0, result.time_alignment.resolution);
    }
  }
}

// The time alignment measures the delay of the signal, within the resolution of the measurement. The delay does not
// change the RSRP or the phase differences between antennas.
TEST(srs_estimator_test, time_alignment_matches_the_delay)
{
  for (double delay_s : {0.05e-6, 0.2e-6, 0.5e-6}) {
    channel_parameters channel;
    channel.amplitude      = 0.5F;
    channel.delay_s        = delay_s;
    channel.noise_variance = 0.0025F;

    for (const port_list& rx_ports : port_lists) {
      SCOPED_TRACE(fmt::format("delay={}us ports={}", delay_s * 1e6, span<const unsigned>(rx_ports)));
      srs_estimator_result result = estimate(rx_ports, channel);

      EXPECT_GT(result.time_alignment.resolution, 0.0);
      EXPECT_LT(result.time_alignment.min, result.time_alignment.max);
      EXPECT_GE(channel.delay_s, result.time_alignment.min);
      EXPECT_LE(channel.delay_s, result.time_alignment.max);
      EXPECT_NEAR(result.time_alignment.time_alignment, channel.delay_s, result.time_alignment.resolution);

      ASSERT_TRUE(result.rsrp_dB.has_value());
      EXPECT_NEAR(result.rsrp_dB.value(), convert_power_to_dB(channel.amplitude * channel.amplitude), 0.3F);

      cf_t coefficient_0 = result.channel_matrix.get_coefficient(0, 0);
      for (unsigned i_antenna = 0; i_antenna != nof_antennas; ++i_antenna) {
        cf_t steering = antenna_coefficient(channel, i_antenna) / antenna_coefficient(channel, 0);
        EXPECT_NEAR(
            std::arg(result.channel_matrix.get_coefficient(i_antenna, 0) / coefficient_0 / steering), 0.0F, 0.1F)
            << "antenna " << i_antenna;
      }
    }
  }
}

// The estimates depend on the order of the requested ports only: requests that differ only in the grid ports give the
// same result.
TEST(srs_estimator_test, measurements_follow_the_order_of_the_requested_ports)
{
  channel_parameters channel;
  channel.amplitude      = 0.5F;
  channel.delay_s        = 0.2e-6;
  channel.noise_variance = 0.0025F;

  srs_estimator_result reference = estimate(port_lists.front(), channel);
  for (const port_list& rx_ports : span<const port_list>(port_lists).last(port_lists.size() - 1)) {
    SCOPED_TRACE(fmt::format("ports={}", span<const unsigned>(rx_ports)));
    srs_estimator_result result = estimate(rx_ports, channel);

    ASSERT_EQ(result.channel_matrix.get_nof_rx_ports(), reference.channel_matrix.get_nof_rx_ports());
    ASSERT_EQ(result.channel_matrix.get_nof_tx_ports(), reference.channel_matrix.get_nof_tx_ports());
    for (unsigned i_rx_port = 0; i_rx_port != result.channel_matrix.get_nof_rx_ports(); ++i_rx_port) {
      for (unsigned i_tx_port = 0; i_tx_port != result.channel_matrix.get_nof_tx_ports(); ++i_tx_port) {
        EXPECT_EQ(result.channel_matrix.get_coefficient(i_rx_port, i_tx_port),
                  reference.channel_matrix.get_coefficient(i_rx_port, i_tx_port))
            << "rx port " << i_rx_port << ", tx port " << i_tx_port;
      }
    }
    EXPECT_EQ(result.noise_variance, reference.noise_variance);
    EXPECT_EQ(result.rsrp_dB, reference.rsrp_dB);
    EXPECT_EQ(result.epre_dB, reference.epre_dB);
    EXPECT_EQ(result.time_alignment.time_alignment, reference.time_alignment.time_alignment);
  }
}

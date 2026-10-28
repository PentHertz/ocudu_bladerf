// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "beamforming_weights_compressor.h"
#include "ofh_compression_test_helpers.h"
#include "tests/ocudu_test_requirements.h"
#include "ocudu/ofh/compression/compression_properties.h"
#include "ocudu/support/math/math_utils.h"
#include <cmath>
#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>
#include <vector>

using namespace ocudu;
using namespace ocudu::ofh;

// Compresses the given beamforming weights following O-RAN.WG4.CUS, Annex A.1.2 and the bit ordering of Annex D.1.
static std::vector<uint8_t> compress_reference(span<const cf_t> weights, const ru_compression_params& params)
{
  // Block floating point quantizes at the full bit width and scales the block down by the shared exponent.
  unsigned quantization_width = (params.type == compression_type::BFP) ? MAX_IQ_WIDTH : params.data_width;

  std::vector<int16_t> quantized;
  for (const cf_t weight : weights) {
    quantized.push_back(ofh::testing::quantize_weight_reference(weight.real(), quantization_width));
    quantized.push_back(ofh::testing::quantize_weight_reference(weight.imag(), quantization_width));
  }

  // The entire weight vector is one compression block (O-RAN.WG4.CUS, 7.7.1.2), and the bfwCompParam field holds the
  // shared exponent.
  std::vector<uint8_t> compressed(get_packed_beamforming_weights_size(weights.size(), params).value(), 0);
  unsigned             bit_pos = 0;
  ofh::testing::compress_block_reference(compressed, bit_pos, quantized, params);

  return compressed;
}

// Generates a beamforming weight vector resembling a DFT beam of the given number of TRXs.
static std::vector<cf_t> generate_dft_weights(unsigned nof_weights, unsigned i_beam)
{
  float amplitude = std::sqrt(1.0F / static_cast<float>(nof_weights));

  std::vector<cf_t> weights;
  for (unsigned i = 0; i != nof_weights; ++i) {
    weights.push_back(std::polar(amplitude, TWOPI * i * i_beam / nof_weights));
  }

  return weights;
}

namespace {

struct test_case {
  const char*       name;
  std::vector<cf_t> weights;
};

class beamforming_weights_compressor_fixture
  : public ::testing::TestWithParam<std::tuple<test_case, compression_type, unsigned>>
{};

} // namespace

TEST_P(beamforming_weights_compressor_fixture, compression_matches_the_specification)
{
  OCUDU_TEST_REQUIREMENTS("RU-OFH-CATB-WDBF");

  const ru_compression_params compr_params = {std::get<1>(GetParam()), std::get<2>(GetParam())};
  span<const cf_t>            weights      = std::get<0>(GetParam()).weights;

  std::vector<uint8_t> compressed(get_packed_beamforming_weights_size(weights.size(), compr_params).value(), 0);
  compress_beamforming_weights(compressed, weights, compr_params);

  ASSERT_EQ(compress_reference(weights, compr_params), compressed);
}

INSTANTIATE_TEST_SUITE_P(
    ofh_beamforming_weights_compressor_test,
    beamforming_weights_compressor_fixture,
    ::testing::Combine(
        ::testing::Values(test_case{"unit_weights", {{1.0F, 0.0F}, {0.0F, 1.0F}, {-1.0F, 0.0F}, {0.0F, -1.0F}}},
                          test_case{"negative_weights", {{-0.5F, -0.5F}, {-0.25F, -0.75F}}},
                          test_case{"small_weights", {{0.001F, -0.002F}, {0.0F, 0.003F}, {-0.001F, 0.0F}}},
                          test_case{"zero_weights", {{0.0F, 0.0F}, {0.0F, 0.0F}}},
                          // Weights quantizing to a mixture of minus one and zero, and to all minus one, are the only
                          // blocks whose maximum absolute value is not positive.
                          test_case{"minus_one_and_zero_weights", {{-3.0e-5F, 0.0F}, {-3.0e-5F, -3.0e-5F}}},
                          test_case{"minus_one_weights", {{-3.0e-5F, -3.0e-5F}, {-3.0e-5F, -3.0e-5F}}},
                          test_case{"dft_beam_four_trx", generate_dft_weights(4, 1)},
                          test_case{"dft_beam_eight_trx", generate_dft_weights(8, 3)}),
        ::testing::Values(compression_type::none, compression_type::BFP),
        ::testing::Values(8U, 9U, 12U, 16U)));

TEST(ofh_beamforming_weights_compressor_test, compressed_size_matches_the_number_of_weights)
{
  OCUDU_TEST_REQUIREMENTS("RU-OFH-CATB-WDBF");

  ASSERT_EQ(get_packed_beamforming_weights_size(4, {compression_type::none, 16}).value(), 16);
  ASSERT_EQ(get_packed_beamforming_weights_size(2, {compression_type::none, 12}).value(), 6);
  ASSERT_EQ(get_packed_beamforming_weights_size(8, {compression_type::none, 9}).value(), 18);

  // Block floating point adds the bfwCompParam field holding the shared exponent.
  ASSERT_EQ(get_packed_beamforming_weights_size(4, {compression_type::BFP, 16}).value(), 17);
  ASSERT_EQ(get_packed_beamforming_weights_size(8, {compression_type::BFP, 9}).value(), 19);
}

TEST(ofh_beamforming_weights_compressor_test, uncompressed_16_bit_weights_should_pass)
{
  OCUDU_TEST_REQUIREMENTS("RU-OFH-CATB-WDBF");

  const std::vector<cf_t> weights = {{1.0F, 0.0F}, {0.0F, 1.0F}, {-1.0F, 0.0F}, {0.0F, -1.0F}};

  const std::vector<uint8_t> expected = {
      0x7f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01};

  const ru_compression_params compr_params = {compression_type::none, 16};

  std::vector<uint8_t> compressed(get_packed_beamforming_weights_size(weights.size(), compr_params).value(), 0);
  compress_beamforming_weights(compressed, weights, compr_params);

  ASSERT_EQ(expected, compressed);
}

TEST(ofh_beamforming_weights_compressor_test, uncompressed_12_bit_weights_should_pass)
{
  OCUDU_TEST_REQUIREMENTS("RU-OFH-CATB-WDBF");

  const std::vector<cf_t> weights = {{1.0F, 0.0F}, {-1.0F, 0.5F}};

  // Quantized weights are 0x7ff, 0x000, 0x801 and 0x400, packed without byte alignment between them.
  const std::vector<uint8_t> expected = {0x7f, 0xf0, 0x00, 0x80, 0x14, 0x00};

  const ru_compression_params compr_params = {compression_type::none, 12};

  std::vector<uint8_t> compressed(get_packed_beamforming_weights_size(weights.size(), compr_params).value(), 0);
  compress_beamforming_weights(compressed, weights, compr_params);

  ASSERT_EQ(expected, compressed);
}

TEST(ofh_beamforming_weights_compressor_test, bfp_9_bit_weights_should_pass)
{
  OCUDU_TEST_REQUIREMENTS("RU-OFH-CATB-WDBF");

  const std::vector<cf_t> weights = {{1.0F, 0.0F}, {0.0F, -1.0F}};

  // Quantized at 16 bits the maximum absolute value is 0x7fff, which has 15 significant bits, so the shared exponent
  // is 15 - 9 + 1 = 7. The 9 bit mantissas are 0x0ff, 0x000, 0x000 and 0x100, followed by 4 zero padding bits.
  const std::vector<uint8_t> expected = {0x07, 0x7f, 0x80, 0x00, 0x10, 0x00};

  const ru_compression_params compr_params = {compression_type::BFP, 9};

  std::vector<uint8_t> compressed(get_packed_beamforming_weights_size(weights.size(), compr_params).value(), 0);
  compress_beamforming_weights(compressed, weights, compr_params);

  ASSERT_EQ(expected, compressed);
}

TEST(ofh_beamforming_weights_compressor_test, out_of_range_weights_should_clip)
{
  OCUDU_TEST_REQUIREMENTS("RU-OFH-CATB-WDBF");

  const std::vector<cf_t> weights = {{2.0F, -3.0F}};

  const std::vector<uint8_t> expected = {0x7f, 0xff, 0x80, 0x01};

  const ru_compression_params compr_params = {compression_type::none, 16};

  std::vector<uint8_t> compressed(get_packed_beamforming_weights_size(weights.size(), compr_params).value(), 0);
  compress_beamforming_weights(compressed, weights, compr_params);

  ASSERT_EQ(expected, compressed);
}

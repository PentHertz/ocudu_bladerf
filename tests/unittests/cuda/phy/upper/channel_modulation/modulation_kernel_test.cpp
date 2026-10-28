// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/cuda/adt/cuda_copy.h"
#include "ocudu/cuda/adt/cuda_stream.h"
#include "ocudu/cuda/adt/device_vector.h"
#include "ocudu/cuda/phy/upper/channel_modulation/modulation.h"
#include "ocudu/phy/upper/channel_modulation/channel_modulation_factories.h"
#include <cmath>
#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using namespace ocudu;
using namespace ocudu::cuda;

namespace {

/// Returns true if a CUDA device is available to run the test.
bool has_cuda_device()
{
  int nof_devices = 0;
  return (::cudaGetDeviceCount(&nof_devices) == cudaSuccess) && (nof_devices > 0);
}

/// Schemes the GPU maps to a floating point constellation.
const std::vector<modulation_scheme> float_schemes = {modulation_scheme::QPSK,
                                                      modulation_scheme::QAM16,
                                                      modulation_scheme::QAM64,
                                                      modulation_scheme::QAM256};

/// Schemes the GPU maps to an 8-bit integer constellation.
const std::vector<modulation_scheme> int8_schemes = {modulation_scheme::QPSK,
                                                     modulation_scheme::QAM16,
                                                     modulation_scheme::QAM64,
                                                     modulation_scheme::QAM256};

class modulation_kernel_test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!has_cuda_device()) {
      GTEST_SKIP() << "No CUDA device available";
    }

    cuda_expected<cuda_stream> created = cuda_stream::create();
    ASSERT_TRUE(created.has_value());
    stream = std::move(created.value());

    modulator   = create_modulation_mapper_factory()->create();
    demodulator = create_demodulation_mapper_factory()->create();
    ASSERT_NE(modulator, nullptr);
    ASSERT_NE(demodulator, nullptr);
  }

  /// Fills a bit buffer with pseudo-random bits, the same for a given size on every run.
  static dynamic_bit_buffer make_bits(unsigned nof_bits)
  {
    dynamic_bit_buffer                      bits(nof_bits);
    std::mt19937                            rgen(0x5eed);
    std::uniform_int_distribution<unsigned> dist(0, 1);
    for (unsigned i = 0; i != nof_bits; ++i) {
      bits.insert(dist(rgen), i, 1);
    }
    return bits;
  }

  /// Packs a bit buffer the way the kernels read it: bit n at position n modulo 32 of word n / 32.
  static std::vector<uint32_t> pack_for_device(const bit_buffer& bits)
  {
    std::vector<uint32_t> words((bits.size() + 31) / 32, 0);
    for (unsigned i = 0, end = bits.size(); i != end; ++i) {
      if (bits.extract(i, 1) != 0) {
        words[i / 32] |= (1U << (i % 32));
      }
    }
    return words;
  }

  /// Maps bits on the device and reads the symbols back.
  std::vector<cf_t> modulate_on_device(const bit_buffer& bits, modulation_scheme scheme)
  {
    unsigned nof_symbols = bits.size() / static_cast<unsigned>(scheme);

    std::vector<uint32_t>                  words  = pack_for_device(bits);
    cuda_expected<device_vector<uint32_t>> d_bits = device_vector<uint32_t>::create(words.size());
    EXPECT_TRUE(d_bits.has_value());
    EXPECT_TRUE(copy_to_device(d_bits.value(), span<const uint32_t>(words.data(), words.size())).has_value());

    cuda_expected<device_vector<cuFloatComplex>> d_symbols = device_vector<cuFloatComplex>::create(nof_symbols);
    EXPECT_TRUE(d_symbols.has_value());

    cuda_result status = modulate(d_symbols.value().data(), d_bits.value().data(), bits.size(), scheme, stream);
    EXPECT_TRUE(status.has_value());
    EXPECT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

    std::vector<cf_t> symbols(nof_symbols);
    EXPECT_TRUE(copy_to_host(span<cuFloatComplex>(reinterpret_cast<cuFloatComplex*>(symbols.data()), nof_symbols),
                             d_symbols.value())
                    .has_value());
    return symbols;
  }

  cuda_stream                          stream;
  std::unique_ptr<modulation_mapper>   modulator;
  std::unique_ptr<demodulation_mapper> demodulator;
};

} // namespace

TEST_F(modulation_kernel_test, mapping_matches_the_software_mapper)
{
  for (modulation_scheme scheme : float_schemes) {
    unsigned           nof_symbols = 4096;
    unsigned           nof_bits    = nof_symbols * static_cast<unsigned>(scheme);
    dynamic_bit_buffer bits        = make_bits(nof_bits);

    std::vector<cf_t> expected(nof_symbols);
    modulator->modulate(expected, bits, scheme);

    std::vector<cf_t> actual = modulate_on_device(bits, scheme);

    ASSERT_EQ(actual.size(), expected.size());
    for (unsigned i = 0; i != nof_symbols; ++i) {
      ASSERT_NEAR(actual[i].real(), expected[i].real(), 1e-5) << to_string(scheme) << ", symbol " << i;
      ASSERT_NEAR(actual[i].imag(), expected[i].imag(), 1e-5) << to_string(scheme) << ", symbol " << i;
    }
  }
}

TEST_F(modulation_kernel_test, the_8_bit_mapping_matches_the_software_mapper)
{
  // Both put the same 8-bit points in the buffer, so the comparison is an equality rather than a
  // tolerance.
  for (modulation_scheme scheme : int8_schemes) {
    unsigned           nof_symbols = 4096;
    unsigned           nof_bits    = nof_symbols * static_cast<unsigned>(scheme);
    dynamic_bit_buffer bits        = make_bits(nof_bits);

    std::vector<ci8_t> expected(nof_symbols);
    modulator->modulate(expected, bits, scheme);

    std::vector<uint32_t>                  words  = pack_for_device(bits);
    cuda_expected<device_vector<uint32_t>> d_bits = device_vector<uint32_t>::create(words.size());
    ASSERT_TRUE(d_bits.has_value());
    ASSERT_TRUE(copy_to_device(d_bits.value(), span<const uint32_t>(words.data(), words.size())).has_value());

    cuda_expected<device_vector<int8_t>> d_symbols = device_vector<int8_t>::create(2 * nof_symbols);
    ASSERT_TRUE(d_symbols.has_value());

    ASSERT_TRUE(modulate_int8(d_symbols.value().data(), d_bits.value().data(), nof_bits, scheme, stream).has_value());
    ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

    std::vector<int8_t> actual(2 * nof_symbols);
    ASSERT_TRUE(copy_to_host(span<int8_t>(actual.data(), actual.size()), d_symbols.value()).has_value());

    for (unsigned i = 0; i != nof_symbols; ++i) {
      ASSERT_EQ(actual[2 * i], expected[i].real()) << to_string(scheme) << ", symbol " << i << ", real part";
      ASSERT_EQ(actual[2 * i + 1], expected[i].imag()) << to_string(scheme) << ", symbol " << i << ", imaginary part";
    }
  }
}

TEST_F(modulation_kernel_test, a_noiseless_round_trip_recovers_the_bits)
{
  // Demodulating what the mapper produced, with a noise variance far below the distance between
  // constellation points, must give back the bits that went in.
  for (modulation_scheme scheme : float_schemes) {
    unsigned           nof_symbols     = 2048;
    unsigned           bits_per_symbol = static_cast<unsigned>(scheme);
    unsigned           nof_bits        = nof_symbols * bits_per_symbol;
    dynamic_bit_buffer bits            = make_bits(nof_bits);

    std::vector<cf_t> symbols = modulate_on_device(bits, scheme);

    cuda_expected<device_vector<cuFloatComplex>> d_symbols = device_vector<cuFloatComplex>::create(nof_symbols);
    ASSERT_TRUE(d_symbols.has_value());
    ASSERT_TRUE(
        copy_to_device(d_symbols.value(),
                       span<const cuFloatComplex>(reinterpret_cast<const cuFloatComplex*>(symbols.data()), nof_symbols))
            .has_value());

    cuda_expected<device_vector<float>> d_llrs = device_vector<float>::create(nof_bits);
    ASSERT_TRUE(d_llrs.has_value());

    cuda_result status = soft_demodulate_constant_noise_var(
        d_llrs.value().data(), d_symbols.value().data(), nof_symbols, scheme, 0.01F, stream);
    ASSERT_TRUE(status.has_value());
    ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

    std::vector<float> llrs(nof_bits);
    ASSERT_TRUE(copy_to_host(span<float>(llrs.data(), llrs.size()), d_llrs.value()).has_value());

    for (unsigned i = 0; i != nof_bits; ++i) {
      uint8_t decided = static_cast<uint8_t>(llrs[i] <= 0.0F);
      ASSERT_EQ(decided, bits.extract(i, 1)) << to_string(scheme) << ", bit " << i;
    }
  }
}

TEST_F(modulation_kernel_test, the_per_symbol_demodulator_agrees_with_the_constant_variance_one)
{
  // Handing the per-symbol entry point the same variance for every symbol must reproduce what the
  // constant variance entry point computes.
  modulation_scheme  scheme      = modulation_scheme::QAM64;
  unsigned           nof_symbols = 1024;
  unsigned           nof_bits    = nof_symbols * static_cast<unsigned>(scheme);
  dynamic_bit_buffer bits        = make_bits(nof_bits);

  std::vector<cf_t> symbols = modulate_on_device(bits, scheme);

  cuda_expected<device_vector<cuFloatComplex>> d_symbols = device_vector<cuFloatComplex>::create(nof_symbols);
  ASSERT_TRUE(d_symbols.has_value());
  ASSERT_TRUE(
      copy_to_device(d_symbols.value(),
                     span<const cuFloatComplex>(reinterpret_cast<const cuFloatComplex*>(symbols.data()), nof_symbols))
          .has_value());

  std::vector<float>                  noise_vars(nof_symbols, 0.05F);
  cuda_expected<device_vector<float>> d_noise_vars = device_vector<float>::create(nof_symbols);
  ASSERT_TRUE(d_noise_vars.has_value());
  ASSERT_TRUE(
      copy_to_device(d_noise_vars.value(), span<const float>(noise_vars.data(), noise_vars.size())).has_value());

  cuda_expected<device_vector<float>> d_constant = device_vector<float>::create(nof_bits);
  cuda_expected<device_vector<float>> d_per      = device_vector<float>::create(nof_bits);
  ASSERT_TRUE(d_constant.has_value());
  ASSERT_TRUE(d_per.has_value());

  ASSERT_TRUE(soft_demodulate_constant_noise_var(
                  d_constant.value().data(), d_symbols.value().data(), nof_symbols, scheme, 0.05F, stream)
                  .has_value());
  ASSERT_TRUE(
      soft_demodulate(
          d_per.value().data(), d_symbols.value().data(), d_noise_vars.value().data(), nof_symbols, scheme, stream)
          .has_value());
  ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

  std::vector<float> constant(nof_bits);
  std::vector<float> per_symbol(nof_bits);
  ASSERT_TRUE(copy_to_host(span<float>(constant.data(), constant.size()), d_constant.value()).has_value());
  ASSERT_TRUE(copy_to_host(span<float>(per_symbol.data(), per_symbol.size()), d_per.value()).has_value());

  for (unsigned i = 0; i != nof_bits; ++i) {
    ASSERT_NEAR(per_symbol[i], constant[i], 1e-4) << "bit " << i;
  }
}

TEST_F(modulation_kernel_test, the_half_precision_demodulator_tracks_the_single_precision_one)
{
  // The half precision path exists to save a conversion pass, not to compute something else, so it
  // must agree with the single precision path to within what half precision can represent.
  for (modulation_scheme scheme : int8_schemes) {
    unsigned           nof_symbols = 1024;
    unsigned           nof_bits    = nof_symbols * static_cast<unsigned>(scheme);
    dynamic_bit_buffer bits        = make_bits(nof_bits);

    std::vector<cf_t> symbols = modulate_on_device(bits, scheme);

    cuda_expected<device_vector<cuFloatComplex>> d_symbols = device_vector<cuFloatComplex>::create(nof_symbols);
    ASSERT_TRUE(d_symbols.has_value());
    ASSERT_TRUE(
        copy_to_device(d_symbols.value(),
                       span<const cuFloatComplex>(reinterpret_cast<const cuFloatComplex*>(symbols.data()), nof_symbols))
            .has_value());

    std::vector<float>                  noise_vars(nof_symbols, 0.1F);
    cuda_expected<device_vector<float>> d_noise_vars = device_vector<float>::create(nof_symbols);
    ASSERT_TRUE(d_noise_vars.has_value());
    ASSERT_TRUE(
        copy_to_device(d_noise_vars.value(), span<const float>(noise_vars.data(), noise_vars.size())).has_value());

    cuda_expected<device_vector<float>>  d_single = device_vector<float>::create(nof_bits);
    cuda_expected<device_vector<__half>> d_half   = device_vector<__half>::create(nof_bits);
    ASSERT_TRUE(d_single.has_value());
    ASSERT_TRUE(d_half.has_value());

    ASSERT_TRUE(
        soft_demodulate(
            d_single.value().data(), d_symbols.value().data(), d_noise_vars.value().data(), nof_symbols, scheme, stream)
            .has_value());
    ASSERT_TRUE(
        soft_demodulate(
            d_half.value().data(), d_symbols.value().data(), d_noise_vars.value().data(), nof_symbols, scheme, stream)
            .has_value());
    ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

    std::vector<float>  single(nof_bits);
    std::vector<__half> half_llrs(nof_bits);
    ASSERT_TRUE(copy_to_host(span<float>(single.data(), single.size()), d_single.value()).has_value());
    ASSERT_TRUE(copy_to_host(span<__half>(half_llrs.data(), half_llrs.size()), d_half.value()).has_value());

    for (unsigned i = 0; i != nof_bits; ++i) {
      float converted = __half2float(half_llrs[i]);
      ASSERT_NEAR(converted, single[i], 0.01F * std::abs(single[i]) + 0.01F) << to_string(scheme) << ", bit " << i;
    }
  }
}

TEST_F(modulation_kernel_test, an_unimplemented_scheme_is_reported)
{
  // The GPU has no pi/2-BPSK constellation. The call fails with a description rather than writing
  // nothing and reporting success.
  cuda_expected<device_vector<uint32_t>>       d_bits    = device_vector<uint32_t>::create(4);
  cuda_expected<device_vector<cuFloatComplex>> d_symbols = device_vector<cuFloatComplex>::create(128);
  ASSERT_TRUE(d_bits.has_value());
  ASSERT_TRUE(d_symbols.has_value());

  cuda_result status =
      modulate(d_symbols.value().data(), d_bits.value().data(), 128, modulation_scheme::PI_2_BPSK, stream);
  ASSERT_FALSE(status.has_value());
  EXPECT_FALSE(status.error().empty());
}

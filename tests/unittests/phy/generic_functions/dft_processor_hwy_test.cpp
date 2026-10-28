// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Unit tests for the Highway DFT processor.
///
/// The processor computes in single precision and rounds the 16-bit integer output once, so that output is the ideal
/// DFT rounded to the nearest integer, except for the values within the floating point error of a rounding tie.
///
/// \note Disclaimer: this file was generated with Claude Code, the agentic coding tool from Anthropic, using the
/// Claude Opus 5.5 model (claude-opus-5-5).

#include "dft_processor_hwy_impl.h"
#include "hwy/targets.h"
#include "ocudu/adt/format.h"
#include "ocudu/phy/generic_functions/generic_functions_factories.h"
#include "ocudu/support/cpu_features.h"
#include "ocudu/support/math/complex_normal_random.h"
#include "fmt/ostream.h"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

using namespace ocudu;

namespace {

using direction = dft_processor::direction;
using cdouble   = std::complex<double>;

std::mt19937 rgen(1234);

/// \brief Computes a decimation-in-time DFT in double precision, reading the input with a stride.
///
/// It uses radix-2 steps while the size is even, and computes the odd sizes directly.
void ideal_dit(span<cdouble> out, span<const cdouble> in, unsigned stride, int sign)
{
  unsigned N = out.size();
  if (N == 1) {
    out[0] = in[0];
    return;
  }
  if (N % 2 != 0) {
    for (unsigned k = 0; k != N; ++k) {
      cdouble sum = 0;
      for (unsigned n = 0; n != N; ++n) {
        sum += std::polar(1.0, sign * 2.0 * M_PI * static_cast<double>((n * k) % N) / N) * in[n * stride];
      }
      out[k] = sum;
    }
    return;
  }
  ideal_dit(out.first(N / 2), in, 2 * stride, sign);
  ideal_dit(out.last(N / 2), in.last(in.size() - stride), 2 * stride, sign);
  for (unsigned k = 0; k != N / 2; ++k) {
    cdouble p      = out[k];
    cdouble q      = std::polar(1.0, sign * 2.0 * M_PI * k / N) * out[k + N / 2];
    out[k]         = p + q;
    out[k + N / 2] = p - q;
  }
}

/// Generates complex Gaussian values with a root mean square level relative to the full scale.
std::vector<cdouble> generate_gaussian(unsigned size, double level_dBFS)
{
  double                               amplitude = 32767.0 * std::pow(10.0, level_dBFS / 20.0);
  complex_normal_distribution<cdouble> dist(0.0, amplitude);
  std::vector<cdouble>                 out(size);
  std::generate(out.begin(), out.end(), [&]() { return dist(rgen); });
  return out;
}

/// Error metrics against the ideal output saturated to the 16-bit range, in LSB.
struct error_metrics {
  double max_error = 0;
  double error_rms = 0;
};

error_metrics measure(span<const ci16_t> dut, span<const cdouble> ideal)
{
  error_metrics result;
  double        energy = 0;
  for (unsigned i = 0, i_end = dut.size(); i != i_end; ++i) {
    cdouble target = {std::clamp(ideal[i].real(), -32768.0, 32767.0), std::clamp(ideal[i].imag(), -32768.0, 32767.0)};
    cdouble error  = cdouble(dut[i].real(), dut[i].imag()) - target;
    energy += std::norm(error);
    result.max_error = std::max({result.max_error, std::abs(error.real()), std::abs(error.imag())});
  }
  result.error_rms = std::sqrt(energy / dut.size());
  return result;
}

/// Writes double precision input values, that single precision represents exactly, in the DFT input.
void set_input(dft_processor& dft, span<const cdouble> x)
{
  std::copy(x.begin(), x.end(), dft.get_input().begin());
}

/// Runs the DFT with the 16-bit integer output.
std::vector<ci16_t> run_ci16(dft_processor& dft, span<const cdouble> x)
{
  set_input(dft, x);
  std::vector<ci16_t> out(x.size());
  dft.run(out);
  return out;
}

/// Quantizes the input to single precision.
std::vector<cdouble> quantize(span<const cdouble> x)
{
  std::vector<cdouble> out(x.size());
  std::transform(x.begin(), x.end(), out.begin(), [](cdouble v) { return cdouble(cf_t(v)); });
  return out;
}

/// Computes the ideal DFT.
std::vector<cdouble> ideal_dft(span<const cdouble> x, direction dir)
{
  std::vector<cdouble> ideal(x.size());
  ideal_dit(ideal, x, 1, (dir == direction::DIRECT) ? -1 : 1);
  return ideal;
}

/// One architecture name for each Highway target, the first is the baseline.
#if defined(__aarch64__)
const std::vector<std::string> architectures = {"neon", "native"};
#else  // defined(__aarch64__)
const std::vector<std::string> architectures = [] {
  std::vector<std::string> x86_archs = {"avx2", "native"};
  if (cpu_supports_feature(cpu_feature::avx512f) && cpu_supports_feature(cpu_feature::avx512bw) &&
      cpu_supports_feature(cpu_feature::avx512cd) && cpu_supports_feature(cpu_feature::avx512dq) &&
      cpu_supports_feature(cpu_feature::avx512vl)) {
    x86_archs.push_back("avx512");
  }
  return x86_archs;
}();
#endif // defined(__aarch64__)

/// Test parameters: architecture, DFT size and direction.
using HwyParams = std::tuple<std::string, unsigned, direction>;

std::string hwy_params_name(const ::testing::TestParamInfo<HwyParams>& info)
{
  std::string arch = std::get<0>(info.param);
  std::replace(arch.begin(), arch.end(), '-', '_');
  return fmt::format(
      "{}_{}_{}", arch, std::get<1>(info.param), dft_processor::direction_to_string(std::get<2>(info.param)));
}

/// \brief Largest error of the 16-bit integer output against the ideal output, in LSB.
///
/// Rounding contributes half an LSB. The single precision arithmetic contributes a small fraction of an LSB.
constexpr double max_error_bound = 0.55;

/// \brief Largest error root mean square of the 16-bit integer output of a DFT size, in LSB.
///
/// The uniform rounding error of the real and imaginary parts has a root mean square of 0.408. Its estimate over 128
/// outputs has a standard deviation of 0.0114, which decreases with the square root of the size. The bound allows four
/// standard deviations, and 0.43 at least.
double error_rms_bound(unsigned size)
{
  return std::max(0.43, 0.408 + 4.0 * 0.0114 * std::sqrt(128.0 / size));
}

/// Largest error root mean square of the single precision output, relative to the output root mean square.
constexpr double max_relative_error_rms = 1e-6;

class DFTHwyFixture : public ::testing::TestWithParam<HwyParams>
{};

} // namespace

// Verifies both outputs against the ideal DFT, for Gaussian outputs at -18 dBFS: the single precision output within
// the single precision error, and the 16-bit integer output rounded once.
TEST_P(DFTHwyFixture, Accuracy)
{
  auto [arch, size, dir] = GetParam();

  std::shared_ptr<dft_processor_factory> factory = create_dft_processor_factory_hwy(arch);
  ASSERT_NE(factory, nullptr) << fmt::format("Highway not available for {}.", arch);

  std::unique_ptr<dft_processor> dft = factory->create({.size = size, .dir = dir});
  ASSERT_NE(dft, nullptr);

  // The input level gives an output level of -18 dBFS.
  double               input_level_dB = -18.0 - 10.0 * std::log10(size);
  std::vector<cdouble> x              = quantize(generate_gaussian(size, input_level_dB));
  std::vector<cdouble> ideal          = ideal_dft(x, dir);

  // Single precision output.
  set_input(*dft, x);
  span<const cf_t> output       = dft->run();
  double           error_energy = 0;
  double           ideal_energy = 0;
  for (unsigned i = 0; i != size; ++i) {
    error_energy += std::norm(cdouble(output[i]) - ideal[i]);
    ideal_energy += std::norm(ideal[i]);
  }
  ASSERT_LE(std::sqrt(error_energy / ideal_energy), max_relative_error_rms);

  // 16-bit integer output.
  error_metrics metrics = measure(run_ci16(*dft, x), ideal);
  ASSERT_LE(metrics.max_error, max_error_bound);
  ASSERT_LE(metrics.error_rms, error_rms_bound(size));
}

INSTANTIATE_TEST_SUITE_P(DFTHwy,
                         DFTHwyFixture,
                         ::testing::Combine(::testing::ValuesIn(architectures),
                                            ::testing::Values(12,
                                                              24,
                                                              36,
                                                              60,
                                                              108,
                                                              128,
                                                              256,
                                                              300,
                                                              384,
                                                              512,
                                                              1024,
                                                              1500,
                                                              1536,
                                                              2048,
                                                              2916,
                                                              3240,
                                                              4096,
                                                              4608,
                                                              8192,
                                                              16384,
                                                              18432,
                                                              32768,
                                                              49152,
                                                              65536,
                                                              393216),
                                            ::testing::Values(direction::DIRECT, direction::INVERSE)),
                         hwy_params_name);

// Verifies that the factory does not create processors for unsupported sizes.
TEST(DFTHwyTest, UnsupportedSizes)
{
  std::shared_ptr<dft_processor_factory> factory = create_dft_processor_factory_hwy();
  if (factory == nullptr) {
    GTEST_SKIP() << "Highway not available.";
  }
  for (unsigned size : {7U, 14U, 839U, 1U << 23}) {
    ASSERT_EQ(factory->create({.size = size, .dir = direction::INVERSE}), nullptr) << size;
  }
}

// Verifies the saturation of the 16-bit integer output with full-scale inputs.
TEST(DFTHwyTest, Saturation)
{
  std::shared_ptr<dft_processor_factory> factory = create_dft_processor_factory_hwy();
  if (factory == nullptr) {
    GTEST_SKIP() << "Highway not available.";
  }
  for (unsigned size : {128, 2048, 16384}) {
    auto                 dft = factory->create({.size = size, .dir = direction::INVERSE});
    std::vector<cdouble> x   = generate_gaussian(size, 0.0);
    std::transform(x.begin(), x.end(), x.begin(), [](cdouble v) {
      return cdouble(std::clamp(std::round(v.real()), -32768.0, 32767.0),
                     std::clamp(std::round(v.imag()), -32768.0, 32767.0));
    });
    std::vector<cdouble> ideal = ideal_dft(x, direction::INVERSE);
    // The values are far beyond the 16-bit range, their single precision error is relative to them.
    double peak = 0;
    for (cdouble v : ideal) {
      peak = std::max({peak, std::abs(v.real()), std::abs(v.imag())});
    }
    error_metrics metrics = measure(run_ci16(*dft, x), ideal);
    ASSERT_LE(metrics.max_error, max_error_bound + 1e-6 * peak) << fmt::format("size {} peak {}", size, peak);
  }
}

// Verifies that every architecture that the CPU supports produces the same outputs as the baseline. The plans depend on
// whether the target has eight or more lanes, so only the architectures on the same side as the baseline are compared.
TEST(DFTHwyTest, ArchitecturesAreBitExact)
{
  std::shared_ptr<dft_processor_factory> reference_factory = create_dft_processor_factory_hwy(architectures.front());
  if (reference_factory == nullptr) {
    GTEST_SKIP() << "Highway not available.";
  }
  auto has_eight_lanes = [](const std::string& arch) {
    return get_dft_processor_hwy_max_lanes(resolve_dft_processor_hwy_target(arch)) >= 8;
  };

  for (const std::string& arch : architectures) {
    std::shared_ptr<dft_processor_factory> factory = create_dft_processor_factory_hwy(arch);
    if ((factory == nullptr) || (has_eight_lanes(arch) != has_eight_lanes(architectures.front()))) {
      continue;
    }
    for (unsigned size : {12, 36, 128, 300, 1024, 1536, 2048, 3240, 4608, 8192, 16384, 18432, 32768}) {
      for (direction dir : {direction::DIRECT, direction::INVERSE}) {
        auto                 reference = reference_factory->create({.size = size, .dir = dir});
        auto                 dft       = factory->create({.size = size, .dir = dir});
        std::vector<cdouble> x         = quantize(generate_gaussian(size, -18.0 - 10.0 * std::log10(size)));
        set_input(*reference, x);
        set_input(*dft, x);

        span<const cf_t>  output = reference->run();
        std::vector<cf_t> y_reference(output.begin(), output.end());
        output = dft->run();
        std::vector<cf_t> y(output.begin(), output.end());

        std::vector<ci16_t> y_reference_ci16(size);
        std::vector<ci16_t> y_ci16(size);
        reference->run(y_reference_ci16);
        dft->run(y_ci16);

        std::string info = fmt::format("{} size {} {}", arch, size, dft_processor::direction_to_string(dir));
        ASSERT_EQ(y, y_reference) << info;
        ASSERT_EQ(y_ci16, y_reference_ci16) << info;
      }
    }
  }
}

// Verifies every plan of the Highway DFT against the ideal DFT, and the 16-bit integer output against the single
// precision output.
TEST(DFTHwyTest, AllPlans)
{
  int64_t  target    = resolve_dft_processor_hwy_target("auto");
  unsigned max_lanes = get_dft_processor_hwy_max_lanes(target);
  if (max_lanes == 0) {
    GTEST_SKIP() << "Highway not available.";
  }

  for (unsigned size : {8, 12, 24, 36, 48, 60, 72, 96, 108, 120, 144, 180, 240, 256, 300, 384, 540, 720, 1500, 3072}) {
    std::vector<dft_hwy::dft_processor_hwy_plan> plans = dft_hwy::get_dft_processor_hwy_plans(size, max_lanes);
    ASSERT_FALSE(plans.empty()) << size;
    ASSERT_TRUE(dft_hwy::select_dft_processor_hwy_plan(size, max_lanes).has_value()) << size;
    for (const dft_hwy::dft_processor_hwy_plan& plan : plans) {
      for (direction dir : {direction::DIRECT, direction::INVERSE}) {
        std::string                    info = fmt::format("size {} {} N1 {} N2 {} L1 {} {}",
                                       size,
                                       plan.fused ? "fused" : "two passes",
                                       plan.N1,
                                       plan.N2,
                                       plan.L1,
                                       dft_processor::direction_to_string(dir));
        std::unique_ptr<dft_processor> dft  = create_dft_processor_hwy({.size = size, .dir = dir}, target, plan);
        ASSERT_NE(dft, nullptr) << info;

        std::vector<cdouble> x     = quantize(generate_gaussian(size, -18.0 - 10.0 * std::log10(size)));
        std::vector<cdouble> ideal = ideal_dft(x, dir);
        set_input(*dft, x);
        span<const cf_t> output       = dft->run();
        double           error_energy = 0;
        double           ideal_energy = 0;
        for (unsigned i = 0; i != size; ++i) {
          error_energy += std::norm(cdouble(output[i]) - ideal[i]);
          ideal_energy += std::norm(ideal[i]);
        }
        ASSERT_LE(std::sqrt(error_energy / ideal_energy), max_relative_error_rms) << info;

        std::vector<ci16_t> expected(size);
        std::transform(output.begin(), output.end(), expected.begin(), [](cf_t v) {
          return ci16_t(static_cast<int16_t>(std::nearbyint(v.real())), static_cast<int16_t>(std::nearbyint(v.imag())));
        });
        ASSERT_EQ(run_ci16(*dft, x), expected) << info;
      }
    }
  }
}

// Verifies the selection of the architecture.
TEST(DFTHwyTest, ArchitectureSelection)
{
  if (create_dft_processor_factory_hwy(architectures.front()) == nullptr) {
    GTEST_SKIP() << "Highway not available.";
  }

  // The automatic selection is always available.
  ASSERT_NE(create_dft_processor_factory_hwy(), nullptr);

  // The list of architectures has the names that give a factory.
  std::string list = get_dft_processor_hwy_architectures();
#if defined(__aarch64__)
  const std::vector<std::string> names = {"neon", "native"};
#else  // defined(__aarch64__)
  const std::vector<std::string> names = {"avx2", "avx512", "native"};
#endif // defined(__aarch64__)
  for (const std::string& arch : names) {
    bool listed = list.find(arch) != std::string::npos;
    ASSERT_EQ(listed, create_dft_processor_factory_hwy(arch) != nullptr) << arch << " in " << list;
  }

#if defined(__aarch64__)
  // NEON is the baseline.
  ASSERT_NE(create_dft_processor_factory_hwy("neon"), nullptr);
  ASSERT_EQ(create_dft_processor_factory_hwy("avx2"), nullptr);
#else  // defined(__aarch64__)
  // The architectures follow the instruction sets of the CPU, and the native architecture is the one of the build.
  int64_t supported = hwy::SupportedTargets();
  ASSERT_NE(create_dft_processor_factory_hwy("avx2"), nullptr);
  ASSERT_EQ(create_dft_processor_factory_hwy("avx512") != nullptr, (supported & HWY_AVX3) != 0);
  ASSERT_EQ(create_dft_processor_factory_hwy("neon"), nullptr);
#endif // defined(__aarch64__)

  // Unknown names give no factory.
  ASSERT_EQ(create_dft_processor_factory_hwy("foo"), nullptr);
  ASSERT_EQ(list.find("auto"), std::string::npos);
}

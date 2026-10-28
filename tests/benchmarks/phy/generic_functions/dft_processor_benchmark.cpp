// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/phy/generic_functions/generic_functions_factories.h"
#include "ocudu/ran/resource_block.h"
#include "ocudu/ran/transform_precoding/transform_precoding_helpers.h"
#include "ocudu/support/benchmark_utils.h"
#include "ocudu/support/error_handling.h"
#include "ocudu/support/ocudu_assert.h"
#include <algorithm>
#include <cmath>
#include <getopt.h>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace ocudu;

// Random generator.
static std::mt19937 rgen(0);

static std::string dft_type                   = "all";
static std::string fftw_optimization_str      = "estimate";
static double      fftw_plan_creation_timeout = 1.0;
static unsigned    nof_repetitions            = 1000;
static bool        silent                     = false;

static void usage(const char* prog)
{
  fmt::print("Usage: {} [-D <dft_type>] [-R repetitions]\n", prog);
  fmt::print("\t-D DFT library (fftw, fftz, generic, hwy, hwy:<arch>, all) [Default {}]\n"
             "\t   Highway architectures that this CPU supports: {}\n",
             dft_type,
             [] {
               std::string architectures = get_dft_processor_hwy_architectures();
               return architectures.empty() ? std::string("none") : architectures;
             }());
  fmt::print("\t-O Select FFTW optimization flag (estimate, measure, exhaustive) [Default {}]\n",
             fftw_optimization_str);
  fmt::print("\t-T Select FFTW plan creation maximum time in seconds, set to zero or lower for infinite [Default {}]\n",
             fftw_plan_creation_timeout);
  fmt::print("\t-R Repetitions [Default {}]\n", nof_repetitions);
  fmt::print("\t-s Toggle silent operation [Default {}]\n", silent);
  fmt::print("\t-h Show this message\n");
}

static void parse_args(int argc, char** argv)
{
  int opt = 0;
  while ((opt = getopt(argc, argv, "D:R:O:T:sh")) != -1) {
    switch (opt) {
      case 'D':
        dft_type = std::string(optarg);
        break;
      case 'R':
        nof_repetitions = std::strtol(optarg, nullptr, 10);
        break;
      case 'O':
        fftw_optimization_str = std::string(optarg);
        break;
      case 'T':
        fftw_plan_creation_timeout = std::strtod(optarg, nullptr);
        break;
      case 's':
        silent = (!silent);
        break;
      case 'h':
      default:
        usage(argv[0]);
        std::exit(0);
    }
  }
}

int main(int argc, char** argv)
{
  parse_args(argc, argv);

  std::uniform_real_distribution<float> dist(-1.0, +1.0);

  // Select the DFT factories. The Highway architecture follows the colon, auto by default. If none is specified
  // ("all"), benchmark all the available factories.
  std::vector<std::pair<std::string, std::shared_ptr<dft_processor_factory>>> dft_factories;
  bool                                                                        all = (dft_type == "all");
  if (all || (dft_type == "generic")) {
    dft_factories.emplace_back("generic", create_dft_processor_factory_generic());
  }
  if (all || (dft_type == "fftw")) {
    dft_factories.emplace_back("fftw",
                               create_dft_processor_factory_fftw(fftw_optimization_str, fftw_plan_creation_timeout));
  }
  if (all || (dft_type == "fftz")) {
    dft_factories.emplace_back("fftz", create_dft_processor_factory_fftz());
  }
  if (all || (dft_type == "hwy") || (dft_type.rfind("hwy:", 0) == 0)) {
    std::string arch = (all || (dft_type == "hwy")) ? "auto" : dft_type.substr(4);
    dft_factories.emplace_back((arch == "auto") ? "hwy" : fmt::format("hwy({})", arch),
                               create_dft_processor_factory_hwy(arch));
  }
  report_fatal_error_if_not(!dft_factories.empty(), "Unknown DFT factory type: {}", dft_type);

  // A selected DFT factory must be available. With all, the factories that are not available are skipped.
  if (!all) {
    report_fatal_error_if_not(dft_factories.front().second, "Failed to create DFT factory ({})", dft_type);
  }
  dft_factories.erase(std::remove_if(dft_factories.begin(),
                                     dft_factories.end(),
                                     [](const auto& entry) { return entry.second == nullptr; }),
                      dft_factories.end());

  benchmarker perf_meas("DFT", nof_repetitions);

  // Most common OFDM DFT sizes.
  std::set<unsigned> dft_sizes = {128,  256,  384,  512,  768,   1024,  1536,  2048,  3072,  4096,
                                  4608, 6144, 8192, 9216, 12288, 16384, 18432, 24576, 36864, 49152};

  // Transform precoding DFT sizes.
  span<const bool> valid_nof_prbs = transform_precoding::get_valid_nof_prbs();
  for (unsigned nof_prb = 0, nof_prb_end = valid_nof_prbs.size(); nof_prb != nof_prb_end; ++nof_prb) {
    if (valid_nof_prbs[nof_prb]) {
      dft_sizes.emplace(nof_prb * NOF_SUBCARRIERS_PER_RB);
    }
  }

  for (unsigned size : dft_sizes) {
    // Small DFTs run several times per measurement, so the clock overhead does not dominate.
    unsigned nof_runs = std::max(1U, 4096U / size);

    for (dft_processor::direction direction : {dft_processor::direction::DIRECT, dft_processor::direction::INVERSE}) {
      // Create FFTW configuration;
      dft_processor::configuration config;
      config.size = size;
      config.dir  = direction;

      for (const auto& [name, factory] : dft_factories) {
        std::unique_ptr<dft_processor> dft = factory->create(config);
        if (dft == nullptr) {
          continue;
        }

        // Generate input random data, with an output in the 16-bit integer range.
        span<cf_t> input = dft->get_input();
        for (cf_t& value : input) {
          value = cf_t(dist(rgen), dist(rgen)) * (8192.0F / std::sqrt(static_cast<float>(size)));
        }

        // Measure performance.
        std::string direction_str = dft_processor::direction_to_string(direction);
        perf_meas.new_measure(fmt::format("{} {} {}", name, size, direction_str), size * nof_runs, [&dft, nof_runs]() {
          for (unsigned i_run = 0; i_run != nof_runs; ++i_run) {
            dft->run();
          }
        });

        // The Highway DFT also writes 16-bit integers directly.
        if (name.rfind("hwy", 0) == 0) {
          std::vector<ci16_t> output(size);
          std::string ci16_name = (name.back() == ')') ? name.substr(0, name.size() - 1) + ",ci16)" : name + "(ci16)";
          perf_meas.new_measure(
              fmt::format("{} {} {}", ci16_name, size, direction_str), size * nof_runs, [&dft, &output, nof_runs]() {
                for (unsigned i_run = 0; i_run != nof_runs; ++i_run) {
                  dft->run(output);
                }
              });
        }
      }
    }
  }

  if (!silent) {
    perf_meas.print_percentiles_throughput("samples");
  }

  return 0;
}

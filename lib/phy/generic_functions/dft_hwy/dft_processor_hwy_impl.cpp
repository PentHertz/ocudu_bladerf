// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief DFT processor based on Google Highway: plans, tables and selection of the target.
///
/// See dft_processor_hwy_params.h for the algorithm. The passes of each target are in dft_processor_hwy_<target>.cpp,
/// and get_compiled_targets() lists them.
///
/// \note Disclaimer: this file was generated with Claude Code, the agentic coding tool from Anthropic, using the
/// Claude Opus 5.5 model (claude-opus-5-5).

#include "dft_processor_hwy_impl.h"
#include "dft_processor_hwy_params.h"
#include "hwy/aligned_allocator.h"
#include "hwy/targets.h"
#include "ocudu/support/error_handling.h"
#include "ocudu/support/ocudu_assert.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

using namespace ocudu;
using namespace dft_hwy;

/// Largest DFT size.
static constexpr unsigned max_dft_size = 1U << 22;

/// Largest size of the vertical DFTs.
static constexpr unsigned max_vertical_size = 4096;

/// \brief Architecture names and the passes of their Highway target.
///
/// The name auto, which is not listed, selects the best target that the CPU supports.
static const std::pair<const char*, target_kernels (*)()> architectures[] = {
#if HWY_ARCH_X86
    {"avx2", &get_target_kernels_avx2},
    {"avx512", &get_target_kernels_avx512},
    {"native", &get_target_kernels_native},
#elif HWY_ARCH_ARM_A64
    {"neon", &get_target_kernels_neon},
    {"native", &get_target_kernels_native},
#endif // HWY_ARCH_X86
};

/// Calculates the twiddle factor \f$e^{j \cdot sign \cdot 2 \pi k / n}\f$ in double precision and rounds it to single
/// precision.
static cf_t calculate_twiddle(uint64_t k, uint64_t n, int sign);

/// Gets the number of lanes of pass 2 for a target.
static unsigned get_pass2_lanes(unsigned max_lanes);

/// Estimates the number of vector operations of a DFT of size R in registers.
static double butterfly_cost(unsigned R);

/// \brief Estimates the number of vector operations of a vertical DFT of size M.
///
/// \param[in] load_cost Number of operations to load an input.
/// \param[in] sink_cost Number of operations to pass an output to the sink.
static double vertical_cost(unsigned M, double load_cost, double sink_cost);

/// Estimates the number of operations of the twiddle factors and the transposes of a block of L outputs of pass 1.
static double block_cost(unsigned L);

/// \brief Estimates the cost of a plan, in vector operations.
///
/// The cost is the same for the 256-bit and 512-bit targets, so that they select the same plan and give the same
/// results. Pass 2 is counted with eight lanes.
static double plan_cost(unsigned size, const dft_processor_hwy_plan& plan);

/// Returns true if the fused DFT supports a size N2 with L lanes.
static bool is_fused_size(unsigned N2, unsigned L);

/// Gets the passes of the targets of the architectures, without repetitions and from the best target.
static std::vector<target_kernels> get_compiled_targets();

/// Passes of the targets that the source files compile, from the best target.
static const std::vector<target_kernels>& compiled_targets();

/// Finds the passes of a target, or returns nullptr if it is not compiled.
static const target_kernels* find_target(int64_t target);

namespace {

/// Owns the tables of a vertical DFT.
class vertical_dft_tables
{
public:
  vertical_dft_tables(unsigned size_, int sign) : size(size_), plan(make_radix_plan(size_))
  {
    ocudu_assert(plan.nof_stages != 0, "Invalid vertical DFT size {}.", size);
    fixed = is_fixed_vertical_size(size);
    kernel_pos.resize(size / plan.radix[0]);
    assign_kernel_positions(plan.nof_stages - 1, 0, 0, 1, size);

    unsigned n = plan.radix[0];
    for (unsigned i_stage = 1; i_stage != plan.nof_stages; ++i_stage) {
      unsigned radix = plan.radix[i_stage];
      n *= radix;
      std::vector<float>& twiddles = stage_twiddles.emplace_back();
      for (unsigned k = 0; k != n / radix; ++k) {
        for (unsigned r = 1; r != radix; ++r) {
          cf_t w = calculate_twiddle(r * k, n, sign);
          twiddles.push_back(w.real());
          twiddles.push_back(w.imag());
        }
      }
    }
    for (const auto& twiddles : stage_twiddles) {
      stage_pointers.push_back(twiddles.data());
    }
  }

  vertical_dft get() const
  {
    return {size,
            kernel_pos.data(),
            static_cast<unsigned>(stage_pointers.size()),
            stage_pointers.empty() ? nullptr : stage_pointers.data(),
            plan,
            fixed};
  }

private:
  /// Assigns the output position of the kernels, following the decimation-in-time recursion from the top stage.
  void assign_kernel_positions(unsigned i_stage, unsigned out_pos, unsigned in_offset, unsigned stride, unsigned n)
  {
    if (i_stage == 0) {
      kernel_pos[in_offset] = out_pos;
      return;
    }
    unsigned radix = plan.radix[i_stage];
    for (unsigned r = 0; r != radix; ++r) {
      assign_kernel_positions(i_stage - 1, out_pos + r * n / radix, in_offset + r * stride, radix * stride, n / radix);
    }
  }

  unsigned                        size;
  radix_plan                      plan;
  bool                            fixed;
  std::vector<uint32_t>           kernel_pos;
  std::vector<std::vector<float>> stage_twiddles;
  std::vector<const float*>       stage_pointers;
};

/// \brief DFT processor based on Google Highway.
///
/// It calls the passes of its target through function pointers, selected at construction.
class dft_processor_hwy_impl : public dft_processor
{
public:
  dft_processor_hwy_impl(const configuration&          config,
                         const dft_processor_hwy_plan& plan,
                         const target_kernels&         kernels) :
    size(config.size), dir(config.dir), dft1(plan.N1, direction_sign()), dft2(plan.N2, direction_sign())
  {
    int      sign = direction_sign();
    unsigned N1   = plan.N1;
    unsigned N2   = plan.N2;
    unsigned L1   = plan.L1;
    unsigned L2   = get_pass2_lanes(kernels.max_lanes);

    // Twiddle factors of pass 1, W_N^(n2 * k1). For each group of lanes, the factors of the lanes are contiguous for
    // each k1, and for consecutive k1, so pass 1 reads the table sequentially.
    unsigned nof_groups = (N2 + L1 - 1) / L1;
    twiddle_re          = hwy::AllocateAligned<float>(nof_groups * L1 * N1);
    twiddle_im          = hwy::AllocateAligned<float>(nof_groups * L1 * N1);
    for (unsigned i_group = 0; i_group != nof_groups; ++i_group) {
      unsigned g = (N2 <= L1) ? 0 : std::min(i_group * L1, N2 - L1);
      for (unsigned k1 = 0; k1 != N1; ++k1) {
        for (unsigned lane = 0; lane != L1; ++lane) {
          cf_t     w        = calculate_twiddle(static_cast<uint64_t>(k1) * (g + lane), size, sign);
          unsigned index    = (i_group * N1 + k1) * L1 + lane;
          twiddle_re[index] = w.real();
          twiddle_im[index] = w.imag();
        }
      }
    }

    // The rows are padded by one cache line, so the columns do not map to few cache sets.
    unsigned work_stride = 0;
    if (!plan.fused) {
      work_stride = N1 + 16;
      work_re     = hwy::AllocateAligned<float>(N2 * work_stride);
      work_im     = hwy::AllocateAligned<float>(N2 * work_stride);
    }

    scratch = hwy::AllocateAligned<float>(2 * (plan.fused ? N1 * L1 : std::max(N1 * L1, N2 * L2)));

    // Pass 1 may read the input after the last sample, without using the values.
    input  = hwy::AllocateAligned<cf_t>(size + input_padding);
    output = hwy::AllocateAligned<cf_t>(size);
    std::fill(input.get(), input.get() + size + input_padding, cf_t());

    params   = {sign,
                N1,
                N2,
                L1,
                L2,
                dft1.get(),
                dft2.get(),
                twiddle_re.get(),
                twiddle_im.get(),
                work_stride,
                work_re.get(),
                work_im.get(),
                scratch.get()};
    run_cf   = kernels.select_cf(params);
    run_ci16 = kernels.select_ci16(params);
    report_fatal_error_if_not(
        (run_cf != nullptr) && (run_ci16 != nullptr), "Invalid Highway DFT plan for size {}.", size);
  }

  // See interface for documentation.
  direction get_direction() const override { return dir; }

  // See interface for documentation.
  unsigned get_size() const override { return size; }

  // See interface for documentation.
  span<cf_t> get_input() override { return {input.get(), size}; }

  // See interface for documentation.
  span<const cf_t> run() override
  {
    run_cf(params, output.get(), input.get());
    return {output.get(), size};
  }

  // See interface for documentation.
  void run(span<ci16_t> out) override
  {
    ocudu_assert(out.size() == size,
                 "The DFT output size (i.e., {}) is not equal to the DFT size (i.e., {}).",
                 out.size(),
                 size);
    run_ci16(params, out.data(), input.get());
  }

private:
  int direction_sign() const { return (dir == direction::DIRECT) ? -1 : 1; }

  unsigned                           size;
  direction                          dir;
  vertical_dft_tables                dft1;
  vertical_dft_tables                dft2;
  hwy::AlignedFreeUniquePtr<float[]> twiddle_re;
  hwy::AlignedFreeUniquePtr<float[]> twiddle_im;
  hwy::AlignedFreeUniquePtr<float[]> work_re;
  hwy::AlignedFreeUniquePtr<float[]> work_im;
  hwy::AlignedFreeUniquePtr<float[]> scratch;
  hwy::AlignedFreeUniquePtr<cf_t[]>  input;
  hwy::AlignedFreeUniquePtr<cf_t[]>  output;
  pass_params                        params;
  void (*run_cf)(const pass_params&, cf_t*, const cf_t*)     = nullptr;
  void (*run_ci16)(const pass_params&, ci16_t*, const cf_t*) = nullptr;
};

} // namespace

static cf_t calculate_twiddle(uint64_t k, uint64_t n, int sign)
{
  double angle = static_cast<double>(sign) * 2.0 * M_PI * static_cast<double>(k % n) / static_cast<double>(n);
  return {static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle))};
}

static unsigned get_pass2_lanes(unsigned max_lanes)
{
  return std::min(max_lanes, 16U);
}

static double butterfly_cost(unsigned R)
{
  switch (R) {
    case 2:
      return 4;
    case 3:
      return 14;
    case 4:
      return 16;
    case 5:
      return 36;
    case 6:
      return 52;
    case 8:
      return 58;
    case 9:
      return 108;
    case 10:
      return 116;
    case 12:
      return 140;
    case 15:
      return 226;
    default:
      return 173;
  }
}

static double vertical_cost(unsigned M, double load_cost, double sink_cost)
{
  radix_plan plan = make_radix_plan(M);
  // Kernels, and the stages with twiddle factors, two loads and two stores per point.
  double cost = (M / plan.radix[0]) * (butterfly_cost(plan.radix[0]) + plan.radix[0] * load_cost);
  for (unsigned i_stage = 1; i_stage != plan.nof_stages; ++i_stage) {
    unsigned R = plan.radix[i_stage];
    cost += (M / R) * (butterfly_cost(R) + 4 * R + 7 * (R - 1));
  }
  // Every stage but the last stores its outputs.
  cost += (plan.nof_stages - 1) * 2.0 * M + M * sink_cost;
  return cost;
}

static double block_cost(unsigned L)
{
  double transpose_cost = (L == 8) ? 24 : 8;
  return L * 10.0 + 2 * transpose_cost;
}

static double plan_cost(unsigned size, const dft_processor_hwy_plan& plan)
{
  unsigned N1         = plan.N1;
  unsigned N2         = plan.N2;
  unsigned L1         = plan.L1;
  unsigned nof_blocks = (N1 + L1 - 1) / L1;
  // The scratch buffer of a pass is slower beyond the first level cache.
  auto cache_factor = [](unsigned scratch_bytes) { return (scratch_bytes > 32768) ? 1.5 : 1.0; };

  double pass1 = vertical_cost(N1, 3, 2) * cache_factor(8 * N1 * L1);
  if (plan.fused) {
    return pass1 + nof_blocks * (block_cost(L1) + butterfly_cost(N2) + 3 * N2);
  }
  pass1 += nof_blocks * (block_cost(L1) + 2 * L1);

  unsigned L2    = (N1 >= 8) ? 8 : 4;
  double   pass2 = vertical_cost(N2, 2, 3) * cache_factor(8 * N2 * L2);
  // The work buffer is slower beyond the second level cache.
  double work = (size * 16 > (1U << 19)) ? size * 0.5 : 0;
  return ((N2 + L1 - 1) / L1) * pass1 + ((N1 + L2 - 1) / L2) * pass2 + work;
}

static bool is_fused_size(unsigned N2, unsigned L)
{
  return (N2 >= 2) && (N2 <= L) && is_kernel_radix(N2);
}

static std::vector<target_kernels> get_compiled_targets()
{
  std::vector<target_kernels> targets;
  for (const auto& [name, get_kernels] : architectures) {
    target_kernels kernels = get_kernels();
    bool           listed  = std::any_of(
        targets.begin(), targets.end(), [&kernels](const target_kernels& t) { return t.target == kernels.target; });
    if ((kernels.target != 0) && !listed) {
      targets.push_back(kernels);
    }
  }
  // The best target has the lowest value.
  std::sort(targets.begin(), targets.end(), [](const target_kernels& lhs, const target_kernels& rhs) {
    return lhs.target < rhs.target;
  });
  return targets;
}

static const std::vector<target_kernels>& compiled_targets()
{
  static const std::vector<target_kernels> targets = get_compiled_targets();
  return targets;
}

static const target_kernels* find_target(int64_t target)
{
  for (const target_kernels& kernels : compiled_targets()) {
    if (kernels.target == target) {
      return &kernels;
    }
  }
  return nullptr;
}

std::vector<dft_processor_hwy_plan> ocudu::dft_hwy::get_dft_processor_hwy_plans(unsigned size, unsigned max_lanes)
{
  std::vector<dft_processor_hwy_plan> plans;
  if ((size > max_dft_size) || (make_radix_plan(size).nof_stages == 0)) {
    return plans;
  }
  for (unsigned N1 = 4; N1 <= std::min(size, max_vertical_size); ++N1) {
    if (size % N1 != 0) {
      continue;
    }
    unsigned N2 = size / N1;
    if ((N2 > max_vertical_size) || (make_radix_plan(N1).nof_stages == 0) || (make_radix_plan(N2).nof_stages == 0)) {
      continue;
    }
    for (unsigned L1 : {4U, 8U}) {
      if ((L1 > max_lanes) || (N1 < L1)) {
        continue;
      }
      if (is_fused_size(N2, L1)) {
        plans.push_back({true, N1, N2, L1});
      }
      // Pass 2 uses the widest vector of the target, up to sixteen lanes.
      if ((N2 >= L1) && (N1 >= ((max_lanes >= 8) ? 16U : 4U))) {
        plans.push_back({false, N1, N2, L1});
      }
    }
  }
  return plans;
}

std::optional<dft_processor_hwy_plan> ocudu::dft_hwy::select_dft_processor_hwy_plan(unsigned size, unsigned max_lanes)
{
  std::vector<dft_processor_hwy_plan> plans = get_dft_processor_hwy_plans(size, std::min(max_lanes, 8U));
  if (plans.empty()) {
    return std::nullopt;
  }
  auto is_valid = [&plans](const dft_processor_hwy_plan& plan) {
    return std::any_of(plans.begin(), plans.end(), [&plan](const dft_processor_hwy_plan& other) {
      return (other.fused == plan.fused) && (other.N1 == plan.N1) && (other.N2 == plan.N2) && (other.L1 == plan.L1);
    });
  };

  // The rules come from measurements of all the plans. They depend on the lanes only through the targets with four
  // lanes, so all the x86 targets select the same plan and give the same results.
  if (max_lanes >= 8) {
    // The powers of two from 4096 use pass 1 DFTs of size 256 that fill the first level cache. The sizes 4096 and 8192
    // use pass 1 DFTs of size 128, which have fewer cache misses for the same speed.
    bool is_pow2 = (size & (size - 1)) == 0;
    if (is_pow2 && (size >= 4096) && (size <= 65536)) {
      unsigned N1 = (size <= 8192) ? 128 : 256;
      return dft_processor_hwy_plan{false, N1, size / N1, 8};
    }

    // The fused DFT is the fastest up to sizes of pass 1 that fit in the second level cache. Eight lanes of pass 1 are
    // the most efficient, so the size of pass 2 is preferably eight, or six.
    if (size <= 18432) {
      for (unsigned N2 : {8U, 6U, 5U, 4U, 3U, 2U}) {
        if (size % N2 != 0) {
          continue;
        }
        dft_processor_hwy_plan plan = {true, size / N2, N2, (N2 > 4) ? 8U : 4U};
        if (is_valid(plan)) {
          return plan;
        }
      }
    }

    // Two passes with pass 1 DFTs of size 256 or 128.
    for (unsigned N1 : {256U, 128U}) {
      if ((size % N1 == 0) && (size / N1 >= 64)) {
        dft_processor_hwy_plan plan = {false, N1, size / N1, 8};
        if (is_valid(plan)) {
          return plan;
        }
      }
    }
  }

  // Otherwise, the plan with the lowest estimated cost.
  std::optional<dft_processor_hwy_plan> best;
  double                                best_cost = 0;
  for (const dft_processor_hwy_plan& plan : plans) {
    double cost = plan_cost(size, plan);
    if (!best.has_value() || (cost < best_cost)) {
      best      = plan;
      best_cost = cost;
    }
  }
  return best;
}

int64_t ocudu::resolve_dft_processor_hwy_target(const std::string& arch)
{
  int64_t supported = hwy::SupportedTargets();

  // The best compiled target that the CPU supports.
  if (arch == "auto") {
    for (const target_kernels& kernels : compiled_targets()) {
      if ((supported & kernels.target) != 0) {
        return kernels.target;
      }
    }
    return 0;
  }

  for (const auto& [name, get_kernels] : architectures) {
    if (arch == std::string(name)) {
      int64_t target = get_kernels().target;
      return ((supported & target) != 0) ? target : 0;
    }
  }
  return 0;
}

unsigned ocudu::get_dft_processor_hwy_max_lanes(int64_t target)
{
  const target_kernels* kernels = find_target(target);
  return (kernels == nullptr) ? 0 : kernels->max_lanes;
}

std::unique_ptr<dft_processor> ocudu::create_dft_processor_hwy(const dft_processor::configuration& config,
                                                               int64_t                             target,
                                                               const dft_processor_hwy_plan&       plan)
{
  const target_kernels* kernels = find_target(target);
  if (kernels == nullptr) {
    return nullptr;
  }
  return std::make_unique<dft_processor_hwy_impl>(config, plan, *kernels);
}

std::unique_ptr<dft_processor> ocudu::create_dft_processor_hwy(const dft_processor::configuration& config,
                                                               int64_t                             target)
{
  unsigned max_lanes = get_dft_processor_hwy_max_lanes(target);
  if (max_lanes == 0) {
    return nullptr;
  }
  std::optional<dft_processor_hwy_plan> plan = select_dft_processor_hwy_plan(config.size, max_lanes);
  if (!plan.has_value()) {
    return nullptr;
  }
  return create_dft_processor_hwy(config, target, *plan);
}

std::string ocudu::list_dft_processor_hwy_architectures()
{
  std::string result;
  for (const auto& [name, get_kernels] : architectures) {
    if (resolve_dft_processor_hwy_target(name) == 0) {
      continue;
    }
    if (!result.empty()) {
      result += ", ";
    }
    result += name;
  }
  return result;
}

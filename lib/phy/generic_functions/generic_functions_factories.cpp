// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/phy/generic_functions/generic_functions_factories.h"
#include "dft_processor_generic_impl.h"
#include "ocudu/support/error_handling.h"

#ifdef HAVE_FFTW
#include "dft_processor_fftw_impl.h"
#endif // HAVE_FFTW

#ifdef HAVE_FFTZ
#include "dft_processor_fftz_impl.h"
#endif // HAVE_FFTZ

#ifdef HAVE_HWY
#include "dft_hwy/dft_processor_hwy_impl.h"
#endif // HAVE_HWY

// Check if any Clang sanitizer is enabled.
#ifdef __has_feature
#if __has_feature(thread_sanitizer) || __has_feature(memory_sanitizer) || __has_feature(realtime_sanitizer) ||         \
    __has_feature(address_sanitizer)
#define ANY_SANITIZER
#endif

// Check if any GCC sanitizer is enabled.
#else
#if defined(__SANITIZE_THREAD__) || defined(__SANITIZE_ADDRESS__)
#define ANY_SANITIZER
#endif
#endif

using namespace ocudu;

namespace {

class dft_processor_factory_generic : public dft_processor_factory
{
public:
  std::unique_ptr<dft_processor> create(const dft_processor::configuration& config) override
  {
    std::unique_ptr<dft_processor_generic_impl> dft = std::make_unique<dft_processor_generic_impl>(config);
    if (!dft->is_valid()) {
      return nullptr;
    }
    return dft;
  }
};

#ifdef HAVE_HWY
class dft_processor_factory_hwy : public dft_processor_factory
{
public:
  explicit dft_processor_factory_hwy(int64_t target_) : target(target_) {}

  std::unique_ptr<dft_processor> create(const dft_processor::configuration& config) override
  {
    return create_dft_processor_hwy(config, target);
  }

private:
  /// Highway target of the processors.
  int64_t target;
};
#endif // HAVE_HWY

#ifdef HAVE_FFTW
class dft_processor_factory_fftw : public dft_processor_factory
{
  dft_processor_fftw_configuration fftw_config;

public:
  dft_processor_factory_fftw(const std::string& optimization_flag,
                             double             plan_creation_timeout_s,
                             bool               avoid_wisdom,
                             const std::string& wisdom_filename)
  {
    fftw_config.avoid_wisdom            = avoid_wisdom;
    fftw_config.wisdom_filename         = wisdom_filename;
    fftw_config.plan_creation_timeout_s = plan_creation_timeout_s;

    if ((optimization_flag == "") || (optimization_flag == "estimate")) {
      fftw_config.optimization_flag = fftw_plan_optimization::fftw_estimate;
    } else if (optimization_flag == "measure") {
      fftw_config.optimization_flag = fftw_plan_optimization::fftw_measure;
    } else if (optimization_flag == "exhaustive") {
      fftw_config.optimization_flag = fftw_plan_optimization::fftw_exhaustive;
    } else {
      report_error("Invalid FFTW optimization flag: {}\n", optimization_flag);
    }
  }

private:
  std::unique_ptr<dft_processor> create(const dft_processor::configuration& dft_config) override
  {
    return std::make_unique<dft_processor_fftw_impl>(fftw_config, dft_config);
  }
};
#endif // HAVE_FFTW

#ifdef HAVE_FFTZ
class dft_processor_factory_fftz : public dft_processor_factory
{
  dft_processor_fftz_configuration fftz_config;

public:
  dft_processor_factory_fftz()
  {
    // Disable bit reproducibility mode and enable maximum optimization level.
    fftz_config.bit_reproducibility = false;
    fftz_config.opt_level           = 3;
  }

private:
  std::unique_ptr<dft_processor> create(const dft_processor::configuration& dft_config) override
  {
    return std::make_unique<dft_processor_fftz_impl>(fftz_config, dft_config);
  }
};
#endif // HAVE_FFTZ

} // namespace

std::shared_ptr<dft_processor_factory> ocudu::create_dft_processor_factory_generic()
{
  return std::make_shared<dft_processor_factory_generic>();
}

std::shared_ptr<dft_processor_factory> ocudu::create_dft_processor_factory_fftw(const std::string& optimization_flag,
                                                                                double plan_creation_timeout_s,
                                                                                bool   avoid_wisdom,
                                                                                const std::string& wisdom_filename)
{
#ifdef HAVE_FFTW
  return std::make_shared<dft_processor_factory_fftw>(
      optimization_flag, plan_creation_timeout_s, avoid_wisdom, wisdom_filename);
#else  // HAVE_FFTW
  return nullptr;
#endif // HAVE_FFTW
}

std::shared_ptr<dft_processor_factory> ocudu::create_dft_processor_factory_fftz()
{
#ifdef HAVE_FFTZ
  return std::make_shared<dft_processor_factory_fftz>();
#else  // HAVE_FFTZ
  return nullptr;
#endif // HAVE_FFTZ
}

std::shared_ptr<dft_processor_factory> ocudu::create_dft_processor_factory_hwy(const std::string& arch)
{
#ifdef HAVE_HWY
  int64_t target = resolve_dft_processor_hwy_target(arch);
  if (target == 0) {
    return nullptr;
  }
  return std::make_shared<dft_processor_factory_hwy>(target);
#else  // HAVE_HWY
  return nullptr;
#endif // HAVE_HWY
}

std::string ocudu::get_dft_processor_hwy_architectures()
{
#ifdef HAVE_HWY
  return list_dft_processor_hwy_architectures();
#else  // HAVE_HWY
  return "";
#endif // HAVE_HWY
}

std::shared_ptr<dft_processor_factory> ocudu::create_dft_processor_factory_fftw()
{
#if !defined(BUILD_TYPE_RELEASE) || !defined(__x86_64__) || defined(ANY_SANITIZER)
  return create_dft_processor_factory_fftw_slow();
#else
  return create_dft_processor_factory_fftw_fast();
#endif
}

std::shared_ptr<dft_processor_factory> ocudu::create_dft_processor_factory(const std::string& type)
{
  if (type == "hwy") {
    return create_dft_processor_factory_hwy();
  }
  if (type.rfind("hwy:", 0) == 0) {
    // The architecture follows the colon.
    return create_dft_processor_factory_hwy(type.substr(4));
  }
  if (type == "fftw") {
    return create_dft_processor_factory_fftw();
  }
  if (type == "fftw_fast") {
    return create_dft_processor_factory_fftw_fast();
  }
  if (type == "fftw_slow") {
    return create_dft_processor_factory_fftw_slow();
  }
  if (type == "fftz") {
    return create_dft_processor_factory_fftz();
  }
  if (type == "generic") {
    return create_dft_processor_factory_generic();
  }
  if (type != "auto") {
    return nullptr;
  }

  std::shared_ptr<dft_processor_factory> dft_proc_factory = create_dft_processor_factory_hwy();
  if (!dft_proc_factory) {
    dft_proc_factory = create_dft_processor_factory_fftw();
  }
  if (!dft_proc_factory) {
    dft_proc_factory = create_dft_processor_factory_fftz();
  }
  if (!dft_proc_factory) {
    dft_proc_factory = create_dft_processor_factory_generic();
  }
  return dft_proc_factory;
}

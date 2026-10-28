// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "ocudu/phy/generic_functions/dft_processor.h"
#include <memory>
#include <string>

namespace ocudu {

/// Discrete Fourier Transform (DFT) processor factory.
class dft_processor_factory
{
public:
  /// Default destructor.
  virtual ~dft_processor_factory() = default;

  /// \brief Creates a DFT processor.
  /// \param[in] config Provides the DFT processor parameters.
  /// \return A unique pointer of a DFT processor if the configuration is valid. Otherwise, \c nullptr.
  virtual std::unique_ptr<dft_processor> create(const dft_processor::configuration& config) = 0;
};

/// \brief Creates a DFT processor factory based on a generic Radix-2 DFT implementation.
std::shared_ptr<dft_processor_factory> create_dft_processor_factory_generic();

/// \brief Creates a DFT processor factory based on FFTW library.
/// \param[in] optimization_flag       FFTW planning optimization mode. Leave empty for default value. Available modes:
///                                      - \c estimate picks a sub-optimal plan quickly, using heuristics.
///                                      - \c measure finds an optimized FFT plan by computing several FFTs and
///                                        measuring their execution time.
///                                      - \c exhaustive is like \c measure but it tries to achieve an even higher
///                                        optimization.
/// \param[in] plan_creation_timeout_s FFTW plan creation maximum duration in seconds. Set to zero or lower for
///                                    infinite.
/// \param[in] avoid_wisdom            Set to true to avoid loading FFTW wisdom from a file.
/// \param[in] wisdom_filename         FFTW wisdom file name. Leave empty for default value. It is ignored if the option
///                                    \c avoid_wisdom is true.
/// \return A valid pointer to a DFT processor factory if FFTW is available. Otherwise, \c nullptr.
/// \note If the plan creation timeout is reached, the selected plan is sub-optimal. However, it will always be more
///       optimal than \c estimate.
std::shared_ptr<dft_processor_factory> create_dft_processor_factory_fftw(const std::string& optimization_flag,
                                                                         double             plan_creation_timeout_s,
                                                                         bool               avoid_wisdom    = false,
                                                                         const std::string& wisdom_filename = "");

/// \brief Creates a DFT processor factory based on the AMD Optimized FFT for Zen (AOCL-FFTZ) library.
/// \return A valid pointer to a DFT processor factory if AOCL-FFTZ is available. Otherwise, \c nullptr.
std::shared_ptr<dft_processor_factory> create_dft_processor_factory_fftz();

/// \brief Creates a sub-optimal DFT processor factory based on FFTW library.
/// \param[in] avoid_wisdom    Set to true to avoid loading FFTW wisdom from a file.
/// \param[in] wisdom_filename FFTW wisdom file name. Leave empty for default value. It is ignored if the option
///                            \c avoid_wisdom is true.
/// \return A valid pointer to a DFT processor factory if FFTW is available. Otherwise, \c nullptr.
inline std::shared_ptr<dft_processor_factory>
create_dft_processor_factory_fftw_slow(bool avoid_wisdom = false, const std::string& wisdom_filename = "")
{
  return create_dft_processor_factory_fftw("estimate", 1.0, avoid_wisdom, wisdom_filename);
}

/// \brief Creates an optimal DFT processor factory based on FFTW library.
/// \param[in] avoid_wisdom    Set to true to avoid loading FFTW wisdom from a file.
/// \param[in] wisdom_filename FFTW wisdom file name. Leave empty for default value. It is ignored if the option
///                            \c avoid_wisdom is true.
/// \return A valid pointer to a DFT processor factory if FFTW is available. Otherwise, \c nullptr.
inline std::shared_ptr<dft_processor_factory>
create_dft_processor_factory_fftw_fast(bool avoid_wisdom = false, const std::string& wisdom_filename = "")
{
  return create_dft_processor_factory_fftw("measure", 5.0, avoid_wisdom, wisdom_filename);
}

/// \brief Creates a DFT processor factory based on the FFTW library using a build-dependent optimization profile.
///
/// It uses a suboptimal FFTW plan in non-release builds, when building on architectures other than \c x86_64, or when
/// thread or memory sanitizers are enabled.
/// \return A valid pointer to a DFT processor factory if FFTW is available. Otherwise, \c nullptr.
std::shared_ptr<dft_processor_factory> create_dft_processor_factory_fftw();

/// \brief Creates a factory for DFT processors based on Google Highway.
///
/// The processors carry out internal computations in single precision floating point, for DFT sizes that factor into 2,
/// 3 and 5. They round the 16-bit integer output once.
///
/// \param[in] arch Architecture of the processors. On x86: avx2 or avx512. On ARM: neon. The name native selects the
///                 architecture of the project compiler options, and the default, auto, selects the best architecture
///                 that the CPU supports.
/// \return A DFT processor factory if the library is built with Google Highway, and the architecture is known and
/// supported by the CPU. Otherwise, \c nullptr.
std::shared_ptr<dft_processor_factory> create_dft_processor_factory_hwy(const std::string& arch = "auto");

/// \brief Factory helper that creates a DFT processor factory of a given type.
///
/// The default type, auto, selects the best available DFT processor according to the following priority list:
/// 1. Google Highway;
/// 2. Fastest FFT in the West (FFTW);
/// 3. AMD Optimized Computing Library FFT for Zen (AOCL-FFTZ); and
/// 4. Generic DFT which might not support all DFT sizes.
///
/// \param[in] type DFT processor type. The types are:
///                 - auto: the best available, as above;
///                 - hwy: Google Highway, with the best architecture that the CPU supports;
///                 - hwy:<arch>: Google Highway, with an architecture of create_dft_processor_factory_hwy();
///                 - fftw: FFTW, with the build-dependent profile of create_dft_processor_factory_fftw();
///                 - fftw_fast: FFTW, with the optimal profile;
///                 - fftw_slow: FFTW, with the suboptimal profile;
///                 - fftz: AOCL-FFTZ; and
///                 - generic: generic DFT.
/// \return A valid pointer to a DFT processor factory if the type is known and available. Otherwise, \c nullptr.
std::shared_ptr<dft_processor_factory> create_dft_processor_factory(const std::string& type = "auto");

/// \brief Gets the architectures of the Highway DFT processors that the CPU supports.
/// \return The architecture names, separated by commas, or an empty string if Highway is not available.
std::string get_dft_processor_hwy_architectures();

} // namespace ocudu

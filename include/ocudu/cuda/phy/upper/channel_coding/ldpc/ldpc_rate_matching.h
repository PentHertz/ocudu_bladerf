// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief LDPC rate matching and rate dematching on the GPU, as per TS 38.212, Section 5.4.2.

#pragma once

#include "ocudu/cuda/adt/cuda_error.h"
#include "ocudu/cuda/adt/cuda_stream.h"
#include "ocudu/phy/upper/codeblock_metadata.h"
#include <cstdint>
#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <memory>

namespace ocudu {
namespace cuda {

/// \brief Selects the bits of a codeword that go on the air, as per TS 38.212, Section 5.4.2.
///
/// A rate matcher is configured for a code block and then matches any number of code blocks of
/// that shape. The entry points take and return device memory and queue their work on a stream,
/// and none of them synchronises.
class ldpc_rate_matcher
{
public:
  /// \brief Creates a rate matcher.
  /// \return A rate matcher, or a description of the error.
  static cuda_expected<ldpc_rate_matcher> create();

  /// Creates an empty rate matcher that holds no state.
  ldpc_rate_matcher();

  ~ldpc_rate_matcher();

  ldpc_rate_matcher(const ldpc_rate_matcher&)            = delete;
  ldpc_rate_matcher& operator=(const ldpc_rate_matcher&) = delete;

  ldpc_rate_matcher(ldpc_rate_matcher&& other) noexcept;
  ldpc_rate_matcher& operator=(ldpc_rate_matcher&& other) noexcept;

  /// Returns true when the rate matcher holds state.
  bool is_valid() const;

  /// \brief Sets the code block shape the next calls match.
  ///
  /// The base graph, the lifting size, the redundancy version, the rate matched length and the
  /// number of filler bits are read from the metadata, which is the same description the software
  /// rate matcher takes.
  ///
  /// \param[in] cfg Code block description.
  /// \return A successful result, or a description of the error.
  cuda_result configure(const codeblock_metadata& cfg);

  /// Number of bits a matched code block occupies, as configured.
  unsigned get_output_bits() const;

  /// \brief Matches a batch of code blocks.
  ///
  /// \param[out] d_matched     Matched bits, in device memory.
  /// \param[in]  d_codewords   Encoded code blocks, in device memory, at the configured stride.
  /// \param[in]  nof_codeblocks Number of code blocks in the batch.
  /// \param[in]  word_stride   Words between the matched code blocks, or 0 to pack them.
  /// \param[in]  stream        Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result rate_match(uint32_t*          d_matched,
                         const uint32_t*    d_codewords,
                         unsigned           nof_codeblocks,
                         unsigned           word_stride,
                         const cuda_stream& stream);

  /// \brief Interleaves the bits across the bits of a modulation symbol.
  ///
  /// \param[out] d_output         Interleaved bits, in device memory.
  /// \param[in]  d_input          Matched bits, in device memory.
  /// \param[in]  nof_bits         Number of bits to interleave.
  /// \param[in]  bits_per_symbol  Number of bits a modulation symbol carries.
  /// \param[in]  stream           Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result interleave(uint32_t*          d_output,
                         const uint32_t*    d_input,
                         unsigned           nof_bits,
                         unsigned           bits_per_symbol,
                         const cuda_stream& stream);

private:
  struct impl;

  explicit ldpc_rate_matcher(std::unique_ptr<impl> state_);

  std::unique_ptr<impl> state;
};

/// \brief Restores a codeword from the log likelihood ratios the air carried, as per TS 38.212,
/// Section 5.4.2.
///
/// Dematching reverses the selection and restores the ratios of the bits that were punctured or
/// filled, which the air never carried and which the decoder must be told nothing about.
class ldpc_rate_dematcher
{
public:
  /// \brief Creates a rate dematcher.
  /// \return A rate dematcher, or a description of the error.
  static cuda_expected<ldpc_rate_dematcher> create();

  /// Creates an empty rate dematcher that holds no state.
  ldpc_rate_dematcher();

  ~ldpc_rate_dematcher();

  ldpc_rate_dematcher(const ldpc_rate_dematcher&)            = delete;
  ldpc_rate_dematcher& operator=(const ldpc_rate_dematcher&) = delete;

  ldpc_rate_dematcher(ldpc_rate_dematcher&& other) noexcept;
  ldpc_rate_dematcher& operator=(ldpc_rate_dematcher&& other) noexcept;

  /// Returns true when the rate dematcher holds state.
  bool is_valid() const;

  /// \brief Sets the code block shape the next calls dematch.
  /// \param[in] cfg Code block description.
  /// \return A successful result, or a description of the error.
  cuda_result configure(const codeblock_metadata& cfg);

  /// Number of ratios a dematched code block occupies, as configured.
  unsigned get_output_bits() const;

  /// \brief Dematches a batch of code blocks.
  ///
  /// \param[out] d_output       Dematched ratios, in device memory.
  /// \param[in]  d_received     Received ratios, in device memory.
  /// \param[in]  nof_codeblocks Number of code blocks in the batch.
  /// \param[in]  stream         Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result
  rate_dematch(float* d_output, const float* d_received, unsigned nof_codeblocks, const cuda_stream& stream);

  /// \brief Dematches a batch of code blocks held in half precision.
  ///
  /// \param[out] d_output       Dematched ratios, in device memory.
  /// \param[in]  d_received     Received ratios, in device memory.
  /// \param[in]  nof_codeblocks Number of code blocks in the batch.
  /// \param[in]  stream         Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result
  rate_dematch(__half* d_output, const __half* d_received, unsigned nof_codeblocks, const cuda_stream& stream);

  /// \brief Reverses the interleaving and the selection in 1 pass, in half precision.
  ///
  /// Doing both in 1 pass spares the buffer that holds the deinterleaved ratios, which is the
  /// larger cost at the sizes a receiver works with.
  ///
  /// \param[out] d_output         Dematched ratios, in device memory.
  /// \param[in]  d_received       Received ratios, interleaved, in device memory.
  /// \param[in]  bits_per_symbol  Number of bits a modulation symbol carries.
  /// \param[in]  nof_codeblocks   Number of code blocks in the batch.
  /// \param[in]  stream           Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result deinterleave_and_dematch(__half*            d_output,
                                       const __half*      d_received,
                                       unsigned           bits_per_symbol,
                                       unsigned           nof_codeblocks,
                                       const cuda_stream& stream);

  /// \brief Reverses the interleaving across the bits of a modulation symbol.
  ///
  /// \param[out] d_output         Deinterleaved ratios, in device memory.
  /// \param[in]  d_input          Received ratios, in device memory.
  /// \param[in]  nof_bits         Number of ratios to deinterleave.
  /// \param[in]  bits_per_symbol  Number of bits a modulation symbol carries.
  /// \param[in]  nof_codeblocks   Number of code blocks in the batch.
  /// \param[in]  stream           Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result deinterleave(float*             d_output,
                           const float*       d_input,
                           unsigned           nof_bits,
                           unsigned           bits_per_symbol,
                           unsigned           nof_codeblocks,
                           const cuda_stream& stream);

private:
  struct impl;

  explicit ldpc_rate_dematcher(std::unique_ptr<impl> state_);

  std::unique_ptr<impl> state;
};

} // namespace cuda
} // namespace ocudu

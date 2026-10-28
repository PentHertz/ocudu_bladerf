// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief LDPC decoding on the GPU, as per TS 38.212, Section 5.3.2.
///
/// The decoder runs layered belief propagation over the base graph. It decodes a batch of code
/// blocks in a single launch rather than one code block at a time, and leaves a code block alone once its
/// parity check passes.
///
/// The decode kernels can also undo the scrambling and the rate matching on the way in, so a
/// caller that holds the received ratios as the air delivered them need not write a descrambled
/// and dematched copy first.

#pragma once

#include "ocudu/cuda/adt/cuda_error.h"
#include "ocudu/cuda/adt/cuda_stream.h"
#include "ocudu/phy/upper/channel_coding/ldpc/ldpc.h"
#include "ocudu/ran/sch/ldpc_base_graph.h"
#include <cstdint>
#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <memory>

namespace ocudu {
namespace cuda {

/// \brief Checksum the decoder uses to decide a code block is already correct.
///
/// TS 38.212 attaches a different checksum depending on how the transport block was segmented.
enum class ldpc_decoder_crc {
  /// No checksum. The decoder stops on confidence alone.
  none,
  /// A transport block of at most 3824 bits carried in 1 code block.
  crc16,
  /// A larger transport block carried in 1 code block.
  crc24a,
  /// A transport block carried in several code blocks.
  crc24b
};

/// \brief What the decoder does, beyond the shape of the code.
///
/// Every member carries the value the decoder uses when a caller sets nothing.
struct ldpc_decoder_params {
  /// Largest number of iterations a code block is given.
  unsigned max_iterations = 10;
  /// Factor the minimum is scaled by, ignored when \c auto_scale is set.
  float min_sum_scale = 0.75F;
  /// Offset subtracted from the minimum, 0 for plain min-sum.
  float min_sum_offset = 0.0F;
  /// Value the log likelihood ratios are clipped to.
  float llr_clamp = 32.0F;
  /// Stop a code block once its parity check passes.
  bool early_termination = true;
  /// Return the ratios rather than the decided bits.
  bool output_soft = false;
  /// Scale by the rate rather than by \c min_sum_scale.
  bool auto_scale = true;
  /// Count the iterations each code block took, without holding up the decode.
  bool deferred_iteration_stats = false;
  /// Number of parity rows that may still fail when stopping early.
  ///
  /// Half precision leaves the parity columns short of convergence even when the information bits
  /// are right, and the checksum still catches a wrong answer.
  unsigned syndrome_tolerance = 0;
  /// Verify with the checksum before stopping early, rather than on confidence alone.
  bool crc_early_termination = true;
  /// Checksum the verification applies.
  ldpc_decoder_crc crc = ldpc_decoder_crc::crc24b;
  /// Confidence a code block must reach before the checksum is computed.
  float confidence_threshold = 0.25F;
  /// Use the box-plus rule rather than min-sum.
  bool use_boxplus = false;
  /// \brief Number of parity nodes to work on, or zero for the whole base graph.
  ///
  /// Limiting the decoder to the rows that received log-likelihood ratios keeps the rows whose
  /// parity columns are all zero from polluting the result.
  unsigned max_nof_parity_nodes = 0;
};

/// \brief Where a decode writes the transport block, when it assembles 1 as it goes.
struct ldpc_decoder_transport_block {
  /// Transport block, in device memory, payload followed by its checksum.
  uint8_t* d_output = nullptr;
  /// Number of bits the transport block holds, its checksum included.
  unsigned nof_bits = 0;
  /// Number of information bits each code block contributes.
  unsigned nof_bits_per_codeblock = 0;
};

/// \brief Describes received ratios that are still scrambled and rate matched.
struct ldpc_decoder_rate_matched_input {
  /// Number of bits a modulation symbol carries.
  unsigned bits_per_symbol = 0;
  /// Number of bits the circular buffer holds.
  unsigned nof_buffer_bits = 0;
  /// Bit the selection started at.
  unsigned k0 = 0;
  /// Rate matched length of each code block, in device memory.
  const int* d_lengths = nullptr;
  /// Offset of each code block in the input, in device memory.
  const unsigned* d_offsets = nullptr;
  /// Scrambling sequence, in device memory, or nullptr when the input is already descrambled.
  const unsigned* d_sequence = nullptr;
  /// Bit of the sequence the first code block starts at.
  unsigned sequence_offset = 0;
};

/// \brief Decodes code blocks with the LDPC code of TS 38.212, Section 5.3.2.
class ldpc_decoder
{
public:
  /// \brief Creates a decoder.
  /// \return A decoder, or a description of the error.
  static cuda_expected<ldpc_decoder> create();

  /// Creates an empty decoder that holds no state.
  ldpc_decoder();

  ~ldpc_decoder();

  ldpc_decoder(const ldpc_decoder&)            = delete;
  ldpc_decoder& operator=(const ldpc_decoder&) = delete;

  ldpc_decoder(ldpc_decoder&& other) noexcept;
  ldpc_decoder& operator=(ldpc_decoder&& other) noexcept;

  /// Returns true when the decoder holds state.
  bool is_valid() const;

  /// \brief Sets the shape of the code and what the decoder does with it.
  ///
  /// \param[in] base_graph      Base graph the code blocks were encoded with.
  /// \param[in] lifting_size    Lifting size.
  /// \param[in] nof_info_bits   Number of information bits a code block carries.
  /// \param[in] nof_filler_bits Number of filler bits a code block carries.
  /// \param[in] params          What the decoder does beyond the shape.
  /// \return A successful result, or a description of the error.
  cuda_result configure(ldpc_base_graph_type       base_graph,
                        ldpc::lifting_size_t       lifting_size,
                        unsigned                   nof_info_bits,
                        unsigned                   nof_filler_bits,
                        const ldpc_decoder_params& params);

  /// \brief Reserves the scratch a batch of the given size needs.
  ///
  /// Reserving before a graph capture keeps the allocation out of the capture.
  ///
  /// \param[in] max_nof_codeblocks Largest batch the caller will ask for.
  /// \return A successful result, or a description of the error.
  cuda_result reserve(unsigned max_nof_codeblocks);

  /// \brief Decodes a batch of code blocks from single precision ratios.
  ///
  /// \param[out] d_output       Decoded bits, in device memory, packed.
  /// \param[in]  d_llrs         Log likelihood ratios, in device memory, 1 per bit.
  /// \param[in]  nof_codeblocks Number of code blocks in the batch.
  /// \param[in]  stream         Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result decode(uint32_t* d_output, const float* d_llrs, unsigned nof_codeblocks, const cuda_stream& stream);

  /// \brief Decodes a batch of code blocks from half precision ratios.
  ///
  /// \param[out] d_output        Decoded bits, in device memory, packed.
  /// \param[in]  d_llrs          Log likelihood ratios, in device memory, 1 per bit.
  /// \param[in]  nof_codeblocks  Number of code blocks in the batch.
  /// \param[in]  stream          Stream the work is queued on.
  /// \param[in]  workspace_index Scratch buffer to use, or -1 to let the decoder choose.
  /// \return A successful result, or a description of the error.
  cuda_result decode(uint32_t*          d_output,
                     const __half*      d_llrs,
                     unsigned           nof_codeblocks,
                     const cuda_stream& stream,
                     int                workspace_index = -1);

  /// \brief Decodes a batch whose ratios are still scrambled and rate matched.
  ///
  /// The kernel undoes both on the way in, which spares the buffer a separate descrambling and
  /// dematching pass would write. Giving \c transport_block assembles the transport block in the
  /// same launch.
  ///
  /// \param[out] d_output         Decoded bits, in device memory, packed.
  /// \param[in]  d_llrs           Received ratios, in device memory, half precision.
  /// \param[in]  nof_codeblocks   Number of code blocks in the batch.
  /// \param[in]  input            How the ratios were rate matched and scrambled.
  /// \param[in]  stream           Stream the work is queued on.
  /// \param[in]  workspace_index  Scratch buffer to use, or -1 to let the decoder choose.
  /// \param[in]  transport_block  Where to assemble the transport block, or nullptr for none.
  /// \param[out] assembled        Set to true when the transport block was assembled, or nullptr.
  /// \return A successful result, or a description of the error.
  cuda_result decode(uint32_t*                              d_output,
                     const __half*                          d_llrs,
                     unsigned                               nof_codeblocks,
                     const ldpc_decoder_rate_matched_input& input,
                     const cuda_stream&                     stream,
                     int                                    workspace_index = -1,
                     const ldpc_decoder_transport_block*    transport_block = nullptr,
                     bool*                                  assembled       = nullptr);

  /// \brief Copies the iteration counts to the host, when they were deferred.
  ///
  /// \param[in] stream Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result flush_iteration_stats(const cuda_stream& stream);

  /// \brief Returns the mean number of iterations the last decode took, over its code blocks.
  ///
  /// \remark When the counts were deferred, this waits for the copy queued by
  /// \ref flush_iteration_stats to finish.
  float get_average_nof_iterations();

  /// Returns true when the last decode left checksum results to read.
  bool has_crc_results() const;

  /// \brief Copies the checksum results of the last decode to the host.
  ///
  /// \param[in] nof_codeblocks Number of code blocks the last decode carried.
  /// \param[in] stream         Stream the copy is queued on.
  /// \return The results, one entry per code block and non-zero where the checksum matched, or
  /// nullptr when the last decode left none. The pointer is borrowed and stays valid until the
  /// next decode.
  ///
  /// \remark This waits for the copy to finish.
  const int* read_crc_results(unsigned nof_codeblocks, const cuda_stream& stream);

private:
  struct impl;

  explicit ldpc_decoder(std::unique_ptr<impl> state_);

  std::unique_ptr<impl> state;
};

} // namespace cuda
} // namespace ocudu

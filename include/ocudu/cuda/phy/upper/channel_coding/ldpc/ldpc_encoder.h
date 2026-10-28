// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief LDPC encoding on the GPU, as per TS 38.212, Section 5.3.2.

#pragma once

#include "ocudu/cuda/adt/cuda_error.h"
#include "ocudu/cuda/adt/cuda_stream.h"
#include "ocudu/phy/upper/channel_coding/ldpc/ldpc.h"
#include "ocudu/ran/sch/ldpc_base_graph.h"
#include <cstdint>
#include <cuda_runtime.h>
#include <memory>

namespace ocudu {
namespace cuda {

/// \brief Encodes code blocks with the LDPC code of TS 38.212, Section 5.3.2.
///
/// An encoder is configured for a base graph and a lifting size, and then encodes any number of
/// code blocks of that shape. Reconfiguring to the shape already in place costs nothing, so an
/// encoder kept across slots pays for the graph tables once.
///
/// The entry points take and return device memory and queue their work on a stream. They do not
/// synchronise, so a caller reads the output only after the stream has run.
class ldpc_encoder
{
public:
  /// \brief Creates an encoder.
  ///
  /// Device memory for the largest graph is reserved here, so configuring later never allocates.
  ///
  /// \return An encoder, or a description of the error.
  static cuda_expected<ldpc_encoder> create();

  /// Creates an empty encoder that holds no state.
  ldpc_encoder();

  ~ldpc_encoder();

  ldpc_encoder(const ldpc_encoder&)            = delete;
  ldpc_encoder& operator=(const ldpc_encoder&) = delete;

  ldpc_encoder(ldpc_encoder&& other) noexcept;
  ldpc_encoder& operator=(ldpc_encoder&& other) noexcept;

  /// Returns true when the encoder holds state, false when it was default constructed or moved
  /// from.
  bool is_valid() const;

  /// \brief Sets the base graph and the lifting size the next codewords are encoded with.
  ///
  /// Calling this with the shape already in place returns without touching the device.
  ///
  /// \param[in] base_graph   Base graph, as selected by get_ldpc_base_graph().
  /// \param[in] lifting_size Lifting size.
  /// \return A successful result, or a description of the error.
  cuda_result configure(ldpc_base_graph_type base_graph, ldpc::lifting_size_t lifting_size);

  /// Number of bits a code block carries into the encoder, as configured.
  unsigned get_input_bits() const;

  /// Number of bits a codeword carries out of the encoder, as configured.
  unsigned get_output_bits() const;

  /// Number of 32-bit words a code block occupies at the input, as configured.
  unsigned get_input_words() const;

  /// Number of 32-bit words a codeword occupies at the output, as configured.
  unsigned get_output_words() const;

  /// \brief Encodes a batch of code blocks.
  ///
  /// Bits are packed most significant bit first inside a byte, and the bytes least significant
  /// first inside a 32-bit word, which is the layout a bit_buffer has when its bytes are read as
  /// words. The code blocks follow one another at the stride the configured shape gives, and the
  /// codewords keep the 2 lifted columns that TS 38.212 punctures, so a reader of the codeword
  /// skips twice the lifting size. The encoder picks the kernel that suits the lifting size and
  /// the batch size.
  ///
  /// \param[out] d_output       Codewords, in device memory.
  /// \param[in]  d_input        Code blocks to encode, in device memory.
  /// \param[in]  nof_codewords  Number of code blocks in the batch.
  /// \param[in]  stream         Stream the work is queued on.
  /// \return A successful result, or a description of the error.
  cuda_result encode(uint32_t* d_output, const uint32_t* d_input, unsigned nof_codewords, const cuda_stream& stream);

private:
  struct impl;

  explicit ldpc_encoder(std::unique_ptr<impl> state_);

  std::unique_ptr<impl> state;
};

} // namespace cuda
} // namespace ocudu

// SPDX-FileCopyrightText: Copyright (C) 2021-2026 Software Radio Systems Limited
// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_pool.h"
#include "ocudu/gateways/baseband/buffer/baseband_gateway_buffer_reader.h"
#include "ocudu/support/ocudu_assert.h"

namespace ocudu {

/// Baseband buffer reader view that keeps the viewed pooled buffer shared ownership.
class baseband_gateway_buffer_ptr_view : public baseband_gateway_buffer_reader
{
public:
  /// Default constructor - creates an empty view.
  baseband_gateway_buffer_ptr_view() = default;

  /// \brief Constructs a view of a pooled baseband buffer for a given offset and a size.
  /// \param[in] buffer_      Buffer that contains the samples.
  /// \param[in] offset_      Offset from the beginning of the buffer.
  /// \param[in] nof_samples_ Number of samples.
  baseband_gateway_buffer_ptr_view(baseband_gateway_buffer_ptr buffer_, unsigned offset_, unsigned nof_samples_) :
    buffer(std::move(buffer_)), offset(offset_), nof_samples(nof_samples_)
  {
    ocudu_assert(buffer, "Invalid baseband buffer.");
    ocudu_assert(buffer->get_nof_samples() >= offset + nof_samples,
                 "The offset (i.e., {}) plus the number of samples (i.e., {}) exceed the buffer size (i.e., {}).",
                 offset,
                 nof_samples,
                 buffer->get_nof_samples());
  }

  /// Default move constructor.
  baseband_gateway_buffer_ptr_view(baseband_gateway_buffer_ptr_view&&) noexcept = default;

  /// Default move assign operator.
  baseband_gateway_buffer_ptr_view& operator=(baseband_gateway_buffer_ptr_view&&) noexcept = default;

  /// Creates a copy of the buffer view, increasing the reference count.
  baseband_gateway_buffer_ptr_view clone() const
  {
    return baseband_gateway_buffer_ptr_view(buffer.clone(), offset, nof_samples);
  }

  /// \brief Creates a slice of the buffer view increasing the reference count of the original buffer.
  ///
  /// \param[in] offset_ Offset in samples from this buffer view.
  /// \param[in] nof_samples_ Number of samples of the slice.
  /// \return A view of the buffer at the given offset and number of samples.
  baseband_gateway_buffer_ptr_view slice(unsigned offset_, unsigned nof_samples_) const
  {
    return baseband_gateway_buffer_ptr_view(buffer.clone(), offset + offset_, nof_samples_);
  }

  /// Releases the buffer and empties the view.
  void release()
  {
    buffer.reset();
    offset      = 0;
    nof_samples = 0;
  }

  /// Determines whether the view holds a buffer.
  explicit operator bool() const { return static_cast<bool>(buffer); }

  // See interface for documentation.
  unsigned get_nof_channels() const override { return buffer->get_nof_channels(); }

  // See interface for documentation.
  unsigned get_nof_samples() const override { return nof_samples; }

  // See interface for documentation.
  span<const ci16_t> get_channel_buffer(unsigned i_channel) const override
  {
    return buffer->get_reader().get_channel_buffer(i_channel).subspan(offset, nof_samples);
  }

private:
  baseband_gateway_buffer_ptr buffer;
  unsigned                    offset      = 0;
  unsigned                    nof_samples = 0;
};

} // namespace ocudu

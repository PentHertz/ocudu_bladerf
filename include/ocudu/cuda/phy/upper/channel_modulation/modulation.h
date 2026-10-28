// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief Modulation mapping and soft demodulation on the GPU, as per TS 38.211, Section 5.1.
///
/// The entry points take and return device memory and queue their work on a stream. None of them
/// synchronises, so a caller reads the output only after the stream has run.

#pragma once

#include "ocudu/cuda/adt/cuda_error.h"
#include "ocudu/cuda/adt/cuda_stream.h"
#include "ocudu/ran/sch/modulation_scheme.h"
#include <cstdint>
#include <cuComplex.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

namespace ocudu {
namespace cuda {

/// \brief Maps packed bits to complex symbols.
///
/// The bits are packed least significant bit first within each word: bit \c n of the sequence sits
/// at position \c n modulo 32 of word \c n divided by 32.
///
/// \param[out] d_symbols Mapped symbols, in device memory. Holds \c nof_bits divided by the number
///                       of bits per symbol entries.
/// \param[in]  d_bits    Bits to map, in device memory.
/// \param[in]  nof_bits  Number of bits to map.
/// \param[in]  scheme    Modulation scheme. BPSK, QPSK, 16QAM, 64QAM and 256QAM are implemented.
/// \param[in]  stream    Stream the work is queued on.
/// \return A successful result, or a description of the error.
cuda_result modulate(cuFloatComplex*    d_symbols,
                     const uint32_t*    d_bits,
                     unsigned           nof_bits,
                     modulation_scheme  scheme,
                     const cuda_stream& stream);

/// \brief Maps packed bits to 8-bit integer complex symbols.
///
/// The bits are packed as in modulate(). The constellation is not normalised, so the points are the
/// odd integers that TS 38.211 scales, and a symbol occupies 2 bytes, the real part first.
///
/// \param[out] d_symbols Mapped symbols, in device memory. Holds 2 bytes per symbol.
/// \param[in]  d_bits    Bits to map, in device memory.
/// \param[in]  nof_bits  Number of bits to map.
/// \param[in]  scheme    Modulation scheme. QPSK, 16QAM, 64QAM and 256QAM are implemented.
/// \param[in]  stream    Stream the work is queued on.
/// \return A successful result, or a description of the error.
cuda_result modulate_int8(int8_t*            d_symbols,
                          const uint32_t*    d_bits,
                          unsigned           nof_bits,
                          modulation_scheme  scheme,
                          const cuda_stream& stream);

/// \brief Demodulates symbols into log likelihood ratios, one noise variance for every symbol.
///
/// \param[out] d_llrs      Log likelihood ratios, in device memory. Holds 1 entry per bit.
/// \param[in]  d_symbols   Received symbols, in device memory.
/// \param[in]  nof_symbols Number of symbols to demodulate.
/// \param[in]  scheme      Modulation scheme the symbols were mapped with.
/// \param[in]  noise_var   Noise variance the whole sequence is demodulated against.
/// \param[in]  stream      Stream the work is queued on.
/// \return A successful result, or a description of the error.
cuda_result soft_demodulate_constant_noise_var(float*                d_llrs,
                                               const cuFloatComplex* d_symbols,
                                               unsigned              nof_symbols,
                                               modulation_scheme     scheme,
                                               float                 noise_var,
                                               const cuda_stream&    stream);

/// \brief Demodulates symbols into log likelihood ratios, with a noise variance per symbol.
///
/// Equalisation leaves a different noise variance on each subcarrier, so the receive chain reads
/// the variances from a buffer rather than passing one value.
///
/// \param[out] d_llrs       Log likelihood ratios, in device memory. Holds 1 entry per bit.
/// \param[in]  d_symbols    Received symbols, in device memory.
/// \param[in]  d_noise_vars Noise variances, in device memory. Holds 1 entry per symbol.
/// \param[in]  nof_symbols  Number of symbols to demodulate.
/// \param[in]  scheme       Modulation scheme the symbols were mapped with.
/// \param[in]  stream       Stream the work is queued on.
/// \return A successful result, or a description of the error.
cuda_result soft_demodulate(float*                d_llrs,
                            const cuFloatComplex* d_symbols,
                            const float*          d_noise_vars,
                            unsigned              nof_symbols,
                            modulation_scheme     scheme,
                            const cuda_stream&    stream);

/// \brief Demodulates symbols into half precision log likelihood ratios.
///
/// The decoder that consumes the ratios reads half precision, so writing them directly saves a
/// conversion pass over the whole buffer.
///
/// \param[out] d_llrs       Log likelihood ratios, in device memory. Holds 1 entry per bit.
/// \param[in]  d_symbols    Received symbols, in device memory.
/// \param[in]  d_noise_vars Noise variances, in device memory. Holds 1 entry per symbol.
/// \param[in]  nof_symbols  Number of symbols to demodulate.
/// \param[in]  scheme       Modulation scheme the symbols were mapped with. QPSK, 16QAM, 64QAM and
///                          256QAM are implemented.
/// \param[in]  stream       Stream the work is queued on.
/// \return A successful result, or a description of the error.
cuda_result soft_demodulate(__half*               d_llrs,
                            const cuFloatComplex* d_symbols,
                            const float*          d_noise_vars,
                            unsigned              nof_symbols,
                            modulation_scheme     scheme,
                            const cuda_stream&    stream);

} // namespace cuda
} // namespace ocudu

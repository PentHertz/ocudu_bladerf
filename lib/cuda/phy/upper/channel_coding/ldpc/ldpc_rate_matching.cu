// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

/// \file
/// \brief LDPC rate matching and rate dematching on the GPU, as per TS 38.212, Section 5.4.2.
///
/// Rate matching selects the bits of a codeword that go on the air, starting at the position the
/// redundancy version fixes and wrapping around the circular buffer, and interleaves them across
/// the bits of a modulation symbol. Dematching reverses both on the log likelihood ratios, and
/// restores the ratios of the bits that were punctured or filled, which the air never carried.

#include "ocudu/cuda/adt/cuda_error.h"
#include "ocudu/cuda/phy/upper/channel_coding/ldpc/ldpc_rate_matching.h"
#include <algorithm>
#include <cuda_fp16.h>
#include <new>

namespace ocudu {
namespace cuda {

// Device Constants for Bit Interleaving
// ============================================================================

// ============================================================================
// Device Helper Functions
// ============================================================================

/// \brief Saturating atomic add for FP16 using CAS loop
///
/// Clamps accumulated result to [-127.0f, 127.0f] for decoder stability.
///
/// \param address Pointer to __half to update
/// \param val Value to add
__device__ __forceinline__ void atomicAddSaturatingHalf(__half* address, __half val)
{
#if __CUDA_ARCH__ >= 700
  // Use native fp16 atomicCAS on Volta+ (requires SM 7.0+)
  unsigned short* address_as_ushort = (unsigned short*)address;
  unsigned short  old_val, new_val;

  do {
    old_val         = *address_as_ushort;
    float old_float = __half2float(*(__half*)&old_val);
    float sum       = old_float + __half2float(val);
    // Clamp to safe FP16 range — large enough to preserve LLR magnitudes
    // at high SNR, while preventing FP16 overflow during accumulation.
    sum             = fmaxf(-2000.0f, fminf(2000.0f, sum));
    __half sum_half = __float2half(sum);
    new_val         = *(unsigned short*)&sum_half;
  } while (atomicCAS(address_as_ushort, old_val, new_val) != old_val);
#else
  // Fallback for older architectures: just store (no accumulation)
  *address = val;
#endif
}

/// \brief Calculate de-rate match output index with filler bit handling
///
/// Per 3GPP TS 38.212 Section 5.4.2.1, the circular buffer has a "hole"
/// for filler bits that must be skipped during de-rate matching.
///
/// Layout: [Systematic (Kd)] [HOLE/Filler (F)] [Parity (Ncb - K)]
/// where K = Kd + F
///
/// The key insight is that the EFFECTIVE circular buffer size is Ncb - F,
/// since F filler bits are skipped on each wrap. The algorithm:
/// 1. Compute position in effective buffer: eff_pos = (k0_eff + inIdx) % (Ncb - F)
/// 2. Map eff_pos to actual CB position by adding F if eff_pos >= Kd
///
/// \param inIdx Input index (0 to E-1)
/// \param Kd Actual info bits (excluding filler) in CB coordinates
/// \param F Filler bits
/// \param k0 Starting position in circular buffer
/// \param Ncb Circular buffer size
/// \return Output index in the LDPC codeword buffer (CB coordinates)
__device__ __forceinline__ int derate_match_calc_index(int inIdx, int Kd, int F, int k0, int Ncb)
{
  // Effective buffer size (excluding filler bits)
  int Ncb_eff = Ncb - F;

  // Convert k0 to effective coordinates
  // If k0 >= Kd + F, we're in parity region - subtract F to get effective position
  // If k0 < Kd, k0_eff = k0 (before filler hole)
  // If Kd <= k0 < Kd + F, starting in filler (shouldn't happen per spec)
  int k0_eff;
  if (k0 < Kd) {
    k0_eff = k0;
  } else if (k0 >= Kd + F) {
    // Convert parity position to effective coordinates
    k0_eff = k0 - F;
  } else {
    // Starting in filler hole - shouldn't happen, but handle it
    // Jump to end of filler
    k0_eff = Kd;
  }

  // Compute position in effective (filler-excluded) circular buffer
  int eff_pos = (k0_eff + inIdx) % Ncb_eff;

  // Map effective position back to actual CB position
  // If eff_pos >= Kd, we're past where filler would be, so add F
  int outIdx;
  if (eff_pos >= Kd) {
    outIdx = eff_pos + F;
  } else {
    outIdx = eff_pos;
  }

  return outIdx;
}

// ============================================================================
// CUDA Kernels
// ============================================================================

/// \brief Calculate rate match input index with filler bit handling (TX direction)
///
/// Per 3GPP TS 38.212, filler bits are "known" values that should be skipped
/// during rate matching. This function maps output index to input position,
/// skipping the filler bit "hole".
///
/// \param outIdx Output index (0 to E-1)
/// \param Kd Actual info bits (excluding filler)
/// \param F Filler bits
/// \param k0 Starting position in circular buffer
/// \param Ncb Circular buffer size
/// \return Input position in the LDPC codeword buffer
__device__ __forceinline__ int rate_match_calc_index(int outIdx, int Kd, int F, int k0, int Ncb)
{
  // Same logic as derate_match_calc_index - symmetric operation
  return derate_match_calc_index(outIdx, Kd, F, k0, Ncb);
}

/// \brief Rate matching kernel (TX direction)
///
/// Per 3GPP TS 38.212 Section 5.4.2.1:
/// e[k] = d[(k0 + k) mod N_cb] for k = 0, 1, ..., E-1
///
/// Selects E bits from the circular buffer (LDPC codeword) starting at k0.
///
/// IMPORTANT: The LDPC encoder uses COLUMN-MAJOR layout:
/// - Each column (VN) has Z bits stored in Z_words = (Z+31)/32 words
/// - Total output = N * Z_words words per codeword
/// - Bit position i is at: column = i/Z, bit_in_col = i%Z,
///   word = col * Z_words + bit_in_col/32, bit = bit_in_col%32
__global__ void rate_match_tx_kernel(const uint32_t* __restrict__ d_encoded,
                                     uint32_t* __restrict__ d_output,
                                     int N_cb,            // Circular buffer size (after puncturing: 50*Z for BG2)
                                     int N_full,          // Full codeword size (52*Z for BG2)
                                     int k0,              // Starting position in circular buffer
                                     int E,               // Number of output bits
                                     int Z,               // Lifting size (needed for column-major addressing)
                                     int N_cols,          // Number of columns (52 for BG2, 68 for BG1)
                                     int puncture_offset, // Offset for punctured bits (2*Z)
                                     int Kd,              // Actual info bits in CB coordinates (for filler skipping)
                                     int F,               // Filler bits (for filler skipping)
                                     int num_cbs,
                                     int output_word_stride // Output stride per CB in words (0 = use E_words)
)
{
  int cb_idx = blockIdx.y;
  if (cb_idx >= num_cbs)
    return;

  int out_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (out_idx >= E)
    return;

  // Use filler-skipping algorithm per 3GPP TS 38.212 Section 5.4.2.1
  // Filler bits at positions [Kd, Kd+F) in the circular buffer are NOT transmitted
  // This must match the RX de-rate matching algorithm for consistency
  int cb_pos = rate_match_calc_index(out_idx, Kd, F, k0, N_cb);

  // Add puncture offset to get position in full codeword
  int enc_pos = cb_pos + puncture_offset;

  // FLAT addressing for LDPC encoder output (uses MSB-first bit ordering within each byte)
  // Bit index enc_pos is at word enc_pos/32
  // Within the word, use MSB-first per byte: byte_in_word*8 + (7 - bit_in_byte)
  int in_word      = enc_pos / 32;
  int bit_in_word  = enc_pos % 32;
  int byte_in_word = bit_in_word / 8;
  // MSB-first within byte
  int bit_in_byte = 7 - (bit_in_word % 8);
  int in_bit      = byte_in_word * 8 + bit_in_byte;

  // Flat word count for input
  int      input_stride = (N_full + 31) / 32;
  uint32_t encoded_word = d_encoded[cb_idx * input_stride + in_word];
  uint32_t bit          = (encoded_word >> in_bit) & 1u;

  // Write to output (flat sequential format, MSB-first within each byte)
  int out_word        = out_idx / 32;
  int out_bit_in_word = out_idx % 32;
  int out_byte        = out_bit_in_word / 8;
  // MSB-first within byte
  int out_bit_in_byte = 7 - (out_bit_in_word % 8);
  int out_bit         = out_byte * 8 + out_bit_in_byte;

  // Use configurable output stride (for uniform per-CB layout with variable E)
  int E_words = (output_word_stride > 0) ? output_word_stride : (E + 31) / 32;

  atomicOr(&d_output[cb_idx * E_words + out_word], bit << out_bit);
}

/// \brief De-rate matching kernel (RX direction)
///
/// Per 3GPP TS 38.212 Section 5.4.2.1:
/// e[k] = d[(k0 + k) mod N_cb] for k = 0, 1, ..., E-1
///
/// This kernel does the reverse: accumulates received LLRs into the circular buffer.
/// Uses the filler-skipping algorithm from cuPHY to correctly handle the filler
/// "hole" in the systematic region - this is critical for repetition (E > N_cb).
///
/// Filler bit LLRs are set separately after this kernel completes.
///
/// IMPORTANT: The LDPC decoder expects COLUMN-MAJOR layout:
/// - Each column (VN) has Z LLRs stored contiguously
/// - Total buffer = N_cols * Z floats per codeword
/// - LLR position i is at: column = i/Z, offset_in_col = i%Z,
///   buffer_index = col * Z + offset_in_col
__global__ void rate_match_rx_kernel(const float* __restrict__ d_received_llrs,
                                     float* __restrict__ d_output_llrs,
                                     int N_cb,            // Circular buffer size (after puncturing: 50*Z for BG2)
                                     int N_full,          // Full LLR buffer size (52*Z for BG2)
                                     int k0,              // Starting position in circular buffer
                                     int E,               // Number of received LLRs
                                     int Z,               // Lifting size (needed for column-major addressing)
                                     int N_cols,          // Number of columns (52 for BG2, 68 for BG1)
                                     int puncture_offset, // Offset for punctured bits (2*Z)
                                     int Kd,              // Actual info bits (excluding filler) - for filler skipping
                                     int F,               // Filler bits - for filler skipping
                                     int num_cbs)
{
  int cb_idx = blockIdx.y;
  if (cb_idx >= num_cbs)
    return;

  int recv_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (recv_idx >= E)
    return;

  // Use filler-skipping algorithm to calculate output position
  // This correctly handles the filler "hole" at [Kd, Kd+F) in the circular buffer
  // Critical for repetition (E > N_cb) where we wrap around multiple times
  int cb_pos = derate_match_calc_index(recv_idx, Kd, F, k0, N_cb);

  // Add puncture offset to get position in full LLR buffer
  int dec_pos = cb_pos + puncture_offset;

  // Column-major addressing for LDPC decoder input
  // The decoder expects LLRs in column-major format: all Z LLRs for column 0,
  // then all Z LLRs for column 1, etc.
  int column        = dec_pos / Z;
  int offset_in_col = dec_pos % Z;
  int llr_idx       = column * Z + offset_in_col;

  // Read received LLR
  float llr = d_received_llrs[cb_idx * E + recv_idx];

  // Accumulate into output (for combining repetitions when E > N_cb).
  // No saturation for FP32 path - the decoder's own llr_clamp handles value range.
  int output_stride = N_cols * Z;
  atomicAdd(&d_output_llrs[cb_idx * output_stride + llr_idx], llr);
}

/// \brief Set filler bit LLRs to +infinity (known to be 0)
///
/// Filler bits are set to 0 before encoding and are not transmitted.
/// For the decoder, these LLRs should indicate high confidence that bit=0.
///
/// Filler bits are at the END of the systematic region in the codeword:
/// - Encoder input bits 0 to Kd-1: actual info (TB + CRC)
/// - Encoder input bits Kd to Kd+F-1: filler bits (always 0)
/// - These map directly to codeword positions 0-Kd-1 and Kd to Kd+F-1
///
/// NOTE: puncture_offset is NOT added here - that's for the circular buffer.
/// The decoder expects LLRs in flat codeword order where systematic bits
/// (including filler) are at positions 0 to Kb*Z-1.
__global__ void
set_filler_llrs_kernel(float* __restrict__ d_llrs,
                       int   Kd,              // Number of actual info bits (filler starts at position Kd in CB coords)
                       int   F,               // Number of filler bits
                       int   Z,               // Lifting size (unused but kept for consistency)
                       int   N_cols,          // Number of columns (52 for BG2, 68 for BG1)
                       int   puncture_offset, // Offset for punctured bits (2*Z), needed to convert to encoder coords
                       float filler_llr,      // LLR value for filler bits (positive = bit=0)
                       int   num_cbs)
{
  int cb_idx = blockIdx.y;
  if (cb_idx >= num_cbs)
    return;

  int filler_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (filler_idx >= F)
    return;

  // Filler bit position in encoder coordinates.
  // Kd is in CB coords (position where filler starts in circular buffer).
  // Add puncture_offset to convert to encoder coordinates.
  // Filler bits are at encoder positions [puncture_offset + Kd, puncture_offset + K).
  int pos = puncture_offset + Kd + filler_idx;

  // Flat LLR addressing
  int output_stride                    = N_cols * Z;
  d_llrs[cb_idx * output_stride + pos] = filler_llr;
}

/// \brief Set punctured column LLRs for min-sum stability
///
/// Per srsRAN and cuPHY reference implementations, punctured bits (first 2*Z)
/// should be initialized to LLR=0.0 representing erasure. However, in the
/// layered min-sum decoder, when V2C = 0 for punctured columns, it becomes
/// the minimum and zeros out C2V messages for all other columns in that row.
///
/// To enable proper information propagation, we use a small non-zero value
/// that prevents punctured columns from dominating the min-sum while still
/// being effectively "erasure-like". The value 0.1 is chosen to be:
/// - Much smaller than typical channel LLRs (~127)
/// - Large enough to not always be the minimum
/// - Positive (representing slight preference for bit=0)
__global__ void set_punctured_llrs_kernel(float* __restrict__ d_llrs,
                                          int puncture_count, // Number of positions to check
                                          int stride,         // Stride per codeblock
                                          int num_cbs)
{
  int cb_idx = blockIdx.y;
  if (cb_idx >= num_cbs)
    return;

  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= puncture_count)
    return;

  // Punctured positions (first 2*Z bits) have no channel information
  // Per 3GPP TS 38.212, punctured bits are always the first 2*Z bits (columns 0,1)
  // These are systematic bits that are not transmitted but must be decoded.
  // Set to 0 (erasure) - the decoder excludes punctured columns from min computation
  int pos = cb_idx * stride + idx;
  // Erasure - decoder learns from parity checks
  d_llrs[pos] = 0.0f;
}

/// \brief Bit interleaving for rate matching (TX)
///
/// Per 3GPP TS 38.212 Section 5.4.2.2:
///   e(j*Qm + i) = f(i*K + j)  for i=0..Qm-1, j=0..K-1
/// where K = E/Qm (number of symbols).
///
/// This is a row-column transpose where input bits are viewed as a matrix
/// with Qm rows and K columns, and output reads column-by-column.
__global__ void bit_interleave_kernel(const uint32_t* __restrict__ d_input,
                                      uint32_t* __restrict__ d_output,
                                      int E,
                                      int Q_m, // Modulation order (bits per symbol)
                                      int num_cbs)
{
  int cb_idx = blockIdx.y;
  if (cb_idx >= num_cbs)
    return;

  int out_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (out_idx >= E)
    return;

  // Per 3GPP TS 38.212 Section 5.4.2.2:
  // Output position out_idx = j*Qm + i maps to input position i*K + j
  // where j = symbol index, i = bit within symbol, K = E/Qm
  // Number of symbols
  int K = E / Q_m;
  // Symbol index
  int j = out_idx / Q_m;
  // Bit index within symbol
  int i = out_idx % Q_m;

  // Calculate input position: i*K + j
  int in_idx = i * K + j;

  // Read the input bit. The packing is the one the rate matching kernel writes: most significant
  // bit first inside a byte, bytes least significant first inside the word.
  int in_word = in_idx / 32;
  int in_bit  = ((in_idx % 32) / 8) * 8 + (7 - (in_idx % 8));
  int E_words = (E + 31) / 32;

  uint32_t bit = (d_input[cb_idx * E_words + in_word] >> in_bit) & 1u;

  // Write to output in the same packing.
  int out_word = out_idx / 32;
  int out_bit  = ((out_idx % 32) / 8) * 8 + (7 - (out_idx % 8));
  atomicOr(&d_output[cb_idx * E_words + out_word], bit << out_bit);
}

/// \brief Bit de-interleaving for de-rate matching (RX)
///
/// Per 3GPP TS 38.212 Section 5.4.2.2:
///   TX: e(j*Qm + i) = f(i*K + j)
/// The inverse is:
///   RX: output at position i*K + j reads from input position j*Qm + i
///
/// For output position out_idx = i*K + j:
///   i = out_idx / K, j = out_idx % K
///   in_idx = j*Qm + i = (out_idx % K)*Qm + (out_idx / K)
__global__ void bit_deinterleave_llr_kernel(const float* __restrict__ d_input_llrs,
                                            float* __restrict__ d_output_llrs,
                                            int E,
                                            int Q_m,
                                            int num_cbs)
{
  int cb_idx = blockIdx.y;
  if (cb_idx >= num_cbs)
    return;

  int out_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (out_idx >= E)
    return;

  // Per 3GPP TS 38.212 Section 5.4.2.2 (inverse of interleaving):
  // Output position out_idx = i*K + j maps to input position j*Qm + i
  // where i = row index (0..Qm-1), j = column index (0..K-1), K = E/Qm
  // Number of symbols
  int K = E / Q_m;
  // Row index in de-interleaved matrix
  int i = out_idx / K;
  // Column index in de-interleaved matrix
  int j = out_idx % K;

  // Calculate input position: j*Qm + i
  int in_idx = j * Q_m + i;

  // Copy LLR
  d_output_llrs[cb_idx * E + out_idx] = d_input_llrs[cb_idx * E + in_idx];
}

// ============================================================================

// Half-precision (fp16) Rate Dematching Kernels
// ============================================================================

/// \brief De-rate matching kernel for fp16 LLRs
///
/// Note: Uses atomicAdd for __half which requires compute capability 7.0+
__global__ void rate_match_rx_half_kernel(const __half* __restrict__ d_received_llrs,
                                          __half* __restrict__ d_output_llrs,
                                          int  N_cb,
                                          int  N_full,
                                          int  k0,
                                          int  E,
                                          int  Z,
                                          int  N_cols,
                                          int  puncture_offset,
                                          int  Kd,
                                          int  F,
                                          int  num_cbs,
                                          bool no_repetition)
{
  int cb_idx = blockIdx.y;
  if (cb_idx >= num_cbs)
    return;

  int recv_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (recv_idx >= E)
    return;

  // Use filler-skipping algorithm
  int cb_pos  = derate_match_calc_index(recv_idx, Kd, F, k0, N_cb);
  int dec_pos = cb_pos + puncture_offset;

  int column        = dec_pos / Z;
  int offset_in_col = dec_pos % Z;
  int llr_idx       = column * Z + offset_in_col;

  __half  llr           = d_received_llrs[cb_idx * E + recv_idx];
  int     output_stride = N_cols * Z;
  __half* out           = &d_output_llrs[cb_idx * output_stride + llr_idx];

  // Use saturating add to clamp to [-127, 127] - prevents decoder instability
  // when E > N_cb causes accumulated values to exceed typical LLR range
  if (no_repetition) {
    *out = llr;
  } else {
    atomicAddSaturatingHalf(out, llr);
  }
}

// ============================================================================
// Fused Deinterleave + Rate Dematch (FP16) — eliminates intermediate buffer
// ============================================================================

/// \brief Fused deinterleave + de-rate matching kernel for FP16 LLRs
///
/// For each recv_idx (0..E-1), applies the deinterleave permutation on-the-fly
/// to find the source position in the interleaved input, then maps recv_idx to
/// the output codeword position via derate_match_calc_index.
__global__ void
rate_dematch_deinterleave_half_kernel(const __half* __restrict__ d_received_llrs, // Interleaved input [num_cbs * E]
                                      __half* __restrict__ d_output_llrs, // Rate-dematched output [num_cbs * N_full]
                                      int  N_cb,
                                      int  N_full,
                                      int  k0,
                                      int  E,
                                      int  Z,
                                      int  N_cols,
                                      int  puncture_offset,
                                      int  Kd,
                                      int  F,
                                      int  Q_m,
                                      int  K, // = E / Q_m
                                      int  num_cbs,
                                      bool no_repetition)
{
  int cb_idx = blockIdx.y;
  if (cb_idx >= num_cbs)
    return;

  int recv_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (recv_idx >= E)
    return;

  // Step 1: Deinterleave — compute source position in interleaved buffer
  // Deinterleave: for deinterleaved position recv_idx,
  //   j = recv_idx / K, i = recv_idx % K
  //   source in interleaved buffer = i * Q_m + j
  int j       = recv_idx / K;
  int i       = recv_idx % K;
  int src_pos = i * Q_m + j;

  // Step 2: Rate dematch — compute output position in codeword buffer
  int cb_pos        = derate_match_calc_index(recv_idx, Kd, F, k0, N_cb);
  int dec_pos       = cb_pos + puncture_offset;
  int column        = dec_pos / Z;
  int offset_in_col = dec_pos % Z;
  int llr_idx       = column * Z + offset_in_col;

  // Read from interleaved input, write to output
  __half  llr           = d_received_llrs[cb_idx * E + src_pos];
  int     output_stride = N_cols * Z;
  __half* out           = &d_output_llrs[cb_idx * output_stride + llr_idx];
  if (no_repetition) {
    *out = llr;
  } else {
    atomicAddSaturatingHalf(out, llr);
  }
}

// ============================================================================
// Geometry
// ============================================================================

/// Shape a code block has in the circular buffer, derived from its description.
struct rate_matching_geometry {
  /// Base graph, 1 or 2.
  int base_graph = 0;
  /// Lifting size.
  int Z = 0;
  /// Number of lifted columns the base graph has.
  int N_cols = 0;
  /// Number of bits the encoder produced, including the punctured columns.
  int N_full = 0;
  /// Number of bits of the circular buffer the selection wraps around, N_cb of TS 38.212, Section 5.4.2.1: what the
  /// punctured columns leave, limited to N_ref with limited-buffer rate matching.
  int N = 0;
  /// Number of bits the punctured columns occupy.
  int puncture_offset = 0;
  /// Number of information bits, filler excluded.
  int Kd = 0;
  /// Number of filler bits.
  int F = 0;
  /// Number of bits a matched code block occupies.
  int E = 0;
  /// Bit the selection starts at, as the redundancy version fixes it.
  int k0 = 0;
};

/// \brief Bit the selection starts at, as per TS 38.212, Table 5.4.2.1-2.
static int starting_position(int base_graph, int Z, unsigned rv, int N_cb)
{
  switch (rv) {
    case 0:
      return 0;
    case 1:
      return (base_graph == 1) ? ((17 * N_cb) / (66 * Z)) * Z : ((13 * N_cb) / (50 * Z)) * Z;
    case 2:
      return (base_graph == 1) ? ((33 * N_cb) / (66 * Z)) * Z : ((25 * N_cb) / (50 * Z)) * Z;
    default:
      return (base_graph == 1) ? ((56 * N_cb) / (66 * Z)) * Z : ((43 * N_cb) / (50 * Z)) * Z;
  }
}

/// \brief Derives the shape of a code block from its description.
///
/// \param[out] geometry Shape the entry points read.
/// \param[in]  cfg      Code block description.
/// \return A successful result, or a description of the error.
static cuda_result derive_geometry(rate_matching_geometry& geometry, const codeblock_metadata& cfg)
{
  int base_graph = (cfg.tb_common.base_graph == ldpc_base_graph_type::BG1) ? 1 : 2;
  int Z          = static_cast<int>(cfg.tb_common.lifting_size);
  if (Z <= 0) {
    return make_unexpected(std::string("The code block description carries an invalid lifting size"));
  }
  if (cfg.cb_specific.rm_length == 0) {
    return make_unexpected(std::string("The code block description carries an empty rate matched length"));
  }

  int Kb       = (base_graph == 1) ? 22 : 10;
  int nof_cols = (base_graph == 1) ? 68 : 52;

  geometry.base_graph = base_graph;
  geometry.Z          = Z;
  geometry.N_cols     = nof_cols;
  geometry.N_full     = nof_cols * Z;
  // The first 2 lifted columns are always punctured, so the circular buffer is what is left.
  geometry.puncture_offset = 2 * Z;
  geometry.N               = (nof_cols - 2) * Z;
  geometry.F               = static_cast<int>(cfg.cb_specific.nof_filler_bits);
  geometry.Kd              = Kb * Z - geometry.F;
  geometry.E               = static_cast<int>(cfg.cb_specific.rm_length);

  // A limited buffer shortens the circular buffer the selection wraps around, and the starting positions with it.
  if (cfg.tb_common.Nref > 0) {
    geometry.N = std::min(static_cast<int>(cfg.tb_common.Nref), geometry.N);
  }
  geometry.k0 = starting_position(base_graph, Z, cfg.tb_common.rv, geometry.N);

  return {};
}

/// Number of threads a rate matching kernel runs per block.
static constexpr unsigned block_size = 256;

/// Number of blocks that covers the given number of items.
static unsigned blocks_for(unsigned nof_items)
{
  return (nof_items + block_size - 1) / block_size;
}

// ============================================================================
// Rate matcher
// ============================================================================

struct ldpc_rate_matcher::impl {
  rate_matching_geometry geometry;
  bool                   configured = false;
};

cuda_expected<ldpc_rate_matcher> ldpc_rate_matcher::create()
{
  std::unique_ptr<impl> created(new (std::nothrow) impl());
  if (created == nullptr) {
    return make_unexpected(std::string("The rate matcher state could not be allocated"));
  }

  return ldpc_rate_matcher(std::move(created));
}

ldpc_rate_matcher::ldpc_rate_matcher() = default;

ldpc_rate_matcher::ldpc_rate_matcher(std::unique_ptr<impl> state_) : state(std::move(state_)) {}

ldpc_rate_matcher::ldpc_rate_matcher(ldpc_rate_matcher&& other) noexcept = default;

ldpc_rate_matcher& ldpc_rate_matcher::operator=(ldpc_rate_matcher&& other) noexcept = default;

ldpc_rate_matcher::~ldpc_rate_matcher() = default;

bool ldpc_rate_matcher::is_valid() const
{
  return state != nullptr;
}

cuda_result ldpc_rate_matcher::configure(const codeblock_metadata& cfg)
{
  cuda_result status = derive_geometry(state->geometry, cfg);
  if (!status.has_value()) {
    return status;
  }
  state->configured = true;

  return {};
}

unsigned ldpc_rate_matcher::get_output_bits() const
{
  return static_cast<unsigned>(state->geometry.E);
}

cuda_result ldpc_rate_matcher::rate_match(uint32_t*          d_matched,
                                          const uint32_t*    d_codewords,
                                          unsigned           nof_codeblocks,
                                          unsigned           word_stride,
                                          const cuda_stream& stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_codewords == nullptr) || (d_matched == nullptr) || (nof_codeblocks == 0)) {
    return make_unexpected(std::string("The rate matching call carries an invalid configuration"));
  }
  if (!state->configured) {
    return make_unexpected(std::string("The rate matcher is not configured"));
  }

  const rate_matching_geometry& g = state->geometry;

  // The information bits are counted from the start of the encoder output, and the kernel wants
  // them counted from the start of the circular buffer.
  int Kd_in_buffer = g.Kd - g.puncture_offset;

  unsigned nof_words = (static_cast<unsigned>(g.E) + 31) / 32;
  unsigned stride    = (word_stride > 0) ? word_stride : nof_words;

  cuda_result status = check_cuda_error(
      static_cast<int>(cudaMemsetAsync(
          d_matched, 0, static_cast<size_t>(stride) * nof_codeblocks * sizeof(uint32_t), native_stream)),
      "clearing the rate matched output");
  if (!status.has_value()) {
    return status;
  }

  rate_match_tx_kernel<<<blocks_for(g.E * nof_codeblocks), block_size, 0, native_stream>>>(
      d_codewords,
      d_matched,
      g.N,
      g.N_full,
      g.k0,
      g.E,
      g.Z,
      g.N_cols,
      g.puncture_offset,
      Kd_in_buffer,
      g.F,
      static_cast<int>(nof_codeblocks),
      static_cast<int>(stride));

  return check_last_cuda_error("rate matching");
}

cuda_result ldpc_rate_matcher::interleave(uint32_t*          d_output,
                                          const uint32_t*    d_input,
                                          unsigned           nof_bits,
                                          unsigned           bits_per_symbol,
                                          const cuda_stream& stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_input == nullptr) || (d_output == nullptr) || (nof_bits == 0) || (bits_per_symbol == 0)) {
    return make_unexpected(std::string("The interleaving call carries an invalid configuration"));
  }
  if ((nof_bits % bits_per_symbol) != 0) {
    // The specification makes the rate matched length a multiple of the modulation order. Without
    // it the last bits of the sequence map nowhere.
    return make_unexpected(std::string("The number of bits to interleave is not a multiple of the modulation order"));
  }

  bit_interleave_kernel<<<blocks_for(nof_bits), block_size, 0, native_stream>>>(
      d_input, d_output, static_cast<int>(nof_bits), static_cast<int>(bits_per_symbol), 1);

  return check_last_cuda_error("bit interleaving");
}

// ============================================================================
// Rate dematcher
// ============================================================================

struct ldpc_rate_dematcher::impl {
  rate_matching_geometry geometry;
  bool                   configured = false;
};

/// \brief Ratio given to a filler bit.
///
/// A filler bit is known to be 0, so the decoder is told so with a ratio far outside the range the
/// demodulator produces.
static constexpr float FILLER_LLR = 10000.0F;

cuda_expected<ldpc_rate_dematcher> ldpc_rate_dematcher::create()
{
  std::unique_ptr<impl> created(new (std::nothrow) impl());
  if (created == nullptr) {
    return make_unexpected(std::string("The rate dematcher state could not be allocated"));
  }

  return ldpc_rate_dematcher(std::move(created));
}

ldpc_rate_dematcher::ldpc_rate_dematcher() = default;

ldpc_rate_dematcher::ldpc_rate_dematcher(std::unique_ptr<impl> state_) : state(std::move(state_)) {}

ldpc_rate_dematcher::ldpc_rate_dematcher(ldpc_rate_dematcher&& other) noexcept = default;

ldpc_rate_dematcher& ldpc_rate_dematcher::operator=(ldpc_rate_dematcher&& other) noexcept = default;

ldpc_rate_dematcher::~ldpc_rate_dematcher() = default;

bool ldpc_rate_dematcher::is_valid() const
{
  return state != nullptr;
}

cuda_result ldpc_rate_dematcher::configure(const codeblock_metadata& cfg)
{
  cuda_result status = derive_geometry(state->geometry, cfg);
  if (!status.has_value()) {
    return status;
  }
  state->configured = true;

  return {};
}

unsigned ldpc_rate_dematcher::get_output_bits() const
{
  return static_cast<unsigned>(state->geometry.N_full);
}

cuda_result ldpc_rate_dematcher::rate_dematch(float*             d_output,
                                              const float*       d_received,
                                              unsigned           nof_codeblocks,
                                              const cuda_stream& stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_received == nullptr) || (d_output == nullptr) || (nof_codeblocks == 0)) {
    return make_unexpected(std::string("The rate dematching call carries an invalid configuration"));
  }
  if (!state->configured) {
    return make_unexpected(std::string("The rate dematcher is not configured"));
  }

  const rate_matching_geometry& g            = state->geometry;
  int                           Kd_in_buffer = g.Kd - g.puncture_offset;

  cuda_result status =
      check_cuda_error(static_cast<int>(cudaMemsetAsync(
                           d_output, 0, static_cast<size_t>(g.N_full) * nof_codeblocks * sizeof(float), native_stream)),
                       "clearing the dematched output");
  if (!status.has_value()) {
    return status;
  }

  rate_match_rx_kernel<<<blocks_for(g.E * nof_codeblocks), block_size, 0, native_stream>>>(
      d_received,
      d_output,
      g.N,
      g.N_full,
      g.k0,
      g.E,
      g.Z,
      g.N_cols,
      g.puncture_offset,
      Kd_in_buffer,
      g.F,
      static_cast<int>(nof_codeblocks));

  // The punctured columns were never transmitted, so the decoder is told nothing about them.
  set_punctured_llrs_kernel<<<blocks_for(g.puncture_offset), block_size, 0, native_stream>>>(
      d_output, g.puncture_offset, g.N_full, static_cast<int>(nof_codeblocks));

  if (g.F > 0) {
    set_filler_llrs_kernel<<<blocks_for(g.F), block_size, 0, native_stream>>>(
        d_output, Kd_in_buffer, g.F, g.Z, g.N_cols, g.puncture_offset, FILLER_LLR, static_cast<int>(nof_codeblocks));
  }

  return check_last_cuda_error("rate dematching");
}

cuda_result ldpc_rate_dematcher::rate_dematch(__half*            d_output,
                                              const __half*      d_received,
                                              unsigned           nof_codeblocks,
                                              const cuda_stream& stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_received == nullptr) || (d_output == nullptr) || (nof_codeblocks == 0)) {
    return make_unexpected(std::string("The rate dematching call carries an invalid configuration"));
  }
  if (!state->configured) {
    return make_unexpected(std::string("The rate dematcher is not configured"));
  }

  const rate_matching_geometry& g            = state->geometry;
  int                           Kd_in_buffer = g.Kd - g.puncture_offset;

  cuda_result status = check_cuda_error(
      static_cast<int>(
          cudaMemsetAsync(d_output, 0, static_cast<size_t>(g.N_full) * nof_codeblocks * sizeof(__half), native_stream)),
      "clearing the dematched output");
  if (!status.has_value()) {
    return status;
  }

  rate_match_rx_half_kernel<<<blocks_for(g.E * nof_codeblocks), block_size, 0, native_stream>>>(
      d_received,
      d_output,
      g.N,
      g.N_full,
      g.k0,
      g.E,
      g.Z,
      g.N_cols,
      g.puncture_offset,
      Kd_in_buffer,
      g.F,
      static_cast<int>(nof_codeblocks),
      true);

  return check_last_cuda_error("rate dematching in half precision");
}

cuda_result ldpc_rate_dematcher::deinterleave_and_dematch(__half*            d_output,
                                                          const __half*      d_received,
                                                          unsigned           bits_per_symbol,
                                                          unsigned           nof_codeblocks,
                                                          const cuda_stream& stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_received == nullptr) || (d_output == nullptr) || (bits_per_symbol == 0) || (nof_codeblocks == 0)) {
    return make_unexpected(std::string("The dematching call carries an invalid configuration"));
  }
  if (!state->configured) {
    return make_unexpected(std::string("The rate dematcher is not configured"));
  }

  const rate_matching_geometry& g = state->geometry;
  if ((static_cast<unsigned>(g.E) % bits_per_symbol) != 0) {
    return make_unexpected(std::string("The rate matched length is not a multiple of the modulation order"));
  }

  int Kd_in_buffer = g.Kd - g.puncture_offset;
  // Number of modulation symbols the matched bits fill.
  int nof_symbols = g.E / static_cast<int>(bits_per_symbol);
  // The selection repeats itself only when it asks for more bits than the buffer holds.
  bool no_repetition = g.E <= (g.N - g.F);

  cuda_result status = check_cuda_error(
      static_cast<int>(
          cudaMemsetAsync(d_output, 0, static_cast<size_t>(g.N_full) * nof_codeblocks * sizeof(__half), native_stream)),
      "clearing the dematched output");
  if (!status.has_value()) {
    return status;
  }

  dim3 grid(blocks_for(g.E), nof_codeblocks);
  rate_dematch_deinterleave_half_kernel<<<grid, block_size, 0, native_stream>>>(d_received,
                                                                                d_output,
                                                                                g.N,
                                                                                g.N_full,
                                                                                g.k0,
                                                                                g.E,
                                                                                g.Z,
                                                                                g.N_cols,
                                                                                g.puncture_offset,
                                                                                Kd_in_buffer,
                                                                                g.F,
                                                                                static_cast<int>(bits_per_symbol),
                                                                                nof_symbols,
                                                                                static_cast<int>(nof_codeblocks),
                                                                                no_repetition);

  return check_last_cuda_error("deinterleaving and rate dematching in half precision");
}

cuda_result ldpc_rate_dematcher::deinterleave(float*             d_output,
                                              const float*       d_input,
                                              unsigned           nof_bits,
                                              unsigned           bits_per_symbol,
                                              unsigned           nof_codeblocks,
                                              const cuda_stream& stream)
{
  ::cudaStream_t native_stream = static_cast<::cudaStream_t>(stream.native());

  if ((d_input == nullptr) || (d_output == nullptr) || (nof_bits == 0) || (bits_per_symbol == 0) ||
      (nof_codeblocks == 0)) {
    return make_unexpected(std::string("The deinterleaving call carries an invalid configuration"));
  }
  if ((nof_bits % bits_per_symbol) != 0) {
    return make_unexpected(
        std::string("The number of ratios to deinterleave is not a multiple of the modulation order"));
  }

  bit_deinterleave_llr_kernel<<<blocks_for(nof_bits * nof_codeblocks), block_size, 0, native_stream>>>(
      d_input,
      d_output,
      static_cast<int>(nof_bits),
      static_cast<int>(bits_per_symbol),
      static_cast<int>(nof_codeblocks));

  return check_last_cuda_error("bit deinterleaving");
}

} // namespace cuda
} // namespace ocudu

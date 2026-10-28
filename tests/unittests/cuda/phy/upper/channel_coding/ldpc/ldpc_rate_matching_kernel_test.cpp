// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/cuda/adt/cuda_copy.h"
#include "ocudu/cuda/adt/cuda_stream.h"
#include "ocudu/cuda/adt/device_vector.h"
#include "ocudu/cuda/phy/upper/channel_coding/ldpc/ldpc_rate_matching.h"
#include "ocudu/phy/upper/channel_coding/channel_coding_factories.h"
#include "ocudu/phy/upper/channel_coding/ldpc/ldpc.h"
#include "ocudu/phy/upper/channel_coding/ldpc/ldpc_encoder_buffer.h"
#include "ocudu/ran/sch/sch_constants.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using namespace ocudu;

namespace {

/// Returns true if a CUDA device is available to run the test.
bool has_cuda_device()
{
  int nof_devices = 0;
  return (::cudaGetDeviceCount(&nof_devices) == cudaSuccess) && (nof_devices > 0);
}

/// Shape a case matches, and what it is chosen to reach.
struct matcher_case {
  ldpc_base_graph_type base_graph;
  ldpc::lifting_size_t lifting_size;
  unsigned             rv;
  unsigned             nof_filler_bits;
  float                rate;
  modulation_scheme    mod;
  const char*          reaches;
  /// Limited circular buffer size N_ref of TS 38.212, Section 5.4.2.1, or 0 for the full buffer.
  unsigned Nref = 0;
};

/// Limited circular buffer size N_ref the gNB uses for a transport block of 76 code blocks under the default TBS_LBRM.
const unsigned gnb_Nref = ldpc::compute_N_ref(tbs_lbrm_default, 76).value();

/// Shapes that between them cover both base graphs, every redundancy version, filler bits, and a
/// rate matched length longer than the circular buffer, which makes the selection wrap.
const std::vector<matcher_case> cases = {
    {ldpc_base_graph_type::BG1, ldpc::LS384, 0, 0, 0.5F, modulation_scheme::QPSK, "base graph 1, redundancy version 0"},
    {ldpc_base_graph_type::BG1,
     ldpc::LS384,
     1,
     0,
     0.5F,
     modulation_scheme::QAM16,
     "base graph 1, redundancy version 1"},
    {ldpc_base_graph_type::BG1,
     ldpc::LS384,
     2,
     0,
     0.5F,
     modulation_scheme::QAM64,
     "base graph 1, redundancy version 2"},
    {ldpc_base_graph_type::BG1,
     ldpc::LS384,
     3,
     0,
     0.5F,
     modulation_scheme::QAM256,
     "base graph 1, redundancy version 3"},
    {ldpc_base_graph_type::BG1, ldpc::LS256, 0, 128, 0.5F, modulation_scheme::QPSK, "filler bits"},
    {ldpc_base_graph_type::BG1,
     ldpc::LS256,
     0,
     0,
     1.5F,
     modulation_scheme::QPSK,
     "a length that wraps the circular buffer"},
    {ldpc_base_graph_type::BG2, ldpc::LS384, 0, 0, 0.5F, modulation_scheme::QPSK, "base graph 2, redundancy version 0"},
    {ldpc_base_graph_type::BG2,
     ldpc::LS384,
     2,
     0,
     0.5F,
     modulation_scheme::QAM64,
     "base graph 2, redundancy version 2"},
    {ldpc_base_graph_type::BG2, ldpc::LS128, 0, 64, 0.5F, modulation_scheme::QAM16, "base graph 2 with filler bits"},
    // Limited-buffer rate matching: N_cb = min(N, N_ref) sets the starting position of each redundancy version and the
    // point where the selection wraps. The rate matched lengths exceed N_ref, so the selection wraps.
    {ldpc_base_graph_type::BG1, ldpc::LS384, 0, 0, 0.5F, modulation_scheme::QPSK, "limited buffer, rv 0", 12000},
    {ldpc_base_graph_type::BG1, ldpc::LS384, 1, 0, 0.5F, modulation_scheme::QAM16, "limited buffer, rv 1", 12000},
    {ldpc_base_graph_type::BG1, ldpc::LS384, 2, 0, 0.5F, modulation_scheme::QAM64, "limited buffer, rv 2", 12000},
    {ldpc_base_graph_type::BG1, ldpc::LS384, 3, 0, 0.5F, modulation_scheme::QAM256, "limited buffer, rv 3", 12000},
    {ldpc_base_graph_type::BG2, ldpc::LS256, 2, 64, 0.5F, modulation_scheme::QPSK, "limited buffer, BG2, filler", 4000},
    // N_ref as the gNB derives it from the default TBS_LBRM for a transport block of 76 code blocks, the fewest for
    // which N_ref < N. The rate matched length is shorter than N_ref, so redundancy versions 0 and 1 never reach the
    // end of the limited buffer, while redundancy versions 2 and 3 start late enough to wrap around it.
    {ldpc_base_graph_type::BG1, ldpc::LS384, 0, 0, 0.5F, modulation_scheme::QAM64, "default TBS_LBRM, rv 0", gnb_Nref},
    {ldpc_base_graph_type::BG1, ldpc::LS384, 1, 0, 0.5F, modulation_scheme::QAM64, "default TBS_LBRM, rv 1", gnb_Nref},
    {ldpc_base_graph_type::BG1, ldpc::LS384, 2, 0, 0.5F, modulation_scheme::QAM256, "default TBS_LBRM, rv 2", gnb_Nref},
    {ldpc_base_graph_type::BG1, ldpc::LS384, 3, 0, 0.5F, modulation_scheme::QAM256, "default TBS_LBRM, rv 3", gnb_Nref},
};

class ldpc_rate_matching_kernel_test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!has_cuda_device()) {
      GTEST_SKIP() << "No CUDA device available";
    }

    cuda::cuda_expected<cuda::cuda_stream> created_stream = cuda::cuda_stream::create();
    ASSERT_TRUE(created_stream.has_value());
    stream = std::move(created_stream.value());

    cuda::cuda_expected<cuda::ldpc_rate_matcher> created_matcher = cuda::ldpc_rate_matcher::create();
    ASSERT_TRUE(created_matcher.has_value()) << created_matcher.error();
    gpu_matcher = std::move(created_matcher.value());

    cuda::cuda_expected<cuda::ldpc_rate_dematcher> created_dematcher = cuda::ldpc_rate_dematcher::create();
    ASSERT_TRUE(created_dematcher.has_value()) << created_dematcher.error();
    gpu_dematcher = std::move(created_dematcher.value());

    encoder           = create_ldpc_encoder_factory_sw("generic")->create();
    reference_matcher = create_ldpc_rate_matcher_factory_sw()->create();
    ASSERT_NE(encoder, nullptr);
    ASSERT_NE(reference_matcher, nullptr);
  }

  /// Fills a bit buffer with pseudo-random bits, the same for a given size on every run.
  static dynamic_bit_buffer make_bits(unsigned nof_bits, unsigned seed)
  {
    dynamic_bit_buffer                      bits(nof_bits);
    std::mt19937                            rgen(seed);
    std::uniform_int_distribution<unsigned> dist(0, 1);
    for (unsigned i = 0; i != nof_bits; ++i) {
      bits.insert(dist(rgen), i, 1);
    }
    return bits;
  }

  /// \brief Position bit \c index occupies inside its 32-bit word.
  ///
  /// The bits run most significant first inside a byte, and the bytes least significant first
  /// inside the word, which is the layout the encoder writes and the rate matcher reads.
  static unsigned bit_position(unsigned index)
  {
    unsigned bit_in_word  = index % 32;
    unsigned byte_in_word = bit_in_word / 8;
    unsigned bit_in_byte  = 7 - (bit_in_word % 8);
    return byte_in_word * 8 + bit_in_byte;
  }

  static uint8_t word_bit(const std::vector<uint32_t>& words, unsigned index)
  {
    return static_cast<uint8_t>((words[index / 32] >> bit_position(index)) & 1U);
  }

  /// \brief Builds the full codeword the rate matcher reads.
  ///
  /// The software encoder returns the codeword the specification transmits, which leaves out the
  /// first 2 lifted columns. Those columns carry the first bits of the message, so putting the
  /// message back in front of the encoder output reconstructs what the GPU encoder writes.
  static std::vector<uint8_t> full_codeword(const bit_buffer& message, const ldpc_encoder_buffer& encoded, unsigned Z)
  {
    unsigned             nof_punctured = 2 * Z;
    std::vector<uint8_t> transmitted(encoded.get_codeblock_length());
    encoded.write_codeblock(transmitted, 0);

    std::vector<uint8_t> full(nof_punctured + transmitted.size());
    for (unsigned i = 0; i != nof_punctured; ++i) {
      full[i] = message.extract(i, 1);
    }
    std::copy(transmitted.begin(), transmitted.end(), full.begin() + nof_punctured);
    return full;
  }

  /// Packs unpacked bits the way the kernels read them.
  static std::vector<uint32_t> pack(const std::vector<uint8_t>& bits)
  {
    std::vector<uint32_t> words((bits.size() + 31) / 32, 0);
    for (unsigned i = 0, end = bits.size(); i != end; ++i) {
      if (bits[i] != 0) {
        words[i / 32] |= (1U << bit_position(i));
      }
    }
    return words;
  }

  /// Builds the description both the software and the GPU rate matcher are configured from.
  static codeblock_metadata make_metadata(const matcher_case& c, unsigned nof_encoded_bits)
  {
    codeblock_metadata cfg;
    cfg.tb_common.base_graph   = c.base_graph;
    cfg.tb_common.lifting_size = c.lifting_size;
    cfg.tb_common.rv           = c.rv;
    cfg.tb_common.mod          = c.mod;
    cfg.tb_common.Nref         = c.Nref;

    unsigned Kb                     = (c.base_graph == ldpc_base_graph_type::BG1) ? 22 : 10;
    cfg.cb_specific.full_length     = nof_encoded_bits;
    cfg.cb_specific.nof_filler_bits = c.nof_filler_bits;
    cfg.cb_specific.rm_length =
        static_cast<unsigned>(static_cast<float>(Kb * static_cast<unsigned>(c.lifting_size)) / c.rate);
    cfg.cb_specific.cw_offset    = 0;
    cfg.cb_specific.nof_crc_bits = 16;
    return cfg;
  }
  cuda::cuda_stream stream;

  cuda::ldpc_rate_matcher                   gpu_matcher;
  cuda::ldpc_rate_dematcher                 gpu_dematcher;
  std::unique_ptr<ocudu::ldpc_encoder>      encoder;
  std::unique_ptr<ocudu::ldpc_rate_matcher> reference_matcher;
};

} // namespace

TEST_F(ldpc_rate_matching_kernel_test, rate_matching_matches_the_software_matcher)
{
  for (const matcher_case& c : cases) {
    unsigned Z  = static_cast<unsigned>(c.lifting_size);
    unsigned Kb = (c.base_graph == ldpc_base_graph_type::BG1) ? 22 : 10;
    unsigned K  = Kb * Z;

    dynamic_bit_buffer message = make_bits(K, 0x5eed);

    ocudu::ldpc_encoder::configuration enc_cfg;
    enc_cfg.base_graph   = c.base_graph;
    enc_cfg.lifting_size = c.lifting_size;

    const ldpc_encoder_buffer& encoded = encoder->encode(message, enc_cfg);

    std::vector<uint8_t> full = full_codeword(message, encoded, Z);
    codeblock_metadata   cfg  = make_metadata(c, static_cast<unsigned>(full.size()));
    unsigned             E    = cfg.cb_specific.rm_length;

    // The software rate matcher reads the encoder buffer directly.
    dynamic_bit_buffer expected(E);
    reference_matcher->rate_match(expected, encoded, cfg);

    // The GPU reads the full codeword, the punctured columns included.
    std::vector<uint32_t>                              words = pack(full);
    cuda::cuda_expected<cuda::device_vector<uint32_t>> d_in  = cuda::device_vector<uint32_t>::create(words.size());
    cuda::cuda_expected<cuda::device_vector<uint32_t>> d_out = cuda::device_vector<uint32_t>::create((E + 31) / 32);
    ASSERT_TRUE(d_in.has_value());
    ASSERT_TRUE(d_out.has_value());
    ASSERT_TRUE(cuda::copy_to_device(d_in.value(), span<const uint32_t>(words.data(), words.size())).has_value());

    cuda::cuda_expected<cuda::device_vector<uint32_t>> d_il = cuda::device_vector<uint32_t>::create((E + 31) / 32);
    ASSERT_TRUE(d_il.has_value());

    ASSERT_TRUE(gpu_matcher.configure(cfg).has_value()) << c.reaches;
    ASSERT_EQ(gpu_matcher.get_output_bits(), E) << c.reaches;
    ASSERT_TRUE(gpu_matcher.rate_match(d_out.value().data(), d_in.value().data(), 1, 0, stream).has_value())
        << c.reaches;
    // The software rate matcher interleaves as part of matching, so the comparison is against the
    // selection followed by the interleaver.
    ASSERT_TRUE(
        gpu_matcher.interleave(d_il.value().data(), d_out.value().data(), E, static_cast<unsigned>(c.mod), stream)
            .has_value())
        << c.reaches;
    ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess) << c.reaches;

    std::vector<uint32_t> actual((E + 31) / 32);
    ASSERT_TRUE(cuda::copy_to_host(span<uint32_t>(actual.data(), actual.size()), d_il.value()).has_value());

    for (unsigned i = 0; i != E; ++i) {
      ASSERT_EQ(word_bit(actual, i), expected.extract(i, 1)) << c.reaches << ", bit " << i;
    }
  }
}

TEST_F(ldpc_rate_matching_kernel_test, dematching_puts_the_ratios_back_where_the_bits_came_from)
{
  // Dematching is the inverse of matching, so feeding it a ratio per matched bit must put each
  // ratio at the position the matcher took that bit from.
  const matcher_case& c  = cases.front();
  unsigned            Z  = static_cast<unsigned>(c.lifting_size);
  unsigned            Kb = 22;
  unsigned            K  = Kb * Z;

  dynamic_bit_buffer message = make_bits(K, 0x5eed);

  ocudu::ldpc_encoder::configuration enc_cfg;
  enc_cfg.base_graph                 = c.base_graph;
  enc_cfg.lifting_size               = c.lifting_size;
  const ldpc_encoder_buffer& encoded = encoder->encode(message, enc_cfg);

  std::vector<uint8_t> full = full_codeword(message, encoded, Z);
  codeblock_metadata   cfg  = make_metadata(c, static_cast<unsigned>(full.size()));
  unsigned             E    = cfg.cb_specific.rm_length;

  // The dematcher reverses the selection, not the interleaving, so it is fed what the GPU matcher
  // selected rather than what the software matcher returns.
  std::vector<uint32_t>                              words = pack(full);
  cuda::cuda_expected<cuda::device_vector<uint32_t>> d_cw  = cuda::device_vector<uint32_t>::create(words.size());
  cuda::cuda_expected<cuda::device_vector<uint32_t>> d_rm  = cuda::device_vector<uint32_t>::create((E + 31) / 32);
  ASSERT_TRUE(d_cw.has_value());
  ASSERT_TRUE(d_rm.has_value());
  ASSERT_TRUE(cuda::copy_to_device(d_cw.value(), span<const uint32_t>(words.data(), words.size())).has_value());
  ASSERT_TRUE(gpu_matcher.configure(cfg).has_value());
  ASSERT_TRUE(gpu_matcher.rate_match(d_rm.value().data(), d_cw.value().data(), 1, 0, stream).has_value());
  ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

  std::vector<uint32_t> matched((E + 31) / 32);
  ASSERT_TRUE(cuda::copy_to_host(span<uint32_t>(matched.data(), matched.size()), d_rm.value()).has_value());

  // A clean channel: a 0 bit becomes a positive ratio, a 1 bit a negative one.
  std::vector<float> received(E);
  for (unsigned i = 0; i != E; ++i) {
    received[i] = (word_bit(matched, i) != 0) ? -1.0F : 1.0F;
  }

  ASSERT_TRUE(gpu_dematcher.configure(cfg).has_value());
  unsigned nof_output = gpu_dematcher.get_output_bits();

  cuda::cuda_expected<cuda::device_vector<float>> d_in  = cuda::device_vector<float>::create(E);
  cuda::cuda_expected<cuda::device_vector<float>> d_out = cuda::device_vector<float>::create(nof_output);
  ASSERT_TRUE(d_in.has_value());
  ASSERT_TRUE(d_out.has_value());
  ASSERT_TRUE(cuda::copy_to_device(d_in.value(), span<const float>(received.data(), received.size())).has_value());

  ASSERT_TRUE(gpu_dematcher.rate_dematch(d_out.value().data(), d_in.value().data(), 1, stream).has_value());
  ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

  std::vector<float> dematched(nof_output);
  ASSERT_TRUE(cuda::copy_to_host(span<float>(dematched.data(), dematched.size()), d_out.value()).has_value());

  // The punctured columns were never transmitted, so they carry no information.
  for (unsigned i = 0; i != 2 * Z; ++i) {
    EXPECT_FLOAT_EQ(dematched[i], 0.0F) << "punctured bit " << i;
  }

  // Every position the dematcher did fill must agree in sign with the codeword bit there.
  unsigned nof_checked = 0;
  for (unsigned i = 2 * Z; i != nof_output; ++i) {
    if (dematched[i] == 0.0F) {
      continue;
    }
    uint8_t decided = static_cast<uint8_t>(dematched[i] <= 0.0F);
    ASSERT_EQ(decided, full[i]) << "bit " << i;
    ++nof_checked;
  }
  EXPECT_GE(nof_checked, E / 2) << "the dematcher filled too few positions to be meaningful";
}

TEST_F(ldpc_rate_matching_kernel_test, dematching_matches_the_software_matcher_with_a_limited_buffer)
{
  // The receiver must dematch with the same limited circular buffer the transmitter matched with. The reference here is
  // the software rate matcher with BPSK, whose bit interleaver is the identity, so its output is the selection itself.
  for (const matcher_case& c : cases) {
    if (c.Nref == 0) {
      continue;
    }
    unsigned Z  = static_cast<unsigned>(c.lifting_size);
    unsigned Kb = (c.base_graph == ldpc_base_graph_type::BG1) ? 22 : 10;
    unsigned K  = Kb * Z;

    dynamic_bit_buffer                 message = make_bits(K, 0x5eed);
    ocudu::ldpc_encoder::configuration enc_cfg;
    enc_cfg.base_graph                 = c.base_graph;
    enc_cfg.lifting_size               = c.lifting_size;
    const ldpc_encoder_buffer& encoded = encoder->encode(message, enc_cfg);

    std::vector<uint8_t> full = full_codeword(message, encoded, Z);
    codeblock_metadata   cfg  = make_metadata(c, static_cast<unsigned>(full.size()));
    cfg.tb_common.mod         = modulation_scheme::BPSK;
    unsigned E                = cfg.cb_specific.rm_length;

    dynamic_bit_buffer selected(E);
    reference_matcher->rate_match(selected, encoded, cfg);

    // A clean channel: a 0 bit becomes a positive ratio, a 1 bit a negative one.
    std::vector<float> received(E);
    for (unsigned i = 0; i != E; ++i) {
      received[i] = (selected.extract(i, 1) != 0) ? -1.0F : 1.0F;
    }

    ASSERT_TRUE(gpu_dematcher.configure(cfg).has_value()) << c.reaches;
    unsigned nof_output = gpu_dematcher.get_output_bits();

    cuda::cuda_expected<cuda::device_vector<float>> d_in  = cuda::device_vector<float>::create(E);
    cuda::cuda_expected<cuda::device_vector<float>> d_out = cuda::device_vector<float>::create(nof_output);
    ASSERT_TRUE(d_in.has_value());
    ASSERT_TRUE(d_out.has_value());
    ASSERT_TRUE(cuda::copy_to_device(d_in.value(), span<const float>(received.data(), received.size())).has_value());
    ASSERT_TRUE(gpu_dematcher.rate_dematch(d_out.value().data(), d_in.value().data(), 1, stream).has_value())
        << c.reaches;
    ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess) << c.reaches;

    std::vector<float> dematched(nof_output);
    ASSERT_TRUE(cuda::copy_to_host(span<float>(dematched.data(), dematched.size()), d_out.value()).has_value());

    // Every position the dematcher filled must agree in sign with the codeword bit there. Filler bits, the last
    // c.nof_filler_bits of the K systematic bits, are reported as known zeros and are checked elsewhere.
    unsigned nof_checked = 0;
    for (unsigned i = 2 * Z; i != nof_output; ++i) {
      if ((dematched[i] == 0.0F) || ((i >= K - c.nof_filler_bits) && (i < K))) {
        continue;
      }
      ASSERT_EQ(static_cast<uint8_t>(dematched[i] <= 0.0F), full[i]) << c.reaches << ", bit " << i;
      ++nof_checked;
    }
    EXPECT_GE(nof_checked, std::min(E, c.Nref) / 2) << "the dematcher filled too few positions, " << c.reaches;
  }
}

TEST_F(ldpc_rate_matching_kernel_test, filler_bits_are_reported_as_known_zeros)
{
  // A filler bit is never transmitted and is known to be 0, so the decoder must be given a large
  // positive ratio rather than no information.
  const matcher_case& c = cases[4];
  unsigned            Z = static_cast<unsigned>(c.lifting_size);
  unsigned            K = 22 * Z;
  unsigned            F = c.nof_filler_bits;

  dynamic_bit_buffer message = make_bits(K, 0x5eed);

  ocudu::ldpc_encoder::configuration enc_cfg;
  enc_cfg.base_graph                 = c.base_graph;
  enc_cfg.lifting_size               = c.lifting_size;
  const ldpc_encoder_buffer& encoded = encoder->encode(message, enc_cfg);

  std::vector<uint8_t> full = full_codeword(message, encoded, Z);
  codeblock_metadata   cfg  = make_metadata(c, static_cast<unsigned>(full.size()));
  unsigned             E    = cfg.cb_specific.rm_length;

  std::vector<float> received(E, 1.0F);

  ASSERT_TRUE(gpu_dematcher.configure(cfg).has_value());
  unsigned nof_output = gpu_dematcher.get_output_bits();

  cuda::cuda_expected<cuda::device_vector<float>> d_in  = cuda::device_vector<float>::create(E);
  cuda::cuda_expected<cuda::device_vector<float>> d_out = cuda::device_vector<float>::create(nof_output);
  ASSERT_TRUE(d_in.has_value());
  ASSERT_TRUE(d_out.has_value());
  ASSERT_TRUE(cuda::copy_to_device(d_in.value(), span<const float>(received.data(), received.size())).has_value());
  ASSERT_TRUE(gpu_dematcher.rate_dematch(d_out.value().data(), d_in.value().data(), 1, stream).has_value());
  ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

  std::vector<float> dematched(nof_output);
  ASSERT_TRUE(cuda::copy_to_host(span<float>(dematched.data(), dematched.size()), d_out.value()).has_value());

  // The filler bits sit at the end of the systematic region.
  unsigned filler_start = K - F;
  for (unsigned i = filler_start; i != K; ++i) {
    EXPECT_GT(dematched[i], 100.0F) << "filler bit " << i;
  }
}

TEST_F(ldpc_rate_matching_kernel_test, deinterleaving_undoes_interleaving)
{
  // The specification makes the rate matched length a multiple of the modulation order, and 3840
  // is a multiple of every order tested here.
  unsigned E = 3840;

  for (unsigned bits_per_symbol : {2U, 4U, 6U, 8U}) {
    std::vector<uint8_t> bits(E);
    std::mt19937         rgen(0x5eed);
    for (uint8_t& b : bits) {
      b = static_cast<uint8_t>(rgen() & 1U);
    }
    std::vector<uint32_t> words = pack(bits);

    cuda::cuda_expected<cuda::device_vector<uint32_t>> d_in  = cuda::device_vector<uint32_t>::create(words.size());
    cuda::cuda_expected<cuda::device_vector<uint32_t>> d_out = cuda::device_vector<uint32_t>::create(words.size());
    ASSERT_TRUE(d_in.has_value());
    ASSERT_TRUE(d_out.has_value());
    ASSERT_TRUE(cuda::copy_to_device(d_in.value(), span<const uint32_t>(words.data(), words.size())).has_value());

    ASSERT_TRUE(
        gpu_matcher.interleave(d_out.value().data(), d_in.value().data(), E, bits_per_symbol, stream).has_value());
    ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

    std::vector<uint32_t> interleaved(words.size());
    ASSERT_TRUE(cuda::copy_to_host(span<uint32_t>(interleaved.data(), interleaved.size()), d_out.value()).has_value());

    // Turn the interleaved bits into ratios and send them back through the deinterleaver.
    std::vector<float> ratios(E);
    for (unsigned i = 0; i != E; ++i) {
      ratios[i] = (word_bit(interleaved, i) != 0) ? -1.0F : 1.0F;
    }

    cuda::cuda_expected<cuda::device_vector<float>> d_r   = cuda::device_vector<float>::create(E);
    cuda::cuda_expected<cuda::device_vector<float>> d_der = cuda::device_vector<float>::create(E);
    ASSERT_TRUE(d_r.has_value());
    ASSERT_TRUE(d_der.has_value());
    ASSERT_TRUE(cuda::copy_to_device(d_r.value(), span<const float>(ratios.data(), ratios.size())).has_value());
    ASSERT_TRUE(gpu_dematcher.deinterleave(d_der.value().data(), d_r.value().data(), E, bits_per_symbol, 1, stream)
                    .has_value());
    ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

    std::vector<float> deinterleaved(E);
    ASSERT_TRUE(cuda::copy_to_host(span<float>(deinterleaved.data(), deinterleaved.size()), d_der.value()).has_value());

    for (unsigned i = 0; i != E; ++i) {
      uint8_t decided = static_cast<uint8_t>(deinterleaved[i] <= 0.0F);
      ASSERT_EQ(decided, bits[i]) << bits_per_symbol << " bits per symbol, bit " << i;
    }
  }
}

TEST_F(ldpc_rate_matching_kernel_test, working_without_a_configuration_is_reported)
{
  cuda::cuda_expected<cuda::device_vector<uint32_t>> d_in  = cuda::device_vector<uint32_t>::create(16);
  cuda::cuda_expected<cuda::device_vector<uint32_t>> d_out = cuda::device_vector<uint32_t>::create(16);
  ASSERT_TRUE(d_in.has_value());
  ASSERT_TRUE(d_out.has_value());

  cuda::cuda_result matched = gpu_matcher.rate_match(d_out.value().data(), d_in.value().data(), 1, 0, stream);
  ASSERT_FALSE(matched.has_value());
  EXPECT_FALSE(matched.error().empty());
}

TEST_F(ldpc_rate_matching_kernel_test, the_fused_pass_agrees_with_deinterleaving_then_dematching)
{
  // Doing both in 1 pass must give what doing them one after the other gives.
  const matcher_case& c  = cases.front();
  unsigned            Z  = static_cast<unsigned>(c.lifting_size);
  unsigned            K  = 22 * Z;
  unsigned            Qm = 2;

  dynamic_bit_buffer                 message = make_bits(K, 0x5eed);
  ocudu::ldpc_encoder::configuration enc_cfg;
  enc_cfg.base_graph                 = c.base_graph;
  enc_cfg.lifting_size               = c.lifting_size;
  const ldpc_encoder_buffer& encoded = encoder->encode(message, enc_cfg);

  std::vector<uint8_t> full = full_codeword(message, encoded, Z);
  codeblock_metadata   cfg  = make_metadata(c, static_cast<unsigned>(full.size()));
  unsigned             E    = cfg.cb_specific.rm_length;

  // An interleaved sequence of ratios, as a demodulator would hand over.
  std::vector<__half> received(E);
  for (unsigned i = 0; i != E; ++i) {
    received[i] = __float2half(((i % 7) < 3) ? 1.5F : -1.5F);
  }

  ASSERT_TRUE(gpu_dematcher.configure(cfg).has_value());
  unsigned nof_output = gpu_dematcher.get_output_bits();

  cuda::cuda_expected<cuda::device_vector<__half>> d_in    = cuda::device_vector<__half>::create(E);
  cuda::cuda_expected<cuda::device_vector<__half>> d_fused = cuda::device_vector<__half>::create(nof_output);
  ASSERT_TRUE(d_in.has_value());
  ASSERT_TRUE(d_fused.has_value());
  ASSERT_TRUE(cuda::copy_to_device(d_in.value(), span<const __half>(received.data(), received.size())).has_value());

  ASSERT_TRUE(
      gpu_dematcher.deinterleave_and_dematch(d_fused.value().data(), d_in.value().data(), Qm, 1, stream).has_value());
  ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

  std::vector<__half> fused(nof_output);
  ASSERT_TRUE(cuda::copy_to_host(span<__half>(fused.data(), fused.size()), d_fused.value()).has_value());

  // The same work in 2 passes, through the single precision path.
  std::vector<float> received_single(E);
  for (unsigned i = 0; i != E; ++i) {
    received_single[i] = __half2float(received[i]);
  }

  cuda::cuda_expected<cuda::device_vector<float>> d_s   = cuda::device_vector<float>::create(E);
  cuda::cuda_expected<cuda::device_vector<float>> d_der = cuda::device_vector<float>::create(E);
  cuda::cuda_expected<cuda::device_vector<float>> d_out = cuda::device_vector<float>::create(nof_output);
  ASSERT_TRUE(d_s.has_value());
  ASSERT_TRUE(d_der.has_value());
  ASSERT_TRUE(d_out.has_value());
  ASSERT_TRUE(
      cuda::copy_to_device(d_s.value(), span<const float>(received_single.data(), received_single.size())).has_value());
  ASSERT_TRUE(gpu_dematcher.deinterleave(d_der.value().data(), d_s.value().data(), E, Qm, 1, stream).has_value());
  ASSERT_TRUE(gpu_dematcher.rate_dematch(d_out.value().data(), d_der.value().data(), 1, stream).has_value());
  ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess);

  std::vector<float> stepwise(nof_output);
  ASSERT_TRUE(cuda::copy_to_host(span<float>(stepwise.data(), stepwise.size()), d_out.value()).has_value());

  for (unsigned i = 0; i != nof_output; ++i) {
    ASSERT_NEAR(__half2float(fused[i]), stepwise[i], 0.01F) << "ratio " << i;
  }
}

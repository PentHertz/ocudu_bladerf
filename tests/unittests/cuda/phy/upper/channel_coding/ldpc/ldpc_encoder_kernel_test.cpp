// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/cuda/adt/cuda_copy.h"
#include "ocudu/cuda/adt/cuda_stream.h"
#include "ocudu/cuda/adt/device_vector.h"
#include "ocudu/cuda/phy/upper/channel_coding/ldpc/ldpc_encoder.h"
#include "ocudu/phy/upper/channel_coding/channel_coding_factories.h"
#include "ocudu/phy/upper/channel_coding/ldpc/ldpc_encoder_buffer.h"
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

/// Shape a case encodes, and the batch size that picks a particular kernel.
struct encoder_case {
  ldpc_base_graph_type base_graph;
  ldpc::lifting_size_t lifting_size;
  unsigned             nof_codewords;
  const char*          selects;
};

/// Shapes that between them reach every branch of the kernel selection.
const std::vector<encoder_case> cases = {
    {ldpc_base_graph_type::BG1, ldpc::LS384, 1, "one code block per block at 384"},
    {ldpc_base_graph_type::BG1, ldpc::LS384, 8, "the split pass at 384"},
    {ldpc_base_graph_type::BG1, ldpc::LS240, 1, "the fixed shape at 240"},
    {ldpc_base_graph_type::BG1, ldpc::LS192, 1, "the fixed shape at 192"},
    {ldpc_base_graph_type::BG1, ldpc::LS320, 1, "the fixed shape at 320"},
    {ldpc_base_graph_type::BG1, ldpc::LS352, 1, "the fixed shape at 352"},
    {ldpc_base_graph_type::BG1, ldpc::LS128, 8, "several code blocks per block"},
    {ldpc_base_graph_type::BG1, ldpc::LS20, 1, "the generic kernel at a lifting size off a warp"},
    {ldpc_base_graph_type::BG2, ldpc::LS384, 1, "base graph 2 at 384"},
    {ldpc_base_graph_type::BG2, ldpc::LS128, 8, "base graph 2, several per block"},
    {ldpc_base_graph_type::BG2, ldpc::LS20, 1, "base graph 2, generic kernel"},
};

class ldpc_encoder_kernel_test : public ::testing::Test
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

    cuda::cuda_expected<cuda::ldpc_encoder> created = cuda::ldpc_encoder::create();
    ASSERT_TRUE(created.has_value()) << created.error();
    gpu = std::move(created.value());

    reference = create_ldpc_encoder_factory_sw("generic")->create();
    ASSERT_NE(reference, nullptr);
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
  /// The bits run most significant first inside a byte, and the bytes run least significant first
  /// inside the word, which is how a bit_buffer reads when its bytes are viewed as words.
  static unsigned bit_position(unsigned index)
  {
    unsigned bit_in_word  = index % 32;
    unsigned byte_in_word = bit_in_word / 8;
    unsigned bit_in_byte  = 7 - (bit_in_word % 8);
    return byte_in_word * 8 + bit_in_byte;
  }

  /// Packs a bit buffer the way the kernels read it.
  static std::vector<uint32_t> pack_for_device(const bit_buffer& bits, unsigned nof_words)
  {
    std::vector<uint32_t> words(nof_words, 0);
    for (unsigned i = 0, end = bits.size(); i != end; ++i) {
      if (bits.extract(i, 1) != 0) {
        words[i / 32] |= (1U << bit_position(i));
      }
    }
    return words;
  }

  /// Returns bit \c index of a codeword the device packed.
  static uint8_t device_bit(const std::vector<uint32_t>& words, unsigned index)
  {
    return static_cast<uint8_t>((words[index / 32] >> bit_position(index)) & 1U);
  }

  cuda::cuda_stream                    stream;
  cuda::ldpc_encoder                   gpu;
  std::unique_ptr<ocudu::ldpc_encoder> reference;
};

} // namespace

TEST_F(ldpc_encoder_kernel_test, encoding_matches_the_software_encoder)
{
  for (const encoder_case& c : cases) {
    ASSERT_TRUE(gpu.configure(c.base_graph, c.lifting_size).has_value()) << c.selects;

    unsigned nof_input_bits   = gpu.get_input_bits();
    unsigned nof_input_words  = gpu.get_input_words();
    unsigned nof_output_words = gpu.get_output_words();

    // Every code block in the batch carries different bits, so a kernel that encoded the first one
    // several times would not pass.
    std::vector<uint32_t>           words(static_cast<size_t>(nof_input_words) * c.nof_codewords, 0);
    std::vector<dynamic_bit_buffer> inputs;
    for (unsigned cb = 0; cb != c.nof_codewords; ++cb) {
      inputs.push_back(make_bits(nof_input_bits, 0x5eed + cb));
      std::vector<uint32_t> one = pack_for_device(inputs.back(), nof_input_words);
      std::copy(one.begin(), one.end(), words.begin() + static_cast<size_t>(cb) * nof_input_words);
    }

    cuda::cuda_expected<cuda::device_vector<uint32_t>> d_in = cuda::device_vector<uint32_t>::create(words.size());
    cuda::cuda_expected<cuda::device_vector<uint32_t>> d_out =
        cuda::device_vector<uint32_t>::create(static_cast<size_t>(nof_output_words) * c.nof_codewords);
    ASSERT_TRUE(d_in.has_value());
    ASSERT_TRUE(d_out.has_value());
    ASSERT_TRUE(cuda::copy_to_device(d_in.value(), span<const uint32_t>(words.data(), words.size())).has_value());

    ASSERT_TRUE(gpu.encode(d_out.value().data(), d_in.value().data(), c.nof_codewords, stream).has_value())
        << c.selects;
    ASSERT_EQ(::cudaDeviceSynchronize(), cudaSuccess) << c.selects;

    std::vector<uint32_t> actual(static_cast<size_t>(nof_output_words) * c.nof_codewords);
    ASSERT_TRUE(cuda::copy_to_host(span<uint32_t>(actual.data(), actual.size()), d_out.value()).has_value());

    for (unsigned cb = 0; cb != c.nof_codewords; ++cb) {
      ocudu::ldpc_encoder::configuration cfg;
      cfg.base_graph   = c.base_graph;
      cfg.lifting_size = c.lifting_size;

      const ldpc_encoder_buffer& expected          = reference->encode(inputs[cb], cfg);
      unsigned                   nof_expected_bits = expected.get_codeblock_length();

      std::vector<uint8_t> expected_bits(nof_expected_bits);
      expected.write_codeblock(expected_bits, 0);

      // The kernels keep the 2 lifted columns the specification punctures, so the software
      // codeword starts 2 lifting sizes into what the device wrote.
      unsigned puncture_offset = 2 * static_cast<unsigned>(c.lifting_size);

      std::vector<uint32_t> one(actual.begin() + static_cast<size_t>(cb) * nof_output_words,
                                actual.begin() + static_cast<size_t>(cb + 1) * nof_output_words);

      for (unsigned i = 0; i != nof_expected_bits; ++i) {
        ASSERT_EQ(device_bit(one, puncture_offset + i), expected_bits[i])
            << c.selects << ", code block " << cb << ", bit " << i;
      }
    }
  }
}

TEST_F(ldpc_encoder_kernel_test, reconfiguring_to_the_same_shape_keeps_the_geometry)
{
  // Reconfiguring to the shape in place skips the upload, so the geometry it reports must still be
  // the one the caller asked for.
  ASSERT_TRUE(gpu.configure(ldpc_base_graph_type::BG1, ldpc::LS384).has_value());
  unsigned input_bits  = gpu.get_input_bits();
  unsigned output_bits = gpu.get_output_bits();

  ASSERT_TRUE(gpu.configure(ldpc_base_graph_type::BG1, ldpc::LS384).has_value());
  EXPECT_EQ(gpu.get_input_bits(), input_bits);
  EXPECT_EQ(gpu.get_output_bits(), output_bits);

  // A different shape must change it, and changing back must restore it.
  ASSERT_TRUE(gpu.configure(ldpc_base_graph_type::BG2, ldpc::LS128).has_value());
  EXPECT_NE(gpu.get_input_bits(), input_bits);

  ASSERT_TRUE(gpu.configure(ldpc_base_graph_type::BG1, ldpc::LS384).has_value());
  EXPECT_EQ(gpu.get_input_bits(), input_bits);
  EXPECT_EQ(gpu.get_output_bits(), output_bits);
}

TEST_F(ldpc_encoder_kernel_test, encoding_without_a_configuration_is_reported)
{
  cuda::cuda_expected<cuda::device_vector<uint32_t>> d_in  = cuda::device_vector<uint32_t>::create(16);
  cuda::cuda_expected<cuda::device_vector<uint32_t>> d_out = cuda::device_vector<uint32_t>::create(64);
  ASSERT_TRUE(d_in.has_value());
  ASSERT_TRUE(d_out.has_value());

  cuda::cuda_result status = gpu.encode(d_out.value().data(), d_in.value().data(), 1, stream);
  ASSERT_FALSE(status.has_value());
  EXPECT_FALSE(status.error().empty());
}

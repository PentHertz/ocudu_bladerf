// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#include "ocudu/cuda/adt/cuda_copy.h"
#include "ocudu/cuda/adt/cuda_stream.h"
#include "ocudu/cuda/adt/device_vector.h"
#include "ocudu/cuda/phy/upper/channel_coding/ldpc/ldpc_decoder.h"
#include "ocudu/phy/upper/channel_coding/channel_coding_factories.h"
#include "ocudu/phy/upper/channel_coding/ldpc/ldpc_encoder_buffer.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

using namespace ocudu;

namespace {

/// Returns true if a CUDA device is available to run the test.
bool has_cuda_device()
{
  int nof_devices = 0;
  return (::cudaGetDeviceCount(&nof_devices) == cudaSuccess) && (nof_devices > 0);
}

/// Lifting size and base graph of the code the tests exercise.
constexpr ldpc::lifting_size_t lifting_size            = ldpc::LS64;
constexpr unsigned             base_graph_1_info_nodes = 22;

class ldpc_decoder_kernel_test : public ::testing::Test
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

    cuda::cuda_expected<cuda::ldpc_decoder> created = cuda::ldpc_decoder::create();
    ASSERT_TRUE(created.has_value()) << created.error();
    gpu = std::move(created.value());

    encoder = create_ldpc_encoder_factory_sw("auto")->create();
    ASSERT_NE(encoder, nullptr);
  }

  /// \brief Encodes a message with the generic encoder the library already provides.
  ///
  /// \param[in] message Information bits, 1 per entry.
  /// \return The codeword, 1 bit per entry, punctured as the standard requires.
  std::vector<uint8_t> encode_with_the_library(const std::vector<uint8_t>& message)
  {
    dynamic_bit_buffer input(message.size());
    for (unsigned i = 0; i != message.size(); ++i) {
      input.insert(message[i], i, 1);
    }

    ldpc_encoder::configuration cfg = {};
    cfg.base_graph                  = ldpc_base_graph_type::BG1;
    cfg.lifting_size                = lifting_size;

    const ldpc_encoder_buffer& buffer = encoder->encode(input, cfg);

    std::vector<uint8_t> codeword(buffer.get_codeblock_length());
    buffer.write_codeblock(codeword, 0);
    return codeword;
  }

  /// \brief Builds the ratios a noiseless channel would deliver for a codeword.
  ///
  /// The first 2 lifting sizes of a base graph 1 codeword are punctured, so they are never
  /// transmitted. The decoder is given the whole codeword with those positions carrying a ratio of
  /// zero, which says that the receiver learned nothing about them.
  ///
  /// \param[in] codeword Codeword the encoder produced, 1 bit per entry.
  /// \return The ratios, of 2 lifting sizes more than the codeword.
  static std::vector<float> make_noiseless_llrs(const std::vector<uint8_t>& codeword)
  {
    std::vector<float> llrs(2 * lifting_size + codeword.size(), 0.0F);
    for (unsigned i = 0; i != codeword.size(); ++i) {
      llrs[2 * lifting_size + i] = codeword[i] ? -llr_magnitude : llr_magnitude;
    }
    return llrs;
  }

  /// \brief Reads one decoded bit out of the words the device wrote.
  ///
  /// The device writes 32 bit words, and within each word the bits are ordered by byte: bit 0 is
  /// the most significant bit of the first byte. Reading a word as 32 bits ordered from its most
  /// significant end therefore gives the wrong answer for every message except one that repeats
  /// with a period the byte order hides.
  static uint8_t decoded_bit(const std::vector<uint32_t>& packed, unsigned index)
  {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(packed.data());
    return static_cast<uint8_t>((bytes[index / 8] >> (7 - (index % 8))) & 1U);
  }

  /// Decodes one codeword on the device and returns the packed information bits.
  std::vector<uint32_t> decode_on_device(const std::vector<float>& llrs, unsigned nof_info_bits)
  {
    cuda::ldpc_decoder_params params;
    // The message carries no checksum, so stopping early has only the parity check to go on.
    params.crc = cuda::ldpc_decoder_crc::none;
    EXPECT_TRUE(gpu.configure(ldpc_base_graph_type::BG1, lifting_size, nof_info_bits, 0, params).has_value());

    // The decoder writes the systematic part of the base graph, whatever the message length is.
    unsigned nof_output_words = (base_graph_1_info_nodes * lifting_size + 31) / 32;

    cuda::cuda_expected<cuda::device_vector<float>>    d_llrs = cuda::device_vector<float>::create(llrs.size());
    cuda::cuda_expected<cuda::device_vector<uint32_t>> d_output =
        cuda::device_vector<uint32_t>::create(nof_output_words);
    EXPECT_TRUE(d_llrs.has_value());
    EXPECT_TRUE(d_output.has_value());
    EXPECT_TRUE(cuda::copy_to_device(d_llrs.value(), span<const float>(llrs.data(), llrs.size())).has_value());

    EXPECT_TRUE(gpu.decode(d_output.value().data(), d_llrs.value().data(), 1, stream).has_value());

    std::vector<uint32_t> packed(nof_output_words);
    EXPECT_TRUE(cuda::copy_to_host(span<uint32_t>(packed.data(), packed.size()), d_output.value()).has_value());
    return packed;
  }

  /// Magnitude of the ratios a noiseless channel delivers.
  static constexpr float llr_magnitude = 8.0F;

  cuda::cuda_stream                    stream;
  cuda::ldpc_decoder                   gpu;
  std::unique_ptr<ocudu::ldpc_encoder> encoder;
};

} // namespace

TEST_F(ldpc_decoder_kernel_test, a_codeword_of_the_generic_encoder_is_decoded_back_to_the_message)
{
  const unsigned nof_info_bits = base_graph_1_info_nodes * lifting_size;

  // A message that is neither constant nor periodic in the lifting size, so a decoder that
  // returned a fixed pattern or a shifted one would not match.
  std::vector<uint8_t> message(nof_info_bits);
  for (unsigned i = 0; i != nof_info_bits; ++i) {
    message[i] = static_cast<uint8_t>(((i * 5 + 1) ^ (i / 37)) & 1U);
  }

  std::vector<uint32_t> packed = decode_on_device(make_noiseless_llrs(encode_with_the_library(message)), nof_info_bits);

  for (unsigned i = 0; i != nof_info_bits; ++i) {
    ASSERT_EQ(decoded_bit(packed, i), message[i]) << "information bit " << i;
  }
}

TEST_F(ldpc_decoder_kernel_test, an_all_zero_message_is_decoded_back_to_zeros)
{
  const unsigned nof_info_bits = base_graph_1_info_nodes * lifting_size;

  // The all zero message encodes to the all zero codeword, which every linear code accepts. It is
  // included because it catches a decoder that returns its input buffer untouched, and on its own
  // it would be a weak check, which is why the test above uses a varied message.
  std::vector<uint8_t>  message(nof_info_bits, 0);
  std::vector<uint32_t> packed = decode_on_device(make_noiseless_llrs(encode_with_the_library(message)), nof_info_bits);

  for (unsigned i = 0; i != nof_info_bits; ++i) {
    ASSERT_EQ(decoded_bit(packed, i), 0) << "information bit " << i;
  }
}

TEST_F(ldpc_decoder_kernel_test, a_corrupted_codeword_is_still_decoded)
{
  const unsigned nof_info_bits = base_graph_1_info_nodes * lifting_size;

  std::vector<uint8_t> message(nof_info_bits);
  for (unsigned i = 0; i != nof_info_bits; ++i) {
    message[i] = static_cast<uint8_t>((i * 3 + 2) & 1U);
  }

  std::vector<float> llrs = make_noiseless_llrs(encode_with_the_library(message));

  // 1 in every 40 transmitted ratios is inverted, which is what the code is for. A decoder that
  // only copied the sign of each ratio would fail here while passing the tests above.
  unsigned nof_flipped = 0;
  for (unsigned i = 2 * lifting_size; i < llrs.size(); i += 40) {
    llrs[i] = -llrs[i];
    ++nof_flipped;
  }
  ASSERT_GT(nof_flipped, 100U);

  std::vector<uint32_t> packed = decode_on_device(llrs, nof_info_bits);

  for (unsigned i = 0; i != nof_info_bits; ++i) {
    ASSERT_EQ(decoded_bit(packed, i), message[i]) << "information bit " << i;
  }
}

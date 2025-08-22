// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
// Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

#ifndef MVEC_LIB__CAN_BITWORK_HPP_
#define MVEC_LIB__CAN_BITWORK_HPP_

#include <algorithm>
#include <array>
#include <climits>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace polymath::sygnal
{

inline void byteswap(unsigned char * data, size_t length)
{
  for (size_t i = 0; i < length / 2; ++i) {
    std::swap(data[i], data[length - 1 - i]);
  }
}

template <typename T>
constexpr bool is_signed_integral()
{
  return std::is_integral<T>::value && std::is_signed<T>::value;
}

template <typename T>
constexpr bool is_signed_integer()
{
  return std::is_integral<T>::value && std::is_signed<T>::value;
}

// Fills the buffer that is passed by reference
template <typename T>
bool packData(
  const T & data,
  std::array<unsigned char, 8> & buffer,
  uint8_t bit_offset,
  uint8_t pack_length,
  bool is_big_endian = false)
{
  if (
    (static_cast<size_t>(pack_length) <= sizeof(T) * CHAR_BIT) &&
    (static_cast<size_t>(pack_length) <= (sizeof(buffer) * CHAR_BIT) - bit_offset))
  {
    uint64_t & buffer_bits = *reinterpret_cast<uint64_t *>(buffer.data());
    const uint64_t raw_data_bits = *reinterpret_cast<const uint64_t *>(&data);

    uint64_t data_bits = raw_data_bits;

    if constexpr (is_signed_integral<T>()) {
      // Adjust the sign bit properly for packing
      uint64_t sign_bit = 1ULL << (sizeof(T) * CHAR_BIT - 1);
      if (data_bits & sign_bit) {
        data_bits |= ~((1ULL << pack_length) - 1);
      }
    }

    if (is_big_endian) {
      // Only swap the necessary bytes that are actually usable
      byteswap(reinterpret_cast<unsigned char *>(&data_bits), sizeof(T));
    }

    uint64_t mask = ((1ULL << pack_length) - 1) << bit_offset;

    buffer_bits &= ~mask;
    buffer_bits |= (data_bits << bit_offset) & mask;

    return true;
  } else {
    return false;
  }
}

// Fills the data object that is passed by reference
template <typename T>
bool unpackData(
  T & data,
  const std::array<unsigned char, 8> & buffer,
  uint8_t bit_offset,
  uint8_t pack_length,
  bool is_big_endian = false)
{
  if (
    (static_cast<size_t>(pack_length) <= sizeof(T) * CHAR_BIT) &&
    (static_cast<size_t>(pack_length) <= (sizeof(buffer) * CHAR_BIT) - bit_offset))
  {
    const uint64_t & buffer_bits = *reinterpret_cast<const uint64_t *>(buffer.data());
    uint64_t mask = ((1ULL << pack_length) - 1);

    uint64_t temp_data = (buffer_bits >> bit_offset) & mask;

    if constexpr (is_signed_integral<T>()) {
      // Sign-extend the extracted bits
      uint64_t sign_bit = 1ULL << (pack_length - 1);
      if (temp_data & sign_bit) {
        temp_data |= ~((1ULL << pack_length) - 1);
      }
    }

    data = *reinterpret_cast<T *>(&temp_data);

    if (is_big_endian) {
      byteswap(reinterpret_cast<unsigned char *>(&data), sizeof(T));
    }
    return true;
  } else {
    return false;
  }
}

template <typename T>
T unpackData(
  const std::array<unsigned char, 8> & buffer, uint8_t bit_offset, uint8_t pack_length, bool is_big_endian = false)
{
  T return_value;
  unpackData(return_value, buffer, bit_offset, pack_length, is_big_endian);
  return return_value;
}

}  // namespace polymath::sygnal

#endif  // MVEC_LIB__CAN_BITWORK_HPP_

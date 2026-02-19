#pragma once

#include <cstdint>
#include <cstring>

namespace LibXR::FOC
{

// Public FOC math/IO data carriers.
struct PhaseCurrentABC
{
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;
};

struct AlphaBeta
{
  float alpha = 0.0f;
  float beta = 0.0f;
};

struct DQ
{
  float d = 0.0f;
  float q = 0.0f;
};

struct DutyUVW
{
  float u = 0.5f;
  float v = 0.5f;
  float w = 0.5f;
};

namespace detail
{

// Internal fast inverse approximations used by hot paths.
inline uint32_t float_to_u32(float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

inline float u32_to_float(uint32_t bits)
{
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline float inv_sqrt_approx_unchecked(float value)
{
  constexpr uint32_t K_INV_SQRT_MAGIC = 0x5F375A86u;
  const float HALF_VALUE = 0.5f * value;
  uint32_t bits = float_to_u32(value);
  bits = K_INV_SQRT_MAGIC - (bits >> 1U);
  float estimate = u32_to_float(bits);
  estimate = estimate * (1.5f - HALF_VALUE * estimate * estimate);
  return estimate;
}

inline float reciprocal_approx_unchecked(float value)
{
  constexpr uint32_t K_RECIP_MAGIC = 0x7EF311C3u;
  uint32_t bits = float_to_u32(value);
  bits = K_RECIP_MAGIC - bits;
  float estimate = u32_to_float(bits);
  estimate = estimate * (2.0f - value * estimate);
  return estimate;
}

}  // namespace detail

}  // namespace LibXR::FOC

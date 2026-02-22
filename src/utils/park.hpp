#pragma once

#include <cmath>

#if defined(__has_include)
#if __has_include("arm_math.h")
#include "arm_math.h"
#define XRFOC_HAS_CMSIS_TRIG 1
#endif
#endif

#include "foc_defs.hpp"

namespace LibXR::FOC
{

inline void sin_cos(float electrical_angle, float& sin_theta, float& cos_theta)
{
#if defined(XRFOC_HAS_CMSIS_TRIG)
  arm_sin_cos_f32(electrical_angle, &sin_theta, &cos_theta);
#elif defined(__has_builtin)
#if __has_builtin(__builtin_sincosf)
  __builtin_sincosf(electrical_angle, &sin_theta, &cos_theta);
#else
  sin_theta = std::sinf(electrical_angle);
  cos_theta = std::cosf(electrical_angle);
#endif
#elif defined(__GNUC__)
  __builtin_sincosf(electrical_angle, &sin_theta, &cos_theta);
#else
  sin_theta = std::sinf(electrical_angle);
  cos_theta = std::cosf(electrical_angle);
#endif
}

inline DQ park(const AlphaBeta& ab, float sin_theta, float cos_theta)
{
  return {ab.alpha * cos_theta + ab.beta * sin_theta,
          -ab.alpha * sin_theta + ab.beta * cos_theta};
}

inline DQ park(const AlphaBeta& ab, float electrical_angle)
{
  float sin_theta = 0.0f;
  float cos_theta = 0.0f;
  sin_cos(electrical_angle, sin_theta, cos_theta);
  return park(ab, sin_theta, cos_theta);
}

inline AlphaBeta inverse_park(const DQ& dq, float sin_theta, float cos_theta)
{
  return {dq.d * cos_theta - dq.q * sin_theta,
          dq.d * sin_theta + dq.q * cos_theta};
}

inline AlphaBeta inverse_park(const DQ& dq, float electrical_angle)
{
  float sin_theta = 0.0f;
  float cos_theta = 0.0f;
  sin_cos(electrical_angle, sin_theta, cos_theta);
  return inverse_park(dq, sin_theta, cos_theta);
}

}  // namespace LibXR::FOC

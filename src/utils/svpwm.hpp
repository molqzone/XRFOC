#pragma once

#include "foc_defs.hpp"

namespace LibXR::FOC
{

inline float inv_bus_for_svpwm(float bus_voltage)
{
  return detail::reciprocal_approx_unchecked(bus_voltage);
}

inline float max3_unchecked(float a, float b, float c)
{
  return __builtin_fmaxf(__builtin_fmaxf(a, b), c);
}

inline float min3_unchecked(float a, float b, float c)
{
  return __builtin_fminf(__builtin_fminf(a, b), c);
}

inline DutyUVW space_vector_modulation(const AlphaBeta& voltage_ab, float bus_voltage)
{
  if (bus_voltage <= 1e-6f)
  {
    return {};
  }

  constexpr float HALF = 0.5f;
  constexpr float SQRT3_OVER_2 = 0.8660254037844386f;

  const float VA = voltage_ab.alpha;
  const float VB = -HALF * voltage_ab.alpha + SQRT3_OVER_2 * voltage_ab.beta;
  const float VC = -HALF * voltage_ab.alpha - SQRT3_OVER_2 * voltage_ab.beta;

  const float VMAX = max3_unchecked(VA, VB, VC);
  const float VMIN = min3_unchecked(VA, VB, VC);
  const float VOFFSET = HALF * (VMAX + VMIN);

  const float INV_BUS = inv_bus_for_svpwm(bus_voltage);
  const float DU = HALF + (VA - VOFFSET) * INV_BUS;
  const float DV = HALF + (VB - VOFFSET) * INV_BUS;
  const float DW = HALF + (VC - VOFFSET) * INV_BUS;

  return {DU, DV, DW};
}

}  // namespace LibXR::FOC

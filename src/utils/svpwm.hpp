#pragma once

#include <cmath>

#include "../driver/foc/foc_types.hpp"

namespace LibXR::FOC
{

inline DutyUVW SpaceVectorModulation(const AlphaBeta& voltage_ab, float bus_voltage)
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

  const float VMAX = (VA > VB) ? ((VA > VC) ? VA : VC) : ((VB > VC) ? VB : VC);
  const float VMIN = (VA < VB) ? ((VA < VC) ? VA : VC) : ((VB < VC) ? VB : VC);
  const float VOFFSET = HALF * (VMAX + VMIN);

  const float INV_BUS = 1.0f / bus_voltage;
  const float DU = HALF + (VA - VOFFSET) * INV_BUS;
  const float DV = HALF + (VB - VOFFSET) * INV_BUS;
  const float DW = HALF + (VC - VOFFSET) * INV_BUS;

  return {DU, DV, DW};
}

}  // namespace LibXR::FOC

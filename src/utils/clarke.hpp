#pragma once

#include "foc_types.hpp"

namespace LibXR::FOC
{

inline AlphaBeta clarke(const PhaseCurrentABC& abc)
{
  constexpr float INV_SQRT3 = 0.5773502691896258f;  // 1/sqrt(3)
  return {abc.a, (abc.a + 2.0f * abc.b) * INV_SQRT3};
}

inline PhaseCurrentABC inverse_clarke(const AlphaBeta& ab)
{
  constexpr float HALF = 0.5f;
  constexpr float SQRT3_OVER_2 = 0.8660254037844386f;  // sqrt(3)/2

  const float IA = ab.alpha;
  const float IB = -HALF * ab.alpha + SQRT3_OVER_2 * ab.beta;
  const float IC = -HALF * ab.alpha - SQRT3_OVER_2 * ab.beta;
  return {IA, IB, IC};
}

}  // namespace LibXR::FOC

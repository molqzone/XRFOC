#pragma once

#include <cstdint>

#include "foc_defs.hpp"
#include "utils/park.hpp"

namespace LibXR::FOC
{

class DumbCurrent
{
 public:
  using SampleType = PhaseCurrentABC;

  DumbCurrent() { UpdateDeltaTrig(); }

  [[nodiscard]] SampleType Read()
  {
    AdvancePhase();

    constexpr float HALF = 0.5f;
    constexpr float SQRT3_OVER_2 = 0.8660254037844386f;
    const float PHASE_A = amplitude_ * sin_phase_;
    const float PHASE_B = amplitude_ * (-HALF * sin_phase_ - SQRT3_OVER_2 * cos_phase_);
    const float PHASE_C = amplitude_ * (-HALF * sin_phase_ + SQRT3_OVER_2 * cos_phase_);
    return {PHASE_A, PHASE_B, PHASE_C};
  }

  void SetPhase(float phase)
  {
    phase_ = NormalizeAngle(phase);
    sin_cos(phase_, sin_phase_, cos_phase_);
  }

  [[nodiscard]] float GetPhase() const { return phase_; }

  void SetDelta(float delta)
  {
    delta_ = delta;
    UpdateDeltaTrig();
  }

  [[nodiscard]] float GetDelta() const { return delta_; }

  void SetAmplitude(float amplitude) { amplitude_ = amplitude; }

  [[nodiscard]] float GetAmplitude() const { return amplitude_; }

 private:
  void AdvancePhase()
  {
    constexpr float TWO_PI = 6.28318530717958647692f;
    phase_ += delta_;
    if (phase_ >= TWO_PI)
    {
      phase_ -= TWO_PI;
    }
    else if (phase_ < 0.0f)
    {
      phase_ += TWO_PI;
    }

    const float PREV_SIN = sin_phase_;
    const float PREV_COS = cos_phase_;
    sin_phase_ = PREV_SIN * cos_delta_ + PREV_COS * sin_delta_;
    cos_phase_ = PREV_COS * cos_delta_ - PREV_SIN * sin_delta_;

    ++step_counter_;
    if ((step_counter_ & 0xFFU) == 0U)
    {
      const float NORM_SQ = sin_phase_ * sin_phase_ + cos_phase_ * cos_phase_;
      const float INV_NORM = detail::inv_sqrt_approx_unchecked(NORM_SQ);
      sin_phase_ *= INV_NORM;
      cos_phase_ *= INV_NORM;
    }
  }

  void UpdateDeltaTrig() { sin_cos(delta_, sin_delta_, cos_delta_); }

  static float NormalizeAngle(float angle)
  {
    constexpr float TWO_PI = 6.28318530717958647692f;
    constexpr float INV_TWO_PI = 0.15915494309189533577f;  // 1 / (2*pi)
    const int32_t REV = static_cast<int32_t>(angle * INV_TWO_PI);
    angle -= static_cast<float>(REV) * TWO_PI;
    if (angle >= TWO_PI)
    {
      angle -= TWO_PI;
    }
    if (angle < 0.0f)
    {
      angle += TWO_PI;
    }
    return angle;
  }

  float phase_ = 0.0f;
  float delta_ = 0.00042f;
  float amplitude_ = 0.8f;
  float sin_phase_ = 0.0f;
  float cos_phase_ = 1.0f;
  float sin_delta_ = 0.0f;
  float cos_delta_ = 1.0f;
  uint32_t step_counter_ = 0U;
};

}  // namespace LibXR::FOC

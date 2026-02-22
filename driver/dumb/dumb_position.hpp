#pragma once

#include <cstdint>

#include "foc_defs.hpp"
#include "utils/park.hpp"

namespace LibXR::FOC
{

class DumbPosition
{
 public:
  using SampleType = float;

  struct TrigSample
  {
    float sin = 0.0f;
    float cos = 1.0f;
  };

  DumbPosition() { UpdateDeltaTrig(); }

  [[nodiscard]] SampleType Read()
  {
    constexpr float TWO_PI = 6.28318530717958647692f;
    angle_ += delta_;
    if (angle_ >= TWO_PI)
    {
      angle_ -= TWO_PI;
    }
    else if (angle_ < 0.0f)
    {
      angle_ += TWO_PI;
    }

    const float PREV_SIN = sin_angle_;
    const float PREV_COS = cos_angle_;
    sin_angle_ = PREV_SIN * cos_delta_ + PREV_COS * sin_delta_;
    cos_angle_ = PREV_COS * cos_delta_ - PREV_SIN * sin_delta_;

    ++step_counter_;
    if ((step_counter_ & 0x1FFU) == 0U)
    {
      const float NORM_SQ = sin_angle_ * sin_angle_ + cos_angle_ * cos_angle_;
      const float INV_NORM = detail::inv_sqrt_approx_unchecked(NORM_SQ);
      sin_angle_ *= INV_NORM;
      cos_angle_ *= INV_NORM;
    }
    return angle_;
  }

  void SetAngle(float angle)
  {
    angle_ = NormalizeAngle(angle);
    sin_cos(angle_, sin_angle_, cos_angle_);
  }

  [[nodiscard]] float GetAngle() const { return angle_; }

  void SetDelta(float delta)
  {
    delta_ = delta;
    UpdateDeltaTrig();
  }

  [[nodiscard]] float GetDelta() const { return delta_; }

  [[nodiscard]] TrigSample SinCosNoConfig(float) const { return {sin_angle_, cos_angle_}; }

 private:
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

  float angle_ = 0.0f;
  float delta_ = 0.00035f;
  float sin_angle_ = 0.0f;
  float cos_angle_ = 1.0f;
  float sin_delta_ = 0.0f;
  float cos_delta_ = 1.0f;
  uint32_t step_counter_ = 0U;
};

}  // namespace LibXR::FOC

#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>

#include "../../utils/clarke.hpp"
#include "../../utils/park.hpp"
#include "../../utils/svpwm.hpp"
#include "foc_types.hpp"

namespace LibXR::FOC
{

template <typename PositionType, typename CurrentType, typename InverterType>
class Controller
{
 public:
  using PositionSampleType = typename PositionType::SampleType;
  using CurrentSampleType = typename CurrentType::SampleType;

  struct PIConfig
  {
    float kp = 0.0f;
    float ki = 0.0f;
    float integral_limit = 0.0f;
    float output_limit = 0.0f;
  };

  struct Configuration
  {
    float pole_pairs = 1.0f;
    float electrical_offset = 0.0f;
    float output_voltage_limit = 0.0f;  // <=0 means auto from bus voltage
    PIConfig d_axis = {};
    PIConfig q_axis = {};
  };

  struct StepResult
  {
    float electrical_angle = 0.0f;
    float dt = 0.0f;

    PositionSampleType position = {};
    CurrentSampleType current = {};
    DQ target_current_dq = {};
    PhaseCurrentABC current_abc = {};
    AlphaBeta current_ab = {};
    DQ current_dq = {};
    DQ voltage_dq = {};
    AlphaBeta voltage_ab = {};
    DutyUVW duty = {};
  };

  Controller(PositionType& position, CurrentType& current, InverterType& inverter)
      : position_(position), current_(current), inverter_(inverter)
  {
    static_assert(std::is_convertible<PositionSampleType, float>::value,
                  "Position::SampleType must be convertible to float angle.");
    static_assert(std::is_convertible<CurrentSampleType, PhaseCurrentABC>::value,
                  "Current::SampleType must be convertible to PhaseCurrentABC.");
  }

  void SetConfig(const Configuration& config) { config_ = config; }
  [[nodiscard]] const Configuration& GetConfig() const { return config_; }

  void SetTargetCurrent(DQ target_current_dq) { target_current_dq_ = target_current_dq; }
  [[nodiscard]] DQ GetTargetCurrent() const { return target_current_dq_; }

  void SetTargetIq(float iq) { target_current_dq_.q = iq; }
  void SetTargetId(float id) { target_current_dq_.d = id; }

  void SetPolePairs(float pole_pairs) { config_.pole_pairs = pole_pairs; }
  void SetElectricalOffset(float electrical_offset)
  {
    config_.electrical_offset = electrical_offset;
  }

  void ResetIntegral()
  {
    d_integral_ = 0.0f;
    q_integral_ = 0.0f;
  }

  decltype(auto) EnablePowerStage(bool in_isr) { return inverter_.Enable(in_isr); }

  decltype(auto) DisablePowerStage(bool in_isr) { return inverter_.Disable(in_isr); }

  decltype(auto) Step(float dt, bool in_isr)
  {
    return inverter_.SetDuty(CoreStepImpl<false>(dt, nullptr), in_isr);
  }

  decltype(auto) StepInto(float dt, bool in_isr, StepResult& out)
  {
    return inverter_.SetDuty(CoreStepImpl<true>(dt, &out), in_isr);
  }

 private:
  static float ClampSymmetric(float value, float limit)
  {
    if (limit < 0.0f)
    {
      limit = 0.0f;
    }
    if (value > limit)
    {
      return limit;
    }
    if (value < -limit)
    {
      return -limit;
    }
    return value;
  }

  static float NormalizeAngle(float angle)
  {
    constexpr float two_pi = 6.28318530717958647692f;
    constexpr float inv_two_pi = 0.15915494309189533577f;  // 1 / (2*pi)
    const int32_t rev = static_cast<int32_t>(angle * inv_two_pi);
    angle -= static_cast<float>(rev) * two_pi;
    if (angle < 0.0f)
    {
      angle += two_pi;
    }
    else if (angle >= two_pi)
    {
      angle -= two_pi;
    }
    return angle;
  }

  static float RunCurrentPI(float target, float measured, float dt, float& integral,
                            const PIConfig& cfg)
  {
    const float error = target - measured;
    const float p_term = cfg.kp * error;
    const float i_candidate =
        ClampSymmetric(integral + cfg.ki * error * dt, cfg.integral_limit);
    const float unsat_output = p_term + i_candidate;
    const float sat_output = ClampSymmetric(unsat_output, cfg.output_limit);
    const float sat_delta = sat_output - unsat_output;
    if (sat_delta * error >= 0.0f)
    {
      integral = i_candidate;
    }

    return sat_output;
  }

  static DQ LimitVoltageVector(DQ v_dq, float limit)
  {
    if (limit <= 0.0f)
    {
      return v_dq;
    }

    const float mag_sq = v_dq.d * v_dq.d + v_dq.q * v_dq.q;
    const float limit_sq = limit * limit;
    if (mag_sq <= limit_sq || mag_sq <= 1e-12f)
    {
      return v_dq;
    }

    const float scale = limit / std::sqrt(mag_sq);
    return {v_dq.d * scale, v_dq.q * scale};
  }

  template <bool CaptureResult>
  DutyUVW CoreStepImpl(float dt, StepResult* out)
  {
    const PositionSampleType position_sample = position_.Read();
    const CurrentSampleType current_sample = current_.Read();

    const float position_angle = static_cast<float>(position_sample);
    const float electrical_angle =
        NormalizeAngle(position_angle * config_.pole_pairs + config_.electrical_offset);
    float sin_theta = 0.0f;
    float cos_theta = 0.0f;
    SinCos(electrical_angle, sin_theta, cos_theta);
    const float bus_voltage = inverter_.BusVoltage();

    const PhaseCurrentABC current_abc = static_cast<PhaseCurrentABC>(current_sample);
    const AlphaBeta current_ab = Clarke(current_abc);
    const DQ current_dq = Park(current_ab, sin_theta, cos_theta);

    DQ voltage_dq = {};
    voltage_dq.d =
        RunCurrentPI(target_current_dq_.d, current_dq.d, dt, d_integral_, config_.d_axis);
    voltage_dq.q =
        RunCurrentPI(target_current_dq_.q, current_dq.q, dt, q_integral_, config_.q_axis);
    constexpr float inv_sqrt3 = 0.5773502691896258f;
    const float voltage_limit =
        (config_.output_voltage_limit > 0.0f)
            ? config_.output_voltage_limit
            : ((bus_voltage > 0.0f) ? (bus_voltage * inv_sqrt3) : 0.0f);
    voltage_dq = LimitVoltageVector(voltage_dq, voltage_limit);

    const AlphaBeta voltage_ab = InversePark(voltage_dq, sin_theta, cos_theta);
    const DutyUVW duty = SpaceVectorModulation(voltage_ab, bus_voltage);

    if constexpr (CaptureResult)
    {
      out->dt = dt;
      out->electrical_angle = electrical_angle;
      out->position = position_sample;
      out->current = current_sample;
      out->target_current_dq = target_current_dq_;
      out->current_abc = current_abc;
      out->current_ab = current_ab;
      out->current_dq = current_dq;
      out->voltage_dq = voltage_dq;
      out->voltage_ab = voltage_ab;
      out->duty = duty;
    }

    return duty;
  }

  PositionType& position_;
  CurrentType& current_;
  InverterType& inverter_;
  Configuration config_ = {};
  DQ target_current_dq_ = {};
  float d_integral_ = 0.0f;
  float q_integral_ = 0.0f;
};

}  // namespace LibXR::FOC

#pragma once

#include <cmath>
#include <type_traits>

#include "foc_types.hpp"
#include "../../utils/clarke.hpp"
#include "../../utils/park.hpp"
#include "../../utils/svpwm.hpp"

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
      : position_(position),
        current_(current),
        inverter_(inverter)
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

  decltype(auto) EnablePowerStage(bool in_isr)
  {
    return inverter_.Enable(in_isr);
  }

  decltype(auto) DisablePowerStage(bool in_isr)
  {
    return inverter_.Disable(in_isr);
  }

  decltype(auto) Step(float dt, bool in_isr)
  {
    DutyUVW duty = CoreStep(dt, nullptr);
    return inverter_.SetDuty(duty, in_isr);
  }

  decltype(auto) StepInto(float dt, bool in_isr, StepResult& out)
  {
    DutyUVW duty = CoreStep(dt, &out);
    return inverter_.SetDuty(duty, in_isr);
  }

 private:
  static float ClampSymmetric(float value, float limit)
  {
    if (limit < 0.0f)
    {
      limit = 0.0f;
    }
    return std::fmax(-limit, std::fmin(limit, value));
  }

  static float NormalizeAngle(float angle)
  {
    constexpr float TWO_PI = 6.28318530717958647692f;
    if (angle >= TWO_PI)
    {
      do
      {
        angle -= TWO_PI;
      } while (angle >= TWO_PI);
    }
    else if (angle < 0.0f)
    {
      do
      {
        angle += TWO_PI;
      } while (angle < 0.0f);
    }
    return angle;
  }

  static float RunCurrentPI(float target, float measured, float dt, float& integral,
                            const PIConfig& cfg)
  {
    const float ERROR = target - measured;
    const float P_TERM = cfg.kp * ERROR;
    const float I_CANDIDATE =
        ClampSymmetric(integral + cfg.ki * ERROR * dt, cfg.integral_limit);
    const float UNSAT_OUTPUT = P_TERM + I_CANDIDATE;
    const float SAT_OUTPUT = ClampSymmetric(UNSAT_OUTPUT, cfg.output_limit);

    const bool SATURATING_HIGH = UNSAT_OUTPUT > SAT_OUTPUT;
    const bool SATURATING_LOW = UNSAT_OUTPUT < SAT_OUTPUT;
    if ((!SATURATING_HIGH && !SATURATING_LOW) ||
        (SATURATING_HIGH && ERROR < 0.0f) || (SATURATING_LOW && ERROR > 0.0f))
    {
      integral = I_CANDIDATE;
    }

    return SAT_OUTPUT;
  }

  static DQ LimitVoltageVector(DQ v_dq, float limit)
  {
    if (limit <= 0.0f)
    {
      return v_dq;
    }

    const float MAG_SQ = v_dq.d * v_dq.d + v_dq.q * v_dq.q;
    const float LIMIT_SQ = limit * limit;
    if (MAG_SQ <= LIMIT_SQ || MAG_SQ <= 1e-12f)
    {
      return v_dq;
    }

    const float SCALE = limit / std::sqrt(MAG_SQ);
    return {v_dq.d * SCALE, v_dq.q * SCALE};
  }

  float ResolveVoltageLimit() const
  {
    if (config_.output_voltage_limit > 0.0f)
    {
      return config_.output_voltage_limit;
    }

    constexpr float INV_SQRT3 = 0.5773502691896258f;
    const float VBUS = inverter_.BusVoltage();
    return (VBUS > 0.0f) ? (VBUS * INV_SQRT3) : 0.0f;
  }

  DutyUVW CoreStep(float dt, StepResult* out)
  {
    const PositionSampleType POSITION_SAMPLE = position_.Read();
    const CurrentSampleType CURRENT_SAMPLE = current_.Read();

    const float POSITION_ANGLE = static_cast<float>(POSITION_SAMPLE);
    const float ELECTRICAL_ANGLE =
        NormalizeAngle(POSITION_ANGLE * config_.pole_pairs + config_.electrical_offset);

    const PhaseCurrentABC CURRENT_ABC = static_cast<PhaseCurrentABC>(CURRENT_SAMPLE);
    const AlphaBeta CURRENT_AB = Clarke(CURRENT_ABC);
    const DQ CURRENT_DQ = Park(CURRENT_AB, ELECTRICAL_ANGLE);

    DQ voltage_dq = {};
    voltage_dq.d = RunCurrentPI(target_current_dq_.d, CURRENT_DQ.d, dt, d_integral_,
                                config_.d_axis);
    voltage_dq.q = RunCurrentPI(target_current_dq_.q, CURRENT_DQ.q, dt, q_integral_,
                                config_.q_axis);
    voltage_dq = LimitVoltageVector(voltage_dq, ResolveVoltageLimit());

    const AlphaBeta VOLTAGE_AB = InversePark(voltage_dq, ELECTRICAL_ANGLE);
    const DutyUVW DUTY = SpaceVectorModulation(VOLTAGE_AB, inverter_.BusVoltage());

    if (out != nullptr)
    {
      out->dt = dt;
      out->electrical_angle = ELECTRICAL_ANGLE;
      out->position = POSITION_SAMPLE;
      out->current = CURRENT_SAMPLE;
      out->target_current_dq = target_current_dq_;
      out->current_abc = CURRENT_ABC;
      out->current_ab = CURRENT_AB;
      out->current_dq = CURRENT_DQ;
      out->voltage_dq = voltage_dq;
      out->voltage_ab = VOLTAGE_AB;
      out->duty = DUTY;
    }

    return DUTY;
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

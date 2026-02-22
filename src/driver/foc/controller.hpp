#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "foc_defs.hpp"
#include "utils/clarke.hpp"
#include "utils/park.hpp"
#include "utils/svpwm.hpp"

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

  void SetConfig(const Configuration& config)
  {
    config_ = config;
    SanitizeConfig(config_);
  }
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
    return inverter_.SetDuty(
        CoreStepImpl<false, true>(
            dt, nullptr,
            [](float electrical_angle, float& sin_theta, float& cos_theta)
            { sin_cos(electrical_angle, sin_theta, cos_theta); }),
        in_isr);
  }

  decltype(auto) StepInto(float dt, bool in_isr, StepResult& out)
  {
    return inverter_.SetDuty(
        CoreStepImpl<true, true>(
            dt, &out,
            [](float electrical_angle, float& sin_theta, float& cos_theta)
            { sin_cos(electrical_angle, sin_theta, cos_theta); }),
        in_isr);
  }

  template <typename TrigProvider>
  decltype(auto) StepWithTrig(float dt, bool in_isr, const TrigProvider& trig)
  {
    return inverter_.SetDuty(
        CoreStepImpl<false, true>(
            dt, nullptr,
            [&trig](float electrical_angle, float& sin_theta, float& cos_theta)
            {
              const auto TRIG_PAIR = trig.SinCos(electrical_angle);
              sin_theta = TRIG_PAIR.sin;
              cos_theta = TRIG_PAIR.cos;
            }),
        in_isr);
  }

  template <typename TrigProvider>
  decltype(auto) StepWithTrigNoConfig(float dt, bool in_isr, const TrigProvider& trig)
  {
    return inverter_.SetDuty(
        CoreStepImpl<false, true>(
            dt, nullptr,
            [&trig](float electrical_angle, float& sin_theta, float& cos_theta)
            {
              SampleTrigNoConfigNormalized(trig, electrical_angle, sin_theta, cos_theta);
            }),
        in_isr);
  }

  template <typename TrigProvider>
  decltype(auto) StepWithTrigInto(float dt, bool in_isr, const TrigProvider& trig,
                                  StepResult& out)
  {
    return inverter_.SetDuty(
        CoreStepImpl<true, true>(
            dt, &out,
            [&trig](float electrical_angle, float& sin_theta, float& cos_theta)
            {
              const auto TRIG_PAIR = trig.SinCos(electrical_angle);
              sin_theta = TRIG_PAIR.sin;
              cos_theta = TRIG_PAIR.cos;
            }),
        in_isr);
  }

  template <typename TrigProvider>
  decltype(auto) StepWithTrigIntoNoConfig(float dt, bool in_isr, const TrigProvider& trig,
                                          StepResult& out)
  {
    return inverter_.SetDuty(
        CoreStepImpl<true, true>(
            dt, &out,
            [&trig](float electrical_angle, float& sin_theta, float& cos_theta)
            {
              SampleTrigNoConfigNormalized(trig, electrical_angle, sin_theta, cos_theta);
            }),
        in_isr);
  }

 private:
  static float ClampNonNegative(float value) { return (value >= 0.0f) ? value : 0.0f; }

  static void SanitizePIConfig(PIConfig& cfg)
  {
    cfg.integral_limit = ClampNonNegative(cfg.integral_limit);
    cfg.output_limit = ClampNonNegative(cfg.output_limit);
  }

  static void SanitizeConfig(Configuration& cfg)
  {
    SanitizePIConfig(cfg.d_axis);
    SanitizePIConfig(cfg.q_axis);
  }

  static float ClampSymmetric(float value, float limit)
  {
    const float NEG_LIMIT = -limit;
    return __builtin_fminf(__builtin_fmaxf(value, NEG_LIMIT), limit);
  }

  static float NormalizeAngle(float angle)
  {
    constexpr float TWO_PI = 6.28318530717958647692f;
    if (angle >= 0.0f && angle < TWO_PI)
    {
      return angle;
    }
    constexpr float INV_TWO_PI = 0.15915494309189533577f;  // 1 / (2*pi)
    const int32_t REV = static_cast<int32_t>(angle * INV_TWO_PI);
    angle -= static_cast<float>(REV) * TWO_PI;
    if (angle < 0.0f)
    {
      angle += TWO_PI;
    }
    return angle;
  }

  template <typename TrigProvider, typename = void>
  struct HasSinCosFastNoConfig : std::false_type
  {
  };

  template <typename TrigProvider>
  struct HasSinCosFastNoConfig<
      TrigProvider,
      std::void_t<decltype(std::declval<const TrigProvider&>().SinCosFastNoConfig(0.0f))>>
      : std::true_type
  {
  };

  template <typename TrigProvider>
  static void SampleTrigNoConfigNormalized(const TrigProvider& trig,
                                           float normalized_electrical_angle,
                                           float& sin_theta, float& cos_theta)
  {
    if constexpr (HasSinCosFastNoConfig<TrigProvider>::value)
    {
      constexpr float K_PI = 3.14159265358979323846f;
      constexpr float TWO_PI = 6.28318530717958647692f;
      const float SIGNED_ANGLE = (normalized_electrical_angle >= K_PI)
                                     ? (normalized_electrical_angle - TWO_PI)
                                     : normalized_electrical_angle;
      const auto TRIG_PAIR = trig.SinCosFastNoConfig(SIGNED_ANGLE);
      sin_theta = TRIG_PAIR.sin;
      cos_theta = TRIG_PAIR.cos;
    }
    else
    {
      const auto TRIG_PAIR = trig.SinCosNoConfig(normalized_electrical_angle);
      sin_theta = TRIG_PAIR.sin;
      cos_theta = TRIG_PAIR.cos;
    }
  }

  static float RunCurrentPI(float target, float measured, float ki_dt, float& integral,
                            const PIConfig& cfg)
  {
    if (cfg.kp == 0.0f && cfg.ki == 0.0f)
    {
      integral = 0.0f;
      return 0.0f;
    }

    const float ERROR = target - measured;
    if (cfg.integral_limit == 0.0f || ki_dt == 0.0f)
    {
      integral = 0.0f;
      const float P_ONLY_OUTPUT = cfg.kp * ERROR;
      return ClampSymmetric(P_ONLY_OUTPUT, cfg.output_limit);
    }

    const float P_TERM = cfg.kp * ERROR;
    const float I_CANDIDATE =
        ClampSymmetric(integral + ki_dt * ERROR, cfg.integral_limit);
    const float UNSAT_OUTPUT = P_TERM + I_CANDIDATE;
    const float SAT_OUTPUT = ClampSymmetric(UNSAT_OUTPUT, cfg.output_limit);
    const float SAT_DELTA = SAT_OUTPUT - UNSAT_OUTPUT;
    if (SAT_DELTA * ERROR >= 0.0f)
    {
      integral = I_CANDIDATE;
    }

    return SAT_OUTPUT;
  }

  static float ScaleForVoltageLimit(float limit, float magnitude_sq)
  {
    return limit * detail::inv_sqrt_approx_unchecked(magnitude_sq);
  }

  static DQ LimitVoltageVector(DQ v_dq, float limit)
  {
    const float MAG_SQ = v_dq.d * v_dq.d + v_dq.q * v_dq.q;
    const float LIMIT_SQ = limit * limit;
    if (MAG_SQ <= LIMIT_SQ)
    {
      return v_dq;
    }

    const float SCALE = ScaleForVoltageLimit(limit, MAG_SQ);
    return {v_dq.d * SCALE, v_dq.q * SCALE};
  }

  template <bool CaptureResult, bool NormalizeForTrig, typename SinCosFn>
  DutyUVW CoreStepImpl(float dt, StepResult* out, SinCosFn&& sin_cos_fn)
  {
    const PositionSampleType POSITION_SAMPLE = position_.Read();
    const CurrentSampleType CURRENT_SAMPLE = current_.Read();

    const float POSITION_ANGLE = static_cast<float>(POSITION_SAMPLE);
    const float RAW_ELECTRICAL_ANGLE =
        POSITION_ANGLE * config_.pole_pairs + config_.electrical_offset;
    float electrical_angle = RAW_ELECTRICAL_ANGLE;
    if constexpr (NormalizeForTrig || CaptureResult)
    {
      electrical_angle = NormalizeAngle(RAW_ELECTRICAL_ANGLE);
    }

    const float TRIG_ANGLE = NormalizeForTrig ? electrical_angle : RAW_ELECTRICAL_ANGLE;
    float sin_theta = 0.0f;
    float cos_theta = 0.0f;
    std::forward<SinCosFn>(sin_cos_fn)(TRIG_ANGLE, sin_theta, cos_theta);
    const float BUS_VOLTAGE = inverter_.BusVoltage();

    const PhaseCurrentABC CURRENT_ABC = static_cast<PhaseCurrentABC>(CURRENT_SAMPLE);
    const AlphaBeta CURRENT_AB = clarke(CURRENT_ABC);
    const DQ CURRENT_DQ = park(CURRENT_AB, sin_theta, cos_theta);

    const float D_KI_DT = config_.d_axis.ki * dt;
    const float Q_KI_DT = config_.q_axis.ki * dt;
    DQ voltage_dq = {};
    voltage_dq.d = RunCurrentPI(target_current_dq_.d, CURRENT_DQ.d, D_KI_DT, d_integral_,
                                config_.d_axis);
    voltage_dq.q = RunCurrentPI(target_current_dq_.q, CURRENT_DQ.q, Q_KI_DT, q_integral_,
                                config_.q_axis);
    constexpr float INV_SQRT3 = 0.5773502691896258f;
    const float AUTO_VOLTAGE_LIMIT = BUS_VOLTAGE * INV_SQRT3;
    const float VOLTAGE_LIMIT =
        (config_.output_voltage_limit > 0.0f)
            ? config_.output_voltage_limit
            : AUTO_VOLTAGE_LIMIT;
    voltage_dq = LimitVoltageVector(voltage_dq, VOLTAGE_LIMIT);

    const AlphaBeta VOLTAGE_AB = inverse_park(voltage_dq, sin_theta, cos_theta);
    const DutyUVW DUTY = space_vector_modulation(VOLTAGE_AB, BUS_VOLTAGE);

    if constexpr (CaptureResult)
    {
      out->dt = dt;
      out->electrical_angle = electrical_angle;
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

#pragma once

#include <cstdint>

#include "stm32g4xx_hal.h"

#ifdef HAL_TIM_MODULE_ENABLED

#include "foc_types.hpp"
#include "libxr_def.hpp"
#include "stm32g4xx_ll_tim.h"

namespace LibXR::FOC
{

class STM32Inverter
{
 public:
  struct Configuration
  {
    uint32_t frequency_hz = 20000;
    uint32_t timer_clock_hz = 0;  // 0 means auto-detect from RCC clock tree
    uint8_t deadtime = 0;         // direct TIMx_BDTR.DTG register value
    bool center_aligned = true;
    bool complementary_output = true;
  };

  STM32Inverter(TIM_HandleTypeDef* htim, float bus_voltage = 0.0f,
                uint32_t channel_u = TIM_CHANNEL_1, uint32_t channel_v = TIM_CHANNEL_2,
                uint32_t channel_w = TIM_CHANNEL_3)
      : htim_(htim),
        channel_u_(channel_u),
        channel_v_(channel_v),
        channel_w_(channel_w),
        bus_voltage_(bus_voltage)
  {
  }

  LibXR::ErrorCode Init(const Configuration& config)
  {
    if (htim_ == nullptr || config.frequency_hz == 0)
    {
      return LibXR::ErrorCode::ARG_ERR;
    }

    if (!IsSupportedTimerInstance(htim_->Instance))
    {
      return LibXR::ErrorCode::NOT_SUPPORT;
    }

    if (!IsValidChannel(channel_u_) || !IsValidChannel(channel_v_) ||
        !IsValidChannel(channel_w_))
    {
      return LibXR::ErrorCode::ARG_ERR;
    }

    if (channel_u_ == channel_v_ || channel_u_ == channel_w_ || channel_v_ == channel_w_)
    {
      return LibXR::ErrorCode::ARG_ERR;
    }

    ccr_u_ = ResolveCompareRegister(channel_u_);
    ccr_v_ = ResolveCompareRegister(channel_v_);
    ccr_w_ = ResolveCompareRegister(channel_w_);
    if (ccr_u_ == nullptr || ccr_v_ == nullptr || ccr_w_ == nullptr)
    {
      return LibXR::ErrorCode::ARG_ERR;
    }

    uint32_t timer_clock_hz = config.timer_clock_hz;
    if (timer_clock_hz == 0)
    {
      timer_clock_hz = ResolveTimerClock();
      if (timer_clock_hz == 0)
      {
        return LibXR::ErrorCode::INIT_ERR;
      }
    }

    uint16_t prescaler = 0;
    uint32_t period = 0;
    if (!ComputeTimerBase(timer_clock_hz, config.frequency_hz, config.center_aligned,
                          prescaler, period))
    {
      return LibXR::ErrorCode::INIT_ERR;
    }

    config_ = config;

    htim_->Init.Prescaler = prescaler;
    htim_->Init.Period = period;
    htim_->Init.CounterMode =
        config.center_aligned ? TIM_COUNTERMODE_CENTERALIGNED1 : TIM_COUNTERMODE_UP;
    htim_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(htim_) != HAL_OK)
    {
      return LibXR::ErrorCode::INIT_ERR;
    }

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
#ifdef TIM_OCNPOLARITY_HIGH
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
#endif
#ifdef TIM_OCIDLESTATE_RESET
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
#endif
#ifdef TIM_OCNIDLESTATE_RESET
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
#endif

    if (HAL_TIM_PWM_ConfigChannel(htim_, &oc, channel_u_) != HAL_OK)
    {
      return LibXR::ErrorCode::INIT_ERR;
    }
    if (HAL_TIM_PWM_ConfigChannel(htim_, &oc, channel_v_) != HAL_OK)
    {
      return LibXR::ErrorCode::INIT_ERR;
    }
    if (HAL_TIM_PWM_ConfigChannel(htim_, &oc, channel_w_) != HAL_OK)
    {
      return LibXR::ErrorCode::INIT_ERR;
    }

    LL_TIM_OC_EnablePreload(htim_->Instance, HalChannelToLLChannel(channel_u_));
    LL_TIM_OC_EnablePreload(htim_->Instance, HalChannelToLLChannel(channel_v_));
    LL_TIM_OC_EnablePreload(htim_->Instance, HalChannelToLLChannel(channel_w_));

#if defined(TIM_BDTR_MOE)
    if (config.complementary_output)
    {
      TIM_BreakDeadTimeConfigTypeDef deadtime_cfg = {};
      deadtime_cfg.OffStateRunMode = TIM_OSSR_DISABLE;
      deadtime_cfg.OffStateIDLEMode = TIM_OSSI_DISABLE;
      deadtime_cfg.LockLevel = TIM_LOCKLEVEL_OFF;
      deadtime_cfg.DeadTime = config.deadtime;
      deadtime_cfg.BreakState = TIM_BREAK_DISABLE;
#ifdef TIM_BREAK2_DISABLE
      deadtime_cfg.Break2State = TIM_BREAK2_DISABLE;
      deadtime_cfg.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
      deadtime_cfg.Break2Filter = 0;
#ifdef TIM_BREAK2_AFMODE_INPUT
      deadtime_cfg.Break2AFMode = TIM_BREAK2_AFMODE_INPUT;
#endif
#endif
      deadtime_cfg.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
      deadtime_cfg.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
      deadtime_cfg.BreakFilter = 0;
#ifdef TIM_BREAK_AFMODE_INPUT
      deadtime_cfg.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
#endif
      if (HAL_TIMEx_ConfigBreakDeadTime(htim_, &deadtime_cfg) != HAL_OK)
      {
        return LibXR::ErrorCode::INIT_ERR;
      }
    }
#endif

    LL_TIM_GenerateEvent_UPDATE(htim_->Instance);
    max_pulse_ = period;
    duty_to_pulse_scale_ = static_cast<float>(period + 1U);
    initialized_ = true;
    enabled_ = false;
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode SetConfig(const Configuration& config) { return Init(config); }

  [[nodiscard]] const Configuration& GetConfig() const { return config_; }

  LibXR::ErrorCode Enable(bool in_isr)
  {
    (void)in_isr;

    if (!initialized_)
    {
      return LibXR::ErrorCode::STATE_ERR;
    }

    if (enabled_)
    {
      return LibXR::ErrorCode::OK;
    }

    if (StartPhase(channel_u_) != LibXR::ErrorCode::OK ||
        StartPhase(channel_v_) != LibXR::ErrorCode::OK ||
        StartPhase(channel_w_) != LibXR::ErrorCode::OK)
    {
      (void)Disable(in_isr);
      return LibXR::ErrorCode::FAILED;
    }

#if defined(TIM_BDTR_MOE)
    LL_TIM_EnableAllOutputs(htim_->Instance);
#endif

    enabled_ = true;
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode Disable(bool in_isr)
  {
    (void)in_isr;

    if (!initialized_)
    {
      return LibXR::ErrorCode::STATE_ERR;
    }

    LibXR::ErrorCode status = LibXR::ErrorCode::OK;
    if (StopPhase(channel_u_) != LibXR::ErrorCode::OK)
    {
      status = LibXR::ErrorCode::FAILED;
    }
    if (StopPhase(channel_v_) != LibXR::ErrorCode::OK)
    {
      status = LibXR::ErrorCode::FAILED;
    }
    if (StopPhase(channel_w_) != LibXR::ErrorCode::OK)
    {
      status = LibXR::ErrorCode::FAILED;
    }

#if defined(TIM_BDTR_MOE)
    LL_TIM_DisableAllOutputs(htim_->Instance);
#endif

    enabled_ = false;
    return status;
  }

  LibXR::ErrorCode SetDuty(DutyUVW duty, bool in_isr)
  {
    (void)in_isr;

    if (!initialized_)
    {
      return LibXR::ErrorCode::STATE_ERR;
    }

    const uint32_t PULSE_U = DutyToPulse(duty.u);
    const uint32_t PULSE_V = DutyToPulse(duty.v);
    const uint32_t PULSE_W = DutyToPulse(duty.w);

    *ccr_u_ = PULSE_U;
    *ccr_v_ = PULSE_V;
    *ccr_w_ = PULSE_W;

    return LibXR::ErrorCode::OK;
  }

  void SetBusVoltage(float bus_voltage) { bus_voltage_ = bus_voltage; }

  [[nodiscard]] float BusVoltage() const { return bus_voltage_; }

  [[nodiscard]] TIM_HandleTypeDef* TimerHandle() { return htim_; }

 private:
  static uint32_t HalChannelToLLChannel(uint32_t channel)
  {
    switch (channel)
    {
      case TIM_CHANNEL_1:
        return LL_TIM_CHANNEL_CH1;
      case TIM_CHANNEL_2:
        return LL_TIM_CHANNEL_CH2;
      case TIM_CHANNEL_3:
        return LL_TIM_CHANNEL_CH3;
      case TIM_CHANNEL_4:
        return LL_TIM_CHANNEL_CH4;
      default:
        return 0U;
    }
  }

  static bool IsValidChannel(uint32_t channel)
  {
    return channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 ||
           channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4;
  }

  static bool IsSupportedTimerInstance(const TIM_TypeDef* instance)
  {
    const auto ADDR = reinterpret_cast<uintptr_t>(instance);
#if defined(TIM1_BASE)
    if (ADDR == TIM1_BASE)
    {
      return true;
    }
#endif
#if defined(TIM8_BASE)
    if (ADDR == TIM8_BASE)
    {
      return true;
    }
#endif
    return false;
  }

  static bool ComputeTimerBase(uint32_t timer_clock_hz, uint32_t target_frequency_hz,
                               bool center_aligned, uint16_t& prescaler_out,
                               uint32_t& period_out)
  {
    const uint32_t ALIGN_FACTOR = center_aligned ? 2U : 1U;
    uint64_t ticks_per_period =
        static_cast<uint64_t>(timer_clock_hz) /
        (static_cast<uint64_t>(target_frequency_hz) * ALIGN_FACTOR);
    if (ticks_per_period < 2U)
    {
      return false;
    }

    uint32_t prescaler = static_cast<uint32_t>((ticks_per_period - 1U) / 65536U);
    if (prescaler > 0xFFFFU)
    {
      return false;
    }

    uint64_t period = ticks_per_period / static_cast<uint64_t>(prescaler + 1U);
    if (period == 0U || period > 65536U)
    {
      return false;
    }

    prescaler_out = static_cast<uint16_t>(prescaler);
    period_out = static_cast<uint32_t>(period - 1U);
    return true;
  }

  uint32_t ResolveTimerClock() const
  {
#if defined(STM32F0) || defined(STM32G0)
    return HAL_RCC_GetHCLKFreq();
#else
    uint32_t clock_hz = HAL_RCC_GetPCLK2Freq();
#ifdef RCC_CFGR_PPRE2
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
    {
      clock_hz *= 2U;
    }
#endif
    return clock_hz;
#endif
  }

  LibXR::ErrorCode StartPhase(uint32_t channel)
  {
    if (HAL_TIM_PWM_Start(htim_, channel) != HAL_OK)
    {
      return LibXR::ErrorCode::FAILED;
    }

    if (config_.complementary_output)
    {
#if defined(TIM_BDTR_MOE)
      if (HAL_TIMEx_PWMN_Start(htim_, channel) != HAL_OK)
      {
        return LibXR::ErrorCode::FAILED;
      }
#else
      return LibXR::ErrorCode::NOT_SUPPORT;
#endif
    }

    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode StopPhase(uint32_t channel)
  {
    if (config_.complementary_output)
    {
#if defined(TIM_BDTR_MOE)
      if (HAL_TIMEx_PWMN_Stop(htim_, channel) != HAL_OK)
      {
        return LibXR::ErrorCode::FAILED;
      }
#endif
    }

    if (HAL_TIM_PWM_Stop(htim_, channel) != HAL_OK)
    {
      return LibXR::ErrorCode::FAILED;
    }

    return LibXR::ErrorCode::OK;
  }

  volatile uint32_t* ResolveCompareRegister(uint32_t channel) const
  {
    TIM_TypeDef* tim = htim_->Instance;
    switch (channel)
    {
      case TIM_CHANNEL_1:
        return &tim->CCR1;
      case TIM_CHANNEL_2:
        return &tim->CCR2;
      case TIM_CHANNEL_3:
        return &tim->CCR3;
      case TIM_CHANNEL_4:
        return &tim->CCR4;
      default:
        return nullptr;
    }
  }

  [[nodiscard]] uint32_t DutyToPulse(float duty) const
  {
    if (duty <= 0.0f)
    {
      return 0U;
    }
    if (duty >= 1.0f)
    {
      return max_pulse_;
    }
    return static_cast<uint32_t>(duty * duty_to_pulse_scale_);
  }

  TIM_HandleTypeDef* htim_ = nullptr;
  uint32_t channel_u_ = TIM_CHANNEL_1;
  uint32_t channel_v_ = TIM_CHANNEL_2;
  uint32_t channel_w_ = TIM_CHANNEL_3;
  volatile uint32_t* ccr_u_ = nullptr;
  volatile uint32_t* ccr_v_ = nullptr;
  volatile uint32_t* ccr_w_ = nullptr;
  Configuration config_ = {};
  float bus_voltage_ = 0.0f;
  uint32_t max_pulse_ = 0U;
  float duty_to_pulse_scale_ = 1.0f;
  bool initialized_ = false;
  bool enabled_ = false;
};

}  // namespace LibXR::FOC

#endif

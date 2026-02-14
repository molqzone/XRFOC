#pragma once

#include <cmath>

#include "foc_types.hpp"

namespace LibXR::FOC
{

template <typename HandleType>
class Inverter
{
 public:
  explicit Inverter(HandleType& handle, float bus_voltage = 0.0f)
      : handle_(handle), bus_voltage_(bus_voltage)
  {
  }

  decltype(auto) Enable(bool in_isr) { return handle_.Enable(in_isr); }

  decltype(auto) Disable(bool in_isr) { return handle_.Disable(in_isr); }

  void SetBusVoltage(float bus_voltage) { bus_voltage_ = bus_voltage; }

  [[nodiscard]] float BusVoltage() const { return bus_voltage_; }

  decltype(auto) SetDuty(DutyUVW duty, bool in_isr)
  {
    duty.u = ClampUnit(duty.u);
    duty.v = ClampUnit(duty.v);
    duty.w = ClampUnit(duty.w);
    return handle_.SetDuty(duty, in_isr);
  }

  [[nodiscard]] HandleType& RawHandle() { return handle_; }
  [[nodiscard]] const HandleType& RawHandle() const { return handle_; }

 private:
  static float ClampUnit(float v) { return std::fmax(0.0f, std::fmin(1.0f, v)); }

  HandleType& handle_;
  float bus_voltage_ = 0.0f;
};

}  // namespace LibXR::FOC

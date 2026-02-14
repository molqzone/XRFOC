#pragma once

#include <type_traits>
#include <utility>

namespace LibXR::FOC
{

template <typename HandleType>
class Position
{
 public:
  using SampleType = std::decay_t<decltype(std::declval<HandleType&>().Read())>;

  explicit Position(HandleType& handle) : handle_(handle) {}

  [[nodiscard]] decltype(auto) Read() { return handle_.Read(); }

  [[nodiscard]] HandleType& RawHandle() { return handle_; }
  [[nodiscard]] const HandleType& RawHandle() const { return handle_; }

 private:
  HandleType& handle_;
};

}  // namespace LibXR::FOC

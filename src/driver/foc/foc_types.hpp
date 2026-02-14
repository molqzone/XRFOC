#pragma once

namespace LibXR::FOC
{

struct PhaseCurrentABC
{
  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;
};

struct AlphaBeta
{
  float alpha = 0.0f;
  float beta = 0.0f;
};

struct DQ
{
  float d = 0.0f;
  float q = 0.0f;
};

struct DutyUVW
{
  float u = 0.5f;
  float v = 0.5f;
  float w = 0.5f;
};

}  // namespace LibXR::FOC

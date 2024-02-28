/**
@defgroup detour Detour

Members in this module are wrappers around the standard math library
*/

#pragma once
#include <cmath>

inline float dtMathFabsf(const float x) { return std::abs(x); }
inline float dtMathSqrtf(const float x) { return std::sqrt(x); }
inline float dtMathFloorf(const float x) { return std::floor(x); }
inline float dtMathCeilf(const float x) { return std::ceil(x); }
inline float dtMathCosf(const float x) { return std::cos(x); }
inline float dtMathSinf(const float x) { return std::sin(x); }
inline float dtMathAtan2f(const float y, const float x) { return std::atan2(y, x); }
inline bool dtMathIsfinite(const float x) { return std::isfinite(x); }

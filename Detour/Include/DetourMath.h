/**
@defgroup detour Detour

Members in this module are wrappers around the standard math library
*/

#ifndef DETOURMATH_H
#define DETOURMATH_H

#include <cmath>

inline float dtMathFabsf(const float x) { return fabsf(x); }
inline float dtMathSqrtf(const float x) { return sqrtf(x); }
inline float dtMathFloorf(const float x) { return floorf(x); }
inline float dtMathCeilf(const float x) { return ceilf(x); }
inline float dtMathCosf(const float x) { return cosf(x); }
inline float dtMathSinf(const float x) { return sinf(x); }
inline float dtMathAtan2f(const float y, const float x) { return atan2f(y, x); }
inline bool dtMathIsfinite(const float x) { return isfinite(x); }

#endif

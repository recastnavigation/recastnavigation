#pragma once

#include "math.h"

inline bool dmIsFinite(float n) {
	return isfinite(n);
}

inline bool dmIsNan(float n) {
	return isnan(n);
}

inline float dmAbs(float n) {
	return fabsf(n);
}

inline float dmSqrt(float n) {
	return sqrtf(n);
}

inline float dmFloor(float n) {
	return floorf(n);
}

inline float dmCeil(float n) {
	return ceilf(n);
}

const float PI = 3.14159265358979323846264338327950288;
const float FRAC_PI_2 = 1.57079632679489661923132169163975144;
const float FRAC_2_PI = 0.636619772367581343075535053490057448;

struct DmSinCos {
    float sin;
    float cos;
};

DmSinCos dmSinCos(float n);

inline float dmSin(float n) {
	DmSinCos res = dmSinCos(n);
	return res.sin;
}

inline float dmCos(float n) {
	DmSinCos res = dmSinCos(n);
	return res.cos;
}

float dmASin(float n);

inline float dmACos(float n) {
	return FRAC_PI_2 - dmASin(n);
}

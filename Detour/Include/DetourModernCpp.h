#ifndef DETOURMODERNCPP_H
#define DETOURMODERNCPP_H

#include <limits>

#include "DetourAssert.h"

#if __cplusplus and __cplusplus >= 201103L
#ifdef NULL
#undef NULL
#endif
#define NULL nullptr
#define DT_OVERRIDE override
#else
#include <stddef.h>
#define DT_OVERRIDE
#endif

template <typename TO, typename FROM>
inline TO dtCast( FROM val )
{
	dtAssert( std::numeric_limits<TO>::min() <= val && val <= std::numeric_limits<TO>::max() );
	return static_cast<TO>( val );
}

#endif

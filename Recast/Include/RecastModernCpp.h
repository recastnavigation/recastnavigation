#ifndef RECASTMODERNCPP_H
#define RECASTMODERNCPP_H

#include <limits>

#include "RecastAssert.h"

#if __cplusplus and __cplusplus >= 201103L
#ifdef NULL
#undef NULL
#endif
#define NULL nullptr
#define RC_OVERRIDE override
#else
#include <stddef.h>
#define RC_OVERRIDE
#endif

template <typename TO, typename FROM>
inline TO rcCast( FROM val )
{
	rcAssert( std::numeric_limits<TO>::min() <= val && val <= std::numeric_limits<TO>::max() );
	return static_cast<TO>( val );
}

#endif

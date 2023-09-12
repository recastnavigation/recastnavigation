#ifndef RECASTMODERNCPP_H
#define RECASTMODERNCPP_H

#include <limits>

#include "RecastAssert.h"

#if defined(__cplusplus) && __cplusplus >= 201103L
#define RC_NULL nullptr
#define RC_OVERRIDE override
#else
#include <stddef.h>
#define RC_NULL NULL
#define RC_OVERRIDE
#endif

template <typename TO, typename FROM>
inline TO rcCast( FROM val )
{
	rcAssert( std::numeric_limits<TO>::min() <= val && val <= std::numeric_limits<TO>::max() );
	return static_cast<TO>( val );
}

#endif

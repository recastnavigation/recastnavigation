#include "catch.hpp"

#include "Recast.h"

TEST_CASE("rcVdot")
{
	SECTION("Dot normalized vector with itself")
	{
		float v1[] = { 1, 0, 0 };
		float result = rcVdot(v1, v1);
		REQUIRE(result == Approx(1));
	}
}


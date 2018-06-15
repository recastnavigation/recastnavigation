#include "catch.hpp"

#include "DetourCommon.h"

TEST_CASE("dtRandomPointInConvexPoly")
{
	SECTION("Properly works when the argument 's' is 1.0f")
	{
		const float pts[] = {
			0, 0, 0,
			0, 0, 1,
			1, 0, 0,
		};
		const int npts = 3;
		float areas[6];
		float out[3];

		dtRandomPointInConvexPoly(pts, npts, areas, 0.0f, 1.0f, out);
		REQUIRE(out[0] == Approx(0));
		REQUIRE(out[1] == Approx(0));
		REQUIRE(out[2] == Approx(1));

		dtRandomPointInConvexPoly(pts, npts, areas, 0.5f, 1.0f, out);
		REQUIRE(out[0] == Approx(1.0f / 2));
		REQUIRE(out[1] == Approx(0));
		REQUIRE(out[2] == Approx(1.0f / 2));

		dtRandomPointInConvexPoly(pts, npts, areas, 1.0f, 1.0f, out);
		REQUIRE(out[0] == Approx(1));
		REQUIRE(out[1] == Approx(0));
		REQUIRE(out[2] == Approx(0));
	}
}

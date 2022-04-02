#include <stdio.h>
#include <string.h>

#include "catch.hpp"

#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

// For comparing to rcVector in benchmarks.
#include <vector>

TEST_CASE("rcSwap")
{
	SECTION("Swap two values")
	{
		int one = 1;
		int two = 2;
		rcSwap(one, two);
		REQUIRE(one == 2);
		REQUIRE(two == 1);
	}
}

TEST_CASE("rcMin")
{
	SECTION("Min returns the lowest value.")
	{
		REQUIRE(rcMin(1, 2) == 1);
		REQUIRE(rcMin(2, 1) == 1);
	}

	SECTION("Min with equal args")
	{
		REQUIRE(rcMin(1, 1) == 1);
	}
}

TEST_CASE("rcMax")
{
	SECTION("Max returns the greatest value.")
	{
		REQUIRE(rcMax(1, 2) == 2);
		REQUIRE(rcMax(2, 1) == 2);
	}

	SECTION("Max with equal args")
	{
		REQUIRE(rcMax(1, 1) == 1);
	}
}

TEST_CASE("rcAbs")
{
	SECTION("Abs returns the absolute value.")
	{
		REQUIRE(rcAbs(-1) == 1);
		REQUIRE(rcAbs(1) == 1);
		REQUIRE(rcAbs(0) == 0);
	}
}

TEST_CASE("rcSqr")
{
	SECTION("Sqr squares a number")
	{
		REQUIRE(rcSqr(2) == 4);
		REQUIRE(rcSqr(-4) == 16);
		REQUIRE(rcSqr(0) == 0);
	}
}

TEST_CASE("rcClamp")
{
	SECTION("Higher than range")
	{
		REQUIRE(rcClamp(2, 0, 1) == 1);
	}

	SECTION("Within range")
	{
		REQUIRE(rcClamp(1, 0, 2) == 1);
	}

	SECTION("Lower than range")
	{
		REQUIRE(rcClamp(0, 1, 2) == 1);
	}
}

TEST_CASE("rcSqrt")
{
	SECTION("Sqrt gets the sqrt of a number")
	{
		REQUIRE(rcSqrt(4) == Approx(2));
		REQUIRE(rcSqrt(81) == Approx(9));
	}
}

TEST_CASE("rcVcross")
{
	SECTION("Computes cross product")
	{
		float v1[3] = {3, -3, 1};
		float v2[3] = {4, 9, 2};
		float result[3];
		rcVcross(result, v1, v2);
		REQUIRE(result[0] == Approx(-15));
		REQUIRE(result[1] == Approx(-2));
		REQUIRE(result[2] == Approx(39));
	}

	SECTION("Cross product with itself is zero")
	{
		float v1[3] = {3, -3, 1};
		float result[3];
		rcVcross(result, v1, v1);
		REQUIRE(result[0] == Approx(0));
		REQUIRE(result[1] == Approx(0));
		REQUIRE(result[2] == Approx(0));
	}
}

TEST_CASE("rcVdot")
{
	SECTION("Dot normalized vector with itself")
	{
		float v1[] = { 1, 0, 0 };
		float result = rcVdot(v1, v1);
		REQUIRE(result == Approx(1));
	}

	SECTION("Dot zero vector with anything is zero")
	{
		float v1[] = { 1, 2, 3 };
		float v2[] = { 0, 0, 0 };

		float result = rcVdot(v1, v2);
		REQUIRE(result == Approx(0));
	}
}

TEST_CASE("rcVmad")
{
	SECTION("scaled add two vectors")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {0, 2, 4};
		float result[3];
		rcVmad(result, v1, v2, 2);
		REQUIRE(result[0] == Approx(1));
		REQUIRE(result[1] == Approx(6));
		REQUIRE(result[2] == Approx(11));
	}

	SECTION("second vector is scaled, first is not")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {5, 6, 7};
		float result[3];
		rcVmad(result, v1, v2, 0);
		REQUIRE(result[0] == Approx(1));
		REQUIRE(result[1] == Approx(2));
		REQUIRE(result[2] == Approx(3));
	}
}

TEST_CASE("rcVadd")
{
	SECTION("add two vectors")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {5, 6, 7};
		float result[3];
		rcVadd(result, v1, v2);
		REQUIRE(result[0] == Approx(6));
		REQUIRE(result[1] == Approx(8));
		REQUIRE(result[2] == Approx(10));
	}
}

TEST_CASE("rcVsub")
{
	SECTION("subtract two vectors")
	{
		float v1[3] = {5, 4, 3};
		float v2[3] = {1, 2, 3};
		float result[3];
		rcVsub(result, v1, v2);
		REQUIRE(result[0] == Approx(4));
		REQUIRE(result[1] == Approx(2));
		REQUIRE(result[2] == Approx(0));
	}
}

TEST_CASE("rcVmin")
{
	SECTION("selects the min component from the vectors")
	{
		float v1[3] = {5, 4, 0};
		float v2[3] = {1, 2, 9};
		rcVmin(v1, v2);
		REQUIRE(v1[0] == Approx(1));
		REQUIRE(v1[1] == Approx(2));
		REQUIRE(v1[2] == Approx(0));
	}

	SECTION("v1 is min")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {4, 5, 6};
		rcVmin(v1, v2);
		REQUIRE(v1[0] == Approx(1));
		REQUIRE(v1[1] == Approx(2));
		REQUIRE(v1[2] == Approx(3));
	}

	SECTION("v2 is min")
	{
		float v1[3] = {4, 5, 6};
		float v2[3] = {1, 2, 3};
		rcVmin(v1, v2);
		REQUIRE(v1[0] == Approx(1));
		REQUIRE(v1[1] == Approx(2));
		REQUIRE(v1[2] == Approx(3));
	}
}

TEST_CASE("rcVmax")
{
	SECTION("selects the max component from the vectors")
	{
		float v1[3] = {5, 4, 0};
		float v2[3] = {1, 2, 9};
		rcVmax(v1, v2);
		REQUIRE(v1[0] == Approx(5));
		REQUIRE(v1[1] == Approx(4));
		REQUIRE(v1[2] == Approx(9));
	}

	SECTION("v2 is max")
	{
		float v1[3] = {1, 2, 3};
		float v2[3] = {4, 5, 6};
		rcVmax(v1, v2);
		REQUIRE(v1[0] == Approx(4));
		REQUIRE(v1[1] == Approx(5));
		REQUIRE(v1[2] == Approx(6));
	}

	SECTION("v1 is max")
	{
		float v1[3] = {4, 5, 6};
		float v2[3] = {1, 2, 3};
		rcVmax(v1, v2);
		REQUIRE(v1[0] == Approx(4));
		REQUIRE(v1[1] == Approx(5));
		REQUIRE(v1[2] == Approx(6));
	}
}

TEST_CASE("rcVcopy")
{
	SECTION("copies a vector into another vector")
	{
		float v1[3] = {5, 4, 0};
		float result[3] = {1, 2, 9};
		rcVcopy(result, v1);
		REQUIRE(result[0] == Approx(5));
		REQUIRE(result[1] == Approx(4));
		REQUIRE(result[2] == Approx(0));
		REQUIRE(v1[0] == Approx(5));
		REQUIRE(v1[1] == Approx(4));
		REQUIRE(v1[2] == Approx(0));
	}
}

TEST_CASE("rcVdist")
{
	SECTION("distance between two vectors")
	{
		float v1[3] = {3, 1, 3};
		float v2[3] = {1, 3, 1};
		float result = rcVdist(v1, v2);

		REQUIRE(result == Approx(3.4641f));
	}

	SECTION("Distance from zero is magnitude")
	{
		float v1[3] = {3, 1, 3};
		float v2[3] = {0, 0, 0};
		float distance = rcVdist(v1, v2);
		float magnitude = rcSqrt(rcSqr(v1[0]) + rcSqr(v1[1]) + rcSqr(v1[2]));
		REQUIRE(distance == Approx(magnitude));
	}
}

TEST_CASE("rcVdistSqr")
{
	SECTION("squared distance between two vectors")
	{
		float v1[3] = {3, 1, 3};
		float v2[3] = {1, 3, 1};
		float result = rcVdistSqr(v1, v2);

		REQUIRE(result == Approx(12));
	}

	SECTION("squared distance from zero is squared magnitude")
	{
		float v1[3] = {3, 1, 3};
		float v2[3] = {0, 0, 0};
		float distance = rcVdistSqr(v1, v2);
		float magnitude = rcSqr(v1[0]) + rcSqr(v1[1]) + rcSqr(v1[2]);
		REQUIRE(distance == Approx(magnitude));
	}
}

TEST_CASE("rcVnormalize")
{
	SECTION("normalizing reduces magnitude to 1")
	{
		float v[3] = {3, 3, 3};
		rcVnormalize(v);
		REQUIRE(v[0] == Approx(rcSqrt(1.0f / 3.0f)));
		REQUIRE(v[1] == Approx(rcSqrt(1.0f / 3.0f)));
		REQUIRE(v[2] == Approx(rcSqrt(1.0f / 3.0f)));
		float magnitude = rcSqrt(rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]));
		REQUIRE(magnitude == Approx(1));
	}
}

TEST_CASE("rcCalcBounds")
{
	SECTION("bounds of one vector")
	{
		float verts[] = {1, 2, 3};
		float bmin[3];
		float bmax[3];
		rcCalcBounds(verts, 1, bmin, bmax);

		REQUIRE(bmin[0] == Approx(verts[0]));
		REQUIRE(bmin[1] == Approx(verts[1]));
		REQUIRE(bmin[2] == Approx(verts[2]));

		REQUIRE(bmax[0] == Approx(verts[0]));
		REQUIRE(bmax[1] == Approx(verts[1]));
		REQUIRE(bmax[2] == Approx(verts[2]));
	}

	SECTION("bounds of more than one vector")
	{
		float verts[] = {
			1, 2, 3,
			0, 2, 5
		};
		float bmin[3];
		float bmax[3];
		rcCalcBounds(verts, 2, bmin, bmax);

		REQUIRE(bmin[0] == Approx(0));
		REQUIRE(bmin[1] == Approx(2));
		REQUIRE(bmin[2] == Approx(3));

		REQUIRE(bmax[0] == Approx(1));
		REQUIRE(bmax[1] == Approx(2));
		REQUIRE(bmax[2] == Approx(5));
	}
}

TEST_CASE("rcCalcGridSize")
{
	SECTION("computes the size of an x & z axis grid")
	{
		float verts[] = {
			1, 2, 3,
			0, 2, 6
		};
		float bmin[3];
		float bmax[3];
		rcCalcBounds(verts, 2, bmin, bmax);

		float cellSize = 1.5f;

		int width;
		int height;

		rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

		REQUIRE(width == 1);
		REQUIRE(height == 2);
	}
}

TEST_CASE("rcCreateHeightfield")
{
	SECTION("create a heightfield")
	{
		float verts[] = {
			1, 2, 3,
			0, 2, 6
		};
		float bmin[3];
		float bmax[3];
		rcCalcBounds(verts, 2, bmin, bmax);

		float cellSize = 1.5f;
		float cellHeight = 2;

		int width;
		int height;

		rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

		rcHeightfield heightfield;

		bool result = rcCreateHeightfield(0, heightfield, width, height, bmin, bmax, cellSize, cellHeight);

		REQUIRE(result);

		REQUIRE(heightfield.width == width);
		REQUIRE(heightfield.height == height);

		REQUIRE(heightfield.bmin[0] == Approx(bmin[0]));
		REQUIRE(heightfield.bmin[1] == Approx(bmin[1]));
		REQUIRE(heightfield.bmin[2] == Approx(bmin[2]));

		REQUIRE(heightfield.bmax[0] == Approx(bmax[0]));
		REQUIRE(heightfield.bmax[1] == Approx(bmax[1]));
		REQUIRE(heightfield.bmax[2] == Approx(bmax[2]));

		REQUIRE(heightfield.cs == Approx(cellSize));
		REQUIRE(heightfield.ch == Approx(cellHeight));

		REQUIRE(heightfield.spans != 0);
		REQUIRE(heightfield.pools == 0);
		REQUIRE(heightfield.freelist == 0);
	}
}

TEST_CASE("rcMarkWalkableTriangles")
{
	rcContext* ctx = 0;
	float walkableSlopeAngle = 45;
	float verts[] = {
		0, 0, 0,
		1, 0, 0,
		0, 0, -1
	};
	int nv = 3;
	int walkable_tri[] = { 0, 1, 2 };
	int unwalkable_tri[] = { 0, 2, 1 };
	int nt = 1;
	unsigned char areas[] = { RC_NULL_AREA };

	SECTION("One walkable triangle")
	{
		rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_WALKABLE_AREA);
	}

	SECTION("One non-walkable triangle")
	{
		rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}

	SECTION("Non-walkable triangle area id's are not modified")
	{
		areas[0] = 42;
		rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas);
		REQUIRE(areas[0] == 42);
	}

	SECTION("Slopes equal to the max slope are considered unwalkable.")
	{
		walkableSlopeAngle = 0;
		rcMarkWalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}
}

TEST_CASE("rcClearUnwalkableTriangles")
{
	rcContext* ctx = 0;
	float walkableSlopeAngle = 45;
	float verts[] = {
		0, 0, 0,
		1, 0, 0,
		0, 0, -1
	};
	int nv = 3;
	int walkable_tri[] = { 0, 1, 2 };
	int unwalkable_tri[] = { 0, 2, 1 };
	int nt = 1;
	unsigned char areas[] = { 42 };

	SECTION("Sets area ID of unwalkable triangle to RC_NULL_AREA")
	{
		rcClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, unwalkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}

	SECTION("Does not modify walkable triangle aread ID's")
	{
		rcClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
		REQUIRE(areas[0] == 42);
	}

	SECTION("Slopes equal to the max slope are considered unwalkable.")
	{
		walkableSlopeAngle = 0;
		rcClearUnwalkableTriangles(ctx, walkableSlopeAngle, verts, nv, walkable_tri, nt, areas);
		REQUIRE(areas[0] == RC_NULL_AREA);
	}
}

TEST_CASE("rcAddSpan")
{
	rcContext ctx(false);

	float verts[] = {
		1, 2, 3,
		0, 2, 6
	};
	float bmin[3];
	float bmax[3];
	rcCalcBounds(verts, 2, bmin, bmax);

	float cellSize = 1.5f;
	float cellHeight = 2;

	int width;
	int height;

	rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, width, height, bmin, bmax, cellSize, cellHeight));

	int x = 0;
	int y = 0;
	unsigned short smin = 0;
	unsigned short smax = 1;
	unsigned char area = 42;
	int flagMergeThr = 1;

	SECTION("Add a span to an empty heightfield.")
	{
		bool result = rcAddSpan(&ctx, hf, x, y, smin, smax, area, flagMergeThr);
		REQUIRE(result);
		REQUIRE(hf.spans[0] != 0);
		REQUIRE(hf.spans[0]->smin == smin);
		REQUIRE(hf.spans[0]->smax == smax);
		REQUIRE(hf.spans[0]->area == area);
	}

	SECTION("Add a span that gets merged with an existing span.")
	{
		bool result = rcAddSpan(&ctx, hf, x, y, smin, smax, area, flagMergeThr);
		REQUIRE(result);
		REQUIRE(hf.spans[0] != 0);
		REQUIRE(hf.spans[0]->smin == smin);
		REQUIRE(hf.spans[0]->smax == smax);
		REQUIRE(hf.spans[0]->area == area);

		smin = 1;
		smax = 2;
		result = rcAddSpan(&ctx, hf, x, y, smin, smax, area, flagMergeThr);
		REQUIRE(result);
		REQUIRE(hf.spans[0] != 0);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 2);
		REQUIRE(hf.spans[0]->area == area);
	}

	SECTION("Add a span that merges with two spans above and below.")
	{
		smin = 0;
		smax = 1;
		REQUIRE(rcAddSpan(&ctx, hf, x, y, smin, smax, area, flagMergeThr));
		REQUIRE(hf.spans[0] != 0);
		REQUIRE(hf.spans[0]->smin == smin);
		REQUIRE(hf.spans[0]->smax == smax);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == 0);

		smin = 2;
		smax = 3;
		REQUIRE(rcAddSpan(&ctx, hf, x, y, smin, smax, area, flagMergeThr));
		REQUIRE(hf.spans[0]->next != 0);
		REQUIRE(hf.spans[0]->next->smin == smin);
		REQUIRE(hf.spans[0]->next->smax == smax);
		REQUIRE(hf.spans[0]->next->area == area);

		smin = 1;
		smax = 2;
		REQUIRE(rcAddSpan(&ctx, hf, x, y, smin, smax, area, flagMergeThr));
		REQUIRE(hf.spans[0] != 0);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 3);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == 0);
	}
}

TEST_CASE("rcRasterizeTriangle")
{
	rcContext ctx;
	float verts[] = {
		0, 0, 0,
		1, 0, 0,
		0, 0, -1
	};
	float bmin[3];
	float bmax[3];
	rcCalcBounds(verts, 3, bmin, bmax);

	float cellSize = .5f;
	float cellHeight = .5f;

	int width;
	int height;

	rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

	rcHeightfield solid;
	REQUIRE(rcCreateHeightfield(&ctx, solid, width, height, bmin, bmax, cellSize, cellHeight));

	unsigned char area = 42;
	int flagMergeThr = 1;

	SECTION("Rasterize a triangle")
	{
		REQUIRE(rcRasterizeTriangle(&ctx, &verts[0], &verts[3], &verts[6], area, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == area);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == area);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == area);
		REQUIRE(!solid.spans[1 + 1 * width]->next);
	}
}

TEST_CASE("rcRasterizeTriangles")
{
	rcContext ctx;
	float verts[] = {
		0, 0, 0,
		1, 0, 0,
		0, 0, -1,
		0, 0, 1
	};
	int tris[] = {
		0, 1, 2,
		0, 3, 1
	};
	unsigned char areas[] = {
		1,
		2
	};
	float bmin[3];
	float bmax[3];
	rcCalcBounds(verts, 4, bmin, bmax);

	float cellSize = .5f;
	float cellHeight = .5f;

	int width;
	int height;

	rcCalcGridSize(bmin, bmax, cellSize, &width, &height);

	rcHeightfield solid; 
	REQUIRE(rcCreateHeightfield(&ctx, solid, width, height, bmin, bmax, cellSize, cellHeight));

	int flagMergeThr = 1;

	SECTION("Rasterize some triangles")
	{
		REQUIRE(rcRasterizeTriangles(&ctx, verts, 4, tris, areas, 2, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}

	SECTION("Unsigned short overload")
	{
		unsigned short utris[] = {
			0, 1, 2,
			0, 3, 1
		};
		REQUIRE(rcRasterizeTriangles(&ctx, verts, 4, utris, areas, 2, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}

	SECTION("Triangle list overload")
	{
		float vertsList[] = {
			0, 0, 0,
			1, 0, 0,
			0, 0, -1,
			0, 0, 0,
			0, 0, 1,
			1, 0, 0,
		};

		REQUIRE(rcRasterizeTriangles(&ctx, vertsList, areas, 2, solid, flagMergeThr));

		REQUIRE(solid.spans[0 + 0 * width]);
		REQUIRE(solid.spans[0 + 1 * width]);
		REQUIRE(solid.spans[0 + 2 * width]);
		REQUIRE(solid.spans[0 + 3 * width]);
		REQUIRE(!solid.spans[1 + 0 * width]);
		REQUIRE(solid.spans[1 + 1 * width]);
		REQUIRE(solid.spans[1 + 2 * width]);
		REQUIRE(!solid.spans[1 + 3 * width]);

		REQUIRE(solid.spans[0 + 0 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 0 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 0 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 0 * width]->next);

		REQUIRE(solid.spans[0 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[0 + 1 * width]->next);

		REQUIRE(solid.spans[0 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 2 * width]->next);

		REQUIRE(solid.spans[0 + 3 * width]->smin == 0);
		REQUIRE(solid.spans[0 + 3 * width]->smax == 1);
		REQUIRE(solid.spans[0 + 3 * width]->area == 2);
		REQUIRE(!solid.spans[0 + 3 * width]->next);

		REQUIRE(solid.spans[1 + 1 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 1 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 1 * width]->area == 1);
		REQUIRE(!solid.spans[1 + 1 * width]->next);

		REQUIRE(solid.spans[1 + 2 * width]->smin == 0);
		REQUIRE(solid.spans[1 + 2 * width]->smax == 1);
		REQUIRE(solid.spans[1 + 2 * width]->area == 2);
		REQUIRE(!solid.spans[1 + 2 * width]->next);
	}
}

// Used to verify that rcVector constructs/destroys objects correctly.
struct Incrementor {
	static int constructions;
	static int destructions;
	static int copies;
	Incrementor() { constructions++; }
	~Incrementor() { destructions++; }
	Incrementor(const Incrementor&) { copies++; }
	Incrementor& operator=(const Incrementor&); // Deleted assignment.

	static void Reset() {
		constructions = 0;
		destructions = 0;
		copies = 0;
	}
};
int Incrementor::constructions = 0;
int Incrementor::destructions = 0;
int Incrementor::copies = 0;

const int kMaxAllocSize = 1024;
const unsigned char kClearValue = 0xff;
// Simple alloc/free that clears the memory on free..
void* AllocAndInit(size_t size, rcAllocHint) {
	rcAssert(kMaxAllocSize >= size);
	return memset(malloc(kMaxAllocSize), 0, kMaxAllocSize);
}
void FreeAndClear(void* mem) {
	if (mem) {
	  memset(mem, kClearValue, kMaxAllocSize);
	}
	free(mem);
}
// Verifies that memory has been initialized by AllocAndInit, and not cleared by FreeAndClear.
struct Copier {
	const static int kAlive;
	const static int kDead;
	Copier() : value(kAlive) {}

	// checks that the source of the copy is valid.
	Copier(const Copier& other) : value(kAlive) {
		other.Verify();
	}
	Copier& operator=(const Copier&);

	// Marks the value as dead.
	~Copier() { value = kDead; }
	void Verify() const {
		REQUIRE(value == kAlive);
	}
	volatile int value;
};
const int Copier::kAlive = 0x1f;
const int Copier::kDead = 0xde;

struct NotDefaultConstructible {
	NotDefaultConstructible(int) {}
};

TEST_CASE("rcVector")
{
	SECTION("Vector basics.")
	{
		rcTempVector<int> vec;
		REQUIRE(vec.size() == 0);
		vec.push_back(10);
		vec.push_back(12);
		REQUIRE(vec.size() == 2);
		REQUIRE(vec.capacity() >= 2);
		REQUIRE(vec[0] == 10);
		REQUIRE(vec[1] == 12);
		vec.pop_back();
		REQUIRE(vec.size() == 1);
		REQUIRE(vec[0] == 10);
		vec.pop_back();
		REQUIRE(vec.size() == 0);
		vec.resize(100, 5);
		REQUIRE(vec.size() == 100);
		for (int i = 0; i < 100; i++) {
			REQUIRE(vec[i] == 5);
			vec[i] = i;
		}
		for (int i = 0; i < 100; i++) {
			REQUIRE(vec[i] == i);
		}
	}

	SECTION("Constructors/Destructors")
	{
		Incrementor::Reset();
		rcTempVector<Incrementor> vec;
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 0);
		vec.push_back(Incrementor());
		// push_back() may create and copy objects internally.
		REQUIRE(Incrementor::constructions == 1);
		REQUIRE(Incrementor::destructions >= 1);
		// REQUIRE(Incrementor::copies >= 2);

		vec.clear();
		Incrementor::Reset();
		vec.resize(100);
		// Initialized with default instance. Temporaries may be constructed, then destroyed.
		REQUIRE(Incrementor::constructions == 100);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 0);

		Incrementor::Reset();
		for (int i = 0; i < 100; i++) {
			REQUIRE(Incrementor::destructions == i);
			vec.pop_back();
		}
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 100);
		REQUIRE(Incrementor::copies == 0);

		vec.resize(100);
		Incrementor::Reset();
		vec.clear();
		// One temp object is constructed for the default argumnet of resize().
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 100);
		REQUIRE(Incrementor::copies == 0);

		Incrementor::Reset();
		vec.resize(100, Incrementor());
		REQUIRE(Incrementor::constructions == 1);
		REQUIRE(Incrementor::destructions == 1);
		REQUIRE(Incrementor::copies == 100);
	}

	SECTION("Copying Contents")
	{

		// veriyf event counts after doubling size -- should require a lot of copying and destorying.
		rcTempVector<Incrementor> vec;
		Incrementor::Reset();
		vec.resize(100);
		REQUIRE(Incrementor::constructions == 100);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 0);
		Incrementor::Reset();
		vec.resize(200);
		REQUIRE(vec.size() == vec.capacity());
		REQUIRE(Incrementor::constructions == 100);  // Construc new elements.
		REQUIRE(Incrementor::destructions == 100);  // Destroy old contents.
		REQUIRE(Incrementor::copies == 100);  // Copy old elements into new array.
	}

	SECTION("Swap")
	{
		rcTempVector<int> a(10, 0xa);
		rcTempVector<int> b;

		int* a_data = a.data();
		int* b_data = b.data();

		a.swap(b);
		REQUIRE(a.size() == 0);
		REQUIRE(b.size() == 10);
		REQUIRE(b[0] == 0xa);
		REQUIRE(b[9] == 0xa);
		REQUIRE(a.data() == b_data);
		REQUIRE(b.data() == a_data);
	}

	SECTION("Overlapping init")
	{
		rcAllocSetCustom(&AllocAndInit, &FreeAndClear);
		rcTempVector<Copier> vec;
		// Force a realloc during push_back().
		vec.resize(64);
		REQUIRE(vec.capacity() == vec.size());
		REQUIRE(vec.capacity() > 0);
		REQUIRE(vec.size() == vec.capacity());

		// Don't crash.
		vec.push_back(vec[0]);
		rcAllocSetCustom(NULL, NULL);
	}

	SECTION("Vector Destructor")
	{
		{
			rcTempVector<Incrementor> vec;
			vec.resize(10);
			Incrementor::Reset();
		}
		REQUIRE(Incrementor::destructions == 10);
	}

	SECTION("Assign")
	{
		rcTempVector<int> a(10, 0xa);
		a.assign(5, 0xb);
		REQUIRE(a.size() == 5);
		REQUIRE(a[0] == 0xb);
		REQUIRE(a[4] == 0xb);
		a.assign(15, 0xc);
		REQUIRE(a.size() == 15);
		REQUIRE(a[0] == 0xc);
		REQUIRE(a[14] == 0xc);

		rcTempVector<int> b;
		b.assign(a.data(), a.data() + a.size());
		REQUIRE(b.size() == a.size());
		REQUIRE(b[0] == a[0]);
	}

	SECTION("Copy")
	{
		rcTempVector<int> a(10, 0xa);
		rcTempVector<int> b(a);
		REQUIRE(a.size() == 10);
		REQUIRE(a.size() == b.size());
		REQUIRE(a[0] == b[0]);
		REQUIRE(a.data() != b.data());
		rcTempVector<int> c(a.data(), a.data() + a.size());
		REQUIRE(c.size() == a.size());
		REQUIRE(c[0] == a[0]);

		rcTempVector<Incrementor> d(10);
		Incrementor::Reset();
		rcTempVector<Incrementor> e(d);
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 10);

		Incrementor::Reset();
		rcTempVector<Incrementor> f(d.data(), d.data() + d.size());
		REQUIRE(Incrementor::constructions == 0);
		REQUIRE(Incrementor::destructions == 0);
		REQUIRE(Incrementor::copies == 10);
	}

	SECTION("Type Requirements")
	{
		// This section verifies that we don't enforce unnecessary
		// requirements on the types we hold.

		// Implementing clear as resize(0) will cause this to fail
		// as resize(0) requires T to be default constructible.
		rcTempVector<NotDefaultConstructible> v;
		v.clear();
	}
}

// TODO: Implement benchmarking for platforms other than posix.
#ifdef __unix__
#include <unistd.h>
#ifdef _POSIX_TIMERS
#include <time.h>
#include <stdint.h>

int64_t NowNanos() {
	struct timespec tp;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tp);
	return tp.tv_nsec + 1000000000LL * tp.tv_sec;
}

#define BM(name, iterations) \
	struct BM_ ## name { \
		static void Run() { \
			int64_t begin_time = NowNanos(); \
			for (int i = 0 ; i < iterations; i++) { \
				Body(); \
			} \
			int64_t nanos = NowNanos() - begin_time; \
			printf("BM_%-35s %ld iterations in %10ld nanos: %10.2f nanos/it\n", #name ":", (int64_t)iterations, nanos, double(nanos) / iterations); \
		} \
		static void Body(); \
	}; \
	TEST_CASE(#name) { \
		BM_ ## name::Run(); \
	} \
	void BM_ ## name::Body()

const int64_t kNumLoops = 100;
const int64_t kNumInserts = 100000;

// Prevent compiler from eliding a calculation.
// TODO: Implement for MSVC.
template <typename T>
void DoNotOptimize(T* v) {
	asm volatile ("" : "+r" (v));
}

BM(FlatArray_Push, kNumLoops)
{
	int cap = 64;
	int* v = (int*)rcAlloc(cap * sizeof(int), RC_ALLOC_TEMP);
	for (int j = 0; j < kNumInserts; j++) {
		if (j == cap) {
			cap *= 2;
			int* tmp  = (int*)rcAlloc(sizeof(int) * cap, RC_ALLOC_TEMP);
			memcpy(tmp, v, j * sizeof(int));
			rcFree(v);
			v = tmp;
		}
		v[j] = 2;
	}

	DoNotOptimize(v);
	rcFree(v);
}
BM(FlatArray_Fill, kNumLoops)
{
	int* v = (int*)rcAlloc(sizeof(int) * kNumInserts, RC_ALLOC_TEMP);
	for (int j = 0; j < kNumInserts; j++) {
		v[j] = 2;
	}

	DoNotOptimize(v);
	rcFree(v);
}
BM(FlatArray_Memset, kNumLoops)
{
	int* v = (int*)rcAlloc(sizeof(int) * kNumInserts, RC_ALLOC_TEMP);
	memset(v, 0, kNumInserts * sizeof(int));

	DoNotOptimize(v);
	rcFree(v);
}

BM(rcVector_Push, kNumLoops)
{
	rcTempVector<int> v;
	for (int j = 0; j < kNumInserts; j++) {
		v.push_back(2);
	}
	DoNotOptimize(v.data());
}
BM(rcVector_PushPreallocated, kNumLoops)
{
	rcTempVector<int> v;
	v.reserve(kNumInserts);
	for (int j = 0; j < kNumInserts; j++) {
		v.push_back(2);
	}
	DoNotOptimize(v.data());
}
BM(rcVector_Assign, kNumLoops)
{
	rcTempVector<int> v;
	v.assign(kNumInserts, 2);
	DoNotOptimize(v.data());
}
BM(rcVector_AssignIndices, kNumLoops)
{
	rcTempVector<int> v;
	v.resize(kNumInserts);
	for (int j = 0; j < kNumInserts; j++) {
		v[j] = 2;
	}
	DoNotOptimize(v.data());
}
BM(rcVector_Resize, kNumLoops)
{
	rcTempVector<int> v;
	v.resize(kNumInserts, 2);
	DoNotOptimize(v.data());
}

BM(stdvector_Push, kNumLoops)
{
	std::vector<int> v;
	for (int j = 0; j < kNumInserts; j++) {
		v.push_back(2);
	}
	DoNotOptimize(v.data());
}
BM(stdvector_PushPreallocated, kNumLoops)
{
	std::vector<int> v;
	v.reserve(kNumInserts);
	for (int j = 0; j < kNumInserts; j++) {
		v.push_back(2);
	}
	DoNotOptimize(v.data());
}
BM(stdvector_Assign, kNumLoops)
{
	std::vector<int> v;
	v.assign(kNumInserts, 2);
	DoNotOptimize(v.data());
}
BM(stdvector_AssignIndices, kNumLoops)
{
	std::vector<int> v;
	v.resize(kNumInserts);
	for (int j = 0; j < kNumInserts; j++) {
		v[j] = 2;
	}
	DoNotOptimize(v.data());
}
BM(stdvector_Resize, kNumLoops)
{
	std::vector<int> v;
	v.resize(kNumInserts, 2);
	DoNotOptimize(v.data());
}

#undef BM
#endif  // _POSIX_TIMERS
#endif  // __unix__

#include "Recast.h"
#include "catch2/catch_amalgamated.hpp"

TEST_CASE("rcAddSpan", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 4;
	constexpr int ySize = 10;
	constexpr int zSize = 4;

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 2.0f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	constexpr unsigned char area = 42;
	constexpr int flagMergeThr = 1;

	SECTION("Add a span to an empty heightfield.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Adding invalid or zero-size spans does nothing.")
	{
		// min == max
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 0, area, flagMergeThr));
		REQUIRE(hf.spans[0] == nullptr);

		// min > maxs
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 0, area, flagMergeThr));
		REQUIRE(hf.spans[0] == nullptr);
	}

	SECTION("Two spans that are not touching are not merged.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->next != nullptr);
		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next == nullptr);
	}

	SECTION("Two spans with different area ids within the flag merge threshold are merged and the highest area ID is used.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, 42, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == 42);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, 24, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 2);
		REQUIRE(hf.spans[0]->area == 42); // Higher area ID takes precedent
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Two spans with different area ids outside the flag merge threshold are merged and the area ID of the last span added is used.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, 42, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == 42);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 8, 24, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == 24); // Area ID of the last-added span takes precedent
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Add a span that gets merged with an existing span.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 2);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Add a span that merges with two spans above and below.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0]->next != nullptr);
		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next == nullptr);

		// After adding the third span, they should all get merged into a single span.
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 1, 2, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 3);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Spans are insertion-sorted in ascending order of Y value.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 1, area, flagMergeThr));
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 6, 7, area, flagMergeThr));

		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 1);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next != nullptr);

		REQUIRE(hf.spans[0]->next->smin == 2);
		REQUIRE(hf.spans[0]->next->smax == 3);
		REQUIRE(hf.spans[0]->next->area == area);
		REQUIRE(hf.spans[0]->next->next != nullptr);

		REQUIRE(hf.spans[0]->next->next->smin == 6);
		REQUIRE(hf.spans[0]->next->next->smax == 7);
		REQUIRE(hf.spans[0]->next->next->area == area);
		REQUIRE(hf.spans[0]->next->next->next == nullptr);
	}

	SECTION("Adding a span inside another span merges them.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 8, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 3, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 8);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}

	SECTION("Overlapping spans are merged.")
	{
		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 0, 4, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 4);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);

		REQUIRE(rcAddSpan(&ctx, hf, 0, 0, 2, 6, area, flagMergeThr));
		REQUIRE(hf.spans[0] != nullptr);
		REQUIRE(hf.spans[0]->smin == 0);
		REQUIRE(hf.spans[0]->smax == 6);
		REQUIRE(hf.spans[0]->area == area);
		REQUIRE(hf.spans[0]->next == nullptr);
	}
}

TEST_CASE("rcRasterizeTriangle", "[recast][rasterization]")
{
	rcContext ctx(false);

	constexpr int xSize = 10;
	constexpr int ySize = 10;
	constexpr int zSize = 10;

	constexpr float cellSize = 1.0f;
	constexpr float cellHeight = 1.0f;

	constexpr float minBounds[3] {0.0f, 0.0f, 0.0f};
	constexpr float maxBounds[3] {cellSize * xSize, cellHeight * ySize, cellSize * zSize};

	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, xSize, zSize, minBounds, maxBounds, cellSize, cellHeight));

	SECTION("Simple triangle in XZ plane")
	{
		// Triangle in the XZ plane with vertices (0,0,0), (2,0,0), (0,0,2)
		//
		//   Z ^
		//     |
		//     |\
		//     |_\__> X
		//
		//
		//   Z ....
		//     X...
		//     XX..  X
		//
		// Should fill cells: (0,0), (0,1), (1,0)

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {2.0f, 0.0f, 0.0f};
		float v2[] = {0.0f, 0.0f, 2.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Simple triangle inside a single voxel")
	{
		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {1.0f, 0.0f, 0.0f};
		float v2[] = {0.0f, 0.0f, 1.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangles are clipped by the heightfield bounds")
	{
		// When clipped, this should be the same as the triangle in the previous test
		// Should fill cells: (0,0), (0,1), (1,0)

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {-2.0f, 0.0f, -2.0f};
		float v1[] = {4.0f, 0.0f, -2.0f};
		float v2[] = {-2.0f, 0.0f, 4.0f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 0 && z == 1) || (x == 1 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangle outside of heightfield bounds rasterizes to nothing")
	{
		{ // Outside xz bounds
			float v0[] = {-5.0f, 0.0f, -5.0f};
			float v1[] = {-5.0f, 0.0f, 5.0f};
			float v2[] = {5.0f, 0.0f, -5.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}

		{  // below y bounds
			float v0[] = {0.0f, -1.0f, 0.0f};
			float v1[] = {5.0f, -1.0f, 5.0f};
			float v2[] = {5.0f, -1.0f, 0.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}

		{  // above y bounds
			float v0[] = {0.0f, 40.0f, 0.0f};
			float v1[] = {5.0f, 40.0f, 5.0f};
			float v2[] = {5.0f, 40.0f, 0.0f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					REQUIRE(hf.spans[x + z * hf.width] == nullptr);
				}
			}
		}
	}

	SECTION("Voxels are rasterized if the triangle overlaps any part of it at all")
	{
		// This triangle when clipped barely overlaps the area of the cell at 0,0

		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {-1.0f, 0.0f, -1.0f};
		float v1[] = {1.01f, 0.0f, -1.0f};
		float v2[] = {-1.0f, 0.0f, 1.01f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("A rasterized triangle vertical span includes all the voxels it is in any way part of.")
	{
		// Clockwise winding order so the normal points in the positive Y
		float v0[] = {0.0f, 0.0f, 0.0f};
		float v1[] = {0.5f, 0.0f, 0.5f};
		float v2[] = {0.5f, 2.01f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 3);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Sloped triangle produces varying span heights")
	{
		//   Y ^
		//    2|     /v2
		//     |    /  \
		//    1|   /    \
		//     |  /      \
		//   --+-v0------v1----> X
		//     |  0   2   4
		//
		//   Should rasterize to...
		//
		//   Y ^
		//    2|  |--|--|
		//     |  |     |
		//    1|--|  +  |--|
		//     |           |
		//   --+--------------> X
		//     |
		//

		float v0[] = {0.0f, 0.0f, 0.5f};
		float v1[] = {4.0f, 0.0f, 0.5f};
		float v2[] = {2.0f, 2.0f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if ((x == 0 && z == 0) || (x == 3 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 1);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else if ((x == 1 && z == 0) || (x == 2 && z == 0))
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 2);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangle crossing Y bounds gets clamped")
	{
		// This trangle should be clipped to both the max and min Y bounds of the heightfield.
		float v0[] = {0.0f, -5.0f, 0.0f};
		float v1[] = {1.0f, -5.0f, 1.0f};
		float v2[] = {0.5f, 15.0f, 0.5f};

		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that only expected cells have spans
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				rcSpan* span = hf.spans[x + z * hf.width];
				if (x == 0 && z == 0)
				{
					REQUIRE(span != nullptr);
					REQUIRE(span->smin == 0);
					REQUIRE(span->smax == 10);
					REQUIRE(span->area == RC_WALKABLE_AREA);
					REQUIRE(span->next == nullptr);
				}
				else
				{
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Degenerate triangles rasterize to nothing")
	{
		SKIP("Currently Recast rasterizes degenerate triangles as if they were a line or point of non-zero volume.");
		{ // Co-linear points
			float v0[] = {1.0f, 0.0f, 0.5f};
			float v1[] = {2.0f, 0.0f, 0.5f};
			float v2[] = {4.0f, 0.0f, 0.5f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					rcSpan* span = hf.spans[x + z * hf.width];
					REQUIRE(span == nullptr);
				}
			}
		}

		{ // All vertices are the same point
			float v0[] = {0.5f, 0.0f, 0.5f};
			REQUIRE(rcRasterizeTriangle(&ctx, v0, v0, v0, RC_WALKABLE_AREA, hf, 1));

			// Check that no spans were added
			for (int x = 0; x < xSize; x++)
			{
				for (int z = 0; z < zSize; z++)
				{
					rcSpan* span = hf.spans[x + z * hf.width];
					REQUIRE(span == nullptr);
				}
			}
		}
	}

	SECTION("Triangles outside the heightfield with bounding boxes that overlap the heightfield rasterize nothing")
	{
		// This is a minimal repro case for the issue fixed in PR #476 (https://github.com/recastnavigation/recastnavigation/pull/476)
		float v0[] = {-10.0, 5.5, -10.0};
		float v1[] = {-10.0, 5.5, 3};
		float v2[] = {3.0, 5.5, -10.0};
		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));

		// Check that no spans were added
		for (int x = 0; x < xSize; x++)
		{
			for (int z = 0; z < zSize; z++)
			{
				REQUIRE(hf.spans[x + z * hf.width] == nullptr);
			}
		}
	}
}

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

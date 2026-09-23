#include <cstring>
#include <vector>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "catch2/catch_amalgamated.hpp"

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
		REQUIRE(out[0] == Catch::Approx(0));
		REQUIRE(out[1] == Catch::Approx(0));
		REQUIRE(out[2] == Catch::Approx(1));

		dtRandomPointInConvexPoly(pts, npts, areas, 0.5f, 1.0f, out);
		REQUIRE(out[0] == Catch::Approx(1.0f / 2));
		REQUIRE(out[1] == Catch::Approx(0));
		REQUIRE(out[2] == Catch::Approx(1.0f / 2));

		dtRandomPointInConvexPoly(pts, npts, areas, 1.0f, 1.0f, out);
		REQUIRE(out[0] == Catch::Approx(1));
		REQUIRE(out[1] == Catch::Approx(0));
		REQUIRE(out[2] == Catch::Approx(0));
	}
}

TEST_CASE("dtNavMesh::addTile validates the tile buffer")
{
	// addTile parses a caller-supplied tile buffer. These inputs are malformed,
	// so the call must fail cleanly instead of reading or writing outside the
	// buffer.
	dtNavMesh mesh;
	dtNavMeshParams params;
	memset(&params, 0, sizeof(params));
	params.orig[0] = 0.0f;
	params.orig[1] = 0.0f;
	params.orig[2] = 0.0f;
	params.tileWidth = 1.0f;
	params.tileHeight = 1.0f;
	params.maxTiles = 8;
	params.maxPolys = 64;
	REQUIRE(dtStatusSucceed(mesh.init(&params)));

	SECTION("a buffer shorter than the header is rejected")
	{
		unsigned char tiny[1] = { 0x44 };
		dtTileRef ref = 0;
		const dtStatus status = mesh.addTile(tiny, 1, 0, 0, &ref);
		REQUIRE(dtStatusFailed(status));
	}

	SECTION("a header whose section layout exceeds the buffer is rejected")
	{
		// A header that declares more vertices than the buffer can hold.
		std::vector<unsigned char> buf(sizeof(dtMeshHeader), 0);
		dtMeshHeader* h = (dtMeshHeader*)buf.data();
		h->magic = DT_NAVMESH_MAGIC;
		h->version = DT_NAVMESH_VERSION;
		h->vertCount = 1 << 20;
		h->polyCount = 1;
		h->maxLinkCount = 1;
		dtTileRef ref = 0;
		const dtStatus status = mesh.addTile(buf.data(), (int)buf.size(), 0, 0, &ref);
		REQUIRE(dtStatusFailed(status));
	}
}

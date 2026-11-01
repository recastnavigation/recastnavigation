#include <string.h>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
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

namespace
{
	// Polygon mesh edge codes, as rcPolyMesh stores them.
	const unsigned short NULL_IDX = 0xffff;
	const unsigned short BORDER = 0x800f;      // solid tile border
	const unsigned short PORTAL_POS_Z = 0x8001; // portal to the tile at +z
	const unsigned short PORTAL_NEG_Z = 0x8003; // portal to the tile at -z

	// Tiles are 10 units square: 20 cells of 0.5 across, heights in cells of 0.2.
	const float TILE_SIZE = 10.0f;

	// A floor across a whole tile at height 2 (10 cells). Vertex order gives edges west, north, east, south.
	const unsigned short FLOOR_VERTS[] = { 0,10,0, 0,10,20, 20,10,20, 20,10,0 };

	unsigned char* buildTile(int tileZ, const unsigned short* verts, int vertCount,
							 const unsigned short* polys, int polyCount, int& dataSize)
	{
		const unsigned short flags[] = { 1, 1 };
		const unsigned char areas[] = { 0, 0 };

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = verts;
		params.vertCount = vertCount;
		params.polys = polys;
		params.polyFlags = flags;
		params.polyAreas = areas;
		params.polyCount = polyCount;
		params.nvp = 6;
		params.tileX = 0;
		params.tileY = tileZ;
		params.bmin[0] = 0.0f;
		params.bmin[1] = 0.0f;
		params.bmin[2] = tileZ * TILE_SIZE;
		params.bmax[0] = TILE_SIZE;
		params.bmax[1] = 5.0f;
		params.bmax[2] = (tileZ + 1) * TILE_SIZE;
		params.walkableHeight = 2.0f;
		params.walkableRadius = 0.6f;
		params.walkableClimb = 0.9f;
		params.cs = 0.5f;
		params.ch = 0.2f;
		params.buildBvTree = true;

		unsigned char* data = 0;
		dataSize = 0;
		REQUIRE(dtCreateNavMeshData(&params, &data, &dataSize));
		return data;
	}

	// Two tiles meeting at z = 10: the given south tile, and a north tile that is one floor polygon.
	dtNavMesh* buildMesh(const unsigned short* southVerts, int southVertCount,
						 const unsigned short* southPolys, int southPolyCount)
	{
		dtNavMeshParams params;
		memset(&params, 0, sizeof(params));
		params.tileWidth = TILE_SIZE;
		params.tileHeight = TILE_SIZE;
		params.maxTiles = 4;
		params.maxPolys = 4;

		dtNavMesh* nav = dtAllocNavMesh();
		REQUIRE(dtStatusSucceed(nav->init(&params)));

		int size = 0;
		unsigned char* south = buildTile(0, southVerts, southVertCount, southPolys, southPolyCount, size);
		REQUIRE(dtStatusSucceed(nav->addTile(south, size, DT_TILE_FREE_DATA, 0, 0)));

		const unsigned short northPolys[] = { 0,1,2,3,NULL_IDX,NULL_IDX, BORDER,BORDER,BORDER,PORTAL_NEG_Z,NULL_IDX,NULL_IDX };
		unsigned char* north = buildTile(1, FLOOR_VERTS, 4, northPolys, 1, size);
		REQUIRE(dtStatusSucceed(nav->addTile(north, size, DT_TILE_FREE_DATA, 0, 0)));
		return nav;
	}

	// The wall segments of the north tile's floor that lie on the tile border (z = 10).
	int borderWalls(dtNavMesh* nav, float* borderSegs)
	{
		dtNavMeshQuery* query = dtAllocNavMeshQuery();
		REQUIRE(dtStatusSucceed(query->init(nav, 64)));
		dtQueryFilter filter;
		const float center[] = { 5.0f, 2.0f, 15.0f };
		const float halfExtents[] = { 1.0f, 2.0f, 1.0f };
		dtPolyRef ref = 0;
		float nearest[3];
		REQUIRE(dtStatusSucceed(query->findNearestPoly(center, halfExtents, &filter, &ref, nearest)));
		REQUIRE(ref != 0);

		float segs[6 * 16];
		int nsegs = 0;
		REQUIRE(dtStatusSucceed(query->getPolyWallSegments(ref, &filter, segs, 0, &nsegs, 16)));
		dtFreeNavMeshQuery(query);

		int n = 0;
		for (int i = 0; i < nsegs; ++i)
		{
			const float* s = &segs[i * 6];
			if (dtAbs(s[2] - TILE_SIZE) < 0.01f && dtAbs(s[5] - TILE_SIZE) < 0.01f)
			{
				dtVcopy(&borderSegs[n * 6], s);
				dtVcopy(&borderSegs[n * 6 + 3], s + 3);
				n++;
			}
		}
		return n;
	}
}

TEST_CASE("dtNavMeshQuery::getPolyWallSegments")
{
	SECTION("A tile border covered by overlapping links is no wall")
	{
		// The south tile has the floor and, under its last 2 units along the border, a ledge 0.8 lower: within
		// the climb, so both link to the north floor's border edge and their link intervals overlap. The ledge
		// comes first, so the edge lists the floor's link (added last) ahead of the ledge's shorter one.
		const unsigned short southVerts[] = {
			0,10,0, 0,10,20, 20,10,20, 20,10,0,
			16,6,16, 16,6,20, 20,6,20, 20,6,16,
		};
		const unsigned short southPolys[] = {
			4,5,6,7,NULL_IDX,NULL_IDX, BORDER,PORTAL_POS_Z,BORDER,BORDER,NULL_IDX,NULL_IDX,
			0,1,2,3,NULL_IDX,NULL_IDX, BORDER,PORTAL_POS_Z,BORDER,BORDER,NULL_IDX,NULL_IDX,
		};
		dtNavMesh* nav = buildMesh(southVerts, 8, southPolys, 2);

		float walls[6 * 16];
		REQUIRE(borderWalls(nav, walls) == 0);
		dtFreeNavMesh(nav);
	}

	SECTION("A tile border only partly covered by a link is a wall on the rest")
	{
		// The south tile's floor reaches only x 0..5 of the border.
		const unsigned short southVerts[] = { 0,10,0, 0,10,20, 10,10,20, 10,10,0 };
		const unsigned short southPolys[] = { 0,1,2,3,NULL_IDX,NULL_IDX, BORDER,PORTAL_POS_Z,BORDER,BORDER,NULL_IDX,NULL_IDX };
		dtNavMesh* nav = buildMesh(southVerts, 4, southPolys, 1);

		float walls[6 * 16];
		REQUIRE(borderWalls(nav, walls) == 1);
		REQUIRE(dtMin(walls[0], walls[3]) == Catch::Approx(5.0f).margin(0.05f));
		REQUIRE(dtMax(walls[0], walls[3]) == Catch::Approx(10.0f).margin(0.05f));
		dtFreeNavMesh(nav);
	}
}

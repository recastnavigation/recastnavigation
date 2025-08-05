//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "SDL.h"
#include "SDL_opengl.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "ChunkyTriMesh.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourTileCache.h"
#include "InputGeom.h"
#include "NavMeshTesterTool.h"
#include "OffMeshConnectionTool.h"
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastDebugDraw.h"
#include "Sample.h"
#include "Sample_TempObstacles.h"
#include "fastlz.h"
#include "imguiHelpers.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

namespace
{
// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
constexpr int EXPECTED_LAYERS_PER_TILE = 4;
constexpr int MAX_LAYERS = 32;

constexpr int TILECACHESET_MAGIC = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T';  //'TSET';
constexpr int TILECACHESET_VERSION = 1;

enum DrawDetailType
{
	DRAWDETAIL_AREAS,
	DRAWDETAIL_REGIONS,
	DRAWDETAIL_CONTOURS,
	DRAWDETAIL_MESH
};

bool intersectSegmentAABB(const float* sp, const float* sq, const float* amin, const float* amax, float& tmin, float& tmax)
{
	static constexpr float EPSILON = 1e-6f;

	float d[3];
	rcVsub(d, sq, sp);
	tmin = 0;        // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;  // set to max distance ray can travel (for segment)

	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPSILON)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
			{
				return false;
			}
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;

			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2)
			{
				rcSwap(t1, t2);
			}

			// Compute the intersection of slab intersections intervals
			tmin = std::max(t1, tmin);
			tmax = std::min(t2, tmax);

			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax)
			{
				return false;
			}
		}
	}

	return true;
}

int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	const int gridSize = gridWidth * gridHeight;
	return headerSize + gridSize * 4;
}

void drawTiles(duDebugDraw* debugDraw, dtTileCache* tileCache)
{
	unsigned int fcol[6];
	float bmin[3];
	float bmax[3];

	for (int i = 0; i < tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tileCache->getTile(i);
		if (!tile->header)
		{
			continue;
		}

		tileCache->calcTightTileBounds(tile->header, bmin, bmax);

		const unsigned int col = duIntToCol(i, 64);
		duCalcBoxColors(fcol, col, col);
		duDebugDrawBox(debugDraw, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], fcol);
	}

	for (int i = 0; i < tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tileCache->getTile(i);
		if (!tile->header)
		{
			continue;
		}

		tileCache->calcTightTileBounds(tile->header, bmin, bmax);

		const float pad = tileCache->getParams()->cs * 0.1f;
		duDebugDrawBoxWire(
			debugDraw,
			bmin[0] - pad,
			bmin[1] - pad,
			bmin[2] - pad,
			bmax[0] + pad,
			bmax[1] + pad,
			bmax[2] + pad,
			duIntToCol(i, 255),
			2.0f);
	}
}

void drawDetail(duDebugDraw* debugDraw, dtTileCache* tileCache, const int tileX, const int tileY, int tileType)
{
	struct TileCacheBuildContext
	{
		dtTileCacheLayer* layer = nullptr;
		dtTileCacheContourSet* lcset = nullptr;
		dtTileCachePolyMesh* lmesh = nullptr;
		dtTileCacheAlloc* alloc = nullptr;

		inline TileCacheBuildContext(struct dtTileCacheAlloc* a) : alloc(a) {}
		inline ~TileCacheBuildContext() { purge(); }

		void purge()
		{
			dtFreeTileCacheLayer(alloc, layer);
			layer = 0;
			dtFreeTileCacheContourSet(alloc, lcset);
			lcset = 0;
			dtFreeTileCachePolyMesh(alloc, lmesh);
			lmesh = 0;
		}
	};

	dtCompressedTileRef tiles[MAX_LAYERS];
	const int ntiles = tileCache->getTilesAt(tileX, tileY, tiles, MAX_LAYERS);

	dtTileCacheAlloc* talloc = tileCache->getAlloc();
	dtTileCacheCompressor* tcomp = tileCache->getCompressor();
	const dtTileCacheParams* params = tileCache->getParams();

	for (int i = 0; i < ntiles; ++i)
	{
		const dtCompressedTile* tile = tileCache->getTileByRef(tiles[i]);

		talloc->reset();

		TileCacheBuildContext bc{talloc};
		const int walkableClimbVx = (int)(params->walkableClimb / params->ch);
		dtStatus status;

		// Decompress tile layer data.
		status = dtDecompressTileCacheLayer(talloc, tcomp, tile->data, tile->dataSize, &bc.layer);
		if (dtStatusFailed(status))
		{
			return;
		}
		if (tileType == DRAWDETAIL_AREAS)
		{
			duDebugDrawTileCacheLayerAreas(debugDraw, *bc.layer, params->cs, params->ch);
			continue;
		}

		// Build navmesh
		status = dtBuildTileCacheRegions(talloc, *bc.layer, walkableClimbVx);
		if (dtStatusFailed(status))
		{
			return;
		}
		if (tileType == DRAWDETAIL_REGIONS)
		{
			duDebugDrawTileCacheLayerRegions(debugDraw, *bc.layer, params->cs, params->ch);
			continue;
		}

		bc.lcset = dtAllocTileCacheContourSet(talloc);
		if (!bc.lcset)
		{
			return;
		}
		status = dtBuildTileCacheContours(talloc, *bc.layer, walkableClimbVx, params->maxSimplificationError, *bc.lcset);
		if (dtStatusFailed(status))
		{
			return;
		}
		if (tileType == DRAWDETAIL_CONTOURS)
		{
			duDebugDrawTileCacheContours(debugDraw, *bc.lcset, tile->header->bmin, params->cs, params->ch);
			continue;
		}

		bc.lmesh = dtAllocTileCachePolyMesh(talloc);
		if (!bc.lmesh)
		{
			return;
		}
		status = dtBuildTileCachePolyMesh(talloc, *bc.lcset, *bc.lmesh);
		if (dtStatusFailed(status))
		{
			return;
		}

		if (tileType == DRAWDETAIL_MESH)
		{
			duDebugDrawTileCachePolyMesh(debugDraw, *bc.lmesh, tile->header->bmin, params->cs, params->ch);
			continue;
		}
	}
}

void drawDetailOverlay(const dtTileCache* tileCache, const int tileX, const int tileY, double* proj, double* model, int* view)
{
	dtCompressedTileRef tiles[MAX_LAYERS];
	const int ntiles = tileCache->getTilesAt(tileX, tileY, tiles, MAX_LAYERS);
	if (!ntiles)
	{
		return;
	}

	const int rawSize = calcLayerBufferSize(tileCache->getParams()->width, tileCache->getParams()->height);

	for (int i = 0; i < ntiles; ++i)
	{
		const dtCompressedTile* tile = tileCache->getTileByRef(tiles[i]);

		float pos[3];
		pos[0] = (tile->header->bmin[0] + tile->header->bmax[0]) / 2.0f;
		pos[1] = tile->header->bmin[1];
		pos[2] = (tile->header->bmin[2] + tile->header->bmax[2]) / 2.0f;

		GLdouble x, y, z;
		if (gluProject(
				static_cast<GLdouble>(pos[0]),
				static_cast<GLdouble>(pos[1]),
				static_cast<GLdouble>(pos[2]),
				model,
				proj,
				view,
				&x,
				&y,
				&z))
		{
			char text[128];
			snprintf(text, 128, "(%d,%d)/%d", tile->header->tx, tile->header->ty, tile->header->tlayer);
			DrawScreenspaceText(static_cast<float>(x), static_cast<float>(y) - 25, IM_COL32(0, 0, 0, 220), text, true);
			snprintf(text, 128, "Compressed: %.1f kB", static_cast<float>(tile->dataSize) / 1024.0f);
			DrawScreenspaceText(static_cast<float>(x), static_cast<float>(y) - 45, IM_COL32(0, 0, 0, 128), text, true);
			snprintf(text, 128, "Raw:%.1fkB", static_cast<float>(rawSize) / 1024.0f);
			DrawScreenspaceText(static_cast<float>(x), static_cast<float>(y) - 65, IM_COL32(0, 0, 0, 128), text, true);
		}
	}
}

dtObstacleRef hitTestObstacle(const dtTileCache* tileCache, const float* sp, const float* sq)
{
	float tmin = FLT_MAX;
	const dtTileCacheObstacle* obmin = 0;
	for (int obstacleIndex = 0; obstacleIndex < tileCache->getObstacleCount(); ++obstacleIndex)
	{
		const dtTileCacheObstacle* ob = tileCache->getObstacle(obstacleIndex);
		if (ob->state == DT_OBSTACLE_EMPTY)
		{
			continue;
		}

		float bmin[3], bmax[3], t0, t1;
		tileCache->getObstacleBounds(ob, bmin, bmax);

		if (intersectSegmentAABB(sp, sq, bmin, bmax, t0, t1))
		{
			if (t0 < tmin)
			{
				tmin = t0;
				obmin = ob;
			}
		}
	}
	return tileCache->getObstacleRef(obmin);
}

void drawObstacles(duDebugDraw* dd, const dtTileCache* tileCache)
{
	// Draw obstacles
	for (int i = 0; i < tileCache->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* obstacle = tileCache->getObstacle(i);
		if (obstacle->state == DT_OBSTACLE_EMPTY)
		{
			continue;
		}

		float bmin[3];
		float bmax[3];
		tileCache->getObstacleBounds(obstacle, bmin, bmax);

		unsigned int col = 0;
		if (obstacle->state == DT_OBSTACLE_PROCESSING)
		{
			col = duRGBA(255, 255, 0, 128);
		}
		else if (obstacle->state == DT_OBSTACLE_PROCESSED)
		{
			col = duRGBA(255, 192, 0, 192);
		}
		else if (obstacle->state == DT_OBSTACLE_REMOVING)
		{
			col = duRGBA(220, 0, 0, 128);
		}

		duDebugDrawCylinder(dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], col);
		duDebugDrawCylinderWire(dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], duDarkenCol(col), 2);
	}
}

}

struct FastLZCompressor : dtTileCacheCompressor
{
	~FastLZCompressor() override = default;

	int maxCompressedSize(const int bufferSize) override { return static_cast<int>(static_cast<float>(bufferSize) * 1.05f); }

	dtStatus compress(
		const unsigned char* buffer,
		const int bufferSize,
		unsigned char* compressed,
		const int /*maxCompressedSize*/,
		int* compressedSize) override
	{
		*compressedSize = fastlz_compress(buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	dtStatus decompress(
		const unsigned char* compressed,
		const int compressedSize,
		unsigned char* buffer,
		const int maxBufferSize,
		int* bufferSize) override
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct LinearAllocator : dtTileCacheAlloc
{
	unsigned char* buffer = nullptr;
	size_t capacity = 0;
	size_t top = 0;
	size_t high = 0;

	explicit LinearAllocator(const size_t cap) { resize(cap); }

	~LinearAllocator() override { dtFree(buffer); }

	void resize(const size_t cap)
	{
		if (buffer)
		{
			dtFree(buffer);
		}
		buffer = static_cast<unsigned char*>(dtAlloc(cap, DT_ALLOC_PERM));
		capacity = cap;
	}

	void reset() override
	{
		high = dtMax(high, top);
		top = 0;
	}

	void* alloc(const size_t size) override
	{
		if (!buffer)
		{
			return 0;
		}
		if (top + size > capacity)
		{
			return 0;
		}

		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	void free(void* /*ptr*/) override {}
};

struct MeshProcess : dtTileCacheMeshProcess
{
	InputGeom* inputGeometry = nullptr;

	~MeshProcess() override = default;

	void init(InputGeom* geom) { inputGeometry = geom; }

	void process(dtNavMeshCreateParams* params, unsigned char* polyAreas, unsigned short* polyFlags) override
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
			{
				polyAreas[i] = SAMPLE_POLYAREA_GROUND;
			}

			if (polyAreas[i] == SAMPLE_POLYAREA_GROUND || polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
			    polyAreas[i] == SAMPLE_POLYAREA_ROAD)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
			{
				polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}

		// Pass in off-mesh connections.
		if (inputGeometry)
		{
			params->offMeshConVerts = inputGeometry->getOffMeshConnectionVerts();
			params->offMeshConRad = inputGeometry->getOffMeshConnectionRads();
			params->offMeshConDir = inputGeometry->getOffMeshConnectionDirs();
			params->offMeshConAreas = inputGeometry->offMeshConAreas;
			params->offMeshConFlags = inputGeometry->offMeshConFlags;
			params->offMeshConUserID = inputGeometry->offMeshConId;
			params->offMeshConCount = inputGeometry->offMeshConCount;
		}
	}
};

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

struct RasterizationContext
{
	rcHeightfield* solid = nullptr;
	unsigned char* triareas = nullptr;
	rcHeightfieldLayerSet* lset = nullptr;
	rcCompactHeightfield* chf = nullptr;
	TileCacheData tiles[MAX_LAYERS]{};
	int ntiles = 0;

	RasterizationContext() { memset(tiles, 0, sizeof(TileCacheData) * MAX_LAYERS); }

	~RasterizationContext()
	{
		rcFreeHeightField(solid);
		delete[] triareas;
		rcFreeHeightfieldLayerSet(lset);
		rcFreeCompactHeightfield(chf);
		for (int i = 0; i < MAX_LAYERS; ++i)
		{
			dtFree(tiles[i].data);
			tiles[i].data = 0;
		}
	}
};

int Sample_TempObstacles::rasterizeTileLayers(
	const int tileX,
	const int tileY,
	const rcConfig& cfg,
	TileCacheData* tiles,
	const int maxTiles) const
{
	if (!inputGeometry || inputGeometry->getVertCount() == 0 || !inputGeometry->getChunkyMesh())
	{
		buildContext->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
		return 0;
	}

	FastLZCompressor comp;
	RasterizationContext rasterContext;

	const float* verts = inputGeometry->verts.data();
	const int nverts = inputGeometry->getVertCount();
	const ChunkyTriMesh* chunkyMesh = inputGeometry->getChunkyMesh();

	// Tile bounds.
	const float tcs = cfg.tileSize * cfg.cs;

	rcConfig tcfg;
	memcpy(&tcfg, &cfg, sizeof(tcfg));

	tcfg.bmin[0] = cfg.bmin[0] + tileX * tcs;
	tcfg.bmin[1] = cfg.bmin[1];
	tcfg.bmin[2] = cfg.bmin[2] + tileY * tcs;
	tcfg.bmax[0] = cfg.bmin[0] + (tileX + 1) * tcs;
	tcfg.bmax[1] = cfg.bmax[1];
	tcfg.bmax[2] = cfg.bmin[2] + (tileY + 1) * tcs;
	tcfg.bmin[0] -= static_cast<float>(tcfg.borderSize) * tcfg.cs;
	tcfg.bmin[2] -= static_cast<float>(tcfg.borderSize) * tcfg.cs;
	tcfg.bmax[0] += static_cast<float>(tcfg.borderSize) * tcfg.cs;
	tcfg.bmax[2] += static_cast<float>(tcfg.borderSize) * tcfg.cs;

	// Allocate voxel heightfield where we rasterize our input data to.
	rasterContext.solid = rcAllocHeightfield();
	if (!rasterContext.solid)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(
			buildContext,
			*rasterContext.solid,
			tcfg.width,
			tcfg.height,
			tcfg.bmin,
			tcfg.bmax,
			tcfg.cs,
			tcfg.ch))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	rasterContext.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!rasterContext.triareas)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}

	float tbmin[2];
	float tbmax[2];
	tbmin[0] = tcfg.bmin[0];
	tbmin[1] = tcfg.bmin[2];
	tbmax[0] = tcfg.bmax[0];
	tbmax[1] = tcfg.bmax[2];
	int cid[512];  // TODO: Make grow when returning too many items.
	const int ncid = chunkyMesh->GetChunksOverlappingRect(tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0;  // empty
	}

	for (int i = 0; i < ncid; ++i)
	{
		const ChunkyTriMesh::Node& node = chunkyMesh->nodes[cid[i]];
		const int* tris = &chunkyMesh->tris[node.i * 3];
		const int ntris = node.n;

		memset(rasterContext.triareas, 0, ntris * sizeof(unsigned char));
		rcMarkWalkableTriangles(buildContext, tcfg.walkableSlopeAngle, verts, nverts, tris, ntris, rasterContext.triareas);
		if (!rcRasterizeTriangles(
				buildContext,
				verts,
				nverts,
				tris,
				rasterContext.triareas,
				ntris,
				*rasterContext.solid,
				tcfg.walkableClimb))
		{
			return 0;
		}
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	if (filterLowHangingObstacles)
	{
		rcFilterLowHangingWalkableObstacles(buildContext, tcfg.walkableClimb, *rasterContext.solid);
	}
	if (filterLedgeSpans)
	{
		rcFilterLedgeSpans(buildContext, tcfg.walkableHeight, tcfg.walkableClimb, *rasterContext.solid);
	}
	if (filterWalkableLowHeightSpans)
	{
		rcFilterWalkableLowHeightSpans(buildContext, tcfg.walkableHeight, *rasterContext.solid);
	}

	rasterContext.chf = rcAllocCompactHeightfield();
	if (!rasterContext.chf)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return 0;
	}
	if (!rcBuildCompactHeightfield(
			buildContext,
			tcfg.walkableHeight,
			tcfg.walkableClimb,
			*rasterContext.solid,
			*rasterContext.chf))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return 0;
	}

	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(buildContext, tcfg.walkableRadius, *rasterContext.chf))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return 0;
	}

	// (Optional) Mark areas.
	const ConvexVolume* vols = inputGeometry->getConvexVolumes();
	for (int i = 0; i < inputGeometry->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(
			buildContext,
			vols[i].verts,
			vols[i].nverts,
			vols[i].hmin,
			vols[i].hmax,
			static_cast<unsigned char>(vols[i].area),
			*rasterContext.chf);
	}

	rasterContext.lset = rcAllocHeightfieldLayerSet();
	if (!rasterContext.lset)
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
		return 0;
	}
	if (!rcBuildHeightfieldLayers(buildContext, *rasterContext.chf, tcfg.borderSize, tcfg.walkableHeight, *rasterContext.lset))
	{
		buildContext->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
		return 0;
	}

	rasterContext.ntiles = 0;
	for (int i = 0; i < rcMin(rasterContext.lset->nlayers, MAX_LAYERS); ++i)
	{
		TileCacheData* tile = &rasterContext.tiles[rasterContext.ntiles++];
		const rcHeightfieldLayer* layer = &rasterContext.lset->layers[i];

		// Store header
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;

		// Tile layer location in the navmesh.
		header.tx = tileX;
		header.ty = tileY;
		header.tlayer = i;
		dtVcopy(header.bmin, layer->bmin);
		dtVcopy(header.bmax, layer->bmax);

		// Tile info.
		header.width = static_cast<unsigned char>(layer->width);
		header.height = static_cast<unsigned char>(layer->height);
		header.minx = static_cast<unsigned char>(layer->minx);
		header.maxx = static_cast<unsigned char>(layer->maxx);
		header.miny = static_cast<unsigned char>(layer->miny);
		header.maxy = static_cast<unsigned char>(layer->maxy);
		header.hmin = static_cast<unsigned short>(layer->hmin);
		header.hmax = static_cast<unsigned short>(layer->hmax);

		dtStatus status =
			dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons, &tile->data, &tile->dataSize);
		if (dtStatusFailed(status))
		{
			return 0;
		}
	}

	// Transfer ownsership of tile data from build context to the caller.
	int n = 0;
	for (int i = 0; i < rcMin(rasterContext.ntiles, maxTiles); ++i)
	{
		tiles[n++] = rasterContext.tiles[i];
		rasterContext.tiles[i].data = 0;
		rasterContext.tiles[i].dataSize = 0;
	}

	return n;
}

class TempObstacleHighlightTool : public SampleTool
{
	Sample_TempObstacles* m_sample = nullptr;
	float m_hitPos[3] = {0, 0, 0};
	bool m_hitPosSet = false;
	int m_drawType = DRAWDETAIL_AREAS;

public:
	~TempObstacleHighlightTool() override = default;

	SampleToolType type() override { return SampleToolType::TILE_HIGHLIGHT; }

	void init(Sample* sample) override { m_sample = static_cast<Sample_TempObstacles*>(sample); }

	void reset() override {}

	void handleMenu() override
	{
		ImGui::Text("Highlight Tile Cache");
		ImGui::Text("Click LMB to highlight a tile.");
		ImGui::Separator();
		if (ImGui::RadioButton("Draw Areas", m_drawType == DRAWDETAIL_AREAS))
		{
			m_drawType = DRAWDETAIL_AREAS;
		}
		if (ImGui::RadioButton("Draw Regions", m_drawType == DRAWDETAIL_REGIONS))
		{
			m_drawType = DRAWDETAIL_REGIONS;
		}
		if (ImGui::RadioButton("Draw Contours", m_drawType == DRAWDETAIL_CONTOURS))
		{
			m_drawType = DRAWDETAIL_CONTOURS;
		}
		if (ImGui::RadioButton("Draw Mesh", m_drawType == DRAWDETAIL_MESH))
		{
			m_drawType = DRAWDETAIL_MESH;
		}
	}

	void handleClick(const float* /*s*/, const float* p, bool /*shift*/) override
	{
		m_hitPosSet = true;
		rcVcopy(m_hitPos, p);
	}

	void handleToggle() override {}

	void handleStep() override {}

	void handleUpdate(const float /*dt*/) override {}

	void handleRender() override
	{
		if (m_hitPosSet && m_sample)
		{
			const float s = m_sample->agentRadius;
			glColor4ub(0, 0, 0, 128);
			glLineWidth(2.0f);
			glBegin(GL_LINES);
			glVertex3f(m_hitPos[0] - s, m_hitPos[1] + 0.1f, m_hitPos[2]);
			glVertex3f(m_hitPos[0] + s, m_hitPos[1] + 0.1f, m_hitPos[2]);
			glVertex3f(m_hitPos[0], m_hitPos[1] - s + 0.1f, m_hitPos[2]);
			glVertex3f(m_hitPos[0], m_hitPos[1] + s + 0.1f, m_hitPos[2]);
			glVertex3f(m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2] - s);
			glVertex3f(m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2] + s);
			glEnd();
			glLineWidth(1.0f);

			int tileX = 0, tileY = 0;
			m_sample->getTilePos(m_hitPos, tileX, tileY);
			m_sample->renderCachedTile(tileX, tileY, m_drawType);
		}
	}

	void handleRenderOverlay(double* proj, double* model, int* view) override
	{
		if (m_hitPosSet)
		{
			if (m_sample)
			{
				int tileX = 0, tileY = 0;
				m_sample->getTilePos(m_hitPos, tileX, tileY);
				m_sample->renderCachedTileOverlay(tileX, tileY, proj, model, view);
			}
		}
	}
};

class TempObstacleCreateTool : public SampleTool
{
	Sample_TempObstacles* m_sample = nullptr;

public:
	~TempObstacleCreateTool() override = default;

	SampleToolType type() override { return SampleToolType::TEMP_OBSTACLE; }

	void init(Sample* sample) override { m_sample = static_cast<Sample_TempObstacles*>(sample); }

	void reset() override {}

	void handleMenu() override
	{
		ImGui::Text("Create Temp Obstacles");

		if (ImGui::Button("Remove All"))
		{
			m_sample->clearAllTempObstacles();
		}

		ImGui::Separator();

		ImGui::Text("Click LMB to create an obstacle.");
		ImGui::Text("Shift+LMB to remove an obstacle.");
	}

	void handleClick(const float* s, const float* p, bool shift) override
	{
		if (m_sample)
		{
			if (shift)
			{
				m_sample->removeTempObstacle(s, p);
			}
			else
			{
				m_sample->addTempObstacle(p);
			}
		}
	}

	void handleToggle() override {}
	void handleStep() override {}
	void handleUpdate(const float /*dt*/) override {}
	void handleRender() override {}
	void handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) override {}
};

Sample_TempObstacles::Sample_TempObstacles()
{
	resetCommonSettings();

	tAllocator = new LinearAllocator(32000);
	tCompressor = new FastLZCompressor;
	tMeshProcess = new MeshProcess;

	setTool(new TempObstacleCreateTool);
}

Sample_TempObstacles::~Sample_TempObstacles()
{
	dtFreeNavMesh(navMesh);
	navMesh = 0;
	dtFreeTileCache(tileCache);
}

void Sample_TempObstacles::drawSettingsUI()
{
	drawCommonSettingsUI();
	ImGui::Checkbox("Keep Itermediate Results", &keepIntermediateResults);

	ImGui::Text("Tiling");
	if (ImGui::SliderInt("TileSize", &tileSize, 16, 128))
	{
		// Snap to multiples of 8
		tileSize = static_cast<int>(roundf(static_cast<float>(tileSize) / 8.0f)) * 8;
	}

	int gridSize = 1;
	if (inputGeometry)
	{
		const float* minBounds = inputGeometry->getNavMeshBoundsMin();
		const float* maxBounds = inputGeometry->getNavMeshBoundsMax();
		int gw = 0;
		int gh = 0;
		rcCalcGridSize(minBounds, maxBounds, cellSize, &gw, &gh);
		const int tw = (gw + tileSize - 1) / tileSize;
		const int th = (gh + tileSize - 1) / tileSize;
		ImGui::Text("Tiles  %d x %d", tw, th);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin(static_cast<int>(dtIlog2(dtNextPow2(tw * th * EXPECTED_LAYERS_PER_TILE))), 14);
		tileBits = std::min(tileBits, 14);
		int polyBits = 22 - tileBits;
		maxTiles = 1 << tileBits;
		maxPolysPerTile = 1 << polyBits;
		ImGui::Text("Max Tiles  %d", maxTiles);
		ImGui::Text("Max Polys  %d", maxPolysPerTile);
		gridSize = tw * th;
	}
	else
	{
		maxTiles = 0;
		maxPolysPerTile = 0;
	}

	ImGui::Separator();

	ImGui::Text("Tile Cache");

	const float compressionRatio = (float)cacheCompressedSize / (float)(cacheRawSize + 1);

	ImGui::Text("Layers  %d", cacheLayerCount);
	ImGui::Text("Layers (per tile)  %.1f", (float)cacheLayerCount / (float)gridSize);

	ImGui::Text(
		"Memory  %.1f kB / %.1f kB (%.1f%%)",
		static_cast<float>(cacheCompressedSize) / 1024.0f,
		static_cast<float>(cacheRawSize) / 1024.0f,
		compressionRatio * 100.0f);
	ImGui::Text("Navmesh Build Time  %.1f ms", cacheBuildTimeMs);
	ImGui::Text("Build Peak Mem Usage  %.1f kB", static_cast<float>(cacheBuildMemUsage) / 1024.0f);

	ImGui::Separator();

	ImGui::Indent();

	if (ImGui::Button("Save"))
	{
		saveAll("all_tiles_tilecache.bin");
	}

	if (ImGui::Button("Load"))
	{
		dtFreeNavMesh(navMesh);
		dtFreeTileCache(tileCache);
		loadAll("all_tiles_tilecache.bin");
		navQuery->init(navMesh, 2048);
	}

	ImGui::Unindent();

	ImGui::Separator();
}

void Sample_TempObstacles::drawToolsUI()
{
	const SampleToolType type = !tool ? SampleToolType::NONE : tool->type();

	if (ImGui::RadioButton("Test Navmesh", type == SampleToolType::NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (ImGui::RadioButton("Highlight Tile Cache", type == SampleToolType::TILE_HIGHLIGHT))
	{
		setTool(new TempObstacleHighlightTool);
	}
	if (ImGui::RadioButton("Create Temp Obstacles", type == SampleToolType::TEMP_OBSTACLE))
	{
		setTool(new TempObstacleCreateTool);
	}
	if (ImGui::RadioButton("Create Off-Mesh Links", type == SampleToolType::OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (ImGui::RadioButton("Create Convex Volumes", type == SampleToolType::CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (ImGui::RadioButton("Create Crowds", type == SampleToolType::CROWD))
	{
		setTool(new CrowdTool);
	}

	ImGui::Separator();

	if (tool)
	{
		tool->handleMenu();
	}
}

void Sample_TempObstacles::drawDebugUI()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
	{
		valid[i] = false;
	}

	if (inputGeometry)
	{
		valid[DRAWMODE_NAVMESH] = navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = navQuery != 0;
		valid[DRAWMODE_NAVMESH_PORTALS] = navMesh != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_CACHE_BOUNDS] = true;
	}

	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
	{
		if (!valid[i])
		{
			unavail++;
		}
	}

	if (unavail == MAX_DRAWMODE)
	{
		return;
	}

	ImGui::Text("Draw");
	ImGui::BeginDisabled(!valid[DRAWMODE_MESH]);
	if (ImGui::RadioButton("Input Mesh", drawMode == DRAWMODE_MESH))
	{
		drawMode = DRAWMODE_MESH;
	}
	ImGui::EndDisabled();
	ImGui::BeginDisabled(!valid[DRAWMODE_NAVMESH]);
	if (ImGui::RadioButton("Navmesh", drawMode == DRAWMODE_NAVMESH))
	{
		drawMode = DRAWMODE_NAVMESH;
	}
	ImGui::EndDisabled();
	ImGui::BeginDisabled(!valid[DRAWMODE_NAVMESH_INVIS]);
	if (ImGui::RadioButton("Navmesh Invis", drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		drawMode = DRAWMODE_NAVMESH_INVIS;
	}
	ImGui::EndDisabled();
	ImGui::BeginDisabled(!valid[DRAWMODE_NAVMESH_TRANS]);
	if (ImGui::RadioButton("Navmesh Trans", drawMode == DRAWMODE_NAVMESH_TRANS))
	{
		drawMode = DRAWMODE_NAVMESH_TRANS;
	}
	ImGui::EndDisabled();
	ImGui::BeginDisabled(!valid[DRAWMODE_NAVMESH_BVTREE]);
	if (ImGui::RadioButton("Navmesh BVTree", drawMode == DRAWMODE_NAVMESH_BVTREE))
	{
		drawMode = DRAWMODE_NAVMESH_BVTREE;
	}
	ImGui::EndDisabled();
	ImGui::BeginDisabled(!valid[DRAWMODE_NAVMESH_NODES]);
	if (ImGui::RadioButton("Navmesh Nodes", drawMode == DRAWMODE_NAVMESH_NODES))
	{
		drawMode = DRAWMODE_NAVMESH_NODES;
	}
	ImGui::EndDisabled();
	ImGui::BeginDisabled(!valid[DRAWMODE_NAVMESH_PORTALS]);
	if (ImGui::RadioButton("Navmesh Portals", drawMode == DRAWMODE_NAVMESH_PORTALS))
	{
		drawMode = DRAWMODE_NAVMESH_PORTALS;
	}
	ImGui::EndDisabled();
	ImGui::BeginDisabled(!valid[DRAWMODE_CACHE_BOUNDS]);
	if (ImGui::RadioButton("Cache Bounds", drawMode == DRAWMODE_CACHE_BOUNDS))
	{
		drawMode = DRAWMODE_CACHE_BOUNDS;
	}
	ImGui::EndDisabled();

	if (unavail)
	{
		ImGui::Text("Tick 'Keep Itermediate Results'");
		ImGui::Text("rebuild some tiles to see");
		ImGui::Text("more debug mode options.");
	}
}

void Sample_TempObstacles::render()
{
	if (!inputGeometry || inputGeometry->getVertCount() == 0)
	{
		return;
	}

	const float texScale = 1.0f / (cellSize * 10.0f);

	// Draw mesh
	if (drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(
			&debugDraw,
			inputGeometry->verts.data(),
			inputGeometry->getVertCount(),
			inputGeometry->tris.data(),
			inputGeometry->normals.data(),
			inputGeometry->getTriCount(),
			agentMaxSlope,
			texScale);
		inputGeometry->drawOffMeshConnections(&debugDraw);
	}

	if (tileCache && drawMode == DRAWMODE_CACHE_BOUNDS)
	{
		drawTiles(&debugDraw, tileCache);
	}

	if (tileCache)
	{
		drawObstacles(&debugDraw, tileCache);
	}

	glDepthMask(GL_FALSE);

	// Draw bounds
	const float* minBounds = inputGeometry->getNavMeshBoundsMin();
	const float* maxBounds = inputGeometry->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&debugDraw, minBounds[0], minBounds[1], minBounds[2], maxBounds[0], maxBounds[1], maxBounds[2], duRGBA(255, 255, 255, 128), 1.0f);

	// Tiling grid.
	int gw = 0;
	int gh = 0;
	rcCalcGridSize(minBounds, maxBounds, cellSize, &gw, &gh);
	const int tw = (gw + tileSize - 1) / tileSize;
	const int th = (gh + tileSize - 1) / tileSize;
	const float s = static_cast<float>(tileSize) * cellSize;
	duDebugDrawGridXZ(&debugDraw, minBounds[0], minBounds[1], minBounds[2], tw, th, s, duRGBA(0, 0, 0, 64), 1.0f);

	if (navMesh && navQuery &&
	    (drawMode == DRAWMODE_NAVMESH || drawMode == DRAWMODE_NAVMESH_TRANS || drawMode == DRAWMODE_NAVMESH_BVTREE ||
	     drawMode == DRAWMODE_NAVMESH_NODES || drawMode == DRAWMODE_NAVMESH_PORTALS ||
	     drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (drawMode != DRAWMODE_NAVMESH_INVIS)
		{
			duDebugDrawNavMeshWithClosedList(&debugDraw, *navMesh, *navQuery, navMeshDrawFlags /*|DU_DRAWNAVMESH_COLOR_TILES*/);
		}
		if (drawMode == DRAWMODE_NAVMESH_BVTREE)
		{
			duDebugDrawNavMeshBVTree(&debugDraw, *navMesh);
		}
		if (drawMode == DRAWMODE_NAVMESH_PORTALS)
		{
			duDebugDrawNavMeshPortals(&debugDraw, *navMesh);
		}
		if (drawMode == DRAWMODE_NAVMESH_NODES)
		{
			duDebugDrawNavMeshNodes(&debugDraw, *navQuery);
		}
		duDebugDrawNavMeshPolysWithFlags(&debugDraw, *navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0, 0, 0, 128));
	}

	glDepthMask(GL_TRUE);

	inputGeometry->drawConvexVolumes(&debugDraw);

	if (tool)
	{
		tool->handleRender();
	}
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_TempObstacles::renderCachedTile(const int tileX, const int tileY, const int type)
{
	if (tileCache)
	{
		drawDetail(&debugDraw, tileCache, tileX, tileY, type);
	}
}

void Sample_TempObstacles::renderCachedTileOverlay(const int tileX, const int tileY, double* proj, double* model, int* view) const
{
	if (tileCache)
	{
		drawDetailOverlay(tileCache, tileX, tileY, proj, model, view);
	}
}

void Sample_TempObstacles::renderOverlay(double* proj, double* model, int* view)
{
	if (tool)
	{
		tool->handleRenderOverlay(proj, model, view);
	}
	renderOverlayToolStates(proj, model, view);

	// Stats
	/*	imguiDrawRect(280,10,300,100,imguiRGBA(0,0,0,64));

	    char text[64];
	    int y = 110-30;

	    snprintf(text,64,"Lean Data: %.1fkB", m_tileCache->getRawSize()/1024.0f);
	    imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	    y -= 20;

	    snprintf(text,64,"Compressed: %.1fkB (%.1f%%)", m_tileCache->getCompressedSize()/1024.0f,
	             m_tileCache->getRawSize() > 0 ? 100.0f*(float)m_tileCache->getCompressedSize()/(float)m_tileCache->getRawSize()
	   : 0); imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255)); y -= 20;

	    if (m_rebuildTileCount > 0 && m_rebuildTime > 0.0f)
	    {
	        snprintf(text,64,"Changed obstacles, rebuild %d tiles: %.3f ms", m_rebuildTileCount, m_rebuildTime);
	        imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,192,0,255));
	        y -= 20;
	    }
	    */
}

void Sample_TempObstacles::onMeshChanged(InputGeom* geom)
{
	Sample::onMeshChanged(geom);

	dtFreeTileCache(tileCache);
	tileCache = 0;

	dtFreeNavMesh(navMesh);
	navMesh = 0;

	if (tool)
	{
		tool->reset();
		tool->init(this);
		tMeshProcess->init(inputGeometry);
	}
	resetToolStates();
	initToolStates(this);
}

void Sample_TempObstacles::addTempObstacle(const float* pos) const
{
	if (!tileCache)
	{
		return;
	}

	float p[3];
	dtVcopy(p, pos);
	p[1] -= 0.5f;
	tileCache->addObstacle(p, 1.0f, 2.0f, 0);
}

void Sample_TempObstacles::removeTempObstacle(const float* sp, const float* sq) const
{
	if (!tileCache)
	{
		return;
	}

	tileCache->removeObstacle(hitTestObstacle(tileCache, sp, sq));
}

void Sample_TempObstacles::clearAllTempObstacles() const
{
	if (!tileCache)
	{
		return;
	}

	for (int i = 0; i < tileCache->getObstacleCount(); ++i)
	{
		const dtTileCacheObstacle* obstacle = tileCache->getObstacle(i);
		if (obstacle->state == DT_OBSTACLE_EMPTY)
		{
			continue;
		}

		tileCache->removeObstacle(tileCache->getObstacleRef(obstacle));
	}
}

bool Sample_TempObstacles::build()
{
	dtStatus status;

	if (!inputGeometry || inputGeometry->getVertCount() == 0)
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}

	tMeshProcess->init(inputGeometry);

	// Init cache
	const float* minBounds = inputGeometry->getNavMeshBoundsMin();
	const float* maxBounds = inputGeometry->getNavMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(minBounds, maxBounds, cellSize, &gw, &gh);
	const int ts = tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	// Generation params.
	rcConfig cfg = {};
	cfg.cs = cellSize;
	cfg.ch = cellHeight;
	cfg.walkableSlopeAngle = agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(agentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(edgeMaxLen / cellSize);
	cfg.maxSimplificationError = edgeMaxError;
	cfg.minRegionArea = (int)rcSqr(regionMinSize);      // Note: area = size*size
	cfg.mergeRegionArea = (int)rcSqr(regionMergeSize);  // Note: area = size*size
	cfg.maxVertsPerPoly = (int)vertsPerPoly;
	cfg.tileSize = tileSize;
	cfg.borderSize = cfg.walkableRadius + 3;  // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize * 2;
	cfg.height = cfg.tileSize + cfg.borderSize * 2;
	cfg.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
	cfg.detailSampleMaxError = cellHeight * detailSampleMaxError;
	rcVcopy(cfg.bmin, minBounds);
	rcVcopy(cfg.bmax, maxBounds);

	// Tile cache params.
	dtTileCacheParams tcparams = {};
	rcVcopy(tcparams.orig, minBounds);
	tcparams.cs = cellSize;
	tcparams.ch = cellHeight;
	tcparams.width = tileSize;
	tcparams.height = tileSize;
	tcparams.walkableHeight = agentHeight;
	tcparams.walkableRadius = agentRadius;
	tcparams.walkableClimb = agentMaxClimb;
	tcparams.maxSimplificationError = edgeMaxError;
	tcparams.maxTiles = tw * th * EXPECTED_LAYERS_PER_TILE;
	tcparams.maxObstacles = 128;

	dtFreeTileCache(tileCache);

	tileCache = dtAllocTileCache();
	if (!tileCache)
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
		return false;
	}
	status = tileCache->init(&tcparams, tAllocator, tCompressor, tMeshProcess);
	if (dtStatusFailed(status))
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
		return false;
	}

	dtFreeNavMesh(navMesh);

	navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params = {};
	rcVcopy(params.orig, minBounds);
	params.tileWidth = static_cast<float>(tileSize) * cellSize;
	params.tileHeight = static_cast<float>(tileSize) * cellSize;
	params.maxTiles = maxTiles;
	params.maxPolys = maxPolysPerTile;

	status = navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}

	status = navQuery->init(navMesh, 2048);
	if (dtStatusFailed(status))
	{
		buildContext->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}

	// Preprocess tiles.

	buildContext->resetTimers();

	cacheLayerCount = 0;
	cacheCompressedSize = 0;
	cacheRawSize = 0;

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			TileCacheData tiles[MAX_LAYERS] = {};
			int ntiles = rasterizeTileLayers(x, y, cfg, tiles, MAX_LAYERS);

			for (int i = 0; i < ntiles; ++i)
			{
				TileCacheData* tile = &tiles[i];
				status = tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
				if (dtStatusFailed(status))
				{
					dtFree(tile->data);
					tile->data = 0;
					continue;
				}

				cacheLayerCount++;
				cacheCompressedSize += tile->dataSize;
				cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
			}
		}
	}

	// Build initial meshes
	buildContext->startTimer(RC_TIMER_TOTAL);
	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			tileCache->buildNavMeshTilesAt(x, y, navMesh);
		}
	}
	buildContext->stopTimer(RC_TIMER_TOTAL);

	cacheBuildTimeMs = static_cast<float>(buildContext->getAccumulatedTime(RC_TIMER_TOTAL)) / 1000.0f;
	cacheBuildMemUsage = static_cast<unsigned int>(tAllocator->high);

	const dtNavMesh* nav = navMesh;
	int navmeshMemUsage = 0;
	for (int i = 0; i < nav->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = nav->getTile(i);
		if (tile->header)
		{
			navmeshMemUsage += tile->dataSize;
		}
	}
	printf("navmeshMemUsage = %.1f kB", static_cast<float>(navmeshMemUsage) / 1024.0f);

	if (tool)
	{
		tool->init(this);
	}
	initToolStates(this);

	return true;
}

void Sample_TempObstacles::update(const float dt)
{
	Sample::update(dt);

	if (!navMesh)
	{
		return;
	}
	if (!tileCache)
	{
		return;
	}

	tileCache->update(dt, navMesh);
}

void Sample_TempObstacles::getTilePos(const float* pos, int& tileX, int& tileY)
{
	if (!inputGeometry)
	{
		return;
	}

	const float* minBounds = inputGeometry->getNavMeshBoundsMin();

	const float worldspaceTileSize = static_cast<float>(tileSize) * cellSize;
	tileX = static_cast<int>((pos[0] - minBounds[0]) / worldspaceTileSize);
	tileY = static_cast<int>((pos[2] - minBounds[2]) / worldspaceTileSize);
}

struct TileCacheSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams meshParams;
	dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader
{
	dtCompressedTileRef tileRef;
	int dataSize;
};

void Sample_TempObstacles::saveAll(const char* path) const
{
	if (!tileCache)
	{
		return;
	}

	FILE* fp = fopen(path, "wb");
	if (!fp)
	{
		return;
	}

	// Store header.
	TileCacheSetHeader header;
	header.magic = TILECACHESET_MAGIC;
	header.version = TILECACHESET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize)
		{
			continue;
		}
		header.numTiles++;
	}
	memcpy(&header.cacheParams, tileCache->getParams(), sizeof(dtTileCacheParams));
	memcpy(&header.meshParams, navMesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(TileCacheSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < tileCache->getTileCount(); ++i)
	{
		const dtCompressedTile* tile = tileCache->getTile(i);
		if (!tile || !tile->header || !tile->dataSize)
		{
			continue;
		}

		TileCacheTileHeader tileHeader;
		tileHeader.tileRef = tileCache->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

void Sample_TempObstacles::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp)
	{
		return;
	}

	// Read header.
	TileCacheSetHeader header;
	size_t headerReadReturnCode = fread(&header, sizeof(TileCacheSetHeader), 1, fp);
	if (headerReadReturnCode != 1)
	{
		// Error or early EOF
		fclose(fp);
		return;
	}
	if (header.magic != TILECACHESET_MAGIC)
	{
		fclose(fp);
		return;
	}
	if (header.version != TILECACHESET_VERSION)
	{
		fclose(fp);
		return;
	}

	navMesh = dtAllocNavMesh();
	if (!navMesh)
	{
		fclose(fp);
		return;
	}
	dtStatus status = navMesh->init(&header.meshParams);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return;
	}

	tileCache = dtAllocTileCache();
	if (!tileCache)
	{
		fclose(fp);
		return;
	}
	status = tileCache->init(&header.cacheParams, tAllocator, tCompressor, tMeshProcess);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		TileCacheTileHeader tileHeader;
		size_t tileHeaderReadReturnCode = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (tileHeaderReadReturnCode != 1)
		{
			// Error or early EOF
			fclose(fp);
			return;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
		{
			break;
		}

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);

		if (!data)
		{
			break;
		}

		memset(data, 0, tileHeader.dataSize);
		size_t tileDataReadReturnCode = fread(data, tileHeader.dataSize, 1, fp);
		if (tileDataReadReturnCode != 1)
		{
			// Error or early EOF
			dtFree(data);
			fclose(fp);
			return;
		}

		dtCompressedTileRef tile = 0;
		dtStatus addTileStatus = tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);
		if (dtStatusFailed(addTileStatus))
		{
			dtFree(data);
		}

		if (tile)
		{
			tileCache->buildNavMeshTile(tile, navMesh);
		}
	}

	fclose(fp);
}

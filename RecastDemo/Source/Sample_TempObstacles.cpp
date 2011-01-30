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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_TempObstacles.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"
#include "fastlz.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


inline unsigned int nextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int ilog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	rcVsub(d, sq, sp);
	tmin = 0;  // set to -FLT_MAX to get first hit on line
	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
	
	// For all three slabs
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			// Ray is parallel to slab. No hit if origin not within slab
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			// Compute intersection t value of ray with near and far plane of slab
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			// Make t1 be intersection with near plane, t2 with far plane
			if (t1 > t2) rcSwap(t1, t2);
			// Compute the intersection of slab intersections intervals
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			// Exit with no collision as soon as slab intersection becomes empty
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}


struct CompressedTile
{
	inline CompressedTile() : tx(-1), ty(-1), compressedData(0), compressedSize(0), dataSize(0) {}
	inline ~CompressedTile() { purge(); }
	
	void purge()
	{
		tx = -1;
		ty = -1;
		delete [] compressedData;
		compressedData = 0;
		compressedSize = 0;
		dataSize = 0;
	}
	
	int tx, ty;
	float bmin[3], bmax[3];
	int triCount;
	unsigned char* compressedData;
	int compressedSize;
	int dataSize;
};

struct RasterizationContext
{
	inline RasterizationContext() : tile(0), solid(0), triareas(0), lhf(0) {}
	inline ~RasterizationContext() { delete tile; rcFreeHeightField(solid); delete [] triareas; rcFree(lhf); }
	
	CompressedTile* tile;
	rcHeightfield* solid;
	unsigned char* triareas;
	rcLeanHeightfield* lhf;
};

static CompressedTile* rasterizeTile(BuildContext* ctx, InputGeom* geom,
									 const int tx, const int ty,
									 const rcConfig& cfg)
{
	if (!geom || !geom->getMesh() || !geom->getChunkyMesh())
	{
		ctx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
		return 0;
	}
	
	RasterizationContext rc;
	
	rc.tile = new CompressedTile;
	if (!rc.tile)
	{
		return 0;
	}

	rc.tile->tx = tx;
	rc.tile->ty = ty;

	const float* verts = geom->getMesh()->getVerts();
	const int nverts = geom->getMesh()->getVertCount();
	const rcChunkyTriMesh* chunkyMesh = geom->getChunkyMesh();
	
	// Tile bounds.
	const float tcs = cfg.tileSize * cfg.cs;
	
	rc.tile->bmin[0] = cfg.bmin[0] + tx*tcs;
	rc.tile->bmin[1] = cfg.bmin[1];
	rc.tile->bmin[2] = cfg.bmin[2] + ty*tcs;
	rc.tile->bmax[0] = cfg.bmin[0] + (tx+1)*tcs;
	rc.tile->bmax[1] = cfg.bmax[1];
	rc.tile->bmax[2] = cfg.bmin[2] + (ty+1)*tcs;
	
	rcConfig tcfg;
	memcpy(&tcfg, &cfg, sizeof(tcfg));
	
	rcVcopy(tcfg.bmin, rc.tile->bmin);
	rcVcopy(tcfg.bmax, rc.tile->bmax);
	tcfg.bmin[0] -= tcfg.borderSize*tcfg.cs;
	tcfg.bmin[2] -= tcfg.borderSize*tcfg.cs;
	tcfg.bmax[0] += tcfg.borderSize*tcfg.cs;
	tcfg.bmax[2] += tcfg.borderSize*tcfg.cs;
	
	// Allocate voxel heightfield where we rasterize our input data to.
	rc.solid = rcAllocHeightfield();
	if (!rc.solid)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return 0;
	}
	if (!rcCreateHeightfield(ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return 0;
	}
	
	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!rc.triareas)
	{
		ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
		return 0;
	}
	
	float tbmin[2], tbmax[2];
	tbmin[0] = tcfg.bmin[0];
	tbmin[1] = tcfg.bmin[2];
	tbmax[0] = tcfg.bmax[0];
	tbmax[1] = tcfg.bmax[2];
	int cid[512];// TODO: Make grow when returning too many items.
	const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
	if (!ncid)
	{
		return 0; // empty
	}
	
	rc.tile->triCount = 0;
	
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* tris = &chunkyMesh->tris[node.i*3];
		const int ntris = node.n;
		
		rc.tile->triCount += ntris;
		
		memset(rc.triareas, 0, ntris*sizeof(unsigned char));
		rcMarkWalkableTriangles(ctx, tcfg.walkableSlopeAngle,
								verts, nverts, tris, ntris, rc.triareas);
		
		rcRasterizeTriangles(ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb);
	}
	
	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	rcFilterLowHangingWalkableObstacles(ctx, tcfg.walkableClimb, *rc.solid);
	rcFilterLedgeSpans(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
	rcFilterWalkableLowHeightSpans(ctx, tcfg.walkableHeight, *rc.solid);
	
	
	if (rcGetHeightFieldSpanCount(ctx, *rc.solid) > 0)
	{
		rc.lhf = rcBuildLeanHeightfield(ctx, *rc.solid, tcfg.walkableHeight);
		
		if (rc.lhf)
		{
			// Compress
			const int outSize = (int)(rc.lhf->size*1.05f);
			rc.tile->dataSize = rc.lhf->size;
			rc.tile->compressedData = new unsigned char[outSize];
			rc.tile->compressedSize = (int)fastlz_compress((const void *const)rc.lhf, rc.lhf->size, rc.tile->compressedData);
		}

		// Everything in rc gets deleted when it goes out of scope, save and return tile.
		CompressedTile* tile = rc.tile;
		rc.tile = 0;
		
		return tile;
	}
	
	return 0;
}



// Simple cylindrical obstacle.
static const int MAX_TOUCHED_TILES = 8;
struct TouchedTile
{
	short tx,ty;
};

struct TempObstacle
{
	float pos[3];
	float r, h;
	TouchedTile touched[MAX_TOUCHED_TILES];
	int ntouched;
};

class ObstacleSet
{
	static const int MAX_OBSTACLES = 128;
	TempObstacle m_obst[MAX_OBSTACLES];
	int m_nobst;

public:
	ObstacleSet() :
		m_nobst(0)
	{
	}
	
	~ObstacleSet()
	{
	}

	TempObstacle* addObstacle(const float* pos, const float r, const float h)
	{
		if (m_nobst >= MAX_OBSTACLES)
			return 0;
		TempObstacle* ob = &m_obst[m_nobst++];
		memset(ob,0,sizeof(TempObstacle));
		rcVcopy(ob->pos, pos);
		ob->r = r;
		ob->h = h;
		return ob;
	}
	
	void removeObstacle(const int idx)
	{
		if (idx < 0 || idx >= m_nobst)
			return;
		if (idx != m_nobst-1)
			memcpy(&m_obst[idx], &m_obst[m_nobst-1], sizeof(TempObstacle));
		m_nobst--;
	}
	
	void removeAllObstacles()
	{
		m_nobst = 0;
	}
	
	int hitTestObstacle(const float* sp, const float* sq)
	{
		float tmin = FLT_MAX, imin = -1;
		for (int i = 0; i < m_nobst; ++i)
		{
			const TempObstacle* ob = &m_obst[i];
			float bmin[3], bmax[3], t0,t1;
			bmin[0] = ob->pos[0] - ob->r;
			bmin[1] = ob->pos[1];
			bmin[2] = ob->pos[2] - ob->r;
			bmax[0] = ob->pos[0] + ob->r;
			bmax[1] = ob->pos[1] + ob->h;
			bmax[2] = ob->pos[2] + ob->r;
			
			if (isectSegAABB(sp,sq, bmin,bmax, t0,t1))
			{
				if (t0 < tmin)
				{
					tmin = t0;
					imin = i;
				}
			}
		}
		return imin;
	}
	
	inline int getObstacleCount() const { return m_nobst; }
	
	inline const TempObstacle* getObstacle(const int idx)
	{
		return &m_obst[idx];
	}
	
	void draw(duDebugDraw* dd)
	{
		// Draw obstacles
		for (int i = 0; i < m_nobst; ++i)
		{
			const TempObstacle* ob = &m_obst[i];
			duDebugDrawCylinder(dd, ob->pos[0]-ob->r, ob->pos[1], ob->pos[2]-ob->r,
								ob->pos[0]+ob->r, ob->pos[1]+ob->h, ob->pos[2]+ob->r,
								duRGBA(192,0,0,255));
			duDebugDrawCylinderWire(dd, ob->pos[0]-ob->r, ob->pos[1], ob->pos[2]-ob->r,
									ob->pos[0]+ob->r, ob->pos[1]+ob->h, ob->pos[2]+ob->r,
									duRGBA(128,0,0,255), 2);
		}
	}
};

class TileCache
{
	CompressedTile** m_tiles;
	int m_ntiles;
	int m_maxTiles;
	
	CompressedTile* getTile(const int x, const int y)
	{
		for (int i = 0; i < m_ntiles; ++i)
		{
			CompressedTile* tile = m_tiles[i]; 
			if (tile->tx == x && tile->ty == y)
				return tile;
		}
		return 0;
	}
	
	unsigned char* m_buffer;
	int m_bufferSize;
	
	rcConfig m_cfg;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	
	bool m_buildDetail;
	
	struct BuildContext
	{
		BuildContext() : chf(0), cset(0), pmesh(0), dmesh(0) {};
		~BuildContext() { }
		
		inline void purge()
		{
			rcFreeCompactHeightfield(chf);
			rcFreeContourSet(cset);
			rcFreePolyMesh(pmesh);
			rcFreePolyMeshDetail(dmesh);
			chf = 0;
			cset = 0;
			pmesh = 0;
			dmesh = 0;
		}
		
		rcCompactHeightfield* chf;
		rcContourSet* cset;
		rcPolyMesh* pmesh;
		rcPolyMeshDetail* dmesh;
	};
	
	struct Request
	{
		int tx,ty;
	};
	static const int MAX_REQUESTS = 64;
	Request m_reqs[MAX_REQUESTS];
	int m_nreqs;

	int m_buildState;
	BuildContext m_bc;
	
public:
	TileCache() :
		m_tiles(0),
		m_ntiles(0),
		m_maxTiles(0),
		m_buffer(0),
		m_bufferSize(0),
		m_buildDetail(false),
		m_nreqs(0),
		m_buildState(0)
	{
	}
	
	~TileCache()
	{
		purge();
	}
	
	void purge()
	{
		for (int i = 0; i < m_ntiles; ++i)
		{
			delete m_tiles[i];
			m_tiles[i] = 0;
		}
		m_ntiles = 0;
		delete [] m_buffer;
		m_buffer = 0;
		m_bufferSize = 0;
		delete [] m_tiles;
		m_tiles = 0;
		m_maxTiles = 0;
		m_nreqs = 0;
	}
	
	bool init(const int maxTiles, const rcConfig& cfg,
			  const float agentHeight, const float agentRadius, const float agentMaxClimb)
	{
		purge();
		
		m_tiles = new CompressedTile*[maxTiles];
		if (!m_tiles)
			return false;
		m_maxTiles = maxTiles;
		m_ntiles = 0;
		
		memcpy(&m_cfg, &cfg, sizeof(m_cfg));
		
		m_agentHeight = agentHeight;
		m_agentRadius = agentRadius;
		m_agentMaxClimb = agentMaxClimb;
		
		m_nreqs = 0;
		
		return true;
	}
	
	const float getBorderSize() const
	{
		return m_cfg.borderSize*m_cfg.cs;
	}
	
	bool initCompressionBuffer(int size)
	{
		delete [] m_buffer;
		m_bufferSize = (size + 512) & ~511;
		m_buffer = new unsigned char[m_bufferSize];
		if (!m_buffer)
			return false;
		return true;
	}
	
	bool addTile(CompressedTile* tile)
	{
		if (m_ntiles >= m_maxTiles)
			return false;
		m_tiles[m_ntiles++] = tile;
		return true;
	}
	
	bool addUpdateRequest(const int tx, const int ty)
	{
		for (int i = 0; i < m_nreqs; ++i)
		{
			if (m_reqs[i].tx == tx && m_reqs[i].ty == ty)
				return true;
		}
		if (m_nreqs >= MAX_REQUESTS)
			return false;
		Request* req = &m_reqs[m_nreqs++];
		req->tx = tx;
		req->ty = ty;
		return true;
	}
	
	void update(const float dt, rcContext* ctx, dtNavMesh* navmesh, ObstacleSet* obs)
	{
		static const int MAX_TIME_USEC = 1000;
		
		if (!m_nreqs)
			return;
		if (!m_buffer)
			return;

		// Process requests until max time has passed.
		
		TimeVal startTime = getPerfTime();
		
		while (m_nreqs > 0)
		{
			if (m_buildState == -1)
			{
				// Pop new request
				m_nreqs--;
				for (int i = 0; i < m_nreqs; ++i)
					m_reqs[i] = m_reqs[i+1];
				m_buildState = 0;
				if (!m_nreqs)
					return;
			}
			
			const int tx = m_reqs[0].tx;
			const int ty = m_reqs[0].ty;
			const CompressedTile* tile = getTile(tx,ty);
			if (!tile)
			{
				ctx->log(RC_LOG_ERROR, "buildNavigation: Could not find tile (%d,%d).", tx, ty);
				m_buildState = -1;
				return;
			}

			if (m_buildState == 0)
			{
				// Decompress tile and build compact heighfield.
				int size = fastlz_decompress(tile->compressedData, tile->compressedSize, m_buffer, m_bufferSize);
				if (size <= 0)
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Could not decompress tile.");
					m_buildState = -1;
					return;
				}
				rcLeanHeightfield* lhf = (rcLeanHeightfield*)m_buffer;

				m_bc.purge();
			
				// Compact the heightfield so that it is faster to handle from now on.
				// This will result more cache coherent data as well as the neighbours
				// between walkable cells will be calculated.
				m_bc.chf = rcAllocCompactHeightfield();
				if (!m_bc.chf)
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
					m_buildState = -1;
					return;
				}
				
				if (!rcBuildCompactHeightfield(ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *lhf, *m_bc.chf))
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
					m_buildState = -1;
					return;
				}

				// Advance state and bail out if used too much time.
				m_buildState++;
				if (getPerfDeltaTimeUsec(startTime, getPerfTime()) > MAX_TIME_USEC)
					return;
			}
			
			if (m_buildState == 1)
			{
				// Mark obstacles.
				for (int i = 0; i < obs->getObstacleCount(); ++i)
				{
					const TempObstacle* ob = obs->getObstacle(i);
					rcMarkCylinderArea(ctx, ob->pos, ob->r, ob->h, RC_NULL_AREA, *m_bc.chf);
				}
				
				// Erode the walkable area by agent radius.
				if (!rcErodeWalkableArea(ctx, m_cfg.walkableRadius, *m_bc.chf))
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
					m_buildState = -1;
					return;
				}
				
				// (Optional) Mark areas.
				/*		const ConvexVolume* vols = m_geom->getConvexVolumes();
				 for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
				 {
				 rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax,
				 (unsigned char)vols[i].area, *bc.chf);
				 }*/
				
				// Advance state and bail out if used too much time.
				m_buildState++;
				if (getPerfDeltaTimeUsec(startTime, getPerfTime()) > MAX_TIME_USEC)
					return;
			}
			
			if (m_buildState == 2)
			{
				// Partition the walkable surface into simple regions without holes.
				if (!rcBuildRegionsMonotone(ctx, *m_bc.chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
					m_buildState = -1;
					return;
				}
				
				// Advance state and bail out if used too much time.
				m_buildState++;
				if (getPerfDeltaTimeUsec(startTime, getPerfTime()) > MAX_TIME_USEC)
					return;
			}
			
			if (m_buildState == 3)
			{
				// Create contours.
				m_bc.cset = rcAllocContourSet();
				if (!m_bc.cset)
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
					m_buildState = -1;
					return;
				}
				if (!rcBuildContours(ctx, *m_bc.chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_bc.cset))
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
					m_buildState = -1;
					return;
				}
				
				if (m_bc.cset->nconts == 0)
				{
					// Done, no mesh.
					m_buildState = -1;
					return;
				}
				
				// Advance state and bail out if used too much time.
				m_buildState++;
				if (getPerfDeltaTimeUsec(startTime, getPerfTime()) > MAX_TIME_USEC)
					return;
			}
			
			if (m_buildState == 4)
			{
				// Build polygon navmesh from the contours.
				m_bc.pmesh = rcAllocPolyMesh();
				if (!m_bc.pmesh)
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
					m_buildState = -1;
					return;
				}
				if (!rcBuildPolyMesh(ctx, *m_bc.cset, m_cfg.maxVertsPerPoly, *m_bc.pmesh))
				{
					ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
					m_buildState = -1;
					return;
				}
				
				// Advance state and bail out if used too much time.
				m_buildState++;
				if (getPerfDeltaTimeUsec(startTime, getPerfTime()) > MAX_TIME_USEC)
					return;
			}
			
			// Build detail mesh.
			if (m_buildState == 5)
			{
				if (m_buildDetail)
				{
					m_bc.dmesh = rcAllocPolyMeshDetail();
					if (!m_bc.dmesh)
					{
						ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'.");
						m_buildState = -1;
						return;
					}
					
					if (!rcBuildPolyMeshDetail(ctx, *m_bc.pmesh, *m_bc.chf,
											   m_cfg.detailSampleDist, m_cfg.detailSampleMaxError,
											   *m_bc.dmesh))
					{
						ctx->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail.");
						m_buildState = -1;
						return;
					}
				}
				
				// Advance state and bail out if used too much time.
				m_buildState++;
				if (getPerfDeltaTimeUsec(startTime, getPerfTime()) > MAX_TIME_USEC)
					return;
			}
			
			if (m_buildState == 6)
			{
				// Remove padding from the polymesh data. TODO: Remove this odditity.
				for (int i = 0; i < m_bc.pmesh->nverts; ++i)
				{
					unsigned short* v = &m_bc.pmesh->verts[i*3];
					v[0] -= (unsigned short)m_cfg.borderSize;
					v[2] -= (unsigned short)m_cfg.borderSize;
				}
				
				if (m_bc.pmesh->nverts >= 0xffff)
				{
					// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
					ctx->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", m_bc.pmesh->nverts, 0xffff);
				}
				
				// Update poly flags from areas.
				for (int i = 0; i < m_bc.pmesh->npolys; ++i)
				{
					if (m_bc.pmesh->areas[i] == RC_WALKABLE_AREA)
						m_bc.pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
					
					if (m_bc.pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
						m_bc.pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
						m_bc.pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
					{
						m_bc.pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
					}
					else if (m_bc.pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
					{
						m_bc.pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
					}
					else if (m_bc.pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
					{
						m_bc.pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
					}
				}
				
				dtNavMeshCreateParams params;
				memset(&params, 0, sizeof(params));
				params.verts = m_bc.pmesh->verts;
				params.vertCount = m_bc.pmesh->nverts;
				params.polys = m_bc.pmesh->polys;
				params.polyAreas = m_bc.pmesh->areas;
				params.polyFlags = m_bc.pmesh->flags;
				params.polyCount = m_bc.pmesh->npolys;
				params.nvp = m_bc.pmesh->nvp;
				
				if (m_bc.dmesh)
				{
					params.detailMeshes = m_bc.dmesh->meshes;
					params.detailVerts = m_bc.dmesh->verts;
					params.detailVertsCount = m_bc.dmesh->nverts;
					params.detailTris = m_bc.dmesh->tris;
					params.detailTriCount = m_bc.dmesh->ntris;
				}
				
				/*		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
				 params.offMeshConRad = m_geom->getOffMeshConnectionRads();
				 params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
				 params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
				 params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
				 params.offMeshConUserID = m_geom->getOffMeshConnectionId();
				 params.offMeshConCount = m_geom->getOffMeshConnectionCount();*/
				
				params.walkableHeight = m_agentHeight;
				params.walkableRadius = m_agentRadius;
				params.walkableClimb = m_agentMaxClimb;
				params.tileX = tx;
				params.tileY = ty;
				
				const float tcs = m_cfg.tileSize * m_cfg.cs;
				params.bmin[0] = m_cfg.bmin[0] + tx*tcs;
				params.bmin[1] = m_cfg.bmin[1];
				params.bmin[2] = m_cfg.bmin[2] + ty*tcs;
				params.bmax[0] = m_cfg.bmin[0] + (tx+1)*tcs;
				params.bmax[1] = m_cfg.bmax[1];
				params.bmax[2] = m_cfg.bmin[2] + (ty+1)*tcs;
				
				params.cs = m_cfg.cs;
				params.ch = m_cfg.ch;
				params.tileSize = m_cfg.tileSize;

				unsigned char* navData = 0;
				int navDataSize = 0;

				if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
				{
					ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
					m_buildState = -1;
					return;
				}
				
				navmesh->removeTile(navmesh->getTileRefAt(tx,ty),0,0);
				// Let the navmesh own the data.
				dtStatus status = navmesh->addTile(navData,navDataSize,DT_TILE_FREE_DATA,0,0);
				if (dtStatusFailed(status))
					dtFree(navData);
				
				// Done!
				m_buildState = -1;
				// Bail out if used too much time.
				if (getPerfDeltaTimeUsec(startTime, getPerfTime()) > MAX_TIME_USEC)
					return;
			}
		}		
	}
	
	unsigned char* buildNavMeshTile(rcContext* ctx, ObstacleSet* obs, const int tx, const int ty, int& dataSize)
	{
		const CompressedTile* tile = getTile(tx,ty);
		if (!tile)
			return 0;
		if (!m_buffer)
			return 0;
		
		int size = fastlz_decompress(tile->compressedData, tile->compressedSize, m_buffer, m_bufferSize);
		
		if (size <= 0)
			return 0;

		BuildContext bc;
		
		rcLeanHeightfield* cchf = (rcLeanHeightfield*)m_buffer;
			
		// Compact the heightfield so that it is faster to handle from now on.
		// This will result more cache coherent data as well as the neighbours
		// between walkable cells will be calculated.
		bc.chf = rcAllocCompactHeightfield();
		if (!bc.chf)
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
			return 0;
		}

		if (!rcBuildCompactHeightfield(ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *cchf, *bc.chf))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
			return 0;
		}
				
		// Mark obstacles.
		for (int i = 0; i < obs->getObstacleCount(); ++i)
		{
			const TempObstacle* ob = obs->getObstacle(i);
			rcMarkCylinderArea(ctx, ob->pos, ob->r, ob->h, RC_NULL_AREA, *bc.chf);
		}
		
		// Erode the walkable area by agent radius.
		if (!rcErodeWalkableArea(ctx, m_cfg.walkableRadius, *bc.chf))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
			return 0;
		}
		
		// (Optional) Mark areas.
/*		const ConvexVolume* vols = m_geom->getConvexVolumes();
		for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
		{
			rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax,
								 (unsigned char)vols[i].area, *bc.chf);
		}*/
		
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegionsMonotone(ctx, *bc.chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
			return 0;
		}
		
		// Create contours.
		bc.cset = rcAllocContourSet();
		if (!bc.cset)
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
			return 0;
		}
		if (!rcBuildContours(ctx, *bc.chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *bc.cset))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
			return 0;
		}
		
		if (bc.cset->nconts == 0)
		{
			return 0;
		}
		
		// Build polygon navmesh from the contours.
		bc.pmesh = rcAllocPolyMesh();
		if (!bc.pmesh)
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
			return 0;
		}
		if (!rcBuildPolyMesh(ctx, *bc.cset, m_cfg.maxVertsPerPoly, *bc.pmesh))
		{
			ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
			return 0;
		}
		
		// Build detail mesh.
		if (m_buildDetail)
		{
			bc.dmesh = rcAllocPolyMeshDetail();
			if (!bc.dmesh)
			{
				ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'.");
				return 0;
			}
			
			if (!rcBuildPolyMeshDetail(ctx, *bc.pmesh, *bc.chf,
									   m_cfg.detailSampleDist, m_cfg.detailSampleMaxError,
									   *bc.dmesh))
			{
				ctx->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail.");
				return 0;
			}
		}
		
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Remove padding from the polymesh data. TODO: Remove this odditity.
		for (int i = 0; i < bc.pmesh->nverts; ++i)
		{
			unsigned short* v = &bc.pmesh->verts[i*3];
			v[0] -= (unsigned short)m_cfg.borderSize;
			v[2] -= (unsigned short)m_cfg.borderSize;
		}
			
		if (bc.pmesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			ctx->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", bc.pmesh->nverts, 0xffff);
		}
			
		// Update poly flags from areas.
		for (int i = 0; i < bc.pmesh->npolys; ++i)
		{
			if (bc.pmesh->areas[i] == RC_WALKABLE_AREA)
				bc.pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
			
			if (bc.pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				bc.pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				bc.pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				bc.pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (bc.pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				bc.pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (bc.pmesh->areas[i] == SAMPLE_POLYAREA_DOOR)
			{
				bc.pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
			}
		}
			
		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = bc.pmesh->verts;
		params.vertCount = bc.pmesh->nverts;
		params.polys = bc.pmesh->polys;
		params.polyAreas = bc.pmesh->areas;
		params.polyFlags = bc.pmesh->flags;
		params.polyCount = bc.pmesh->npolys;
		params.nvp = bc.pmesh->nvp;
		
		if (bc.dmesh)
		{
			params.detailMeshes = bc.dmesh->meshes;
			params.detailVerts = bc.dmesh->verts;
			params.detailVertsCount = bc.dmesh->nverts;
			params.detailTris = bc.dmesh->tris;
			params.detailTriCount = bc.dmesh->ntris;
		}
		
/*		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();*/
		
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		params.tileX = tx;
		params.tileY = ty;
		
		const float tcs = m_cfg.tileSize * m_cfg.cs;
		params.bmin[0] = m_cfg.bmin[0] + tx*tcs;
		params.bmin[1] = m_cfg.bmin[1];
		params.bmin[2] = m_cfg.bmin[2] + ty*tcs;
		params.bmax[0] = m_cfg.bmin[0] + (tx+1)*tcs;
		params.bmax[1] = m_cfg.bmax[1];
		params.bmax[2] = m_cfg.bmin[2] + (ty+1)*tcs;
		
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.tileSize = m_cfg.tileSize;
		
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return 0;
		}
		
		dataSize = navDataSize;
		
		return navData;
	}
	
	void drawTiles(duDebugDraw* dd)
	{
		for (int i = 0; i < m_ntiles; ++i)
		{
			const CompressedTile* tile = m_tiles[i];
			
			duDebugDrawBoxWire(dd, tile->bmin[0],tile->bmin[1],tile->bmin[2],
							   tile->bmax[0],tile->bmin[1],tile->bmax[2], duRGBA(255,255,255,32), 2.0f);
		}
	}
	
	void drawDetail(duDebugDraw* dd, const int tx, const int ty)
	{
		const CompressedTile* tile = getTile(tx,ty);
		if (!tile)
			return;

		int size = fastlz_decompress(tile->compressedData, tile->compressedSize, m_buffer, m_bufferSize);
		
		if (size > 0)
		{
			rcLeanHeightfield* chf = (rcLeanHeightfield*)m_buffer;
			
			duDebugDrawBoxWire(dd, chf->bmin[0],chf->bmin[1],chf->bmin[2],
							   chf->bmax[0],chf->bmax[1],chf->bmax[2], duRGBA(0,192,255,128), 1.0f);

			duDebugDrawLeanHeightfieldSolid(dd, *chf);
		}
	}
	
	void drawDetailOverlay(const int tx, const int ty, double* proj, double* model, int* view)
	{
		const CompressedTile* tile = getTile(tx,ty);
		if (!tile)
			return;
		
		char text[128];

		float pos[3];
		pos[0] = (tile->bmin[0]+tile->bmax[0])/2.0f;
		pos[1] = tile->bmin[1];
		pos[2] = (tile->bmin[2]+tile->bmax[2])/2.0f;
		
		GLdouble x, y, z;
		if (gluProject((GLdouble)pos[0], (GLdouble)pos[1], (GLdouble)pos[2],
					   model, proj, view, &x, &y, &z))
		{
			snprintf(text,128,"(%d,%d)", tile->tx,tile->ty);
			imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
			snprintf(text,128,"Lean Data:%.1fkB", tile->dataSize/1024.0f);
			imguiDrawText((int)x, (int)y-45, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,128));
			snprintf(text,128,"Compressed:%.1fkB", tile->compressedSize/1024.0f);
			imguiDrawText((int)x, (int)y-65, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,128));
		}		
	}
	
	int getRawSize() const
	{
		int size = 0;
		for (int i = 0; i < m_ntiles; ++i)
		{
			const CompressedTile* tile = m_tiles[i];
			size += tile->dataSize;
		}
		return size;
	}

	int getCompressedSize() const
	{
		int size = 0;
		for (int i = 0; i < m_ntiles; ++i)
		{
			const CompressedTile* tile = m_tiles[i];
			size += tile->compressedSize;
		}
		return size;
	}
	
};




class TempObstacleHilightTool : public SampleTool
{
	Sample_TempObstacles* m_sample;
	float m_hitPos[3];
	bool m_hitPosSet;
	float m_agentRadius;
	
public:

	TempObstacleHilightTool() :
		m_sample(0),
		m_hitPosSet(false),
		m_agentRadius(0)
	{
		m_hitPos[0] = m_hitPos[1] = m_hitPos[2] = 0;
	}

	virtual ~TempObstacleHilightTool()
	{
	}

	virtual int type() { return TOOL_TILE_HIGHLIGHT; }

	virtual void init(Sample* sample)
	{
		m_sample = (Sample_TempObstacles*)sample; 
	}
	
	virtual void reset() {}

	virtual void handleMenu()
	{
		imguiLabel("Highlight Tile Cache");
		imguiValue("Click LMB to highlight a tile.");
	}

	virtual void handleClick(const float* /*s*/, const float* p, bool shift)
	{
		m_hitPosSet = true;
		rcVcopy(m_hitPos,p);
	}

	virtual void handleToggle() {}

	virtual void handleStep() {}

	virtual void handleUpdate(const float /*dt*/) {}
	
	virtual void handleRender()
	{
		if (m_hitPosSet)
		{
			const float s = m_sample->getAgentRadius();
			glColor4ub(0,0,0,128);
			glLineWidth(2.0f);
			glBegin(GL_LINES);
			glVertex3f(m_hitPos[0]-s,m_hitPos[1]+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0]+s,m_hitPos[1]+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]-s+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]+s+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]-s);
			glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]+s);
			glEnd();
			glLineWidth(1.0f);
			
			if (m_sample)
			{
				int tx=0, ty=0;
				m_sample->getTilePos(m_hitPos, tx, ty);
				m_sample->renderCachedTile(tx,ty);
			}

		}
	}
	
	virtual void handleRenderOverlay(double* proj, double* model, int* view)
	{
		if (m_hitPosSet)
		{
			if (m_sample)
			{
				int tx=0, ty=0;
				m_sample->getTilePos(m_hitPos, tx, ty);
				m_sample->renderCachedTileOverlay(tx,ty,proj,model,view);
			}
		}		
	}
};


class TempObstacleCreateTool : public SampleTool
{
	Sample_TempObstacles* m_sample;
	
public:
	
	TempObstacleCreateTool()
	{
	}
	
	virtual ~TempObstacleCreateTool()
	{
	}
	
	virtual int type() { return TOOL_TEMP_OBSTACLE; }
	
	virtual void init(Sample* sample)
	{
		m_sample = (Sample_TempObstacles*)sample; 
	}
	
	virtual void reset() {}
	
	virtual void handleMenu()
	{
		imguiLabel("Create Temp Obstacles");
		
		if (imguiButton("Remove All"))
			m_sample->clearAllTempObstacles();
		
		imguiSeparator();

		imguiValue("Click LMB to create an obstacle.");
		imguiValue("Shift+LMB to remove an obstacle.");
	}
	
	virtual void handleClick(const float* s, const float* p, bool shift)
	{
		if (m_sample)
		{
			if (shift)
				m_sample->removeTempObstacle(s,p);
			else
				m_sample->addTempObstacle(p);
		}
	}
	
	virtual void handleToggle() {}
	virtual void handleStep() {}
	virtual void handleUpdate(const float /*dt*/) {}
	virtual void handleRender() {}
	virtual void handleRenderOverlay(double* proj, double* model, int* view) { }
};





Sample_TempObstacles::Sample_TempObstacles() :
	m_keepInterResults(false),
	m_cacheBuildTimeMs(0),
	m_drawPortals(false),
	m_drawMode(DRAWMODE_NAVMESH),
	m_maxTiles(0),
	m_maxPolysPerTile(0),
	m_tileSize(48),
	m_rebuildTileCount(0),
	m_rebuildTime(0)
{
	resetCommonSettings();
	
	m_tileCache = new TileCache;
	m_obs = new ObstacleSet;
	
	setTool(new TempObstacleCreateTool);
}

Sample_TempObstacles::~Sample_TempObstacles()
{
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
	delete m_tileCache;
	delete m_obs;
}

void Sample_TempObstacles::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;

	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_tileSize, 16.0f, 128.0f, 8.0f);
	
	if (m_geom)
	{
		const float* bmin = m_geom->getMeshBoundsMin();
		const float* bmax = m_geom->getMeshBoundsMax();
		char text[64];
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		const int ts = (int)m_tileSize;
		const int tw = (gw + ts-1) / ts;
		const int th = (gh + ts-1) / ts;
		snprintf(text, 64, "Tiles  %d x %d", tw, th);
		imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)ilog2(nextPow2(tw*th)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;
		m_maxTiles = 1 << tileBits;
		m_maxPolysPerTile = 1 << polyBits;
		snprintf(text, 64, "Max Tiles  %d", m_maxTiles);
		imguiValue(text);
		snprintf(text, 64, "Max Polys  %d", m_maxPolysPerTile);
		imguiValue(text);
	}
	else
	{
		m_maxTiles = 0;
		m_maxPolysPerTile = 0;
	}
	
	imguiSeparator();
	
	char msg[64];
	snprintf(msg, 64, "Tile Cache Build Time  %.1fms", m_cacheBuildTimeMs);
	imguiValue(msg);

	imguiSeparator();
	
	imguiSeparator();
	
}

void Sample_TempObstacles::handleTools()
{
	int type = !m_tool ? TOOL_NONE : m_tool->type();

	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Highlight Tile Cache", type == TOOL_TILE_HIGHLIGHT))
	{
		setTool(new TempObstacleHilightTool);
	}
	if (imguiCheck("Create Temp Obstacles", type == TOOL_TEMP_OBSTACLE))
	{
		setTool(new TempObstacleCreateTool);
	}
	if (imguiCheck("Create Off-Mesh Links", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (imguiCheck("Create Crowds", type == TOOL_CROWD))
	{
		setTool(new CrowdTool);
	}
	
	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
		m_tool->handleMenu();

	imguiUnindent();
}

void Sample_TempObstacles::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;
	
	if (m_geom)
	{
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_PORTALS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
	}
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;
	
	if (unavail == MAX_DRAWMODE)
		return;
	
	imguiLabel("Draw");
	if (imguiCheck("Input Mesh", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_drawMode = DRAWMODE_MESH;
	if (imguiCheck("Navmesh", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_drawMode = DRAWMODE_NAVMESH;
	if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Navmesh Nodes", m_drawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_drawMode = DRAWMODE_NAVMESH_NODES;
	if (imguiCheck("Navmesh Portals", m_drawMode == DRAWMODE_NAVMESH_PORTALS, valid[DRAWMODE_NAVMESH_PORTALS]))
		m_drawMode = DRAWMODE_NAVMESH_PORTALS;
	
	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("rebuild some tiles to see");
		imguiValue("more debug mode options.");
	}
}

void Sample_TempObstacles::handleRender()
{
	if (!m_geom || !m_geom->getMesh())
		return;
	
	DebugDrawGL dd;

	// Draw mesh
	if (m_drawMode == DRAWMODE_MESH)
	{
		// Draw mesh
		duDebugDrawTriMeshSlope(&dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
								m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(),
								m_agentMaxSlope);
		m_geom->drawOffMeshConnections(&dd);
	}
	else if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		duDebugDrawTriMesh(&dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
						   m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(), 0);
		m_geom->drawOffMeshConnections(&dd);
	}
	
	m_tileCache->drawTiles(&dd);
	m_obs->draw(&dd);
	
	glDepthMask(GL_FALSE);
	
	// Draw bounds
	const float* bmin = m_geom->getMeshBoundsMin();
	const float* bmax = m_geom->getMeshBoundsMax();
	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
	
	// Tiling grid.
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int tw = (gw + (int)m_tileSize-1) / (int)m_tileSize;
	const int th = (gh + (int)m_tileSize-1) / (int)m_tileSize;
	const float s = m_tileSize*m_cellSize;
	duDebugDrawGridXZ(&dd, bmin[0],bmin[1],bmin[2], tw,th, s, duRGBA(0,0,0,64), 1.0f);
		
	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
		 m_drawMode == DRAWMODE_NAVMESH_TRANS ||
		 m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_drawMode == DRAWMODE_NAVMESH_NODES ||
		 m_drawMode == DRAWMODE_NAVMESH_PORTALS ||
		 m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_PORTALS)
			duDebugDrawNavMeshPortals(&dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&dd, *m_navQuery);
	}
	
	
	glDepthMask(GL_TRUE);
		
	m_geom->drawConvexVolumes(&dd);
	
	if (m_tool)
		m_tool->handleRender();
	
	glDepthMask(GL_TRUE);
}

void Sample_TempObstacles::renderCachedTile(const int tx, const int ty)
{
	DebugDrawGL dd;
	m_tileCache->drawDetail(&dd,tx,ty);
}

void Sample_TempObstacles::renderCachedTileOverlay(const int tx, const int ty, double* proj, double* model, int* view)
{
	m_tileCache->drawDetailOverlay(tx, ty, proj, model, view);
}

void Sample_TempObstacles::handleRenderOverlay(double* proj, double* model, int* view)
{	
	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);

	// Stats
	imguiDrawRect(280,10,300,100,imguiRGBA(0,0,0,64));
	
	char text[64];
	float y = 110-30;
	
	snprintf(text,64,"Lean Data: %.1fkB", m_tileCache->getRawSize()/1024.0f);
	imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	y -= 20;
	
	snprintf(text,64,"Compressed: %.1fkB (%.1f%%)", m_tileCache->getCompressedSize()/1024.0f,
			 m_tileCache->getRawSize() > 0 ? 100.0f*(float)m_tileCache->getCompressedSize()/(float)m_tileCache->getRawSize() : 0);
	imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,255,255,255));
	y -= 20;

	if (m_rebuildTileCount > 0 && m_rebuildTime > 0.0f)
	{
		snprintf(text,64,"Changed obstacles, rebuild %d tiles: %.3f ms", m_rebuildTileCount, m_rebuildTime);
		imguiDrawText(300, y, IMGUI_ALIGN_LEFT, text, imguiRGBA(255,192,0,255));
		y -= 20;
	}
	
}

void Sample_TempObstacles::handleMeshChanged(class InputGeom* geom)
{
	Sample::handleMeshChanged(geom);

	m_tileCache->purge();
	m_obs->removeAllObstacles();
	
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
}

int Sample_TempObstacles::calcTouchedTiles(const float minx, const float minz, const float maxx, const float maxz,
										  TouchedTile* touched, const int maxTouched)
{
	const float* orig = m_geom->getMeshBoundsMin();
	const float ts = m_tileSize*m_cellSize;
	const int tminx = (int)((minx - orig[0]) / ts);
	const int tminy = (int)((minz - orig[2]) / ts);
	const int tmaxx = (int)((maxx - orig[0]) / ts);
	const int tmaxy = (int)((maxz - orig[2]) / ts);

	int n = 0;
	for (int y = tminy; y <= tmaxy; y++)
	{
		for (int x = tminx; x <= tmaxx; x++)
		{
			if (n < maxTouched)
			{
				touched[n].tx = x;
				touched[n].ty = y;
				n++;
			}
		}
	}
	return n;
}

void Sample_TempObstacles::addTempObstacle(const float* pos)
{
	if (!m_tileCache)
		return;

	const float orad = 1.0f;
	
	TempObstacle* ob = m_obs->addObstacle(pos, orad, 2.0f);
	
	
	// Update all touched tiles.
	const float borderSize = m_tileCache->getBorderSize();
	const float rad2 = orad + borderSize + 0.001f;
	const float minx = pos[0]-rad2;
	const float minz = pos[2]-rad2;
	const float maxx = pos[0]+rad2;
	const float maxz = pos[2]+rad2;

	ob->ntouched = calcTouchedTiles(minx,minz,maxx,maxz, ob->touched, MAX_TOUCHED_TILES);

	rebuildTiles(ob->touched, ob->ntouched);
}

void Sample_TempObstacles::rebuildTiles(const TouchedTile* touched, const int ntouched)
{
	m_ctx->resetTimers();
	m_ctx->startTimer(RC_TIMER_TOTAL);

	// rebuild tiles.
	for (int i = 0; i < ntouched; ++i)
	{
		const int x = touched[i].tx;
		const int y = touched[i].ty;
		
		m_tileCache->addUpdateRequest(x,y);
		
/*		int dataSize = 0;
		unsigned char* data = m_tileCache->buildNavMeshTile(m_ctx, m_obs, x,y, dataSize);
		if (data)
		{
			m_navMesh->removeTile(m_navMesh->getTileRefAt(x,y),0,0);
			// Let the navmesh own the data.
			dtStatus status = m_navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
			if (dtStatusFailed(status))
				dtFree(data);
		}*/
	}
	
	m_ctx->stopTimer(RC_TIMER_TOTAL);
	
	// Show performance stats.
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	
	m_rebuildTime = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
	m_rebuildTileCount = ntouched;
}

void Sample_TempObstacles::removeTempObstacle(const float* sp, const float* sq)
{
	if (!m_tileCache)
		return;
	int idx = m_obs->hitTestObstacle(sp, sq);
	if (idx != -1)
	{
		const TempObstacle* ob = m_obs->getObstacle(idx);
		
		int ntouched = ob->ntouched;
		TouchedTile touched[MAX_TOUCHED_TILES];
		if (ntouched > 0)
			memcpy(touched, ob->touched, sizeof(TouchedTile)*ntouched);
		
		m_obs->removeObstacle(idx);

		rebuildTiles(touched, ntouched);
	}
}

void Sample_TempObstacles::clearAllTempObstacles()
{
	if (!m_tileCache)
		return;
	m_obs->removeAllObstacles();
}

bool Sample_TempObstacles::handleBuild()
{
	if (!m_geom || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
		return false;
	}
	
	// Init cache
	const float* bmin = m_geom->getMeshBoundsMin();
	const float* bmax = m_geom->getMeshBoundsMax();
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
	const int ts = (int)m_tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;

	rcConfig cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.cs = m_cellSize;
	cfg.ch = m_cellHeight;
	cfg.walkableSlopeAngle = m_agentMaxSlope;
	cfg.walkableHeight = (int)ceilf(m_agentHeight / cfg.ch);
	cfg.walkableClimb = (int)floorf(m_agentMaxClimb / cfg.ch);
	cfg.walkableRadius = (int)ceilf(m_agentRadius / cfg.cs);
	cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	cfg.maxSimplificationError = m_edgeMaxError;
	cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	cfg.tileSize = (int)m_tileSize;
	cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
	cfg.width = cfg.tileSize + cfg.borderSize*2;
	cfg.height = cfg.tileSize + cfg.borderSize*2;
	cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	rcVcopy(cfg.bmin, bmin);
	rcVcopy(cfg.bmax, bmax);
	
	m_tileCache->init(tw*th, cfg, m_agentHeight, m_agentRadius, m_agentMaxClimb);
	
	
	dtFreeNavMesh(m_navMesh);
	
	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
		return false;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, m_geom->getMeshBoundsMin());
	params.tileWidth = m_tileSize*m_cellSize;
	params.tileHeight = m_tileSize*m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;
	
	dtStatus status;
	
	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh.");
		return false;
	}
	
	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query");
		return false;
	}
	

	// Preprocess tiles.
	
	m_ctx->resetTimers();
	m_ctx->startTimer(RC_TIMER_TOTAL);
	
	int maxBufferSize = 0;
	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			CompressedTile* tile = rasterizeTile(m_ctx, m_geom, x, y, cfg);
			if (!tile)
				continue;
			
			maxBufferSize = rcMax(maxBufferSize, tile->dataSize);
			
			if (!m_tileCache->addTile(tile))
			{
				delete tile;
				break;
			}
		}
	}
	
	m_tileCache->initCompressionBuffer(maxBufferSize);

	m_ctx->stopTimer(RC_TIMER_TOTAL);

	m_cacheBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;

	// Build initial meshes
	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			int dataSize = 0;
			unsigned char* data = m_tileCache->buildNavMeshTile(m_ctx, m_obs, x,y, dataSize);
			if (data)
			{
				m_navMesh->removeTile(m_navMesh->getTileRefAt(x,y),0,0);
				// Let the navmesh own the data.
				dtStatus status = m_navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
				if (dtStatusFailed(status))
					dtFree(data);
			}
		}
	}
	
	m_rebuildTileCount = 0;
	m_rebuildTime = 0;
	
	if (m_tool)
		m_tool->init(this);

	return true;
}

void Sample_TempObstacles::handleUpdate(const float dt)
{
	if (m_tool)
		m_tool->handleUpdate(dt);
	
	if (!m_navMesh)
		return;
	
	m_tileCache->update(dt, m_ctx, m_navMesh, m_obs);
}

void Sample_TempObstacles::getTilePos(const float* pos, int& tx, int& ty)
{
	if (!m_geom) return;
	
	const float* bmin = m_geom->getMeshBoundsMin();
	
	const float ts = m_tileSize*m_cellSize;
	tx = (int)((pos[0] - bmin[0]) / ts);
	ty = (int)((pos[2] - bmin[2]) / ts);
}

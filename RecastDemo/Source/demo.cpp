//
// Copyright (c) 2009 Mikko Mononen memon@inside.org
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

#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#ifdef WIN32
#	include <io.h>
#else
#	include <dirent.h>
#endif
#include "SDL.h"
#include "SDL_Opengl.h"
#include "GLFont.h"
#include "RecastTimer.h"
#include "MeshLoaderObj.h"
#include "ChunkyTriMesh.h"
#include "Recast.h"
#include "RecastLog.h"
#include "RecastDebugDraw.h"
#include "imgui.h"
#include "DetourStatNavMesh.h"
#include "DetourStatNavMeshBuilder.h"
#include "DetourDebugDraw.h"


#ifdef WIN32
#	define snprintf _snprintf
#endif

bool intersectSegmentTriangle(const float* sp, const float* sq,
							  const float* a, const float* b, const float* c,
							  float &t)
{
	float v, w;
	float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
	vsub(ab, b, a);
	vsub(ac, c, a);
	vsub(qp, sp, sq);
	
	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	vcross(norm, ab, ac);
	
	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = vdot(qp, norm);
	if (d <= 0.0f) return false;
	
	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	vsub(ap, sp, a);
	t = vdot(ap, norm);
	if (t < 0.0f) return false;
	if (t > d) return false; // For segment; exclude this code line for a ray test
	
	// Compute barycentric coordinate components and test if within bounds
	vcross(e, qp, ap);
	v = vdot(ac, e);
	if (v < 0.0f || v > d) return false;
	w = -vdot(ab, e);
	if (w < 0.0f || v + w > d) return false;
	
	// Segment/ray intersects triangle. Perform delayed division
	t /= d;
	
	return true;
}

static bool raycast(rcMeshLoaderObj& mesh, float* src, float* dst, float& tmin)
{
	float dir[3];
	vsub(dir, dst, src);
	
	int nt = mesh.getTriCount();
	const float* verts = mesh.getVerts();
	const float* normals = mesh.getNormals();
	const int* tris = mesh.getTris();
	tmin = 1.0f;
	bool hit = false;
	
	for (int i = 0; i < nt*3; i += 3)
	{
		const float* n = &normals[i];
		if (vdot(dir, n) > 0)
			continue;
		
		float t = 1;
		if (intersectSegmentTriangle(src, dst,
									 &verts[tris[i]*3],
									 &verts[tris[i+1]*3],
									 &verts[tris[i+2]*3], t))
		{
			if (t < tmin)
				tmin = t;
			hit = true;
		}
	}
	
	return hit;
}

struct FileList
{
	static const int MAX_FILES = 256;
	inline FileList() : size(0) {}
	inline ~FileList()
	{
		clear();
	}
	
	void clear()
	{
		for (int i = 0; i < size; ++i)
			delete [] files[i];
		size = 0;
	}
	
	void add(const char* path)
	{
		if (size >= MAX_FILES)
			return;
		int n = strlen(path);
		files[size] = new char[n+1];
		strcpy(files[size], path);
		size++;
	}
	
	static int cmp(const void* a, const void* b)
	{
		return strcmp(*(const char**)a, *(const char**)b);
	}
	
	void sort()
	{
		if (size > 1)
			qsort(files, size, sizeof(char*), cmp);
	}
	
	char* files[MAX_FILES];
	int size;
};

void scanDirectory(const char* path, const char* ext, FileList& list)
{
	list.clear();

#ifdef WIN32
	_finddata_t dir;
	char pathWithExt[MAX_PATH];
	long fh;
	strcpy(pathWithExt, path);
	strcat(pathWithExt, "/*");
	strcat(pathWithExt, ext);
	fh = _findfirst(pathWithExt, &dir);
	if (fh == -1L)
		return;
	do
	{
		list.add(dir.name);
	}
	while (_findnext(fh, &dir) == 0);
	_findclose(fh);
#else
	dirent* current = 0;
	DIR* dp = opendir(path);
	if (!dp)
		return;
	
	while ((current = readdir(dp)) != 0)
	{
		int len = strlen(current->d_name);
		if (len > 4 && strncmp(current->d_name+len-4, ext, 4) == 0)
		{
			list.add(current->d_name);
		}
	}
	closedir(dp);
#endif
	list.sort();
}


enum DrawMode
{
	DRAWMODE_NAVMESH,
	DRAWMODE_NAVMESH_TRANS,
	DRAWMODE_NAVMESH_BVTREE,
	DRAWMODE_NAVMESH_INVIS,
	DRAWMODE_MESH,
	DRAWMODE_VOXELS,
	DRAWMODE_VOXELS_WALKABLE,
	DRAWMODE_COMPACT,
	DRAWMODE_COMPACT_DISTANCE,
	DRAWMODE_COMPACT_REGIONS,
	DRAWMODE_REGION_CONNECTIONS,
	DRAWMODE_RAW_CONTOURS,
	DRAWMODE_BOTH_CONTOURS,
	DRAWMODE_CONTOURS,
	DRAWMODE_POLYMESH,
};

enum ToolMode
{
	TOOLMODE_PATHFIND,
	TOOLMODE_RAYCAST,
	TOOLMODE_DISTANCE_TO_WALL,
	TOOLMODE_FIND_POLYS_AROUND,
};


GLFont g_font;

void drawText(int x, int y, int dir, const char* text, unsigned int col)
{
	if (dir < 0)
		g_font.drawText((float)x - g_font.getTextLength(text), (float)y, text, col);
	else
		g_font.drawText((float)x, (float)y, text, col);
}



struct Tile
{
	inline Tile() : chf(0), cset(0), solid(0), buildTime(0) {}
	inline ~Tile() { delete chf; delete cset; delete solid; }
	rcCompactHeightfield* chf;
	rcHeightfield* solid;
	rcContourSet* cset;
	int buildTime;
};

struct TileSet
{
	inline TileSet() : width(0), height(0), tiles(0) {}
	inline ~TileSet() { delete [] tiles; }
	int width, height;
	float bmin[3], bmax[3];
	float cs, ch;
	Tile* tiles;
};


rcMeshLoaderObj* g_mesh = 0;
float g_meshBMin[3], g_meshBMax[3];
rcChunkyTriMesh* g_chunkyMesh = 0;
rcPolyMesh* g_polyMesh = 0;
dtStatNavMesh* g_navMesh = 0;
TileSet* g_tileSet = 0;
rcLog g_log;
rcBuildTimes g_buildTimes; 


bool buildTiledNavigation(const rcConfig& cfg,
						  const rcMeshLoaderObj* mesh,
						  const rcChunkyTriMesh* chunkyMesh,
						  TileSet* tileSet,
						  rcPolyMesh* polyMesh,
						  dtStatNavMesh* navMesh,
						  bool keepInterResults)
{
	if (!mesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Input mesh is not valid.");
		return false;
	}
	if (!chunkyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Input chunky mesh is not valid.");
		return false;
	}

	if (!tileSet)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Output tile set is not valid.");
		return false;
	}
	if (!polyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Output polymesh is not valid.");
		return false;
	}
	if (!navMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Output navmesh is not valid.");
		return false;
	}
	
	memset(&g_buildTimes, 0, sizeof(g_buildTimes));
	rcSetBuildTimes(&g_buildTimes);
	
	rcTimeVal totStartTime = rcGetPerformanceTimer();

	// Calculate the number of tiles in the output and initialize tiles.
	int gw = 0, gh = 0;
	rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &gw, &gh);
	vcopy(tileSet->bmin, cfg.bmin);
	vcopy(tileSet->bmax, cfg.bmax);
	tileSet->cs = cfg.cs;
	tileSet->ch = cfg.ch;
	tileSet->width = (gw + cfg.tileSize-1) / cfg.tileSize;
	tileSet->height = (gh + cfg.tileSize-1) / cfg.tileSize;
	tileSet->tiles = new Tile[tileSet->height * tileSet->width];
	if (!tileSet->tiles)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'tileSet->tiles' (%d).", tileSet->height * tileSet->width);
		return false;
	}

	if (rcGetLog())
	{
		rcGetLog()->log(RC_LOG_PROGRESS, "Building navigation:");
		rcGetLog()->log(RC_LOG_PROGRESS, " - %d x %d cells", gw, gh);
		rcGetLog()->log(RC_LOG_PROGRESS, " - %d x %d tiles", tileSet->width, tileSet->height);
		rcGetLog()->log(RC_LOG_PROGRESS, " - %d verts, %d tris", mesh->getVertCount(), mesh->getTriCount());
	}
	
	// Initialize per tile config.
	rcConfig tileCfg;
	memcpy(&tileCfg, &cfg, sizeof(rcConfig));
	tileCfg.width = cfg.tileSize + cfg.borderSize*2;
	tileCfg.height = cfg.tileSize + cfg.borderSize*2;
	
	// Allocate array that can hold triangle flags for all geom chunks.
	unsigned char* triangleFlags = new unsigned char[chunkyMesh->maxTrisPerChunk];
	if (!triangleFlags)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'triangleFlags' (%d).", chunkyMesh->maxTrisPerChunk);
		return false;
	}
	
	rcHeightfield* solid = 0;
	rcCompactHeightfield* chf = 0;
	rcContourSet* cset = 0;
	
	const float* verts = mesh->getVerts();
	const int nverts = mesh->getVertCount();
	
	for (int y = 0; y < tileSet->height; ++y)
	{
		for (int x = 0; x < tileSet->width; ++x)
		{
			rcTimeVal startTime = rcGetPerformanceTimer();

			Tile& tile = tileSet->tiles[x + y*tileSet->width]; 
			
			// Calculate the per tile bounding box.
			tileCfg.bmin[0] = cfg.bmin[0] + (x*cfg.tileSize - cfg.borderSize)*cfg.cs;
			tileCfg.bmin[2] = cfg.bmin[2] + (y*cfg.tileSize - cfg.borderSize)*cfg.cs;
			tileCfg.bmax[0] = cfg.bmin[0] + ((x+1)*cfg.tileSize + cfg.borderSize)*cfg.cs;
			tileCfg.bmax[2] = cfg.bmin[2] + ((y+1)*cfg.tileSize + cfg.borderSize)*cfg.cs;
			
			delete solid;
			delete chf;
			solid = 0;
			chf = 0;
			
			float tbmin[2], tbmax[2];
			tbmin[0] = tileCfg.bmin[0];
			tbmin[1] = tileCfg.bmin[2];
			tbmax[0] = tileCfg.bmax[0];
			tbmax[1] = tileCfg.bmax[2];
			int cid[256];// TODO: Make grow when returning too many items.
			const int ncid = rcGetChunksInRect(chunkyMesh, tbmin, tbmax, cid, 256);
			if (!ncid)
			{
				printf("Skipping empty %d,%d\n", x, y);
				continue;
			}
									
			solid = new rcHeightfield;
			if (!solid)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Out of memory 'solid'.", x, y);
				continue;
			}
			if (!rcCreateHeightfield(*solid, tileCfg.width, tileCfg.height, tileCfg.bmin, tileCfg.bmax, tileCfg.cs, tileCfg.ch))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not create solid heightfield.", x, y);
				continue;
			}
			
			for (int i = 0; i < ncid; ++i)
			{
				const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
				const int* tris = &chunkyMesh->tris[node.i*3];
				const int ntris = node.n;
				
				memset(triangleFlags, 0, ntris*sizeof(unsigned char));
				rcMarkWalkableTriangles(tileCfg.walkableSlopeAngle,
										verts, nverts, tris, ntris, triangleFlags);
				
				rcRasterizeTriangles(verts, nverts, tris, triangleFlags, ntris, *solid);
			}	
			
			rcFilterLedgeSpans(tileCfg.walkableHeight, tileCfg.walkableClimb, *solid);
			
			rcFilterWalkableLowHeightSpans(tileCfg.walkableHeight, *solid);
			
			chf = new rcCompactHeightfield;
			if (!chf)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Out of memory 'chf'.", x, y);
				continue;
			}
			if (!rcBuildCompactHeightfield(tileCfg.walkableHeight, tileCfg.walkableClimb,
										   RC_WALKABLE/*|RC_REACHABLE*/, *solid, *chf))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not build compact data.", x, y);
				continue;
			}
			
			if (!rcBuildDistanceField(*chf))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not build distance fields.", x, y);
				continue;
			}
			
			if (!rcBuildRegions(*chf, tileCfg.walkableRadius, tileCfg.borderSize, tileCfg.minRegionSize, tileCfg.mergeRegionSize))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not build regions.", x, y);
				continue;
			}
			
			cset = new rcContourSet;
			if (!cset)
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Out of memory 'cset'.", x, y);
				continue;
			}
			if (!rcBuildContours(*chf, tileCfg.maxSimplificationError, tileCfg.maxEdgeLen, *cset))
			{
				if (rcGetLog())
					rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not create contours.", x, y);
				continue;
			}
			
			if (keepInterResults)
			{
				tile.solid = solid;
				solid = 0;
				tile.chf = chf;
				chf = 0;
			}
			
			if (!cset->nconts)
			{
				delete cset;
				cset = 0;
				continue;
			}
			
			tile.cset = cset;
			// Offset the vertices in the cset.
			rcTranslateContours(tile.cset, x*tileCfg.tileSize - tileCfg.borderSize, 0, y*tileCfg.tileSize - tileCfg.borderSize);
							
			rcTimeVal endTime = rcGetPerformanceTimer();
			tile.buildTime += rcGetDeltaTimeUsec(startTime, endTime);
		}
	}
	
	delete [] triangleFlags;
	delete solid;
	delete chf;
	
	
	for (int y = 0; y < tileSet->height; ++y)
	{
		for (int x = 0; x < tileSet->width; ++x)
		{
			rcTimeVal startTime = rcGetPerformanceTimer();
			if ((x+1) < tileSet->width)
			{
				if (!rcFixupAdjacentContours(tileSet->tiles[x + y*tileSet->width].cset,
											 tileSet->tiles[x+1 + y*tileSet->width].cset,
											 cfg.walkableClimb, (x+1)*cfg.tileSize, -1))
				{
					if (rcGetLog())
						rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not fixup x+1.", x, y);
					return false;
				}
			}
			
			if ((y+1) < tileSet->height)
			{
				if (!rcFixupAdjacentContours(tileSet->tiles[x + y*tileSet->width].cset,
											 tileSet->tiles[x + (y+1)*tileSet->width].cset,
											 cfg.walkableClimb, -1, (y+1)*cfg.tileSize))
				{
					if (rcGetLog())
						rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: [%d,%d] Could not fixup y+1.", x, y);
					return false;
				}
			}
			rcTimeVal endTime = rcGetPerformanceTimer();
			tileSet->tiles[x+y*tileSet->width].buildTime += rcGetDeltaTimeUsec(startTime, endTime);
		}
	}

	 
	// Combine contours.
	rcContourSet combSet;

	combSet.nconts = 0;
	for (int y = 0; y < tileSet->height; ++y)
	{
		for (int x = 0; x < tileSet->width; ++x)
		{
			Tile& tile = tileSet->tiles[x + y*tileSet->width]; 
			if (!tile.cset) continue;
			combSet.nconts += tile.cset->nconts;
		}
	}
	combSet.conts = new rcContour[combSet.nconts];
	if (!combSet.conts)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'combSet.conts' (%d).", combSet.nconts);
		return false;
	}
	int n = 0;
	for (int y = 0; y < tileSet->height; ++y)
	{
		for (int x = 0; x < tileSet->width; ++x)
		{
			Tile& tile = tileSet->tiles[x + y*tileSet->width]; 
			if (!tile.cset) continue;
			for (int i = 0; i < tile.cset->nconts; ++i)
			{
				combSet.conts[n].verts = tile.cset->conts[i].verts;
				combSet.conts[n].nverts = tile.cset->conts[i].nverts;
				combSet.conts[n].reg = tile.cset->conts[i].reg;
				n++;
			}
		}
	}

	bool polyRes = rcBuildPolyMesh(combSet, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch, cfg.maxVertsPerPoly, *polyMesh);

	// Remove vertex binding to avoid double deletion.
	for (int i = 0; i < combSet.nconts; ++i)
	{
		combSet.conts[i].verts = 0;
		combSet.conts[i].nverts = 0;
	}
	
	if (!polyRes)
	{	
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildTiledNavigation: Could not triangulate contours.");
		return false;
	}
	

	if (cfg.maxVertsPerPoly == DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;
		if (!dtCreateNavMeshData(polyMesh->verts, polyMesh->nverts,
								 polyMesh->polys, polyMesh->npolys, polyMesh->nvp,
								 cfg.bmin, cfg.bmax, cfg.cs, cfg.ch, &navData, &navDataSize))
		{
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}
		
		if (!navMesh->init(navData, navDataSize, true))
		{
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}
	}
	
	rcTimeVal totEndTime = rcGetPerformanceTimer();

	if (rcGetLog())
	{
		const float pc = 100.0f / rcGetDeltaTimeUsec(totStartTime, totEndTime);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Rasterize: %.1fms (%.1f%%)", g_buildTimes.rasterizeTriangles/1000.0f, g_buildTimes.rasterizeTriangles*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Compact: %.1fms (%.1f%%)", g_buildTimes.buildCompact/1000.0f, g_buildTimes.buildCompact*pc);

		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Border: %.1fms (%.1f%%)", g_buildTimes.filterBorder/1000.0f, g_buildTimes.filterBorder*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Walkable: %.1fms (%.1f%%)", g_buildTimes.filterWalkable/1000.0f, g_buildTimes.filterWalkable*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Reachable: %.1fms (%.1f%%)", g_buildTimes.filterMarkReachable/1000.0f, g_buildTimes.filterMarkReachable*pc);

		rcGetLog()->log(RC_LOG_PROGRESS, "Build Distancefield: %.1fms (%.1f%%)", g_buildTimes.buildDistanceField/1000.0f, g_buildTimes.buildDistanceField*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - distance: %.1fms (%.1f%%)", g_buildTimes.buildDistanceFieldDist/1000.0f, g_buildTimes.buildDistanceFieldDist*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - blur: %.1fms (%.1f%%)", g_buildTimes.buildDistanceFieldBlur/1000.0f, g_buildTimes.buildDistanceFieldBlur*pc);

		rcGetLog()->log(RC_LOG_PROGRESS, "Build Regions: %.1fms (%.1f%%)", g_buildTimes.buildRegions/1000.0f, g_buildTimes.buildRegions*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - watershed: %.1fms (%.1f%%)", g_buildTimes.buildRegionsReg/1000.0f, g_buildTimes.buildRegionsReg*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "    - expand: %.1fms (%.1f%%)", g_buildTimes.buildRegionsExp/1000.0f, g_buildTimes.buildRegionsExp*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "    - find catchment basins: %.1fms (%.1f%%)", g_buildTimes.buildRegionsFlood/1000.0f, g_buildTimes.buildRegionsFlood*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - filter: %.1fms (%.1f%%)", g_buildTimes.buildRegionsFilter/1000.0f, g_buildTimes.buildRegionsFilter*pc);

		rcGetLog()->log(RC_LOG_PROGRESS, "Build Contours: %.1fms (%.1f%%)", g_buildTimes.buildContours/1000.0f, g_buildTimes.buildContours*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - trace: %.1fms (%.1f%%)", g_buildTimes.buildContoursTrace/1000.0f, g_buildTimes.buildContoursTrace*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - simplify: %.1fms (%.1f%%)", g_buildTimes.buildContoursSimplify/1000.0f, g_buildTimes.buildContoursSimplify*pc);

		rcGetLog()->log(RC_LOG_PROGRESS, "Fixup contours: %.1fms (%.1f%%)", g_buildTimes.fixupContours/1000.0f, g_buildTimes.fixupContours*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Polymesh: %.1fms (%.1f%%)", g_buildTimes.buildPolymesh/1000.0f, g_buildTimes.buildPolymesh*pc);

		if (polyMesh)
			rcGetLog()->log(RC_LOG_PROGRESS, "Polymesh: Verts:%d  Polys:%d", polyMesh->nverts, polyMesh->npolys);

		rcGetLog()->log(RC_LOG_PROGRESS, "TOTAL: %.1fms", rcGetDeltaTimeUsec(totStartTime, totEndTime)/1000.0f);
	}
	
	return true;
}


bool buildNavigation(const rcConfig& cfg,
					 const rcMeshLoaderObj* mesh,
					 const rcChunkyTriMesh* chunkyMesh,
					 TileSet* tileSet,
					 rcPolyMesh* polyMesh,
					 dtStatNavMesh* navMesh,
					 bool keepInterResults)
{
	if (!mesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not valid.");
		return false;
	}
	if (!chunkyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Input chunky mesh is not valid.");
		return false;
	}
	
	if (!tileSet)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Output tile set is not valid.");
		return false;
	}
	if (!polyMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Output polymesh is not valid.");
		return false;
	}
	if (!navMesh)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Output navmesh is not valid.");
		return false;
	}
	
	memset(&g_buildTimes, 0, sizeof(g_buildTimes));
	rcSetBuildTimes(&g_buildTimes);
	
	rcTimeVal totStartTime = rcGetPerformanceTimer();
	
	// Create one tile so that we can use the same debug output as with the tiled generation.
	vcopy(tileSet->bmin, cfg.bmin);
	vcopy(tileSet->bmax, cfg.bmax);
	tileSet->cs = cfg.cs;
	tileSet->ch = cfg.ch;
	tileSet->width = 1;
	tileSet->height = 1;
	tileSet->tiles = new Tile[1];
	if (!tileSet->tiles)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'tileSet->tiles'.");
		return false;
	}
	
	if (rcGetLog())
	{
		rcGetLog()->log(RC_LOG_PROGRESS, "Building navigation:");
		rcGetLog()->log(RC_LOG_PROGRESS, " - %d x %d cells", cfg.width, cfg.height);
		rcGetLog()->log(RC_LOG_PROGRESS, " - %d verts, %d tris", mesh->getVertCount(), mesh->getTriCount());
	}
	
	// Initialize per tile config.
	
	// Allocate array that can hold triangle flags for all geom chunks.
	unsigned char* triangleFlags = new unsigned char[mesh->getTriCount()];
	if (!triangleFlags)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'triangleFlags' (%d).", mesh->getTriCount());
		return false;
	}
	
	rcHeightfield* solid = 0;
	rcCompactHeightfield* chf = 0;
	rcContourSet* cset = 0;
	
	solid = new rcHeightfield;
	if (!solid)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	if (!rcCreateHeightfield(*solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}
	
	rcTimeVal startTime = rcGetPerformanceTimer();

	memset(triangleFlags, 0, mesh->getTriCount()*sizeof(unsigned char));
	rcMarkWalkableTriangles(cfg.walkableSlopeAngle,
							mesh->getVerts(), mesh->getVertCount(), mesh->getTris(), mesh->getTriCount(), triangleFlags);
	
	rcRasterizeTriangles(mesh->getVerts(), mesh->getVertCount(), mesh->getTris(), triangleFlags, mesh->getTriCount(), *solid);
	
	rcFilterLedgeSpans(cfg.walkableHeight, cfg.walkableClimb, *solid);
	
	rcFilterWalkableLowHeightSpans(cfg.walkableHeight, *solid);
	
	chf = new rcCompactHeightfield;
	if (!chf)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(cfg.walkableHeight, cfg.walkableClimb,
								   RC_WALKABLE/*|RC_REACHABLE*/, *solid, *chf))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}
	
	if (!rcBuildDistanceField(*chf))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not build distance fields.");
		return false;
	}
	
	if (!rcBuildRegions(*chf, cfg.walkableRadius, cfg.borderSize, cfg.minRegionSize, cfg.mergeRegionSize))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not build regions.");
		return false;
	}
	
	cset = new rcContourSet;
	if (!cset)
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(*chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}
	
	if (keepInterResults)
	{
		tileSet->tiles[0].solid = solid;
		solid = 0;
		tileSet->tiles[0].chf = chf;
		chf = 0;
	}
	
	tileSet->tiles[0].cset = cset;
	
	rcTimeVal endTime = rcGetPerformanceTimer();
	tileSet->tiles[0].buildTime += rcGetDeltaTimeUsec(startTime, endTime);
	
	delete [] triangleFlags;
	delete solid;
	delete chf;
	
	if (!rcBuildPolyMesh(*tileSet->tiles[0].cset, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch,
						 cfg.maxVertsPerPoly, *polyMesh))
	{
		if (rcGetLog())
			rcGetLog()->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}
	
	if (cfg.maxVertsPerPoly == DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;
		if (!dtCreateNavMeshData(polyMesh->verts, polyMesh->nverts,
								 polyMesh->polys, polyMesh->npolys, polyMesh->nvp,
								 cfg.bmin, cfg.bmax, cfg.cs, cfg.ch, &navData, &navDataSize))
		{
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}

		if (!navMesh->init(navData, navDataSize, true))
		{
			if (rcGetLog())
				rcGetLog()->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}
	}
	
	rcTimeVal totEndTime = rcGetPerformanceTimer();
	
	if (rcGetLog())
	{
		const float pc = 100.0f / rcGetDeltaTimeUsec(totStartTime, totEndTime);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Rasterize: %.1fms (%.1f%%)", g_buildTimes.rasterizeTriangles/1000.0f, g_buildTimes.rasterizeTriangles*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Compact: %.1fms (%.1f%%)", g_buildTimes.buildCompact/1000.0f, g_buildTimes.buildCompact*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Border: %.1fms (%.1f%%)", g_buildTimes.filterBorder/1000.0f, g_buildTimes.filterBorder*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Walkable: %.1fms (%.1f%%)", g_buildTimes.filterWalkable/1000.0f, g_buildTimes.filterWalkable*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "Filter Reachable: %.1fms (%.1f%%)", g_buildTimes.filterMarkReachable/1000.0f, g_buildTimes.filterMarkReachable*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Distancefield: %.1fms (%.1f%%)", g_buildTimes.buildDistanceField/1000.0f, g_buildTimes.buildDistanceField*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - distance: %.1fms (%.1f%%)", g_buildTimes.buildDistanceFieldDist/1000.0f, g_buildTimes.buildDistanceFieldDist*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - blur: %.1fms (%.1f%%)", g_buildTimes.buildDistanceFieldBlur/1000.0f, g_buildTimes.buildDistanceFieldBlur*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Regions: %.1fms (%.1f%%)", g_buildTimes.buildRegions/1000.0f, g_buildTimes.buildRegions*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - watershed: %.1fms (%.1f%%)", g_buildTimes.buildRegionsReg/1000.0f, g_buildTimes.buildRegionsReg*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "    - expand: %.1fms (%.1f%%)", g_buildTimes.buildRegionsExp/1000.0f, g_buildTimes.buildRegionsExp*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "    - find catchment basins: %.1fms (%.1f%%)", g_buildTimes.buildRegionsFlood/1000.0f, g_buildTimes.buildRegionsFlood*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - filter: %.1fms (%.1f%%)", g_buildTimes.buildRegionsFilter/1000.0f, g_buildTimes.buildRegionsFilter*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Contours: %.1fms (%.1f%%)", g_buildTimes.buildContours/1000.0f, g_buildTimes.buildContours*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - trace: %.1fms (%.1f%%)", g_buildTimes.buildContoursTrace/1000.0f, g_buildTimes.buildContoursTrace*pc);
		rcGetLog()->log(RC_LOG_PROGRESS, "  - simplify: %.1fms (%.1f%%)", g_buildTimes.buildContoursSimplify/1000.0f, g_buildTimes.buildContoursSimplify*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Fixup contours: %.1fms (%.1f%%)", g_buildTimes.fixupContours/1000.0f, g_buildTimes.fixupContours*pc);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "Build Polymesh: %.1fms (%.1f%%)", g_buildTimes.buildPolymesh/1000.0f, g_buildTimes.buildPolymesh*pc);
		
		if (polyMesh)
			rcGetLog()->log(RC_LOG_PROGRESS, "Polymesh: Verts:%d  Polys:%d", polyMesh->nverts, polyMesh->npolys);
		
		rcGetLog()->log(RC_LOG_PROGRESS, "TOTAL: %.1fms", rcGetDeltaTimeUsec(totStartTime, totEndTime)/1000.0f);
	}
	
	return true;
}


int main(int argc, char *argv[])
{
	// Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		printf("Could not initialise SDL\n");
		return -1;
	}
	
	// Init OpenGL
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 32);
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);

	int menuOffset = 0;
	
	int width = 1200;
	int height = 700;
	SDL_Surface* screen = SDL_SetVideoMode(width, height, 0, SDL_OPENGL);
	if (!screen)
	{
		printf("Could not initialise SDL opengl\n");
		return -1;
	}
	
	SDL_WM_SetCaption("Recast Demo", 0);
	
	if(!g_font.create("font.cfnt"))
	{
		printf("Could not load font.\n");
		SDL_Quit();
		return -1;
	}
	
	float cellSize = 0.3f;
	float cellHeight = 0.2f;
	float agentHeight = 2.0f;
	float agentRadius = 0.6f;
	float agentMaxClimb = 0.9f;
	float agentMaxSlope = 45.0f;
	float regionMinSize = 50;
	float regionMergeSize = 20;
	float edgeMaxLen = 12.0f;
	float edgeMaxError = 1.3f;
	float vertsPerPoly = 6.0f;
	float tileSize = 0.0f;
	int drawMode = DRAWMODE_NAVMESH;
	int toolMode = TOOLMODE_PATHFIND;
	bool showMenu = true;
	bool showLevels = false;
	bool showLog = false;
	bool showTools = true;
	char curLevel[256] = "Choose Level...";
	bool mouseOverMenu = false;
	bool keepInterResults = false;
	FileList fileList;	
	
	dtPolyRef startRef = 0, endRef = 0;

	const float polyPickExt[3] = {2,4,2};
	
	static const int MAX_POLYS = 256;
	dtPolyRef polys[MAX_POLYS];
	int npolys = 0;
	float straightPath[MAX_POLYS*3];
	int nstraightPath = 0;
	
	float t = 0.0f;
	Uint32 lastTime = SDL_GetTicks();
	int mx = 0, my = 0;
	float rx = 45;
	float ry = -45;
	float moveW = 0, moveS = 0, moveA = 0, moveD = 0;
	float camx = 0, camy = 0, camz = 0, camr=10;
	float origrx, origry;
	int origx, origy;
	bool rotate = false;
	float rays[3], raye[3]; 
	float spos[3] = {0,0,0};
	float epos[3] = {0,0,0};
	float hitPos[3] = {0,0,0};
	float hitNormal[3] = {0,0,0};
	float distanceToWall = 0;
	bool sposSet = false, eposSet = false;
	static const float startCol[4] = { 0.5f, 0.1f, 0.0f, 0.75f };
	static const float endCol[4] = { 0.2f, 0.4f, 0.0f, 0.75f };
	bool recalcTool = false;
	
	glEnable(GL_CULL_FACE);

//	float fogCol[4] = { 0.1f,0.12f,0.14f,1 };
	float fogCol[4] = { 0.32f,0.25f,0.25f,1 };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, 0);
	glFogf(GL_FOG_END, 10);
	glFogfv(GL_FOG_COLOR, fogCol);
	
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	
	bool done = false;
	while(!done)
	{
		// Handle input events.
		SDL_Event event;
		while(SDL_PollEvent(&event))
		{
			switch(event.type)
			{
				case SDL_KEYDOWN:
					// Handle any key presses here.
					if (event.key.keysym.sym == SDLK_ESCAPE)
					{
						done = true;
					}
					break;

				case SDL_MOUSEBUTTONDOWN:
					// Handle mouse clicks here.
					if (!mouseOverMenu)
					{
						if (event.button.button == SDL_BUTTON_RIGHT)
						{
							// Rotate view
							rotate = true;
							origx = mx;
							origy = my;
							origrx = rx;
							origry = ry;
						}
						else if (event.button.button == SDL_BUTTON_LEFT)
						{
							// Hit test mesh.
							if (g_mesh)
							{
								float t;
								if (raycast(*g_mesh, rays, raye, t))
								{
									if (SDL_GetModState() & KMOD_SHIFT)
									{
										sposSet = true;
										spos[0] = rays[0] + (raye[0] - rays[0])*t;
										spos[1] = rays[1] + (raye[1] - rays[1])*t;
										spos[2] = rays[2] + (raye[2] - rays[2])*t;
										if (g_navMesh)
											startRef = g_navMesh->findNearestPoly(spos, polyPickExt);
										recalcTool = true;
									}
									else
									{
										eposSet = true;
										epos[0] = rays[0] + (raye[0] - rays[0])*t;
										epos[1] = rays[1] + (raye[1] - rays[1])*t;
										epos[2] = rays[2] + (raye[2] - rays[2])*t;
										if (g_navMesh)
											endRef = g_navMesh->findNearestPoly(epos, polyPickExt);
										recalcTool = true;
									}
								}
							}
						}
					}	
					break;
					
				case SDL_MOUSEBUTTONUP:
					// Handle mouse clicks here.
					if(event.button.button == SDL_BUTTON_RIGHT)
					{
						rotate = false;
					}
					break;
					
				case SDL_MOUSEMOTION:
					mx = event.motion.x;
					my = height - 1 - event.motion.y;
					if (rotate)
					{
						int dx = mx - origx;
						int dy = my - origy;
						rx = origrx - dy*0.25f;
						ry = origry + dx*0.25f;
					}
					break;
					
				case SDL_QUIT:
					done = true;
					break;
					
				default:
					break;
			}
		}
		
		Uint32	time = SDL_GetTicks();
		float	dt = (time - lastTime) / 1000.0f;
		lastTime = time;
		
		t += dt;
				
		// Update and render
		glViewport(0, 0, width, height);
		glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		
		// Render 3d
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, (float)width/(float)height, 1.0f, camr);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(rx,1,0,0);
		glRotatef(ry,0,1,0);
		glTranslatef(-camx, -camy, -camz);
		
		// Get hit ray position and direction.
		GLdouble proj[16];
		GLdouble model[16];
		GLint view[4];
		glGetDoublev(GL_PROJECTION_MATRIX, proj);
		glGetDoublev(GL_MODELVIEW_MATRIX, model);
		glGetIntegerv(GL_VIEWPORT, view);
		GLdouble x, y, z;
		gluUnProject(mx, my, 0.0f, model, proj, view, &x, &y, &z);
		rays[0] = (float)x; rays[1] = (float)y; rays[2] = (float)z;
		gluUnProject(mx, my, 1.0f, model, proj, view, &x, &y, &z);
		raye[0] = (float)x; raye[1] = (float)y; raye[2] = (float)z;
		
		// Handle keyboard movement.
		Uint8* keystate = SDL_GetKeyState(NULL);
		moveW = rcClamp(moveW + dt * 4 * (keystate[SDLK_w] ? 1 : -1), 0.0f, 1.0f);
		moveS = rcClamp(moveS + dt * 4 * (keystate[SDLK_s] ? 1 : -1), 0.0f, 1.0f);
		moveA = rcClamp(moveA + dt * 4 * (keystate[SDLK_a] ? 1 : -1), 0.0f, 1.0f);
		moveD = rcClamp(moveD + dt * 4 * (keystate[SDLK_d] ? 1 : -1), 0.0f, 1.0f);
		
		float keybSpeed = 22.0f;
		if (SDL_GetModState() & KMOD_SHIFT)
			keybSpeed *= 4.0f;
		
		float movex = (moveD - moveA) * keybSpeed * dt;
		float movey = (moveS - moveW) * keybSpeed * dt;
		
		camx += movex * (float)model[0];
		camy += movex * (float)model[4];
		camz += movex * (float)model[8];
		
		camx += movey * (float)model[2];
		camy += movey * (float)model[6];
		camz += movey * (float)model[10];
		
		glEnable(GL_FOG);
		
		if (drawMode == DRAWMODE_MESH)
		{
			if (g_mesh)
				rcDebugDrawMeshSlope(*g_mesh, agentMaxSlope);
		}
		else if (drawMode != DRAWMODE_NAVMESH_TRANS)
		{
			if (g_mesh)
				rcDebugDrawMesh(*g_mesh, 0);
		}
		
		glDisable(GL_FOG);
		
		glDepthMask(GL_FALSE);
		
		if (drawMode == DRAWMODE_NAVMESH ||
			drawMode == DRAWMODE_NAVMESH_TRANS ||
			drawMode == DRAWMODE_NAVMESH_BVTREE ||
			drawMode == DRAWMODE_NAVMESH_INVIS)
		{
			if (g_navMesh)
			{
				if (drawMode != DRAWMODE_NAVMESH_INVIS)
					dtDebugDrawStatNavMesh(g_navMesh);

				if (toolMode == TOOLMODE_PATHFIND)
				{
					dtDebugDrawStatNavMeshPoly(g_navMesh, startRef, startCol);
					dtDebugDrawStatNavMeshPoly(g_navMesh, endRef, endCol);
					
					if (npolys)
					{
						const float pathCol[4] = {0,0,0,0.25f}; //{1,0.75f,0,0.25f};
						for (int i = 1; i < npolys-1; ++i)
							dtDebugDrawStatNavMeshPoly(g_navMesh, polys[i], pathCol);
					}
					if (nstraightPath)
					{
						glColor4ub(128,16,0,220);
						glLineWidth(3.0f);
						glBegin(GL_LINE_STRIP);
						for (int i = 0; i < nstraightPath; ++i)
							glVertex3f(straightPath[i*3], straightPath[i*3+1]+0.4f, straightPath[i*3+2]);
						glEnd();
						glLineWidth(1.0f);
						glPointSize(4.0f);
						glBegin(GL_POINTS);
						for (int i = 0; i < nstraightPath; ++i)
							glVertex3f(straightPath[i*3], straightPath[i*3+1]+0.4f, straightPath[i*3+2]);
						glEnd();
						glPointSize(1.0f);
					}
				}
				else if (toolMode == TOOLMODE_RAYCAST)
				{
					dtDebugDrawStatNavMeshPoly(g_navMesh, startRef, startCol);

					if (nstraightPath)
					{
						const float pathCol[4] = {1,0.75f,0,0.25f};
						dtDebugDrawStatNavMeshPoly(g_navMesh, polys[0], pathCol);
						
						glColor4ub(128,16,0,220);
						glLineWidth(3.0f);
						glBegin(GL_LINE_STRIP);
						for (int i = 0; i < nstraightPath; ++i)
							glVertex3f(straightPath[i*3], straightPath[i*3+1]+0.4f, straightPath[i*3+2]);
						glEnd();
						glLineWidth(1.0f);
						glPointSize(4.0f);
						glBegin(GL_POINTS);
						for (int i = 0; i < nstraightPath; ++i)
							glVertex3f(straightPath[i*3], straightPath[i*3+1]+0.4f, straightPath[i*3+2]);
						glEnd();
						glPointSize(1.0f);
					}
				}
				else if (toolMode == TOOLMODE_DISTANCE_TO_WALL)
				{
					dtDebugDrawStatNavMeshPoly(g_navMesh, startRef, startCol);
					const float col[4] = {1,1,1,0.5f};
					rcDebugDrawCylinderWire(spos[0]-distanceToWall, spos[1]+0.02f, spos[2]-distanceToWall,
											spos[0]+distanceToWall, spos[1]+agentHeight, spos[2]+distanceToWall, col);
					glLineWidth(3.0f);
					glColor4fv(col);
					glBegin(GL_LINES);
					glVertex3f(hitPos[0], hitPos[1] + 0.02f, hitPos[2]);
					glVertex3f(hitPos[0], hitPos[1] + agentHeight, hitPos[2]);
					glEnd();
					glLineWidth(1.0f);
				}
				else if (toolMode == TOOLMODE_FIND_POLYS_AROUND)
				{
					const float pathCol[4] = {0,0,0,0.25f}; //{1,0.75f,0,0.25f};
					for (int i = 0; i < npolys; ++i)
						dtDebugDrawStatNavMeshPoly(g_navMesh, polys[i], pathCol);
					
					const float dx = epos[0] - spos[0];
					const float dz = epos[2] - spos[2];
					float dist = sqrtf(dx*dx + dz*dz);
					const float col[4] = {1,1,1,0.5f};
					rcDebugDrawCylinderWire(spos[0]-dist, spos[1]+0.02f, spos[2]-dist,
											spos[0]+dist, spos[1]+agentHeight, spos[2]+dist, col);					
				}
			}
		}
		if (drawMode == DRAWMODE_NAVMESH_BVTREE)
		{
			if (g_navMesh)
				dtDebugDrawStatNavMeshBVTree(g_navMesh);
		}

		glDepthMask(GL_TRUE);
		
		if (drawMode == DRAWMODE_COMPACT)
		{
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].chf)
						rcDebugDrawCompactHeightfieldSolid(*(g_tileSet->tiles[i].chf));
				}
			}
		}
		if (drawMode == DRAWMODE_COMPACT_DISTANCE)
		{
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].chf)
						rcDebugDrawCompactHeightfieldDistance(*(g_tileSet->tiles[i].chf));
				}
			}
		}
		if (drawMode == DRAWMODE_COMPACT_REGIONS)
		{
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].chf)
						rcDebugDrawCompactHeightfieldRegions(*(g_tileSet->tiles[i].chf));
				}
			}
		}
		if (drawMode == DRAWMODE_VOXELS)
		{
			glEnable(GL_FOG);
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].solid)
						rcDebugDrawHeightfieldSolid(*g_tileSet->tiles[i].solid);
				}
			}
			glDisable(GL_FOG);
		}
		if (drawMode == DRAWMODE_VOXELS_WALKABLE)
		{
			glEnable(GL_FOG);
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].solid)
						rcDebugDrawHeightfieldWalkable(*g_tileSet->tiles[i].solid);
				}
			}
			glDisable(GL_FOG);
		}
		if (drawMode == DRAWMODE_RAW_CONTOURS)
		{
			glDepthMask(GL_FALSE);
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].cset)
						rcDebugDrawRawContours(*(g_tileSet->tiles[i].cset), g_tileSet->bmin, g_tileSet->cs, g_tileSet->ch);
				}
			}
			glDepthMask(GL_TRUE);
		}
		if (drawMode == DRAWMODE_BOTH_CONTOURS)
		{
			glDepthMask(GL_FALSE);
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].cset)
					{
						rcDebugDrawRawContours(*(g_tileSet->tiles[i].cset), g_tileSet->bmin, g_tileSet->cs, g_tileSet->ch, 0.5f);
						rcDebugDrawContours(*(g_tileSet->tiles[i].cset), g_tileSet->bmin, g_tileSet->cs, g_tileSet->ch);
					}
				}
			}
			glDepthMask(GL_TRUE);
		}
		if (drawMode == DRAWMODE_CONTOURS)
		{
			glDepthMask(GL_FALSE);
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].cset)
						rcDebugDrawContours(*(g_tileSet->tiles[i].cset), g_tileSet->bmin, g_tileSet->cs, g_tileSet->ch);
				}
			}
			glDepthMask(GL_TRUE);
		}
		if (drawMode == DRAWMODE_REGION_CONNECTIONS)
		{
			if (g_tileSet)
			{
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].chf)
						rcDebugDrawCompactHeightfieldRegions(*(g_tileSet->tiles[i].chf));
				}
				glDepthMask(GL_TRUE);
				
				glDepthMask(GL_FALSE);
				for (int i = 0; i < g_tileSet->width*g_tileSet->height; ++i)
				{
					if (g_tileSet->tiles[i].cset)
					{
						//						rcDebugDrawRawContours(*(g_tileSet->tiles[i].cset), g_tileSet->bmin, g_tileSet->cs, g_tileSet->ch);
						rcDebugDrawRegionConnections(*(g_tileSet->tiles[i].cset), g_tileSet->bmin, g_tileSet->cs, g_tileSet->ch);
					}
				}
				glDepthMask(GL_TRUE);
			}
		}
		if (drawMode == DRAWMODE_POLYMESH)
		{
			glDepthMask(GL_FALSE);
			if (g_polyMesh)
				rcDebugDrawPolyMesh(*g_polyMesh);
			glDepthMask(GL_TRUE);
		}
		
		if (g_mesh)
		{
			glDepthMask(GL_FALSE);
			
			// Agent dimensions.
			const float r = agentRadius;
			const float h = agentHeight;
			
			float col[4];
			
			for (int i = 0; i < 2; ++i)
			{
				const float* pos = 0;
				const float* c = 0;
				if (i == 0 && sposSet)
				{
					pos = spos;
					c = startCol;
				}
				else if (i == 1 && eposSet)
				{
					pos = epos;
					c = endCol;
				}
				if (!pos)
					continue;
				glLineWidth(2.0f);
				rcDebugDrawCylinderWire(pos[0]-r, pos[1]+0.02f, pos[2]-r, pos[0]+r, pos[1]+h, pos[2]+r, c);
				glLineWidth(1.0f);
				
				glColor4ub(0,0,0,196);
				glBegin(GL_LINES);
				glVertex3f(pos[0], pos[1]-agentMaxClimb, pos[2]);
				glVertex3f(pos[0], pos[1]+agentMaxClimb, pos[2]);
				glVertex3f(pos[0]-r/2, pos[1]+0.02f, pos[2]);
				glVertex3f(pos[0]+r/2, pos[1]+0.02f, pos[2]);
				glVertex3f(pos[0], pos[1]+0.02f, pos[2]-r/2);
				glVertex3f(pos[0], pos[1]+0.02f, pos[2]+r/2);
				glEnd();
			}
			
			// Tile bboxes
			if ((int)tileSize > 0)
			{
				const int ts = (int)tileSize;
				col[0] = 0.5f; col[1] = 0.1f; col[2] = 0.1f; col[3] = 0.15f;
				int gw = 0, gh = 0;
				rcCalcGridSize(g_meshBMin, g_meshBMax, cellSize, &gw, &gh);
				int tx = (gw + ts-1) / ts;
				int ty = (gh + ts-1) / ts;
				
				const float s = ts*cellSize;
				
				glBegin(GL_LINES);
				glColor4ub(0,0,0,64);
				for (int y = 0; y < ty; ++y)
				{
					for (int x = 0; x < tx; ++x)
					{
						float fx, fy, fz;
						fx = g_meshBMin[0] + x*s;
						fy = g_meshBMin[1];
						fz = g_meshBMin[2] + y*s;
						
						glVertex3f(fx,fy,fz);
						glVertex3f(fx+s,fy,fz);
						glVertex3f(fx,fy,fz);
						glVertex3f(fx,fy,fz+s);
						
						if (x+1 >= tx)
						{
							glVertex3f(fx+s,fy,fz);
							glVertex3f(fx+s,fy,fz+s);
						}
						if (y+1 >= ty)
						{
							glVertex3f(fx,fy,fz+s);
							glVertex3f(fx+s,fy,fz+s);
						}
					}
				}
				glEnd();
			}
			
			// Mesh bbox.
			col[0] = 1.0f; col[1] = 1.0f; col[2] = 1.0f; col[3] = 0.25f;
			rcDebugDrawBoxWire(g_meshBMin[0], g_meshBMin[1], g_meshBMin[2],
							   g_meshBMax[0], g_meshBMax[1], g_meshBMax[2], col);
			

			glDepthMask(GL_TRUE);
		}
		
		
		// Render GUI
		glDisable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0, width, 0, height);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		
		
		imguiBeginFrame();
		
		mouseOverMenu = false;
		
		if (showMenu)
		{
		
		static int propScroll = 0;
		if (imguiBeginScrollArea(GENID, "Properties", width - (250+menuOffset) - 10, 10, 250, height-20, &propScroll))
			mouseOverMenu = true;
		
		if (imguiButton(GENID, curLevel))
		{
			if (showLevels)
			{
				showLevels = false;
			}
			else
			{
				showLevels = true;
				scanDirectory("meshes", ".obj", fileList);
			}
		}
		
		
		imguiSeparator();
		
		if (g_mesh)
		{
			if (imguiButton(GENID, "Build"))
			{
				npolys = 0;
				nstraightPath = 0;
				sposSet = false;
				eposSet = false;
				startRef = 0;
				endRef = 0;
				distanceToWall = 0;
				
				rcConfig cfg;
				memset(&cfg, 0, sizeof(cfg));
				cfg.cs = cellSize;
				cfg.ch = cellHeight;
				cfg.walkableSlopeAngle = agentMaxSlope;
				cfg.walkableHeight = (int)ceilf(agentHeight / cfg.ch);
				cfg.walkableClimb = (int)ceilf(agentMaxClimb / cfg.ch);
				cfg.walkableRadius = (int)ceilf(agentRadius / cfg.cs);
				cfg.maxEdgeLen = (int)(edgeMaxLen / cellSize);
				cfg.maxSimplificationError = edgeMaxError;
				cfg.minRegionSize = (int)rcSqr(regionMinSize);
				cfg.mergeRegionSize = (int)rcSqr(regionMergeSize);
				cfg.maxVertsPerPoly = /*DT_VERTS_PER_POLYGON; // TODO: Handle better.*/ (int)vertsPerPoly;
				rcCalcBounds(g_mesh->getVerts(), g_mesh->getVertCount(), cfg.bmin, cfg.bmax);
				rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

				g_log.clear();
				rcSetLog(&g_log);
				
				if ((int)tileSize > 0)
				{
					cfg.borderSize = cfg.walkableRadius*2 + 2;
					cfg.tileSize = (int)tileSize;
					
					delete g_tileSet;
					g_tileSet = new TileSet;
					delete g_polyMesh;
					g_polyMesh = new rcPolyMesh;
					delete g_navMesh;
					g_navMesh = new dtStatNavMesh;
					
					buildTiledNavigation(cfg, g_mesh, g_chunkyMesh, g_tileSet, g_polyMesh, g_navMesh, keepInterResults);
				}
				else
				{					
					cfg.borderSize = 0;
					cfg.tileSize = 0;
					
					delete g_tileSet;
					g_tileSet = new TileSet;
					delete g_polyMesh;
					g_polyMesh = new rcPolyMesh;
					delete g_navMesh;
					g_navMesh = new dtStatNavMesh;

					buildNavigation(cfg, g_mesh, g_chunkyMesh, g_tileSet, g_polyMesh, g_navMesh, keepInterResults);
				}

			}
		}
		
		imguiSeparator();
		
		if (imguiCheck(GENID, "Show Log", showLog))
			showLog = !showLog;

		if (imguiCheck(GENID, "Show Tools", showTools))
			showTools = !showTools;

		if (imguiCheck(GENID, "Keep Itermediate Results", keepInterResults))
			keepInterResults = !keepInterResults;
		
		imguiSeparator();
		imguiLabel(GENID, "Rasterization");
		imguiSlider(GENID, "Cell Size", &cellSize, 0.1f, 1.0f, 0.01f);
		imguiSlider(GENID, "Cell Height", &cellHeight, 0.1f, 1.0f, 0.01f);
		imguiSlider(GENID, "TileSize", &tileSize, 0.0f, 1024.0f, 16.0f);
		
		if (g_mesh)
		{
			int gw = 0, gh = 0;
			rcCalcGridSize(g_meshBMin, g_meshBMax, cellSize, &gw, &gh);
			char text[64];
			snprintf(text, 64, "Verts: %.1fk  Tris: %.1fk", g_mesh->getVertCount()/1000.0f, g_mesh->getTriCount()/1000.0f);
			imguiValue(GENID, text);
			snprintf(text, 64, "Grid %d x %d", gw, gh);
			imguiValue(GENID, text);
		}
		
		imguiSeparator();
		imguiLabel(GENID, "Agent");
		imguiSlider(GENID, "Height", &agentHeight, 0.1f, 5.0f, 0.1f);
		imguiSlider(GENID, "Radius", &agentRadius, 0.0f, 5.0f, 0.1f);
		imguiSlider(GENID, "Max Climb", &agentMaxClimb, 0.1f, 5.0f, 0.1f);
		imguiSlider(GENID, "Max Slope", &agentMaxSlope, 0.0f, 90.0f, 1.0f);
		
		imguiSeparator();
		imguiLabel(GENID, "Region");
		imguiSlider(GENID, "Min Region Size", &regionMinSize, 0.0f, 150.0f, 1.0f);
		imguiSlider(GENID, "Merged Region Size", &regionMergeSize, 0.0f, 150.0f, 1.0f);
		
		imguiSeparator();
		imguiLabel(GENID, "Polygonization");
		imguiSlider(GENID, "Max Edge Length", &edgeMaxLen, 0.0f, 50.0f, 1.0f);
		imguiSlider(GENID, "Max Edge Error", &edgeMaxError, 0.1f, 3.0f, 0.1f);
		imguiSlider(GENID, "Verts Per Poly", &vertsPerPoly, 3.0f, 12.0f, 1.0f);		
		
		imguiSeparator();
		imguiLabel(GENID, "Draw");
		if (imguiCheck(GENID, "Input Mesh", drawMode == DRAWMODE_MESH))
			drawMode = DRAWMODE_MESH;
		if (imguiCheck(GENID, "Navmesh", drawMode == DRAWMODE_NAVMESH))
			drawMode = DRAWMODE_NAVMESH;
		if (imguiCheck(GENID, "Navmesh Invis", drawMode == DRAWMODE_NAVMESH_INVIS))
			drawMode = DRAWMODE_NAVMESH_INVIS;
		if (imguiCheck(GENID, "Navmesh Trans", drawMode == DRAWMODE_NAVMESH_TRANS))
			drawMode = DRAWMODE_NAVMESH_TRANS;
		if (imguiCheck(GENID, "Navmesh BVTree", drawMode == DRAWMODE_NAVMESH_BVTREE))
			drawMode = DRAWMODE_NAVMESH_BVTREE;
		if (imguiCheck(GENID, "Voxels", drawMode == DRAWMODE_VOXELS))
			drawMode = DRAWMODE_VOXELS;
		if (imguiCheck(GENID, "Walkable Voxels", drawMode == DRAWMODE_VOXELS_WALKABLE))
			drawMode = DRAWMODE_VOXELS_WALKABLE;
		if (imguiCheck(GENID, "Compact", drawMode == DRAWMODE_COMPACT))
			drawMode = DRAWMODE_COMPACT;
		if (imguiCheck(GENID, "Compact Distance", drawMode == DRAWMODE_COMPACT_DISTANCE))
			drawMode = DRAWMODE_COMPACT_DISTANCE;
		if (imguiCheck(GENID, "Compact Regions", drawMode == DRAWMODE_COMPACT_REGIONS))
			drawMode = DRAWMODE_COMPACT_REGIONS;
		if (imguiCheck(GENID, "Region Connections", drawMode == DRAWMODE_REGION_CONNECTIONS))
			drawMode = DRAWMODE_REGION_CONNECTIONS;
		if (imguiCheck(GENID, "Raw Contours", drawMode == DRAWMODE_RAW_CONTOURS))
			drawMode = DRAWMODE_RAW_CONTOURS;
		if (imguiCheck(GENID, "Both Contours", drawMode == DRAWMODE_BOTH_CONTOURS))
			drawMode = DRAWMODE_BOTH_CONTOURS;
		if (imguiCheck(GENID, "Contours", drawMode == DRAWMODE_CONTOURS))
			drawMode = DRAWMODE_CONTOURS;
		if (imguiCheck(GENID, "Poly Mesh", drawMode == DRAWMODE_POLYMESH))
			drawMode = DRAWMODE_POLYMESH;
		
		imguiEndScrollArea();
		
		}

		// Tools
		if (showTools)
		{
			static int toolsScroll = 0;
			if (imguiBeginScrollArea(GENID, "Tools", 10, height - 10 - 200, 150, 200, &toolsScroll))
				mouseOverMenu = true;

			if (imguiCheck(GENID, "Pathfind", toolMode == TOOLMODE_PATHFIND))
			{
				toolMode = TOOLMODE_PATHFIND;
				recalcTool = true;
			}
			if (imguiCheck(GENID, "Distance to Wall", toolMode == TOOLMODE_DISTANCE_TO_WALL))
			{
				toolMode = TOOLMODE_DISTANCE_TO_WALL;
				recalcTool = true;
			}
			if (imguiCheck(GENID, "Raycast", toolMode == TOOLMODE_RAYCAST))
			{
				toolMode = TOOLMODE_RAYCAST;
				recalcTool = true;
			}
			if (imguiCheck(GENID, "Find Polys Around", toolMode == TOOLMODE_FIND_POLYS_AROUND))
			{
				toolMode = TOOLMODE_FIND_POLYS_AROUND;
				recalcTool = true;
			}
			
			imguiEndScrollArea();
		}

		if (g_navMesh && recalcTool)
		{
			recalcTool = false;
			if (toolMode == TOOLMODE_PATHFIND)
			{
				if (!startRef || !endRef)
				{
					npolys = 0;
					nstraightPath = 0;
				}
				else
				{
					npolys = g_navMesh->findPath(startRef, endRef, polys, MAX_POLYS);
					if (npolys)
						nstraightPath = g_navMesh->findStraightPath(spos, epos, polys, npolys, straightPath, MAX_POLYS);
				}
			}
			else if (toolMode == TOOLMODE_RAYCAST)
			{
				nstraightPath = 0;
				if (sposSet && eposSet && startRef)
				{
					float t = 0;
					npolys = 0;
					nstraightPath = 2;
					straightPath[0] = spos[0];
					straightPath[1] = spos[1];
					straightPath[2] = spos[2];
					if (g_navMesh->raycast(startRef, spos, epos, t, polys[0]))
					{
						npolys = 1;
						straightPath[3] = spos[0] + (epos[0] - spos[0]) * t;
						straightPath[4] = spos[1] + (epos[1] - spos[1]) * t;
						straightPath[5] = spos[2] + (epos[2] - spos[2]) * t;
					}
					else
					{
						straightPath[3] = epos[0];
						straightPath[4] = epos[1];
						straightPath[5] = epos[2];
					}
				}
			}
			else if (toolMode == TOOLMODE_DISTANCE_TO_WALL)
			{
				distanceToWall = 0;
				if (sposSet && startRef)
					distanceToWall = g_navMesh->findDistanceToWall(startRef, spos, 100.0f, hitPos, hitNormal);
			}
			else if (toolMode == TOOLMODE_FIND_POLYS_AROUND)
			{
				distanceToWall = 0;
				if (sposSet && startRef && eposSet)
				{
					const float dx = epos[0] - spos[0];
					const float dz = epos[2] - spos[2];
					float dist = sqrtf(dx*dx + dz*dz);
					npolys = g_navMesh->findPolysAround(startRef, spos, dist, polys, 0, 0, 0, MAX_POLYS);
				}
			}
		}
		
		
		// Log
		if (showLog)
		{
			static int logScroll = 0;
			if (imguiBeginScrollArea(GENID, "Log", 10, 10, width - 300, 200, &logScroll))
				mouseOverMenu = true;
			for (int i = 0; i < g_log.getMessageCount(); ++i)
				imguiLabel(GENID1(i), g_log.getMessageText(i));
			imguiEndScrollArea();
		}
		
		// Level selection dialog.
		if (showLevels)
		{
			static int scroll = 0;
			if (imguiBeginScrollArea(GENID, "Choose Level", width-10-250-10-200, height-10-250, 200, 250, &scroll))
				mouseOverMenu = true;
			
			int levelToLoad = -1;
			for (int i = 0; i < fileList.size; ++i)
			{
				if (imguiItem(GENID1(i), fileList.files[i]))
					levelToLoad = i;
			}
			
			if (levelToLoad != -1)
			{
				strncpy(curLevel, fileList.files[levelToLoad], sizeof(curLevel));
				curLevel[sizeof(curLevel)-1] = '\0';
				showLevels = false;
				
				delete g_mesh;
				delete g_chunkyMesh;
				delete g_navMesh;
				delete g_tileSet;
				delete g_polyMesh;
				g_mesh = 0;
				g_chunkyMesh = 0;
				g_navMesh = 0;
				g_tileSet = 0;
				g_polyMesh = 0;

				npolys = 0;
				nstraightPath = 0;
				sposSet = false;
				eposSet = false;
				startRef = 0;
				endRef = 0;
				distanceToWall = 0;
				
				char path[256];
				strcpy(path, "meshes/");
				strcat(path, curLevel);
				
				g_mesh = new rcMeshLoaderObj;
				
				if (!g_mesh || !g_mesh->load(path))
				{
					printf("Could not load mesh\n");
					delete g_mesh;
					g_mesh = 0;
				}
				
				if (g_mesh)
				{
					rcCalcBounds(g_mesh->getVerts(), g_mesh->getVertCount(), g_meshBMin, g_meshBMax);
					
					g_chunkyMesh = new rcChunkyTriMesh;
					
//					rcTimeVal startTime = rcGetPerformanceTimer();
					rcCreateChunkyTriMesh(g_mesh->getVerts(), g_mesh->getTris(), g_mesh->getTriCount(), 256, g_chunkyMesh);
//					rcTimeVal endTime = rcGetPerformanceTimer();					
//					printf("%.3fms\n", rcGetDeltaTimeUsec(startTime, endTime)/1000.0f);

					// Reset camera.
					camr = sqrtf(rcSqr(g_meshBMax[0]-g_meshBMin[0]) +
								 rcSqr(g_meshBMax[1]-g_meshBMin[1]) +
								 rcSqr(g_meshBMax[2]-g_meshBMin[2])) / 2;
					camx = (g_meshBMax[0] + g_meshBMin[0]) / 2 + camr;
					camy = (g_meshBMax[1] + g_meshBMin[1]) / 2 + camr;
					camz = (g_meshBMax[2] + g_meshBMin[2]) / 2 + camr;
					camr *= 3;
					rx = 45;
					ry = -45;
					
					glFogf(GL_FOG_START, camr*0.2f);
					glFogf(GL_FOG_END, camr*1.25f);
				}
				
			}
			
			imguiEndScrollArea();
			
		}

		
		{
			const char msg[] = "W/S/A/D: Move  RMB: Rotate   LMB: Place Start   LMB+SHIFT: Place End";
			const float len = g_font.getTextLength(msg);
			g_font.drawText(width/2-len/2, (float)height-20.0f, msg, GLFont::RGBA(255,255,255,128));
		}
		
		// Draw start and end point labels
		if (sposSet && gluProject((GLdouble)spos[0], (GLdouble)spos[1], (GLdouble)spos[2],
								  model, proj, view, &x, &y, &z))
		{
			const float len = g_font.getTextLength("Start");
			g_font.drawText((float)x - len/2, (float)y-g_font.getLineHeight(), "Start", GLFont::RGBA(0,0,0,220));
		}
		if (eposSet && gluProject((GLdouble)epos[0], (GLdouble)epos[1], (GLdouble)epos[2],
								  model, proj, view, &x, &y, &z))
		{
			const float len = g_font.getTextLength("End");
			g_font.drawText((float)x-len/2, (float)y-g_font.getLineHeight(), "End", GLFont::RGBA(0,0,0,220));
		}
		
		glDisable(GL_TEXTURE_2D);
		
		
		imguiEndFrame();
		imguiRender(&drawText);
		
			
		glEnable(GL_DEPTH_TEST);
		SDL_GL_SwapBuffers();
	}
	
	SDL_Quit();
	
	delete g_mesh;
	delete g_chunkyMesh;
	delete g_navMesh;
	delete g_tileSet;
	delete g_polyMesh;

	
	return 0;
}

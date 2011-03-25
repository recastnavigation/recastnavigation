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

#ifndef DETOURTILECACHEBUILDER_H
#define DETOURTILECACHEBUILDER_H

#include "DetourAlloc.h"
#include "DetourStatus.h"

static const int DT_TILECACHE_MAGIC = 'D'<<24 | 'T'<<16 | 'I'<<8 | 'C'; //'DTIC';
static const int DT_TILECACHE_VERSION = 1;

struct dtTileCacheLayerParams
{
	float bmin[3], bmax[3];				// Bounding box of the heightfield.
	float cs, ch;						// Cell size and height.
};

struct dtTileCacheLayerHeader
{
	int magic;								// Data magic
	int version;							// Data version
	unsigned short hmin, hmax;				// Height min/max range
	unsigned char width, height;			// Dimension of the layer.
	unsigned char minx, maxx, miny, maxy;	// Usable sub-region.
};

struct dtTileCacheLayer
{
	dtTileCacheLayerHeader* header;
	unsigned char regCount;					// Region count.
	unsigned char* heights;
	unsigned char* areas;
	unsigned char* cons;
	unsigned char* regs;
};

struct dtTileCacheContour
{
	int nverts;
	unsigned char* verts;
	unsigned char reg;
	unsigned char area;
};

struct dtTileCacheContourSet
{
	int nconts;
	dtTileCacheContour* conts;
};

struct dtTileCachePolyMesh
{
	int nverts;				// Number of vertices.
	int npolys;				// Number of polygons.
	unsigned short* verts;	// Vertices of the mesh, 3 elements per vertex.
	unsigned short* polys;	// Polygons of the mesh, nvp*2 elements per polygon.
	unsigned short* flags;	// Per polygon flags.
	unsigned char* areas;	// Area ID of polygons.
};


struct dtTileCacheAlloc
{
	virtual void* alloc(const int size)
	{
		return dtAlloc(size, DT_ALLOC_TEMP);
	}
	
	virtual void free(void* ptr)
	{
		dtFree(ptr);
	}
};


dtStatus dtCastTileCacheLayer(dtTileCacheLayer& layer, unsigned char* buffer, const int bufferSize);

// Returns the amount of memory requires for specific grid size.
int dtCalcTileCacheLayerBufferSize(const int gridWidth, const int gridHeight);
									

dtTileCacheContourSet* dtAllocTileCacheContourSet(dtTileCacheAlloc* alloc);
void dtFreeTileCacheContourSet(dtTileCacheAlloc* alloc, dtTileCacheContourSet* cset);

dtTileCachePolyMesh* dtAllocTileCachePolyMesh(dtTileCacheAlloc* alloc);
void dtFreeTileCachePolyMesh(dtTileCacheAlloc* alloc, dtTileCachePolyMesh* lmesh);


dtStatus dtBuildTileCacheRegions(dtTileCacheAlloc* alloc,
								 dtTileCacheLayer& layer,
								 const int walkableClimb);

dtStatus dtBuildTileCacheContours(dtTileCacheAlloc* alloc,
								  dtTileCacheLayer& layer,
								  const int walkableClimb, 	const float maxError,
								  dtTileCacheContourSet& lcset);

dtStatus dtBuildTileCachePolyMesh(dtTileCacheAlloc* alloc,
								  dtTileCacheContourSet& lcset,
								  dtTileCachePolyMesh& mesh);


#endif // DETOURTILECACHEBUILDER_H

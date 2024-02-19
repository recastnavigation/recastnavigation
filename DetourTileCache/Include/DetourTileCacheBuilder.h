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

#pragma once

#include <DetourAlloc.h>
#include <DetourStatus.h>

static constexpr int DT_TILECACHE_MAGIC = 'D'<<24 | 'T'<<16 | 'L'<<8 | 'R'; ///< 'DTLR';
static constexpr int DT_TILECACHE_VERSION = 1;

static constexpr unsigned char DT_TILECACHE_NULL_AREA = 0;
static constexpr unsigned char DT_TILECACHE_WALKABLE_AREA = 63;
static constexpr unsigned short DT_TILECACHE_NULL_IDX = 0xffff;

struct dtTileCacheLayerHeader
{
	int magic;								///< Data magic
	int version;							///< Data version
	int tx,ty,tlayer;
	float bmin[3], bmax[3];
	unsigned short hmin, hmax;				///< Height min/max range
	unsigned char width, height;			///< Dimension of the layer.
	unsigned char minx, maxx, miny, maxy;	///< Usable sub-region.
};

struct dtTileCacheLayer
{
	dtTileCacheLayerHeader* header;
	unsigned char regCount;					///< Region count.
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
	int nvp;
	int nverts;				///< Number of vertices.
	int npolys;				///< Number of polygons.
	unsigned short* verts;	///< Vertices of the mesh, 3 elements per vertex.
	unsigned short* polys;	///< Polygons of the mesh, nvp*2 elements per polygon.
	unsigned short* flags;	///< Per polygon flags.
	unsigned char* areas;	///< Area ID of polygons.
};


struct dtTileCacheAlloc
{
	virtual ~dtTileCacheAlloc()=default;

	virtual void reset() {}
	
	virtual void* alloc(const std::size_t size)
	{
		return dtAlloc(size, DT_ALLOC_TEMP);
	}
	
	virtual void free(void* ptr)
	{
		dtFree(ptr);
	}
};

struct dtTileCacheCompressor
{
	virtual ~dtTileCacheCompressor()=default;

	virtual int maxCompressedSize(int bufferSize) = 0;
	virtual dtStatus compress(const unsigned char* buffer, int bufferSize,
							  unsigned char* compressed, int maxCompressedSize, int* compressedSize) = 0;
	virtual dtStatus decompress(const unsigned char* compressed, int compressedSize,
								unsigned char* buffer, int maxBufferSize, int* bufferSize) = 0;
};


dtStatus dtBuildTileCacheLayer(dtTileCacheCompressor* comp,
                               const dtTileCacheLayerHeader* header,
							   const unsigned char* heights,
							   const unsigned char* areas,
							   const unsigned char* cons,
							   unsigned char** outData, int* outDataSize);

void dtFreeTileCacheLayer(dtTileCacheAlloc* alloc, dtTileCacheLayer* layer);

dtStatus dtDecompressTileCacheLayer(dtTileCacheAlloc* alloc, dtTileCacheCompressor* comp,
									unsigned char* compressed, int compressedSize,
									dtTileCacheLayer** layerOut);

dtTileCacheContourSet* dtAllocTileCacheContourSet(dtTileCacheAlloc* alloc);
void dtFreeTileCacheContourSet(dtTileCacheAlloc* alloc, dtTileCacheContourSet* cset);

dtTileCachePolyMesh* dtAllocTileCachePolyMesh(dtTileCacheAlloc* alloc);
void dtFreeTileCachePolyMesh(dtTileCacheAlloc* alloc, dtTileCachePolyMesh* lmesh);

dtStatus dtMarkCylinderArea(const dtTileCacheLayer& layer, const float* orig, float cs, float ch,
                            const float* pos, float radius, float height, unsigned char areaId);

dtStatus dtMarkBoxArea(const dtTileCacheLayer& layer, const float* orig, float cs, float ch,
                       const float* bmin, const float* bmax, unsigned char areaId);

dtStatus dtMarkBoxArea(const dtTileCacheLayer& layer, const float* orig, float cs, float ch,
                       const float* center, const float* halfExtents, const float* rotAux, unsigned char areaId);

dtStatus dtBuildTileCacheRegions(dtTileCacheAlloc* alloc,
								 dtTileCacheLayer& layer,
								 int walkableClimb);

dtStatus dtBuildTileCacheContours(dtTileCacheAlloc* alloc,
                                  const dtTileCacheLayer& layer,
								  int walkableClimb, float maxError,
								  dtTileCacheContourSet& lcset);

dtStatus dtBuildTileCachePolyMesh(dtTileCacheAlloc* alloc,
                                  const dtTileCacheContourSet& lcset,
								  dtTileCachePolyMesh& mesh);

/// Swaps the endianess of the compressed tile data's header (#dtTileCacheLayerHeader).
/// Tile layer data does not need endian swapping as it consist only of bytes.
///  @param[in,out]	data		The tile data array.
///  @param[in]		dataSize	The size of the data array.
bool dtTileCacheHeaderSwapEndian(unsigned char* data, int dataSize);

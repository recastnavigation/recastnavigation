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

#include <cstddef>
#include <cstdint>

static constexpr int DT_TILECACHE_MAGIC = 'D' << 24 | 'T' << 16 | 'L' << 8 | 'R'; ///< 'DTLR';
static constexpr int DT_TILECACHE_VERSION = 1;

static constexpr uint8_t DT_TILECACHE_NULL_AREA = 0;
static constexpr uint8_t DT_TILECACHE_WALKABLE_AREA = 63;
static constexpr uint16_t DT_TILECACHE_NULL_IDX = 0xffff;

struct dtTileCacheLayerHeader {
  int magic;   ///< Data magic
  int version; ///< Data version
  int tx, ty, tlayer;
  float bmin[3], bmax[3];
  uint16_t hmin, hmax;            ///< Height min/max range
  uint8_t width, height;          ///< Dimension of the layer.
  uint8_t minx, maxx, miny, maxy; ///< Usable sub-region.
};

struct dtTileCacheLayer {
  dtTileCacheLayerHeader *header;
  uint8_t regCount; ///< Region count.
  uint8_t *heights;
  uint8_t *areas;
  uint8_t *cons;
  uint8_t *regs;
};

struct dtTileCacheContour {
  int nverts;
  uint8_t *verts;
  uint8_t reg;
  uint8_t area;
};

struct dtTileCacheContourSet {
  int nconts;
  dtTileCacheContour *conts;
};

struct dtTileCachePolyMesh {
  int nvp;
  int nverts;            ///< Number of vertices.
  int npolys;            ///< Number of polygons.
  uint16_t *verts; ///< Vertices of the mesh, 3 elements per vertex.
  uint16_t *polys; ///< Polygons of the mesh, nvp*2 elements per polygon.
  uint16_t *flags; ///< Per polygon flags.
  uint8_t *areas;  ///< Area ID of polygons.
};

struct dtTileCacheAlloc {
  virtual ~dtTileCacheAlloc();

  virtual void reset() {}

  virtual void *alloc(const std::size_t size) {
    return dtAlloc(size, DT_ALLOC_TEMP);
  }

  virtual void free(void *ptr) {
    dtFree(ptr);
  }
};

struct dtTileCacheCompressor {
  virtual ~dtTileCacheCompressor();

  virtual int maxCompressedSize(int bufferSize) = 0;
  virtual dtStatus compress(const uint8_t *buffer, int bufferSize,
                            uint8_t *compressed, int maxCompressedSize, int *compressedSize) = 0;
  virtual dtStatus decompress(const uint8_t *compressed, int compressedSize,
                              uint8_t *buffer, int maxBufferSize, int *bufferSize) = 0;
};

dtStatus dtBuildTileCacheLayer(dtTileCacheCompressor *comp,
                               const dtTileCacheLayerHeader *header,
                               const uint8_t *heights,
                               const uint8_t *areas,
                               const uint8_t *cons,
                               uint8_t **outData, int *outDataSize);

void dtFreeTileCacheLayer(dtTileCacheAlloc *alloc, dtTileCacheLayer *layer);

dtStatus dtDecompressTileCacheLayer(dtTileCacheAlloc *alloc, dtTileCacheCompressor *comp,
                                    uint8_t *compressed, int compressedSize,
                                    dtTileCacheLayer **layerOut);

dtTileCacheContourSet *dtAllocTileCacheContourSet(dtTileCacheAlloc *alloc);
void dtFreeTileCacheContourSet(dtTileCacheAlloc *alloc, dtTileCacheContourSet *cset);

dtTileCachePolyMesh *dtAllocTileCachePolyMesh(dtTileCacheAlloc *alloc);
void dtFreeTileCachePolyMesh(dtTileCacheAlloc *alloc, dtTileCachePolyMesh *lmesh);

dtStatus dtMarkCylinderArea(const dtTileCacheLayer &layer, const float *orig, float cs, float ch,
                            const float *pos, float radius, float height, uint8_t areaId);

dtStatus dtMarkBoxArea(const dtTileCacheLayer &layer, const float *orig, float cs, float ch,
                       const float *bmin, const float *bmax, uint8_t areaId);

dtStatus dtMarkBoxArea(const dtTileCacheLayer &layer, const float *orig, float cs, float ch,
                       const float *center, const float *halfExtents, const float *rotAux, uint8_t areaId);

dtStatus dtBuildTileCacheRegions(dtTileCacheAlloc *alloc,
                                 dtTileCacheLayer &layer,
                                 int walkableClimb);

dtStatus dtBuildTileCacheContours(dtTileCacheAlloc *alloc,
                                  const dtTileCacheLayer &layer,
                                  int walkableClimb, float maxError,
                                  dtTileCacheContourSet &lcset);

dtStatus dtBuildTileCachePolyMesh(dtTileCacheAlloc *alloc,
                                  const dtTileCacheContourSet &lcset,
                                  dtTileCachePolyMesh &mesh);

/// Swaps the endianess of the compressed tile data's header (#dtTileCacheLayerHeader).
/// Tile layer data does not need endian swapping as it consist only of bytes.
///  @param[in,out]	data		The tile data array.
///  @param[in]		dataSize	The size of the data array.
bool dtTileCacheHeaderSwapEndian(uint8_t *data, int dataSize);

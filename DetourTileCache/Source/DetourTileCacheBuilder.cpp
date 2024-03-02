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

#include "DetourTileCacheBuilder.h"

#include <DetourAssert.h>
#include <DetourCommon.h>

#include <cstring>

dtTileCacheAlloc::~dtTileCacheAlloc() {
  // Defined out of line to fix the weak v-tables warning
}

dtTileCacheCompressor::~dtTileCacheCompressor() {
  // Defined out of line to fix the weak v-tables warning
}

template <class T>
class dtFixedArray {
  dtTileCacheAlloc *m_alloc;
  T *m_ptr;
  const int m_size;
  void operator=(dtFixedArray &p);

public:
  dtFixedArray(dtTileCacheAlloc *a, const int s) : m_alloc(a), m_ptr(static_cast<T *>(a->alloc(sizeof(T) * s))), m_size(s) {}
  ~dtFixedArray() {
    if (m_alloc)
      m_alloc->free(m_ptr);
  }
  operator T *() { return m_ptr; }
  int size() const { return m_size; }
};

inline int getDirOffsetX(const int dir) {
  constexpr int offset[4] = {
      -1,
      0,
      1,
      0,
  };
  return offset[dir & 0x03];
}

inline int getDirOffsetY(const int dir) {
  constexpr int offset[4] = {0, 1, 0, -1};
  return offset[dir & 0x03];
}

static constexpr int MAX_VERTS_PER_POLY = 6; // TODO: use the DT_VERTS_PER_POLYGON
static constexpr int MAX_REM_EDGES = 48;     // TODO: make this an expression.

dtTileCacheContourSet *dtAllocTileCacheContourSet(dtTileCacheAlloc *alloc) {
  dtAssert(alloc);

  auto *const cset = static_cast<dtTileCacheContourSet *>(alloc->alloc(sizeof(dtTileCacheContourSet)));
  std::memset(cset, 0, sizeof(dtTileCacheContourSet));
  return cset;
}

void dtFreeTileCacheContourSet(dtTileCacheAlloc *alloc, dtTileCacheContourSet *cset) {
  dtAssert(alloc);

  if (!cset || !alloc)
    return;
  for (int i = 0; i < cset->nconts; ++i)
    alloc->free(cset->conts[i].verts);
  alloc->free(cset->conts);
  alloc->free(cset);
}

dtTileCachePolyMesh *dtAllocTileCachePolyMesh(dtTileCacheAlloc *alloc) {
  dtAssert(alloc);

  auto *const lmesh = static_cast<dtTileCachePolyMesh *>(alloc->alloc(sizeof(dtTileCachePolyMesh)));
  std::memset(lmesh, 0, sizeof(dtTileCachePolyMesh));
  return lmesh;
}

void dtFreeTileCachePolyMesh(dtTileCacheAlloc *alloc, dtTileCachePolyMesh *lmesh) {
  dtAssert(alloc);

  if (!lmesh || !alloc)
    return;
  alloc->free(lmesh->verts);
  alloc->free(lmesh->polys);
  alloc->free(lmesh->flags);
  alloc->free(lmesh->areas);
  alloc->free(lmesh);
}

struct dtLayerSweepSpan {
  uint16_t ns; // number samples
  uint8_t id;  // region id
  uint8_t nei; // neighbour id
};
namespace {

constexpr int DT_LAYER_MAX_NEIS = 16;

struct dtLayerMonotoneRegion {
  int area;
  uint8_t neis[DT_LAYER_MAX_NEIS];
  uint8_t nneis;
  uint8_t regId;
  uint8_t areaId;
};

struct dtTempContour {
  dtTempContour(uint8_t *vbuf, const int nvbuf,
                uint16_t *pbuf, const int npbuf) : verts(vbuf), nverts(0), cverts(nvbuf),
                                                   poly(pbuf), npoly(0), cpoly(npbuf) {
  }
  uint8_t *verts;
  int nverts;
  int cverts;
  uint16_t *poly;
  int npoly;
  int cpoly;
};

bool overlapRangeExl(const uint16_t amin, const uint16_t amax,
                     const uint16_t bmin, const uint16_t bmax) {
  return amin < bmax && amax > bmin;
}

void addUniqueLast(uint8_t *a, uint8_t &an, const uint8_t v) {
  if (an > 0 && a[an - 1] == v)
    return;
  a[an] = v;
  an++;
}

bool isConnected(const dtTileCacheLayer &layer,
                 const int ia, const int ib, const int walkableClimb) {
  if (layer.areas[ia] != layer.areas[ib])
    return false;
  if (dtAbs(static_cast<int>(layer.heights[ia]) - static_cast<int>(layer.heights[ib])) > walkableClimb)
    return false;
  return true;
}

bool canMerge(const uint8_t oldRegId, const uint8_t newRegId, const dtLayerMonotoneRegion *regs, const int nregs) {
  if (!regs)
    return false;

  int count = 0;
  for (int i = 0; i < nregs; ++i) {
    const dtLayerMonotoneRegion &reg = regs[i];
    if (reg.regId != oldRegId)
      continue;
    const int nnei = reg.nneis;
    for (int j = 0; j < nnei; ++j) {
      if (regs[reg.neis[j]].regId == newRegId)
        count++;
    }
  }
  return count == 1;
}

bool appendVertex(dtTempContour &cont, const int x, const int y, const int z, const int r) {
  // Try to merge with existing segments.
  if (cont.nverts > 1) {
    const uint8_t *pa = &cont.verts[(cont.nverts - 2) * 4];
    uint8_t *pb = &cont.verts[(cont.nverts - 1) * 4];
    if (static_cast<int>(pb[3]) == r) {
      if (pa[0] == pb[0] && static_cast<int>(pb[0]) == x) {
        // The verts are aligned aling x-axis, update z.
        pb[1] = static_cast<uint8_t>(y);
        pb[2] = static_cast<uint8_t>(z);
        return true;
      }
      if (pa[2] == pb[2] && static_cast<int>(pb[2]) == z) {
        // The verts are aligned aling z-axis, update x.
        pb[0] = static_cast<uint8_t>(x);
        pb[1] = static_cast<uint8_t>(y);
        return true;
      }
    }
  }

  // Add new point.
  if (cont.nverts + 1 > cont.cverts)
    return false;

  uint8_t *v = &cont.verts[cont.nverts * 4];
  v[0] = static_cast<uint8_t>(x);
  v[1] = static_cast<uint8_t>(y);
  v[2] = static_cast<uint8_t>(z);
  v[3] = static_cast<uint8_t>(r);
  cont.nverts++;

  return true;
}

uint8_t getNeighbourReg(const dtTileCacheLayer &layer,
                        const int ax, const int ay, const int dir) {
  const int w = layer.header->width;
  const int ia = ax + ay * w;

  const uint8_t con = layer.cons[ia] & 0xf;
  const uint8_t portal = layer.cons[ia] >> 4;
  const uint8_t mask = static_cast<uint8_t>(1 << dir);

  if ((con & mask) == 0) {
    // No connection, return portal or hard edge.
    if (portal & mask)
      return 0xf8 + static_cast<uint8_t>(dir);
    return 0xff;
  }

  const int bx = ax + getDirOffsetX(dir);
  const int by = ay + getDirOffsetY(dir);
  const int ib = bx + by * w;

  return layer.regs[ib];
}

bool walkContour(const dtTileCacheLayer &layer, int x, int y, dtTempContour &cont) {
  const int w = layer.header->width;
  const int h = layer.header->height;

  cont.nverts = 0;

  const int startX = x;
  const int startY = y;
  int startDir = -1;

  for (int i = 0; i < 4; ++i) {
    const int dir = i + 3 & 3;
    if (getNeighbourReg(layer, x, y, dir) != layer.regs[x + y * w]) {
      startDir = dir;
      break;
    }
  }
  if (startDir == -1)
    return true;

  int dir = startDir;
  const int maxIter = w * h;

  int iter = 0;
  while (iter < maxIter) {
    const uint8_t rn = getNeighbourReg(layer, x, y, dir);

    int nx = x;
    int ny = y;
    int ndir;

    if (rn != layer.regs[x + y * w]) {
      // Solid edge.
      int px = x;
      int pz = y;
      switch (dir) {
      case 0:
        pz++;
        break;
      case 1:
        px++;
        pz++;
        break;
      case 2:
        px++;
        break;
      default:
        break;
      }

      // Try to merge with previous vertex.
      if (!appendVertex(cont, px, layer.heights[x + y * w], pz, rn))
        return false;

      ndir = dir + 1 & 0x3; // Rotate CW
    } else {
      // Move to next.
      nx = x + getDirOffsetX(dir);
      ny = y + getDirOffsetY(dir);
      ndir = dir + 3 & 0x3; // Rotate CCW
    }

    if (iter > 0 && x == startX && y == startY && dir == startDir)
      break;

    x = nx;
    y = ny;
    dir = ndir;

    iter++;
  }

  // Remove last vertex if it is duplicate of the first one.
  const uint8_t *pa = &cont.verts[(cont.nverts - 1) * 4];
  const uint8_t *pb = &cont.verts[0];
  if (pa[0] == pb[0] && pa[2] == pb[2])
    cont.nverts--;

  return true;
}

float distancePtSeg(const int x, const int z,
                    const int px, const int pz,
                    const int qx, const int qz) {
  const float pqx = static_cast<float>(qx - px);
  const float pqz = static_cast<float>(qz - pz);
  float dx = static_cast<float>(x - px);
  float dz = static_cast<float>(z - pz);
  const float d = pqx * pqx + pqz * pqz;
  float t = pqx * dx + pqz * dz;
  if (d > 0)
    t /= d;
  if (t < 0)
    t = 0;
  else if (t > 1)
    t = 1;

  dx = px + t * pqx - x;
  dz = pz + t * pqz - z;

  return dx * dx + dz * dz;
}

void simplifyContour(dtTempContour &cont, const float maxError) {
  if (!cont.verts || !cont.poly)
    return;
  cont.npoly = 0;

  for (int i = 0; i < cont.nverts; ++i) {
    const int j = (i + 1) % cont.nverts;
    // Check for start of a wall segment.
    if (cont.verts[i * 4 + 3] != cont.verts[j * 4 + 3])
      cont.poly[cont.npoly++] = static_cast<uint16_t>(i);
  }
  if (cont.npoly < 2) {
    // If there is no transitions at all,
    // create some initial points for the simplification process.
    // Find lower-left and upper-right vertices of the contour.
    int llx = cont.verts[0];
    int llz = cont.verts[2];
    int lli = 0;
    int urx = cont.verts[0];
    int urz = cont.verts[2];
    int uri = 0;
    for (int i = 1; i < cont.nverts; ++i) {
      const int x = cont.verts[i * 4 + 0];
      const int z = cont.verts[i * 4 + 2];
      if (x < llx || (x == llx && z < llz)) {
        llx = x;
        llz = z;
        lli = i;
      }
      if (x > urx || (x == urx && z > urz)) {
        urx = x;
        urz = z;
        uri = i;
      }
    }
    cont.npoly = 0;
    cont.poly[cont.npoly++] = static_cast<uint16_t>(lli);
    cont.poly[cont.npoly++] = static_cast<uint16_t>(uri);
  }

  // Add points until all raw points are within
  // error tolerance to the simplified shape.
  for (int i = 0; i < cont.npoly;) {
    const int ii = (i + 1) % cont.npoly;

    const int ai = cont.poly[i];
    const int ax = cont.verts[ai * 4 + 0];
    const int az = cont.verts[ai * 4 + 2];

    const int bi = cont.poly[ii];
    const int bx = cont.verts[bi * 4 + 0];
    const int bz = cont.verts[bi * 4 + 2];

    // Find maximum deviation from the segment.
    float maxd = 0;
    int maxi = -1;
    int ci, cinc, endi;

    // Traverse the segment in lexilogical order so that the
    // max deviation is calculated similarly when traversing
    // opposite segments.
    if (bx > ax || (bx == ax && bz > az)) {
      cinc = 1;
      ci = (ai + cinc) % cont.nverts;
      endi = bi;
    } else {
      cinc = cont.nverts - 1;
      ci = (bi + cinc) % cont.nverts;
      endi = ai;
    }

    // Tessellate only outer edges or edges between areas.
    while (ci != endi) {
      const float d = distancePtSeg(cont.verts[ci * 4 + 0], cont.verts[ci * 4 + 2], ax, az, bx, bz);
      if (d > maxd) {
        maxd = d;
        maxi = ci;
      }
      ci = (ci + cinc) % cont.nverts;
    }

    // If the max deviation is larger than accepted error,
    // add new point, else continue to next segment.
    if (maxi != -1 && maxd > maxError * maxError) {
      cont.npoly++;
      for (int j = cont.npoly - 1; j > i; --j)
        cont.poly[j] = cont.poly[j - 1];
      cont.poly[i + 1] = static_cast<uint16_t>(maxi);
    } else {
      ++i;
    }
  }

  // Remap vertices
  int start = 0;
  for (int i = 1; i < cont.npoly; ++i)
    if (cont.poly[i] < cont.poly[start])
      start = i;

  cont.nverts = 0;
  for (int i = 0; i < cont.npoly; ++i) {
    const int j = (start + i) % cont.npoly;
    const uint8_t *src = &cont.verts[cont.poly[j] * 4];
    uint8_t *dst = &cont.verts[cont.nverts * 4];
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    cont.nverts++;
  }
}

uint8_t getCornerHeight(const dtTileCacheLayer &layer,
                        const int x, const int y, const int z,
                        const int walkableClimb,
                        bool &shouldRemove) {
  const int w = layer.header->width;
  const int h = layer.header->height;

  int n = 0;

  uint8_t portal = 0xf;
  uint8_t height = 0;
  uint8_t preg = 0xff;
  bool allSameReg = true;

  for (int dz = -1; dz <= 0; ++dz) {
    for (int dx = -1; dx <= 0; ++dx) {
      const int px = x + dx;
      const int pz = z + dz;
      if (px >= 0 && pz >= 0 && px < w && pz < h) {
        const int idx = px + pz * w;
        const int lh = layer.heights[idx];
        if (dtAbs(lh - y) <= walkableClimb && layer.areas[idx] != DT_TILECACHE_NULL_AREA) {
          height = dtMax(height, static_cast<uint8_t>(lh));
          portal &= layer.cons[idx] >> 4;
          if (preg != 0xff && preg != layer.regs[idx])
            allSameReg = false;
          preg = layer.regs[idx];
          n++;
        }
      }
    }
  }

  int portalCount = 0;
  for (int dir = 0; dir < 4; ++dir)
    if (portal & 1 << dir)
      portalCount++;

  shouldRemove = false;
  if (n > 1 && portalCount == 1 && allSameReg) {
    shouldRemove = true;
  }

  return height;
}

constexpr int VERTEX_BUCKET_COUNT2 = 1 << 8;

int computeVertexHash2(const int x, const int y, const int z) {
  constexpr uint32_t h1 = 0x8da6b343; // Large multiplicative constants;
  constexpr uint32_t h2 = 0xd8163841; // here arbitrarily chosen primes
  constexpr uint32_t h3 = 0xcb1ab31f;
  const uint32_t n = h1 * x + h2 * y + h3 * z;
  return static_cast<int>(n & VERTEX_BUCKET_COUNT2 - 1);
}

uint16_t addVertex(const uint16_t x, const uint16_t y, const uint16_t z, uint16_t *verts, uint16_t *firstVert, uint16_t *nextVert, int &nv) {
  const int bucket = computeVertexHash2(x, 0, z);
  uint16_t i = firstVert[bucket];

  while (i != DT_TILECACHE_NULL_IDX) {
    const uint16_t *v = &verts[i * 3];
    if (v[0] == x && v[2] == z && dtAbs(v[1] - y) <= 2)
      return i;
    i = nextVert[i]; // next
  }

  // Could not find, create new.
  i = static_cast<uint16_t>(nv);
  nv++;
  uint16_t *v = &verts[i * 3];
  v[0] = x;
  v[1] = y;
  v[2] = z;
  nextVert[i] = firstVert[bucket];
  firstVert[bucket] = i;

  return i;
}

struct rcEdge {
  uint16_t vert[2];
  uint16_t polyEdge[2];
  uint16_t poly[2];
};

bool buildMeshAdjacency(dtTileCacheAlloc *alloc,
                        uint16_t *polys, const int npolys,
                        const uint16_t *verts, const int nverts,
                        const dtTileCacheContourSet &lcset) {
  // Based on code by Eric Lengyel from:
  // https://web.archive.org/web/20080704083314/http://www.terathon.com/code/edges.php

  const int maxEdgeCount = npolys * MAX_VERTS_PER_POLY;
  dtFixedArray<uint16_t> firstEdge(alloc, nverts + maxEdgeCount);
  if (!firstEdge)
    return false;
  uint16_t *nextEdge = firstEdge + nverts;
  int edgeCount = 0;

  dtFixedArray<rcEdge> edges(alloc, maxEdgeCount);
  if (!edges)
    return false;

  for (int i = 0; i < nverts; i++)
    firstEdge[i] = DT_TILECACHE_NULL_IDX;

  for (int i = 0; i < npolys; ++i) {
    const uint16_t *t = &polys[i * MAX_VERTS_PER_POLY * 2];
    for (int j = 0; j < MAX_VERTS_PER_POLY; ++j) {
      if (t[j] == DT_TILECACHE_NULL_IDX)
        break;
      const uint16_t v0 = t[j];
      const uint16_t v1 = j + 1 >= MAX_VERTS_PER_POLY || t[j + 1] == DT_TILECACHE_NULL_IDX ? t[0] : t[j + 1];
      if (v0 < v1) {
        rcEdge &edge = edges[edgeCount];
        edge.vert[0] = v0;
        edge.vert[1] = v1;
        edge.poly[0] = static_cast<uint16_t>(i);
        edge.polyEdge[0] = static_cast<uint16_t>(j);
        edge.poly[1] = static_cast<uint16_t>(i);
        edge.polyEdge[1] = 0xff;
        // Insert edge
        nextEdge[edgeCount] = firstEdge[v0];
        firstEdge[v0] = static_cast<uint16_t>(edgeCount);
        edgeCount++;
      }
    }
  }

  for (int i = 0; i < npolys; ++i) {
    const uint16_t *t = &polys[i * MAX_VERTS_PER_POLY * 2];
    for (int j = 0; j < MAX_VERTS_PER_POLY; ++j) {
      if (t[j] == DT_TILECACHE_NULL_IDX)
        break;
      const uint16_t v0 = t[j];
      const uint16_t v1 = j + 1 >= MAX_VERTS_PER_POLY || t[j + 1] == DT_TILECACHE_NULL_IDX ? t[0] : t[j + 1];
      if (v0 > v1) {
        bool found = false;
        for (uint16_t e = firstEdge[v1]; e != DT_TILECACHE_NULL_IDX; e = nextEdge[e]) {
          rcEdge &edge = edges[e];
          if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1]) {
            edge.poly[1] = static_cast<uint16_t>(i);
            edge.polyEdge[1] = static_cast<uint16_t>(j);
            found = true;
            break;
          }
        }
        if (!found) {
          // Matching edge not found, it is an open edge, add it.
          rcEdge &edge = edges[edgeCount];
          edge.vert[0] = v1;
          edge.vert[1] = v0;
          edge.poly[0] = static_cast<uint16_t>(i);
          edge.polyEdge[0] = static_cast<uint16_t>(j);
          edge.poly[1] = static_cast<uint16_t>(i);
          edge.polyEdge[1] = 0xff;
          // Insert edge
          nextEdge[edgeCount] = firstEdge[v1];
          firstEdge[v1] = static_cast<uint16_t>(edgeCount);
          edgeCount++;
        }
      }
    }
  }

  // Mark portal edges.
  for (int i = 0; i < lcset.nconts; ++i) {
    const dtTileCacheContour &cont = lcset.conts[i];
    if (cont.nverts < 3)
      continue;

    for (int j = 0, k = cont.nverts - 1; j < cont.nverts; k = j++) {
      const uint8_t *va = &cont.verts[k * 4];
      const uint8_t *vb = &cont.verts[j * 4];
      const uint8_t dir = va[3] & 0xf;
      if (dir == 0xf)
        continue;

      if (dir == 0 || dir == 2) {
        // Find matching vertical edge
        const uint16_t x = va[0];
        uint16_t zmin = va[2];
        uint16_t zmax = vb[2];
        if (zmin > zmax)
          dtSwap(zmin, zmax);

        for (int m = 0; m < edgeCount; ++m) {
          rcEdge &e = edges[m];
          // Skip connected edges.
          if (e.poly[0] != e.poly[1])
            continue;
          const uint16_t *eva = &verts[e.vert[0] * 3];
          const uint16_t *evb = &verts[e.vert[1] * 3];
          if (eva[0] == x && evb[0] == x) {
            uint16_t ezmin = eva[2];
            uint16_t ezmax = evb[2];
            if (ezmin > ezmax)
              dtSwap(ezmin, ezmax);
            if (overlapRangeExl(zmin, zmax, ezmin, ezmax)) {
              // Reuse the other polyedge to store dir.
              e.polyEdge[1] = dir;
            }
          }
        }
      } else {
        // Find matching vertical edge
        const uint16_t z = va[2];
        uint16_t xmin = va[0];
        uint16_t xmax = vb[0];
        if (xmin > xmax)
          dtSwap(xmin, xmax);
        for (int m = 0; m < edgeCount; ++m) {
          rcEdge &e = edges[m];
          // Skip connected edges.
          if (e.poly[0] != e.poly[1])
            continue;
          const uint16_t *eva = &verts[e.vert[0] * 3];
          const uint16_t *evb = &verts[e.vert[1] * 3];
          if (eva[2] == z && evb[2] == z) {
            uint16_t exmin = eva[0];
            uint16_t exmax = evb[0];
            if (exmin > exmax)
              dtSwap(exmin, exmax);
            if (overlapRangeExl(xmin, xmax, exmin, exmax)) {
              // Reuse the other polyedge to store dir.
              e.polyEdge[1] = dir;
            }
          }
        }
      }
    }
  }

  // Store adjacency
  for (int i = 0; i < edgeCount; ++i) {
    const rcEdge &e = edges[i];
    if (e.poly[0] != e.poly[1]) {
      uint16_t *p0 = &polys[e.poly[0] * MAX_VERTS_PER_POLY * 2];
      uint16_t *p1 = &polys[e.poly[1] * MAX_VERTS_PER_POLY * 2];
      p0[MAX_VERTS_PER_POLY + e.polyEdge[0]] = e.poly[1];
      p1[MAX_VERTS_PER_POLY + e.polyEdge[1]] = e.poly[0];
    } else if (e.polyEdge[1] != 0xff) {
      uint16_t *p0 = &polys[e.poly[0] * MAX_VERTS_PER_POLY * 2];
      p0[MAX_VERTS_PER_POLY + e.polyEdge[0]] = 0x8000 | static_cast<uint16_t>(e.polyEdge[1]);
    }
  }

  return true;
}

// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
int prev(const int i, const int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
int next(const int i, const int n) { return i + 1 < n ? i + 1 : 0; }

int area2(const uint8_t *a, const uint8_t *b, const uint8_t *c) {
  return (static_cast<int>(b[0]) - static_cast<int>(a[0])) * (static_cast<int>(c[2]) - static_cast<int>(a[2])) - (static_cast<int>(c[0]) - static_cast<int>(a[0])) * (static_cast<int>(b[2]) - static_cast<int>(a[2]));
}

//	Exclusive or: true iff exactly one argument is true.
//	The arguments are negated to ensure that they are 0/1
//	values.  Then the bitwise Xor operator may apply.
//	(This idea is due to Michael Baldwin.)
bool xorb(const bool x, const bool y) {
  return !x ^ !y;
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
bool left(const uint8_t *a, const uint8_t *b, const uint8_t *c) {
  return area2(a, b, c) < 0;
}

bool leftOn(const uint8_t *a, const uint8_t *b, const uint8_t *c) {
  return area2(a, b, c) <= 0;
}

bool collinear(const uint8_t *a, const uint8_t *b, const uint8_t *c) {
  return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
bool intersectProp(const uint8_t *a, const uint8_t *b,
                   const uint8_t *c, const uint8_t *d) {
  // Eliminate improper cases.
  if (collinear(a, b, c) || collinear(a, b, d) ||
      collinear(c, d, a) || collinear(c, d, b))
    return false;

  return xorb(left(a, b, c), left(a, b, d)) && xorb(left(c, d, a), left(c, d, b));
}

// Returns T iff (a,b,c) are collinear and point c lies
// on the closed segement ab.
bool between(const uint8_t *a, const uint8_t *b, const uint8_t *c) {
  if (!collinear(a, b, c))
    return false;
  // If ab not vertical, check betweenness on x; else on y.
  if (a[0] != b[0])
    return (a[0] <= c[0] && c[0] <= b[0]) || (a[0] >= c[0] && c[0] >= b[0]);
  return (a[2] <= c[2] && c[2] <= b[2]) || (a[2] >= c[2] && c[2] >= b[2]);
}

// Returns true iff segments ab and cd intersect, properly or improperly.
bool intersect(const uint8_t *a, const uint8_t *b,
               const uint8_t *c, const uint8_t *d) {
  if (intersectProp(a, b, c, d))
    return true;
  if (between(a, b, c) || between(a, b, d) ||
      between(c, d, a) || between(c, d, b))
    return true;
  return false;
}

bool vequal(const uint8_t *a, const uint8_t *b) {
  return a[0] == b[0] && a[2] == b[2];
}

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
bool diagonalie(const int i, const int j, const int n, const uint8_t *verts, const uint16_t *indices) {
  const uint8_t *d0 = &verts[(indices[i] & 0x7fff) * 4];
  const uint8_t *d1 = &verts[(indices[j] & 0x7fff) * 4];

  // For each edge (k,k+1) of P
  for (int k = 0; k < n; k++) {
    const int k1 = next(k, n);
    // Skip edges incident to i or j
    if (!(k == i || k1 == i || k == j || k1 == j)) {
      const uint8_t *p0 = &verts[(indices[k] & 0x7fff) * 4];
      const uint8_t *p1 = &verts[(indices[k1] & 0x7fff) * 4];

      if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
        continue;

      if (intersect(d0, d1, p0, p1))
        return false;
    }
  }
  return true;
}

// Returns true iff the diagonal (i,j) is strictly internal to the
// polygon P in the neighborhood of the i endpoint.
bool inCone(const int i, const int j, const int n, const uint8_t *verts, const uint16_t *indices) {
  const uint8_t *pi = &verts[(indices[i] & 0x7fff) * 4];
  const uint8_t *pj = &verts[(indices[j] & 0x7fff) * 4];
  const uint8_t *pi1 = &verts[(indices[next(i, n)] & 0x7fff) * 4];
  const uint8_t *pin1 = &verts[(indices[prev(i, n)] & 0x7fff) * 4];

  // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
  if (leftOn(pin1, pi, pi1))
    return left(pi, pj, pin1) && left(pj, pi, pi1);
  // Assume (i-1,i,i+1) not collinear.
  // else P[i] is reflex.
  return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

// Returns T iff (v_i, v_j) is a proper internal
// diagonal of P.
bool diagonal(const int i, const int j, const int n, const uint8_t *verts, const uint16_t *indices) {
  return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
}

int triangulate(int n, const uint8_t *verts, uint16_t *indices, uint16_t *tris) {
  int ntris = 0;
  uint16_t *dst = tris;

  // The last bit of the index is used to indicate if the vertex can be removed.
  for (int i = 0; i < n; i++) {
    const int i1 = next(i, n);
    const int i2 = next(i1, n);
    if (diagonal(i, i2, n, verts, indices))
      indices[i1] |= 0x8000;
  }

  while (n > 3) {
    int minLen = -1;
    int mini = -1;
    for (int i = 0; i < n; i++) {
      const int i1 = next(i, n);
      if (indices[i1] & 0x8000) {
        const uint8_t *p0 = &verts[(indices[i] & 0x7fff) * 4];
        const uint8_t *p2 = &verts[(indices[next(i1, n)] & 0x7fff) * 4];

        const int dx = static_cast<int>(p2[0]) - static_cast<int>(p0[0]);
        const int dz = static_cast<int>(p2[2]) - static_cast<int>(p0[2]);
        const int len = dx * dx + dz * dz;
        if (minLen < 0 || len < minLen) {
          minLen = len;
          mini = i;
        }
      }
    }

    if (mini == -1) {
      // Should not happen.
      /*			printf("mini == -1 ntris=%d n=%d\n", ntris, n);
       for (int i = 0; i < n; i++)
       {
       printf("%d ", indices[i] & 0x0fffffff);
       }
       printf("\n");*/
      return -ntris;
    }

    int i = mini;
    int i1 = next(i, n);
    const int i2 = next(i1, n);

    *dst++ = indices[i] & 0x7fff;
    *dst++ = indices[i1] & 0x7fff;
    *dst++ = indices[i2] & 0x7fff;
    ntris++;

    // Removes P[i1] by copying P[i+1]...P[n-1] left one index.
    n--;
    for (int k = i1; k < n; k++)
      indices[k] = indices[k + 1];

    if (i1 >= n)
      i1 = 0;
    i = prev(i1, n);
    // Update diagonal flags.
    if (diagonal(prev(i, n), i1, n, verts, indices))
      indices[i] |= 0x8000;
    else
      indices[i] &= 0x7fff;

    if (diagonal(i, next(i1, n), n, verts, indices))
      indices[i1] |= 0x8000;
    else
      indices[i1] &= 0x7fff;
  }

  // Append the remaining triangle.
  *dst++ = indices[0] & 0x7fff;
  *dst++ = indices[1] & 0x7fff;
  *dst = indices[2] & 0x7fff;
  ntris++;

  return ntris;
}

int countPolyVerts(const uint16_t *p) {
  for (int i = 0; i < MAX_VERTS_PER_POLY; ++i)
    if (p[i] == DT_TILECACHE_NULL_IDX)
      return i;
  return MAX_VERTS_PER_POLY;
}

bool uleft(const uint16_t *a, const uint16_t *b, const uint16_t *c) {
  return (static_cast<int>(b[0]) - static_cast<int>(a[0])) * (static_cast<int>(c[2]) - static_cast<int>(a[2])) -
             (static_cast<int>(c[0]) - static_cast<int>(a[0])) * (static_cast<int>(b[2]) - static_cast<int>(a[2])) <
         0;
}

int getPolyMergeValue(const uint16_t *pa, const uint16_t *pb,
                      const uint16_t *verts, int &ea, int &eb) {
  const int na = countPolyVerts(pa);
  const int nb = countPolyVerts(pb);

  // If the merged polygon would be too big, do not merge.
  if (na + nb - 2 > MAX_VERTS_PER_POLY)
    return -1;

  // Check if the polygons share an edge.
  ea = -1;
  eb = -1;

  for (int i = 0; i < na; ++i) {
    uint16_t va0 = pa[i];
    uint16_t va1 = pa[(i + 1) % na];
    if (va0 > va1)
      dtSwap(va0, va1);
    for (int j = 0; j < nb; ++j) {
      uint16_t vb0 = pb[j];
      uint16_t vb1 = pb[(j + 1) % nb];
      if (vb0 > vb1)
        dtSwap(vb0, vb1);
      if (va0 == vb0 && va1 == vb1) {
        ea = i;
        eb = j;
        break;
      }
    }
  }

  // No common edge, cannot merge.
  if (ea == -1 || eb == -1)
    return -1;

  // Check to see if the merged polygon would be convex.

  uint16_t va = pa[(ea + na - 1) % na];
  uint16_t vb = pa[ea];
  uint16_t vc = pb[(eb + 2) % nb];
  if (!uleft(&verts[va * 3], &verts[vb * 3], &verts[vc * 3]))
    return -1;

  va = pb[(eb + nb - 1) % nb];
  vb = pb[eb];
  vc = pa[(ea + 2) % na];
  if (!uleft(&verts[va * 3], &verts[vb * 3], &verts[vc * 3]))
    return -1;

  va = pa[ea];
  vb = pa[(ea + 1) % na];

  const int dx = static_cast<int>(verts[va * 3 + 0]) - static_cast<int>(verts[vb * 3 + 0]);
  const int dy = static_cast<int>(verts[va * 3 + 2]) - static_cast<int>(verts[vb * 3 + 2]);

  return dx * dx + dy * dy;
}

void mergePolys(uint16_t *pa, const uint16_t *pb, const int ea, const int eb) {
  uint16_t tmp[MAX_VERTS_PER_POLY * 2];

  const int na = countPolyVerts(pa);
  const int nb = countPolyVerts(pb);

  // Merge polygons.
  std::memset(tmp, 0xff, sizeof(uint16_t) * MAX_VERTS_PER_POLY * 2);
  int n = 0;
  // Add pa
  for (int i = 0; i < na - 1; ++i)
    tmp[n++] = pa[(ea + 1 + i) % na];
  // Add pb
  for (int i = 0; i < nb - 1; ++i)
    tmp[n++] = pb[(eb + 1 + i) % nb];

  std::memcpy(pa, tmp, sizeof(uint16_t) * MAX_VERTS_PER_POLY);
}

void pushFront(const uint16_t v, uint16_t *arr, int &an) {
  an++;
  for (int i = an - 1; i > 0; --i)
    arr[i] = arr[i - 1];
  arr[0] = v;
}

void pushBack(const uint16_t v, uint16_t *arr, int &an) {
  arr[an] = v;
  an++;
}

bool canRemoveVertex(const dtTileCachePolyMesh &mesh, const uint16_t rem) {
  // Count number of polygons to remove.
  int numTouchedVerts = 0;
  int numRemainingEdges = 0;
  for (int i = 0; i < mesh.npolys; ++i) {
    const uint16_t *p = &mesh.polys[i * MAX_VERTS_PER_POLY * 2];
    const int nv = countPolyVerts(p);
    int numRemoved = 0;
    int numVerts = 0;
    for (int j = 0; j < nv; ++j) {
      if (p[j] == rem) {
        numTouchedVerts++;
        numRemoved++;
      }
      numVerts++;
    }
    if (numRemoved) {
      numRemainingEdges += numVerts - (numRemoved + 1);
    }
  }

  // There would be too few edges remaining to create a polygon.
  // This can happen for example when a tip of a triangle is marked
  // as deletion, but there are no other polys that share the vertex.
  // In this case, the vertex should not be removed.
  if (numRemainingEdges <= 2)
    return false;

  // Check that there is enough memory for the test.
  const int maxEdges = numTouchedVerts * 2;
  if (maxEdges > MAX_REM_EDGES)
    return false;

  // Find edges which share the removed vertex.
  uint16_t edges[MAX_REM_EDGES];
  int nedges = 0;

  for (int i = 0; i < mesh.npolys; ++i) {
    const uint16_t *p = &mesh.polys[i * MAX_VERTS_PER_POLY * 2];
    const int nv = countPolyVerts(p);

    // Collect edges which touches the removed vertex.
    for (int j = 0, k = nv - 1; j < nv; k = j++) {
      if (p[j] == rem || p[k] == rem) {
        // Arrange edge so that a=rem.
        int a = p[j], b = p[k];
        if (b == rem)
          dtSwap(a, b);

        // Check if the edge exists
        bool exists = false;
        for (int m = 0; m < nedges; ++m) {
          uint16_t *e = &edges[m * 3];
          if (e[1] == b) {
            // Exists, increment vertex share count.
            e[2]++;
            exists = true;
          }
        }
        // Add new edge.
        if (!exists) {
          uint16_t *e = &edges[nedges * 3];
          e[0] = static_cast<uint16_t>(a);
          e[1] = static_cast<uint16_t>(b);
          e[2] = 1;
          nedges++;
        }
      }
    }
  }

  // There should be no more than 2 open edges.
  // This catches the case that two non-adjacent polygons
  // share the removed vertex. In that case, do not remove the vertex.
  int numOpenEdges = 0;
  for (int i = 0; i < nedges; ++i) {
    if (edges[i * 3 + 2] < 2)
      numOpenEdges++;
  }
  if (numOpenEdges > 2)
    return false;

  return true;
}

dtStatus removeVertex(dtTileCachePolyMesh &mesh, const uint16_t rem, const int maxTris) {
  // Count number of polygons to remove.
  int numRemovedVerts = 0;
  for (int i = 0; i < mesh.npolys; ++i) {
    uint16_t *p = &mesh.polys[i * MAX_VERTS_PER_POLY * 2];
    const int nv = countPolyVerts(p);
    for (int j = 0; j < nv; ++j) {
      if (p[j] == rem)
        numRemovedVerts++;
    }
  }

  int nedges = 0;
  uint16_t edges[MAX_REM_EDGES * 3];
  int nhole = 0;
  uint16_t hole[MAX_REM_EDGES];
  int nharea = 0;
  uint16_t harea[MAX_REM_EDGES];

  for (int i = 0; i < mesh.npolys; ++i) {
    uint16_t *p = &mesh.polys[i * MAX_VERTS_PER_POLY * 2];
    const int nv = countPolyVerts(p);
    bool hasRem = false;
    for (int j = 0; j < nv; ++j)
      if (p[j] == rem)
        hasRem = true;
    if (hasRem) {
      // Collect edges which does not touch the removed vertex.
      for (int j = 0, k = nv - 1; j < nv; k = j++) {
        if (p[j] != rem && p[k] != rem) {
          if (nedges >= MAX_REM_EDGES)
            return DT_FAILURE | DT_BUFFER_TOO_SMALL;
          uint16_t *e = &edges[nedges * 3];
          e[0] = p[k];
          e[1] = p[j];
          e[2] = mesh.areas[i];
          nedges++;
        }
      }
      // Remove the polygon.
      uint16_t *p2 = &mesh.polys[(mesh.npolys - 1) * MAX_VERTS_PER_POLY * 2];
      std::memcpy(p, p2, sizeof(uint16_t) * MAX_VERTS_PER_POLY);
      std::memset(p + MAX_VERTS_PER_POLY, 0xff, sizeof(uint16_t) * MAX_VERTS_PER_POLY);
      mesh.areas[i] = mesh.areas[mesh.npolys - 1];
      mesh.npolys--;
      --i;
    }
  }

  // Remove vertex.
  for (int i = rem; i < mesh.nverts - 1; ++i) {
    mesh.verts[i * 3 + 0] = mesh.verts[(i + 1) * 3 + 0];
    mesh.verts[i * 3 + 1] = mesh.verts[(i + 1) * 3 + 1];
    mesh.verts[i * 3 + 2] = mesh.verts[(i + 1) * 3 + 2];
  }
  mesh.nverts--;

  // Adjust indices to match the removed vertex layout.
  for (int i = 0; i < mesh.npolys; ++i) {
    uint16_t *p = &mesh.polys[i * MAX_VERTS_PER_POLY * 2];
    const int nv = countPolyVerts(p);
    for (int j = 0; j < nv; ++j)
      if (p[j] > rem)
        p[j]--;
  }
  for (int i = 0; i < nedges; ++i) {
    if (edges[i * 3 + 0] > rem)
      edges[i * 3 + 0]--;
    if (edges[i * 3 + 1] > rem)
      edges[i * 3 + 1]--;
  }

  if (nedges == 0)
    return DT_SUCCESS;

  // Start with one vertex, keep appending connected
  // segments to the start and end of the hole.
  pushBack(edges[0], hole, nhole);
  pushBack(edges[2], harea, nharea);

  while (nedges) {
    bool match = false;

    for (int i = 0; i < nedges; ++i) {
      const uint16_t ea = edges[i * 3 + 0];
      const uint16_t eb = edges[i * 3 + 1];
      const uint16_t a = edges[i * 3 + 2];
      bool add = false;
      if (hole[0] == eb) {
        // The segment matches the beginning of the hole boundary.
        if (nhole >= MAX_REM_EDGES)
          return DT_FAILURE | DT_BUFFER_TOO_SMALL;
        pushFront(ea, hole, nhole);
        pushFront(a, harea, nharea);
        add = true;
      } else if (hole[nhole - 1] == ea) {
        // The segment matches the end of the hole boundary.
        if (nhole >= MAX_REM_EDGES)
          return DT_FAILURE | DT_BUFFER_TOO_SMALL;
        pushBack(eb, hole, nhole);
        pushBack(a, harea, nharea);
        add = true;
      }
      if (add) {
        // The edge segment was added, remove it.
        edges[i * 3 + 0] = edges[(nedges - 1) * 3 + 0];
        edges[i * 3 + 1] = edges[(nedges - 1) * 3 + 1];
        edges[i * 3 + 2] = edges[(nedges - 1) * 3 + 2];
        --nedges;
        match = true;
        --i;
      }
    }

    if (!match)
      break;
  }

  uint16_t tris[MAX_REM_EDGES * 3];
  uint8_t tverts[MAX_REM_EDGES * 3];
  uint16_t tpoly[MAX_REM_EDGES * 3];

  // Generate temp vertex array for triangulation.
  for (int i = 0; i < nhole; ++i) {
    const uint16_t pi = hole[i];
    tverts[i * 4 + 0] = static_cast<uint8_t>(mesh.verts[pi * 3 + 0]);
    tverts[i * 4 + 1] = static_cast<uint8_t>(mesh.verts[pi * 3 + 1]);
    tverts[i * 4 + 2] = static_cast<uint8_t>(mesh.verts[pi * 3 + 2]);
    tverts[i * 4 + 3] = 0;
    tpoly[i] = static_cast<uint16_t>(i);
  }

  // Triangulate the hole.
  int ntris = triangulate(nhole, tverts, tpoly, tris);
  if (ntris < 0) {
    // TODO: issue warning!
    ntris = -ntris;
  }

  if (ntris > MAX_REM_EDGES)
    return DT_FAILURE | DT_BUFFER_TOO_SMALL;

  uint16_t polys[MAX_REM_EDGES * MAX_VERTS_PER_POLY];
  uint8_t pareas[MAX_REM_EDGES];

  // Build initial polygons.
  int npolys = 0;
  std::memset(polys, 0xff, ntris * MAX_VERTS_PER_POLY * sizeof(uint16_t));
  for (int j = 0; j < ntris; ++j) {
    uint16_t *t = &tris[j * 3];
    if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2]) {
      polys[npolys * MAX_VERTS_PER_POLY + 0] = hole[t[0]];
      polys[npolys * MAX_VERTS_PER_POLY + 1] = hole[t[1]];
      polys[npolys * MAX_VERTS_PER_POLY + 2] = hole[t[2]];
      pareas[npolys] = static_cast<uint8_t>(harea[t[0]]);
      npolys++;
    }
  }
  if (!npolys)
    return DT_SUCCESS;

  // Merge polygons.
  if constexpr (MAX_VERTS_PER_POLY > 3) {
    for (;;) {
      // Find best polygons to merge.
      int bestMergeVal = 0;
      int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

      for (int j = 0; j < npolys - 1; ++j) {
        uint16_t *pj = &polys[j * MAX_VERTS_PER_POLY];
        for (int k = j + 1; k < npolys; ++k) {
          uint16_t *pk = &polys[k * MAX_VERTS_PER_POLY];
          int ea, eb;
          const int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb);
          if (v > bestMergeVal) {
            bestMergeVal = v;
            bestPa = j;
            bestPb = k;
            bestEa = ea;
            bestEb = eb;
          }
        }
      }

      if (bestMergeVal > 0) {
        // Found best, merge.
        uint16_t *pa = &polys[bestPa * MAX_VERTS_PER_POLY];
        uint16_t *pb = &polys[bestPb * MAX_VERTS_PER_POLY];
        mergePolys(pa, pb, bestEa, bestEb);
        std::memcpy(pb, &polys[(npolys - 1) * MAX_VERTS_PER_POLY], sizeof(uint16_t) * MAX_VERTS_PER_POLY);
        pareas[bestPb] = pareas[npolys - 1];
        npolys--;
      } else {
        // Could not merge any polygons, stop.
        break;
      }
    }
  }

  // Store polygons.
  for (int i = 0; i < npolys; ++i) {
    if (mesh.npolys >= maxTris)
      break;
    uint16_t *p = &mesh.polys[mesh.npolys * MAX_VERTS_PER_POLY * 2];
    std::memset(p, 0xff, sizeof(uint16_t) * MAX_VERTS_PER_POLY * 2);
    for (int j = 0; j < MAX_VERTS_PER_POLY; ++j)
      p[j] = polys[i * MAX_VERTS_PER_POLY + j];
    mesh.areas[mesh.npolys] = pareas[i];
    mesh.npolys++;
    if (mesh.npolys > maxTris)
      return DT_FAILURE | DT_BUFFER_TOO_SMALL;
  }

  return DT_SUCCESS;
}

} // namespace

dtStatus dtBuildTileCacheRegions(dtTileCacheAlloc *alloc,
                                 dtTileCacheLayer &layer,
                                 const int walkableClimb) {
  dtAssert(alloc);

  const int w = layer.header->width;
  const int h = layer.header->height;

  std::memset(layer.regs, 0xff, sizeof(uint8_t) * w * h);

  const int nsweeps = w;
  dtFixedArray<dtLayerSweepSpan> sweeps(alloc, nsweeps);
  if (!sweeps)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  std::memset(sweeps, 0, sizeof(dtLayerSweepSpan) * nsweeps);

  // Partition walkable area into monotone regions.
  uint8_t prevCount[256];
  uint8_t regId = 0;

  for (int y = 0; y < h; ++y) {
    if (regId > 0)
      std::memset(prevCount, 0, sizeof(uint8_t) * regId);
    uint8_t sweepId = 0;

    for (int x = 0; x < w; ++x) {
      const int idx = x + y * w;
      if (layer.areas[idx] == DT_TILECACHE_NULL_AREA)
        continue;

      uint8_t sid = 0xff;

      // -x
      const int xidx = x - 1 + y * w;
      if (x > 0 && isConnected(layer, idx, xidx, walkableClimb)) {
        if (layer.regs[xidx] != 0xff)
          sid = layer.regs[xidx];
      }

      if (sid == 0xff) {
        sid = sweepId++;
        sweeps[sid].nei = 0xff;
        sweeps[sid].ns = 0;
      }

      // -y
      const int yidx = x + (y - 1) * w;
      if (y > 0 && isConnected(layer, idx, yidx, walkableClimb)) {
        const uint8_t nr = layer.regs[yidx];
        if (nr != 0xff) {
          // Set neighbour when first valid neighbour is encoutered.
          if (sweeps[sid].ns == 0)
            sweeps[sid].nei = nr;

          if (sweeps[sid].nei == nr) {
            // Update existing neighbour
            sweeps[sid].ns++;
            prevCount[nr]++;
          } else {
            // This is hit if there is nore than one neighbour.
            // Invalidate the neighbour.
            sweeps[sid].nei = 0xff;
          }
        }
      }

      layer.regs[idx] = sid;
    }

    // Create unique ID.
    for (int i = 0; i < sweepId; ++i) {
      // If the neighbour is set and there is only one continuous connection to it,
      // the sweep will be merged with the previous one, else new region is created.
      if (sweeps[i].nei != 0xff && static_cast<uint16_t>(prevCount[sweeps[i].nei]) == sweeps[i].ns) {
        sweeps[i].id = sweeps[i].nei;
      } else {
        if (regId == 255) {
          // Region ID's overflow.
          return DT_FAILURE | DT_BUFFER_TOO_SMALL;
        }
        sweeps[i].id = regId++;
      }
    }

    // Remap local sweep ids to region ids.
    for (int x = 0; x < w; ++x) {
      const int idx = x + y * w;
      if (layer.regs[idx] != 0xff)
        layer.regs[idx] = sweeps[layer.regs[idx]].id;
    }
  }

  // Allocate and init layer regions.
  const int nregs = regId;
  dtFixedArray<dtLayerMonotoneRegion> regs(alloc, nregs);
  if (!regs)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  std::memset(regs, 0, sizeof(dtLayerMonotoneRegion) * nregs);
  for (int i = 0; i < nregs; ++i)
    regs[i].regId = 0xff;

  // Find region neighbours.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = x + y * w;
      const uint8_t ri = layer.regs[idx];
      if (ri == 0xff)
        continue;

      // Update area.
      regs[ri].area++;
      regs[ri].areaId = layer.areas[idx];

      // Update neighbours
      const int ymi = x + (y - 1) * w;
      if (y > 0 && isConnected(layer, idx, ymi, walkableClimb)) {
        const uint8_t rai = layer.regs[ymi];
        if (rai != 0xff && rai != ri) {
          addUniqueLast(regs[ri].neis, regs[ri].nneis, rai);
          addUniqueLast(regs[rai].neis, regs[rai].nneis, ri);
        }
      }
    }
  }

  for (int i = 0; i < nregs; ++i)
    regs[i].regId = static_cast<uint8_t>(i);

  for (int i = 0; i < nregs; ++i) {
    const dtLayerMonotoneRegion &reg = regs[i];

    int merge = -1;
    int mergea = 0;
    for (int j = 0; j < static_cast<int>(reg.nneis); ++j) {
      const uint8_t nei = reg.neis[j];
      const dtLayerMonotoneRegion &regn = regs[nei];
      if (reg.regId == regn.regId)
        continue;
      if (reg.areaId != regn.areaId)
        continue;
      if (regn.area > mergea) {
        if (canMerge(reg.regId, regn.regId, regs, nregs)) {
          mergea = regn.area;
          merge = static_cast<int>(nei);
        }
      }
    }
    if (merge != -1) {
      const uint8_t oldId = reg.regId;
      const uint8_t newId = regs[merge].regId;
      for (int j = 0; j < nregs; ++j)
        if (regs[j].regId == oldId)
          regs[j].regId = newId;
    }
  }

  // Compact ids.
  uint8_t remap[256] = {};
  // Find number of unique regions.
  regId = 0;
  for (int i = 0; i < nregs; ++i)
    remap[regs[i].regId] = 1;
  for (int i = 0; i < 256; ++i)
    if (remap[i])
      remap[i] = regId++;
  // Remap ids.
  for (int i = 0; i < nregs; ++i)
    regs[i].regId = remap[regs[i].regId];

  layer.regCount = regId;

  for (int i = 0; i < w * h; ++i) {
    if (layer.regs[i] != 0xff)
      layer.regs[i] = regs[layer.regs[i]].regId;
  }

  return DT_SUCCESS;
}
// TODO: move this somewhere else, once the layer meshing is done.
dtStatus dtBuildTileCacheContours(dtTileCacheAlloc *alloc,
                                  const dtTileCacheLayer &layer,
                                  const int walkableClimb, const float maxError,
                                  dtTileCacheContourSet &lcset) {
  dtAssert(alloc);
  if (!alloc)
    return DT_FAILURE | DT_INVALID_PARAM;

  const int w = layer.header->width;
  const int h = layer.header->height;

  lcset.nconts = layer.regCount;
  lcset.conts = static_cast<dtTileCacheContour *>(alloc->alloc(sizeof(dtTileCacheContour) * lcset.nconts));
  if (!lcset.conts)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  std::memset(lcset.conts, 0, sizeof(dtTileCacheContour) * lcset.nconts);

  // Allocate temp buffer for contour tracing.
  const int maxTempVerts = (w + h) * 2 * 2; // Twice around the layer.

  dtFixedArray<uint8_t> tempVerts(alloc, maxTempVerts * 4);
  if (!tempVerts)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  dtFixedArray<uint16_t> tempPoly(alloc, maxTempVerts);
  if (!tempPoly)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  dtTempContour temp(tempVerts, maxTempVerts, tempPoly, maxTempVerts);

  // Find contours.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = x + y * w;
      const uint8_t ri = layer.regs[idx];
      if (ri == 0xff)
        continue;

      dtTileCacheContour &cont = lcset.conts[ri];

      if (cont.nverts > 0)
        continue;

      cont.reg = ri;
      cont.area = layer.areas[idx];

      if (!walkContour(layer, x, y, temp)) {
        // Too complex contour.
        // Note: If you hit here ofte, try increasing 'maxTempVerts'.
        return DT_FAILURE | DT_BUFFER_TOO_SMALL;
      }

      simplifyContour(temp, maxError);

      // Store contour.
      cont.nverts = temp.nverts;
      if (cont.nverts > 0) {
        cont.verts = static_cast<uint8_t *>(alloc->alloc(sizeof(uint8_t) * 4 * temp.nverts));
        if (!cont.verts)
          return DT_FAILURE | DT_OUT_OF_MEMORY;

        for (int i = 0, j = temp.nverts - 1; i < temp.nverts; j = i++) {
          uint8_t *dst = &cont.verts[j * 4];
          const uint8_t *v = &temp.verts[j * 4];
          const uint8_t *vn = &temp.verts[i * 4];
          const uint8_t nei = vn[3]; // The neighbour reg is stored at segment vertex of a segment.
          bool shouldRemove = false;
          const uint8_t lh = getCornerHeight(layer, v[0], v[1], v[2],
                                             walkableClimb, shouldRemove);

          dst[0] = v[0];
          dst[1] = lh;
          dst[2] = v[2];

          // Store portal direction and remove status to the fourth component.
          dst[3] = 0x0f;
          if (nei != 0xff && nei >= 0xf8)
            dst[3] = nei - 0xf8;
          if (shouldRemove)
            dst[3] |= 0x80;
        }
      }
    }
  }

  return DT_SUCCESS;
}
dtStatus dtBuildTileCachePolyMesh(dtTileCacheAlloc *alloc,
                                  const dtTileCacheContourSet &lcset,
                                  dtTileCachePolyMesh &mesh) {
  dtAssert(alloc);

  int maxVertices = 0;
  int maxTris = 0;
  int maxVertsPerCont = 0;
  for (int i = 0; i < lcset.nconts; ++i) {
    // Skip null contours.
    if (lcset.conts[i].nverts < 3)
      continue;
    maxVertices += lcset.conts[i].nverts;
    maxTris += lcset.conts[i].nverts - 2;
    maxVertsPerCont = dtMax(maxVertsPerCont, lcset.conts[i].nverts);
  }

  // TODO: warn about too many vertices?

  mesh.nvp = MAX_VERTS_PER_POLY;

  dtFixedArray<uint8_t> vflags(alloc, maxVertices);
  if (!vflags)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  std::memset(vflags, 0, maxVertices);

  mesh.verts = static_cast<uint16_t *>(alloc->alloc(sizeof(uint16_t) * maxVertices * 3));
  if (!mesh.verts)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  mesh.polys = static_cast<uint16_t *>(alloc->alloc(sizeof(uint16_t) * maxTris * MAX_VERTS_PER_POLY * 2));
  if (!mesh.polys)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  mesh.areas = static_cast<uint8_t *>(alloc->alloc(sizeof(uint8_t) * maxTris));
  if (!mesh.areas)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  mesh.flags = static_cast<uint16_t *>(alloc->alloc(sizeof(uint16_t) * maxTris));
  if (!mesh.flags)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  // Just allocate and clean the mesh flags array. The user is resposible for filling it.
  std::memset(mesh.flags, 0, sizeof(uint16_t) * maxTris);

  mesh.nverts = 0;
  mesh.npolys = 0;

  std::memset(mesh.verts, 0, sizeof(uint16_t) * maxVertices * 3);
  std::memset(mesh.polys, 0xff, sizeof(uint16_t) * maxTris * MAX_VERTS_PER_POLY * 2);
  std::memset(mesh.areas, 0, sizeof(uint8_t) * maxTris);

  uint16_t firstVert[VERTEX_BUCKET_COUNT2];
  for (int i = 0; i < VERTEX_BUCKET_COUNT2; ++i)
    firstVert[i] = DT_TILECACHE_NULL_IDX;

  dtFixedArray<uint16_t> nextVert(alloc, maxVertices);
  if (!nextVert)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  std::memset(nextVert, 0, sizeof(uint16_t) * maxVertices);

  dtFixedArray<uint16_t> indices(alloc, maxVertsPerCont);
  if (!indices)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  dtFixedArray<uint16_t> tris(alloc, maxVertsPerCont * 3);
  if (!tris)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  dtFixedArray<uint16_t> polys(alloc, maxVertsPerCont * MAX_VERTS_PER_POLY);
  if (!polys)
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  for (int i = 0; i < lcset.nconts; ++i) {
    const dtTileCacheContour &cont = lcset.conts[i];

    // Skip null contours.
    if (cont.nverts < 3)
      continue;

    // Triangulate contour
    for (int j = 0; j < cont.nverts; ++j)
      indices[j] = static_cast<uint16_t>(j);

    int ntris = triangulate(cont.nverts, cont.verts, &indices[0], &tris[0]);
    if (ntris <= 0) {
      // TODO: issue warning!
      ntris = -ntris;
    }

    // Add and merge vertices.
    for (int j = 0; j < cont.nverts; ++j) {
      const uint8_t *v = &cont.verts[j * 4];
      indices[j] = addVertex(v[0], v[1], v[2],
                             mesh.verts, firstVert, nextVert, mesh.nverts);
      if (v[3] & 0x80) {
        // This vertex should be removed.
        vflags[indices[j]] = 1;
      }
    }

    // Build initial polygons.
    int npolys = 0;
    std::memset(polys, 0xff, sizeof(uint16_t) * maxVertsPerCont * MAX_VERTS_PER_POLY);
    for (int j = 0; j < ntris; ++j) {
      const uint16_t *t = &tris[j * 3];
      if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2]) {
        polys[npolys * MAX_VERTS_PER_POLY + 0] = indices[t[0]];
        polys[npolys * MAX_VERTS_PER_POLY + 1] = indices[t[1]];
        polys[npolys * MAX_VERTS_PER_POLY + 2] = indices[t[2]];
        npolys++;
      }
    }
    if (!npolys)
      continue;

    // Merge polygons.
    if constexpr (MAX_VERTS_PER_POLY > 3) {
      for (;;) {
        // Find best polygons to merge.
        int bestMergeVal = 0;
        int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;

        for (int j = 0; j < npolys - 1; ++j) {
          const uint16_t *pj = &polys[j * MAX_VERTS_PER_POLY];
          for (int k = j + 1; k < npolys; ++k) {
            const uint16_t *pk = &polys[k * MAX_VERTS_PER_POLY];
            int ea, eb;
            const int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb);
            if (v > bestMergeVal) {
              bestMergeVal = v;
              bestPa = j;
              bestPb = k;
              bestEa = ea;
              bestEb = eb;
            }
          }
        }

        if (bestMergeVal > 0) {
          // Found best, merge.
          uint16_t *pa = &polys[bestPa * MAX_VERTS_PER_POLY];
          uint16_t *pb = &polys[bestPb * MAX_VERTS_PER_POLY];
          mergePolys(pa, pb, bestEa, bestEb);
          std::memcpy(pb, &polys[(npolys - 1) * MAX_VERTS_PER_POLY], sizeof(uint16_t) * MAX_VERTS_PER_POLY);
          npolys--;
        } else {
          // Could not merge any polygons, stop.
          break;
        }
      }
    }

    // Store polygons.
    for (int j = 0; j < npolys; ++j) {
      uint16_t *p = &mesh.polys[mesh.npolys * MAX_VERTS_PER_POLY * 2];
      const uint16_t *q = &polys[j * MAX_VERTS_PER_POLY];
      for (int k = 0; k < MAX_VERTS_PER_POLY; ++k)
        p[k] = q[k];
      mesh.areas[mesh.npolys] = cont.area;
      mesh.npolys++;
      if (mesh.npolys > maxTris)
        return DT_FAILURE | DT_BUFFER_TOO_SMALL;
    }
  }

  // Remove edge vertices.
  for (int i = 0; i < mesh.nverts; ++i) {
    if (vflags[i]) {
      if (!canRemoveVertex(mesh, static_cast<uint16_t>(i)))
        continue;
      const dtStatus status = removeVertex(mesh, static_cast<uint16_t>(i), maxTris);
      if (dtStatusFailed(status))
        return status;
      // Remove vertex
      // Note: mesh.nverts is already decremented inside removeVertex()!
      for (int j = i; j < mesh.nverts; ++j)
        vflags[j] = vflags[j + 1];
      --i;
    }
  }

  // Calculate adjacency.
  if (!buildMeshAdjacency(alloc, mesh.polys, mesh.npolys, mesh.verts, mesh.nverts, lcset))
    return DT_FAILURE | DT_OUT_OF_MEMORY;

  return DT_SUCCESS;
}

dtStatus dtMarkCylinderArea(const dtTileCacheLayer &layer, const float *orig, const float cs, const float ch,
                            const float *pos, const float radius, const float height, const uint8_t areaId) {
  float bmin[3], bmax[3];
  bmin[0] = pos[0] - radius;
  bmin[1] = pos[1];
  bmin[2] = pos[2] - radius;
  bmax[0] = pos[0] + radius;
  bmax[1] = pos[1] + height;
  bmax[2] = pos[2] + radius;
  const float r2 = dtSqr(radius / cs + 0.5f);

  const int w = layer.header->width;
  const int h = layer.header->height;
  const float ics = 1.0f / cs;
  const float ich = 1.0f / ch;

  const float px = (pos[0] - orig[0]) * ics;
  const float pz = (pos[2] - orig[2]) * ics;

  int minx = static_cast<int>(dtMathFloorf((bmin[0] - orig[0]) * ics));
  const int miny = static_cast<int>(dtMathFloorf((bmin[1] - orig[1]) * ich));
  int minz = static_cast<int>(dtMathFloorf((bmin[2] - orig[2]) * ics));
  int maxx = static_cast<int>(dtMathFloorf((bmax[0] - orig[0]) * ics));
  const int maxy = static_cast<int>(dtMathFloorf((bmax[1] - orig[1]) * ich));
  int maxz = static_cast<int>(dtMathFloorf((bmax[2] - orig[2]) * ics));

  if (maxx < 0)
    return DT_SUCCESS;
  if (minx >= w)
    return DT_SUCCESS;
  if (maxz < 0)
    return DT_SUCCESS;
  if (minz >= h)
    return DT_SUCCESS;

  if (minx < 0)
    minx = 0;
  if (maxx >= w)
    maxx = w - 1;
  if (minz < 0)
    minz = 0;
  if (maxz >= h)
    maxz = h - 1;

  for (int z = minz; z <= maxz; ++z) {
    for (int x = minx; x <= maxx; ++x) {
      const float dx = x + 0.5f - px;
      const float dz = z + 0.5f - pz;
      if (dx * dx + dz * dz > r2)
        continue;
      const int y = layer.heights[x + z * w];
      if (y < miny || y > maxy)
        continue;
      layer.areas[x + z * w] = areaId;
    }
  }

  return DT_SUCCESS;
}

dtStatus dtMarkBoxArea(const dtTileCacheLayer &layer, const float *orig, const float cs, const float ch,
                       const float *bmin, const float *bmax, const uint8_t areaId) {
  const int w = layer.header->width;
  const int h = layer.header->height;
  const float ics = 1.0f / cs;
  const float ich = 1.0f / ch;

  int minx = static_cast<int>(floorf((bmin[0] - orig[0]) * ics));
  const int miny = static_cast<int>(floorf((bmin[1] - orig[1]) * ich));
  int minz = static_cast<int>(floorf((bmin[2] - orig[2]) * ics));
  int maxx = static_cast<int>(floorf((bmax[0] - orig[0]) * ics));
  const int maxy = static_cast<int>(floorf((bmax[1] - orig[1]) * ich));
  int maxz = static_cast<int>(floorf((bmax[2] - orig[2]) * ics));

  if (maxx < 0)
    return DT_SUCCESS;
  if (minx >= w)
    return DT_SUCCESS;
  if (maxz < 0)
    return DT_SUCCESS;
  if (minz >= h)
    return DT_SUCCESS;

  if (minx < 0)
    minx = 0;
  if (maxx >= w)
    maxx = w - 1;
  if (minz < 0)
    minz = 0;
  if (maxz >= h)
    maxz = h - 1;

  for (int z = minz; z <= maxz; ++z) {
    for (int x = minx; x <= maxx; ++x) {
      const int y = layer.heights[x + z * w];
      if (y < miny || y > maxy)
        continue;
      layer.areas[x + z * w] = areaId;
    }
  }

  return DT_SUCCESS;
}

dtStatus dtMarkBoxArea(const dtTileCacheLayer &layer, const float *orig, const float cs, const float ch,
                       const float *center, const float *halfExtents, const float *rotAux, const uint8_t areaId) {
  const int w = layer.header->width;
  const int h = layer.header->height;
  const float ics = 1.0f / cs;
  const float ich = 1.0f / ch;

  const float cx = (center[0] - orig[0]) * ics;
  const float cz = (center[2] - orig[2]) * ics;

  const float maxr = 1.41f * dtMax(halfExtents[0], halfExtents[2]);
  int minx = static_cast<int>(floorf(cx - maxr * ics));
  int maxx = static_cast<int>(floorf(cx + maxr * ics));
  int minz = static_cast<int>(floorf(cz - maxr * ics));
  int maxz = static_cast<int>(floorf(cz + maxr * ics));
  const int miny = static_cast<int>(floorf((center[1] - halfExtents[1] - orig[1]) * ich));
  const int maxy = static_cast<int>(floorf((center[1] + halfExtents[1] - orig[1]) * ich));

  if (maxx < 0)
    return DT_SUCCESS;
  if (minx >= w)
    return DT_SUCCESS;
  if (maxz < 0)
    return DT_SUCCESS;
  if (minz >= h)
    return DT_SUCCESS;

  if (minx < 0)
    minx = 0;
  if (maxx >= w)
    maxx = w - 1;
  if (minz < 0)
    minz = 0;
  if (maxz >= h)
    maxz = h - 1;

  const float xhalf = halfExtents[0] * ics + 0.5f;
  const float zhalf = halfExtents[2] * ics + 0.5f;

  for (int z = minz; z <= maxz; ++z) {
    for (int x = minx; x <= maxx; ++x) {
      const float x2 = 2.0f * (static_cast<float>(x) - cx);
      const float z2 = 2.0f * (static_cast<float>(z) - cz);
      const float xrot = rotAux[1] * x2 + rotAux[0] * z2;
      if (xrot > xhalf || xrot < -xhalf)
        continue;
      const float zrot = rotAux[1] * z2 - rotAux[0] * x2;
      if (zrot > zhalf || zrot < -zhalf)
        continue;
      const int y = layer.heights[x + z * w];
      if (y < miny || y > maxy)
        continue;
      layer.areas[x + z * w] = areaId;
    }
  }

  return DT_SUCCESS;
}

dtStatus dtBuildTileCacheLayer(dtTileCacheCompressor *comp,
                               const dtTileCacheLayerHeader *header,
                               const uint8_t *heights,
                               const uint8_t *areas,
                               const uint8_t *cons,
                               uint8_t **outData, int *outDataSize) {
  const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
  const int gridSize = static_cast<int>(header->width) * static_cast<int>(header->height);
  const int maxDataSize = headerSize + comp->maxCompressedSize(gridSize * 3);
  auto *const data = static_cast<uint8_t *>(dtAlloc(maxDataSize, DT_ALLOC_PERM));
  if (!data)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  std::memset(data, 0, maxDataSize);

  // Store header
  std::memcpy(data, header, sizeof(dtTileCacheLayerHeader));

  // Concatenate grid data for compression.
  const int bufferSize = gridSize * 3;
  auto *const buffer = static_cast<uint8_t *>(dtAlloc(bufferSize, DT_ALLOC_TEMP));
  if (!buffer) {
    dtFree(data);
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  }

  std::memcpy(buffer, heights, gridSize);
  std::memcpy(buffer + gridSize, areas, gridSize);
  std::memcpy(buffer + gridSize * 2, cons, gridSize);

  // Compress
  uint8_t *compressed = data + headerSize;
  const int maxCompressedSize = maxDataSize - headerSize;
  int compressedSize = 0;
  const dtStatus status = comp->compress(buffer, bufferSize, compressed, maxCompressedSize, &compressedSize);
  if (dtStatusFailed(status)) {
    dtFree(buffer);
    dtFree(data);
    return status;
  }

  *outData = data;
  *outDataSize = headerSize + compressedSize;

  dtFree(buffer);

  return DT_SUCCESS;
}

void dtFreeTileCacheLayer(dtTileCacheAlloc *alloc, dtTileCacheLayer *layer) {
  dtAssert(alloc);
  if (!alloc)
    return;
  // The layer is allocated as one conitguous blob of data.
  alloc->free(layer);
}

dtStatus dtDecompressTileCacheLayer(dtTileCacheAlloc *alloc, dtTileCacheCompressor *comp,
                                    uint8_t *compressed, const int compressedSize,
                                    dtTileCacheLayer **layerOut) {
  dtAssert(alloc);
  dtAssert(comp);
  if (!alloc || !comp)
    return DT_FAILURE | DT_INVALID_PARAM;

  if (!layerOut)
    return DT_FAILURE | DT_INVALID_PARAM;
  if (!compressed)
    return DT_FAILURE | DT_INVALID_PARAM;

  *layerOut = nullptr;

  const dtTileCacheLayerHeader *compressedHeader = reinterpret_cast<dtTileCacheLayerHeader *>(compressed);
  if (compressedHeader->magic != DT_TILECACHE_MAGIC)
    return DT_FAILURE | DT_WRONG_MAGIC;
  if (compressedHeader->version != DT_TILECACHE_VERSION)
    return DT_FAILURE | DT_WRONG_VERSION;

  const int layerSize = dtAlign4(sizeof(dtTileCacheLayer));
  const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
  const int gridSize = static_cast<int>(compressedHeader->width) * static_cast<int>(compressedHeader->height);
  const int bufferSize = layerSize + headerSize + gridSize * 4;

  auto *const buffer = static_cast<uint8_t *>(alloc->alloc(bufferSize));
  if (!buffer)
    return DT_FAILURE | DT_OUT_OF_MEMORY;
  std::memset(buffer, 0, bufferSize);

  auto *const layer = reinterpret_cast<dtTileCacheLayer *>(buffer);
  auto *const header = reinterpret_cast<dtTileCacheLayerHeader *>(buffer + layerSize);
  uint8_t *grids = buffer + layerSize + headerSize;
  const int gridsSize = bufferSize - (layerSize + headerSize);

  // Copy header
  std::memcpy(header, compressedHeader, headerSize);
  // Decompress grid.
  int size = 0;
  const dtStatus status = comp->decompress(compressed + headerSize, compressedSize - headerSize,
                                           grids, gridsSize, &size);
  if (dtStatusFailed(status)) {
    alloc->free(buffer);
    return status;
  }

  layer->header = header;
  layer->heights = grids;
  layer->areas = grids + gridSize;
  layer->cons = grids + gridSize * 2;
  layer->regs = grids + gridSize * 3;

  *layerOut = layer;

  return DT_SUCCESS;
}

bool dtTileCacheHeaderSwapEndian(uint8_t *data, const int dataSize) {
  dtIgnoreUnused(dataSize);
  auto *const header = reinterpret_cast<dtTileCacheLayerHeader *>(data);

  int swappedMagic = DT_TILECACHE_MAGIC;
  int swappedVersion = DT_TILECACHE_VERSION;
  dtSwapEndian(&swappedMagic);
  dtSwapEndian(&swappedVersion);

  if ((header->magic != DT_TILECACHE_MAGIC || header->version != DT_TILECACHE_VERSION) &&
      (header->magic != swappedMagic || header->version != swappedVersion)) {
    return false;
  }

  dtSwapEndian(&header->magic);
  dtSwapEndian(&header->version);
  dtSwapEndian(&header->tx);
  dtSwapEndian(&header->ty);
  dtSwapEndian(&header->tlayer);
  dtSwapEndian(&header->bmin[0]);
  dtSwapEndian(&header->bmin[1]);
  dtSwapEndian(&header->bmin[2]);
  dtSwapEndian(&header->bmax[0]);
  dtSwapEndian(&header->bmax[1]);
  dtSwapEndian(&header->bmax[2]);
  dtSwapEndian(&header->hmin);
  dtSwapEndian(&header->hmax);

  // width, height, minx, maxx, miny, maxy are uint8_t, no need to swap.

  return true;
}

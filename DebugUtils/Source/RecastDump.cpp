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

#include "RecastDump.h"

#include <Recast.h>
#include <RecastAlloc.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

namespace {
void ioprintf(duFileIO *io, const char *format, ...) {
  char line[256];
  va_list ap;
  va_start(ap, format);
  const int n = std::vsnprintf(line, sizeof(line), format, ap);
  va_end(ap);
  if (n > 0)
    io->write(line, sizeof(char) * n);
}
void logLine(rcContext &ctx, const rcTimerLabel label, const char *name, const float pc) {
  const int t = ctx.getAccumulatedTime(label);
  if (t < 0)
    return;
  ctx.log(RC_LOG_PROGRESS, "%s:\t%.2fms\t(%.1f%%)", name, static_cast<float>(t) / 1000.0f, static_cast<float>(t) * pc);
}
} // namespace

bool duDumpPolyMeshToObj(const rcPolyMesh &pmesh, duFileIO *io) {
  if (!io) {
    std::cout << "duDumpPolyMeshToObj: input IO is null." << std::endl;
    return false;
  }
  if (!io->isWriting()) {
    std::cout << "duDumpPolyMeshToObj: input IO not writing." << std::endl;
    return false;
  }

  const int nvp = pmesh.nvp;
  const float cs = pmesh.cs;
  const float ch = pmesh.ch;
  const float *orig = pmesh.bmin;

  ioprintf(io, "# Recast Navmesh\n");
  ioprintf(io, "o NavMesh\n");

  ioprintf(io, "\n");

  for (int i = 0; i < pmesh.nverts; ++i) {
    const uint16_t *v = &pmesh.verts[i * 3];
    const float x = orig[0] + static_cast<float>(v[0]) * cs;
    const float y = orig[1] + static_cast<float>(v[1] + 1) * ch + 0.1f;
    const float z = orig[2] + static_cast<float>(v[2]) * cs;
    ioprintf(io, "v %f %f %f\n", x, y, z);
  }

  ioprintf(io, "\n");

  for (int i = 0; i < pmesh.npolys; ++i) {
    const unsigned short *p = &pmesh.polys[i * nvp * 2];
    for (int j = 2; j < nvp; ++j) {
      if (p[j] == RC_MESH_NULL_IDX)
        break;
      ioprintf(io, "f %d %d %d\n", p[0] + 1, p[j - 1] + 1, p[j] + 1);
    }
  }

  return true;
}

bool duDumpPolyMeshDetailToObj(const rcPolyMeshDetail &dmesh, duFileIO *io) {
  if (!io) {
    std::cout << "duDumpPolyMeshDetailToObj: input IO is null." << std::endl;
    return false;
  }
  if (!io->isWriting()) {
    std::cout << "duDumpPolyMeshDetailToObj: input IO not writing." << std::endl;
    return false;
  }

  ioprintf(io, "# Recast Navmesh\n");
  ioprintf(io, "o NavMesh\n");

  ioprintf(io, "\n");

  for (int i = 0; i < dmesh.nverts; ++i) {
    const float *v = &dmesh.verts[i * 3];
    ioprintf(io, "v %f %f %f\n", v[0], v[1], v[2]);
  }

  ioprintf(io, "\n");

  for (int i = 0; i < dmesh.nmeshes; ++i) {
    const uint32_t *m = &dmesh.meshes[i * 4];
    const uint32_t bverts = m[0];
    const uint32_t btris = m[2];
    const uint32_t ntris = m[3];
    const unsigned char *tris = &dmesh.tris[btris * 4];
    for (uint32_t j = 0; j < ntris; ++j) {
      ioprintf(io, "f %d %d %d\n",
               static_cast<int>(bverts + tris[j * 4 + 0]) + 1,
               static_cast<int>(bverts + tris[j * 4 + 1]) + 1,
               static_cast<int>(bverts + tris[j * 4 + 2]) + 1);
    }
  }

  return true;
}

static constexpr int CSET_MAGIC = ('c' << 24) | ('s' << 16) | ('e' << 8) | 't';
static constexpr int CSET_VERSION = 2;

bool duDumpContourSet(const rcContourSet &cset, duFileIO *io) {
  if (!io) {
    std::cout << "duDumpContourSet: input IO is null." << std::endl;
    return false;
  }
  if (!io->isWriting()) {
    std::cout << "duDumpContourSet: input IO not writing." << std::endl;
    return false;
  }

  io->write(&CSET_MAGIC, sizeof(CSET_MAGIC));
  io->write(&CSET_VERSION, sizeof(CSET_VERSION));

  io->write(&cset.nconts, sizeof(cset.nconts));

  io->write(cset.bmin, sizeof(cset.bmin));
  io->write(cset.bmax, sizeof(cset.bmax));

  io->write(&cset.cs, sizeof(cset.cs));
  io->write(&cset.ch, sizeof(cset.ch));

  io->write(&cset.width, sizeof(cset.width));
  io->write(&cset.height, sizeof(cset.height));
  io->write(&cset.borderSize, sizeof(cset.borderSize));

  for (int i = 0; i < cset.nconts; ++i) {
    const auto &[verts, nverts, rverts, nrverts, reg, area] = cset.conts[i];
    io->write(&nverts, sizeof(nverts));
    io->write(&nrverts, sizeof(nrverts));
    io->write(&reg, sizeof(reg));
    io->write(&area, sizeof(area));
    io->write(verts, sizeof(int) * 4 * nverts);
    io->write(rverts, sizeof(int) * 4 * nrverts);
  }

  return true;
}

bool duReadContourSet(rcContourSet &cset, duFileIO *io) {
  if (!io) {
    std::cout << "duReadContourSet: input IO is null." << std::endl;
    return false;
  }
  if (!io->isReading()) {
    std::cout << "duReadContourSet: input IO not reading." << std::endl;
    return false;
  }

  int magic = 0;
  int version = 0;

  io->read(&magic, sizeof(magic));
  io->read(&version, sizeof(version));

  if (magic != CSET_MAGIC) {
    std::cout << "duReadContourSet: Bad voodoo." << std::endl;
    return false;
  }
  if (version != CSET_VERSION) {
    std::cout << "duReadContourSet: Bad version." << std::endl;
    return false;
  }

  io->read(&cset.nconts, sizeof(cset.nconts));

  cset.conts = static_cast<rcContour *>(rcAlloc(sizeof(rcContour) * cset.nconts, RC_ALLOC_PERM));
  if (!cset.conts) {
    std::cout << "duReadContourSet: Could not alloc contours (" << cset.nconts << ")." << std::endl;
    return false;
  }
  std::memset(cset.conts, 0, sizeof(rcContour) * cset.nconts);

  io->read(cset.bmin, sizeof(cset.bmin));
  io->read(cset.bmax, sizeof(cset.bmax));

  io->read(&cset.cs, sizeof(cset.cs));
  io->read(&cset.ch, sizeof(cset.ch));

  io->read(&cset.width, sizeof(cset.width));
  io->read(&cset.height, sizeof(cset.height));
  io->read(&cset.borderSize, sizeof(cset.borderSize));

  for (int i = 0; i < cset.nconts; ++i) {
    auto &[verts, nverts, rverts, nrverts, reg, area] = cset.conts[i];
    io->read(&nverts, sizeof(nverts));
    io->read(&nrverts, sizeof(nrverts));
    io->read(&reg, sizeof(reg));
    io->read(&area, sizeof(area));

    verts = static_cast<int *>(rcAlloc(sizeof(int) * 4 * nverts, RC_ALLOC_PERM));
    if (!verts) {
      std::cout << "duReadContourSet: Could not alloc contours verts (" << nverts << ")." << std::endl;
      return false;
    }
    rverts = static_cast<int *>(rcAlloc(sizeof(int) * 4 * nrverts, RC_ALLOC_PERM));
    if (!rverts) {
      std::cout << "duReadContourSet: Could not alloc contours rverts (" << nrverts << ")." << std::endl;
      return false;
    }

    io->read(verts, sizeof(int) * 4 * nverts);
    io->read(rverts, sizeof(int) * 4 * nrverts);
  }

  return true;
}

static constexpr int CHF_MAGIC = ('r' << 24) | ('c' << 16) | ('h' << 8) | 'f';
static constexpr int CHF_VERSION = 3;

bool duDumpCompactHeightfield(const rcCompactHeightfield &chf, duFileIO *io) {
  if (!io) {
    std::cout << "duDumpCompactHeightfield: input IO is null." << std::endl;
    return false;
  }
  if (!io->isWriting()) {
    std::cout << "duDumpCompactHeightfield: input IO not writing." << std::endl;
    return false;
  }

  io->write(&CHF_MAGIC, sizeof(CHF_MAGIC));
  io->write(&CHF_VERSION, sizeof(CHF_VERSION));

  io->write(&chf.width, sizeof(chf.width));
  io->write(&chf.height, sizeof(chf.height));
  io->write(&chf.spanCount, sizeof(chf.spanCount));

  io->write(&chf.walkableHeight, sizeof(chf.walkableHeight));
  io->write(&chf.walkableClimb, sizeof(chf.walkableClimb));
  io->write(&chf.borderSize, sizeof(chf.borderSize));

  io->write(&chf.maxDistance, sizeof(chf.maxDistance));
  io->write(&chf.maxRegions, sizeof(chf.maxRegions));

  io->write(chf.bmin, sizeof(chf.bmin));
  io->write(chf.bmax, sizeof(chf.bmax));

  io->write(&chf.cs, sizeof(chf.cs));
  io->write(&chf.ch, sizeof(chf.ch));

  int tmp = 0;
  if (chf.cells)
    tmp |= 1;
  if (chf.spans)
    tmp |= 2;
  if (chf.dist)
    tmp |= 4;
  if (chf.areas)
    tmp |= 8;

  io->write(&tmp, sizeof(tmp));

  if (chf.cells)
    io->write(chf.cells, sizeof(rcCompactCell) * chf.width * chf.height);
  if (chf.spans)
    io->write(chf.spans, sizeof(rcCompactSpan) * chf.spanCount);
  if (chf.dist)
    io->write(chf.dist, sizeof(unsigned short) * chf.spanCount);
  if (chf.areas)
    io->write(chf.areas, sizeof(unsigned char) * chf.spanCount);

  return true;
}

bool duReadCompactHeightfield(rcCompactHeightfield &chf, duFileIO *io) {
  if (!io) {
    std::cout << "duReadCompactHeightfield: input IO is null." << std::endl;
    return false;
  }
  if (!io->isReading()) {
    std::cout << "duReadCompactHeightfield: input IO not reading." << std::endl;
    return false;
  }

  int magic = 0;
  int version = 0;

  io->read(&magic, sizeof(magic));
  io->read(&version, sizeof(version));

  if (magic != CHF_MAGIC) {
    std::cout << "duReadCompactHeightfield: Bad voodoo." << std::endl;
    return false;
  }
  if (version != CHF_VERSION) {
    std::cout << "duReadCompactHeightfield: Bad version." << std::endl;
    return false;
  }

  io->read(&chf.width, sizeof(chf.width));
  io->read(&chf.height, sizeof(chf.height));
  io->read(&chf.spanCount, sizeof(chf.spanCount));

  io->read(&chf.walkableHeight, sizeof(chf.walkableHeight));
  io->read(&chf.walkableClimb, sizeof(chf.walkableClimb));
  io->read(&chf.borderSize, sizeof(chf.borderSize));

  io->read(&chf.maxDistance, sizeof(chf.maxDistance));
  io->read(&chf.maxRegions, sizeof(chf.maxRegions));

  io->read(chf.bmin, sizeof(chf.bmin));
  io->read(chf.bmax, sizeof(chf.bmax));

  io->read(&chf.cs, sizeof(chf.cs));
  io->read(&chf.ch, sizeof(chf.ch));

  int tmp = 0;
  io->read(&tmp, sizeof(tmp));

  if (tmp & 1) {
    chf.cells = static_cast<rcCompactCell *>(rcAlloc(sizeof(rcCompactCell) * chf.width * chf.height, RC_ALLOC_PERM));
    if (!chf.cells) {
      std::cout << "duReadCompactHeightfield: Could not alloc cells (" << chf.width * chf.height << ")." << std::endl;
      return false;
    }
    io->read(chf.cells, sizeof(rcCompactCell) * chf.width * chf.height);
  }
  if (tmp & 2) {
    chf.spans = static_cast<rcCompactSpan *>(rcAlloc(sizeof(rcCompactSpan) * chf.spanCount, RC_ALLOC_PERM));
    if (!chf.spans) {
      std::cout << "duReadCompactHeightfield: Could not alloc spans (" << chf.spanCount << ")." << std::endl;
      return false;
    }
    io->read(chf.spans, sizeof(rcCompactSpan) * chf.spanCount);
  }
  if (tmp & 4) {
    chf.dist = static_cast<unsigned short *>(rcAlloc(sizeof(unsigned short) * chf.spanCount, RC_ALLOC_PERM));
    if (!chf.dist) {
      std::cout << "duReadCompactHeightfield: Could not alloc dist (" << chf.spanCount << ")." << std::endl;
      return false;
    }
    io->read(chf.dist, sizeof(unsigned short) * chf.spanCount);
  }
  if (tmp & 8) {
    chf.areas = static_cast<unsigned char *>(rcAlloc(sizeof(unsigned char) * chf.spanCount, RC_ALLOC_PERM));
    if (!chf.areas) {
      std::cout << "duReadCompactHeightfield: Could not alloc areas (" << chf.spanCount << ")." << std::endl;
      return false;
    }
    io->read(chf.areas, sizeof(unsigned char) * chf.spanCount);
  }

  return true;
}

void duLogBuildTimes(rcContext &ctx, const int totalTimeUsec) {
  const float pc = 100.0f / static_cast<float>(totalTimeUsec);

  ctx.log(RC_LOG_PROGRESS, "Build Times");
  logLine(ctx, RC_TIMER_RASTERIZE_TRIANGLES, "- Rasterize", pc);
  logLine(ctx, RC_TIMER_BUILD_COMPACTHEIGHTFIELD, "- Build Compact", pc);
  logLine(ctx, RC_TIMER_FILTER_BORDER, "- Filter Border", pc);
  logLine(ctx, RC_TIMER_FILTER_WALKABLE, "- Filter Walkable", pc);
  logLine(ctx, RC_TIMER_ERODE_AREA, "- Erode Area", pc);
  logLine(ctx, RC_TIMER_MEDIAN_AREA, "- Median Area", pc);
  logLine(ctx, RC_TIMER_MARK_BOX_AREA, "- Mark Box Area", pc);
  logLine(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA, "- Mark Convex Area", pc);
  logLine(ctx, RC_TIMER_MARK_CYLINDER_AREA, "- Mark Cylinder Area", pc);
  logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD, "- Build Distance Field", pc);
  logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST, "    - Distance", pc);
  logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR, "    - Blur", pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS, "- Build Regions", pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS_WATERSHED, "    - Watershed", pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS_EXPAND, "      - Expand", pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS_FLOOD, "      - Find Basins", pc);
  logLine(ctx, RC_TIMER_BUILD_REGIONS_FILTER, "    - Filter", pc);
  logLine(ctx, RC_TIMER_BUILD_LAYERS, "- Build Layers", pc);
  logLine(ctx, RC_TIMER_BUILD_CONTOURS, "- Build Contours", pc);
  logLine(ctx, RC_TIMER_BUILD_CONTOURS_TRACE, "    - Trace", pc);
  logLine(ctx, RC_TIMER_BUILD_CONTOURS_SIMPLIFY, "    - Simplify", pc);
  logLine(ctx, RC_TIMER_BUILD_POLYMESH, "- Build Polymesh", pc);
  logLine(ctx, RC_TIMER_BUILD_POLYMESHDETAIL, "- Build Polymesh Detail", pc);
  logLine(ctx, RC_TIMER_MERGE_POLYMESH, "- Merge Polymeshes", pc);
  logLine(ctx, RC_TIMER_MERGE_POLYMESHDETAIL, "- Merge Polymesh Details", pc);
  ctx.log(RC_LOG_PROGRESS, "=== TOTAL:\t%.2fms", static_cast<float>(totalTimeUsec) / 1000.0f);
}

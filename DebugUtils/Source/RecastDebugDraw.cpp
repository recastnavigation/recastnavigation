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

#include "RecastDebugDraw.h"

#include <cmath>

#include <DebugDraw.h>
#include <Recast.h>

namespace {
void drawLayerPortals(duDebugDraw *dd, const rcHeightfieldLayer *layer) {
  const float cs = layer->cs;
  const float ch = layer->ch;
  const int w = layer->width;
  const int h = layer->height;

  const uint32_t pcol = duRGBA(255, 255, 255, 255);

  const int segs[4 * 4] = {0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0};

  // Layer portals
  dd->begin(DU_DRAW_LINES, 2.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = x + y * w;
      const int lh = layer->heights[idx];
      if (lh == 255)
        continue;

      for (int dir = 0; dir < 4; ++dir) {
        if (layer->cons[idx] & (1 << (dir + 4))) {
          const int *seg = &segs[dir * 4];
          const float ax = layer->bmin[0] + static_cast<float>(x + seg[0]) * cs;
          const float ay = layer->bmin[1] + static_cast<float>(lh + 2) * ch;
          const float az = layer->bmin[2] + static_cast<float>(y + seg[1]) * cs;
          const float bx = layer->bmin[0] + static_cast<float>(x + seg[2]) * cs;
          const float by = layer->bmin[1] + static_cast<float>(lh + 2) * ch;
          const float bz = layer->bmin[2] + static_cast<float>(y + seg[3]) * cs;
          dd->vertex(ax, ay, az, pcol);
          dd->vertex(bx, by, bz, pcol);
        }
      }
    }
  }
  dd->end();
}

void getContourCenter(const rcContour *cont, const float *orig, const float cs, const float ch, float *center) {
  center[0] = 0;
  center[1] = 0;
  center[2] = 0;
  if (!cont->nverts)
    return;
  for (int i = 0; i < cont->nverts; ++i) {
    const int *v = &cont->verts[i * 4];
    center[0] += static_cast<float>(v[0]);
    center[1] += static_cast<float>(v[1]);
    center[2] += static_cast<float>(v[2]);
  }
  const float s = 1.0f / static_cast<float>(cont->nverts);
  center[0] *= s * cs;
  center[1] *= s * ch;
  center[2] *= s * cs;
  center[0] += orig[0];
  center[1] += orig[1] + 4 * ch;
  center[2] += orig[2];
}

const rcContour *findContourFromSet(const rcContourSet &cset, const unsigned short reg) {
  for (int i = 0; i < cset.nconts; ++i) {
    if (cset.conts[i].reg == reg)
      return &cset.conts[i];
  }
  return nullptr;
}
} // namespace
void duDebugDrawTriMesh(duDebugDraw *dd, const float *verts, int /*nverts*/, const int *tris, const float *normals, const int ntris, const unsigned char *flags, const float texScale) {
  if (!dd)
    return;
  if (!verts)
    return;
  if (!tris)
    return;
  if (!normals)
    return;

  float uva[2];
  float uvb[2];
  float uvc[2];

  const uint32_t unwalkable = duRGBA(192, 128, 0, 255);

  dd->texture(true);

  dd->begin(DU_DRAW_TRIS);
  for (int i = 0; i < ntris * 3; i += 3) {
    const float *norm = &normals[i];
    uint32_t color;
    const auto a = static_cast<unsigned char>(220 * (2 + norm[0] + norm[1]) / 4);
    if (flags && !flags[i / 3])
      color = duLerpCol(duRGBA(a, a, a, 255), unwalkable, 64);
    else
      color = duRGBA(a, a, a, 255);

    const float *va = &verts[tris[i + 0] * 3];
    const float *vb = &verts[tris[i + 1] * 3];
    const float *vc = &verts[tris[i + 2] * 3];

    int ax = 0;
    if (rcAbs(norm[1]) > rcAbs(norm[ax]))
      ax = 1;
    if (rcAbs(norm[2]) > rcAbs(norm[ax]))
      ax = 2;
    ax = 1 << ax & 3; // +1 mod 3
    const int ay = 1 << ax & 3; // +1 mod 3

    uva[0] = va[ax] * texScale;
    uva[1] = va[ay] * texScale;
    uvb[0] = vb[ax] * texScale;
    uvb[1] = vb[ay] * texScale;
    uvc[0] = vc[ax] * texScale;
    uvc[1] = vc[ay] * texScale;

    dd->vertex(va, color, uva);
    dd->vertex(vb, color, uvb);
    dd->vertex(vc, color, uvc);
  }
  dd->end();
  dd->texture(false);
}

void duDebugDrawTriMeshSlope(duDebugDraw *dd, const float *verts, int /*nverts*/, const int *tris, const float *normals, const int ntris, const float walkableSlopeAngle, const float texScale) {
  if (!dd)
    return;
  if (!verts)
    return;
  if (!tris)
    return;
  if (!normals)
    return;

  const float walkableThr = std::cos(walkableSlopeAngle / 180.0f * DU_PI);

  float uva[2];
  float uvb[2];
  float uvc[2];

  dd->texture(true);

  const uint32_t unwalkable = duRGBA(192, 128, 0, 255);

  dd->begin(DU_DRAW_TRIS);
  for (int i = 0; i < ntris * 3; i += 3) {
    const float *norm = &normals[i];
    uint32_t color;
    const auto a = static_cast<unsigned char>(220 * (2 + norm[0] + norm[1]) / 4);
    if (norm[1] < walkableThr)
      color = duLerpCol(duRGBA(a, a, a, 255), unwalkable, 64);
    else
      color = duRGBA(a, a, a, 255);

    const float *va = &verts[tris[i + 0] * 3];
    const float *vb = &verts[tris[i + 1] * 3];
    const float *vc = &verts[tris[i + 2] * 3];

    int ax = 0;
    if (rcAbs(norm[1]) > rcAbs(norm[ax]))
      ax = 1;
    if (rcAbs(norm[2]) > rcAbs(norm[ax]))
      ax = 2;
    ax = 1 << ax & 3; // +1 mod 3
    const int ay = 1 << ax & 3; // +1 mod 3

    uva[0] = va[ax] * texScale;
    uva[1] = va[ay] * texScale;
    uvb[0] = vb[ax] * texScale;
    uvb[1] = vb[ay] * texScale;
    uvc[0] = vc[ax] * texScale;
    uvc[1] = vc[ay] * texScale;

    dd->vertex(va, color, uva);
    dd->vertex(vb, color, uvb);
    dd->vertex(vc, color, uvc);
  }
  dd->end();

  dd->texture(false);
}

void duDebugDrawHeightfieldSolid(duDebugDraw *dd, const rcHeightfield &hf) {
  if (!dd)
    return;

  const float *orig = hf.bmin;
  const float cs = hf.cs;
  const float ch = hf.ch;

  const int w = hf.width;
  const int h = hf.height;

  uint32_t fcol[6];
  duCalcBoxColors(fcol, duRGBA(255, 255, 255, 255), duRGBA(255, 255, 255, 255));

  dd->begin(DU_DRAW_QUADS);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const float fx = orig[0] + static_cast<float>(x) * cs;
      const float fz = orig[2] + static_cast<float>(y) * cs;
      const rcSpan *s = hf.spans[x + y * w];
      while (s) {
        duAppendBox(dd, fx, orig[1] + static_cast<float>(s->smin) * ch, fz, fx + cs, orig[1] + static_cast<float>(s->smax) * ch, fz + cs,
                    fcol);
        s = s->next;
      }
    }
  }
  dd->end();
}

void duDebugDrawHeightfieldWalkable(duDebugDraw *dd, const rcHeightfield &hf) {
  if (!dd)
    return;

  const float *orig = hf.bmin;
  const float cs = hf.cs;
  const float ch = hf.ch;

  const int w = hf.width;
  const int h = hf.height;

  uint32_t fcol[6];
  duCalcBoxColors(fcol, duRGBA(255, 255, 255, 255), duRGBA(217, 217, 217, 255));

  dd->begin(DU_DRAW_QUADS);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const float fx = orig[0] + static_cast<float>(x) * cs;
      const float fz = orig[2] + static_cast<float>(y) * cs;
      const rcSpan *s = hf.spans[x + y * w];
      while (s) {
        if (s->area == RC_WALKABLE_AREA)
          fcol[0] = duRGBA(64, 128, 160, 255);
        else if (s->area == RC_NULL_AREA)
          fcol[0] = duRGBA(64, 64, 64, 255);
        else
          fcol[0] = duMultCol(dd->areaToCol(s->area), 200);

        duAppendBox(dd, fx, orig[1] + static_cast<float>(s->smin) * ch, fz, fx + cs, orig[1] + static_cast<float>(s->smax) * ch, fz + cs, fcol);
        s = s->next;
      }
    }
  }

  dd->end();
}

void duDebugDrawCompactHeightfieldSolid(duDebugDraw *dd, const rcCompactHeightfield &chf) {
  if (!dd)
    return;

  const float cs = chf.cs;
  const float ch = chf.ch;

  dd->begin(DU_DRAW_QUADS);

  for (int y = 0; y < chf.height; ++y) {
    for (int x = 0; x < chf.width; ++x) {
      const float fx = chf.bmin[0] + static_cast<float>(x) * cs;
      const float fz = chf.bmin[2] + static_cast<float>(y) * cs;
      const auto &[index, count] = chf.cells[x + y * chf.width];

      for (unsigned i{index}, ni = index + count; i < ni; ++i) {
        const auto &[y, region, connection, height] = chf.spans[i];

        const unsigned char area = chf.areas[i];
        uint32_t color;
        if (area == RC_WALKABLE_AREA)
          color = duRGBA(0, 192, 255, 64);
        else if (area == RC_NULL_AREA)
          color = duRGBA(0, 0, 0, 64);
        else
          color = dd->areaToCol(area);

        const float fy = chf.bmin[1] + static_cast<float>(y + 1) * ch;
        dd->vertex(fx, fy, fz, color);
        dd->vertex(fx, fy, fz + cs, color);
        dd->vertex(fx + cs, fy, fz + cs, color);
        dd->vertex(fx + cs, fy, fz, color);
      }
    }
  }
  dd->end();
}

void duDebugDrawCompactHeightfieldRegions(duDebugDraw *dd, const rcCompactHeightfield &chf) {
  if (!dd)
    return;

  const float cs = chf.cs;
  const float ch = chf.ch;

  dd->begin(DU_DRAW_QUADS);

  for (int y = 0; y < chf.height; ++y) {
    for (int x = 0; x < chf.width; ++x) {
      const float fx = chf.bmin[0] + static_cast<float>(x) * cs;
      const float fz = chf.bmin[2] + static_cast<float>(y) * cs;
      const auto &[index, count] = chf.cells[x + y * chf.width];

      for (unsigned i = index, ni = index + count; i < ni; ++i) {
        const auto &[y, region, connection, height] = chf.spans[i];
        const float fy = chf.bmin[1] + static_cast<float>(y) * ch;
        uint32_t color;
        if (region)
          color = duIntToCol(region, 192);
        else
          color = duRGBA(0, 0, 0, 64);

        dd->vertex(fx, fy, fz, color);
        dd->vertex(fx, fy, fz + cs, color);
        dd->vertex(fx + cs, fy, fz + cs, color);
        dd->vertex(fx + cs, fy, fz, color);
      }
    }
  }

  dd->end();
}

void duDebugDrawCompactHeightfieldDistance(duDebugDraw *dd, const rcCompactHeightfield &chf) {
  if (!dd)
    return;
  if (!chf.dist)
    return;

  const float cs = chf.cs;
  const float ch = chf.ch;

  float maxd = chf.maxDistance;
  if (maxd < 1.0f)
    maxd = 1;
  const float dscale = 255.0f / maxd;

  dd->begin(DU_DRAW_QUADS);

  for (int y = 0; y < chf.height; ++y) {
    for (int x = 0; x < chf.width; ++x) {
      const float fx = chf.bmin[0] + static_cast<float>(x) * cs;
      const float fz = chf.bmin[2] + static_cast<float>(y) * cs;
      const auto &[index, count] = chf.cells[x + y * chf.width];

      for (uint32_t i{index}, ni{uint32_t(index + count)}; i < ni; ++i) {
        const auto &[y, region, connectoin, height] = chf.spans[i];
        const float fy = chf.bmin[1] + static_cast<float>(y + 1) * ch;
        const auto cd = static_cast<unsigned char>(static_cast<float>(chf.dist[i]) * dscale);
        const uint32_t color = duRGBA(cd, cd, cd, 255);
        dd->vertex(fx, fy, fz, color);
        dd->vertex(fx, fy, fz + cs, color);
        dd->vertex(fx + cs, fy, fz + cs, color);
        dd->vertex(fx + cs, fy, fz, color);
      }
    }
  }
  dd->end();
}

void duDebugDrawHeightfieldLayer(duDebugDraw *dd, const rcHeightfieldLayer &layer, const int idx) {
  const float cs = layer.cs;
  const float ch = layer.ch;
  const int w = layer.width;
  const int h = layer.height;

  const uint32_t color = duIntToCol(idx + 1, 255);

  // Layer bounds
  float bmin[3], bmax[3];
  bmin[0] = layer.bmin[0] + static_cast<float>(layer.minx) * cs;
  bmin[1] = layer.bmin[1];
  bmin[2] = layer.bmin[2] + static_cast<float>(layer.miny) * cs;
  bmax[0] = layer.bmin[0] + static_cast<float>(layer.maxx + 1) * cs;
  bmax[1] = layer.bmax[1];
  bmax[2] = layer.bmin[2] + static_cast<float>(layer.maxy + 1) * cs;
  duDebugDrawBoxWire(dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], duTransCol(color, 128), 2.0f);

  // Layer height
  dd->begin(DU_DRAW_QUADS);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int lidx = x + y * w;
      const int lh = layer.heights[lidx];
      if (h == 0xff)
        continue;
      const unsigned char area = layer.areas[lidx];

      uint32_t col;
      if (area == RC_WALKABLE_AREA)
        col = duLerpCol(color, duRGBA(0, 192, 255, 64), 32);
      else if (area == RC_NULL_AREA)
        col = duLerpCol(color, duRGBA(0, 0, 0, 64), 32);
      else
        col = duLerpCol(color, dd->areaToCol(area), 32);

      const float fx = layer.bmin[0] + static_cast<float>(x) * cs;
      const float fy = layer.bmin[1] + static_cast<float>(lh + 1) * ch;
      const float fz = layer.bmin[2] + static_cast<float>(y) * cs;

      dd->vertex(fx, fy, fz, col);
      dd->vertex(fx, fy, fz + cs, col);
      dd->vertex(fx + cs, fy, fz + cs, col);
      dd->vertex(fx + cs, fy, fz, col);
    }
  }
  dd->end();

  // Portals
  drawLayerPortals(dd, &layer);
}

void duDebugDrawHeightfieldLayers(duDebugDraw *dd, const rcHeightfieldLayerSet &lset) {
  if (!dd)
    return;
  for (int i = 0; i < lset.nlayers; ++i)
    duDebugDrawHeightfieldLayer(dd, lset.layers[i], i);
}

void duDebugDrawRegionConnections(duDebugDraw *dd, const rcContourSet &cset, const float alpha) {
  if (!dd)
    return;

  const float *orig = cset.bmin;
  const float cs = cset.cs;
  const float ch = cset.ch;

  // Draw centers
  float pos[3];

  const uint32_t color = duRGBA(0, 0, 0, 196);

  dd->begin(DU_DRAW_LINES, 2.0f);

  for (int i = 0; i < cset.nconts; ++i) {
    const rcContour *cont = &cset.conts[i];
    getContourCenter(cont, orig, cs, ch, pos);
    for (int j = 0; j < cont->nverts; ++j) {
      const int *v = &cont->verts[j * 4];
      if (v[3] == 0 || static_cast<unsigned short>(v[3]) < cont->reg)
        continue;
      if (const rcContour *cont2 = findContourFromSet(cset, static_cast<unsigned short>(v[3]))) {
        float pos2[3];
        getContourCenter(cont2, orig, cs, ch, pos2);
        duAppendArc(dd, pos[0], pos[1], pos[2], pos2[0], pos2[1], pos2[2], 0.25f, 0.6f, 0.6f, color);
      }
    }
  }

  dd->end();

  const auto a = static_cast<unsigned char>(alpha * 255.0f);

  dd->begin(DU_DRAW_POINTS, 7.0f);

  for (int i = 0; i < cset.nconts; ++i) {
    const rcContour *cont = &cset.conts[i];
    const uint32_t col = duDarkenCol(duIntToCol(cont->reg, a));
    getContourCenter(cont, orig, cs, ch, pos);
    dd->vertex(pos, col);
  }
  dd->end();
}

void duDebugDrawRawContours(duDebugDraw *dd, const rcContourSet &cset, const float alpha) {
  if (!dd)
    return;

  const float *orig = cset.bmin;
  const float cs = cset.cs;
  const float ch = cset.ch;

  const auto a = static_cast<unsigned char>(alpha * 255.0f);

  dd->begin(DU_DRAW_LINES, 2.0f);

  for (int i = 0; i < cset.nconts; ++i) {
    const auto &[vertices, vertexCount, rawVertices, rawVertexCount, region, area] = cset.conts[i];
    const uint32_t color = duIntToCol(region, a);

    for (int j = 0; j < rawVertexCount; ++j) {
      const int *v = &rawVertices[j * 4];
      const float fx = orig[0] + static_cast<float>(v[0]) * cs;
      const float fy = orig[1] + static_cast<float>(v[1] + 1 + (i & 1)) * ch;
      const float fz = orig[2] + static_cast<float>(v[2]) * cs;
      dd->vertex(fx, fy, fz, color);
      if (j > 0)
        dd->vertex(fx, fy, fz, color);
    }
    // Loop last segment.
    const int *v = &rawVertices[0];
    const float fx = orig[0] + static_cast<float>(v[0]) * cs;
    const float fy = orig[1] + static_cast<float>(v[1] + 1 + (i & 1)) * ch;
    const float fz = orig[2] + static_cast<float>(v[2]) * cs;
    dd->vertex(fx, fy, fz, color);
  }
  dd->end();

  dd->begin(DU_DRAW_POINTS, 2.0f);

  for (int i = 0; i < cset.nconts; ++i) {
    const auto &[vertices, vertexCount, rawVertices, rawVertexCount, region, area] = cset.conts[i];
    const uint32_t color = duDarkenCol(duIntToCol(region, a));

    for (int j = 0; j < rawVertexCount; ++j) {
      const int *v = &rawVertices[j * 4];
      float off = 0;
      uint32_t colv = color;
      if (v[3] & RC_BORDER_VERTEX) {
        colv = duRGBA(255, 255, 255, a);
        off = ch * 2;
      }

      const float fx = orig[0] + static_cast<float>(v[0]) * cs;
      const float fy = orig[1] + static_cast<float>(v[1] + 1 + (i & 1)) * ch + off;
      const float fz = orig[2] + static_cast<float>(v[2]) * cs;
      dd->vertex(fx, fy, fz, colv);
    }
  }
  dd->end();
}

void duDebugDrawContours(duDebugDraw *dd, const rcContourSet &cset, const float alpha) {
  if (!dd)
    return;

  const float *orig = cset.bmin;
  const float cs = cset.cs;
  const float ch = cset.ch;

  const auto a = static_cast<unsigned char>(alpha * 255.0f);

  dd->begin(DU_DRAW_LINES, 2.5f);

  for (int i = 0; i < cset.nconts; ++i) {
    const auto &[vertices, vertexCount, rawVertices, rawVertecCount, region, area] = cset.conts[i];
    if (!vertexCount)
      continue;
    const uint32_t color = duIntToCol(region, a);
    const uint32_t bcolor = duLerpCol(color, duRGBA(255, 255, 255, a), 128);
    for (int j = 0, k = vertexCount - 1; j < vertexCount; k = j++) {
      const int *va = &vertices[k * 4];
      const int *vb = &vertices[j * 4];
      const uint32_t col = va[3] & RC_AREA_BORDER ? bcolor : color;
      float fx = orig[0] + static_cast<float>(va[0]) * cs;
      float fy = orig[1] + static_cast<float>(va[1] + 1 + (i & 1)) * ch;
      float fz = orig[2] + static_cast<float>(va[2]) * cs;
      dd->vertex(fx, fy, fz, col);
      fx = orig[0] + static_cast<float>(vb[0]) * cs;
      fy = orig[1] + static_cast<float>(vb[1] + 1 + (i & 1)) * ch;
      fz = orig[2] + static_cast<float>(vb[2]) * cs;
      dd->vertex(fx, fy, fz, col);
    }
  }
  dd->end();

  dd->begin(DU_DRAW_POINTS, 3.0f);

  for (int i = 0; i < cset.nconts; ++i) {
    const auto &[vertices, vertexCount, rawVertices, rawVertexCount, region, area] = cset.conts[i];
    const uint32_t color = duDarkenCol(duIntToCol(region, a));
    for (int j = 0; j < vertexCount; ++j) {
      const int *v = &vertices[j * 4];
      float off = 0;
      uint32_t colv = color;
      if (v[3] & RC_BORDER_VERTEX) {
        colv = duRGBA(255, 255, 255, a);
        off = ch * 2;
      }

      const float fx = orig[0] + static_cast<float>(v[0]) * cs;
      const float fy = orig[1] + static_cast<float>(v[1] + 1 + (i & 1)) * ch + off;
      const float fz = orig[2] + static_cast<float>(v[2]) * cs;
      dd->vertex(fx, fy, fz, colv);
    }
  }
  dd->end();
}

void duDebugDrawPolyMesh(duDebugDraw *dd, const rcPolyMesh &mesh) {
  if (!dd)
    return;

  const int nvp = mesh.nvp;
  const float cs = mesh.cs;
  const float ch = mesh.ch;
  const float *orig = mesh.bmin;

  dd->begin(DU_DRAW_TRIS);

  for (int i = 0; i < mesh.npolys; ++i) {
    const unsigned short *p = &mesh.polys[i * nvp * 2];
    const unsigned char area = mesh.areas[i];

    uint32_t color;
    if (area == RC_WALKABLE_AREA)
      color = duRGBA(0, 192, 255, 64);
    else if (area == RC_NULL_AREA)
      color = duRGBA(0, 0, 0, 64);
    else
      color = dd->areaToCol(area);

    unsigned short vi[3];
    for (int j = 2; j < nvp; ++j) {
      if (p[j] == RC_MESH_NULL_IDX)
        break;
      vi[0] = p[0];
      vi[1] = p[j - 1];
      vi[2] = p[j];
      for (const unsigned short k : vi) {
        const unsigned short *v = &mesh.verts[k * 3];
        const float x = orig[0] + static_cast<float>(v[0]) * cs;
        const float y = orig[1] + static_cast<float>(v[1] + 1) * ch;
        const float z = orig[2] + static_cast<float>(v[2]) * cs;
        dd->vertex(x, y, z, color);
      }
    }
  }
  dd->end();

  // Draw neighbours edges
  const uint32_t coln = duRGBA(0, 48, 64, 32);
  dd->begin(DU_DRAW_LINES, 1.5f);
  for (int i = 0; i < mesh.npolys; ++i) {
    const unsigned short *p = &mesh.polys[i * nvp * 2];
    for (int j = 0; j < nvp; ++j) {
      if (p[j] == RC_MESH_NULL_IDX)
        break;
      if (p[nvp + j] & 0x8000)
        continue;

      const auto drawVertex{[orig, cs, ch, coln, &mesh, &dd](const int k) -> void {
        const unsigned short *v = &mesh.verts[k * 3];
        const float x = orig[0] + static_cast<float>(v[0]) * cs;
        const float y = orig[1] + static_cast<float>(v[1] + 1) * ch + 0.1f;
        const float z = orig[2] + static_cast<float>(v[2]) * cs;
        dd->vertex(x, y, z, coln);
      }};
      drawVertex(p[j]);
      drawVertex(p[j + 1 >= nvp || p[j + 1] == RC_MESH_NULL_IDX ? 0 : j + 1]);
    }
  }
  dd->end();

  // Draw boundary edges
  const uint32_t colb = duRGBA(0, 48, 64, 220);
  dd->begin(DU_DRAW_LINES, 2.5f);
  for (int i = 0; i < mesh.npolys; ++i) {
    const unsigned short *p = &mesh.polys[i * nvp * 2];
    for (int j = 0; j < nvp; ++j) {
      if (p[j] == RC_MESH_NULL_IDX)
        break;
      if ((p[nvp + j] & 0x8000) == 0)
        continue;
      const int nj = j + 1 >= nvp || p[j + 1] == RC_MESH_NULL_IDX ? 0 : j + 1;
      const int vi[2] = {p[j], p[nj]};

      uint32_t col = colb;
      if ((p[nvp + j] & 0xf) != 0xf)
        col = duRGBA(255, 255, 255, 128);
      for (const int k : vi) {
        const unsigned short *v = &mesh.verts[k * 3];
        const float x = orig[0] + static_cast<float>(v[0]) * cs;
        const float y = orig[1] + static_cast<float>(v[1] + 1) * ch + 0.1f;
        const float z = orig[2] + static_cast<float>(v[2]) * cs;
        dd->vertex(x, y, z, col);
      }
    }
  }
  dd->end();

  dd->begin(DU_DRAW_POINTS, 3.0f);
  const uint32_t colv = duRGBA(0, 0, 0, 220);
  for (int i = 0; i < mesh.nverts; ++i) {
    const unsigned short *v = &mesh.verts[i * 3];
    const float x = orig[0] + static_cast<float>(v[0]) * cs;
    const float y = orig[1] + static_cast<float>(v[1] + 1) * ch + 0.1f;
    const float z = orig[2] + static_cast<float>(v[2]) * cs;
    dd->vertex(x, y, z, colv);
  }
  dd->end();
}

void duDebugDrawPolyMeshDetail(duDebugDraw *dd, const rcPolyMeshDetail &dmesh) {
  if (!dd)
    return;

  dd->begin(DU_DRAW_TRIS);

  for (int i = 0; i < dmesh.nmeshes; ++i) {
    const uint32_t *m = &dmesh.meshes[i * 4];
    const uint32_t bverts = m[0];
    const uint32_t btris = m[2];
    const int ntris = static_cast<int>(m[3]);
    const float *verts = &dmesh.verts[bverts * 3];
    const unsigned char *tris = &dmesh.tris[btris * 4];

    const uint32_t color = duIntToCol(i, 192);

    for (int j = 0; j < ntris; ++j) {
      dd->vertex(&verts[tris[j * 4 + 0] * 3], color);
      dd->vertex(&verts[tris[j * 4 + 1] * 3], color);
      dd->vertex(&verts[tris[j * 4 + 2] * 3], color);
    }
  }
  dd->end();

  // Internal edges.
  dd->begin(DU_DRAW_LINES, 1.0f);
  const uint32_t coli = duRGBA(0, 0, 0, 64);
  for (int i = 0; i < dmesh.nmeshes; ++i) {
    const uint32_t *m = &dmesh.meshes[i * 4];
    const uint32_t bverts = m[0];
    const uint32_t btris = m[2];
    const int ntris = static_cast<int>(m[3]);
    const float *verts = &dmesh.verts[bverts * 3];
    const unsigned char *tris = &dmesh.tris[btris * 4];

    for (int j = 0; j < ntris; ++j) {
      const unsigned char *t = &tris[j * 4];
      for (int k = 0, kp = 2; k < 3; kp = k++) {
        if ((t[3] >> kp * 2 & 0x3) == 0) {
          // Internal edge
          if (t[kp] < t[k]) {
            dd->vertex(&verts[t[kp] * 3], coli);
            dd->vertex(&verts[t[k] * 3], coli);
          }
        }
      }
    }
  }
  dd->end();

  // External edges.
  dd->begin(DU_DRAW_LINES, 2.0f);
  const uint32_t cole = duRGBA(0, 0, 0, 64);
  for (int i = 0; i < dmesh.nmeshes; ++i) {
    const uint32_t *m = &dmesh.meshes[i * 4];
    const uint32_t bverts = m[0];
    const uint32_t btris = m[2];
    const int ntris = static_cast<int>(m[3]);
    const float *verts = &dmesh.verts[bverts * 3];
    const unsigned char *tris = &dmesh.tris[btris * 4];

    for (int j = 0; j < ntris; ++j) {
      const unsigned char *t = &tris[j * 4];
      for (int k = 0, kp = 2; k < 3; kp = k++) {
        if ((t[3] >> kp * 2 & 0x3) != 0) {
          // Ext edge
          dd->vertex(&verts[t[kp] * 3], cole);
          dd->vertex(&verts[t[k] * 3], cole);
        }
      }
    }
  }
  dd->end();

  dd->begin(DU_DRAW_POINTS, 3.0f);
  const uint32_t colv = duRGBA(0, 0, 0, 64);
  for (int i = 0; i < dmesh.nmeshes; ++i) {
    const uint32_t *m = &dmesh.meshes[i * 4];
    const uint32_t bverts = m[0];
    const int nverts = static_cast<int>(m[1]);
    const float *verts = &dmesh.verts[bverts * 3];
    for (int j = 0; j < nverts; ++j)
      dd->vertex(&verts[j * 3], colv);
  }
  dd->end();
}

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

class Sample_TileMesh final : public Sample {
protected:
  bool m_keepInterResults;
  bool m_buildAll;
  float m_totalBuildTimeMs;

  unsigned char *m_triareas;
  rcHeightfield *m_solid;
  rcCompactHeightfield *m_chf;
  rcContourSet *m_cset;
  rcPolyMesh *m_pmesh;
  rcPolyMeshDetail *m_dmesh;
  rcConfig m_cfg{};

  enum DrawMode {
    DRAWMODE_NAVMESH,
    DRAWMODE_NAVMESH_TRANS,
    DRAWMODE_NAVMESH_BVTREE,
    DRAWMODE_NAVMESH_NODES,
    DRAWMODE_NAVMESH_PORTALS,
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
    DRAWMODE_POLYMESH_DETAIL,
    MAX_DRAWMODE
  };

  DrawMode m_drawMode;

  int m_maxTiles;
  int m_maxPolysPerTile;
  float m_tileSize;

  uint32_t m_tileCol;
  float m_lastBuiltTileBmin[3]{};
  float m_lastBuiltTileBmax[3]{};
  float m_tileBuildTime;
  float m_tileMemUsage;
  int m_tileTriCount;

  unsigned char *buildTileMesh(int tx, int ty, const float *bmin, const float *bmax, int &dataSize);

  void cleanup();

  void saveAll(const char *path, const dtNavMesh *mesh);
  dtNavMesh *loadAll(const char *path);

public:
  Sample_TileMesh();
  ~Sample_TileMesh() override;

  void handleSettings() override;
  void handleTools() override;
  void handleDebugMode() override;
  void handleRender() override;
  void handleRenderOverlay(double *proj, double *model, int *view) override;
  void handleMeshChanged(InputGeom *geom) override;
  bool handleBuild() override;
  void collectSettings(BuildSettings &settings) override;

  void getTilePos(const float *pos, int &tx, int &ty) const;

  void buildTile(const float *pos);
  void removeTile(const float *pos);
  void buildAllTiles();
  void removeAllTiles() const;

private:
  // Explicitly disabled copy constructor and copy assignment operator.
  Sample_TileMesh(const Sample_TileMesh &);
  Sample_TileMesh &operator=(const Sample_TileMesh &);
};

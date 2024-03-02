#pragma once
#include <DetourStatus.h>

#include <cstdint>

typedef uint32_t dtObstacleRef;
typedef uint32_t dtCompressedTileRef;

/// Flags for addTile
enum dtCompressedTileFlags {
  DT_COMPRESSEDTILE_FREE_DATA = 0x01 ///< Navmesh owns the tile memory and should free it.
};

struct dtCompressedTile {
  uint32_t salt; ///< Counter describing modifications to the tile.
  struct dtTileCacheLayerHeader *header;
  uint8_t *compressed;
  int compressedSize;
  uint8_t *data;
  int dataSize;
  uint32_t flags;
  dtCompressedTile *next;
};

enum ObstacleState {
  DT_OBSTACLE_EMPTY,
  DT_OBSTACLE_PROCESSING,
  DT_OBSTACLE_PROCESSED,
  DT_OBSTACLE_REMOVING
};

enum ObstacleType {
  DT_OBSTACLE_CYLINDER,
  DT_OBSTACLE_BOX,         // AABB
  DT_OBSTACLE_ORIENTED_BOX // OBB
};

struct dtObstacleCylinder {
  float pos[3];
  float radius;
  float height;
};

struct dtObstacleBox {
  float bmin[3];
  float bmax[3];
};

struct dtObstacleOrientedBox {
  float center[3];
  float halfExtents[3];
  float rotAux[2]; //{ cos(0.5f*angle)*sin(-0.5f*angle); cos(0.5f*angle)*cos(0.5f*angle) - 0.5 }
};

static constexpr int DT_MAX_TOUCHED_TILES = 8;
struct dtTileCacheObstacle {
  union {
    dtObstacleCylinder cylinder;
    dtObstacleBox box;
    dtObstacleOrientedBox orientedBox;
  };

  dtCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
  dtCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
  uint16_t salt;
  uint8_t type;
  uint8_t state;
  uint8_t ntouched;
  uint8_t npending;
  dtTileCacheObstacle *next;
};

struct dtTileCacheParams {
  float orig[3];
  float cs, ch;
  int width, height;
  float walkableHeight;
  float walkableRadius;
  float walkableClimb;
  float maxSimplificationError;
  int maxTiles;
  int maxObstacles;
};

struct dtTileCacheMeshProcess {
  virtual ~dtTileCacheMeshProcess();
  virtual void process(struct dtNavMeshCreateParams *params, uint8_t *polyAreas, uint16_t *polyFlags) = 0;
};

class dtTileCache {
public:
  dtTileCache();
  ~dtTileCache();

  struct dtTileCacheAlloc *getAlloc() const { return m_talloc; }
  struct dtTileCacheCompressor *getCompressor() const { return m_tcomp; }
  const dtTileCacheParams *getParams() const { return &m_params; }

  int getTileCount() const { return m_params.maxTiles; }
  const dtCompressedTile *getTile(const int i) const { return &m_tiles[i]; }

  int getObstacleCount() const { return m_params.maxObstacles; }
  const dtTileCacheObstacle *getObstacle(const int i) const { return &m_obstacles[i]; }

  const dtTileCacheObstacle *getObstacleByRef(dtObstacleRef ref) const;

  dtObstacleRef getObstacleRef(const dtTileCacheObstacle *ob) const;

  dtStatus init(const dtTileCacheParams *params,
                dtTileCacheAlloc *talloc,
                dtTileCacheCompressor *tcomp,
                dtTileCacheMeshProcess *tmproc);

  int getTilesAt(int tx, int ty, dtCompressedTileRef *tiles, int maxTiles) const;

  dtCompressedTile *getTileAt(int tx, int ty, int tlayer) const;
  dtCompressedTileRef getTileRef(const dtCompressedTile *tile) const;
  const dtCompressedTile *getTileByRef(dtCompressedTileRef ref) const;

  dtStatus addTile(uint8_t *data, int dataSize, uint8_t flags, dtCompressedTileRef *result);

  dtStatus removeTile(dtCompressedTileRef ref, uint8_t **data, int *dataSize);

  // Cylinder obstacle.
  dtStatus addObstacle(const float *pos, float radius, float height, dtObstacleRef *result);

  // Aabb obstacle.
  dtStatus addBoxObstacle(const float *bmin, const float *bmax, dtObstacleRef *result);

  // Box obstacle: can be rotated in Y.
  dtStatus addBoxObstacle(const float *center, const float *halfExtents, float yRadians, dtObstacleRef *result);

  dtStatus removeObstacle(dtObstacleRef ref);

  dtStatus queryTiles(const float *bmin, const float *bmax,
                      dtCompressedTileRef *results, int *resultCount, int maxResults) const;

  /// Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
  ///  @param[in]		dt			The time step size. Currently not used.
  ///  @param[in]		navmesh		The mesh to affect when rebuilding tiles.
  ///  @param[out]	upToDate	Whether the tile cache is fully up to date with obstacle requests and tile rebuilds.
  ///  							If the tile cache is up to date another (immediate) call to update will have no effect;
  ///  							otherwise another call will continue processing obstacle requests and tile rebuilds.
  dtStatus update(float dt, class dtNavMesh *navmesh, bool *upToDate = nullptr);

  dtStatus buildNavMeshTilesAt(int tx, int ty, dtNavMesh *navmesh) const;

  dtStatus buildNavMeshTile(dtCompressedTileRef ref, dtNavMesh *navmesh) const;

  void calcTightTileBounds(const dtTileCacheLayerHeader *header, float *bmin, float *bmax) const;

  static void getObstacleBounds(const dtTileCacheObstacle *ob, float *bmin, float *bmax);

  /// Encodes a tile id.
  dtCompressedTileRef encodeTileId(const uint32_t salt, const uint32_t it) const {
    return salt << m_tileBits | it;
  }

  /// Decodes a tile salt.
  uint32_t decodeTileIdSalt(const dtCompressedTileRef ref) const {
    const dtCompressedTileRef saltMask = (static_cast<dtCompressedTileRef>(1) << m_saltBits) - 1;
    return ref >> m_tileBits & saltMask;
  }

  /// Decodes a tile id.
  uint32_t decodeTileIdTile(const dtCompressedTileRef ref) const {
    const dtCompressedTileRef tileMask = (static_cast<dtCompressedTileRef>(1) << m_tileBits) - 1;
    return ref & tileMask;
  }

  /// Encodes an obstacle id.
  static dtObstacleRef encodeObstacleId(const uint32_t salt, const uint32_t it) {
    return salt << 16 | it;
  }

  /// Decodes an obstacle salt.
  static uint32_t decodeObstacleIdSalt(const dtObstacleRef ref) {
    constexpr dtObstacleRef saltMask = (static_cast<dtObstacleRef>(1) << 16) - 1;
    return ref >> 16 & saltMask;
  }

  /// Decodes an obstacle id.
  static uint32_t decodeObstacleIdObstacle(const dtObstacleRef ref) {
    constexpr dtObstacleRef tileMask = (static_cast<dtObstacleRef>(1) << 16) - 1;
    return ref & tileMask;
  }

  // Explicitly disabled copy constructor and copy assignment operator.
  dtTileCache(const dtTileCache &) = delete;
  dtTileCache &operator=(const dtTileCache &) = delete;

private:
  enum ObstacleRequestAction {
    REQUEST_ADD,
    REQUEST_REMOVE
  };

  struct ObstacleRequest {
    int action{};
    dtObstacleRef ref{};
  };

  int m_tileLutSize{}; ///< Tile hash lookup size (must be pot).
  int m_tileLutMask{}; ///< Tile hash lookup mask.

  dtCompressedTile **m_posLookup{};   ///< Tile hash lookup.
  dtCompressedTile *m_nextFreeTile{}; ///< Freelist of tiles.
  dtCompressedTile *m_tiles{};        ///< List of tiles.

  uint32_t m_saltBits{}; ///< Number of salt bits in the tile ID.
  uint32_t m_tileBits{}; ///< Number of tile bits in the tile ID.

  dtTileCacheParams m_params{};

  dtTileCacheAlloc *m_talloc{};
  dtTileCacheCompressor *m_tcomp{};
  dtTileCacheMeshProcess *m_tmproc{};

  dtTileCacheObstacle *m_obstacles{};
  dtTileCacheObstacle *m_nextFreeObstacle{};

  static constexpr int MAX_REQUESTS = 64;
  ObstacleRequest m_reqs[MAX_REQUESTS]{};
  int m_nreqs{};

  static constexpr int MAX_UPDATE = 64;
  dtCompressedTileRef m_update[MAX_UPDATE]{};
  int m_nupdate{};
};

dtTileCache *dtAllocTileCache();
void dtFreeTileCache(dtTileCache *tc);

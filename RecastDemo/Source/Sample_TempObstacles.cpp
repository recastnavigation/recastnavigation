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

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cfloat>
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_TempObstacles.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "NavMeshTesterTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "RecastAlloc.h"
#include "fastlz.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif


// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
static constexpr int EXPECTED_LAYERS_PER_TILE = 4;


static bool isectSegAABB(const float* sp, const float* sq,
                         const float* amin, const float* amax,
                         float& tmin, float& tmax)
{
    static constexpr float EPS = 1e-6f;

    float d[3];
    rcVsub(d, sq, sp);
    tmin = 0; // set to -FLT_MAX to get first hit on line
    tmax = FLT_MAX; // set to max distance ray can travel (for segment)

    // For all three slabs
    for (int i = 0; i < 3; i++)
    {
        if (std::abs(d[i]) < EPS)
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

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
    const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
    const int gridSize = gridWidth * gridHeight;
    return headerSize + gridSize * 4;
}


struct FastLZCompressor final : dtTileCacheCompressor
{
    ~FastLZCompressor() override = default;

    int maxCompressedSize(const int bufferSize) override
    {
        return static_cast<int>(static_cast<float>(bufferSize) * 1.05f);
    }

    dtStatus compress(const unsigned char* buffer, const int bufferSize,
                      unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize) override
    {
        *compressedSize = fastlz_compress(buffer, bufferSize, compressed);
        return DT_SUCCESS;
    }

    dtStatus decompress(const unsigned char* compressed, const int compressedSize,
                        unsigned char* buffer, const int maxBufferSize, int* bufferSize) override
    {
        *bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
        return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
    }
};

struct LinearAllocator final : dtTileCacheAlloc
{
    unsigned char* buffer;
    size_t capacity;
    size_t top;
    size_t high;

    explicit LinearAllocator(const size_t cap) : buffer(nullptr), capacity(0), top(0), high(0)
    {
        resize(cap);
    }

    ~LinearAllocator() override;

    void resize(const size_t cap)
    {
        if (buffer) dtFree(buffer);
        buffer = static_cast<unsigned char*>(dtAlloc(cap, DT_ALLOC_PERM));
        capacity = cap;
    }

    void reset() override
    {
        high = dtMax(high, top);
        top = 0;
    }

    void* alloc(const size_t size) override
    {
        if (!buffer)
            return nullptr;
        if (top + size > capacity)
            return nullptr;
        unsigned char* mem = &buffer[top];
        top += size;
        return mem;
    }

    void free(void* /*ptr*/) override
    {
        // Empty
    }
};

LinearAllocator::~LinearAllocator()
{
    // Defined out of line to fix the weak v-tables warning
    dtFree(buffer);
}

struct MeshProcess final : dtTileCacheMeshProcess
{
    InputGeom* m_geom;

    MeshProcess() : m_geom(nullptr)
    {
    }

    ~MeshProcess() override = default;

    void init(InputGeom* geom)
    {
        m_geom = geom;
    }

    void process(dtNavMeshCreateParams* params,
                 unsigned char* polyAreas, unsigned short* polyFlags) override
    {
        // Update poly flags from areas.
        for (int i = 0; i < params->polyCount; ++i)
        {
            if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
                polyAreas[i] = SAMPLE_POLYAREA_GROUND;

            if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
                polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
                polyAreas[i] == SAMPLE_POLYAREA_ROAD)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
            }
            else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
            }
            else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
            {
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
            }
        }

        // Pass in off-mesh connections.
        if (m_geom)
        {
            params->offMeshConVerts = m_geom->getOffMeshConnectionVerts();
            params->offMeshConRad = m_geom->getOffMeshConnectionRads();
            params->offMeshConDir = m_geom->getOffMeshConnectionDirs();
            params->offMeshConAreas = m_geom->getOffMeshConnectionAreas();
            params->offMeshConFlags = m_geom->getOffMeshConnectionFlags();
            params->offMeshConUserID = m_geom->getOffMeshConnectionId();
            params->offMeshConCount = m_geom->getOffMeshConnectionCount();
        }
    }
};

static constexpr int MAX_LAYERS = 32;

struct TileCacheData
{
    unsigned char* data;
    int dataSize;
};

struct RasterizationContext
{
    RasterizationContext() :
        solid(nullptr),
        triareas(nullptr),
        lset(nullptr),
        chf(nullptr),
        ntiles(0)
    {
        memset(tiles, 0, sizeof(TileCacheData) * MAX_LAYERS);
    }

    ~RasterizationContext()
    {
        rcFreeHeightField(solid);
        delete [] triareas;
        rcFreeHeightfieldLayerSet(lset);
        rcFreeCompactHeightfield(chf);
        for (auto & tile : tiles)
        {
            dtFree(tile.data);
            tile.data = nullptr;
        }
    }

    rcHeightfield* solid;
    unsigned char* triareas;
    rcHeightfieldLayerSet* lset;
    rcCompactHeightfield* chf;
    TileCacheData tiles[MAX_LAYERS]{};
    int ntiles;
};

int Sample_TempObstacles::rasterizeTileLayers(
    const int tx, const int ty,
    const rcConfig& cfg,
    TileCacheData* tiles,
    const int maxTiles) const
{
    if (!m_geom || !m_geom->getMesh() || !m_geom->getChunkyMesh())
    {
        m_ctx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified.");
        return 0;
    }

    FastLZCompressor comp;
    RasterizationContext rc;

    const float* verts = m_geom->getMesh()->getVerts();
    const int nverts = m_geom->getMesh()->getVertCount();
    const rcChunkyTriMesh* chunkyMesh = m_geom->getChunkyMesh();

    // Tile bounds.
    const float tcs = static_cast<float>(cfg.tileSize) * cfg.cs;

    rcConfig tcfg{};
    memcpy(&tcfg, &cfg, sizeof(tcfg));

    tcfg.bmin[0] = cfg.bmin[0] + static_cast<float>(tx) * tcs;
    tcfg.bmin[1] = cfg.bmin[1];
    tcfg.bmin[2] = cfg.bmin[2] + static_cast<float>(ty) * tcs;
    tcfg.bmax[0] = cfg.bmin[0] + static_cast<float>(tx + 1) * tcs;
    tcfg.bmax[1] = cfg.bmax[1];
    tcfg.bmax[2] = cfg.bmin[2] + static_cast<float>(ty + 1) * tcs;
    tcfg.bmin[0] -= static_cast<float>(tcfg.borderSize) * tcfg.cs;
    tcfg.bmin[2] -= static_cast<float>(tcfg.borderSize) * tcfg.cs;
    tcfg.bmax[0] += static_cast<float>(tcfg.borderSize) * tcfg.cs;
    tcfg.bmax[2] += static_cast<float>(tcfg.borderSize) * tcfg.cs;

    // Allocate voxel heightfield where we rasterize our input data to.
    rc.solid = rcAllocHeightfield();
    if (!rc.solid)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
        return 0;
    }
    if (!rcCreateHeightfield(m_ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
        return 0;
    }

    // Allocate array that can hold triangle flags.
    // If you have multiple meshes you need to process, allocate
    // and array which can hold the max number of triangles you need to process.
    rc.triareas = new (std::nothrow) unsigned char[chunkyMesh->maxTrisPerChunk];
    if (!rc.triareas)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk);
        return 0;
    }

    float tbmin[2], tbmax[2];
    tbmin[0] = tcfg.bmin[0];
    tbmin[1] = tcfg.bmin[2];
    tbmax[0] = tcfg.bmax[0];
    tbmax[1] = tcfg.bmax[2];
    int cid[512]; // TODO: Make grow when returning too many items.
    const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
    if (!ncid)
    {
        return 0; // empty
    }

    for (int i = 0; i < ncid; ++i)
    {
        const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
        const int* tris = &chunkyMesh->tris[node.i * 3];
        const int ntris = node.n;

        memset(rc.triareas, 0, ntris * sizeof(unsigned char));
        rcMarkWalkableTriangles(m_ctx, tcfg.walkableSlopeAngle,
                                verts, nverts, tris, ntris, rc.triareas);

        if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb))
            return 0;
    }

    // Once all geometry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    if (m_filterLowHangingObstacles)
        rcFilterLowHangingWalkableObstacles(m_ctx, tcfg.walkableClimb, *rc.solid);
    if (m_filterLedgeSpans)
        rcFilterLedgeSpans(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
    if (m_filterWalkableLowHeightSpans)
        rcFilterWalkableLowHeightSpans(m_ctx, tcfg.walkableHeight, *rc.solid);


    rc.chf = rcAllocCompactHeightfield();
    if (!rc.chf)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
        return 0;
    }
    if (!rcBuildCompactHeightfield(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
        return 0;
    }

    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(m_ctx, tcfg.walkableRadius, *rc.chf))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
        return 0;
    }

    // (Optional) Mark areas.
    const ConvexVolume* vols = m_geom->getConvexVolumes();
    for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
    {
        rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts,
                             vols[i].hmin, vols[i].hmax,
                             static_cast<unsigned char>(vols[i].area), *rc.chf);
    }

    rc.lset = rcAllocHeightfieldLayerSet();
    if (!rc.lset)
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'.");
        return 0;
    }
    if (!rcBuildHeightfieldLayers(m_ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
    {
        m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
        return 0;
    }

    rc.ntiles = 0;
    for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
    {
        TileCacheData* tile = &rc.tiles[rc.ntiles++];
        const rcHeightfieldLayer* layer = &rc.lset->layers[i];

        // Store header
        dtTileCacheLayerHeader header{};
        header.magic = DT_TILECACHE_MAGIC;
        header.version = DT_TILECACHE_VERSION;

        // Tile layer location in the navmesh.
        header.tx = tx;
        header.ty = ty;
        header.tlayer = i;
        dtVcopy(header.bmin, layer->bmin);
        dtVcopy(header.bmax, layer->bmax);

        // Tile info.
        header.width = static_cast<unsigned char>(layer->width);
        header.height = static_cast<unsigned char>(layer->height);
        header.minx = static_cast<unsigned char>(layer->minx);
        header.maxx = static_cast<unsigned char>(layer->maxx);
        header.miny = static_cast<unsigned char>(layer->miny);
        header.maxy = static_cast<unsigned char>(layer->maxy);
        header.hmin = static_cast<unsigned short>(layer->hmin);
        header.hmax = static_cast<unsigned short>(layer->hmax);

        const dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
                                                      &tile->data, &tile->dataSize);
        if (dtStatusFailed(status))
        {
            return 0;
        }
    }

    // Transfer ownsership of tile data from build context to the caller.
    int n = 0;
    for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
    {
        tiles[n++] = rc.tiles[i];
        rc.tiles[i].data = nullptr;
        rc.tiles[i].dataSize = 0;
    }

    return n;
}


void drawTiles(duDebugDraw* dd, const dtTileCache* tc)
{
    float bmin[3], bmax[3];

    for (int i = 0; i < tc->getTileCount(); ++i)
    {
        unsigned int fcol[6];
        const dtCompressedTile* tile = tc->getTile(i);
        if (!tile->header) continue;

        tc->calcTightTileBounds(tile->header, bmin, bmax);

        const unsigned int col = duIntToCol(i, 64);
        duCalcBoxColors(fcol, col, col);
        duDebugDrawBox(dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], fcol);
    }

    for (int i = 0; i < tc->getTileCount(); ++i)
    {
        const dtCompressedTile* tile = tc->getTile(i);
        if (!tile->header) continue;

        tc->calcTightTileBounds(tile->header, bmin, bmax);

        const unsigned int col = duIntToCol(i, 255);
        const float pad = tc->getParams()->cs * 0.1f;
        duDebugDrawBoxWire(dd, bmin[0] - pad, bmin[1] - pad, bmin[2] - pad,
                           bmax[0] + pad, bmax[1] + pad, bmax[2] + pad, col, 2.0f);
    }
}

enum DrawDetailType
{
    DRAWDETAIL_AREAS,
    DRAWDETAIL_REGIONS,
    DRAWDETAIL_CONTOURS,
    DRAWDETAIL_MESH
};

void drawDetail(duDebugDraw* dd, const dtTileCache* tc, const int tx, const int ty, const int type)
{
    struct TileCacheBuildContext
    {
        explicit TileCacheBuildContext(dtTileCacheAlloc* a) : layer(nullptr), lcset(nullptr), lmesh(nullptr), alloc(a)
        {
        }

        ~TileCacheBuildContext() { purge(); }

        void purge()
        {
            dtFreeTileCacheLayer(alloc, layer);
            layer = nullptr;
            dtFreeTileCacheContourSet(alloc, lcset);
            lcset = nullptr;
            dtFreeTileCachePolyMesh(alloc, lmesh);
            lmesh = nullptr;
        }

        dtTileCacheLayer* layer;
        dtTileCacheContourSet* lcset;
        dtTileCachePolyMesh* lmesh;
        dtTileCacheAlloc* alloc;
    };

    dtCompressedTileRef tiles[MAX_LAYERS];
    const int ntiles = tc->getTilesAt(tx, ty, tiles, MAX_LAYERS);

    dtTileCacheAlloc* talloc = tc->getAlloc();
    dtTileCacheCompressor* tcomp = tc->getCompressor();
    const dtTileCacheParams* params = tc->getParams();

    for (int i = 0; i < ntiles; ++i)
    {
        const dtCompressedTile* tile = tc->getTileByRef(tiles[i]);

        talloc->reset();

        TileCacheBuildContext bc(talloc);
        const int walkableClimbVx = static_cast<int>(params->walkableClimb / params->ch);

        // Decompress tile layer data.
        dtStatus status = dtDecompressTileCacheLayer(talloc, tcomp, tile->data, tile->dataSize, &bc.layer);
        if (dtStatusFailed(status))
            return;
        if (type == DRAWDETAIL_AREAS)
        {
            duDebugDrawTileCacheLayerAreas(dd, *bc.layer, params->cs, params->ch);
            continue;
        }

        // Build navmesh
        status = dtBuildTileCacheRegions(talloc, *bc.layer, walkableClimbVx);
        if (dtStatusFailed(status))
            return;
        if (type == DRAWDETAIL_REGIONS)
        {
            duDebugDrawTileCacheLayerRegions(dd, *bc.layer, params->cs, params->ch);
            continue;
        }

        bc.lcset = dtAllocTileCacheContourSet(talloc);
        if (!bc.lcset)
            return;
        status = dtBuildTileCacheContours(talloc, *bc.layer, walkableClimbVx,
                                          params->maxSimplificationError, *bc.lcset);
        if (dtStatusFailed(status))
            return;
        if (type == DRAWDETAIL_CONTOURS)
        {
            duDebugDrawTileCacheContours(dd, *bc.lcset, tile->header->bmin, params->cs, params->ch);
            continue;
        }

        bc.lmesh = dtAllocTileCachePolyMesh(talloc);
        if (!bc.lmesh)
            return;
        status = dtBuildTileCachePolyMesh(talloc, *bc.lcset, *bc.lmesh);
        if (dtStatusFailed(status))
            return;

        if (type == DRAWDETAIL_MESH)
        {
            duDebugDrawTileCachePolyMesh(dd, *bc.lmesh, tile->header->bmin, params->cs, params->ch);
        }
    }
}


void drawDetailOverlay(const dtTileCache* tc, const int tx, const int ty, const double* proj, const double* model,
                       const int* view)
{
    dtCompressedTileRef tiles[MAX_LAYERS];
    const int ntiles = tc->getTilesAt(tx, ty, tiles, MAX_LAYERS);
    if (!ntiles)
        return;

    const int rawSize = calcLayerBufferSize(tc->getParams()->width, tc->getParams()->height);

    for (int i = 0; i < ntiles; ++i)
    {
        const dtCompressedTile* tile = tc->getTileByRef(tiles[i]);

        float pos[3];
        pos[0] = (tile->header->bmin[0] + tile->header->bmax[0]) / 2.0f;
        pos[1] = tile->header->bmin[1];
        pos[2] = (tile->header->bmin[2] + tile->header->bmax[2]) / 2.0f;

        GLdouble x, y, z;
        if (gluProject(pos[0], pos[1], pos[2],
                       model, proj, view, &x, &y, &z))
        {
            char text[128];
            sprintf_s(text, "(%d,%d)/%d", tile->header->tx, tile->header->ty, tile->header->tlayer);
            imguiDrawText(static_cast<int>(x), static_cast<int>(y) - 25, IMGUI_ALIGN_CENTER, text,
                          imguiRGBA(0, 0, 0, 220));
            sprintf_s(text, "Compressed: %.1f kB", static_cast<float>(tile->dataSize) / 1024.0f);
            imguiDrawText(static_cast<int>(x), static_cast<int>(y) - 45, IMGUI_ALIGN_CENTER, text,
                          imguiRGBA(0, 0, 0, 128));
            sprintf_s(text, "Raw:%.1fkB", static_cast<float>(rawSize) / 1024.0f);
            imguiDrawText(static_cast<int>(x), static_cast<int>(y) - 65, IMGUI_ALIGN_CENTER, text,
                          imguiRGBA(0, 0, 0, 128));
        }
    }
}

dtObstacleRef hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq)
{
    float tmin = FLT_MAX;
    const dtTileCacheObstacle* obmin = nullptr;
    for (int i = 0; i < tc->getObstacleCount(); ++i)
    {
        const dtTileCacheObstacle* ob = tc->getObstacle(i);
        if (ob->state == DT_OBSTACLE_EMPTY)
            continue;

        float bmin[3], bmax[3], t0, t1;
        dtTileCache::getObstacleBounds(ob, bmin, bmax);

        if (isectSegAABB(sp, sq, bmin, bmax, t0, t1))
        {
            if (t0 < tmin)
            {
                tmin = t0;
                obmin = ob;
            }
        }
    }
    return tc->getObstacleRef(obmin);
}

void drawObstacles(duDebugDraw* dd, const dtTileCache* tc)
{
    // Draw obstacles
    for (int i = 0; i < tc->getObstacleCount(); ++i)
    {
        const dtTileCacheObstacle* ob = tc->getObstacle(i);
        if (ob->state == DT_OBSTACLE_EMPTY) continue;
        float bmin[3], bmax[3];
        dtTileCache::getObstacleBounds(ob, bmin, bmax);

        unsigned int col = 0;
        if (ob->state == DT_OBSTACLE_PROCESSING)
            col = duRGBA(255, 255, 0, 128);
        else if (ob->state == DT_OBSTACLE_PROCESSED)
            col = duRGBA(255, 192, 0, 192);
        else if (ob->state == DT_OBSTACLE_REMOVING)
            col = duRGBA(220, 0, 0, 128);

        duDebugDrawCylinder(dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], col);
        duDebugDrawCylinderWire(dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], duDarkenCol(col), 2);
    }
}

class TempObstacleHilightTool final : public SampleTool
{
    Sample_TempObstacles* m_sample;
    float m_hitPos[3]{};
    bool m_hitPosSet;
    int m_drawType;

public:
    TempObstacleHilightTool() :
        m_sample(nullptr),
        m_hitPosSet(false),
        m_drawType(DRAWDETAIL_AREAS)
    {
        m_hitPos[0] = m_hitPos[1] = m_hitPos[2] = 0;
    }

    ~TempObstacleHilightTool() override = default;

    int type() override { return TOOL_TILE_HIGHLIGHT; }

    void init(Sample* sample) override
    {
        m_sample = dynamic_cast<Sample_TempObstacles*>(sample);
    }

    void reset() override
    {
    }

    void handleMenu() override
    {
        imguiLabel("Highlight Tile Cache");
        imguiValue("Click LMB to highlight a tile.");
        imguiSeparator();
        if (imguiCheck("Draw Areas", m_drawType == DRAWDETAIL_AREAS))
            m_drawType = DRAWDETAIL_AREAS;
        if (imguiCheck("Draw Regions", m_drawType == DRAWDETAIL_REGIONS))
            m_drawType = DRAWDETAIL_REGIONS;
        if (imguiCheck("Draw Contours", m_drawType == DRAWDETAIL_CONTOURS))
            m_drawType = DRAWDETAIL_CONTOURS;
        if (imguiCheck("Draw Mesh", m_drawType == DRAWDETAIL_MESH))
            m_drawType = DRAWDETAIL_MESH;
    }

    void handleClick(const float* /*s*/, const float* p, bool /*shift*/) override
    {
        m_hitPosSet = true;
        rcVcopy(m_hitPos, p);
    }

    void handleToggle() override
    {
    }

    void handleStep() override
    {
    }

    void handleUpdate(const float /*dt*/) override
    {
    }

    void handleRender() override
    {
        if (m_hitPosSet && m_sample)
        {
            const float s = m_sample->getAgentRadius();
            glColor4ub(0, 0, 0, 128);
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glVertex3f(m_hitPos[0] - s, m_hitPos[1] + 0.1f, m_hitPos[2]);
            glVertex3f(m_hitPos[0] + s, m_hitPos[1] + 0.1f, m_hitPos[2]);
            glVertex3f(m_hitPos[0], m_hitPos[1] - s + 0.1f, m_hitPos[2]);
            glVertex3f(m_hitPos[0], m_hitPos[1] + s + 0.1f, m_hitPos[2]);
            glVertex3f(m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2] - s);
            glVertex3f(m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2] + s);
            glEnd();
            glLineWidth(1.0f);

            int tx = 0, ty = 0;
            m_sample->getTilePos(m_hitPos, tx, ty);
            m_sample->renderCachedTile(tx, ty, m_drawType);
        }
    }

    void handleRenderOverlay(double* proj, double* model, int* view) override
    {
        if (m_hitPosSet)
        {
            if (m_sample)
            {
                int tx = 0, ty = 0;
                m_sample->getTilePos(m_hitPos, tx, ty);
                m_sample->renderCachedTileOverlay(tx, ty, proj, model, view);
            }
        }
    }
};

class TempObstacleCreateTool final : public SampleTool
{
    Sample_TempObstacles* m_sample;

public:
    TempObstacleCreateTool() : m_sample(nullptr)
    {
    }

    ~TempObstacleCreateTool() override = default;

    int type() override { return TOOL_TEMP_OBSTACLE; }

    void init(Sample* sample) override
    {
        m_sample = dynamic_cast<Sample_TempObstacles*>(sample);
    }

    void reset() override
    {
    }

    void handleMenu() override
    {
        imguiLabel("Create Temp Obstacles");

        if (imguiButton("Remove All"))
            m_sample->clearAllTempObstacles();

        imguiSeparator();

        imguiValue("Click LMB to create an obstacle.");
        imguiValue("Shift+LMB to remove an obstacle.");
    }

    void handleClick(const float* s, const float* p, const bool shift) override
    {
        if (m_sample)
        {
            if (shift)
                m_sample->removeTempObstacle(s, p);
            else
                m_sample->addTempObstacle(p);
        }
    }

    void handleToggle() override
    {
    }

    void handleStep() override
    {
    }

    void handleUpdate(const float /*dt*/) override
    {
    }

    void handleRender() override
    {
    }

    void handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) override
    {
    }
};

Sample_TempObstacles::Sample_TempObstacles() :
    m_keepInterResults(false),
    m_tileCache(nullptr),
    m_cacheBuildTimeMs(0),
    m_cacheCompressedSize(0),
    m_cacheRawSize(0),
    m_cacheLayerCount(0),
    m_cacheBuildMemUsage(0),
    m_drawMode(DRAWMODE_NAVMESH),
    m_maxTiles(0),
    m_maxPolysPerTile(0),
    m_tileSize(48)
{
    resetCommonSettings();

    m_talloc = new LinearAllocator(32000);
    m_tcomp = new FastLZCompressor;
    m_tmproc = new MeshProcess;

    setTool(new TempObstacleCreateTool);
}

Sample_TempObstacles::~Sample_TempObstacles()
{
    dtFreeNavMesh(m_navMesh);
    m_navMesh = nullptr;
    dtFreeTileCache(m_tileCache);
}

void Sample_TempObstacles::handleSettings()
{
    handleCommonSettings();

    if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
        m_keepInterResults = !m_keepInterResults;

    imguiLabel("Tiling");
    imguiSlider("TileSize", &m_tileSize, 16.0f, 128.0f, 8.0f);

    int gridSize = 1;
    if (m_geom)
    {
        const float* bmin = m_geom->getNavMeshBoundsMin();
        const float* bmax = m_geom->getNavMeshBoundsMax();
        char text[64];
        int gw = 0, gh = 0;
        rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
        const int ts = static_cast<int>(m_tileSize);
        const int tw = (gw + ts - 1) / ts;
        const int th = (gh + ts - 1) / ts;
        sprintf_s(text, "Tiles  %d x %d", tw, th);
        imguiValue(text);

        // Max tiles and max polys affect how the tile IDs are caculated.
        // There are 22 bits available for identifying a tile and a polygon.
        int tileBits = rcMin(static_cast<int>(dtIlog2(dtNextPow2(tw * th * EXPECTED_LAYERS_PER_TILE))), 14);
        if (tileBits > 14) tileBits = 14;
        const int polyBits = 22 - tileBits;
        m_maxTiles = 1 << tileBits;
        m_maxPolysPerTile = 1 << polyBits;
        sprintf_s(text, "Max Tiles  %d", m_maxTiles);
        imguiValue(text);
        sprintf_s(text, "Max Polys  %d", m_maxPolysPerTile);
        imguiValue(text);
        gridSize = tw * th;
    }
    else
    {
        m_maxTiles = 0;
        m_maxPolysPerTile = 0;
    }

    imguiSeparator();

    imguiLabel("Tile Cache");
    char msg[64];

    const float compressionRatio = static_cast<float>(m_cacheCompressedSize) / static_cast<float>(m_cacheRawSize + 1);

    sprintf_s(msg, "Layers  %d", m_cacheLayerCount);
    imguiValue(msg);
    sprintf_s(msg, "Layers (per tile)  %.1f", static_cast<float>(m_cacheLayerCount) / static_cast<float>(gridSize));
    imguiValue(msg);

    sprintf_s(msg, "Memory  %.1f kB / %.1f kB (%.1f%%)", static_cast<float>(m_cacheCompressedSize) / 1024.0f, static_cast<float>(m_cacheRawSize) / 1024.0f,
             compressionRatio * 100.0f);
    imguiValue(msg);
    sprintf_s(msg, "Navmesh Build Time  %.1f ms", m_cacheBuildTimeMs);
    imguiValue(msg);
    sprintf_s(msg, "Build Peak Mem Usage  %.1f kB", static_cast<float>(m_cacheBuildMemUsage) / 1024.0f);
    imguiValue(msg);

    imguiSeparator();

    imguiIndent();
    imguiIndent();

    if (imguiButton("Save"))
    {
        saveAll("all_tiles_tilecache.bin");
    }

    if (imguiButton("Load"))
    {
        dtFreeNavMesh(m_navMesh);
        dtFreeTileCache(m_tileCache);
        loadAll("all_tiles_tilecache.bin");
        m_navQuery->init(m_navMesh, 2048);
    }

    imguiUnindent();
    imguiUnindent();

    imguiSeparator();
}

void Sample_TempObstacles::handleTools()
{
    const int type = !m_tool ? TOOL_NONE : m_tool->type();

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
    for (bool & i : valid)
        i = false;

    if (m_geom)
    {
        valid[DRAWMODE_NAVMESH] = m_navMesh != nullptr;
        valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != nullptr;
        valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != nullptr;
        valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != nullptr;
        valid[DRAWMODE_NAVMESH_PORTALS] = m_navMesh != nullptr;
        valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != nullptr;
        valid[DRAWMODE_MESH] = true;
        valid[DRAWMODE_CACHE_BOUNDS] = true;
    }

    int unavail = 0;
    for (const bool i : valid)
        if (!i) unavail++;

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
    if (imguiCheck("Cache Bounds", m_drawMode == DRAWMODE_CACHE_BOUNDS, valid[DRAWMODE_CACHE_BOUNDS]))
        m_drawMode = DRAWMODE_CACHE_BOUNDS;

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

    const float texScale = 1.0f / (m_cellSize * 10.0f);

    // Draw mesh
    if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
    {
        // Draw mesh
        duDebugDrawTriMeshSlope(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
                                m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(),
                                m_geom->getMesh()->getTriCount(),
                                m_agentMaxSlope, texScale);
        m_geom->drawOffMeshConnections(&m_dd);
    }

    if (m_tileCache && m_drawMode == DRAWMODE_CACHE_BOUNDS)
        drawTiles(&m_dd, m_tileCache);

    if (m_tileCache)
        drawObstacles(&m_dd, m_tileCache);


    glDepthMask(GL_FALSE);

    // Draw bounds
    const float* bmin = m_geom->getNavMeshBoundsMin();
    const float* bmax = m_geom->getNavMeshBoundsMax();
    duDebugDrawBoxWire(&m_dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], duRGBA(255, 255, 255, 128), 1.0f);

    // Tiling grid.
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int tw = (gw + static_cast<int>(m_tileSize) - 1) / static_cast<int>(m_tileSize);
    const int th = (gh + static_cast<int>(m_tileSize) - 1) / static_cast<int>(m_tileSize);
    const float s = m_tileSize * m_cellSize;
    duDebugDrawGridXZ(&m_dd, bmin[0], bmin[1], bmin[2], tw, th, s, duRGBA(0, 0, 0, 64), 1.0f);

    if (m_navMesh && m_navQuery &&
        (m_drawMode == DRAWMODE_NAVMESH ||
            m_drawMode == DRAWMODE_NAVMESH_TRANS ||
            m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
            m_drawMode == DRAWMODE_NAVMESH_NODES ||
            m_drawMode == DRAWMODE_NAVMESH_PORTALS ||
            m_drawMode == DRAWMODE_NAVMESH_INVIS))
    {
        if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
            duDebugDrawNavMeshWithClosedList(&m_dd, *m_navMesh, *m_navQuery,
                                             m_navMeshDrawFlags/*|DU_DRAWNAVMESH_COLOR_TILES*/);
        if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
            duDebugDrawNavMeshBVTree(&m_dd, *m_navMesh);
        if (m_drawMode == DRAWMODE_NAVMESH_PORTALS)
            duDebugDrawNavMeshPortals(&m_dd, *m_navMesh);
        if (m_drawMode == DRAWMODE_NAVMESH_NODES)
            duDebugDrawNavMeshNodes(&m_dd, *m_navQuery);
        duDebugDrawNavMeshPolysWithFlags(&m_dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0, 0, 0, 128));
    }


    glDepthMask(GL_TRUE);

    m_geom->drawConvexVolumes(&m_dd);

    if (m_tool)
        m_tool->handleRender();
    renderToolStates();

    glDepthMask(GL_TRUE);
}

void Sample_TempObstacles::renderCachedTile(const int tx, const int ty, const int type)
{
    if (m_tileCache)
        drawDetail(&m_dd, m_tileCache, tx, ty, type);
}

void Sample_TempObstacles::renderCachedTileOverlay(const int tx, const int ty, const double* proj, const double* model,
                                                   const int* view) const
{
    if (m_tileCache)
        drawDetailOverlay(m_tileCache, tx, ty, proj, model, view);
}

void Sample_TempObstacles::handleRenderOverlay(double* proj, double* model, int* view)
{
    if (m_tool)
        m_tool->handleRenderOverlay(proj, model, view);
    renderOverlayToolStates(proj, model, view);

    // Stats
    /*	imguiDrawRect(280,10,300,100,imguiRGBA(0,0,0,64));

        char text[64];
        int y = 110-30;

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
        */
}

void Sample_TempObstacles::handleMeshChanged(InputGeom* geom)
{
    Sample::handleMeshChanged(geom);

    dtFreeTileCache(m_tileCache);
    m_tileCache = nullptr;

    dtFreeNavMesh(m_navMesh);
    m_navMesh = nullptr;

    if (m_tool)
    {
        m_tool->reset();
        m_tool->init(this);
        m_tmproc->init(m_geom);
    }
    resetToolStates();
    initToolStates(this);
}

void Sample_TempObstacles::addTempObstacle(const float* pos) const
{
    if (!m_tileCache)
        return;
    float p[3];
    dtVcopy(p, pos);
    p[1] -= 0.5f;
    m_tileCache->addObstacle(p, 1.0f, 2.0f, nullptr);
}

void Sample_TempObstacles::removeTempObstacle(const float* sp, const float* sq) const
{
    if (!m_tileCache)
        return;
    const dtObstacleRef ref = hitTestObstacle(m_tileCache, sp, sq);
    m_tileCache->removeObstacle(ref);
}

void Sample_TempObstacles::clearAllTempObstacles() const
{
    if (!m_tileCache)
        return;
    for (int i = 0; i < m_tileCache->getObstacleCount(); ++i)
    {
        const dtTileCacheObstacle* ob = m_tileCache->getObstacle(i);
        if (ob->state == DT_OBSTACLE_EMPTY) continue;
        m_tileCache->removeObstacle(m_tileCache->getObstacleRef(ob));
    }
}

bool Sample_TempObstacles::handleBuild()
{
    dtStatus status;

    if (!m_geom || !m_geom->getMesh())
    {
        m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles.");
        return false;
    }

    m_tmproc->init(m_geom);

    // Init cache
    const float* bmin = m_geom->getNavMeshBoundsMin();
    const float* bmax = m_geom->getNavMeshBoundsMax();
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int ts = static_cast<int>(m_tileSize);
    const int tw = (gw + ts - 1) / ts;
    const int th = (gh + ts - 1) / ts;

    // Generation params.
    rcConfig cfg = {};
    cfg.cs = m_cellSize;
    cfg.ch = m_cellHeight;
    cfg.walkableSlopeAngle = m_agentMaxSlope;
    cfg.walkableHeight = static_cast<int>(std::ceil(m_agentHeight / cfg.ch));
    cfg.walkableClimb = static_cast<int>(std::floor(m_agentMaxClimb / cfg.ch));
    cfg.walkableRadius = static_cast<int>(std::ceil(m_agentRadius / cfg.cs));
    cfg.maxEdgeLen = static_cast<int>(m_edgeMaxLen / m_cellSize);
    cfg.maxSimplificationError = m_edgeMaxError;
    cfg.minRegionArea = static_cast<int>(rcSqr(m_regionMinSize)); // Note: area = size*size
    cfg.mergeRegionArea = static_cast<int>(rcSqr(m_regionMergeSize)); // Note: area = size*size
    cfg.maxVertsPerPoly = static_cast<int>(m_vertsPerPoly);
    cfg.tileSize = static_cast<int>(m_tileSize);
    cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
    cfg.width = cfg.tileSize + cfg.borderSize * 2;
    cfg.height = cfg.tileSize + cfg.borderSize * 2;
    cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
    cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
    rcVcopy(cfg.bmin, bmin);
    rcVcopy(cfg.bmax, bmax);

    // Tile cache params.
    dtTileCacheParams tcparams = {};
    rcVcopy(tcparams.orig, bmin);
    tcparams.cs = m_cellSize;
    tcparams.ch = m_cellHeight;
    tcparams.width = static_cast<int>(m_tileSize);
    tcparams.height = static_cast<int>(m_tileSize);
    tcparams.walkableHeight = m_agentHeight;
    tcparams.walkableRadius = m_agentRadius;
    tcparams.walkableClimb = m_agentMaxClimb;
    tcparams.maxSimplificationError = m_edgeMaxError;
    tcparams.maxTiles = tw * th * EXPECTED_LAYERS_PER_TILE;
    tcparams.maxObstacles = 128;

    dtFreeTileCache(m_tileCache);

    m_tileCache = dtAllocTileCache();
    if (!m_tileCache)
    {
        m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate tile cache.");
        return false;
    }
    status = m_tileCache->init(&tcparams, m_talloc, m_tcomp, m_tmproc);
    if (dtStatusFailed(status))
    {
        m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init tile cache.");
        return false;
    }

    dtFreeNavMesh(m_navMesh);

    m_navMesh = dtAllocNavMesh();
    if (!m_navMesh)
    {
        m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh.");
        return false;
    }

    dtNavMeshParams params = {};
    rcVcopy(params.orig, bmin);
    params.tileWidth = m_tileSize * m_cellSize;
    params.tileHeight = m_tileSize * m_cellSize;
    params.maxTiles = m_maxTiles;
    params.maxPolys = m_maxPolysPerTile;

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

    m_cacheLayerCount = 0;
    m_cacheCompressedSize = 0;
    m_cacheRawSize = 0;

    for (int y = 0; y < th; ++y)
    {
        for (int x = 0; x < tw; ++x)
        {
            TileCacheData tiles[MAX_LAYERS] = {};
            int ntiles = rasterizeTileLayers(x, y, cfg, tiles, MAX_LAYERS);

            for (int i = 0; i < ntiles; ++i)
            {
                TileCacheData* tile = &tiles[i];
                status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, nullptr);
                if (dtStatusFailed(status))
                {
                    dtFree(tile->data);
                    tile->data = nullptr;
                    continue;
                }

                m_cacheLayerCount++;
                m_cacheCompressedSize += tile->dataSize;
                m_cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
            }
        }
    }

    // Build initial meshes
    m_ctx->startTimer(RC_TIMER_TOTAL);
    for (int y = 0; y < th; ++y)
        for (int x = 0; x < tw; ++x)
            m_tileCache->buildNavMeshTilesAt(x, y, m_navMesh);
    m_ctx->stopTimer(RC_TIMER_TOTAL);

    m_cacheBuildTimeMs = static_cast<float>(m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)) / 1000.0f;
    m_cacheBuildMemUsage = static_cast<unsigned int>(m_talloc->high);


    const dtNavMesh* nav = m_navMesh;
    int navmeshMemUsage = 0;
    for (int i = 0; i < nav->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = nav->getTile(i);
        if (tile->header)
            navmeshMemUsage += tile->dataSize;
    }
    printf("navmeshMemUsage = %.1f kB", static_cast<float>(navmeshMemUsage) / 1024.0f);


    if (m_tool)
        m_tool->init(this);
    initToolStates(this);

    return true;
}

void Sample_TempObstacles::handleUpdate(const float dt)
{
    Sample::handleUpdate(dt);

    if (!m_navMesh)
        return;
    if (!m_tileCache)
        return;

    m_tileCache->update(dt, m_navMesh);
}

void Sample_TempObstacles::getTilePos(const float* pos, int& tx, int& ty) const
{
    if (!m_geom) return;

    const float* bmin = m_geom->getNavMeshBoundsMin();

    const float ts = m_tileSize * m_cellSize;
    tx = static_cast<int>((pos[0] - bmin[0]) / ts);
    ty = static_cast<int>((pos[2] - bmin[2]) / ts);
}

static constexpr int TILECACHESET_MAGIC = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'TSET';
static constexpr int TILECACHESET_VERSION = 1;

struct TileCacheSetHeader
{
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams meshParams;
    dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader
{
    dtCompressedTileRef tileRef;
    int dataSize;
};

void Sample_TempObstacles::saveAll(const char* path) const
{
    if (!m_tileCache) return;

	FILE* fp;
	fopen_s(&fp, path, "wb");
    if (!fp)
        return;

    // Store header.
    TileCacheSetHeader header{};
    header.magic = TILECACHESET_MAGIC;
    header.version = TILECACHESET_VERSION;
    header.numTiles = 0;
    for (int i = 0; i < m_tileCache->getTileCount(); ++i)
    {
        const dtCompressedTile* tile = m_tileCache->getTile(i);
        if (!tile || !tile->header || !tile->dataSize) continue;
        header.numTiles++;
    }
    memcpy(&header.cacheParams, m_tileCache->getParams(), sizeof(dtTileCacheParams));
    memcpy(&header.meshParams, m_navMesh->getParams(), sizeof(dtNavMeshParams));
    fwrite(&header, sizeof(TileCacheSetHeader), 1, fp);

    // Store tiles.
    for (int i = 0; i < m_tileCache->getTileCount(); ++i)
    {
        const dtCompressedTile* tile = m_tileCache->getTile(i);
        if (!tile || !tile->header || !tile->dataSize) continue;

        TileCacheTileHeader tileHeader{};
        tileHeader.tileRef = m_tileCache->getTileRef(tile);
        tileHeader.dataSize = tile->dataSize;
        fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

        fwrite(tile->data, tile->dataSize, 1, fp);
    }

    fclose(fp);
}

void Sample_TempObstacles::loadAll(const char* path)
{
	FILE* fp;
	fopen_s(&fp, path, "wb");
    if (!fp) return;

    // Read header.
    TileCacheSetHeader header{};
    const size_t headerReadReturnCode = fread(&header, sizeof(TileCacheSetHeader), 1, fp);
    if (headerReadReturnCode != 1)
    {
        // Error or early EOF
        fclose(fp);
        return;
    }
    if (header.magic != TILECACHESET_MAGIC)
    {
        fclose(fp);
        return;
    }
    if (header.version != TILECACHESET_VERSION)
    {
        fclose(fp);
        return;
    }

    m_navMesh = dtAllocNavMesh();
    if (!m_navMesh)
    {
        fclose(fp);
        return;
    }
    dtStatus status = m_navMesh->init(&header.meshParams);
    if (dtStatusFailed(status))
    {
        fclose(fp);
        return;
    }

    m_tileCache = dtAllocTileCache();
    if (!m_tileCache)
    {
        fclose(fp);
        return;
    }
    status = m_tileCache->init(&header.cacheParams, m_talloc, m_tcomp, m_tmproc);
    if (dtStatusFailed(status))
    {
        fclose(fp);
        return;
    }

    // Read tiles.
    for (int i = 0; i < header.numTiles; ++i)
    {
        TileCacheTileHeader tileHeader{};
        const size_t tileHeaderReadReturnCode = fread(&tileHeader, sizeof(tileHeader), 1, fp);
        if (tileHeaderReadReturnCode != 1)
        {
            // Error or early EOF
            fclose(fp);
            return;
        }
        if (!tileHeader.tileRef || !tileHeader.dataSize)
            break;

        const auto data = static_cast<unsigned char*>(dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM));
        if (!data) break;
        memset(data, 0, tileHeader.dataSize);
        const size_t tileDataReadReturnCode = fread(data, tileHeader.dataSize, 1, fp);
        if (tileDataReadReturnCode != 1)
        {
            // Error or early EOF
            dtFree(data);
            fclose(fp);
            return;
        }

        dtCompressedTileRef tile = 0;
        const dtStatus addTileStatus = m_tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA,
                                                            &tile);
        if (dtStatusFailed(addTileStatus))
        {
            dtFree(data);
        }

        if (tile)
            m_tileCache->buildNavMeshTile(tile, m_navMesh);
    }

    fclose(fp);
}

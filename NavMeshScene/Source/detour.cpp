#include "detour.h"
#include "detour_util.h"
#include "scene.h"
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <DetourCommon.h>
#include <DetourTileCache.h>
#include <cstdio>
#include <string>
#include <cassert>

namespace NavMeshScene {

    std::unordered_map<std::string, dtNavMesh*> Detour::mStaticMesh;
    std::mutex Detour::mMutex;

#pragma pack(push, 1)

    struct NavMeshSetHeader
    {
        int32_t magic;
        int32_t version;
        int32_t numTiles;
        dtNavMeshParams params;
    };
    struct NavMeshSetHeaderExt
    {
        float boundsMinX;
        float boundsMinY;
        float boundsMinZ;
        float boundsMaxX;
        float boundsMaxY;
        float boundsMaxZ;
    };

    struct NavMeshTileHeader
    {
        dtTileRef tileRef;
        int32_t dataSize;
    };

    struct TileCacheSetHeader
    {
        int32_t magic;
        int32_t version;
        int32_t numTiles;
        dtNavMeshParams meshParams;
        dtTileCacheParams cacheParams;
    };
    struct TileCacheSetHeaderExt
    {
        float boundsMinX;
        float boundsMinY;
        float boundsMinZ;
        float boundsMaxX;
        float boundsMaxY;
        float boundsMaxZ;
    };

    struct TileCacheTileHeader
    {
        dtCompressedTileRef tileRef;
        int32_t dataSize;
    };

#pragma pack(pop)

    static const int32_t NAVMESHSET_MAGIC_RAW = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';
    static const int32_t NAVMESHSET_MAGIC_EXT = 'M' << 24 | 'S' << 16 | 'A' << 8 | 'T';
    static const int32_t NAVMESHSET_VERSION = 1;
    static const int32_t TILECACHESET_MAGIC_RAW = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T';
    static const int32_t TILECACHESET_MAGIC_EXT = 'T' << 24 | 'S' << 16 | 'A' << 8 | 'T';
    static const int32_t TILECACHESET_VERSION = 1;


    class FileReader {
    public:
        FileReader(const char* path)
            :fp(nullptr)
        {
#pragma warning(push)
#pragma warning(disable: 4996)
            fp = fopen(path, "rb");
#pragma warning(pop)
        }
        ~FileReader() {
            if (fp) {
                fclose(fp);
                fp = nullptr;
            }
        }
        operator FILE*() {
            return fp;
        }

    private:
        FILE* fp;
    };

    Detour::Detour(bool bStaticMesh, uint16_t maxNode)
        : mbStaticMesh(bStaticMesh)
        , mMaxNode(maxNode)
        , mMesh(nullptr)
        , mTileCache(nullptr)
        , mQuery(nullptr)
        , mTalloc(nullptr)
        , mTcomp(nullptr)
        , mTmproc(nullptr)
        , mHeightMode(0)
        , mQueryForHeightMode2(nullptr)
    {
        assert(maxNode != 0);
    }

    Detour::~Detour() {
        if (mQuery) {
            dtFreeNavMeshQuery(mQuery);
            mQuery = nullptr;
        }
        if (mQueryForHeightMode2) {
            dtFreeNavMeshQuery(mQueryForHeightMode2);
            mQueryForHeightMode2 = nullptr;
        }
        if (mMesh) {
            dtFreeNavMesh(mMesh);
            mMesh = nullptr;
        }
        if (mTileCache) {
            dtFreeTileCache(mTileCache);
            mTileCache = nullptr;
        }
        if (mTalloc) {
            delete mTalloc;
            mTalloc = nullptr;
        }
        if (mTcomp) {
            delete mTcomp;
            mTcomp = nullptr;
        }
        if (mTmproc) {
            delete mTmproc;
            mTmproc = nullptr;
        }
    }

    static const char* FILE_SUFFIX_0 = ".tile.bin";
    static const char* FILE_SUFFIX_1 = ".tilecache.bin";
    static const size_t FILE_SUFFIX_0_LEN = strlen(FILE_SUFFIX_0);
    static const size_t FILE_SUFFIX_1_LEN = strlen(FILE_SUFFIX_1);

    int Detour::Load(const char*path) {
        int errCode = 0;
        dtNavMesh* mesh = nullptr;
        if (mbStaticMesh) {
            mesh = createStaticMesh(path, errCode);
        }
        else {
			assert(int(std::string(path).find(FILE_SUFFIX_0)) > 0 || int(std::string(path).find(FILE_SUFFIX_1)) > 0);
            dtNavMesh* tempMesh = nullptr;
            if (mHeightMode == DynamicScene::HEIGHT_MODE_2) {
                std::string tempPath = path;
                int index = int(tempPath.find(FILE_SUFFIX_1));
                if (index > 0) {
                    tempPath = tempPath.replace(index, index + FILE_SUFFIX_1_LEN, FILE_SUFFIX_0, 0, FILE_SUFFIX_0_LEN);
                }
                tempMesh = createStaticMesh(tempPath.c_str(), errCode);
                if (errCode) {
                    return errCode;
                }
                mQueryForHeightMode2 = dtAllocNavMeshQuery();
                if (!mQueryForHeightMode2) {
                    return 1;
                }

                dtStatus status = mQueryForHeightMode2->init(tempMesh, mMaxNode);
                if (!dtStatusSucceed(status)) {
                    return 2;
                }
            }
            std::string tempPath = path;
            int index = int(tempPath.find(FILE_SUFFIX_0));
            if (index > 0) {
                tempPath = tempPath.replace(index, index + FILE_SUFFIX_0_LEN, FILE_SUFFIX_1, 0, FILE_SUFFIX_1_LEN);
            }
            mesh = loadDynamicMesh(tempPath.c_str(), errCode);
        }

        if (errCode) {
            return errCode;
        }

        mQuery = dtAllocNavMeshQuery();
        if (!mQuery) {
            return 3;
        }

        dtStatus status = mQuery->init(mesh, mMaxNode);
        if (!dtStatusSucceed(status)) {
            return 4;
        }
        return 0;
    }

    dtNavMesh* Detour::createStaticMesh(const char*path, int& errCode) {
        dtNavMesh* mesh = nullptr;
        std::lock_guard<std::mutex> lock(mMutex);
        auto it = Detour::mStaticMesh.find(path);
        if (it != Detour::mStaticMesh.end()) {
            mesh = it->second;
        }
        else {
            mesh = loadStaticMesh(path, errCode);
            if (!errCode) {
                Detour::mStaticMesh[path] = mesh;
            }
        }
        return mesh;
    }

    dtNavMesh* Detour::loadStaticMesh(const char*path, int& errCode) {
        errCode = 0;
        FileReader fp(path);
        if (fp == 0) {
            errCode = 101;
            return nullptr;
        }

        // Read header.
        NavMeshSetHeader header;
        size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
        if (readLen != 1)
        {
            errCode = 102;
            return nullptr;
        }
        if (header.magic != NAVMESHSET_MAGIC_RAW && header.magic != NAVMESHSET_MAGIC_EXT)
        {
            errCode = 103;
            return nullptr;
        }
        if (header.version != NAVMESHSET_VERSION)
        {
            errCode = 104;
            return nullptr;
        }

        if (header.magic == NAVMESHSET_MAGIC_EXT) {
            NavMeshSetHeaderExt headerExt;
            size_t readLen = fread(&headerExt, sizeof(NavMeshSetHeaderExt), 1, fp);
            if (readLen != 1)
            {
                errCode = 102;
                return nullptr;
            }
            mBoundsMin[0] = headerExt.boundsMinX;
            mBoundsMin[1] = headerExt.boundsMinY;
            mBoundsMin[2] = headerExt.boundsMinZ;
            mBoundsMax[0] = headerExt.boundsMaxX;
            mBoundsMax[1] = headerExt.boundsMaxY;
            mBoundsMax[2] = headerExt.boundsMaxZ;
        }
        else {
            mBoundsMin[0] = 0;
            mBoundsMin[1] = 0;
            mBoundsMin[2] = 0;
            mBoundsMax[0] = 0;
            mBoundsMax[1] = 0;
            mBoundsMax[2] = 0;
        }

        dtNavMesh* mesh = dtAllocNavMesh();
        if (!mesh)
        {
            errCode = 105;
            return nullptr;
        }
        dtStatus status = mesh->init(&header.params);
        if (!dtStatusSucceed(status))
        {
            dtFreeNavMesh(mesh);
            errCode = 106;
            return nullptr;
        }

        // Read tiles.
        for (int i = 0; i < header.numTiles; ++i)
        {
            NavMeshTileHeader tileHeader;
            readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
            if (readLen != 1)
            {
                dtFreeNavMesh(mesh);
                errCode = 107;
                return nullptr;
            }

            if (!tileHeader.tileRef || !tileHeader.dataSize)
                break;

            unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
            if (!data) break;
            memset(data, 0, tileHeader.dataSize);
            readLen = fread(data, tileHeader.dataSize, 1, fp);
            if (readLen != 1)
            {
                dtFree(data);
                dtFreeNavMesh(mesh);
                errCode = 108;
                return nullptr;
            }

            mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
        }
        return mesh;
    }

    dtNavMesh* Detour::loadDynamicMesh(const char*path, int& errCode) {
        errCode = 0;
        FileReader fp(path);
        if (fp == 0) {
            errCode = 201;
            return nullptr;
        }

        // Read header.
        TileCacheSetHeader header;
        size_t headerReadReturnCode = fread(&header, sizeof(TileCacheSetHeader), 1, fp);
        if (headerReadReturnCode != 1)
        {
            // Error or early EOF
            errCode = 202;
            return nullptr;
        }
        if (header.magic != TILECACHESET_MAGIC_RAW && header.magic != TILECACHESET_MAGIC_EXT)
        {
            errCode = 203;
            return nullptr;
        }
        if (header.version != TILECACHESET_VERSION)
        {
            errCode = 204;
            return nullptr;
        }

        if (header.magic == TILECACHESET_MAGIC_EXT) {
            TileCacheSetHeaderExt headerExt;
            size_t readLen = fread(&headerExt, sizeof(TileCacheSetHeaderExt), 1, fp);
            if (readLen != 1)
            {
                errCode = 102;
                return nullptr;
            }
            mBoundsMin[0] = headerExt.boundsMinX;
            mBoundsMin[1] = headerExt.boundsMinY;
            mBoundsMin[2] = headerExt.boundsMinZ;
            mBoundsMax[0] = headerExt.boundsMaxX;
            mBoundsMax[1] = headerExt.boundsMaxY;
            mBoundsMax[2] = headerExt.boundsMaxZ;
        }
        else {
            mBoundsMin[0] = 0;
            mBoundsMin[1] = 0;
            mBoundsMin[2] = 0;
            mBoundsMax[0] = 0;
            mBoundsMax[1] = 0;
            mBoundsMax[2] = 0;
        }

        mMesh = dtAllocNavMesh();
        if (!mMesh)
        {
            errCode = 205;
            return nullptr;
        }
        dtStatus status = mMesh->init(&header.meshParams);
        if (!dtStatusSucceed(status))
        {
            errCode = 206;
            return nullptr;
        }

        mTileCache = dtAllocTileCache();
        if (!mTileCache)
        {
            errCode = 207;
            return nullptr;
        }

        mTalloc = new LinearAllocator(32 * 1024);
        mTcomp = new FastLZCompressor();
        mTmproc = new MeshProcess();
        status = mTileCache->init(&header.cacheParams, mTalloc, mTcomp, mTmproc);
        if (!dtStatusSucceed(status))
        {
            errCode = 208;
            return nullptr;
        }

        // Read tiles.
        for (int i = 0; i < header.numTiles; ++i)
        {
            TileCacheTileHeader tileHeader;
            size_t tileHeaderReadReturnCode = fread(&tileHeader, sizeof(tileHeader), 1, fp);
            if (tileHeaderReadReturnCode != 1)
            {
                // Error or early EOF
                errCode = 209;
                return nullptr;
            }
            if (!tileHeader.tileRef || !tileHeader.dataSize)
                break;

            unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
            if (!data) break;
            memset(data, 0, tileHeader.dataSize);
            size_t tileDataReadReturnCode = fread(data, tileHeader.dataSize, 1, fp);
            if (tileDataReadReturnCode != 1)
            {
                // Error or early EOF
                dtFree(data);
                errCode = 210;
                return nullptr;
            }

            dtCompressedTileRef tile = 0;
            dtStatus addTileStatus = mTileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);
            if (dtStatusFailed(addTileStatus))
            {
                dtFree(data);
                errCode = 211;
                return nullptr;
            }

            if (tile) {
                mTileCache->buildNavMeshTile(tile, mMesh);
            }
            else {
                errCode = 212;
                return nullptr;
            }
        }
        return mMesh;
    }

    bool Detour::TryMove(
        uint64_t startPolyRef,
        float startPos[3],
        float endPos[3],
        float halfExtents[3],
        const dtQueryFilter& filter,
        uint64_t& realEndPolyRef,
        float realEndPos[3],
        bool& bHit)
    {
        bHit = false;
        if (!mQuery) {
            return false;
        }
        dtPolyRef visited[16];
        int nvisited = 0;
        dtStatus status = mQuery->moveAlongSurface(
            (dtPolyRef)startPolyRef,
            startPos,
            endPos,
            &filter,
            realEndPos,
            visited,
            &nvisited,
            sizeof(visited) / sizeof(visited[0]),
            bHit
        );

        if (dtStatusDetail(status, DT_INVALID_PARAM)) {
            dtPolyRef tempRef;
            float tempPos[3];
            mQuery->findNearestPoly(startPos, halfExtents, &filter, &tempRef, tempPos);
            startPolyRef = tempRef;
            dtVcopy(startPos, tempPos);

            status = mQuery->moveAlongSurface(
                (dtPolyRef)startPolyRef,
                startPos,
                endPos,
                &filter,
                realEndPos,
                visited,
                &nvisited,
                sizeof(visited) / sizeof(visited[0]),
                bHit
            );
        }

        if (!dtStatusSucceed(status)) {
            return false;
        }

        realEndPolyRef = startPolyRef;
        if (nvisited > 0) {
            realEndPolyRef = visited[nvisited - 1];
        }

        if (mHeightMode != DynamicScene::HEIGHT_MODE_2) {
            float h = 0;
            mQuery->getPolyHeight((dtPolyRef)realEndPolyRef, realEndPos, &h);
            realEndPos[1] = h;
        }
        else {
            dtPolyRef tempRef;
            float tempPos[3];
            mQueryForHeightMode2->findNearestPoly(realEndPos, halfExtents, &filter, &tempRef, tempPos);
            realEndPos[1] = tempPos[1];
        }
        return true;
    }

    bool Detour::GetPoly(
        float pos[3],
        float halfExtents[3],
        const dtQueryFilter& filter,
        uint64_t& nearestRef,
        float nearestPt[3])
    {
        if (!mQuery) {
            return false;
        }
        dtPolyRef nRef;
        dtStatus status = mQuery->findNearestPoly(pos, halfExtents, &filter, &nRef, nearestPt);
        if (!dtStatusSucceed(status)) {
            return false;
        }
        nearestRef = nRef;
        return true;
    }

    bool Detour::Raycast(
        uint64_t startPolyRef,
        float startPos[3],
        float endPos[3],
        const dtQueryFilter &filter,
        bool& bHit,
        float hitPos[3])
    {
        if (!mQuery) {
            return false;
        }
        float t = 0;
        float hitNormal[3];
        dtPolyRef polys[16];
        int npolys = 0;
        dtStatus status = mQuery->raycast((dtPolyRef)startPolyRef, startPos, endPos, &filter,
            &t, hitNormal, polys, &npolys, sizeof(polys) / sizeof(polys[0]));
        if (!dtStatusSucceed(status)) {
            return false;
        }
        bHit = (t <= 1);
        if (bHit)
        {
            dtVlerp(hitPos, startPos, endPos, t);
            if (npolys > 0)
            {
                float h = 0;
                mQuery->getPolyHeight(polys[npolys - 1], hitPos, &h);
                hitPos[1] = h;
            }
        }
        return true;
    }

    bool Detour::RandomPosition(float halfExtents[3],
        const dtQueryFilter* filter,
        float(*frand)(),
        uint64_t& randomRef,
        float randomPt[3])
    {
        if (!mQuery) {
            return false;
        }
        dtPolyRef ref;
        dtStatus status = mQuery->findRandomPoint(filter, frand, &ref, randomPt);
        if (!dtStatusSucceed(status)) {
            return false;
        }
        randomRef = ref;
        if (mHeightMode == DynamicScene::HEIGHT_MODE_2) {
            dtPolyRef tempRef;
            float tempPos[3];
            mQueryForHeightMode2->findNearestPoly(randomPt, halfExtents, filter, &tempRef, tempPos);
            randomPt[1] = tempPos[1];
        }
        return true;
    }

    unsigned int Detour::AddCapsuleObstacle(const float pos[3], const float radius, const float height) {
        if (!mTileCache) {
            return 0;
        }
        dtObstacleRef obstacleId;
        dtStatus status = mTileCache->addObstacle(pos, radius, height, &obstacleId);
        if (!dtStatusSucceed(status)) {
            return 0;
        }
        return (unsigned int)obstacleId;
    }

    unsigned int Detour::AddBoxObstacle(const float bmin[3], const float bmax[3]) {
        if (!mTileCache) {
            return 0;
        }
        dtObstacleRef obstacleId;
        dtStatus status = mTileCache->addBoxObstacle(bmin, bmax, &obstacleId);
        if (!dtStatusSucceed(status)) {
            return 0;
        }
        return (unsigned int)obstacleId;
    }

    unsigned int Detour::AddBoxObstacle(const float center[3], const float halfExtents[3], const float yRadians) {
        if (!mTileCache) {
            return 0;
        }
        dtObstacleRef obstacleId;
        dtStatus status = mTileCache->addBoxObstacle(center, halfExtents, yRadians, &obstacleId);
        if (!dtStatusSucceed(status)) {
            return 0;
        }
        return (unsigned int)obstacleId;
    }

    void Detour::RemoveObstacle(unsigned int obstacleId) {
        if (!mTileCache || obstacleId == 0) {
            return;
        }
        mTileCache->removeObstacle((dtObstacleRef)obstacleId);
    }

}
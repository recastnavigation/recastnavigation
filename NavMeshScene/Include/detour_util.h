#ifndef __NMS_DETOUR_UTIL_H__
#define __NMS_DETOUR_UTIL_H__

#include <DetourTileCacheBuilder.h>
#include <DetourTileCache.h>

namespace NavMeshScene {

    class LinearAllocator : public dtTileCacheAlloc
    {
    public:
        LinearAllocator(const size_t cap);
        ~LinearAllocator();
        void resize(const size_t cap);
        void reset() override;
        void* alloc(const size_t size) override;
        void free(void* ptr) override;
    private:
        unsigned char* buffer;
        size_t capacity;
        size_t top;
        size_t high;
    };

    class FastLZCompressor : public dtTileCacheCompressor
    {
    public:
        int maxCompressedSize(const int bufferSize) override;
        dtStatus compress(const unsigned char* buffer, const int bufferSize,
            unsigned char* compressed, const int maxCompressedSize, int* compressedSize) override;
        dtStatus decompress(const unsigned char* compressed, const int compressedSize,
            unsigned char* buffer, const int maxBufferSize, int* bufferSize) override;
    };

    class MeshProcess : public dtTileCacheMeshProcess
    {
    public:
        MeshProcess();
        void process(struct dtNavMeshCreateParams* params,
            unsigned char* polyAreas, unsigned short* polyFlags) override;
    };

}

#endif
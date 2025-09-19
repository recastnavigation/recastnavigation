#ifndef __NMS_AGENT_H__
#define __NMS_AGENT_H__

#include <memory>
#include <aoi/aoi.h>

namespace NavMeshScene {

    const float DEFAULT_HALF_EXTENTS[3] = { 0.6f, 2.0f, 0.6f };

    class Filter;
    class Scene;

    class Agent : public aoi::Object
    {
    public:

        Agent();
        virtual ~Agent();

        inline uint64_t GetId() { return mId; }
        inline void SetId(uint64_t id) { mId = id; }
        inline float* GetPosition() { return mPosition; }
        inline float* GetHalfExtents() { return mHalfExtents; }
        inline float* GetVelocity() { return mVelocity; }
        inline Filter& GetFilter() { return *mFilter; }

        inline void SetHalfExtents(float v[3]) { mHalfExtents[0] = v[0]; mHalfExtents[1] = v[1]; mHalfExtents[2] = v[2]; }
        inline void SetVelocity(float v[3]) { mVelocity[0] = v[0]; mVelocity[1] = v[1]; mVelocity[2] = v[2]; }
        inline void SetFilter(const std::shared_ptr<Filter>& filter) { mFilter = filter; }
        inline void SetScene(Scene* scene) { mScene = scene; }

        virtual void Update(float delta);
        bool SetPosition(float v[3]);
        void RandomPosition();
        bool Raycast(float endPos[3], bool& bHit, float hitPos[3]);

        virtual void OnHit(Agent* agent = nullptr) {}


    protected:
        bool TryMove(float endPos[3], uint64_t& realEndPolyRef, float realEndPos[3], bool& bHit);

        inline aoi::Rect getRect() { return aoi::Rect(mPosition[0] - mHalfExtents[0], mPosition[0] + mHalfExtents[0], mPosition[2] - mHalfExtents[2], mPosition[2] + mHalfExtents[2]); }
        Agent* checkPosByAOI(float srcX, float srcY, float& dstX, float& dstY, bool bMove);

        uint64_t mId;
        float mHalfExtents[3];
        float mPosition[3];
        float mVelocity[3];
        uint64_t mCurPolyRef;
        std::shared_ptr<Filter> mFilter;
        Scene* mScene;
    };

}

#endif
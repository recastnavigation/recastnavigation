#ifndef __NMS_SCENE_H__
#define __NMS_SCENE_H__

#include <memory>
#include <unordered_map>
#include <aoi/aoi.h>

namespace NavMeshScene {
    class Agent;
}

typedef aoi::Scene<NavMeshScene::Agent, 16> AOISceneType;

namespace NavMeshScene {

    class Detour;
    class Filter;

    class Scene : public AOISceneType
    {
    public:

        Scene(bool bStatic);
        virtual ~Scene();

        int Load(const char* path);
        void Simulation(float delta);
        void AddAgent(uint64_t id, const std::shared_ptr<Agent>& agent);
        void RemoveAgent(uint64_t id);

        float* GetBoundsMin();
        float* GetBoundsMax();

    public:
        inline Detour& GetDetour() { return *mDetour; }
        inline Filter& GetDefaultFilter() { return *mDefaultFilter; }

    protected:
        std::unique_ptr<Detour> mDetour;
        std::unordered_map<uint64_t, std::shared_ptr<Agent>> mAgents;
        std::unique_ptr<Filter> mDefaultFilter;
    };

    class StaticScene : public Scene {
    public:
        StaticScene() : Scene(true) {}
        virtual ~StaticScene() {}
    };

    class DynamicScene :public Scene {
    public:
        DynamicScene(int heightMode = HEIGHT_MODE_1);
        virtual ~DynamicScene();

        enum {
            HEIGHT_MODE_1 = 1,   // 原始的，精度不是很高，但是没多余消耗。可以通过使Tile Size变小来提高精度
            HEIGHT_MODE_2 = 2,   // 通过公共的StaticScene上，获取精确高度值。
        };

        unsigned int AddCapsuleObstacle(const float pos[3], const float radius, const float height);
        unsigned int AddBoxObstacle(const float bmin[3], const float bmax[3]);
        unsigned int AddBoxObstacle(const float center[3], const float halfExtents[3], const float yRadians);
        void RemoveObstacle(unsigned int obstacleId);
    };
}

#endif
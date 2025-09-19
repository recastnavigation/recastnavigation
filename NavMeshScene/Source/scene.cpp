#include "scene.h"
#include "agent.h"
#include "detour.h"
#include "filter.h"
#include <DetourTileCache.h>

namespace NavMeshScene {

    Scene::Scene(bool bStatic)
        : AOISceneType()
        , mDetour(new Detour(bStatic, 16))
        , mDefaultFilter(new Filter())
    {

    }

    Scene::~Scene() {

    }

    int Scene::Load(const char* path) {
        int retCode = mDetour->Load(path);
        if (!retCode)
        {
            float* bmin = mDetour->GetBoundsMin();
            float* bmax = mDetour->GetBoundsMax();
            printf("bounds min:(%f, %f, %f)\n", bmin[0], bmin[1], bmin[2]);
            printf("bounds max:(%f, %f, %f)\n", bmax[0], bmax[1], bmax[2]);
            SetBounds(aoi::Rect(bmin[0], bmax[0], bmin[2], bmax[2]));
        }
        return retCode;
    }

    void Scene::Simulation(float delta) {
        if (mDetour->GetTileCache()) {
            mDetour->GetTileCache()->update(delta, mDetour->GetMesh());
        }
        for (auto it = mAgents.begin(); it != mAgents.end(); it++) {
            auto &agent = it->second;
            agent->Update(delta);
        }
    }

    void Scene::AddAgent(uint64_t id, const std::shared_ptr<Agent>& agent) {
        if (id && agent) {
            agent->SetId(id);
            mAgents[id] = agent;
            agent->SetScene(this);
        }
    }

    void Scene::RemoveAgent(uint64_t id) {
        auto it = mAgents.find(id);
        if (it != mAgents.end()) {
            Remove(it->second.get());
            mAgents.erase(it);
        }
    }

    float* Scene::GetBoundsMin()
    {
        return mDetour->GetBoundsMin();
    }

    float* Scene::GetBoundsMax()
    {
        return mDetour->GetBoundsMax();
    }


    DynamicScene::DynamicScene(int heightMode)
        : Scene(false)
    {
        mDetour->SetHeightMode(heightMode);
    }

    DynamicScene::~DynamicScene() {

    }

    unsigned int DynamicScene::AddCapsuleObstacle(const float pos[3], const float radius, const float height) {
        return mDetour->AddCapsuleObstacle(pos, radius, height);
    }

    unsigned int DynamicScene::AddBoxObstacle(const float bmin[3], const float bmax[3]) {
        return mDetour->AddBoxObstacle(bmin, bmax);
    }

    unsigned int DynamicScene::AddBoxObstacle(const float center[3], const float halfExtents[3], const float yRadians) {
        return mDetour->AddBoxObstacle(center, halfExtents, yRadians);
    }

    void DynamicScene::RemoveObstacle(unsigned int obstacleId) {
        mDetour->RemoveObstacle(obstacleId);
    }
}
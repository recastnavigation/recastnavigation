#include "NavMeshSceneTool.h"
#include "imgui.h"
#include <iostream>
#include <DetourTileCache.h>
#include <DetourCommon.h>
#include <detour.h>

extern std::string gMeshName;
extern std::string gMeshesFolder;
extern float gMoveFront;
extern float gMoveBack;
extern float gMoveLeft;
extern float gMoveRight;

NavMeshSceneTool::NavMeshSceneTool()
    : m_sample(0)
    , mMeshMode(0)
    , mCurrentAgent(-1)
{

}

void NavMeshSceneTool::init(Sample* sample)
{
    m_sample = sample;
    doInit();
}

void NavMeshSceneTool::doInit() {
    std::string fname = "";
    if (mMeshMode == 0) {
        fname = gMeshesFolder + "/" + gMeshName + ".tile.bin";
        mScene = std::make_shared<NavMeshScene::StaticScene>();
    }
    else {
        fname = gMeshesFolder + "/" + gMeshName + ".tilecache.bin";
        mScene = std::make_shared<NavMeshScene::DynamicScene>(mMeshMode);
    }
    if (int ec = mScene->Load(fname.c_str())) {
        std::cout << "load scene fail! errcode: " << ec << std::endl;
        return;
    }

    for (size_t i = 0; i < mAgents.size(); i++)
    {
        mScene->RemoveAgent(mAgents[i]->GetId());
    }
    mAgents.clear();
    for (size_t i = 0; i < 100; i++)
    {
        auto agent = std::make_shared<Player>();
        mAgents.push_back(agent);
        mScene->AddAgent(i + 1, agent);
        agent->RandomPosition();

        // 3/4 agent移动
        if (rand() % 4 <= 2) {
            agent->changeDir();
        }
    }
    if (mCurrentAgent >= 0)
    {
        mAgents[mCurrentAgent]->mRobot = true;
    }
    mCurrentAgent = int(mAgents.size() - 1);
    mAgents[mCurrentAgent]->mRobot = false;
}

void NavMeshSceneTool::handleMenu()
{
    if (imguiCheck("Static Scene", mMeshMode == 0))
    {
        mMeshMode = 0;
        doInit();
    }

    if (imguiCheck("Dynamic Scene(height mode: 1)", mMeshMode == 1))
    {
        mMeshMode = 1;
        doInit();
    }

    if (imguiCheck("Dynamic Scene(height mode: 2)", mMeshMode == 2))
    {
        mMeshMode = 2;
        doInit();
    }

    imguiSeparator();

    if (imguiButton("Select Agent"))
    {
        mCurrentAgent = rand() % mAgents.size();
    }
}

void NavMeshSceneTool::handleClick(const float* s, const float* p, bool shift)
{
    if (mScene && mMeshMode != 0) {
        auto scn = std::static_pointer_cast<NavMeshScene::DynamicScene>(mScene);
        if (shift) {
            dtObstacleRef hitTestObstacle(const dtTileCache* tc, const float* sp, const float* sq);
            dtObstacleRef ref = hitTestObstacle(scn->GetDetour().GetTileCache(), s, p);
            if (ref != 0) {
                scn->RemoveObstacle(ref);
                printf("remove obstacle, id = %d\n", ref);
            }
        }
        else {
            float pos[3];
            dtVcopy(pos, p);
            pos[1] -= 0.5f;
            dtObstacleRef ref = scn->AddCapsuleObstacle(pos, 1.0f, 2.0f);
            printf("add obstacle, id = %d\n", ref);
        }
    }
}

void NavMeshSceneTool::handleStep()
{

}

void NavMeshSceneTool::handleToggle()
{

}

void NavMeshSceneTool::handleUpdate(const float dt)
{
    if (!mScene || mCurrentAgent < 0) {
        return;
    }

    float velocity[3] = { (gMoveLeft*-1 + gMoveRight) * 3, 0, (gMoveFront*-1 + gMoveBack) * 3 };
    mAgents[mCurrentAgent]->SetVelocity(velocity);
    mScene->Simulation(dt);
}

void NavMeshSceneTool::reset()
{

}

void NavMeshSceneTool::handleRender()
{
    if (mScene && mMeshMode != 0) {
        auto scn = std::static_pointer_cast<NavMeshScene::DynamicScene>(mScene);
        void drawObstacles(duDebugDraw* dd, const dtTileCache* tc);
        drawObstacles(&m_sample->getDebugDraw(), scn->GetDetour().GetTileCache());
    }

    for (size_t i = 0; i < mAgents.size(); i++)
    {
        auto color = duRGBA(51, 102, 0, 129);
        if (i == mCurrentAgent)
        {
            color = duRGBA(128, 25, 0, 192);
        }
        auto pos = mAgents[i]->GetPosition();
        drawAgent(pos, 0.6f, 2.0f, 0.9f, color);
    }
}

void NavMeshSceneTool::handleRenderOverlay(double* proj, double* model, int* view)
{

}

void NavMeshSceneTool::drawAgent(const float* pos, float r, float h, float c, const unsigned int col)
{
    duDebugDraw& dd = m_sample->getDebugDraw();

    dd.depthMask(false);

    // Agent dimensions.	
    duDebugDrawCylinderWire(&dd, pos[0] - r, pos[1] + 0.02f, pos[2] - r, pos[0] + r, pos[1] + h, pos[2] + r, col, 2.0f);

    duDebugDrawCircle(&dd, pos[0], pos[1] + c, pos[2], r, duRGBA(0, 0, 0, 64), 1.0f);

    unsigned int colb = duRGBA(0, 0, 0, 196);
    dd.begin(DU_DRAW_LINES);
    dd.vertex(pos[0], pos[1] - c, pos[2], colb);
    dd.vertex(pos[0], pos[1] + c, pos[2], colb);
    dd.vertex(pos[0] - r / 2, pos[1] + 0.02f, pos[2], colb);
    dd.vertex(pos[0] + r / 2, pos[1] + 0.02f, pos[2], colb);
    dd.vertex(pos[0], pos[1] + 0.02f, pos[2] - r / 2, colb);
    dd.vertex(pos[0], pos[1] + 0.02f, pos[2] + r / 2, colb);
    dd.end();

    dd.depthMask(true);
}
